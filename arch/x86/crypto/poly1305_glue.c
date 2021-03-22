// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Poly1305 authenticator algorithm, RFC7539, SIMD glue code
 *
 * Copyright (C) 2015 Martin Willi
 */

#include <crypto/algapi.h>
#include <crypto/internal/hash.h>
#include <crypto/internal/simd.h>
#include <crypto/poly1305.h>
#include <linux/crypto.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <asm/simd.h>

asmlinkage void poly1305_init_x86_64(void *ctx,
				     const u8 key[POLY1305_BLOCK_SIZE]);
asmlinkage void poly1305_blocks_x86_64(void *ctx, const u8 *inp,
				       const size_t len, const u32 padbit);
asmlinkage void poly1305_emit_x86_64(void *ctx, u8 mac[POLY1305_DIGEST_SIZE],
				     const u32 nonce[4]);
asmlinkage void poly1305_emit_avx(void *ctx, u8 mac[POLY1305_DIGEST_SIZE],
				  const u32 nonce[4]);
asmlinkage void poly1305_blocks_avx(void *ctx, const u8 *inp, const size_t len,
				    const u32 padbit);
asmlinkage void poly1305_blocks_avx2(void *ctx, const u8 *inp, const size_t len,
				     const u32 padbit);
asmlinkage void poly1305_blocks_avx512(void *ctx, const u8 *inp,
				       const size_t len, const u32 padbit);

static __ro_after_init DEFINE_STATIC_KEY_FALSE(poly1305_use_avx);
static __ro_after_init DEFINE_STATIC_KEY_FALSE(poly1305_use_avx2);
static __ro_after_init DEFINE_STATIC_KEY_FALSE(poly1305_use_avx512);

struct poly1305_arch_internal {
	union {
		struct {
			u32 h[5];
			u32 is_base2_26;
		};
		u64 hs[3];
	};
	u64 r[2];
	u64 pad;
	struct { u32 r2, r1, r4, r3; } rn[9];
};

asmlinkage void poly1305_block_sse2(u32 *h, const u8 *src,
				    const u32 *r, unsigned int blocks);
asmlinkage void poly1305_2block_sse2(u32 *h, const u8 *src, const u32 *r,
				     unsigned int blocks, const u32 *u);
#ifdef CONFIG_AS_AVX2
asmlinkage void poly1305_4block_avx2(u32 *h, const u8 *src, const u32 *r,
				     unsigned int blocks, const u32 *u);
static bool poly1305_use_avx2;
#endif

static int poly1305_simd_init(struct shash_desc *desc)
{
	struct poly1305_simd_desc_ctx *sctx = shash_desc_ctx(desc);

	sctx->uset = false;
#ifdef CONFIG_AS_AVX2
	sctx->wset = false;
#endif

	return crypto_poly1305_init(desc);
}

static void poly1305_simd_init(void *ctx, const u8 key[POLY1305_BLOCK_SIZE])
{
	u8 m[POLY1305_BLOCK_SIZE];

	memset(m, 0, sizeof(m));
	/* The poly1305 block function adds a hi-bit to the accumulator which
	 * we don't need for key multiplication; compensate for it. */
	a[4] -= 1 << 24;
	poly1305_block_sse2(a, m, b, 1);
}

static unsigned int poly1305_simd_blocks(struct poly1305_desc_ctx *dctx,
					 const u8 *src, unsigned int srclen)
{
	struct poly1305_simd_desc_ctx *sctx;
	unsigned int blocks, datalen;

	BUILD_BUG_ON(offsetof(struct poly1305_simd_desc_ctx, base));
	sctx = container_of(dctx, struct poly1305_simd_desc_ctx, base);

	if (!IS_ENABLED(CONFIG_AS_AVX) || !static_branch_likely(&poly1305_use_avx) ||
	    (len < (POLY1305_BLOCK_SIZE * 18) && !state->is_base2_26) ||
	    !crypto_simd_usable()) {
		convert_to_base2_64(ctx);
		poly1305_blocks_x86_64(ctx, inp, len, padbit);
		return;
	}

	do {
		const size_t bytes = min_t(size_t, len, SZ_4K);

		kernel_fpu_begin();
		if (IS_ENABLED(CONFIG_AS_AVX512) && static_branch_likely(&poly1305_use_avx512))
			poly1305_blocks_avx512(ctx, inp, bytes, padbit);
		else if (IS_ENABLED(CONFIG_AS_AVX2) && static_branch_likely(&poly1305_use_avx2))
			poly1305_blocks_avx2(ctx, inp, bytes, padbit);
		else
			poly1305_blocks_avx(ctx, inp, bytes, padbit);
		kernel_fpu_end();

		len -= bytes;
		inp += bytes;
	} while (len);
}

static void poly1305_simd_emit(void *ctx, u8 mac[POLY1305_DIGEST_SIZE],
			       const u32 nonce[4])
{
	if (!IS_ENABLED(CONFIG_AS_AVX) || !static_branch_likely(&poly1305_use_avx))
		poly1305_emit_x86_64(ctx, mac, nonce);
	else
		poly1305_emit_avx(ctx, mac, nonce);
}

void poly1305_init_arch(struct poly1305_desc_ctx *dctx, const u8 key[POLY1305_KEY_SIZE])
{
	poly1305_simd_init(&dctx->h, key);
	dctx->s[0] = get_unaligned_le32(&key[16]);
	dctx->s[1] = get_unaligned_le32(&key[20]);
	dctx->s[2] = get_unaligned_le32(&key[24]);
	dctx->s[3] = get_unaligned_le32(&key[28]);
	dctx->buflen = 0;
	dctx->sset = true;
}
EXPORT_SYMBOL(poly1305_init_arch);

static unsigned int crypto_poly1305_setdctxkey(struct poly1305_desc_ctx *dctx,
					       const u8 *inp, unsigned int len)
{
	unsigned int acc = 0;
	if (unlikely(!dctx->sset)) {
		datalen = crypto_poly1305_setdesckey(dctx, src, srclen);
		src += srclen - datalen;
		srclen = datalen;
	}

#ifdef CONFIG_AS_AVX2
	if (poly1305_use_avx2 && srclen >= POLY1305_BLOCK_SIZE * 4) {
		if (unlikely(!sctx->wset)) {
			if (!sctx->uset) {
				memcpy(sctx->u, dctx->r.r, sizeof(sctx->u));
				poly1305_simd_mult(sctx->u, dctx->r.r);
				sctx->uset = true;
			}
			memcpy(sctx->u + 5, sctx->u, sizeof(sctx->u));
			poly1305_simd_mult(sctx->u + 5, dctx->r.r);
			memcpy(sctx->u + 10, sctx->u + 5, sizeof(sctx->u));
			poly1305_simd_mult(sctx->u + 10, dctx->r.r);
			sctx->wset = true;
		}
		blocks = srclen / (POLY1305_BLOCK_SIZE * 4);
		poly1305_4block_avx2(dctx->h.h, src, dctx->r.r, blocks,
				     sctx->u);
		src += POLY1305_BLOCK_SIZE * 4 * blocks;
		srclen -= POLY1305_BLOCK_SIZE * 4 * blocks;
	}
#endif
	if (likely(srclen >= POLY1305_BLOCK_SIZE * 2)) {
		if (unlikely(!sctx->uset)) {
			memcpy(sctx->u, dctx->r.r, sizeof(sctx->u));
			poly1305_simd_mult(sctx->u, dctx->r.r);
			sctx->uset = true;
		}
		blocks = srclen / (POLY1305_BLOCK_SIZE * 2);
		poly1305_2block_sse2(dctx->h.h, src, dctx->r.r, blocks,
				     sctx->u);
		src += POLY1305_BLOCK_SIZE * 2 * blocks;
		srclen -= POLY1305_BLOCK_SIZE * 2 * blocks;
	}
	if (srclen >= POLY1305_BLOCK_SIZE) {
		poly1305_block_sse2(dctx->h.h, src, dctx->r.r, 1);
		srclen -= POLY1305_BLOCK_SIZE;
	}
	return srclen;
}

static int poly1305_simd_update(struct shash_desc *desc,
				const u8 *src, unsigned int srclen)
{
	struct poly1305_desc_ctx *dctx = shash_desc_ctx(desc);
	unsigned int bytes;

	/* kernel_fpu_begin/end is costly, use fallback for small updates */
	if (srclen <= 288 || !crypto_simd_usable())
		return crypto_poly1305_update(desc, src, srclen);

	kernel_fpu_begin();

	if (unlikely(dctx->buflen)) {
		bytes = min(srclen, POLY1305_BLOCK_SIZE - dctx->buflen);
		memcpy(dctx->buf + dctx->buflen, src, bytes);
		src += bytes;
		srclen -= bytes;
		dctx->buflen += bytes;

		if (dctx->buflen == POLY1305_BLOCK_SIZE) {
			poly1305_simd_blocks(dctx, dctx->buf,
					     POLY1305_BLOCK_SIZE);
			dctx->buflen = 0;
		}
	}

	if (likely(srclen >= POLY1305_BLOCK_SIZE)) {
		bytes = poly1305_simd_blocks(dctx, src, srclen);
		src += srclen - bytes;
		srclen = bytes;
	}

	kernel_fpu_end();

	if (unlikely(srclen)) {
		dctx->buflen = srclen;
		memcpy(dctx->buf, src, srclen);
	}

	return 0;
}

static struct shash_alg alg = {
	.digestsize	= POLY1305_DIGEST_SIZE,
	.init		= poly1305_simd_init,
	.update		= poly1305_simd_update,
	.final		= crypto_poly1305_final,
	.descsize	= sizeof(struct poly1305_simd_desc_ctx),
	.base		= {
		.cra_name		= "poly1305",
		.cra_driver_name	= "poly1305-simd",
		.cra_priority		= 300,
		.cra_blocksize		= POLY1305_BLOCK_SIZE,
		.cra_module		= THIS_MODULE,
	},
};

static int __init poly1305_simd_mod_init(void)
{
	if (!boot_cpu_has(X86_FEATURE_XMM2))
		return -ENODEV;

#ifdef CONFIG_AS_AVX2
	poly1305_use_avx2 = boot_cpu_has(X86_FEATURE_AVX) &&
			    boot_cpu_has(X86_FEATURE_AVX2) &&
			    cpu_has_xfeatures(XFEATURE_MASK_SSE | XFEATURE_MASK_YMM, NULL);
	alg.descsize = sizeof(struct poly1305_simd_desc_ctx);
	if (poly1305_use_avx2)
		alg.descsize += 10 * sizeof(u32);
#endif
	return crypto_register_shash(&alg);
}

static void __exit poly1305_simd_mod_exit(void)
{
	crypto_unregister_shash(&alg);
}

module_init(poly1305_simd_mod_init);
module_exit(poly1305_simd_mod_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Martin Willi <martin@strongswan.org>");
MODULE_DESCRIPTION("Poly1305 authenticator");
MODULE_ALIAS_CRYPTO("poly1305");
MODULE_ALIAS_CRYPTO("poly1305-simd");
