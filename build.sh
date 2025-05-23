#!/bin/bash

PATCH_MARKER=".patch_applied_ksu_susfs"
PATCH_FILE="$PWD/patches/patch-susfs.patch"
PATCH_DIR="$PWD/KernelSU-Next"

apply_ksu_susfs_patch() {
    if [[ "$KSU_OPTION" == "y" ]]; then
        if [ ! -f "$PATCH_MARKER" ]; then
            echo "Applying SuSFS patch to KernelSU..."
            patch -d "$PATCH_DIR" -p1 < "$PATCH_FILE" || {
                echo "Failed to apply SuSFS patch!"
                exit 1
            }
            touch "$PATCH_MARKER"
        else
            echo "SuSFS patch already applied. Skipping..."
        fi
    fi
}

if [[ "$KSU_OPTION" == "y" ]]; then
    if [ ! -d "KernelSU-Next" ]; then
        echo "KernelSU-Next not found. Initializing submodules..."
        git submodule update --init --recursive || {
            echo "Failed to initialize KernelSU-Next submodule!"
            exit 1
        }
    fi
fi

abort()
{
    cd -
    echo "-----------------------------------------------"
    echo "Kernel compilation failed! Exiting..."
    echo "-----------------------------------------------"
    exit -1
}

unset_flags()
{
    cat << EOF
Usage: $(basename "$0") [options]
Options:
    -m, --model [value]    Specify the model code of the phone
    -k, --ksu [y/N]        Include KernelSU
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --model|-m)
            MODEL="$2"
            shift 2
            ;;
        --ksu|-k)
            KSU_OPTION="$2"
            shift 2
            ;;
        *)\
            unset_flags
            exit 1
            ;;
    esac
done

echo "Preparing the build environment..."

pushd $(dirname "$0") > /dev/null
CORES=`cat /proc/cpuinfo | grep -c processor`

# Define toolchain variables
CLANG_DIR=$PWD/toolchain/clang-r416183b
GCC_DIR=$PWD/toolchain/gcc_4.9
PATH=$CLANG_DIR/bin:$CLANG_DIR/lib:$GCC_DIR/bin:$GCC_DIR/lib:$PATH

MAKE_ARGS="
LLVM=1 \
LLVM_IAS=1 \
ARCH=arm64 \
READELF=$CLANG_DIR/bin/llvm-readelf \
CROSS_COMPILE=$GCC_DIR/bin/aarch64-linux-gnu- \
O=out
"

# Define specific variables
case $MODEL in
r9s)
    KERNEL_DEFCONFIG=chiclet_r9s_defconfig
    BOARD=SRPUG16A010KU
;;
o1s)
    KERNEL_DEFCONFIG=chiclet_o1s_defconfig
    BOARD=SRPTH19C011KU
;;
t2s)
    KERNEL_DEFCONFIG=chiclet_t2s_defconfig
    BOARD=SRPTG24B014KU
;;
p3s)
    KERNEL_DEFCONFIG=chiclet_p3s_defconfig
    BOARD=SRPTH19D013KU
;;
*)
    unset_flags
    exit
esac

if [ -z $KSU_OPTION ]; then
    read -p "Include KernelSU (y/N): " KSU_OPTION
fi

if [[ "$KSU_OPTION" == "y" ]]; then
    KSU=ksu.config
fi

rm -rf build/out/$MODEL
mkdir -p build/out/$MODEL/zip/files
mkdir -p build/out/$MODEL/zip/META-INF/com/google/android

build_kernel() {
    # Build kernel image
    echo "-----------------------------------------------"
    echo "Defconfig: "$KERNEL_DEFCONFIG""

    if [ -z "$KSU" ]; then
        echo "KSU: N"
    else
        echo "KSU: $KSU"
    fi

    echo "-----------------------------------------------"
    echo "Building kernel using "$KERNEL_DEFCONFIG""
    echo "Generating configuration file..."
    echo "-----------------------------------------------"
    make ${MAKE_ARGS} -j$CORES $KERNEL_DEFCONFIG $KSU || abort

    echo "Building kernel..."
    echo "-----------------------------------------------"
    make ${MAKE_ARGS} -j$CORES || abort
}

build_boot() {
    echo "-----------------------------------------------"
    echo "Building boot.img RAMDisk..."
    mkdir -p build/out/$MODEL/boot_ramdisk00

    # Copy common files for boot.img's RAMDisk
    cp -a build/ramdisk/boot/boot_ramdisk00 build/out/$MODEL

    pushd build/out/$MODEL/boot_ramdisk00 > /dev/null
    find . ! -name . | LC_ALL=C sort | cpio -o -H newc -R root:root | lz4 -l > ../boot_ramdisk || abort
    popd > /dev/null

    echo "-----------------------------------------------"
    echo "Building boot.img..."

    cp -a out/arch/arm64/boot/Image build/out/$MODEL

    OUTPUT_FILE=build/out/$MODEL/boot.img
    RAMDISK_00=build/out/$MODEL/boot_ramdisk
    KERNEL=build/out/$MODEL/Image
    HEADER_VERSION=3
    OS_VERSION=11.0.0
    OS_PATCH_LEVEL=2025-04
    CMDLINE="androidboot.selinux=permissive loop.max_part=7"

	python3 toolchain/mkbootimg/mkbootimg.py --header_version $HEADER_VERSION --cmdline "$CMDLINE" --ramdisk $RAMDISK_00 \
	--os_version $OS_VERSION --os_patch_level $OS_PATCH_LEVEL --kernel $KERNEL --output $OUTPUT_FILE || abort
}

build_dtb() {
    echo "-----------------------------------------------"
    # echo "Building DTB image..."
    ./toolchain/mkdtimg cfg_create build/out/$MODEL/dtb.img build/dtconfigs/exynos2100.cfg -d out/arch/arm64/boot/dts/exynos || abort 

    echo "-----------------------------------------------"
    echo "Building DTBO image..."
    ./toolchain/mkdtimg cfg_create build/out/$MODEL/dtbo.img build/dtconfigs/$MODEL.cfg -d out/arch/arm64/boot/dts/samsung/$MODEL || abort
    
}

build_modules() {
    MODULES_FOLDER=modules
    rm -rf out/$MODULES_FOLDER

    echo "-----------------------------------------------"
    echo "Building modules..."
    # Strip modules and place them in modules folder
    make ${MAKE_ARGS} INSTALL_MOD_PATH=$MODULES_FOLDER INSTALL_MOD_STRIP=1 modules_install || abort

    # List of kernel modules to remove
    # Some of the kernel modules are in /vendor_dlkm or /vendor/lib/modules and not in vendor_boot
    # So we will remove them from the folder and run depmod again to update the files
    FILENAMES="
    input_booster_lkm.ko
    sec_debug_sched_info.ko
    "
    for FILENAME in $FILENAMES; do
        FILE=$(find out/$MODULES_FOLDER -type f -name "$FILENAME")
        echo "$FILE" | xargs rm -f
    done

    # Now we run depmod to update the dep/softdep files
    # For this we need the kernel version
    KERNEL_DIR_PATH=$(find "out/$MODULES_FOLDER/lib/modules" -maxdepth 1 -type d -name "5.4*") || abort
    KERNEL_VERSION=$(basename $KERNEL_DIR_PATH) || abort

    # And finally depmod itself
    depmod -a -b out/$MODULES_FOLDER $KERNEL_VERSION || abort

    # depmod updates modules.alias, modules.dep and modules.softdep
    # But the module order is not updated by depmod
    # We have to remove the filenames ourselves
    # But first, our vendor_boot needs modules.order definitions that go like:
    # fingerprint.ko
    # Clang generates modules.order definitions like this
    # kernel/drivers/fingerprint/fingerprint.ko
    # So we sed the file to adapt
    sed -i 's/.*\///g' $KERNEL_DIR_PATH/modules.order 

    # Now we sed the bad filenames out of the file with a loop
    for FILENAME in $FILENAMES; do
        sed -i "/$FILENAME/d" "$KERNEL_DIR_PATH/modules.order"
    done

    # Now we have to order the modules
    # These files have to be at the top of modules.order in this order, and then we can keep the default order.
    # Samsung wants the file to be renamed to modules.load anyways, so we will craft our own modules.load file based on modules.order
    touch $KERNEL_DIR_PATH/modules.load

    INITIAL_ORDER="
    dss.ko
    exynos-chipid_v2.ko
    exynos-reboot.ko
    exynos2100-itmon.ko
    exynos-pmu-if.ko
    s3c2410_wdt.ko
    exynos-ecc-handler.ko
    exynos-coresight.ko
    debug-snapshot-qd.ko
    eat.ko
    exynos-adv-tracer-s2d.ko
    ehld.ko
    exynos-debug-test.ko
    hardlockup-debug.ko
    exynos_acpm.ko
    exynos_pm_qos.ko
    exynos-s2mpu.ko
    exynos-pd_el3.ko
    ect_parser.ko
    cmupmucal.ko
    clk_exynos.ko
    clk-exynos-audss.ko
    exynos_mct.ko
    pinctrl-samsung-core.ko
    exynos-cpupm.ko
    i2c-exynos5.ko
    acpm-mfd-bus.ko
    s2mps24_mfd.ko
    s2mps23_mfd.ko
    pmic_class.ko
    s2mps23-regulator.ko
    s2mps24-regulator.ko
    phy-exynos-usbdrd-super.ko
    sec_debug_mode.ko
    fingerprint.ko
    "

    # First we add the order from Samsung into our new modules.load
    # And we sed it out of modules.order
    for LINE in $INITIAL_ORDER; do
        echo $LINE >> $KERNEL_DIR_PATH/modules.load
        sed -i "/$LINE/d" "$KERNEL_DIR_PATH/modules.order"
    done

    # Now we add the remaining lines from modules.order into modules.load
    while IFS= read -r line; do
        echo "$line" >> "$KERNEL_DIR_PATH/modules.load"
    done < "$KERNEL_DIR_PATH/modules.order"

    # Now we have to also modify modules.dep
    # Android generates them like this
    # kernel/drivers/dma/samsung-dma.ko: kernel/drivers/dma/pl330.ko
    # But Samsung wants them like this
    # /lib/modules/samsung-dma.ko: /lib/modules/pl330.ko
    # So we will format it with sed
    sed -i 's/\(kernel\/[^: ]*\/\)\([^: ]*\.ko\)/\/lib\/modules\/\2/g' "$KERNEL_DIR_PATH/modules.dep"

    # Now the modules and their configuration descriptor files are ready, we move them to a folder and create the new second ramdisk
    # The second ramdisk should contain a /lib/modules where the modules are located
    mkdir -p build/out/$MODEL/modules/lib/modules

    find $KERNEL_DIR_PATH -name '*.ko' -exec cp '{}' build/out/$MODEL/modules/lib/modules ';'

    # We also copy the module configuration descriptors
    cp $KERNEL_DIR_PATH/modules.{alias,dep,softdep,load} build/out/$MODEL/modules/lib/modules
}

build_vendor_boot() {
    echo "-----------------------------------------------"
    echo "Building vendor_boot RAMDisks..."
    # Copy common vendor_ramdisk00 files to build/out
    cp -a build/ramdisk/vendor_boot/ramdisk00 build/out/$MODEL/vendor_ramdisk00

    # Copy module files for vendor_ramdisk00
    cp -a build/out/$MODEL/modules/lib/* build/out/$MODEL/vendor_ramdisk00/lib

    # Copy device firmware files for vendor_ramdisk00
    cp -a build/ramdisk/vendor_boot/vendor_firmware/$MODEL/* build/out/$MODEL/vendor_ramdisk00

    # Pack RAMDisks
    # vendor_ramdisk == ramdisk00
    pushd build/out/$MODEL/vendor_ramdisk00 > /dev/null
    find . ! -name . | LC_ALL=C sort | cpio -o -H newc -R root:root | gzip > ../$MODEl/vendor_ramdisk || abort
    popd > /dev/null

    echo "-----------------------------------------------"
    echo "Building vendor_boot image..."

    OUTPUT_FILE=build/out/$MODEL/vendor_boot.img
    DTB_PATH=build/out/$MODEL/dtb.img
    RAMDISK_00=build/out/$MODEL/vendor_ramdisk
    HEADER_VERSION=3
    BASE=0x00000000
    PAGESIZE=0x00001000
    KERNEL_OFFSET=0x80008000
    RAMDISK_OFFSET=0x84000000
    TAGS_OFFSET=0x80000000
    DTB_OFFSET=0x0000000081F00000
    CMDLINE="androidboot.selinux=permissive loop.max_part=7"

    python3 toolchain/mkbootimg/mkbootimg.py --header_version $HEADER_VERSION --pagesize $PAGESIZE --base $BASE --kernel_offset $KERNEL_OFFSET \
	--ramdisk_offset $RAMDISK_OFFSET --tags_offset $TAGS_OFFSET --dtb_offset $DTB_OFFSET --vendor_cmdline "$CMDLINE" --board $BOARD --dtb $DTB_PATH  \
	--vendor_ramdisk $RAMDISK_00 --vendor_boot $OUTPUT_FILE || abort
}

build_zip() {
    echo "-----------------------------------------------"
    echo "Building zip..."
    cp build/out/$MODEL/boot.img build/out/$MODEL/zip/files/boot.img
    cp build/out/$MODEL/vendor_boot.img build/out/$MODEL/zip/files/vendor_boot.img
    cp build/out/$MODEL/dtbo.img build/out/$MODEL/zip/files/dtbo.img
    cp build/update-binary build/out/$MODEL/zip/META-INF/com/google/android/update-binary
    cp build/updater-script build/out/$MODEL/zip/META-INF/com/google/android/updater-script

    version=$(grep -o 'CONFIG_LOCALVERSION="[^"]*"' arch/arm64/configs/$KERNEL_DEFCONFIG | cut -d '"' -f 2)
    version=${version:1}
    pushd build/out/$MODEL/zip > /dev/null
    DATE=`date +"%d-%m-%Y_%H-%M-%S"`

    if [[ "$KSU_OPTION" == "y" ]]; then
        NAME="$version"_"$MODEL"_UNOFFICIAL_KSU_"$DATE".zip
    else
        NAME="$version"_"$MODEL"_UNOFFICIAL_"$DATE".zip
    fi
    zip -r -qq ../"$NAME" .
    popd > /dev/null
}

apply_ksu_susfs_patch
build_kernel
build_boot
build_dtb
build_modules
build_vendor_boot
build_zip

popd > /dev/null
echo "-----------------------------------------------"
echo "Build finished successfully!"