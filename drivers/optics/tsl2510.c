/*
 * Copyright (C) 2018 Samsung Electronics Co., Ltd. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#define VENDOR				"AMS"

#define TSL2510_CHIP_NAME	"TSL2510"

#define VERSION				"2"
#define SUB_VERSION			"0"
#define VENDOR_VERSION		"a"

#define MODULE_NAME_ALS		"als_rear"

#define TSL2510_SLAVE_I2C_ADDR_REVID_V0 0x39
#define TSL2510_SLAVE_I2C_ADDR_REVID_V1 0x29

#define AMSDRIVER_I2C_RETRY_DELAY	10
#define AMSDRIVER_I2C_MAX_RETRIES	5

#define I2C_1P8_DISABLE
//#define CONFIG_AMS_OPTICAL_SENSOR_FIFO

/* AWB/Flicker Definition */
#define ALS_AUTOGAIN
#define BYTE				2
#define AWB_INTERVAL		20 /* 20 sample(from 17 to 28) */

#define CONFIG_SKIP_CNT		8
#define FLICKER_FIFO_THR	16
#define FLICKER_DATA_CNT	200
#define FLICKER_FIFO_READ	-2

#define TSL2510_IOCTL_MAGIC		0xFD
#define TSL2510_IOCTL_READ_FLICKER	_IOR(TSL2510_IOCTL_MAGIC, 0x01, uint16_t *)

#if IS_ENABLED(CONFIG_LEDS_S2MPB02)
#define CONFIG_AMS_OPTICAL_SENSOR_EOL_MODE
#endif
#ifdef AMS_BUILD
#include <linux/i2c/ams/tsl2510.h>
#else
#include "tsl2510.h"
#endif
#include <linux/kfifo.h>

#ifdef CONFIG_AMS_OPTICAL_SENSOR_EOL_MODE
#include <linux/leds-s2mpb02.h>

#define DEFAULT_DUTY_50HZ		5000
#define DEFAULT_DUTY_60HZ		4166

#define MAX_TEST_RESULT			256
#define EOL_COUNT				20
#define EOL_SKIP_COUNT			5
#define EOL_GAIN				1000

#define DEFAULT_IR_SPEC_MIN		0
#define DEFAULT_IR_SPEC_MAX		5000000
#define DEFAULT_IC_SPEC_MIN		0
#define DEFAULT_IC_SPEC_MAX		200000


static u32 gSpec_ir_min = DEFAULT_IR_SPEC_MIN;
static u32 gSpec_ir_max = DEFAULT_IR_SPEC_MAX;
static u32 gSpec_clear_min = DEFAULT_IR_SPEC_MIN;
static u32 gSpec_clear_max = DEFAULT_IR_SPEC_MAX;
static u32 gSpec_icratio_min = DEFAULT_IC_SPEC_MIN;
static u32 gSpec_icratio_max = DEFAULT_IC_SPEC_MAX;

#define FREQ_SPEC_MARGIN        5
#define MIN_SPEC_COUNT          (EOL_COUNT/2)
#define FREQ100_SPEC_IN(X,Y)    (((X >= (100 - FREQ_SPEC_MARGIN)) && (X <= (100 + FREQ_SPEC_MARGIN)) && (Y > MIN_SPEC_COUNT))?"PASS":"FAIL")
#define FREQ120_SPEC_IN(X,Y)    (((X >= (120 - FREQ_SPEC_MARGIN)) && (X <= (120 + FREQ_SPEC_MARGIN)) && (Y > MIN_SPEC_COUNT))?"PASS":"FAIL")

#define IR_SPEC_IN(X)		((X >= gSpec_ir_min && X <= gSpec_ir_max)?"PASS":"FAIL")
#define CLEAR_SPEC_IN(X)	((X >= gSpec_clear_min && X <= gSpec_clear_max)?"PASS":"FAIL")
//#define ICRATIO_SPEC_IN(X)	((X >= gSpec_icratio_min && X <= gSpec_icratio_max)?"PASS":"FAIL")
#define ICRATIO_SPEC_IN(X)	"PASS"
#endif

static int als_debug = 1;
static int als_info;



static DECLARE_KFIFO(ams_fifo, u8, 2 * PAGE_SIZE);

module_param(als_debug, int, S_IRUGO | S_IWUSR);
module_param(als_info, int, S_IRUGO | S_IWUSR);

static struct tsl2510_device_data *tsl2510_data;

#define AMS_ROUND_SHFT_VAL				4
#define AMS_ROUND_ADD_VAL				(1 << (AMS_ROUND_SHFT_VAL - 1))
#define AMS_ALS_GAIN_FACTOR				1000
#define CPU_FRIENDLY_FACTOR_1024		1
#define AMS_ALS_Cc						(118 * CPU_FRIENDLY_FACTOR_1024)
#define AMS_ALS_Rc						(112 * CPU_FRIENDLY_FACTOR_1024)
#define AMS_ALS_Gc						(172 * CPU_FRIENDLY_FACTOR_1024)
#define AMS_ALS_Bc						(180 * CPU_FRIENDLY_FACTOR_1024)
#define AMS_ALS_Wbc						(111 * CPU_FRIENDLY_FACTOR_1024)

#define AMS_ALS_FACTOR					1000

#ifdef CONFIG_AMS_OPTICAL_SENSOR_259x
#define AMS_ALS_TIMEBASE				(100000) /* in uSec, see data sheet */
#define AMS_ALS_ADC_MAX_COUNT			(37888) /* see data sheet */
#else
#define AMS_ALS_TIMEBASE				(2780) /* in uSec, see data sheet */
#define AMS_ALS_ADC_MAX_COUNT			(1024) /* see data sheet */
#endif
#define AMS_ALS_THRESHOLD_LOW			(5) /* in % */
#define AMS_ALS_THRESHOLD_HIGH			(5) /* in % */

#define AMS_ALS_ATIME					(50000)

#define AMS_AGC_MAX_GAIN                        (4096000)

#define AMS_AGC_NUM_SAMPLES                        (20)

#define WIDEBAND_CONST          (1.6)
#define CLEAR_CONST             (2)



static int ams_getWord(AMS_PORT_portHndl *portHndl, ams_deviceRegister_t reg, uint16_t *readData);
static int ams_setWord(AMS_PORT_portHndl *portHndl, ams_deviceRegister_t reg, uint16_t setData);




/* REENABLE only enables those that were on record as being enabled */
#define AMS_REENABLE(ret)				{ret = ams_setByte(ctx->portHndl, DEVREG_ENABLE, ctx->shadowEnableReg); }
/* DISABLE_ALS disables ALS w/o recording that as its new state */
#define AMS_DISABLE_ALS(ret)			{ret = ams_setField(ctx->portHndl, DEVREG_ENABLE, LOW, (MASK_AEN)); }
#define AMS_REENABLE_ALS(ret)			{ret = ams_setField(ctx->portHndl, DEVREG_ENABLE, HIGH, (MASK_AEN)); }

#define AMS_DISABLE_FD(ret)			{ret = ams_setField(ctx->portHndl, DEVREG_ENABLE, LOW, (MASK_FDEN)); }
#define AMS_REENABLE_FD(ret)			{ret = ams_setField(ctx->portHndl, DEVREG_ENABLE, HIGH, (MASK_FDEN)); }
#define AMS_REENABLE_FD_PON(ret)			{ret = ams_setField(ctx->portHndl, DEVREG_ENABLE, HIGH, (MASK_FDEN | PON)); }

#define AMS_ENABLE_PON(ret)			{ret = ams_setByte(ctx->portHndl, DEVREG_ENABLE, PON); }
#define AMS_DISABLE_FDINT(ret)			{ret = ams_setField(ctx->portHndl, DEVREG_INTENAB, LOW, MASK_FIEN); }
#define AMS_REENABLE_FDINT(ret)			{ret = ams_setField(ctx->portHndl, DEVREG_INTENAB, HIGH, MASK_FIEN); }


#define AMS_SET_SAMPLE_TIME(uSec, ret); {ret = ams_setByte(ctx->portHndl, DEVREG_SAMPLE_TIME0,   (0xFF) & ((alsSampleTimeUsToReg(uSec))<<0));\
                                        ret = ams_setByte(ctx->portHndl, DEVREG_SAMPLE_TIME1,   (0x07) & ((alsSampleTimeUsToReg(uSec))>>8));}


#define AMS_SET_ALS_TIME(uSec, ret)     {ret = ams_setByte(ctx->portHndl, DEVREG_ALS_NR_SAMPLES0,   (0xFF) & ((alsTimeUsToReg(uSec, ctx->portHndl))<<0));\
                                        ret = ams_setByte(ctx->portHndl, DEVREG_ALS_NR_SAMPLES1,   (0xFF) & ((alsTimeUsToReg(uSec, ctx->portHndl))>>8));}


// Flicker Measurement time = SAMPLE_TIME * FLICKER_NUM_SAMPLES
#define AMS_SET_FLICKER_NUM_SAMPLES(Num, ret)       {ret = ams_setByte(ctx->portHndl, DEVREG_FD_NR_SAMPLES0,   (0xFF) & ((Num-1)<<0));\
                                                    ret = ams_setField(ctx->portHndl, DEVREG_FD_NR_SAMPLES1,   ((0x07) & ((Num-1)>>8)), 0x07);}

//#define AMS_GET_ALS_TIME(uSec, ret)		{ret = ams_getByte(ctx->portHndl, DEVREG_ATIME,   alsTimeUsToReg(uSec)); }

#define AMS_GET_ALS_GAIN(scaledGain, scaledGain1, gain, ret)    {ret = ams_getByte(ctx->portHndl, DEVREG_ALS_STATUS2, &(gain)); \
                                                                scaledGain = alsGain_conversion[(gain) & 0x0F]; \
                                                                scaledGain1 = alsGain_conversion[(((gain) & 0xF0)>> 4)]; }

//#define AMS_SET_ALS_STEP_TIME(uSec, ret)		{ret = ams_setWord(ctx->portHndl, DEVREG_ASTEPL, alsTimeUsToReg(uSec * 1000)); }

#define AMS_SET_ALS_GAIN0(mGain, ret)	{ret = ams_setField(ctx->portHndl, DEVREG_MEAS_SEQR_STEP0_MOD_GAINX_L, alsGainToReg(mGain), MASK_AGAIN0); }
#define AMS_SET_ALS_GAIN1(mGain, ret)	{ret = ams_setField(ctx->portHndl, DEVREG_MEAS_SEQR_STEP0_MOD_GAINX_L, (((alsGainToReg(mGain))<<4) & 0xF0), MASK_AGAIN1); }

#define AMS_SET_ALS_PERS(persCode, ret)	{ret = ams_setField(ctx->portHndl, DEVREG_CFG5, (persCode), MASK_APERS); }

#define AMS_SET_ALS_AINT_DIRECT(x, ret)	{ret = ams_setField(ctx->portHndl, DEVREG_CFG2, x, MASK_AINT_DIRECT); }

#define AMS_SET_ALS_THRS_LOW(x, ret)    {ret = ams_setByte(ctx->portHndl, DEVREG_AILT0, (uint8_t)(((0x000000ff)&(x))>>0) );\
                                        ret = ams_setByte(ctx->portHndl, DEVREG_AILT1, (uint8_t)(((0x0000ff00)&(x))>>8));\
                                        ret = ams_setByte(ctx->portHndl, DEVREG_AILT2, (uint8_t)(((0x00ff0000)&(x))>>16));}

#define AMS_SET_ALS_THRS_HIGH(x, ret)   {ret = ams_setByte(ctx->portHndl, DEVREG_AIHT0, (uint8_t)(((0x000000ff)&(x))>>0) );\
                                        ret = ams_setByte(ctx->portHndl, DEVREG_AIHT1, (uint8_t)(((0x0000ff00)&(x))>>8));\
                                        ret = ams_setByte(ctx->portHndl, DEVREG_AIHT2, (uint8_t)(((0x00ff0000)&(x))>>16));}

/* Get CRGB and whatever Wideband it may have */
#define AMS_ALS_GET_ALS_DATA(x, ret)		{ret = ams_getBuf(ctx->portHndl, DEVREG_ALS_DATAL0, (uint8_t *) (x), 4); }
#define AMS_AGC_ASAT_MODE(x,ret)     {ret = ams_setField(ctx->portHndl, DEVREG_MEAS_SEQR_STEP1_MOD_PHDX_SMUX_H, x, (MASK_AGC_ASAT)); }
#define AMS_AGC_PREDICT_MODE(x,ret)  {ret = ams_setField(ctx->portHndl, DEVREG_MEAS_SEQR_STEP2_MOD_PHDX_SMUX_H, x, (MASK_AGC_PREDICT)); }

#define AMS_AGC_ENABLE(ret)     {ret = ams_setByte(ctx->portHndl, DEVREG_MOD_CALIB_CFG0,   0x01);\
                                ret = ams_setField(ctx->portHndl, DEVREG_MOD_CALIB_CFG2,   HIGH, (MASK_MOD_CALIB_NTH_ITERATION_AGC_ENABLE));}

#define AMS_AGC_DISABLE(ret)    {ret = ams_setByte(ctx->portHndl, DEVREG_MOD_CALIB_CFG0,   0xFF);\
                                ret = ams_setField(ctx->portHndl, DEVREG_MOD_CALIB_CFG2,   LOW, (MASK_MOD_CALIB_NTH_ITERATION_AGC_ENABLE));}


// SET AGC MAX GAIN
#define AMS_SET_AGC_MAX_GAIN(mGain, ret)	{ret = ams_setField(ctx->portHndl, DEVREG_CFG8, (((alsGainToReg(mGain))<<4) & 0xF0), MASK_MAX_MOD_GAIN);}

// AGC Number of sample
#define AMS_SET_AGC_NR_SAMPLES(Num, ret)    {ret = ams_setByte(ctx->portHndl, DEVREG_AGC_NR_SAMPLES_LO,   ((0xFF) & ((Num - 1)>>0)));\
                                            ret = ams_setByte(ctx->portHndl, DEVREG_AGC_NR_SAMPLES_HI,   ((0xFF) & ((Num - 1)>>8)));}

#define AMS_FIFO_CLEAR(ret)            {ret = ams_setField(ctx->portHndl, DEVREG_CONTROL, HIGH, MASK_FIFO_CLR);} //FIFO Buffer , FINT, FIFO_OV, FIFO_LVL all clear


// AGC Number of sample  0 ~ 511
#define AMS_SET_FIFO_THR(Num, ret)      {ret = ams_setByte(ctx->portHndl, DEVREG_FIFO_THR,   ((0x01FF & Num) >>1) & (0xFF));\
                                        ret = ams_setByte(ctx->portHndl, DEVREG_CFG2,   ((0x0001 & Num) & 0xFF));}


// MEAS_SEQR_RESIDUAL_0,//0xD2
#define AMS_RESIDUAL_MODE(x,ret)  {ret = ams_setField(ctx->portHndl, DEVREG_MEAS_SEQR_RESIDUAL_0, x, 0xFF); }


// Enable writing of FD_GAIN to FIFO after each complete flicker measurment
#define AMS_FD_GAIN_TO_FIFO(ret)  {ret = ams_setField(ctx->portHndl, DEVREG_MEAS_MODE1, HIGH, MASK_MOD_FIFO_FD_GAIN_WRITE_ENABLE); }

// Enable writing of FD_END_MARKER to FIFO after each complete flicker measurment
#define AMS_FD_END_MARKER_TO_FIFO(ret)  {ret = ams_setField(ctx->portHndl, DEVREG_MEAS_MODE1, HIGH, MASK_MOD_FIFO_FD_END_MARKER_WRITE_ENABLE); }


#define AMS_ALS_SENSOR_2PD_TURNON(ret)          {ret = ams_setByte(ctx->portHndl, DEVREG_MEAS_SEQR_STEP0_MOD_PHDX_SMUX_L, 0x06); \
                                                ret = ams_setByte(ctx->portHndl, DEVREG_MEAS_SEQR_STEP0_MOD_PHDX_SMUX_H, 0x00);}

#define AMS_ALS_SENSOR_6PD_TURNON(ret)          {ret = ams_setByte(ctx->portHndl, DEVREG_MEAS_SEQR_STEP0_MOD_PHDX_SMUX_L, 0x66); \
                                                ret = ams_setByte(ctx->portHndl, DEVREG_MEAS_SEQR_STEP0_MOD_PHDX_SMUX_H, 0x06);}


#define AMS_MOD_FREQ_HALF_FREQ(ret)  {ret = ams_setByte(ctx->portHndl, DEVREG_CFG7, 0x00 ); } //MASK_MOD_DIVIDER_SELECT 0 , 6 clock half of freq
#define AMS_MOD_FREQ_FULL_FREQ(ret)  {ret = ams_setByte(ctx->portHndl, DEVREG_CFG7, 0x01); } //MASK_MOD_DIVIDER_SELECT 1 , 12 clock default


#define TSL2510_MEASUREMENT_SEQUENCER_MOD0_FD_PATTERN_SHIFT 0
#define TSL2510_MASK_MEASUREMENT_SEQUENCER_MOD0_FD_PATTERN (0x0F << TSL2510_MEASUREMENT_SEQUENCER_MOD0_FD_PATTERN_SHIFT)

#define TSL2510_MEASUREMENT_SEQUENCER_MOD1_FD_PATTERN_SHIFT 4
#define TSL2510_MASK_MEASUREMENT_SEQUENCER_MOD1_FD_PATTERN (0x0F << TSL2510_MEASUREMENT_SEQUENCER_MOD1_FD_PATTERN_SHIFT)

#define TSL2510_MEASUREMENT_SEQUENCER_MOD2_FD_PATTERN_SHIFT 4
#define TSL2510_MASK_MEASUREMENT_SEQUENCER_MOD2_FD_PATTERN (0x0F << TSL2510_MEASUREMENT_SEQUENCER_MOD2_FD_PATTERN_SHIFT)

#define TSL2510_MEASUREMENT_SEQUENCER_ALS_PATTERN_SHIFT 0
#define TSL2510_MASK_MEASUREMENT_SEQUENCER_ALS_PATTERN (0x0F << TSL2510_MEASUREMENT_SEQUENCER_ALS_PATTERN_SHIFT)

#define TSL2510_MOD_FIFO_FD_GAIN_WRITE_ENABLE_SHIFT 5
#define TSL2510_MASK_MOD_FIFO_FD_GAIN_WRITE_ENABLE (0x01 << TSL2510_MOD_FIFO_FD_GAIN_WRITE_ENABLE_SHIFT)

#define TSL2510_MOD_FIFO_FD_END_MARKER_WRITE_ENABLE_SHIFT 7
#define TSL2510_MASK_MOD_FIFO_FD_END_MARKER_WRITE_ENABLE (0x01 << TSL2510_MOD_FIFO_FD_END_MARKER_WRITE_ENABLE_SHIFT)

typedef struct {
    uint8_t deviceId;
    uint8_t deviceIdMask;
    uint8_t deviceRef;
    uint8_t deviceRefMask;
    ams_deviceIdentifier_e device;
} ams_deviceIdentifier_t;

typedef struct _fifo {
    uint32_t AdcClear;
    uint32_t AdcWb;
} adcDataSet_t;

#define AMS_PORT_LOG_CRGB_W(dataset) \
    ALS_info("%s - C = %u, WB = %u\n", __func__ \
            , dataset.AdcClear \
            , dataset.AdcWb	\
            )

static ams_deviceIdentifier_t deviceIdentifier[] = {
    { AMS_DEVICE_ID, AMS_DEVICE_ID_MASK, AMS_REV_ID, AMS_REV_ID_MASK, AMS_TSL2510 },
    { AMS_DEVICE_ID, AMS_DEVICE_ID_MASK, AMS_REV_ID_UNTRIM, AMS_REV_ID_MASK, AMS_TSL2510_UNTRIM },
    { 0, 0, 0, 0, AMS_LAST_DEVICE }
};
#define coef_a 61ULL //  0.06061 * 1000 , scaled
#define coef_b 45ULL //  0.04537 * 1000 , scaled

deviceRegisterTable_t deviceRegisterDefinition[DEVREG_REG_MAX] = {
    {	0x40	,	0x00	},		/*	DEVREG_MOD_CHANNEL_CTRL	*/
    {	0x80	,	0x00	},		/*	DEVREG_ENABLE	*/
    {	0x81	,	0x04	},		/*	DEVREG_MEAS_MODE0	*/
    {	0x82	,	0x0C	},		/*	DEVREG_MEAS_MODE1	*/
    {	0x83	,	0xB3	},		/*	DEVREG_SAMPLE_TIME0	*/
    {	0x84	,	0x00	},		/*	DEVREG_SAMPLE_TIME1	*/
    {	0x85	,	0x00	},		/*	DEVREG_ALS_NR_SAMPLES0	*/
    {	0x86	,	0x00	},		/*	DEVREG_ALS_NR_SAMPLES1	*/
    {	0x87	,	0x00	},		/*	DEVREG_FD_NR_SAMPLES0	*/
    {	0x88	,	0x00	},		/*	DEVREG_FD_NR_SAMPLES1	*/
    {	0x89	,	0x00	},		/*	DEVREG_WTIME	*/
    {	0x8A	,	0x00	},		/*	DEVREG_AILT0	*/
    {	0x8B	,	0x00	},		/*	DEVREG_AILT1	*/
    {	0x8C	,	0x00	},		/*	DEVREG_AILT2	*/
    {	0x8D	,	0x00	},		/*	DEVREG_AIHT0	*/
    {	0x8E	,	0x00	},		/*	DEVREG_AIHT1	*/
    {	0x8F	,	0x00	},		/*	DEVREG_AIHT2	*/
    {	0x90	,	0x00	},		/*	DEVREG_AUXID	*/
    {	0x91	,	0x00	},		/*	DEVREG_REVID	*/
    {	0x92	,	0x17	},		/*	DEVREG_ID	*/
    {	0x93	,	0x00	},		/*	DEVREG_STATUS	*/
    {	0x94	,	0x00	},		/*	DEVREG_ALS_STATUS	*/
    {	0x95	,	0x00	},		/*	DEVREG_DATAL0	*/
    {	0x96	,	0x00	},		/*	DEVREG_DATAH0	*/
    {	0x97	,	0x00	},		/*	DEVREG_DATAL1	*/
    {	0x98	,	0x00	},		/*	DEVREG_DATAH1	*/
    {	0x99	,	0x00	},		/*	DEVREG_DATAL2	*/
    {	0x9A	,	0x00	},		/*	DEVREG_DATAH2	*/
    {	0x9B	,	0x00	},		/*	DEVREG_ALS_STATUS2	*/
    {	0x9C	,	0x00	},		/*	DEVREG_ALS_STATUS3	*/
    {	0x9D	,	0x00	},		/*	DEVREG_STATUS2	*/
    {	0x9E	,	0x08	},		/*	DEVREG_STATUS3	*/
    {	0x9F	,	0x00	},		/*	DEVREG_STATUS4	*/
    {	0xA0	,	0x00	},		/*	DEVREG_STATUS5	*/
    {	0xA1	,	0x08	},		/*	DEVREG_CFG0	*/
    {	0xA2	,	0x00	},		/*	DEVREG_CFG1	*/
    {	0xA3	,	0x01	},		/*	DEVREG_CFG2	*/
    {	0xA4	,	0x00	},		/*	DEVREG_CFG3	*/
    {	0xA5	,	0x00	},		/*	DEVREG_CFG4	*/
    {	0xA6	,	0x00	},		/*	DEVREG_CFG5	*/
    {	0xA7	,	0x03	},		/*	DEVREG_CFG6	*/
    {	0xA8	,	0x01	},		/*	DEVREG_CFG7	*/
    {	0xA9	,	0xC4	},		/*	DEVREG_CFG8	*/
    {	0xAA	,	0x00	},		/*	DEVREG_CFG9	*/
    {	0xAC	,	0x00	},		/*	DEVREG_AGC_NR_SAMPLES_LO	*/
    {	0xAD	,	0x00	},		/*	DEVREG_AGC_NR_SAMPLES_HI	*/
    {	0xAE	,	0x00	},		/*	DEVREG_TRIGGER_MODE	*/
    {	0xB1	,	0x00	},		/*	DEVREG_CONTROL	*/
    {	0xBA	,	0x00	},		/*	DEVREG_INTENAB	*/
    {	0xBB	,	0x00	},		/*	DEVREG_SIEN	*/
    {	0xCE	,	0x80	},		/*	DEVREG_MOD_COMP_CFG1	*/
    {	0xCF	,	0x01	},		/*	DEVREG_MEAS_SEQR_FD_0	*/
    {	0xD0	,	0x01	},		/*	DEVREG_MEAS_SEQR_ALS_FD_1	*/
    {	0xD1	,	0x01	},		/*	DEVREG_MEAS_SEQR_APERS_AND_VSYNC_WAIT	*/
    {	0xD2	,	0xFF	},		/*	DEVREG_MEAS_SEQR_RESIDUAL_0	*/
    {	0xD3	,	0x1F	},		/*	DEVREG_MEAS_SEQR_RESIDUAL_1_AND_WAIT	*/
    {	0xD4	,	0x88	},		/*	DEVREG_MEAS_SEQR_STEP0_MOD_GAINX_L	*/
    {	0xD6	,	0x88	},		/*	DEVREG_MEAS_SEQR_STEP1_MOD_GAINX_L	*/
    {	0xD8	,	0x88	},		/*	DEVREG_MEAS_SEQR_STEP2_MOD_GAINX_L	*/
    {	0xDA	,	0x88	},		/*	DEVREG_MEAS_SEQR_STEP3_MOD_GAINX_L	*/
    {	0xDC	,	0x66	},		/*	DEVREG_MEAS_SEQR_STEP0_MOD_PHDX_SMUX_L	*/
    {	0xDD	,	0x06	},		/*	DEVREG_MEAS_SEQR_STEP0_MOD_PHDX_SMUX_H	*/
    {	0xDE	,	0x84	},		/*	DEVREG_MEAS_SEQR_STEP1_MOD_PHDX_SMUX_L	*/
    {	0xDF	,	0xF3	},		/*	DEVREG_MEAS_SEQR_STEP1_MOD_PHDX_SMUX_H	*/
    {	0xE0	,	0x07	},		/*	DEVREG_MEAS_SEQR_STEP2_MOD_PHDX_SMUX_L	*/
    {	0xE1	,	0xF8	},		/*	DEVREG_MEAS_SEQR_STEP2_MOD_PHDX_SMUX_H	*/
    {	0xE2	,	0x24	},		/*	DEVREG_MEAS_SEQR_STEP3_MOD_PHDX_SMUX_L	*/
    {	0xE3	,	0x03	},		/*	DEVREG_MEAS_SEQR_STEP3_MOD_PHDX_SMUX_H	*/
    {	0xE4	,	0xFF	},		/*	DEVREG_MOD_CALIB_CFG0	*/
    {	0xE6	,	0xD3	},		/*	DEVREG_MOD_CALIB_CFG2	*/
    {	0xF2	,	0x00	},		/*	DEVREG_VSYNC_PERIOD_L	*/
    {	0xF3	,	0x00	},		/*	DEVREG_VSYNC_PERIOD_H	*/
    {	0xF4	,	0x00	},		/*	DEVREG_VSYNC_PERIOD_TARGET_L	*/
    {	0xF5	,	0x00	},		/*	DEVREG_VSYNC_PERIOD_TARGET_H	*/
    {	0xF6	,	0x00	},		/*	DEVREG_VSYNC_CONTROL	*/
    {	0xF7	,	0x00	},		/*	DEVREG_VSYNC_CFG	*/
    {	0xF8	,	0x02	},		/*	DEVREG_VSYNC_GPIO_INT	*/
    {	0xF9	,	0x8F	},		/*	DEVREG_MOD_FIFO_DATA_CFG0	*/
    {	0xFA	,	0x8F	},		/*	DEVREG_MOD_FIFO_DATA_CFG1	*/
    {	0xFB	,	0x8F	},		/*	DEVREG_MOD_FIFO_DATA_CFG2	*/
    {	0xFC	,	0x7F	},		/*	DEVREG_FIFO_THR	*/
    {	0xFD	,	0x00	},		/*	DEVREG_FIFO_LEVEL	*/
    {	0xFE	,	0x00	},		/*	DEVREG_FIFO_STATUS0	*/
    {	0xFF	,	0x00	},		/*	DEVREG_FIFO_DATA	*/
};

/* Gain x2 */
uint32_t alsGain_conversion[] = {
    1 * 1000,
    2 * 1000,
    4 * 1000,
    8 * 1000,
    16 * 1000,
    32 * 1000,
    64 * 1000,
    128 * 1000,
    256 * 1000,
    512 * 1000,
    1024 * 1000,
    2048 * 1000,
    4096 * 1000,
    8192 * 1000,
};

uint16_t tsl2510_gain_conversion[] = {
    1, /* == 0.5 */
    2,
    4,
    8,
    16,
    32,
    64,
    128,
    256,
    512,
    1024,
    2048,
    4096,
    8192,
};
uint16_t fdGain_conversion[] = {
    1,
    2,
    4,
    8,
    16,
    32,
    64,
    128,
    256,
    512,
    1024,
    2048,
    4096,
    8192,
};

#define MAX_FFT_LEN 1024
#define MODIFIER(x) (MAX_FFT_LEN/x)


// sec_hamming : 0~65535, Q16
static const int32_t hamming[128] = {
    5243, 5279, 5388, 5569, 5822, 6146, 6541, 7005, 7538, 8137, 8803, 9532, 10323, 11175, 12086, 13052,
    14073, 15144, 16265, 17431, 18641, 19891, 21178, 22500, 23853, 25233, 26638, 28064, 29508, 30966, 32435, 33910,
    35389, 36869, 38344, 39813, 41271, 42714, 44141, 45546, 46926, 48279, 49600, 50888, 52138, 53348, 54514, 55635,
    56706, 57727, 58693, 59603, 60455, 61247, 61976, 62642, 63241, 63774, 64238, 64633, 64957, 65210, 65391, 65500,
    65536, 65500, 65391, 65210, 64957, 64633, 64238, 63774, 63241, 62642, 61976, 61247, 60455, 59603, 58693, 57727,
    56706, 55635, 54514, 53348, 52138, 50888, 49600, 48279, 46926, 45546, 44141, 42714, 41271, 39813, 38344, 36869,
    35389, 33910, 32435, 30966, 29508, 28064, 26638, 25233, 23853, 22500, 21178, 19891, 18641, 17431, 16265, 15144,
    14073, 13052, 12086, 11175, 10323, 9532, 8803, 8137, 7538, 7005, 6541, 6146, 5822, 5569, 5388, 5279,
};

// sec_sin, sec_cos : -65536~65536
static const int32_t sin[512] = {
    0, 804, 1608, 2412, 3216, 4019, 4821, 5623, 6424, 7224, 8022, 8820, 9616, 10411, 11204, 11996, 12785, 13573, 14359, 15143, 15924, 16703, 17479, 18253, 19024, 19792, 20557, 21320, 22078, 22834, 23586, 24335, 25080, 25821, 26558, 27291, 28020, 28745, 29466, 30182, 30893, 31600, 32303, 33000, 33692, 34380, 35062, 35738, 36410, 37076, 37736, 38391, 39040, 39683, 40320, 40951, 41576, 42194, 42806, 43412, 44011, 44604, 45190, 45769, 46341, 46906, 47464, 48015, 48559, 49095, 49624, 50146, 50660, 51166, 51665, 52156, 52639, 53114, 53581, 54040, 54491, 54934, 55368, 55794, 56212, 56621, 57022, 57414, 57798, 58172, 58538, 58896, 59244, 59583, 59914, 60235, 60547, 60851, 61145, 61429, 61705, 61971, 62228, 62476, 62714, 62943, 63162, 63372, 63572, 63763, 63944, 64115, 64277, 64429, 64571, 64704, 64827, 64940, 65043, 65137, 65220, 65294, 65358, 65413, 65457, 65492, 65516, 65531, 65536, 65531, 65516, 65492, 65457, 65413, 65358, 65294, 65220, 65137, 65043, 64940, 64827, 64704, 64571, 64429, 64277, 64115, 63944, 63763, 63572, 63372, 63162, 62943, 62714, 62476, 62228, 61971, 61705, 61429, 61145, 60851, 60547, 60235, 59914, 59583, 59244, 58896, 58538, 58172, 57798, 57414, 57022, 56621, 56212, 55794, 55368, 54934, 54491, 54040, 53581, 53114, 52639, 52156, 51665, 51166, 50660, 50146, 49624, 49095, 48559, 48015, 47464, 46906, 46341, 45769, 45190, 44604, 44011, 43412, 42806, 42194, 41576, 40951, 40320, 39683, 39040, 38391, 37736, 37076, 36410, 35738, 35062, 34380, 33692, 33000, 32303, 31600, 30893, 30182, 29466, 28745, 28020, 27291, 26558, 25821, 25080, 24335, 23586, 22834, 22078, 21320, 20557, 19792, 19024, 18253, 17479, 16703, 15924, 15143, 14359, 13573, 12785, 11996, 11204, 10411, 9616, 8820, 8022, 7224, 6424, 5623, 4821, 4019, 3216, 2412, 1608, 804, 0, -804, -1608, -2412, -3216, -4019, -4821, -5623, -6424, -7224, -8022, -8820, -9616, -10411, -11204, -11996, -12785, -13573, -14359, -15143, -15924, -16703, -17479, -18253, -19024, -19792, -20557, -21320, -22078, -22834, -23586, -24335, -25080, -25821, -26558, -27291, -28020, -28745, -29466, -30182, -30893, -31600, -32303, -33000, -33692, -34380, -35062, -35738, -36410, -37076, -37736, -38391, -39040, -39683, -40320, -40951, -41576, -42194, -42806, -43412, -44011, -44604, -45190, -45769, -46341, -46906, -47464, -48015, -48559, -49095, -49624, -50146, -50660, -51166, -51665, -52156, -52639, -53114, -53581, -54040, -54491, -54934, -55368, -55794, -56212, -56621, -57022, -57414, -57798, -58172, -58538, -58896, -59244, -59583, -59914, -60235, -60547, -60851, -61145, -61429, -61705, -61971, -62228, -62476, -62714, -62943, -63162, -63372, -63572, -63763, -63944, -64115, -64277, -64429, -64571, -64704, -64827, -64940, -65043, -65137, -65220, -65294, -65358, -65413, -65457, -65492, -65516, -65531, -65536, -65531, -65516, -65492, -65457, -65413, -65358, -65294, -65220, -65137, -65043, -64940, -64827, -64704, -64571, -64429, -64277, -64115, -63944, -63763, -63572, -63372, -63162, -62943, -62714, -62476, -62228, -61971, -61705, -61429, -61145, -60851, -60547, -60235, -59914, -59583, -59244, -58896, -58538, -58172, -57798, -57414, -57022, -56621, -56212, -55794, -55368, -54934, -54491, -54040, -53581, -53114, -52639, -52156, -51665, -51166, -50660, -50146, -49624, -49095, -48559, -48015, -47464, -46906, -46341, -45769, -45190, -44604, -44011, -43412, -42806, -42194, -41576, -40951, -40320, -39683, -39040, -38391, -37736, -37076, -36410, -35738, -35062, -34380, -33692, -33000, -32303, -31600, -30893, -30182, -29466, -28745, -28020, -27291, -26558, -25821, -25080, -24335, -23586, -22834, -22078, -21320, -20557, -19792, -19024, -18253, -17479, -16703, -15924, -15143, -14359, -13573, -12785, -11996, -11204, -10411, -9616, -8820, -8022, -7224, -6424, -5623, -4821, -4019, -3216, -2412, -1608, -804
};

static const int32_t cos[512] = {
    65536, 65531, 65516, 65492, 65457, 65413, 65358, 65294, 65220, 65137, 65043, 64940, 64827, 64704, 64571, 64429, 64277, 64115, 63944, 63763, 63572, 63372, 63162, 62943, 62714, 62476, 62228, 61971, 61705, 61429, 61145, 60851, 60547, 60235, 59914, 59583, 59244, 58896, 58538, 58172, 57798, 57414, 57022, 56621, 56212, 55794, 55368, 54934, 54491, 54040, 53581, 53114, 52639, 52156, 51665, 51166, 50660, 50146, 49624, 49095, 48559, 48015, 47464, 46906, 46341, 45769, 45190, 44604, 44011, 43412, 42806, 42194, 41576, 40951, 40320, 39683, 39040, 38391, 37736, 37076, 36410, 35738, 35062, 34380, 33692, 33000, 32303, 31600, 30893, 30182, 29466, 28745, 28020, 27291, 26558, 25821, 25080, 24335, 23586, 22834, 22078, 21320, 20557, 19792, 19024, 18253, 17479, 16703, 15924, 15143, 14359, 13573, 12785, 11996, 11204, 10411, 9616, 8820, 8022, 7224, 6424, 5623, 4821, 4019, 3216, 2412, 1608, 804, 0, -804, -1608, -2412, -3216, -4019, -4821, -5623, -6424, -7224, -8022, -8820, -9616, -10411, -11204, -11996, -12785, -13573, -14359, -15143, -15924, -16703, -17479, -18253, -19024, -19792, -20557, -21320, -22078, -22834, -23586, -24335, -25080, -25821, -26558, -27291, -28020, -28745, -29466, -30182, -30893, -31600, -32303, -33000, -33692, -34380, -35062, -35738, -36410, -37076, -37736, -38391, -39040, -39683, -40320, -40951, -41576, -42194, -42806, -43412, -44011, -44604, -45190, -45769, -46341, -46906, -47464, -48015, -48559, -49095, -49624, -50146, -50660, -51166, -51665, -52156, -52639, -53114, -53581, -54040, -54491, -54934, -55368, -55794, -56212, -56621, -57022, -57414, -57798, -58172, -58538, -58896, -59244, -59583, -59914, -60235, -60547, -60851, -61145, -61429, -61705, -61971, -62228, -62476, -62714, -62943, -63162, -63372, -63572, -63763, -63944, -64115, -64277, -64429, -64571, -64704, -64827, -64940, -65043, -65137, -65220, -65294, -65358, -65413, -65457, -65492, -65516, -65531, -65536, -65531, -65516, -65492, -65457, -65413, -65358, -65294, -65220, -65137, -65043, -64940, -64827, -64704, -64571, -64429, -64277, -64115, -63944, -63763, -63572, -63372, -63162, -62943, -62714, -62476, -62228, -61971, -61705, -61429, -61145, -60851, -60547, -60235, -59914, -59583, -59244, -58896, -58538, -58172, -57798, -57414, -57022, -56621, -56212, -55794, -55368, -54934, -54491, -54040, -53581, -53114, -52639, -52156, -51665, -51166, -50660, -50146, -49624, -49095, -48559, -48015, -47464, -46906, -46341, -45769, -45190, -44604, -44011, -43412, -42806, -42194, -41576, -40951, -40320, -39683, -39040, -38391, -37736, -37076, -36410, -35738, -35062, -34380, -33692, -33000, -32303, -31600, -30893, -30182, -29466, -28745, -28020, -27291, -26558, -25821, -25080, -24335, -23586, -22834, -22078, -21320, -20557, -19792, -19024, -18253, -17479, -16703, -15924, -15143, -14359, -13573, -12785, -11996, -11204, -10411, -9616, -8820, -8022, -7224, -6424, -5623, -4821, -4019, -3216, -2412, -1608, -804, 0, 804, 1608, 2412, 3216, 4019, 4821, 5623, 6424, 7224, 8022, 8820, 9616, 10411, 11204, 11996, 12785, 13573, 14359, 15143, 15924, 16703, 17479, 18253, 19024, 19792, 20557, 21320, 22078, 22834, 23586, 24335, 25080, 25821, 26558, 27291, 28020, 28745, 29466, 30182, 30893, 31600, 32303, 33000, 33692, 34380, 35062, 35738, 36410, 37076, 37736, 38391, 39040, 39683, 40320, 40951, 41576, 42194, 42806, 43412, 44011, 44604, 45190, 45769, 46341, 46906, 47464, 48015, 48559, 49095, 49624, 50146, 50660, 51166, 51665, 52156, 52639, 53114, 53581, 54040, 54491, 54934, 55368, 55794, 56212, 56621, 57022, 57414, 57798, 58172, 58538, 58896, 59244, 59583, 59914, 60235, 60547, 60851, 61145, 61429, 61705, 61971, 62228, 62476, 62714, 62943, 63162, 63372, 63572, 63763, 63944, 64115, 64277, 64429, 64571, 64704, 64827, 64940, 65043, 65137, 65220, 65294, 65358, 65413, 65457, 65492, 65516, 65531
};

static uint64_t ams_isqrt(uint64_t x)
{
    register uint64_t result, tmp;

    result = 0;
    tmp = (1LL << 62);  // second-to-top bit set
    while (tmp > x) {
        tmp >>= 2;
    }
    while (tmp != 0) {
        if (x >= (result + tmp)) {
            x -= result + tmp;
            result += 2 * tmp;  // <-- faster than 2 * one
        }
        result >>= 1;
        tmp >>= 2;
    }
    return result;
}

void ams_get_magnitude(int64_t* data_r, int64_t* data_i, int32_t* buffer, int size)
// data : 41~50 bits
{   //data must be twice as long as size
    int i;

    for (i = 0; i < size; ++i) {
        //sqrt(real^2 + imaginary^2)
        uint64_t square = 0;
        int64_t t_r = data_r[i] >> 4;
        int64_t t_i = data_i[i] >> 4;
        square = ((t_r * t_r) >> 7) + ((t_i * t_i) >> 7);
        buffer[i] = (int32_t)ams_isqrt(square); //26 bit
    }
}

// n is a power of 2
int _log2n(int n)
{
    //int len = sizeof(int) * 8;
    int len = 32, i;
    for (i = 0; i < len; i++)
    {
        if ((n&1) == 1)
            return i;
        else
            n >>= 1;
    }
    return -1;
}

// Utility function for reversing the bits
// of given index x
unsigned int bitReverse(unsigned int x, int log2n)
{
    int n = 0, i;
    for (i = 0; i < log2n; i++)
    {
        n <<= 1;
        n |= (x & 1);
        x >>= 1;
    }
    return n;
}

void _fft(int32_t* a_r, int32_t* a_i, int64_t* A_r, int64_t* A_i, int n)
{
    int log2n = _log2n(n), s, k, j;
    unsigned int i;
    // bit reversal of the given array
    for (i = 0; i < n; ++i) {
        int rev = bitReverse(i, log2n);
        A_r[i] = ((int64_t)a_r[rev]) << 16; // 16+16=32   Q16
        A_i[i] = ((int64_t)a_i[rev]) << 16; // 16+16=32   Q16
    }
    for (s = 1; s <= log2n; s++) {
        int m = 1 << s;
        int m_2 = m >> 1;
        int64_t wm_r = (int64_t)cos[512 >> s];    //wm_r = cos(2 pi / m)    Q16
        int64_t wm_i = -(int64_t)sin[512 >> s];   //wm_i = -sin(2 pi / m)    Q16

        for (k = 0; k < n; k += m)
        {
            int64_t w_r = 1LL << 16;  // Q16 => 65536
            int64_t w_i = 0;          // Q16

            for (j = 0; j < m_2; j++)
            {
                int i1 = k + j;
                int i2 = i1 + m_2;
                int64_t t_r = w_r * A_r[i2] - w_i * A_i[i2];   // Q16*Q16,  16+32=48~55
                int64_t t_i = w_r * A_i[i2] + w_i * A_r[i2];   // Q16*Q16,  16+32=48~55
                int64_t u_r = A_r[i1];    // Q16,  32
                int64_t u_i = A_i[i1];    // Q16
                int64_t w2_r = w_r * wm_r - w_i * wm_i;   // Q16*Q16,  16+16=32
                int64_t w2_i = w_r * wm_i + w_i * wm_r;   // Q16*Q16
                t_r >>= 16;    // Q16,  32~49
                t_i >>= 16;    // Q16
                A_r[i1] = u_r + t_r;      // Q16,  48~55
                A_i[i1] = u_i + t_i;
                A_r[i2] = u_r - t_r;
                A_i[i2] = u_i - t_i;
                w_r = w2_r >> 16;    // Q16
                w_i = w2_i >> 16;    // Q16
            }
        }
    }
}

void FFT(int32_t* data, enum fft_size size)
// buf_r <= 16bits data
{
    int i;
    unsigned int hamming_interval = 1;
    int32_t buf_r[AMS_FFT_SIZE] = {0};
    int32_t buf_i[AMS_FFT_SIZE] = {0};
    int64_t out_r[AMS_FFT_SIZE] = {0};
    int64_t out_i[AMS_FFT_SIZE] = {0};

    if (size > MAX_FFT_LEN >> 2 || size > AMS_FFT_SIZE || (size & 0x7f) != 0 ) { //FFT size : multiply of 128
        //@TODO add return codes so we know it failed
        return;
    }
    if (size > 128)
	hamming_interval = size/128;

    for (i = 0; i < size; i++) {
        if (!tsl2510_data->saturation && data[i] >= 0x3FFF) {
            ALS_info("DEBUG_FLICKER saturation");
            tsl2510_data->saturation = true;
        }
        buf_r[i] = ((int64_t)data[i]*(int64_t)hamming[i/hamming_interval]) >> 10;        // 16+16-10=22   Q6
        ALS_info("DEBUG_FLICKER data[%d] => %d buf[%d] => %lld", i, data[i], i, buf_r[i]);
    }
    _fft(buf_r, buf_i, out_r, out_i, size);
    for (i = 0; i < AMS_FFT_SIZE; i++) {
        out_r[i] >>= 6;
        out_i[i] >>= 6;
    }
    ams_get_magnitude(out_r, out_i, data, size);
}

// AMS FFT END

static const struct of_device_id tsl2510_match_table[] = {
    {.compatible = "ams,tsl2510",},
    {},
};


static int ams_deviceGetFlickerData(ams_deviceCtx_t *ctx, void *exportData)
{
    ams_flicker_ctx_t *flickerCtx = (ams_flicker_ctx_t *)&ctx->flickerCtx;
#if 0
    if((flickerCtx->flicker_data_cnt != 0 ) && (flickerCtx->flicker_data_cnt >=AMS_FFT_SIZE))
    {
        memcpy((uint16_t*)&exportData[0], &flickerCtx->flicker_data[0], sizeof(uint16_t)*AMS_FFT_SIZE);
    }
    else
    {
        return 0;
    }
    return flickerCtx->flicker_data_cnt;
#else
    memcpy((uint16_t*)&exportData[0], &flickerCtx->flicker_data[0], sizeof(uint16_t)*AMS_FFT_SIZE);
    return AMS_FFT_SIZE;
#endif
}

#if 1 // def CONFIG_AMS_OPTICAL_SENSOR_FIFO
static long tsl2510_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    void __user *argp = (void __user *)arg;
    int ret = 0;
    //int i = 0;
    int data_length = 0;
    struct tsl2510_device_data *data = container_of(file->private_data,
            struct tsl2510_device_data, miscdev);

    ALS_dbg("%s - ioctl start, %d\n", __func__, cmd);
    mutex_lock(&data->flickerdatalock);

    switch (cmd) {
        case TSL2510_IOCTL_READ_FLICKER:

            data_length = ams_deviceGetFlickerData(data->deviceCtx, (void*)&data->flicker_data[0]);
            ALS_dbg("%s - TSL2510_IOCTL_READ_FLICKER = %d\n", __func__, data->flicker_data[0]);

            //for(i=0 ; i < data_lengh; i++)
            //    ALS_dbg("%s flicker_data[%d] = %d\n", __func__, i,data->flicker_data[i]);
            if(data_length != 0)
            {
                ret = copy_to_user(argp,
                        data->flicker_data,
                        sizeof(int16_t)*data_length);
                if (unlikely(ret))
                    goto ioctl_error;

            }

            break;

        default:
            ALS_err("%s - invalid cmd\n", __func__);
            break;
    }

    mutex_unlock(&data->flickerdatalock);
    return ret;

ioctl_error:
    mutex_unlock(&data->flickerdatalock);
    ALS_err("%s - read flicker data err(%d)\n", __func__, ret);
    return -ret;
}

static const struct file_operations tsl2510_fops = {
    .owner = THIS_MODULE,
    .open = nonseekable_open,
    .unlocked_ioctl = tsl2510_ioctl,
};
#endif

static uint8_t alsGainToReg(uint32_t x)
{
    int i;

    for (i = sizeof(alsGain_conversion) / sizeof(uint32_t) - 1; i != 0; i--) {
        if (x >= alsGain_conversion[i])
            break;
    }
    return (i << 0);
}



static uint16_t alsSampleTimeUsToReg(uint32_t x)
{
    uint16_t regValue;

    regValue = (x * 1000) / AMS_USEC_PER_TICK;

    return regValue;
}

static uint16_t alsTimeUsToReg(uint32_t x, AMS_PORT_portHndl *portHndl)
{
    uint16_t regValue;
    uint16_t sample_time;

    if (portHndl)
    {
        ams_getWord(portHndl, DEVREG_SAMPLE_TIME0, &regValue);
        ALS_dbg("===== %s regValue =%x =====\n", __func__, regValue);
        regValue = regValue & MASK_ALS_SAMPLE_TIME;

        sample_time = ((regValue + 1) * AMS_USEC_PER_TICK) / 1000;

        ALS_dbg("===== %s Sameple time =%d =====\n", __func__, sample_time);
    }
    else
    {
        sample_time = 250; // default 250usec
    }

    regValue = (x / sample_time) - 1;

    return regValue;
}

static void tsl2510_debug_var(struct tsl2510_device_data *data)
{
    ALS_dbg("===== %s =====\n", __func__);
    ALS_dbg("%s client %p slave_addr 0x%x\n", __func__,
            data->client, data->client->addr);
    ALS_dbg("%s dev %p\n", __func__, data->dev);
    ALS_dbg("%s als_input_dev %p\n", __func__, data->als_input_dev);
    ALS_dbg("%s als_pinctrl %p\n", __func__, data->als_pinctrl);
    ALS_dbg("%s pins_sleep %p\n", __func__, data->pins_sleep);
    ALS_dbg("%s pins_idle %p\n", __func__, data->pins_idle);
    ALS_dbg("%s i2c_1p8 control %d\n", __func__, data->i2c_1p8);
    ALS_dbg("%s als_enabled %d\n", __func__, data->enabled);
    ALS_dbg("%s als_sampling_rate %d\n", __func__, data->sampling_period_ns);
    ALS_dbg("%s regulator_state %d\n", __func__, data->regulator_state);
    ALS_dbg("%s als_int %d\n", __func__, data->pin_als_int);
    ALS_dbg("%s als_en %d\n", __func__, data->pin_als_en);
    ALS_dbg("%s als_irq %d\n", __func__, data->dev_irq);
    ALS_dbg("%s irq_state %d\n", __func__, data->irq_state);
    ALS_dbg("===== %s =====\n", __func__);
}



static int tsl2510_write_reg_bulk(struct tsl2510_device_data *device, u8 reg_addr, u8* data, u8 length)
{
    int err = -1;
    int tries = 0;
    int num = 1;

    u8* buffer = devm_kzalloc(&device->client->dev, (length+1), GFP_KERNEL); // address + data

    struct i2c_msg msgs[] = {
        {
            .addr = device->client->addr,
            .flags = device->client->flags & I2C_M_TEN,
            .len = (length+1),
            .buf = buffer,
        },
    };

    buffer[0] = reg_addr;
    memcpy(&buffer[1] , &data[0], length);

#ifndef AMS_BUILD
    if (!device->pm_state || device->regulator_state == 0) {
        ALS_err("%s - write error, pm suspend or reg_state %d\n",
                __func__, device->regulator_state);
        err = -EFAULT;
        return err;
    }
#endif
    mutex_lock(&device->suspendlock);

    do {
        err = i2c_transfer(device->client->adapter, msgs, num);
        if (err != num)
            msleep_interruptible(AMSDRIVER_I2C_RETRY_DELAY);
        if (err < 0)
            ALS_err("%s - i2c_transfer error = %d\n", __func__, err);
    } while ((err != num) && (++tries < AMSDRIVER_I2C_MAX_RETRIES));

    mutex_unlock(&device->suspendlock);

    if (err != num) {
        ALS_err("%s -write transfer error:%d\n", __func__, err);
        err = -EIO;
        device->i2c_err_cnt++;
        return err;
    }
    //devm_kfree(&device->client->dev, buffer);
    //buffer = 0;

    return 0;
}

static int tsl2510_write_reg(struct tsl2510_device_data *device,
        u8 reg_addr, u8 data)
{
    int err = -1;
    int tries = 0;
    int num = 1;
    u8 buffer[2] = { reg_addr, data };
    struct i2c_msg msgs[] = {
        {
            .addr = device->client->addr,
            .flags = device->client->flags & I2C_M_TEN,
            .len = 2,
            .buf = buffer,
        },
    };
#ifndef AMS_BUILD
    if (!device->pm_state || device->regulator_state == 0) {
        ALS_err("%s - write error, pm suspend or reg_state %d\n",
                __func__, device->regulator_state);
        err = -EFAULT;
        return err;
    }
#endif
    mutex_lock(&device->suspendlock);

    do {
        err = i2c_transfer(device->client->adapter, msgs, num);
        if (err != num)
            msleep_interruptible(AMSDRIVER_I2C_RETRY_DELAY);
        if (err < 0)
            ALS_err("%s - i2c_transfer error = %d\n", __func__, err);
    } while ((err != num) && (++tries < AMSDRIVER_I2C_MAX_RETRIES));

    mutex_unlock(&device->suspendlock);

    if (err != num) {
        ALS_err("%s -write transfer error:%d\n", __func__, err);
        err = -EIO;
        device->i2c_err_cnt++;
        return err;
    }

    return 0;
}

static int tsl2510_read_reg(struct tsl2510_device_data *device,
        u8 reg_addr, u8 *buffer, int length)
{
    int err = -1;
    int tries = 0; /* # of attempts to read the device */
    int num = 2;
    struct i2c_msg msgs[] = {
        {
            .addr = device->client->addr,
            .flags = device->client->flags & I2C_M_TEN,
            .len = 1,
            .buf = buffer,
        },
        {
            .addr = device->client->addr,
            .flags = (device->client->flags & I2C_M_TEN) | I2C_M_RD,
            .len = length,
            .buf = buffer,
        },
    };
#ifndef AMS_BUILD
    if (!device->pm_state || device->regulator_state == 0) {
        ALS_err("%s - read error, pm suspend or reg_state %d\n",
                __func__, device->regulator_state);
        err = -EFAULT;
        return err;
    }
#endif

    mutex_lock(&device->suspendlock);

    do {
        buffer[0] = reg_addr;
        err = i2c_transfer(device->client->adapter, msgs, num);
        if (err != num)
            msleep_interruptible(AMSDRIVER_I2C_RETRY_DELAY);
        if (err < 0)
            ALS_err("%s - i2c_transfer error = %d\n", __func__, err);
    } while ((err != num) && (++tries < AMSDRIVER_I2C_MAX_RETRIES));

    mutex_unlock(&device->suspendlock);

    if (err != num) {
        ALS_err("%s -read transfer error:%d\n", __func__, err);
        err = -EIO;
        device->i2c_err_cnt++;
    }
    else
        err = 0;

    return err;
}

static int ams_getByte(AMS_PORT_portHndl *portHndl, ams_deviceRegister_t reg, uint8_t *readData)
{
    struct tsl2510_device_data *data = i2c_get_clientdata(portHndl);
    int err = 0;
    uint8_t length = 1;

    /* Sanity check input param */
    if (reg >= DEVREG_REG_MAX)
        return 0;

    err = tsl2510_read_reg(data, deviceRegisterDefinition[reg].address, readData, length);

    return err;
}

static int ams_setByte(AMS_PORT_portHndl *portHndl, ams_deviceRegister_t reg, uint8_t setData)
{
    struct tsl2510_device_data *data = i2c_get_clientdata(portHndl);
    int err = 0;

    /* Sanity check input param */
    if (reg >= DEVREG_REG_MAX)
        return 0;

    err = tsl2510_write_reg(data, deviceRegisterDefinition[reg].address, setData);

    return err;
}

static int ams_getBuf(AMS_PORT_portHndl *portHndl, ams_deviceRegister_t reg, uint8_t *readData, uint8_t length)
{
    struct tsl2510_device_data *data = i2c_get_clientdata(portHndl);
    int err = 0;

    /* Sanity check input param */
    if (reg >= DEVREG_REG_MAX)
        return 0;

    err = tsl2510_read_reg(data, deviceRegisterDefinition[reg].address, readData, length);

    return err;
}

int ams_setBuf(AMS_PORT_portHndl *portHndl, ams_deviceRegister_t reg, uint8_t *setData, uint8_t length)
{
    struct tsl2510_device_data *data = i2c_get_clientdata(portHndl);
    int err = 0;

    /* Sanity check input param */
    if (reg >= DEVREG_REG_MAX)
        return 0;

    err = tsl2510_write_reg_bulk(data, deviceRegisterDefinition[reg].address, setData, length);
    return err;
}

static int ams_getWord(AMS_PORT_portHndl *portHndl, ams_deviceRegister_t reg, uint16_t *readData)
{
    struct tsl2510_device_data *data = i2c_get_clientdata(portHndl);
    int err = 0;
    uint8_t length = sizeof(uint16_t);
    uint8_t buffer[sizeof(uint16_t)];

    /* Sanity check input param */
    if (reg >= DEVREG_REG_MAX)
        return 0;

    err = tsl2510_read_reg(data, deviceRegisterDefinition[reg].address, buffer, length);

    *readData = ((buffer[0] << AMS_ENDIAN_1) + (buffer[1] << AMS_ENDIAN_2));

    return err;
}

static int ams_setWord(AMS_PORT_portHndl *portHndl, ams_deviceRegister_t reg, uint16_t setData)
{
    struct tsl2510_device_data *data = i2c_get_clientdata(portHndl);
    int err = 0;
    uint8_t buffer[sizeof(uint16_t)];

    /* Sanity check input param */
    if (reg >= (DEVREG_REG_MAX - 1))
        return 0;

    buffer[0] = ((setData >> AMS_ENDIAN_1) & 0xff);
    buffer[1] = ((setData >> AMS_ENDIAN_2) & 0xff);

    err = tsl2510_write_reg(data, deviceRegisterDefinition[reg].address, buffer[0]);
    err = tsl2510_write_reg(data, deviceRegisterDefinition[reg + 1].address, buffer[1]);

    return err;
}

int ams_getField(AMS_PORT_portHndl *portHndl, ams_deviceRegister_t reg, uint8_t *setData, ams_regMask_t mask)
{
    struct tsl2510_device_data *data = i2c_get_clientdata(portHndl);
    int err = 0;
    uint8_t length = 1;

    /* Sanity check input param */
    if (reg >= DEVREG_REG_MAX)
        return 0;

    err = tsl2510_read_reg(data, deviceRegisterDefinition[reg].address, setData, length);

    *setData &= mask;

    return err;
}

static int ams_setField(AMS_PORT_portHndl *portHndl, ams_deviceRegister_t reg, uint8_t setData, ams_regMask_t mask)
{
    struct tsl2510_device_data *data = i2c_get_clientdata(portHndl);
    int err = 1;
    uint8_t length = 1;
    uint8_t original_data;
    uint8_t new_data;

    /* Sanity check input param */
    if (reg >= DEVREG_REG_MAX)
        return 0;

    err = tsl2510_read_reg(data, deviceRegisterDefinition[reg].address, &original_data, length);
    if (err < 0)
        return err;

    new_data = original_data & ~mask;
    new_data |= (setData & mask);

    if (new_data != original_data)
        err = tsl2510_write_reg(data, deviceRegisterDefinition[reg].address, new_data);

    return err;
}

static void als_getDefaultCalibrationData(ams_ccb_als_calibration_t *data)
{
    if (data != NULL) {
        data->Time_base = AMS_ALS_TIMEBASE;
        data->thresholdLow = AMS_ALS_THRESHOLD_LOW;
        data->thresholdHigh = AMS_ALS_THRESHOLD_HIGH;
        data->calibrationFactor = 1000;
    }
}
#if 0
static void als_update_statics(amsAlsContext_t *ctx);
#endif
static int amsAlg_als_processData(amsAlsContext_t *ctx, amsAlsDataSet_t *inputData)
{
    int64_t lux =0;
    uint32_t CWRatio =0;
    uint32_t tempWb = 0, tempClear = 0;

    ALS_info("%s - raw: %d, %d\n", __func__, inputData->datasetArray->clearADC, inputData->datasetArray->widebandADC);

    if (inputData->status & ALS_STATUS_RDY) {
        ctx->results.rawClear = inputData->datasetArray->clearADC;
        ctx->results.rawWideband = inputData->datasetArray->widebandADC;
        ctx->results.irrClear = inputData->datasetArray->clearADC;
        ctx->results.irrWideband = inputData->datasetArray->widebandADC;

        if (!tsl2510_data->saturation) {
            if (ctx->results.irrWideband < ctx->results.irrClear) {
                ctx->results.IR = 0;
            } else {
                tempWb = (WIDEBAND_CONST * AMS_ALS_FACTOR) * ctx->results.irrWideband;
                tempClear = (CLEAR_CONST * AMS_ALS_FACTOR) * ctx->results.irrClear;

                if (tempWb < tempClear) {
                    ctx->results.IR = 0;
                } else {
                    ctx->results.IR = (tempWb - tempClear) / AMS_ALS_FACTOR;
                }
            }
        } else {
            ctx->results.IR = 0;
        }
    }

    CWRatio = (ctx->results.rawClear /ctx->results.rawWideband);
    ALS_info("%s - IRR Clear : %d, IRR WIDEBAND :%d, CWRatio :%d\n", __func__, ctx->results.irrClear, ctx->results.irrWideband,CWRatio);
    ALS_info("%s - RAW Clear : %d, RAW WIDEBAND :%d\n", __func__, ctx->results.rawClear, ctx->results.rawWideband);
    ALS_info("%s - calculated IR :%d\n", __func__, ctx->results.IR);

    if(CWRatio > 15)
        CWRatio = 15;

    if(ctx->results.rawWideband  == 0)
        CWRatio = 15;

    if(CWRatio == 0){ // Normal lux
        lux = ctx->results.rawClear * (((coef_a * ctx->results.rawClear) /ctx->results.rawWideband) + coef_b);
    } else { //CWRatio data have over 15
        lux = ctx->results.rawClear * ((coef_a * CWRatio) + coef_b);
    }

    lux = lux >> 10; //devide 1024
    ALS_info("%s - Lux :%llu\n", __func__, lux );

    return 0;
}

static bool ams_getMode(ams_deviceCtx_t *ctx, ams_mode_t *mode)
{
    *mode = ctx->mode;

    return false;
}

uint32_t ams_getResult(ams_deviceCtx_t *ctx)
{
    uint32_t returnValue = ctx->updateAvailable;

    ctx->updateAvailable = 0;

    return returnValue;
}

static int amsAlg_als_initAlg(amsAlsContext_t *ctx, amsAlsInitData_t *initData);
static int amsAlg_als_getAlgInfo(amsAlsAlgoInfo_t *info);
static int amsAlg_als_processData(amsAlsContext_t *ctx, amsAlsDataSet_t *inputData);
//static int ams_smux_set(ams_deviceCtx_t *ctx);

static int ccb_alsInit(void *dcbCtx, ams_ccb_als_init_t *initData)
{
    ams_deviceCtx_t *ctx = (ams_deviceCtx_t *)dcbCtx;
    ams_ccb_als_ctx_t *ccbCtx = &ctx->ccbAlsCtx;
    amsAlsInitData_t initAlsData;
    amsAlsAlgoInfo_t infoAls;
    int ret = 0;

    ALS_dbg("%s - ccb_alsInit\n", __func__);

    if (initData)
        memcpy(&ccbCtx->initData, initData, sizeof(ams_ccb_als_init_t));
    else
        ccbCtx->initData.calibrate = false;

    initAlsData.adaptive = false;
    initAlsData.irRejection = false;
    initAlsData.gain = ccbCtx->initData.configData.gain;
    initAlsData.time_us = ccbCtx->initData.configData.uSecTime;
    initAlsData.calibration.adcMaxCount = ccbCtx->initData.calibrationData.adcMaxCount;
    initAlsData.calibration.calibrationFactor = ccbCtx->initData.calibrationData.calibrationFactor;
    initAlsData.calibration.Time_base = ccbCtx->initData.calibrationData.Time_base;
    initAlsData.calibration.thresholdLow = ccbCtx->initData.calibrationData.thresholdLow;
    initAlsData.calibration.thresholdHigh = ccbCtx->initData.calibrationData.thresholdHigh;
    //initAlsData.calibration.calibrationFactor = ccbCtx->initData.calibrationData.calibrationFactor;
    amsAlg_als_getAlgInfo(&infoAls);

    amsAlg_als_initAlg(&ccbCtx->ctxAlgAls, &initAlsData);

    AMS_SET_ALS_TIME(ccbCtx->initData.configData.uSecTime, ret);
    if (ret < 0) {
        ALS_err("%s - failed to AMS_SET_ALS_TIME\n", __func__);
        return ret;
    }

    if( ctx->sensor_mode == 1){ //ambient mode : als only
        AMS_SET_ALS_PERS(0x00, ret);
    }else{ //camera mode : als + flicker
        AMS_SET_ALS_PERS(0x01, ret);
    }
    if (ret < 0) {
        ALS_err("%s - failed to AMS_SET_ALS_PERS\n", __func__);
        return ret;
    }
    ccbCtx->shadowAiltReg = 0x00ffffff;
    ccbCtx->shadowAihtReg = 0;

    AMS_SET_ALS_THRS_LOW(ccbCtx->shadowAiltReg, ret);
    if (ret < 0) {
        ALS_err("%s - failed to AMS_SET_ALS_THRS_LOW\n", __func__);
        return ret;
    }

    AMS_SET_ALS_THRS_HIGH(ccbCtx->shadowAihtReg, ret);
    if (ret < 0) {
        ALS_err("%s - failed to AMS_SET_ALS_THRS_HIGH\n", __func__);
        return ret;
    }

    AMS_SET_ALS_GAIN0(ctx->ccbAlsCtx.initData.configData.gain, ret);
    if (ret < 0) {
        ALS_err("%s - failed to AMS_SET_ALS_GAIN\n", __func__);
        return ret;
    }

    AMS_SET_ALS_GAIN1(ctx->ccbAlsCtx.initData.configData.gain, ret);
    if (ret < 0) {
        ALS_err("%s - failed to AMS_SET_ALS_GAIN\n", __func__);
        return ret;
    }

    if( ctx->sensor_mode == 0){ //camera mode
        AMS_SET_ALS_AINT_DIRECT(HIGH, ret);
        if (ret < 0) {
            ALS_err("%s - failed to AMS_SET_ALS_AINT_DIRECT\n", __func__);
            return ret;
        }
    }
    ccbCtx->state = AMS_CCB_ALS_RGB;

    return ret;
}

static int AMS_SET_FIFO_MAP(ams_deviceCtx_t *ctx);
static int AMS_SET_FIFO_MAP(ams_deviceCtx_t *ctx)
{
    int ret = 0;
    ret = ams_setField(ctx->portHndl, DEVREG_MOD_FIFO_DATA_CFG0, LOW, MASK_MOD_ALS_FIFO_DATA0_WRITE_ENABLE);

    if (ret < 0) {
        ALS_err("%s - failed to MASK_MOD_ALS_FIFO_DATA0_WRITE_ENABLE\n", __func__);
        return ret;
    }
#if 0
    ret = ams_setField(ctx->portHndl, DEVREG_MOD_FIFO_DATA_CFG0, HIGH, MASK_MOD_FD_FIFO_DATA0_COMPRESSION_ENABLE);

    if (ret < 0) {
        ALS_err("%s - failed to MASK_MOD_FD_FIFO_DATA0_COMPRESSION_ENABLE\n", __func__);
        return ret;
    }
#endif

    ret = ams_setField(ctx->portHndl, DEVREG_MOD_FIFO_DATA_CFG1, LOW, MASK_MOD_ALS_FIFO_DATA1_WRITE_ENABLE);

    if (ret < 0) {
        ALS_err("%s - failed to MASK_MOD_ALS_FIFO_DATA1_WRITE_ENABLE\n", __func__);
        return ret;
    }

#if 0
    ret = ams_setField(ctx->portHndl, DEVREG_MOD_FIFO_DATA_CFG1, HIGH, MASK_MOD_FD_FIFO_DATA1_COMPRESSION_ENABLE);

    if (ret < 0) {
        ALS_err("%s - failed to MASK_MOD_FD_FIFO_DATA1_COMPRESSION_ENABLE\n", __func__);
        return ret;
    }
#endif

    ret = ams_setField(ctx->portHndl, DEVREG_MOD_FIFO_DATA_CFG2, LOW, MASK_MOD_ALS_FIFO_DATA2_WRITE_ENABLE);

    if (ret < 0) {
        ALS_err("%s - failed to MASK_MOD_ALS_FIFO_DATA2_WRITE_ENABLE\n", __func__);
        return ret;
    }


    return ret;
}

static void tsl2510_sequencer_init(ams_deviceCtx_t *ctx)
{
    int ret = 0;
    ALS_dbg("%s \n",__func__) ;
    /* Assign ALS to sequencer step 0 */
    ret = ams_setField(ctx->portHndl,
                   DEVREG_MEAS_SEQR_ALS_FD_1,
                   0x01,
                   TSL2510_MASK_MEASUREMENT_SEQUENCER_ALS_PATTERN );
    if (ret < 0) {
	ALS_err("%s - failed to set SEQR_ALS_FD_1",__func__);
    }

    /* Assign Flicker to sequencer step 1 */
    /* Assign modulator 0 to flicker */
    ret = ams_setField(ctx->portHndl,
                   DEVREG_MEAS_SEQR_FD_0,
                   0x01,
                   TSL2510_MASK_MEASUREMENT_SEQUENCER_MOD0_FD_PATTERN );
    if (ret < 0) {
	ALS_err("%s - failed to set SEQR_FD_0",__func__);
    }

    /* Assign modulator 1 to flicker */
    ret = ams_setField(ctx->portHndl,
                   DEVREG_MEAS_SEQR_FD_0,
                   0x10,
                   TSL2510_MASK_MEASUREMENT_SEQUENCER_MOD1_FD_PATTERN );
    if (ret < 0) {
	ALS_err("%s - failed to set SEQR_FD_0",__func__);
    }
}

static void tsl2510_fifo_format_init(ams_deviceCtx_t *ctx)
{
    int ret = 0;
    /* Enable the end marker */
    ret = ams_setField(ctx->portHndl,
                   DEVREG_MEAS_MODE1,
                   HIGH,
                   TSL2510_MASK_MOD_FIFO_FD_END_MARKER_WRITE_ENABLE);
    if (ret < 0) {
	ALS_err("%s - set FIFO_FD_END_MARKER_WRITE_ENABLE err");
    }

    ctx->has_fifo_fd_end_marker = true;

    /* Enable the gain */
    ret = ams_setField(ctx->portHndl,
                   DEVREG_MEAS_MODE1,
                   HIGH,
                   TSL2510_MASK_MOD_FIFO_FD_GAIN_WRITE_ENABLE);

    if (ret < 0) {
	ALS_err("%s - set FIFO_FD_GAIN_WRITE_ENABLE err");
    }

    ctx->has_fifo_fd_gain = true;
    ALS_dbg("%s \n",__func__) ;
}

int ccb_flickerInit(void *dcbCtx/*, ams_ccb_als_init_t *initData*/)
{
    ams_deviceCtx_t *ctx = (ams_deviceCtx_t *)dcbCtx;
    ams_flicker_ctx_t *ccbCtx = (ams_flicker_ctx_t *)&ctx->flickerCtx;
    int ret = 0;


    //AMS_SET_FLICKER_NUM_SAMPLES(AMS_FLICKER_NUM_SAMPLES, ret);
    AMS_SET_FLICKER_NUM_SAMPLES(AMS_CLR_WIDE_FLICKER_NUM_SAMPLES, ret);
    ctx->flicker_num_samples = AMS_FLICKER_NUM_SAMPLES;

    if (ret < 0) {
        ALS_err("%s - failed to AMS_SET_FLICKER_NUM_SAMPLES\n", __func__);
        return ret;
    }

    AMS_SET_FIFO_THR(AMS_FLICKER_THR_LVL, ret);
    if (ret < 0) {
        ALS_err("%s - failed to AMS_SET_FIFO_THR\n", __func__);
        return ret;
    }

    ret = AMS_SET_FIFO_MAP(ctx);
    if (ret < 0) {
        ALS_err("%s - failed to AMS_SET_FIFO_MAP\n", __func__);
        return ret;
    }

    // Test
    //ret = ams_setByte(ctx->portHndl, DEVREG_MEAS_SEQR_FD_0, 0x10);
    //ret = ams_setByte(ctx->portHndl, DEVREG_MEAS_SEQR_ALS_FD_1, 0x00);

#ifdef CONFIG_AMS_ADD_MARKER_AND_GAIN_FIFO
    AMS_FD_GAIN_TO_FIFO(ret);
    if (ret < 0) {
        ALS_err("%s - failed to AMS_FD_GAIN_TO_FIFO\n", __func__);
        return ret;
    }

    AMS_FD_END_MARKER_TO_FIFO(ret);
    if (ret < 0) {
        ALS_err("%s - failed to AMS_FD_END_MARKER_TO_FIFO\n", __func__);
        return ret;
    }
#endif
    tsl2510_sequencer_init(ctx);
    tsl2510_fifo_format_init(ctx);

    AMS_FIFO_CLEAR(ret);
    if (ret < 0) {
        ALS_err("%s - failed to FIFO CLEAR\n", __func__);
        return ret;
    }

   /* ams_setField(ctx->portHndl,
                   DEVREG_MEAS_MODE0,
                   HIGH,
                   0x20);*/

    ams_setField(ctx->portHndl,
                   DEVREG_SIEN,
                   HIGH,
                   0x02);

    ccbCtx->flicker_data_cnt = 0;
    ctx->hamming_status = true;
    memset(&ccbCtx->flicker_data[0], 0, (sizeof(uint16_t)*AMS_FFT_SIZE));
    ccbCtx->data_ready = 0;

    ccbCtx->gain = 0;
    return ret;
}

static void ccb_alsInfo(ams_ccb_als_info_t *infoData)
{
    if (infoData != NULL) {
        infoData->algName = "ALS";
        infoData->contextMemSize = sizeof(ams_ccb_als_ctx_t);
        infoData->scratchMemSize = 0;
        infoData->defaultCalibrationData.calibrationFactor = 1000;
        als_getDefaultCalibrationData(&infoData->defaultCalibrationData);
    }
}

static void ccb_alsSetConfig(void *dcbCtx, ams_ccb_als_config_t *configData)
{
    ams_ccb_als_ctx_t *ccbCtx = &((ams_deviceCtx_t *)dcbCtx)->ccbAlsCtx;

    ccbCtx->initData.configData.threshold = configData->threshold;
}

/*
static uint32_t get_sqrt(uint32_t x)
{
    uint32_t result;
    uint32_t tmp;

    result = 0;
    tmp = (1 << 30);
    while (tmp > x) {
        tmp >>= 2;
    }
    while (tmp != 0) {
        if (x >= (result + tmp)) {
            x -= result + tmp;
            result += 2 * tmp;
        }
        result >>= 1;
        tmp >>= 2;
    }
    return result;
}
static int get_stdev(uint16_t *buff, int mean, int size)
{
    int i;
    uint32_t sum = 0;

    for (i = 0; i < size; ++i) {
        sum += ((buff[i] - mean) * (buff[i] - mean));
    }
    sum = sum / (size - 1);
    return get_sqrt(sum);
}

static int get_mean(uint16_t *buff, int size)
{
    int i;
    int sum = 0;

    for (i = 0; i < size; ++i) {
        sum += buff[i];
    }
    return sum / size;
}
*/

#define MAX_FIFO_LEN (2048)
#define SIGMA 3

//static uint16_t flicker_data[AMS_FFT_SIZE];
static uint8_t fifodata[MAX_FIFO_LEN];

#define NUM_SAMPLE_BYTES (AMS_FFT_SIZE*2)

static uint32_t calc_average(uint32_t *buffer, int count)
{
    uint32_t sum = 0;
    int i;
    uint16_t average = 0;

    if ((NULL != buffer) && (count > 0))
    {
        for (i = 0; i < count; i++)
        {
            sum += buffer[i];
        }
        average = sum / count;
    }
    return average;
}

#define TSL2510_THD_CLEAR      1800LL    /* 1.8 * 1000 */
#define TSL2510_THD_RATIO      3LL       /* 0.003 * 1000 */
#define TSL2510_GAIN_MAX       4096LL    /* 4096 */
#define TSL2510_THD_RATIO_AUTO 1000LL       /*  */
#define MAX_NUM_FLICKER_SAMPLES 2048LL
#define MAX_NUM_FLICKER_SAMPLE_BYTES (MAX_NUM_FLICKER_SAMPLES * 2)

static uint64_t calc_thd(uint16_t clear_avg_fifo, uint32_t clear_avg, uint32_t *fft_out_data)
{
    uint64_t threshold;
    uint64_t ratio_fixed;
    uint64_t ratio_final;

    if (clear_avg_fifo <= 1) {
        ratio_fixed = (TSL2510_THD_CLEAR * TSL2510_GAIN_MAX);
    } else {
        if (clear_avg)
            ratio_fixed = (TSL2510_THD_CLEAR << 7)/ clear_avg; // clear_avg left-shifted 7.
        else
            ratio_fixed = (TSL2510_THD_CLEAR << 7); // clear_avg left-shifted 7.
    }

    if ((TSL2510_THD_RATIO_AUTO) > ratio_fixed) {
        ratio_final = TSL2510_THD_RATIO_AUTO;
    } else {
        ratio_final = ratio_fixed;
    }

    threshold = ((uint64_t)fft_out_data[0] * ratio_final * TSL2510_THD_RATIO)/1000000;

    ALS_info("calc_thd threshold = %lld, fifoout[5] %lld, ratio_final %lld \n", threshold, fft_out_data[5], ratio_final);
    return threshold;
}

static uint8_t fifo_out_data[MAX_NUM_FLICKER_SAMPLE_BYTES];

static void parse_fifo_end_data(ams_deviceCtx_t * ctx )
{
    uint8_t fifo_end_data[7];
    int fifo_end_size = 0;
    int fifo_end_len;
    int gain_index = 0;
    int checksum_index = 0;
    uint16_t tmp;
    uint8_t kfifolen = 0;

    /* Calculate the number of end bytes */
    if (true == ctx ->has_fifo_fd_end_marker)
    {
        fifo_end_size += 3;
    }
    if (true == ctx ->has_fifo_fd_checksum)
    {
        fifo_end_size += 2;
    }
    if (true == ctx ->has_fifo_fd_gain)
    {
        fifo_end_size += 2;
    }

    /* Get the end bytes */
    memset(fifo_end_data, 0x0, sizeof(fifo_end_data));
    if ((kfifolen = kfifo_len(&ams_fifo)) >= fifo_end_size)
    {
        fifo_end_len = kfifo_out(&ams_fifo, fifo_end_data, fifo_end_size);
    }

    /* Determine the index for the various end data */
    if ((true == ctx ->has_fifo_fd_end_marker) &&
        (true == ctx ->has_fifo_fd_checksum) &&
        (true == ctx ->has_fifo_fd_gain))
    {
        checksum_index = 3;
        gain_index = 5;
    }
    else if ((true == ctx ->has_fifo_fd_end_marker) &&
             (false == ctx ->has_fifo_fd_checksum) &&
             (true == ctx ->has_fifo_fd_gain))
    {
        checksum_index = 0;
        gain_index = 3;
    }
    else if ((true == ctx ->has_fifo_fd_end_marker) &&
             (true == ctx ->has_fifo_fd_checksum) &&
             (false == ctx ->has_fifo_fd_gain))
    {
        checksum_index = 3;
        gain_index = 0;
    }
    else if ((false == ctx ->has_fifo_fd_end_marker) &&
             (true == ctx ->has_fifo_fd_checksum) &&
             (true == ctx ->has_fifo_fd_gain))
    {
        checksum_index = 0;
        gain_index = 2;
    }
    else if ((false == ctx ->has_fifo_fd_end_marker) &&
             (false == ctx ->has_fifo_fd_checksum) &&
             (true == ctx ->has_fifo_fd_gain))
    {
        checksum_index = 0;
        gain_index = 0;
    }
    else if ((false == ctx ->has_fifo_fd_end_marker) &&
             (true == ctx ->has_fifo_fd_checksum) &&
             (false == ctx ->has_fifo_fd_gain))
    {
        checksum_index = 0;
        gain_index = 0;
    }


    /* Parse the data */
    /* Data is written to the fifo in little endian, byte swap is required */
    if (true == ctx->has_fifo_fd_checksum)
    {
        tmp = ((fifo_end_data[checksum_index + 1] << 8) | fifo_end_data[checksum_index]);
        ctx->fifo_checksum = tmp;
    }

    if ((true == ctx->has_fifo_fd_gain) && ((fifo_end_data[0] == 0) && (fifo_end_data[1] == 0) && (fifo_end_data[2] == 0)) && (kfifolen >= fifo_end_size))
    {
        tmp = ((fifo_end_data[gain_index + 1] << 8) | fifo_end_data[gain_index]);

        ALS_info("%s: tmp: %04x ,  fifo_end_data[%d] =0x%x , fifo_end_data[+1] = 0x%x , \n", __func__, tmp,gain_index,fifo_end_data[gain_index],fifo_end_data[gain_index+1]);
        /* After the byte swap the gain data is like so
         *
         *                       Bits
         *      15-12       11-8        7-4          3-0
         *  -------------------------------------------------
         * |  not used | mod2 gain | mod1 gain  | mod0 gain  |
         *  -------------------------------------------------
         *
         */
        ctx->fifo_mod0_gain = tmp & 0xFF;
        //ctx->fifo_mod1_gain = (tmp >> 4) & 0x0F;
    }

}

ssize_t read_fifo(ams_deviceCtx_t * ctx , uint16_t *buf, int size)
{
    int len;
    int kfifo_Len;
    int i =0,j =0 ;
    uint16_t tmp =0;

    memset(fifo_out_data, 0x0, sizeof(fifo_out_data));

    kfifo_Len =  kfifo_len(&ams_fifo);

    if(kfifo_Len >=size){ //512
        len = kfifo_out(&ams_fifo,fifo_out_data,size);
        /* byte swap flicker data*/
        for (i = 0, j = 0; i < size; i+=2, j++) {
            tmp = ((fifo_out_data[i + 1] << 8) | fifo_out_data[i]); //256
            buf[j] = tmp;
        }

        parse_fifo_end_data(ctx);
    } else {
        len = 0;
    }
    ALS_info("read_fifo read size  %d , kfifo_Len =%d\n",len,kfifo_Len);
    return len;
}

int get_fft(ams_deviceCtx_t * ctx , uint32_t *out)
{
    static uint16_t buffer[2048] ;
    static uint32_t clear_buffer[MAX_NUM_FLICKER_SAMPLES] = { 0 };
    static uint32_t wideband_buffer[MAX_NUM_FLICKER_SAMPLES] = { 0 };
    //int num_samples = ctx->flicker_num_samples;

    //  int num_sample_bytes = num_samples * 4;

    ssize_t size = 0;
    unsigned int i ,j =0 ;
    uint16_t clear_gain , wideband_gain = 0;
    uint8_t fifo_mod0_gain ,fifo_mod1_gain =0;
    int ret = 0;

    memset(clear_buffer, 0, MAX_NUM_FLICKER_SAMPLE_BYTES);
    memset(wideband_buffer, 0, MAX_NUM_FLICKER_SAMPLE_BYTES);
    memset(buffer,0x00,4096);


    size = read_fifo(ctx,buffer,AMS_FLICKER_NUM_SAMPLES); //512 byte

    //size = read_fifo(ctx,buffer,512);//512byte
    //size = read_fifo(ctx,buffer,num_sample_bytes);//256byte = 128byte(64sample) + 128byte

    //if(size < 4096){
    //return 0;
    //}
    kfifo_reset(&ams_fifo);

    /* Separate Clear and Wideband data */
    for (i = 0, j = 0; i < (int)(size/2); i+=2, j++) { //256 sample (clear & wide)
        /* Clear */
        clear_buffer[j] = buffer[i]; //seperate 128

        /* Wideband */
        out[j] = wideband_buffer[j] = buffer[i + 1]; //seperate 128
        //ALS_info("CLEAR[ %d]=%d, WIDE[%d]=%d \n",j,clear_buffer[j],j, wideband_buffer[j]);
    }

    ctx->clear_average_fifo = calc_average(clear_buffer, AMS_CLR_WIDE_FLICKER_NUM_SAMPLES);//128
    ctx->wideband_average_fifo = calc_average(wideband_buffer, AMS_CLR_WIDE_FLICKER_NUM_SAMPLES);//128
    ALS_info("clear_average_fifo %d , wideband_average_fifo %d ,mod0_gain 0x%x , mod1_gain 0x%x  \n",ctx->clear_average_fifo, ctx->wideband_average_fifo, ctx->fifo_mod0_gain, ctx->fifo_mod1_gain);

    fifo_mod0_gain = ctx->fifo_mod0_gain & 0x0F;//clear
    fifo_mod1_gain = (ctx->fifo_mod0_gain >> 4) & 0x0F;//wide

    clear_gain = tsl2510_gain_conversion[fifo_mod0_gain];
    wideband_gain = tsl2510_gain_conversion[fifo_mod1_gain];

    ctx->clear_average = ((uint32_t)ctx->clear_average_fifo) << 7;
    ctx->wideband_average = ((uint32_t)ctx->wideband_average_fifo) << 7;
    ctx->clear_average /= clear_gain;
    ctx->wideband_average /= wideband_gain;

    ALS_info( "cavg_fifo: %d, wavg_fifo: %d,  gain0 =%d , gain1 = %d cavg: %d, wavg: %d\n",
            ctx->clear_average_fifo, ctx->wideband_average_fifo,
            fifo_mod0_gain,fifo_mod1_gain,
            ctx->clear_average, ctx->wideband_average);

    /* dev_info(&chip->client->dev, "calling ams_rfft with %d samples\n", num_samples); */

    FFT(out, AMS_FFT_SIZE);
    AMS_FIFO_CLEAR(ret);

    if (ret < 0) {
        ALS_err("%s - failed to FIFO CLEAR\n", __func__);
        return ret;
    }
    //ams_rfft(buffer, AMS_FFT_SIZE);
    //ams_get_magnitude((int16_t*)buffer,(uint16_t*)out,AMS_FFT_SIZE/2);
    return 1;
}

bool  ccb_sw_bin4096_flicker_GetResult(void * dcbCtx);
bool  ccb_sw_bin4096_flicker_GetResult(void * dcbCtx)
{
    ams_deviceCtx_t * ctx = (ams_deviceCtx_t*)dcbCtx;
    static uint32_t buf[MAX_NUM_FLICKER_SAMPLES] = {0,};
    int max = -1;
    ams_flicker_ctx_t *ccbCtx = (ams_flicker_ctx_t *)&ctx->flickerCtx;
    //amsAlsDataSet_t inputData;
    //adcDataSet_t dataSet;
    //  int mean;
    //  int stdev;

    int i =0;
    uint16_t mHz = 9999;
    int sampling_freq;
    uint64_t thd =0;
    //int  num_samples =ctx->flicker_num_samples;

    memset(buf,0,sizeof(buf));

    if(get_fft(ctx,buf)){
        //buf[0] = 0;
        for(i = 5 ; i <= 50; ++i){			// 50~500Hz
            //ALS_info("ccb_sw_bin4096_flicker_GetResult: buf[%d]=%d \n",i,buf[i]);
            if(max < 0 || buf[i] > buf[max]){
                max = i;
            }
        }

        /* New code */
        sampling_freq = 1000000 / AMS_SAMPLING_TIME;

        thd = calc_thd(ctx->clear_average_fifo, ctx->clear_average, buf);
        ALS_info("DEBUG_FLICKER DC:%lld", buf[0]);
        ALS_info("DEBUG_FLICKER buf[%d]=%d  , thd %lld \n", max,buf[max],thd);

        if (buf[max] > thd) {
            mHz = ((max * sampling_freq) / AMS_FFT_SIZE);
            ALS_info( "flicker_freq %d\n", mHz);

        } else {
            mHz = 0;
            ALS_info( "flicker_freq is zero");
        }
        ccbCtx->frequency = mHz;
    }
    ctx->flickerCtx.data_ready = 0;

    return true;
}


static bool ccb_FIFOEvent(void *dcbCtx)
{
    int len = 0;
    uint8_t fifo_lvl = 0;
    uint8_t fifo_ov = 0;
    uint8_t fifo_uf = 0;
    int size =0;
    uint16_t fifo_size=0;
//    uint16_t data = 0;
    int  num_samples = 0;
     int num_sample_bytes= 0;

#ifdef CONFIG_AMS_ADD_MARKER_AND_GAIN_FIFO
    uint8_t fd_gain = 0;
#endif
    int ret = 0 ;

    ams_deviceCtx_t *ctx = (ams_deviceCtx_t *)dcbCtx;
    ams_flicker_ctx_t *ccbCtx = (ams_flicker_ctx_t *)&ctx->flickerCtx;

    ccbCtx->statusReg = ctx->shadowFIFOStatusReg;
    num_samples =ctx->flicker_num_samples;
    num_sample_bytes = num_samples*4;

    ams_getByte(ctx->portHndl, DEVREG_FIFO_LEVEL, &fifo_lvl); //current fifo count

    ccbCtx->fifolvl = (uint16_t)((fifo_lvl << 2) | (ccbCtx->statusReg & 0x03));

    fifo_ov = (ccbCtx->statusReg & MASK_FIFO_OVERFLOW) >> 7;
    fifo_uf = (ccbCtx->statusReg & MASK_FIFO_UNDERFLOW) >> 6;

    ccbCtx->overflow += fifo_ov;
    if (fifo_ov > 0) {
        AMS_FIFO_CLEAR(ret);
        if (ret < 0) {
            ALS_err("%s - failed to FIFO CLEAR\n", __func__);
            return ret;
        }
        ccbCtx->fifolvl = 0;

        kfifo_reset(&ams_fifo);
        ALS_err("%s - fifo over flow [0x%x]\n", __func__, fifo_ov);
        return false;
    }
    if (fifo_uf > 0) {
        ALS_err("%s - fifo under flow [0x%x]\n", __func__, fifo_uf);
    }

    fifo_size = ccbCtx->fifolvl;

    //quotient = fifo_size / 32;
    //remainder = fifo_size % 32;

    ALS_info("%s - FIFO LVL or FIFO size = %d\n ", __func__, ccbCtx->fifolvl);
    while (fifo_size > 0) {
        if (fifo_size >= I2C_SMBUS_BLOCK_MAX) {
            size = I2C_SMBUS_BLOCK_MAX;
        }
        else {
            size = fifo_size;
        }
        memset(&fifodata, 0x0, sizeof(fifodata));
        ams_getBuf(ctx->portHndl, DEVREG_FIFO_DATA, (uint8_t *)&fifodata, size);

        fifo_size -= size;
        kfifo_in(&ams_fifo, fifodata, size);
        if (kfifo_is_full(&ams_fifo)) {
            kfifo_reset(&ams_fifo);
            ALS_err("%s - ams_fifo is full\n", __func__);
            break;
        }
    }


     if((len = kfifo_len(&ams_fifo)) >= (num_samples)) //512 byte , 256 level
     //if((len = kfifo_len(&ams_fifo)) >= (num_sample_bytes)) // 512 SAMPLE * clear & wide
     {
         ctx->flickerCtx.data_ready = 1;
         //dev_info(&chip->client->dev, "Sample Rate: %ld Hz\n", sample_rate);
         ALS_info("%s FIFO now is full!!! ready  to calc freq   fifo size %d ",__func__,len);

     }
     //AMS_FIFO_CLEAR(ret);


    return false;
}


static int ccb_alsHandle(void *dcbCtx, ams_ccb_als_dataSet_t *alsData)
{
    ams_deviceCtx_t *ctx = (ams_deviceCtx_t *)dcbCtx;
    ams_ccb_als_ctx_t *ccbCtx = &((ams_deviceCtx_t *)dcbCtx)->ccbAlsCtx;
    amsAlsDataSet_t inputDataAls;
    static adcDataSet_t adcData; /* QC - is this really needed? */
    uint8_t ADCs[4]; //Clear + WIDEBAND
    int ret = 0;

    /* get gain from HW register if so configured */
    if (ctx->ccbAlsCtx.initData.autoGain) {
        uint32_t scaledGain;
        uint32_t scaledGain1;

        uint8_t gain;

        AMS_GET_ALS_GAIN(scaledGain, scaledGain1, gain, ret);
        if (ret < 0) {
            ALS_err("%s - failed to AMS_GET_ALS_GAIN\n", __func__);
            return ret;
        }
        ctx->ccbAlsCtx.ctxAlgAls.ClearGain = scaledGain;
        ctx->ccbAlsCtx.ctxAlgAls.WBGain = scaledGain1;
    }

    switch (ccbCtx->state) {
        case AMS_CCB_ALS_RGB: /* state to measure RGB */
#ifdef HAVE_OPTION__ALWAYS_READ
            if ((alsData->status2Reg & (ALS_DATA_VALID)) || ctx->alwaysReadAls)
#else
            if (alsData->status2Reg & (ALS_DATA_VALID))
#endif
            {
                AMS_ALS_GET_ALS_DATA(&ADCs[0], ret);
                if (ret < 0) {
                    ALS_err("%s - failed to AMS_ALS_GET_CRGB_W\n", __func__);
                    return ret;
                }
                inputDataAls.status = ALS_STATUS_RDY;

                if ((alsData->alsstatusReg & 0x04) == 0)
                    adcData.AdcClear = ((ADCs[1] << 8) | (ADCs[0] << 0)) << 4;
                else
                    adcData.AdcClear = ((ADCs[1] << 8) | (ADCs[0] << 0));

                if ((alsData->alsstatusReg & 0x02) == 0)
                    adcData.AdcWb = ((ADCs[3] << 8) | (ADCs[2] << 0)) << 4;
                else
                    adcData.AdcWb = ((ADCs[3] << 8) | (ADCs[2] << 0));

                if ((alsData->alsstatus2Reg & 0x0F) == 0)
                    adcData.AdcClear = (((adcData.AdcClear) << 1)); // 0.5x
                else
                    adcData.AdcClear = (((adcData.AdcClear) >> ((alsData->alsstatus2Reg & 0x0F) - 1)));

                if (((alsData->alsstatus2Reg & 0xF0) >> 4) == 0)
                    adcData.AdcWb = (((adcData.AdcWb) << 1)); // 0.5x
                else
                    adcData.AdcWb = (((adcData.AdcWb) >> (((alsData->alsstatus2Reg & 0xF0) >> 4) - 1)));

                inputDataAls.datasetArray = (alsData_t *)&adcData;
                AMS_PORT_LOG_CRGB_W(adcData);
                ALS_info("Clear AGAIN = %d, WIDE AGAIN =%d\n", ctx->ccbAlsCtx.ctxAlgAls.ClearGain, ctx->ccbAlsCtx.ctxAlgAls.WBGain);

                amsAlg_als_processData(&ctx->ccbAlsCtx.ctxAlgAls, &inputDataAls);

                if (ctx->mode & MODE_ALS_LUX)
                    ctx->updateAvailable |= (1 << AMS_AMBIENT_SENSOR);
                ccbCtx->state = AMS_CCB_ALS_RGB;
            }
            break;

        default:
            ccbCtx->state = AMS_CCB_ALS_RGB;
            break;
    }
    return false;
}

static void ccb_alsGetResult(void *dcbCtx, ams_ccb_als_result_t *exportData)
{
    ams_ccb_als_ctx_t *ccbCtx = &((ams_deviceCtx_t *)dcbCtx)->ccbAlsCtx;

    /* export data */
    exportData->clear = ccbCtx->ctxAlgAls.results.irrClear;
    exportData->ir = ccbCtx->ctxAlgAls.results.IR;
    exportData->wideband = ccbCtx->ctxAlgAls.results.irrWideband;
    exportData->time_us = ccbCtx->ctxAlgAls.time_us;
    exportData->ClearGain = ccbCtx->ctxAlgAls.ClearGain;
    exportData->WBGain = ccbCtx->ctxAlgAls.WBGain;
    exportData->rawClear = ccbCtx->ctxAlgAls.results.rawClear;
    exportData->rawWideband = ccbCtx->ctxAlgAls.results.rawWideband;
}

//#ifdef CONFIG_AMS_OPTICAL_SENSOR_ALS_CCB
static bool _2510_alsSetThreshold(ams_deviceCtx_t *ctx, int32_t threshold)
{
    ams_ccb_als_config_t configData;

    configData.threshold = threshold;
    ccb_alsSetConfig(ctx, &configData);

    return false;
}
//#endif


static void  ams_sensor_mode_set(ams_deviceCtx_t *ctx, uint8_t  sensor_mode)
{
    int ret = 0;

    ctx->sensor_mode = sensor_mode;

    ALS_dbg("%s - ams_sensor_mode_set %d \n", __func__,sensor_mode);


    if (sensor_mode == 0) {
        AMS_ALS_SENSOR_6PD_TURNON(ret);
    }else{
        AMS_ALS_SENSOR_2PD_TURNON(ret);
    }
}


static int ams_deviceSetConfig(ams_deviceCtx_t *ctx, ams_configureFeature_t feature, deviceConfigOptions_t option, uint32_t data)
{
    int ret = 0;

    if (feature == AMS_CONFIG_ALS_LUX) {
        ALS_dbg("%s - ams_configureFeature_t AMS_CONFIG_ALS_LUX\n", __func__);
        switch (option) {
            case AMS_CONFIG_ENABLE: /* ON / OFF */
                ALS_info("%s - deviceConfigOptions_t AMS_CONFIG_ENABLE(%u)\n", __func__, data);
                ALS_info("%s - current mode %d\n", __func__, ctx->mode);
                if (data == 0) {
                    if (ctx->mode == MODE_ALS_LUX) {
                        /* if no other active features, turn off device */
                        ctx->shadowEnableReg = 0;
                        ctx->shadowIntenabReg = 0;
                        ctx->mode = MODE_OFF;
                    } else {
                        if ((ctx->mode & MODE_ALS_ALL) == MODE_ALS_LUX) {
                            ctx->shadowEnableReg &= ~MASK_AEN;
                            ctx->shadowIntenabReg &= ~MASK_ALS_INT_ALL;
                        }
                        ctx->mode &= ~(MODE_ALS_LUX);
                    }
                }
                else {
                    if ((ctx->mode & MODE_ALS_ALL) == 0) {
                        ret = ccb_alsInit(ctx, &ctx->ccbAlsCtx.initData);
                        if (ret < 0) {
                            ALS_err("%s - failed to ccb_alsInit\n", __func__);
                            return ret;
                        }

                        ctx->shadowEnableReg |= (AEN | PON);
                        //if( ctx->sensor_mode == 1)	{ //als need polling mode , 200msec
                        //ctx->shadowIntenabReg |= AIEN;
                        //}
                    }
                    else {
                        /* force interrupt */
                        ret = ams_setWord(ctx->portHndl, DEVREG_AILT0, 0x00);

                        if (ret < 0) {
                            ALS_err("%s - failed to set DEVREG_AIHTL\n", __func__);
                            return ret;
                        }

                        ret = ams_setByte(ctx->portHndl, DEVREG_AILT2, 0x00);

                        if (ret < 0) {
                            ALS_err("%s - failed to set DEVREG_AIHTL\n", __func__);
                            return ret;
                        }

                    }
                    ctx->mode |= MODE_ALS_LUX;
                }
                break;
            case AMS_CONFIG_THRESHOLD: /* set threshold */
                ALS_info("%s - deviceConfigOptions_t AMS_CONFIG_THRESHOLD\n", __func__);
                ALS_info("%s - data %d\n", __func__, data);
                _2510_alsSetThreshold(ctx, data);
                break;
            default:
                break;
        }
    }
#ifdef CONFIG_AMS_OPTICAL_SENSOR_FLICKER
    if (feature == AMS_CONFIG_FLICKER) {
        ALS_dbg("%s - ams_configureFeature_t AMS_CONFIG_FLICKER\n", __func__);
        switch (option) {
            case AMS_CONFIG_ENABLE: /* power on */
                ALS_info("%s - deviceConfigOptions_t AMS_CONFIG_ENABLE(%u)\n", __func__, data);
                ALS_info("%s - current mode %d\n", __func__, ctx->mode);
                if (data == 0) {
                    ret = ams_setField(ctx->portHndl, DEVREG_INTENAB, LOW, MASK_FIEN);// FIFO Interrupt disable
                    if (ret < 0) {
                        ALS_err("%s - failed to set DEVREG_CFG9\n", __func__);
                        return ret;
                    }
                    if (ctx->mode == MODE_FLICKER) {
                        /* if no other active features, turn off device */
                        ctx->shadowEnableReg = 0;
                        ctx->shadowIntenabReg = 0;
                        ctx->mode = MODE_OFF;
                        ams_setByte(ctx->portHndl, DEVREG_CONTROL, 0x02); //20180828 FIFO Buffer , FINT, FIFO_OV, FIFO_LVL all clear
                    }
                    else {
                        ctx->mode &= ~MODE_FLICKER;
                        ctx->shadowEnableReg &= ~(FDEN);
                    }
                }
                else {
                    ctx->shadowEnableReg |= (PON | FDEN);
                    //#ifndef CONFIG_AMS_OPTICAL_SENSOR_POLLING
                    ctx->shadowIntenabReg |= (SIEN|FIEN);
                    //#endif
                    ctx->mode |= MODE_FLICKER;
                    ccb_flickerInit(ctx /*,&ctx->ccbAlsCtx.initData*/);
                }
                break;
            case AMS_CONFIG_THRESHOLD: /* set threshold */
                ALS_info("%s - deviceConfigOptions_t AMS_CONFIG_THRESHOLD\n", __func__);
                /* TODO?:  set FD_COMPARE value? */
                break;
            default:
                break;
        }
    }
#endif
    ret = ams_setByte(ctx->portHndl, DEVREG_INTENAB, ctx->shadowIntenabReg);
    if (ret < 0) {
        ALS_err("%s - failed to set DEVREG_INTENAB\n", __func__);
        return ret;
    }

    ret = ams_setByte(ctx->portHndl, DEVREG_ENABLE, ctx->shadowEnableReg);
    if (ret < 0) {
        ALS_err("%s - failed to set DEVREG_ENABLE\n", __func__);
        return ret;
    }

    return 0;
}

#define STAR_ATIME  50 //50 msec
#define STAR_D_FACTOR  2266
#if 0
static void als_update_statics(amsAlsContext_t *ctx)
{
    uint64_t tempCpl;
    uint64_t tempTime_us = ctx->time_us;
    uint64_t tempGain = ctx->gain;

    /* test for the potential of overflowing */
    uint32_t maxOverFlow;
#ifdef __KERNEL__
    u64 tmpTerm1;
    u64 tmpTerm2;
#endif
#ifdef __KERNEL__
    u64 tmp = ULLONG_MAX;

    do_div(tmp, ctx->time_us);

    maxOverFlow = (uint32_t)tmp;
#else
    maxOverFlow = (uint64_t)ULLONG_MAX / ctx->time_us;
#endif

    if (maxOverFlow < ctx->gain) {
        /* TODO: need to find use-case to test */
#ifdef __KERNEL__
        tmpTerm1 = tempTime_us;
        do_div(tmpTerm1, 2);
        tmpTerm2 = tempGain;
        do_div(tmpTerm2, 2);
        tempCpl = tmpTerm1 * tmpTerm2;
        do_div(tempCpl, (AMS_ALS_GAIN_FACTOR / 4));
#else
        tempCpl = ((tempTime_us / 2) * (tempGain / 2)) / (AMS_ALS_GAIN_FACTOR / 4);
#endif

    }
    else {
#ifdef __KERNEL__
        tempCpl = (tempTime_us * tempGain);
        do_div(tempCpl, AMS_ALS_GAIN_FACTOR);
#else
        tempCpl = (tempTime_us * tempGain) / AMS_ALS_GAIN_FACTOR;
#endif
    }
    if (tempCpl > (uint32_t)ULONG_MAX) {
        /* if we get here, we have a problem */
        //AMS_PORT_log_Msg_1(AMS_ERROR, "als_update_statics: overflow, setting cpl=%u\n", (uint32_t)ULONG_MAX);
        tempCpl = (uint32_t)ULONG_MAX;
    }

#ifdef AMS_BUILD
    /*UVIR CPL refer as a const value with STAR Proejct */
    ctx->uvir_cpl = (tempTime_us * tempGain);
    do_div(ctx->uvir_cpl, AMS_ALS_GAIN_FACTOR);
    do_div(ctx->uvir_cpl, STAR_D_FACTOR);
#else
    /*UVIR CPL refer as a const value with STAR Proejct */
    ctx->uvir_cpl = (tempTime_us * tempGain) / AMS_ALS_GAIN_FACTOR;
    ctx->uvir_cpl = ctx->uvir_cpl / STAR_D_FACTOR;
#endif
    ctx->previousGain = ctx->gain;

    //	AMS_PORT_log_Msg_4(AMS_DEBUG, "als_update_statics: time=%d, gain=%d, dFactor=%d => cpl=%u\n", ctx->time_us, ctx->gain, ctx->calibration.D_factor, ctx->cpl);
}
#endif
static int amsAlg_als_setConfig(amsAlsContext_t *ctx, amsAlsConf_t *inputData)
{
    int ret = 0;

    if (inputData != NULL) {
        //ctx->gain = inputData->gain;
        ctx->time_us = inputData->time_us;
    }
    //als_update_statics(ctx);

    return ret;
}

/*
 * getConfig: is used to quarry the algorithm's configuration
 */
static int amsAlg_als_getConfig(amsAlsContext_t *ctx, amsAlsConf_t *outputData)
{
    int ret = 0;

    //outputData->gain = ctx->gain;
    outputData->time_us = ctx->time_us;

    return ret;
}

static int amsAlg_als_getResult(amsAlsContext_t *ctx, amsAlsResult_t *outData)
{
    int ret = 0;

    outData->rawClear = ctx->results.rawClear;
    outData->rawWideband = ctx->results.rawWideband;
    outData->irrClear = ctx->results.irrClear;
    outData->irrWideband = ctx->results.irrWideband;
    outData->mLux_ave = ctx->results.mLux_ave / AMS_LUX_AVERAGE_COUNT;
    outData->IR = ctx->results.IR;
    outData->CCT = ctx->results.CCT;
    outData->adaptive = ctx->results.adaptive;

    if (ctx->notStableMeasurement)
        ctx->notStableMeasurement = false;

    outData->mLux = ctx->results.mLux;

    return ret;
}

static int amsAlg_als_initAlg(amsAlsContext_t *ctx, amsAlsInitData_t *initData)
{
    int ret = 0;

    memset(ctx, 0, sizeof(amsAlsContext_t));

    if (initData != NULL) {
        ctx->calibration.Time_base = initData->calibration.Time_base;
        ctx->calibration.thresholdLow = initData->calibration.thresholdLow;
        ctx->calibration.thresholdHigh = initData->calibration.thresholdHigh;
        ctx->calibration.calibrationFactor = initData->calibration.calibrationFactor;
    }

    if (initData != NULL) {
        //        ctx->gain = initData->gain;
        ctx->time_us = initData->time_us;
        ctx->adaptive = initData->adaptive;
    }
    else {
        ALS_dbg("error: initData == NULL\n");
    }

    //als_update_statics(ctx);
    return ret;
}

static int amsAlg_als_getAlgInfo(amsAlsAlgoInfo_t *info)
{
    int ret = 0;

    info->algName = "AMS_ALS";
    info->contextMemSize = sizeof(amsAlsContext_t);
    info->scratchMemSize = 0;

    info->initAlg = &amsAlg_als_initAlg;
    info->processData = &amsAlg_als_processData;
    info->getResult = &amsAlg_als_getResult;
    info->setConfig = &amsAlg_als_setConfig;
    info->getConfig = &amsAlg_als_getConfig;

    return ret;
}
static int tsl2510_print_reg_status(void)
{
    int reg, err;
    u8 recvData;

    for (reg = 0; reg < DEVREG_REG_MAX; reg++) {
        err = tsl2510_read_reg(tsl2510_data, deviceRegisterDefinition[reg].address, &recvData, 1);
        if (err != 0) {
            ALS_err("%s - error reading 0x%02x err:%d\n",
                    __func__, reg, err);
        }
        else {
            ALS_dbg("%s - 0x%02x = 0x%02x\n",
                    __func__, deviceRegisterDefinition[reg].address, recvData);
        }
    }
    return 0;
}

static int tsl2510_set_sampling_rate(u32 sampling_period_ns)
{
    ALS_dbg("%s - sensor_info_data not support\n", __func__);

    return 0;
}


static int tsl2510_set_nr_sample(struct tsl2510_device_data *data , u16 fifo_thr)
{
    ams_deviceCtx_t *ctx = data->deviceCtx;
    int ret = 0;

    AMS_DISABLE_FD(ret);
    if (ret < 0) {
        ALS_err("%s - failed to AMS_DISABLE_FD\n", __func__);
        return ret;
    }
    AMS_SET_FLICKER_NUM_SAMPLES(fifo_thr, ret);
    if (ret < 0) {
        ALS_err("%s - failed to AMS_SET_FLICKER_NUM_SAMPLES\n", __func__);
        return ret;
    }
    AMS_REENABLE_FD(ret);
    if (ret < 0) {
        ALS_err("%s - failed to AMS_REENABLE_FD\n", __func__);
        return ret;
    }
    return 1;
}

static int tsl2510_set_sampling_time(struct tsl2510_device_data *data , u16 time)
{
    ams_deviceCtx_t *ctx = data->deviceCtx;
    int ret = 0;

    AMS_DISABLE_FD(ret);
    if (ret < 0) {
        ALS_err("%s - failed to AMS_DISABLE_FD\n", __func__);
        return ret;
    }
    AMS_SET_SAMPLE_TIME(time, ret);
    if (ret < 0) {
        ALS_err("%s - failed to AMS_SET_SAMPLE_TIME\n", __func__);
        return ret;
    }
    AMS_REENABLE_FD(ret);
    if (ret < 0) {
        ALS_err("%s - failed to AMS_REENABLE_FD\n", __func__);
        return ret;
    }
    return 1;
}


static int tsl2510_hamming_status(struct tsl2510_device_data *data , bool on_off)
{
    ams_deviceCtx_t *ctx = data->deviceCtx;
    int ret = 0;

    AMS_DISABLE_FD(ret);
    if (ret < 0) {
        ALS_err("%s - failed to AMS_DISABLE_FD\n", __func__);
        return ret;
    }

    ctx->hamming_status = on_off;

    AMS_REENABLE_FD(ret);
    if (ret < 0) {
        ALS_err("%s - failed to AMS_REENABLE_FD\n", __func__);
        return ret;
    }
    return 1;
}


static int tsl2510_polling_enable(struct tsl2510_device_data *data , bool on_off)
{
    ams_deviceCtx_t *ctx = data->deviceCtx;
    int ret = 0;

    if( on_off == true){
        AMS_DISABLE_FD(ret); //disable FD
        if (ret < 0) {
            ALS_err("%s - failed to AMS_DISABLE_FD\n", __func__);
            return ret;
        }

        AMS_DISABLE_FDINT(ret);	//disable FINT
        if (ret < 0) {
            ALS_err("%s - failed to AMS_DISABLE_FDINT\n", __func__);
            return ret;
        }
        AMS_REENABLE_ALS(ret);
        hrtimer_start(&data->timer, data->light_poll_delay, HRTIMER_MODE_REL);   /* polling start*/
    }else{
        hrtimer_cancel(&data->timer);/*polling stop*/
        AMS_DISABLE_ALS(ret);
        AMS_REENABLE_FD(ret);
        if (ret < 0) {
            ALS_err("%s - failed to AMS_REENABLE_FD\n", __func__);
            return ret;
        }
        AMS_REENABLE_FDINT(ret);
        if (ret < 0) {
            ALS_err("%s - failed to AMS_REENABLE_FDINT\n", __func__);
            return ret;
        }
    }

    return 1;
}

static int tsl2510_pon_reenable(struct tsl2510_device_data *data , bool on_off)
{
    ams_deviceCtx_t *ctx = data->deviceCtx;
    int ret = 0;

    if(on_off){
        AMS_ENABLE_PON(ret);
        if (ret < 0) {
            ALS_err("%s - failed to AMS_ENABLE_PON\n", __func__);
            return ret;
        }
    } else {
        AMS_REENABLE(ret);
        if (ret < 0) {
            ALS_err("%s - failed to AMS_REENABLE\n", __func__);
            return ret;
        }
    }
    return 1;
}

static int tsl2510_set_fifo_thr(struct tsl2510_device_data *data , u16 fifo_thr)
{
    ams_deviceCtx_t *ctx = data->deviceCtx;
    int ret = 0;

    AMS_DISABLE_FD(ret);
    if (ret < 0) {
        ALS_err("%s - failed to AMS_DISABLE_FD\n", __func__);
        return ret;
    }
    AMS_SET_FIFO_THR(fifo_thr, ret);
    if (ret < 0) {
        ALS_err("%s - failed to AMS_SET_FIFO_THR\n", __func__);
        return ret;
    }

    AMS_REENABLE_FD(ret);
    if (ret < 0) {
        ALS_err("%s - failed to AMS_REENABLE_FD\n", __func__);
        return ret;
    }
    return 1;
}


static void tsl2510_irq_set_state(struct tsl2510_device_data *data, int irq_enable)
{
    ALS_dbg("%s - irq_enable : %d, irq_state : %d\n",
            __func__, irq_enable, data->irq_state);

    if (irq_enable) {
        if (data->irq_state++ == 0)
            enable_irq(data->dev_irq);
    }
    else {
        if (data->irq_state == 0)
            return;
        if (--data->irq_state <= 0) {
            disable_irq(data->dev_irq);
            data->irq_state = 0;
        }
    }
}

static int tsl2510_power_ctrl(struct tsl2510_device_data *data, int onoff)
{
    int rc = 0;
#ifndef AMS_BUILD
    static int i2c_enable_cnt = 0;

    ALS_dbg("%s - onoff : %d, state : %d i2c_enable_cnt : %d\n",
            __func__, onoff, data->regulator_state, i2c_enable_cnt);

    if (onoff == PWR_ON) {
        if (data->regulator_state != 0) {
            ALS_dbg("%s - duplicate regulator\n", __func__);
            data->regulator_state++;
            return 0;
        }

        if (data->i2c_1p8 == true) {
            if (!regulator_is_enabled(data->regulator_i2c_1p8)) {
                rc = regulator_enable(data->regulator_i2c_1p8);
                i2c_enable_cnt++;
            }
            if (rc) {
                ALS_err("%s - enable i2c_1p8 failed, rc=%d\n", __func__, rc);
                goto enable_i2c_1p8_failed;
            }
        }
        switch (data->power_type) {
            case VDD_GPIO :
                rc = gpio_direction_output(data->pin_als_en, 1);
                if (rc) {
                    ALS_err("%s - gpio direction output failed, rc=%d\n",
                            __func__, rc);
                    goto enable_vdd_1p8_failed;
                }
                break;
            case VDD_REGULATOR :
                rc = regulator_set_voltage(data->regulator_vdd_1p8, 1800000, 1800000);
                if (rc) {
                    ALS_err("%s - set voltage failed", __func__);
                    if (regulator_get_voltage(data->regulator_vdd_1p8) > 1800000){
                        ALS_err("%s - voltage too high don't turn on vdd",__func__);
                        goto enable_vdd_1p8_failed;
                    }
                }

                rc = regulator_enable(data->regulator_vdd_1p8);
                if (rc) {
                    ALS_err("%s - enable vdd_1p8 failed, rc=%d\n",
                            __func__, rc);
                    goto enable_vdd_1p8_failed;
                }
                break;
        }
        data->regulator_state++;
        data->pm_state = PM_RESUME;
    } else {    /* POWER OFF */
        if (data->regulator_state == 0) {
            ALS_dbg("%s - already off the regulator\n", __func__);
            return 0;
        }
        else if (data->regulator_state != 1) {
            ALS_dbg("%s - duplicate regulator\n", __func__);
            data->regulator_state--;
            return 0;
        }

        data->regulator_state--;
        switch (data->power_type) {
            case VDD_GPIO :
                gpio_set_value(data->pin_als_en, 0);
                rc = gpio_direction_input(data->pin_als_en);
                if (rc) {
                    ALS_err("%s - gpio direction input failed, rc=%d\n",
                            __func__, rc);
                    goto done;
                }
                break;
            case VDD_REGULATOR :
                if (data->regulator_vdd_1p8 != NULL) {
                    rc = regulator_disable(data->regulator_vdd_1p8);
                    if (rc) {
                        ALS_err("%s - disable vdd_1p8 failed, rc=%d\n",
                                __func__, rc);
                        goto done;
                    }
                }
                break;
        }
#ifdef I2C_1P8_DISABLE
        if (data->i2c_1p8 == true && i2c_enable_cnt > 0) {
            rc = regulator_disable(data->regulator_i2c_1p8);
            if (rc) {
                ALS_err("%s - disable i2c_1p8 failed, rc=%d\n", __func__, rc);
                goto done;
            }
            i2c_enable_cnt--;
        }
#endif
    }

    usleep_range(2000, 2100);
    goto done;
enable_vdd_1p8_failed:
#ifdef I2C_1P8_DISABLE
    if (data->i2c_1p8 == true && i2c_enable_cnt > 0) {
        regulator_disable(data->regulator_i2c_1p8);
        if (rc) {
            ALS_err("%s - disable i2c_1p8 failed, rc=%d\n", __func__, rc);
            goto done;
        }
        i2c_enable_cnt--;
    }
#endif
enable_i2c_1p8_failed:
done:
#endif
    return rc;
}

static bool ams_deviceGetAls(ams_deviceCtx_t *ctx, ams_apiAls_t *exportData);
static bool ams_deviceGetFlicker(ams_deviceCtx_t *ctx, ams_apiAlsFlicker_t *exportData);

static void report_als(struct tsl2510_device_data *chip)
{
    ams_apiAls_t outData = {0};
    static unsigned int als_cnt;
    int temp_ir = 0;

    if (chip->als_input_dev) {
        ams_deviceGetAls(chip->deviceCtx, &outData);
#ifndef AMS_BUILD
        if (chip->saturation) {
            temp_ir = -3;
        } else {
            temp_ir = outData.ir;
        }

        input_report_rel(chip->als_input_dev, REL_X, temp_ir + 1);
        input_report_rel(chip->als_input_dev, REL_RY, outData.clear + 1);
        input_report_abs(chip->als_input_dev, ABS_X, outData.time_us + 1);
        input_report_abs(chip->als_input_dev, ABS_Y, outData.ClearGain + 1);
        input_report_abs(chip->als_input_dev, ABS_Z, outData.WBGain + 1);
        input_sync(chip->als_input_dev);

        if (als_cnt++ > 10) {
            ALS_dbg("%s - I:%d, W:%d, C:%d, TIME:%d, Clear GAIN:%d WB GAIN : %d\n", __func__,
                    temp_ir, outData.wideband, outData.clear, outData.time_us, outData.ClearGain, outData.WBGain);
            als_cnt = 0;
        } else {
            ALS_info("%s - I:%d, W:%d, C:%d, TIME:%d, Clear GAIN:%d WB GAIN : %d\n", __func__,
                    temp_ir, outData.wideband, outData.clear, outData.time_us, outData.ClearGain, outData.WBGain);
        }
#endif
        chip->user_ir_data = temp_ir;
#ifdef CONFIG_AMS_OPTICAL_SENSOR_EOL_MODE
        if (chip->eol_enable && chip->eol_count >= EOL_SKIP_COUNT) {
            chip->eol_awb += outData.ir;
            chip->eol_clear += outData.clear;
            chip->eol_wideband += outData.wideband;
        }
#endif
    }
    chip->saturation = false;
}

static void report_flicker(struct tsl2510_device_data *chip)
{
    ams_apiAlsFlicker_t outData;
    uint flicker = 0;
    static unsigned int flicker_cnt;

    if (chip->als_input_dev) {
        ams_deviceGetFlicker(chip->deviceCtx, &outData);
#ifndef AMS_BUILD
        flicker = outData.mHz;
        input_report_rel(chip->als_input_dev, REL_RZ, flicker + 1);
        input_sync(chip->als_input_dev);

        if (flicker_cnt++ > 10) {
            ALS_dbg("%s - flicker = %d\n", __func__, flicker);
            flicker_cnt = 0;
        }
        else {
            ALS_info("%s - flicker = %d, %d\n", __func__, flicker, outData.mHz);
        }
#endif
        chip->user_flicker_data = outData.mHz;

#ifdef CONFIG_AMS_OPTICAL_SENSOR_EOL_MODE
        if (chip->eol_enable && flicker != 0 && chip->eol_count >= EOL_SKIP_COUNT) {
            if ((chip->eol_state == EOL_STATE_100 && flicker >= 90 && flicker <= 110)
                    || (chip->eol_state == EOL_STATE_120 && flicker >= 110 && flicker <= 130)) {
                chip->eol_flicker += flicker;
                chip->eol_flicker_count++;
            } else {
                ALS_err("%s - out of range! (eol_state = %d, eol_flicker = %d, flicker_avr = %d (sum = %d / cnt = %d))\n", __func__,
                        chip->eol_state, flicker, (chip->eol_flicker/chip->eol_flicker_count), chip->eol_flicker, chip->eol_flicker_count);
            }
        }
#endif

    }
}

static ssize_t als_ir_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    ams_apiAls_t outData = {0};
    struct tsl2510_device_data *chip = dev_get_drvdata(dev);

    ams_deviceGetAls(chip->deviceCtx, &outData);

    return snprintf(buf, PAGE_SIZE, "%d\n", outData.ir);
}

static ssize_t als_clear_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    ams_apiAls_t outData = {0};
    struct tsl2510_device_data *chip = dev_get_drvdata(dev);

    ams_deviceGetAls(chip->deviceCtx, &outData);

    return snprintf(buf, PAGE_SIZE, "%d\n", outData.clear);
}

static ssize_t als_wideband_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    ams_apiAls_t outData = {0};
    struct tsl2510_device_data *chip = dev_get_drvdata(dev);

    ams_deviceGetAls(chip->deviceCtx, &outData);

    return snprintf(buf, PAGE_SIZE, "%d\n", outData.wideband);
}

static ssize_t als_raw_data_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    ams_apiAls_t outData = {0};
    struct tsl2510_device_data *chip = dev_get_drvdata(dev);

    ams_deviceGetAls(chip->deviceCtx, &outData);

    return snprintf(buf, PAGE_SIZE, "%d,%d\n", outData.rawWideband, outData.rawClear);
}

static size_t als_enable_set(struct tsl2510_device_data *chip, uint8_t valueToSet)
{
    int rc = 0;

    ams_sensor_mode_set(chip->deviceCtx,chip->sensor_mode);
    if(chip->sensor_mode == 0){ /*Camera(flicker) + als sensor*/
#if 0
#ifdef CONFIG_AMS_OPTICAL_SENSOR_ALS_CCB
        rc = ams_deviceSetConfig(chip->deviceCtx, AMS_CONFIG_ALS_LUX, AMS_CONFIG_ENABLE, valueToSet);
        if (rc < 0) {
            ALS_err("%s - ams_deviceSetConfig ALS_LUX fail, rc=%d\n", __func__, rc);
            return rc;
        }
#endif
#endif

#ifdef CONFIG_AMS_OPTICAL_SENSOR_FLICKER

        rc = ams_deviceSetConfig(chip->deviceCtx, AMS_CONFIG_FLICKER, AMS_CONFIG_ENABLE, valueToSet);
        if (rc < 0) {
            ALS_err("%s - ams_deviceSetConfig FLICKER fail, rc=%d\n", __func__, rc);
            return rc;
        }
#endif

    }else{ /*ALS ONLY work, should be set PD ( 1 Clear ch , 1 Wide band ch)*/
#ifdef CONFIG_AMS_OPTICAL_SENSOR_ALS_CCB
        rc = ams_deviceSetConfig(chip->deviceCtx, AMS_CONFIG_ALS_LUX, AMS_CONFIG_ENABLE, valueToSet);
        if (rc < 0) {
            ALS_err("%s - ams_deviceSetConfig ALS_LUX fail, rc=%d\n", __func__, rc);
            return rc;
        }
#endif
    }
    rc = tsl2510_polling_enable(chip,chip->sensor_mode);
    if (rc < 0) {
        ALS_err("%s - tsl2510_polling_enable  fail, rc=%d\n", __func__, rc);
        return rc;
    }

    chip->enabled = (u8)valueToSet;
    return 0;
}

#ifdef CONFIG_AMS_OPTICAL_SENSOR_FLICKER
static ssize_t flicker_data_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    ams_apiAlsFlicker_t outData;
    struct tsl2510_device_data *chip = dev_get_drvdata(dev);

    ams_deviceGetFlicker(chip->deviceCtx, &outData);

    return snprintf(buf, PAGE_SIZE, "%d\n", outData.mHz);
}
#endif
/* als input enable/disable sysfs */
static ssize_t tsl2510_enable_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct tsl2510_device_data *data = dev_get_drvdata(dev);
    ams_mode_t mode;

    ams_getMode(data->deviceCtx, &mode);

    if (mode & MODE_ALS_ALL)
        return snprintf(buf, PAGE_SIZE, "%d\n", 1);
    else
        return snprintf(buf, PAGE_SIZE, "%d\n", 0);
}

static int ams_deviceInit(ams_deviceCtx_t *ctx, AMS_PORT_portHndl *portHndl, ams_calibrationData_t *calibrationData);

int tsl2510_stop(struct tsl2510_device_data *data);
int tsl2510_start(struct tsl2510_device_data *data);

static ssize_t tsl2510_enable_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    struct tsl2510_device_data *data = dev_get_drvdata(dev);
    bool value;

    if (strtobool(buf, &value))
        return -EINVAL;

    ALS_dbg("%s - en : %d, c : %d\n", __func__, value, data->enabled);

    if (value)
        tsl2510_start(data);
    else
        tsl2510_stop(data);

    return count;
}

int tsl2510_start(struct tsl2510_device_data *data)
{
    int err = 0;

    ALS_dbg("%s",__func__);
    mutex_lock(&data->activelock);
    if(data->sensor_mode == 0){ /*Camera(flicker) + als sensor*/
        tsl2510_irq_set_state(data, PWR_ON);
    }

    err = tsl2510_power_ctrl(data, PWR_ON);
    if (err < 0) {
        ALS_err("%s - als_regulator_on fail err = %d\n", __func__, err);
        goto mutex_unlock;
    }

    if (data->regulator_state == 1) {
        err = ams_deviceInit(data->deviceCtx, data->client, NULL);
        if (err < 0) {
            ALS_err("%s - ams_deviceInit failed.\n", __func__);
            goto err_device_init;
        }
        ALS_dbg("%s - ams_amsDeviceInit ok\n", __func__);

        err = als_enable_set(data, AMSDRIVER_ALS_ENABLE);
        if (err < 0) {
            input_report_rel(data->als_input_dev,
                    REL_RZ, -5 + 1); /* F_ERR_I2C -5 detected i2c error */
            input_sync(data->als_input_dev);
            ALS_err("%s - enable error %d\n", __func__, err);
            goto err_device_init;
        }
    }

    data->mode_cnt.amb_cnt++;
    goto done;

err_device_init:
    tsl2510_power_ctrl(data, PWR_OFF);
mutex_unlock:
done:
    mutex_unlock(&data->activelock);

    return err;
}

int tsl2510_stop(struct tsl2510_device_data *data)
{
    int err = 0;

    ALS_dbg("%s",__func__);
    mutex_lock(&data->activelock);

    if (data->regulator_state == 0) {
        ALS_dbg("%s - already power off - disable skip\n",
                __func__);
        goto err_already_off;
    } else if (data->regulator_state == 1) {
        err = als_enable_set(data, AMSDRIVER_ALS_DISABLE);
        if (err != 0)
            ALS_err("%s - disable err : %d\n", __func__, err);
    }

    err = tsl2510_power_ctrl(data, PWR_OFF);
    if (err < 0)
        ALS_err("%s - als_regulator_off fail err = %d\n",
                __func__, err);

#ifndef AMS_BUILD
    if(data->sensor_mode == 0){ /*Camera(flicker) + als sensor*/
        tsl2510_irq_set_state(data, PWR_OFF);
    }
#endif

#ifdef CONFIG_AMS_OPTICAL_SENSOR_POLLING
    hrtimer_cancel(&data->timer);
    cancel_work_sync(&data->work_light);
#endif


err_already_off:
    mutex_unlock(&data->activelock);

    return err;
}

static ssize_t tsl2510_poll_delay_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct tsl2510_device_data *data = dev_get_drvdata(dev);

    return snprintf(buf, PAGE_SIZE, "%d\n", data->sampling_period_ns);
}

static ssize_t tsl2510_poll_delay_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t size)
{
    struct tsl2510_device_data *data = dev_get_drvdata(dev);
    u32 sampling_period_ns = 0;
    int err = 0;

    mutex_lock(&data->activelock);

    err = kstrtoint(buf, 10, &sampling_period_ns);

    if (err < 0) {
        ALS_err("%s - kstrtoint failed.(%d)\n", __func__, err);
        mutex_unlock(&data->activelock);
        return err;
    }
    data->light_poll_delay = ns_to_ktime(sampling_period_ns);
    err = tsl2510_set_sampling_rate(sampling_period_ns);

    if (err > 0)
        data->sampling_period_ns = sampling_period_ns;

    ALS_dbg("%s - tsl2510_poll_delay_store  as %d\n", __func__, sampling_period_ns);

    mutex_unlock(&data->activelock);

    return size;
}

static ssize_t tsl2510_fifo_thr_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct tsl2510_device_data *data = dev_get_drvdata(dev);

    return snprintf(buf, PAGE_SIZE, "%d\n", data->fifo_thr);
}

static ssize_t tsl2510_fifo_thr_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t size)
{
    struct tsl2510_device_data *data = dev_get_drvdata(dev);
    u32 fifo_thr = 0;
    int err = 0;

    mutex_lock(&data->activelock);

    err = kstrtoint(buf, 10, &fifo_thr);

    if (err < 0) {
        ALS_err("%s - kstrtoint failed.(%d)\n", __func__, err);
        mutex_unlock(&data->activelock);
        return err;
    }
    err = tsl2510_set_fifo_thr(data,fifo_thr);


    ALS_dbg("%s - tsl2510_fifo_thr_store  as %d\n", __func__, fifo_thr);

    mutex_unlock(&data->activelock);

    return size;
}

static ssize_t tsl2510_fd_nr_sample_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct tsl2510_device_data *data = dev_get_drvdata(dev);

    return snprintf(buf, PAGE_SIZE, "%d\n", data->fifo_thr);
}

static ssize_t tsl2510_fd_nr_sample_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t size)
{
    struct tsl2510_device_data *data = dev_get_drvdata(dev);
    u32 fifo_thr = 0;
    int err = 0;

    mutex_lock(&data->activelock);

    err = kstrtoint(buf, 10, &fifo_thr);

    if (err < 0) {
        ALS_err("%s - kstrtoint failed.(%d)\n", __func__, err);
        mutex_unlock(&data->activelock);
        return err;
    }
    err = tsl2510_set_nr_sample(data,fifo_thr);


    ALS_dbg("%s - tsl2510_fd_nr_sample_store  as %d\n", __func__, fifo_thr);

    mutex_unlock(&data->activelock);

    return size;
}

static ssize_t tsl2510_sampling_time_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct tsl2510_device_data *data = dev_get_drvdata(dev);

    return snprintf(buf, PAGE_SIZE, "%d\n", data->fifo_thr);
}

static ssize_t tsl2510_sampling_time_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t size)
{
    struct tsl2510_device_data *data = dev_get_drvdata(dev);
    u32 fifo_thr = 0;
    int err = 0;

    mutex_lock(&data->activelock);

    err = kstrtoint(buf, 10, &fifo_thr);

    if (err < 0) {
        ALS_err("%s - kstrtoint failed.(%d)\n", __func__, err);
        mutex_unlock(&data->activelock);
        return err;
    }
    err = tsl2510_set_sampling_time(data,fifo_thr);


    ALS_dbg("%s - tsl2510_set_sampling_time as %d\n", __func__, fifo_thr);

    mutex_unlock(&data->activelock);

    return size;
}


static ssize_t tsl2510_hamming_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct tsl2510_device_data *data = dev_get_drvdata(dev);

    return snprintf(buf, PAGE_SIZE, "%d\n", data->fifo_thr);
}

static ssize_t tsl2510_hamming_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t size)
{
    struct tsl2510_device_data *data = dev_get_drvdata(dev);
    u32 hamming_on_ff = 0;
    int err = 0;

    mutex_lock(&data->activelock);

    err = kstrtoint(buf, 10, &hamming_on_ff);

    if (err < 0) {
        ALS_err("%s - kstrtoint failed.(%d)\n", __func__, err);
        mutex_unlock(&data->activelock);
        return err;
    }


    err = tsl2510_hamming_status(data,hamming_on_ff);


    ALS_dbg("%s - tsl2510_hamming_store %d \n", __func__, hamming_on_ff);

    mutex_unlock(&data->activelock);

    return size;
}


static ssize_t tsl2510_polling_enable_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t size)
{
    struct tsl2510_device_data *data = dev_get_drvdata(dev);
    u32 hamming_on_ff = 0;
    int err = 0;

    mutex_lock(&data->activelock);

    err = kstrtoint(buf, 10, &hamming_on_ff);

    if (err < 0) {
        ALS_err("%s - kstrtoint failed.(%d)\n", __func__, err);
        mutex_unlock(&data->activelock);
        return err;
    }

    err = tsl2510_polling_enable(data,hamming_on_ff);

    ALS_dbg("%s - tsl2510_polling_enable_store %d \n", __func__, hamming_on_ff);

    mutex_unlock(&data->activelock);

    return size;
}

static ssize_t tsl2510_regs_write_store (struct device *dev,
        struct device_attribute *attr, const char *buf, size_t size)
{
    int num = 0;
    int ret = 0;
    u8 reg = 0x00;

    char r_value = 0x00;
    char w_value = 0x00;

    struct tsl2510_device_data *data = dev_get_drvdata(dev);

    num = sscanf(buf,"W:0x%hhx,0x%hhx",&reg,&w_value);

    mutex_lock(&data->activelock);

    ret = tsl2510_read_reg(data, (u8)reg, &r_value, 1);
    if (ret != 0) {
        ALS_err("%s - err=%d, val=0x%06x\n",
                __func__, ret, r_value);
        mutex_unlock(&data->i2clock);
        return ret;
    }

    ret = tsl2510_pon_reenable(data,true);//pon

    if (ret < 0) {
        ALS_err("%s - failed to AMS_ENABLE_PON\n", __func__);
        return 0;
    }

    ALS_dbg("%s   read reg 0x%x , val 0x%x \n",__func__, reg,r_value);

    ret = tsl2510_write_reg(data, (u8)reg, (u8)w_value);
    if (ret < 0) {
        ALS_err("%s - fail err = %d\n", __func__, ret);
        mutex_unlock(&data->i2clock);
        return ret;
    }

    ALS_dbg("%s   write reg 0x%x , val 0x%x \n",__func__, reg,w_value);

    ret = tsl2510_pon_reenable(data,false);//reenable

    if (ret < 0) {
        ALS_err("%s - failed to AMS_REENABLE\n", __func__);
        return 0;
    }

    mutex_unlock(&data->activelock);
    return size;
}

static ssize_t tsl2510_sensor_mode_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct tsl2510_device_data *data = dev_get_drvdata(dev);

    return snprintf(buf, PAGE_SIZE, "%d\n", data->sensor_mode);
}

static ssize_t tsl2510_sensor_mode_store (struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t size)
{
    struct tsl2510_device_data *data = dev_get_drvdata(dev);
    u32 sensor_mode = 0;
    int err = 0;
    int prev_state = 0;

    prev_state = data->enabled;

    if (data->enabled > 0) {
        tsl2510_stop(data);
    }
    mutex_lock(&data->activelock);

    err = kstrtoint(buf, 10, &sensor_mode);

    if (err < 0) {
        ALS_err("%s - kstrtoint failed.(%d)\n", __func__, err);
        mutex_unlock(&data->activelock);
        return err;
    }

    data->sensor_mode = (u8)sensor_mode;
    ALS_dbg("%s - tsl2510_sensor_mode %d \n", __func__, sensor_mode);

    mutex_unlock(&data->activelock);

    if (prev_state > 0) {
        tsl2510_start(data);
    }

    return size;
}


static DEVICE_ATTR(enable, S_IRUGO | S_IWUSR | S_IWGRP,
        tsl2510_enable_show, tsl2510_enable_store);
static DEVICE_ATTR(poll_delay, S_IRUGO | S_IWUSR | S_IWGRP,
        tsl2510_poll_delay_show, tsl2510_poll_delay_store);
static DEVICE_ATTR(fifo_thr, S_IRUGO | S_IWUSR | S_IWGRP,
        tsl2510_fifo_thr_show, tsl2510_fifo_thr_store);
static DEVICE_ATTR(fd_nr_sample, S_IRUGO | S_IWUSR | S_IWGRP,
        tsl2510_fd_nr_sample_show, tsl2510_fd_nr_sample_store);
static DEVICE_ATTR(sampling_time, S_IRUGO | S_IWUSR | S_IWGRP,
        tsl2510_sampling_time_show, tsl2510_sampling_time_store);
static DEVICE_ATTR(hamming_on_off, S_IRUGO | S_IWUSR | S_IWGRP,
        tsl2510_hamming_show, tsl2510_hamming_store);
static DEVICE_ATTR(poll_enable, S_IRUGO | S_IWUSR | S_IWGRP,
        NULL, tsl2510_polling_enable_store);
static DEVICE_ATTR(2510_regs, S_IRUGO | S_IWUSR | S_IWGRP,
        NULL, tsl2510_regs_write_store);
static DEVICE_ATTR(sensor_mode, S_IRUGO | S_IWUSR | S_IWGRP,
        tsl2510_sensor_mode_show, tsl2510_sensor_mode_store);

static struct attribute *als_sysfs_attrs[] = {
    &dev_attr_enable.attr,
    &dev_attr_poll_delay.attr,
    &dev_attr_poll_enable.attr,
    &dev_attr_fifo_thr.attr,
    &dev_attr_fd_nr_sample.attr,
    &dev_attr_sampling_time.attr,
    &dev_attr_hamming_on_off.attr,
    &dev_attr_2510_regs.attr,
    &dev_attr_sensor_mode.attr,
    NULL
};

static struct attribute_group als_attribute_group = {
    .attrs = als_sysfs_attrs,
};

/* als_sensor sysfs */
static ssize_t tsl2510_name_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct tsl2510_device_data *data = dev_get_drvdata(dev);
    ams_deviceCtx_t *ctx = data->deviceCtx;
    char chip_name[NAME_LEN];

    switch (ctx->deviceId) {
        case AMS_TSL2510:
        case AMS_TSL2510_UNTRIM:
            strlcpy(chip_name, TSL2510_CHIP_NAME, sizeof(chip_name));
            break;
        default:
            strlcpy(chip_name, TSL2510_CHIP_NAME, sizeof(chip_name));
            break;
    }

    return snprintf(buf, PAGE_SIZE, "%s\n", chip_name);
}

static ssize_t tsl2510_vendor_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    return snprintf(buf, PAGE_SIZE, "%s\n", VENDOR);
}

static ssize_t tsl2510_flush_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t size)
{
    struct tsl2510_device_data *data = dev_get_drvdata(dev);
    ams_deviceCtx_t *ctx = data->deviceCtx;
    int ret = 0;
    u8 handle = 0;

    mutex_lock(&data->activelock);
    ret = kstrtou8(buf, 10, &handle);
    if (ret < 0) {
        ALS_err("%s - kstrtou8 failed.(%d)\n", __func__, ret);
        mutex_unlock(&data->activelock);
        return ret;
    }
    ALS_dbg("%s - handle = %d\n", __func__, handle);
    mutex_unlock(&data->activelock);

    AMS_FIFO_CLEAR(ret);
    if (ret < 0) {
        ALS_err("%s - failed to FIFO CLEAR\n", __func__);
        return ret;
    }

    input_report_rel(data->als_input_dev, REL_MISC, handle);

    return size;
}

static ssize_t tsl2510_int_pin_check_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    /* need to check if this should be implemented */
    ALS_dbg("%s - not implement\n", __func__);
    return snprintf(buf, PAGE_SIZE, "%d\n", 0);
}

static ssize_t tsl2510_read_reg_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct tsl2510_device_data *data = dev_get_drvdata(dev);

    ALS_info("%s - val=0x%06x\n", __func__, data->reg_read_buf);

    return snprintf(buf, PAGE_SIZE, "%d\n", data->reg_read_buf);
}

static ssize_t tsl2510_read_reg_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t size)
{
    struct tsl2510_device_data *data = dev_get_drvdata(dev);

    int err = -1;
    unsigned int cmd = 0;
    u8 val = 0;

    mutex_lock(&data->i2clock);
    if (data->regulator_state == 0) {
        ALS_dbg("%s - need to power on\n", __func__);
        mutex_unlock(&data->i2clock);
        return size;
    }
    err = sscanf(buf, "%8x", &cmd);
    if (err == 0) {
        ALS_err("%s - sscanf fail\n", __func__);
        mutex_unlock(&data->i2clock);
        return size;
    }

    err = tsl2510_read_reg(data, (u8)cmd, &val, 1);
    if (err != 0) {
        ALS_err("%s - err=%d, val=0x%06x\n",
                __func__, err, val);
        mutex_unlock(&data->i2clock);
        return size;
    }
    data->reg_read_buf = (u32)val;
    mutex_unlock(&data->i2clock);

    return size;
}
static ssize_t tsl2510_write_reg_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t size)
{
    struct tsl2510_device_data *data = dev_get_drvdata(dev);

    int err = -1;
    unsigned int cmd = 0;
    unsigned int val = 0;

    mutex_lock(&data->i2clock);
    if (data->regulator_state == 0) {
        ALS_dbg("%s - need to power on.\n", __func__);
        mutex_unlock(&data->i2clock);
        return size;
    }
    err = sscanf(buf, "%8x, %8x", &cmd, &val);
    if (err == 0) {
        ALS_err("%s - sscanf fail %s\n", __func__, buf);
        mutex_unlock(&data->i2clock);
        return size;
    }

    err = tsl2510_write_reg(data, (u8)cmd, (u8)val);
    if (err < 0) {
        ALS_err("%s - fail err = %d\n", __func__, err);
        mutex_unlock(&data->i2clock);
        return err;
    }
    mutex_unlock(&data->i2clock);

    return size;
}

static ssize_t tsl2510_debug_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct tsl2510_device_data *data = dev_get_drvdata(dev);

    ALS_info("%s - debug mode = %u\n", __func__, data->debug_mode);

    return snprintf(buf, PAGE_SIZE, "%u\n", data->debug_mode);
}

static ssize_t tsl2510_debug_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t size)
{
    struct tsl2510_device_data *data = dev_get_drvdata(dev);
    int err;
    s32 mode;

    mutex_lock(&data->activelock);
    err = kstrtoint(buf, 10, &mode);
    if (err < 0) {
        ALS_err("%s - kstrtoint failed.(%d)\n", __func__, err);
        mutex_unlock(&data->activelock);
        return err;
    }
    data->debug_mode = (u8)mode;
    ALS_info("%s - mode = %d\n", __func__, mode);

    switch (data->debug_mode) {
        case DEBUG_REG_STATUS:
            tsl2510_print_reg_status();
            break;
        case DEBUG_VAR:
            tsl2510_debug_var(data);
            break;
        default:
            break;
    }
    mutex_unlock(&data->activelock);

    return size;
}

static ssize_t tsl2510_device_id_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    ALS_dbg("%s - device_id not support\n", __func__);

    return snprintf(buf, PAGE_SIZE, "NOT SUPPORT\n");
}

static ssize_t tsl2510_part_type_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct tsl2510_device_data *data = dev_get_drvdata(dev);
    ams_deviceCtx_t *ctx = data->deviceCtx;

    return snprintf(buf, PAGE_SIZE, "%d\n", ctx->deviceId);
}

static ssize_t tsl2510_i2c_err_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct tsl2510_device_data *data = dev_get_drvdata(dev);
    u32 err_cnt = 0;

    err_cnt = data->i2c_err_cnt;

    return snprintf(buf, PAGE_SIZE, "%d\n", err_cnt);
}

static ssize_t tsl2510_i2c_err_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t size)
{
    struct tsl2510_device_data *data = dev_get_drvdata(dev);

    data->i2c_err_cnt = 0;

    return size;
}

static ssize_t tsl2510_curr_adc_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct tsl2510_device_data *data = dev_get_drvdata(dev);

    return snprintf(buf, PAGE_SIZE,
            "\"HRIC\":\"%d\",\"HRRC\":\"%d\",\"HRIA\":\"%d\",\"HRRA\":\"%d\"\n",
            0, 0, data->user_ir_data, data->user_flicker_data);
}

static ssize_t tsl2510_curr_adc_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t size)
{
    struct tsl2510_device_data *data = dev_get_drvdata(dev);

    data->user_ir_data = 0;
    data->user_flicker_data = 0;

    return size;
}

static ssize_t tsl2510_mode_cnt_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct tsl2510_device_data *data = dev_get_drvdata(dev);

    return snprintf(buf, PAGE_SIZE,
            "\"CNT_HRM\":\"%d\",\"CNT_AMB\":\"%d\",\"CNT_PROX\":\"%d\",\"CNT_SDK\":\"%d\",\"CNT_CGM\":\"%d\",\"CNT_UNKN\":\"%d\"\n",
            data->mode_cnt.hrm_cnt, data->mode_cnt.amb_cnt, data->mode_cnt.prox_cnt,
            data->mode_cnt.sdk_cnt, data->mode_cnt.cgm_cnt, data->mode_cnt.unkn_cnt);
}

static ssize_t tsl2510_mode_cnt_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t size)
{
    struct tsl2510_device_data *data = dev_get_drvdata(dev);

    data->mode_cnt.hrm_cnt = 0;
    data->mode_cnt.amb_cnt = 0;
    data->mode_cnt.prox_cnt = 0;
    data->mode_cnt.sdk_cnt = 0;
    data->mode_cnt.cgm_cnt = 0;
    data->mode_cnt.unkn_cnt = 0;

    return size;
}

static ssize_t tsl2510_factory_cmd_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct tsl2510_device_data *data = dev_get_drvdata(dev);
    static int cmd_result;

    mutex_lock(&data->activelock);

    if (data->isTrimmed)
        cmd_result = 1;
    else
        cmd_result = 0;

    ALS_dbg("%s - cmd_result = %d\n", __func__, cmd_result);

    mutex_unlock(&data->activelock);

    return snprintf(buf, PAGE_SIZE, "%d\n", cmd_result);
}

static ssize_t tsl2510_version_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    ALS_info("%s - cmd_result = %s.%s.%s%s\n", __func__,
            VERSION, SUB_VERSION, HEADER_VERSION, VENDOR_VERSION);

    return snprintf(buf, PAGE_SIZE, "%s.%s.%s%s\n",
            VERSION, SUB_VERSION, HEADER_VERSION, VENDOR_VERSION);
}

static ssize_t tsl2510_sensor_info_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    ALS_dbg("%s - sensor_info_data not support\n", __func__);

    return snprintf(buf, PAGE_SIZE, "NOT SUPPORT\n");
}

#ifdef CONFIG_AMS_OPTICAL_SENSOR_EOL_MODE

static int tsl2510_eol_mode(struct tsl2510_device_data *data)
{
    int led_curr = 0;
    int pulse_duty = 0;
    int curr_state = EOL_STATE_INIT;
    int ret = 0;
    int icRatio100 = 0;
    int icRatio120 = 0;
    s32 pin_eol_en = 0, eol_led_mode;
    u32 prev_eol_count = 0, loop_count = 0;

    if (data->eol_flash_type == EOL_FLASH) {
        ALS_dbg("%s - flash gpio", __func__);
        pin_eol_en = data->pin_flash_en;
        eol_led_mode = S2MPB02_FLASH_LED_1;
    } else {
        ALS_dbg("%s - torch gpio", __func__);
        pin_eol_en = data->pin_torch_en;
        eol_led_mode = S2MPB02_TORCH_LED_1;
    }

    ret = gpio_request(pin_eol_en, NULL);
    if (ret < 0)
        return ret;

    data->eol_state = EOL_STATE_INIT;
    data->eol_enable = 1;
    data->eol_result_status = 1;

    s2mpb02_led_en(eol_led_mode, led_curr, S2MPB02_LED_TURN_WAY_GPIO);

    /* set min flash current */
    if (data->eol_flash_type == EOL_FLASH) {
        led_curr = S2MPB02_FLASH_OUT_I_150MA;
    } else {
        led_curr = S2MPB02_TORCH_OUT_I_80MA;
    }

    ALS_dbg("%s - eol_loop start", __func__);
    while (data->eol_state < EOL_STATE_DONE) {
        if (prev_eol_count == data->eol_count)
            loop_count++;
        else
            loop_count = 0;

        prev_eol_count = data->eol_count;

        switch (data->eol_state) {
            case EOL_STATE_INIT:
                pulse_duty = 1000;
                break;
            case EOL_STATE_100:
                pulse_duty = data->eol_pulse_duty[0];
                break;
            case EOL_STATE_120:
                pulse_duty = data->eol_pulse_duty[1];
                break;
            default:
                break;
        }

        if (data->eol_state >= EOL_STATE_100) {
            if (curr_state != data->eol_state) {
                s2mpb02_led_en(eol_led_mode, led_curr, S2MPB02_LED_TURN_WAY_GPIO);
                curr_state = data->eol_state;
            }
            else
                gpio_direction_output(pin_eol_en, 1);

            udelay(pulse_duty);

            gpio_direction_output(pin_eol_en, 0);

            data->eol_pulse_count++;
        }

        if (loop_count > 1000) {
            ALS_err("ERR NO Interrupt");
            tsl2510_print_reg_status();
            break;
        }

        udelay(pulse_duty);
    }
    ALS_dbg("%s - eol loop end",__func__);
    s2mpb02_led_en(eol_led_mode, 0, S2MPB02_LED_TURN_WAY_GPIO);
    gpio_free(pin_eol_en);

    if (data->eol_state >= EOL_STATE_DONE) {
        icRatio100 = data->eol_flicker_awb[EOL_STATE_100][1] * 100 / data->eol_flicker_awb[EOL_STATE_100][2];
        icRatio120 = data->eol_flicker_awb[EOL_STATE_120][1] * 100 / data->eol_flicker_awb[EOL_STATE_120][2];

        snprintf(data->eol_result, MAX_TEST_RESULT,
                "%d, %s, %d, %s, %d, %s, %d, %s, %d, %s, %d, %s, %d, %s, %d, %s\n",
                data->eol_flicker_awb[EOL_STATE_100][0], FREQ100_SPEC_IN(data->eol_flicker_awb[EOL_STATE_100][0], data->eol_flicker_pass_cnt[EOL_STATE_100]),
                data->eol_flicker_awb[EOL_STATE_120][0], FREQ120_SPEC_IN(data->eol_flicker_awb[EOL_STATE_120][0], data->eol_flicker_pass_cnt[EOL_STATE_120]),
                data->eol_flicker_awb[EOL_STATE_100][3], IR_SPEC_IN(data->eol_flicker_awb[EOL_STATE_100][3]),
                data->eol_flicker_awb[EOL_STATE_120][3], IR_SPEC_IN(data->eol_flicker_awb[EOL_STATE_120][3]),
                data->eol_flicker_awb[EOL_STATE_100][2], CLEAR_SPEC_IN(data->eol_flicker_awb[EOL_STATE_100][2]),
                data->eol_flicker_awb[EOL_STATE_120][2], CLEAR_SPEC_IN(data->eol_flicker_awb[EOL_STATE_120][2]),
                icRatio100, ICRATIO_SPEC_IN(icRatio100),
                icRatio120, ICRATIO_SPEC_IN(icRatio120));
    } else {
        ALS_err("%s - abnormal termination\n", __func__);
    }

    ALS_dbg("%s - %s", __func__, data->eol_result);

    data->eol_enable = 0;

    return 0;
}

static ssize_t tsl2510_eol_mode_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct tsl2510_device_data *data = dev_get_drvdata(dev);

    mutex_lock(&data->activelock);

    if (data->eol_result == NULL) {
        ALS_err("%s - data->eol_result is NULL\n", __func__);
        mutex_unlock(&data->activelock);
        return snprintf(buf, PAGE_SIZE, "%s\n", "NO_EOL_TEST");
    }
    if (data->eol_enable == 1) {
        mutex_unlock(&data->activelock);
        return snprintf(buf, PAGE_SIZE, "%s\n", "EOL_RUNNING");
    }
    else if (data->eol_enable == 0 && data->eol_result_status == 0) {
        mutex_unlock(&data->activelock);
        return snprintf(buf, PAGE_SIZE, "%s\n", "NO_EOL_TEST");
    }
    mutex_unlock(&data->activelock);

    data->eol_result_status = 0;
    return snprintf(buf, PAGE_SIZE, "%s\n", data->eol_result);
}

static ssize_t tsl2510_eol_mode_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t size)
{
    struct tsl2510_device_data *data = dev_get_drvdata(dev);
    ams_deviceCtx_t *ctx = data->deviceCtx;

    int err = 0;
    int mode = 0;
    static int prev_sensor_mode = 0, prev_state = 0;

    err = kstrtoint(buf, 10, &mode);
    if (err < 0) {
        ALS_err("%s - kstrtoint failed.(%d)\n", __func__, err);
        return err;
    }

    data->eol_flash_type = EOL_TORCH;

    switch (mode) {
        case 1:
            gSpec_ir_min = data->eol_ir_spec[0];
            gSpec_ir_max = data->eol_ir_spec[1];
            gSpec_clear_min = data->eol_clear_spec[0];
            gSpec_clear_max = data->eol_clear_spec[1];
            gSpec_icratio_min = data->eol_icratio_spec[0];
            gSpec_icratio_max = data->eol_icratio_spec[1];
            break;

        case 2:
            gSpec_ir_min = data->eol_ir_spec[2];
            gSpec_ir_max = data->eol_ir_spec[3];
            gSpec_clear_min = data->eol_clear_spec[2];
            gSpec_clear_max = data->eol_clear_spec[3];
            gSpec_icratio_min = data->eol_icratio_spec[2];
            gSpec_icratio_max = data->eol_icratio_spec[3];
            break;

        case 100:
            // USE OPEN SPEC
            gSpec_ir_min = DEFAULT_IR_SPEC_MIN;
            gSpec_ir_max = DEFAULT_IR_SPEC_MAX;
            gSpec_clear_min = DEFAULT_IR_SPEC_MIN;
            gSpec_clear_max = DEFAULT_IR_SPEC_MAX;
            gSpec_icratio_min = DEFAULT_IC_SPEC_MIN;
            gSpec_icratio_max = DEFAULT_IC_SPEC_MAX;

            if (data->pin_flash_en >= 0) {
                data->eol_flash_type = EOL_FLASH;
                ALS_dbg("%s - use flash gpio", __func__);
            } else {
                ALS_dbg("%s - flash gpio not setted. use torch gpio", __func__);
            }
            break;

        default:
            break;
    }

    ALS_dbg("%s - mode = %d, gSpec_ir_min = %d, gSpec_ir_max = %d, gSpec_clear_min = %d, gSpec_clear_max = %d, gSpec_icratio = %d - %d, eol_flash_type : %d\n",
            __func__, mode, gSpec_ir_min, gSpec_ir_max, gSpec_clear_min, gSpec_clear_max, gSpec_icratio_min, gSpec_icratio_max, data->eol_flash_type);

    prev_sensor_mode = data->sensor_mode;
    data->sensor_mode = 0;

    if (data->regulator_state > 0) {
        ALS_dbg("%s - init sensor mode", __func__);
        prev_state = data->regulator_state;
        /* Initialize to set sensor mode CAMERA */
        while (data->regulator_state)
            tsl2510_stop(data);
        while (data->regulator_state < prev_state)
            tsl2510_start(data);
    }

    err = tsl2510_start(data);

    AMS_AGC_DISABLE(err);					//AMS_SET_ALS_AUTOGAIN(LOW, err);
    if (err < 0) {
        ALS_err("%s - failed to disable AGC\n", __func__);
        return 0;
    }

    AMS_SET_ALS_GAIN0(EOL_GAIN, err);
    if (err < 0) {
        ALS_err("%s - failed to AMS_SET_ALS_GAIN\n", __func__);
        return 0;
    }

    AMS_SET_ALS_GAIN1(EOL_GAIN, err);
    if (err < 0) {
        ALS_err("%s - failed to AMS_SET_ALS_GAIN\n", __func__);
        return 0;
    }
    ALS_dbg("%s - fixed ALS GAIN : %d\n", __func__, EOL_GAIN);

    tsl2510_eol_mode(data);

    AMS_SET_ALS_GAIN0(ctx->ccbAlsCtx.initData.configData.gain, err);
    if (err < 0) {
        ALS_err("%s - failed to AMS_SET_ALS_GAIN\n", __func__);
        return 0;
    }

    AMS_SET_ALS_GAIN1(ctx->ccbAlsCtx.initData.configData.gain, err);
    if (err < 0) {
        ALS_err("%s - failed to AMS_SET_ALS_GAIN\n", __func__);
        return 0;
    }

    AMS_AGC_ENABLE(err);					//AMS_SET_ALS_AUTOGAIN(LOW, err);
    if(err < 0){
        ALS_err("%s - Set ALS GAIN fail", __func__);
        return 0;
    }

    err = tsl2510_stop(data);

    data->sensor_mode = prev_sensor_mode;

    if (data->regulator_state > 0) {
        ALS_dbg("%s - init sensor mode", __func__);
        prev_state = data->regulator_state;
        /* Initialize to set sensor mode CAMERA */
        while (data->regulator_state)
            tsl2510_stop(data);
        while (data->regulator_state < prev_state)
            tsl2510_start(data);
    }
    prev_state = 0;

    if (err<0)
        ALS_dbg("%s - err in stop", __func__);

    return size;
}

static ssize_t tsl2510_eol_spec_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    struct tsl2510_device_data *data = dev_get_drvdata(dev);

    ALS_dbg("%s - ir_spec = %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d\n", __func__,
            data->eol_ir_spec[0], data->eol_ir_spec[1], data->eol_clear_spec[0], data->eol_clear_spec[1],
            data->eol_ir_spec[2], data->eol_ir_spec[3], data->eol_clear_spec[2], data->eol_clear_spec[3],
            data->eol_icratio_spec[0], data->eol_icratio_spec[1], data->eol_icratio_spec[2], data->eol_icratio_spec[3]);

    return snprintf(buf, PAGE_SIZE, "%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d\n",
            data->eol_ir_spec[0], data->eol_ir_spec[1], data->eol_clear_spec[0], data->eol_clear_spec[1],
            data->eol_ir_spec[2], data->eol_ir_spec[3], data->eol_clear_spec[2], data->eol_clear_spec[3],
            data->eol_icratio_spec[0], data->eol_icratio_spec[1], data->eol_icratio_spec[2], data->eol_icratio_spec[3]);
}
#endif

static DEVICE_ATTR(name, S_IRUGO, tsl2510_name_show, NULL);
static DEVICE_ATTR(vendor, S_IRUGO, tsl2510_vendor_show, NULL);
static DEVICE_ATTR(als_flush, S_IWUSR | S_IWGRP, NULL, tsl2510_flush_store);
static DEVICE_ATTR(int_pin_check, S_IRUGO, tsl2510_int_pin_check_show, NULL);
static DEVICE_ATTR(read_reg, S_IRUGO | S_IWUSR | S_IWGRP,
        tsl2510_read_reg_show, tsl2510_read_reg_store);
static DEVICE_ATTR(write_reg, S_IWUSR | S_IWGRP, NULL, tsl2510_write_reg_store);
static DEVICE_ATTR(als_debug, S_IRUGO | S_IWUSR | S_IWGRP,
        tsl2510_debug_show, tsl2510_debug_store);
static DEVICE_ATTR(device_id, S_IRUGO, tsl2510_device_id_show, NULL);
static DEVICE_ATTR(part_type, S_IRUGO, tsl2510_part_type_show, NULL);
static DEVICE_ATTR(i2c_err_cnt, S_IRUGO | S_IWUSR | S_IWGRP, tsl2510_i2c_err_show, tsl2510_i2c_err_store);
static DEVICE_ATTR(curr_adc, S_IRUGO | S_IWUSR | S_IWGRP, tsl2510_curr_adc_show, tsl2510_curr_adc_store);
static DEVICE_ATTR(mode_cnt, S_IRUGO | S_IWUSR | S_IWGRP, tsl2510_mode_cnt_show, tsl2510_mode_cnt_store);
static DEVICE_ATTR(als_factory_cmd, S_IRUGO, tsl2510_factory_cmd_show, NULL);
static DEVICE_ATTR(als_version, S_IRUGO, tsl2510_version_show, NULL);
static DEVICE_ATTR(sensor_info, S_IRUGO, tsl2510_sensor_info_show, NULL);
static DEVICE_ATTR(als_ir, S_IRUGO, als_ir_show, NULL);
static DEVICE_ATTR(als_clear, S_IRUGO, als_clear_show, NULL);
static DEVICE_ATTR(als_wideband, S_IRUGO, als_wideband_show, NULL);
static DEVICE_ATTR(als_raw_data, S_IRUGO, als_raw_data_show, NULL);
#ifdef CONFIG_AMS_OPTICAL_SENSOR_FLICKER
static DEVICE_ATTR(flicker_data, S_IRUGO, flicker_data_show, NULL);
#endif
#ifdef CONFIG_AMS_OPTICAL_SENSOR_EOL_MODE
static DEVICE_ATTR(eol_mode, S_IRUGO | S_IWUSR | S_IWGRP, tsl2510_eol_mode_show, tsl2510_eol_mode_store);
static DEVICE_ATTR(eol_spec, S_IRUGO, tsl2510_eol_spec_show, NULL);
#endif

static struct device_attribute *tsl2510_sensor_attrs[] = {
    &dev_attr_name,
    &dev_attr_vendor,
    &dev_attr_als_flush,
    &dev_attr_int_pin_check,
    &dev_attr_read_reg,
    &dev_attr_write_reg,
    &dev_attr_als_debug,
    &dev_attr_device_id,
    &dev_attr_part_type,
    &dev_attr_i2c_err_cnt,
    &dev_attr_curr_adc,
    &dev_attr_mode_cnt,
    &dev_attr_als_factory_cmd,
    &dev_attr_als_version,
    &dev_attr_sensor_info,
    &dev_attr_als_ir,
    &dev_attr_als_clear,
    &dev_attr_als_wideband,
    &dev_attr_als_raw_data,
#ifdef CONFIG_AMS_OPTICAL_SENSOR_FLICKER
    &dev_attr_flicker_data,
#endif
#ifdef CONFIG_AMS_OPTICAL_SENSOR_EOL_MODE
    &dev_attr_eol_mode,
    &dev_attr_eol_spec,
#endif
    NULL,
};
static int _2510_handleAlsEvent(ams_deviceCtx_t *ctx);
static void _2510_handleFIFOEvent(ams_deviceCtx_t *ctx);

static void _2510_handleFIFOEvent(ams_deviceCtx_t *ctx)
{
    int err =0;
    int clear_gain;
    int wideband_gain;
    int fifo_mod0_gain;
    int fifo_mod1_gain;
    amsAlsDataSet_t inputData;
    adcDataSet_t dataSet;

    inputData.status = ALS_STATUS_RDY;
    inputData.datasetArray = (alsData_t *)&dataSet;

    ccb_FIFOEvent(ctx);

    if( ctx->flickerCtx.data_ready == 1){
        AMS_ENABLE_PON(err);
        if (err < 0) {
            ALS_err("%s - failed to AMS_ENABLE_PON\n", __func__);
        }

        ccb_sw_bin4096_flicker_GetResult(ctx);

        dataSet.AdcClear = ctx->clear_average;
        dataSet.AdcWb = ctx->wideband_average;
        fifo_mod0_gain = ctx->fifo_mod0_gain & 0x0F;//clear
        fifo_mod1_gain = (ctx->fifo_mod0_gain >> 4) & 0x0F;//wide
        clear_gain = tsl2510_gain_conversion[fifo_mod0_gain] * 500;
        wideband_gain = tsl2510_gain_conversion[fifo_mod1_gain] * 500;

        ctx->ccbAlsCtx.ctxAlgAls.ClearGain = clear_gain;
        ctx->ccbAlsCtx.ctxAlgAls.WBGain = wideband_gain;
        amsAlg_als_processData(&ctx->ccbAlsCtx.ctxAlgAls, &inputData);
        //ctx->flickerCtx.als_data_ready = 1;

        ctx->updateAvailable |= (1 << AMS_SW_FLICKER_SENSOR);        /* request from IQ team. als_data & flicker_data should be reported as quickly as possible */

        AMS_REENABLE_FD_PON(err);
        if (err < 0) {
            ALS_err("%s - failed to AMS_REENABLE_FD_PON\n", __func__);
        }

        //#endif
    } else if(ctx->flickerCtx.als_data_ready == 1) {
        ctx->updateAvailable |= (1 << AMS_AMBIENT_SENSOR);
    }
}

static int ams_devicePollingHandler(ams_deviceCtx_t *ctx)
{
    int ret = 0;
    ret = ams_getByte(ctx->portHndl, DEVREG_STATUS2, &ctx->shadowStatus2Reg);

    if (ret < 0) {
        ALS_err("%s - failed to get DEVREG_STATUS\n", __func__);
        return ret;
    }

    // _2510_handleFIFOEvent(ctx);

    if ((ctx->shadowStatus2Reg & ALS_DATA_VALID) /*|| ctx->alwaysReadAls*/) {
        ret = ams_getByte(ctx->portHndl, DEVREG_ALS_STATUS, &ctx->shadowAlsStatusReg);
        if (ret < 0) {
            ALS_err("%s - failed to get DEVREG_ALS_STATUS\n", __func__);
            return ret;
        }
        ret = ams_getByte(ctx->portHndl, DEVREG_ALS_STATUS2, &ctx->shadowAlsStatus2Reg);
        if (ret < 0) {
            ALS_err("%s - failed to get DEVREG_ALS_STATUS2\n", __func__);
            return ret;
        }
        if (ctx->mode & MODE_ALS_ALL) {
            ALS_info("%s - _2510_handleAlsEvent :%d alwaysReadAls = %d\n", __func__, (ctx->shadowStatus1Reg & AINT), ctx->alwaysReadAls);
            ret = _2510_handleAlsEvent(ctx);
            if (ret < 0) {
                ALS_err("%s - failed to _2510_handleAlsEvent\n", __func__);
                //return ret;
            }
        }
    }
    return 0;
}


static int ams_deviceEventHandler(ams_deviceCtx_t *ctx)
{
    int ret = 0;
    uint8_t status5 = 0;

    ret = ams_getByte(ctx->portHndl, DEVREG_STATUS, &ctx->shadowStatus1Reg);

    if (ret < 0) {
        ALS_err("%s - failed to get DEVREG_STATUS\n", __func__);
        return ret;
    }

    ret = ams_getByte(ctx->portHndl, DEVREG_STATUS2, &ctx->shadowStatus2Reg);
    //ret = ams_getByte(ctx->portHndl, DEVREG_MEAS_SEQR_STEP0_MOD_GAINX_L, &ctx->fifo_mod0_gain);
    //ret = ams_getByte(ctx->portHndl, DEVREG_MOD_GAIN_H, &ctx->fifo_mod1_gain);

         //clear_gain = tsl2510_gain_conversion[ctx->fifo_mod0_gain];
         //wideband_gain = tsl2510_gain_conversion[ctx->fifo_mod1_gain];


    if (ret < 0) {
        ALS_err("%s - failed to get DEVREG_STATUS\n", __func__);
        return ret;
    }

    if (ctx->shadowStatus1Reg & SINT) {
        ret = ams_getByte(ctx->portHndl, DEVREG_STATUS5, &status5);
        if (ret < 0) {
            ALS_err("%s - failed to get DEVREG_STATUS5\n", __func__);
            return ret;
        }
        ALS_info("%s - ctx->shadowStatus1Reg %x, status5 %x, mode %x", __func__, ctx->shadowStatus1Reg, status5, ctx->mode);
    }

#if 1
loop:
#endif
    ALS_info("%s - loop: DCB 0x%02x, STATUS 0x%02x, ALS_STATUS 0x%02x, ALS_STATUS2 0x%02x\n", __func__, ctx->mode, ctx->shadowStatus1Reg, ctx->shadowAlsStatusReg, ctx->shadowAlsStatus2Reg);


#ifdef CONFIG_AMS_OPTICAL_SENSOR_FLICKER
    if (ctx->shadowStatus1Reg & FINT) {
        ret = ams_getByte(ctx->portHndl, DEVREG_FIFO_STATUS0, &ctx->shadowFIFOStatusReg);
        if (ret < 0) {
            ALS_err("%s - failed to get DEVREG_FIFO_STATUS0\n", __func__);
            //return ret;
        }
        if (ctx->mode & MODE_FLICKER) {
            ALS_info("%s - _2510_handleFIFOEvent \n", __func__);
            _2510_handleFIFOEvent(ctx);
        }
    }
#endif

    //    if ((ctx->shadowStatus1Reg & AINT) /*|| ctx->alwaysReadAls*/) {
    if ((ctx->shadowStatus2Reg & ALS_DATA_VALID) /*|| ctx->alwaysReadAls*/) {
        ret = ams_getByte(ctx->portHndl, DEVREG_ALS_STATUS, &ctx->shadowAlsStatusReg);
        if (ret < 0) {
            ALS_err("%s - failed to get DEVREG_ALS_STATUS\n", __func__);
            return ret;
        }
        ret = ams_getByte(ctx->portHndl, DEVREG_ALS_STATUS2, &ctx->shadowAlsStatus2Reg);
        if (ret < 0) {
            ALS_err("%s - failed to get DEVREG_ALS_STATUS2\n", __func__);
            return ret;
        }
        if (ctx->mode & MODE_ALS_ALL) {
            ALS_info("%s - _2510_handleAlsEvent :%d alwaysReadAls = %d\n", __func__, (ctx->shadowStatus1Reg & AINT), ctx->alwaysReadAls);
            ret = _2510_handleAlsEvent(ctx);
            if (ret < 0) {
                ALS_err("%s - failed to _2510_handleAlsEvent\n", __func__);
                //return ret;
            }
        }
    }

    /* Clear Processed Interrupt */
    /* this clears interrupt(s) and STATUS5 */
    if (ctx->shadowStatus1Reg != 0) {
        /* this clears interrupt(s) and STATUS5 */
        ret = ams_setByte(ctx->portHndl, DEVREG_STATUS, ctx->shadowStatus1Reg);
        if (ret < 0) {
            ALS_err("%s - failed to set DEVREG_STATUS\n", __func__);
            return ret;
        }
    }

    if (status5 != 0) {
        ret = ams_setByte(ctx->portHndl, DEVREG_STATUS5, status5);
        if (ret < 0) {
            ALS_err("%s - failed to set DEVREG_STATUS5\n", __func__);
            return ret;
        }
    }

    /* Check Remainning Interrupt */
    ret = ams_getByte(ctx->portHndl, DEVREG_STATUS, &ctx->shadowStatus1Reg);
    if (ret < 0) {
        ALS_err("%s - failed to get DEVREG_STATUS\n", __func__);
        return ret;
    }
    if (ctx->shadowStatus1Reg != 0) {
        ALS_err("%s - goto loop", __func__);
        goto loop;
    }

    /*
     *	the individual handlers may have temporarily disabled things
     *	AMS_REENABLE(ret);
     *	if (ret < 0) {
     *		ALS_err("%s - failed to AMS_REENABLE\n", __func__);
     *		return ret;
     *	}
     */
    return ret;
}

#ifdef CONFIG_AMS_OPTICAL_SENSOR_EOL_MODE
static int tsl2510_eol_mode_handler(struct tsl2510_device_data *data)
{
    int i;
    ams_deviceCtx_t *ctx = data->deviceCtx;

    switch (data->eol_state) {
        case EOL_STATE_INIT:
            if (data->eol_result == NULL)
                data->eol_result = devm_kzalloc(&data->client->dev, MAX_TEST_RESULT, GFP_KERNEL);

            for (i = 0; i < EOL_STATE_DONE; i++) {
                data->eol_flicker_awb[i][0] = 0;
                data->eol_flicker_awb[i][1] = 0;
                data->eol_flicker_awb[i][2] = 0;
                data->eol_flicker_awb[i][3] = 0;
            }
            data->eol_count = 0;
            data->eol_awb = 0;
            data->eol_clear = 0;
            data->eol_wideband = 0;
            data->eol_flicker = 0;
            data->eol_flicker_count = 0;
            data->eol_state = EOL_STATE_100;
            data->eol_pulse_count = 0;
            break;
        default:
            data->eol_count++;
            ALS_dbg("%s - eol_state:%d, eol_cnt:%d, flk_avr:%d (sum:%d, cnt:%d), ir:%d, clear:%d, wide:%d, rawClear:%d, rawWide:%d\n", __func__,
                    data->eol_state, data->eol_count, (data->eol_flicker / data->eol_flicker_count), data->eol_flicker, data->eol_flicker_count,
                    data->eol_awb, data->eol_clear, data->eol_wideband,
                    ctx->ccbAlsCtx.ctxAlgAls.results.rawClear, ctx->ccbAlsCtx.ctxAlgAls.results.rawWideband);

            if (data->eol_count >= (EOL_COUNT + EOL_SKIP_COUNT)) {
                data->eol_flicker_awb[data->eol_state][0] = data->eol_flicker / data->eol_flicker_count;
                data->eol_flicker_awb[data->eol_state][1] = data->eol_awb / EOL_COUNT;
                data->eol_flicker_awb[data->eol_state][2] = data->eol_clear / EOL_COUNT;
                data->eol_flicker_awb[data->eol_state][3] = data->eol_wideband / EOL_COUNT;

                ALS_dbg("%s - eol_state = %d, pulse_duty[100,120] = [%d, %d], pulse_count = %d, flicker_result = %d\n",
                    __func__, data->eol_state, data->eol_pulse_duty[0], data->eol_pulse_duty[1], data->eol_pulse_count, data->eol_flicker_awb[data->eol_state][0]);

                if(data->eol_state == EOL_STATE_100 || data->eol_state == EOL_STATE_120) {
                    data->eol_flicker_pass_cnt[data->eol_state] = data->eol_flicker_count;
                }

                data->eol_count = 0;
                data->eol_awb = 0;
                data->eol_clear = 0;
                data->eol_wideband = 0;
                data->eol_flicker = 0;
                data->eol_flicker_count = 0;
                data->eol_pulse_count = 0;
                data->eol_state++;
            }
            break;
    }

    return 0;
}
#endif
irqreturn_t tsl2510_irq_handler(int dev_irq, void *device)
{
    int err;
    struct tsl2510_device_data *data = device;
    int interruptsHandled = 0;

    ALS_info("%s - als_irq = %d\n", __func__, dev_irq);

    if (data->regulator_state == 0) {
        ALS_dbg("%s - stop irq handler (reg_state : %d, enabled : %d)\n",
                __func__, data->regulator_state, data->enabled);
        return IRQ_HANDLED;
    } else if (data->enabled == 0) {
        ALS_dbg("%s - ALS not enabled, clear irq (regulator_state : %d, enabled : %d)",
                __func__, data->regulator_state, data->enabled);
        ams_setByte(data->client, DEVREG_STATUS, AMS_ALL_INT);
        return IRQ_HANDLED;
    }

    //mutex_lock(&data->activelock);

    err = ams_deviceEventHandler(data->deviceCtx);
    interruptsHandled = ams_getResult(data->deviceCtx);

    if (err == 0) {
        if (data->als_input_dev == NULL) {
            ALS_err("%s - als_input_dev is NULL\n", __func__);
        }
        else {
#ifdef CONFIG_AMS_OPTICAL_SENSOR_ALS
            if (interruptsHandled & (1 << AMS_AMBIENT_SENSOR)) {
                report_als(data);
#ifdef CONFIG_AMS_OPTICAL_SENSOR_EOL_MODE
                if (data->eol_enable)
                    tsl2510_eol_mode_handler(data);
#endif
            }
#endif

#ifdef CONFIG_AMS_OPTICAL_SENSOR_FLICKER
            if (interruptsHandled & (1 << AMS_FLICKER_SENSOR))
                report_flicker(data);

            if (interruptsHandled & (1 << AMS_SW_FLICKER_SENSOR)) {
                report_als(data);
#ifdef CONFIG_AMS_OPTICAL_SENSOR_EOL_MODE
                if (data->eol_enable)
                    tsl2510_eol_mode_handler(data);
#endif
                msleep_interruptible(30);
                /* HwModuleTest need this delay to distinguish ALS / Flicker */
                report_flicker(data);
            }
#endif
        }
    }
    else {
        ALS_err("%s - ams_deviceEventHandler failed\n", __func__);
    }

    //mutex_unlock(&data->activelock);

    return IRQ_HANDLED;
}

#ifdef CONFIG_AMS_OPTICAL_SENSOR_POLLING
static void  tsl2510_work_func_light(struct work_struct *work)
{
    int err;
    //    int lux;
    int pollingHandled = 0;

    struct tsl2510_device_data *data
        = container_of(work, struct tsl2510_device_data, work_light);
    ALS_info("%s -msec \n", __func__);

    // testing
    //return 0;
    if (data->regulator_state == 0 || data->enabled == 0) {
        ALS_dbg("%s - stop irq handler (reg_state : %d, enabled : %d)\n",
                __func__, data->regulator_state, data->enabled);

        ams_setByte(data->client, DEVREG_STATUS, (AINT | AMS_ALL_INT));
    }
    mutex_lock(&data->flickerdatalock);

    err = ams_devicePollingHandler(data->deviceCtx);
    pollingHandled = ams_getResult(data->deviceCtx);

    if (err == 0) {
        if (data->als_input_dev == NULL) {
            ALS_err("%s - als_input_dev is NULL\n", __func__);
        } else {
#ifdef CONFIG_AMS_OPTICAL_SENSOR_ALS
            if (pollingHandled & (1 << AMS_AMBIENT_SENSOR)) {
                report_als(data);
#ifdef CONFIG_AMS_OPTICAL_SENSOR_EOL_MODE
                if (data->eol_enable)
                    tsl2510_eol_mode_handler(data);
#endif
            }
#endif

#ifdef CONFIG_AMS_OPTICAL_SENSOR_FLICKER
            if (pollingHandled & (1 << AMS_FLICKER_SENSOR))
                report_flicker(data);
#endif
        }
    } else {
        ALS_err("%s - ams_deviceEventHandler failed\n", __func__);
    }
    mutex_unlock(&data->flickerdatalock);
}

static enum hrtimer_restart tsl2510_timer_func(struct hrtimer *timer)
{
    struct tsl2510_device_data *data = container_of(timer, struct tsl2510_device_data, timer);
    queue_work(data->wq, &data->work_light);
    hrtimer_forward_now(&data->timer, data->light_poll_delay);
    return HRTIMER_RESTART;
}
#endif
static int tsl2510_setup_irq(struct tsl2510_device_data *data)
{
    int errorno = -EIO;
    errorno = request_threaded_irq(data->dev_irq, NULL,
            tsl2510_irq_handler, IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
            "als_rear_sensor_irq", data);

    if (errorno < 0) {
        ALS_err("%s - failed for setup dev_irq errono= %d\n",
                __func__, errorno);
        errorno = -ENODEV;
        return errorno;
    }

    disable_irq(data->dev_irq);

    return errorno;
}

static void tsl2510_init_var(struct tsl2510_device_data *data)
{
    data->client = NULL;
    data->dev = NULL;
    data->als_input_dev = NULL;
    data->als_pinctrl = NULL;
    data->pins_sleep = NULL;
    data->pins_idle = NULL;
    data->i2c_1p8 = false;
    data->enabled = 0;
    data->sampling_period_ns = 0;
#ifndef AMS_BUILD
    data->regulator_state = 0;
#else
    data->regulator_state = 1;
#endif
    data->irq_state = 0;
    data->suspend_cnt = 0;
    data->reg_read_buf = 0;
    data->pm_state = PM_RESUME;
    data->i2c_err_cnt = 0;
    data->user_ir_data = 0;
    data->user_flicker_data = 0;
#ifdef CONFIG_AMS_OPTICAL_SENSOR_EOL_MODE
    data->eol_pulse_duty[0] = DEFAULT_DUTY_50HZ;
    data->eol_pulse_duty[1] = DEFAULT_DUTY_60HZ;
    data->eol_pulse_count = 0;
#endif
    data->awb_sample_cnt = 0;
    data->flicker_data_cnt = 0;
    //flicker_data_cnt = 0;
    data->saturation = false;
}

static int tsl2510_parse_dt(struct tsl2510_device_data *data)
{
    struct device *dev = &data->client->dev;
    struct device_node *dNode = dev->of_node;
#ifdef CONFIG_AMS_OPTICAL_SENSOR_EOL_MODE
    enum of_gpio_flags flags;
#endif
#ifdef CONFIG_TCS3407
    u32 gain_max = 0;
#endif
#ifdef CONFIG_OF
    if (dNode == NULL)
        return -ENODEV;

    data->pin_als_int = of_get_named_gpio(dNode, "als_rear,int-gpio", 0);
    if (data->pin_als_int < 0) {
        ALS_err("%s - get als_rear_int error\n", __func__);
        return -ENODEV;
    }

    if (of_get_property(dNode, "als_rear,i2c_1p8-control", NULL)) {
        data->regulator_i2c_1p8 = regulator_get(&data->client->dev, "i2c_1p8");
        if (IS_ERR(data->regulator_i2c_1p8) || data->regulator_i2c_1p8 == NULL) {
            ALS_err("%s - get i2c_1p8 regulator failed\n", __func__);
            data->regulator_i2c_1p8 = NULL;
        } else {
            data->i2c_1p8 = true;
        }
    }

    data->pin_als_en = of_get_named_gpio(dNode, "als_rear,als_en-gpio", 0);
    if (data->pin_als_en < 0) {
        ALS_dbg("%s - use VDD_REGULATOR\n", __func__);

        data->regulator_vdd_1p8 = regulator_get(&data->client->dev, "vdd_1p8");
        if (IS_ERR(data->regulator_vdd_1p8) || data->regulator_vdd_1p8 == NULL) {
            ALS_dbg("%s - get vdd_1p8 regulator failed\n", __func__);
            data->regulator_vdd_1p8 = NULL;
        }

        data->power_type = VDD_REGULATOR;
    } else {
        ALS_dbg("%s - use VDD_GPIO\n", __func__);
        data->power_type = VDD_GPIO;
    }

#ifdef CONFIG_AMS_OPTICAL_SENSOR_EOL_MODE
    data->pin_torch_en = of_get_named_gpio_flags(dNode,
            "als_rear,led_en-gpio", 0, &flags);
    if (data->pin_torch_en < 0) {
        ALS_err("%s - get pin_torch_en error\n", __func__);
        return -ENODEV;
    }

    data->pin_flash_en = of_get_named_gpio_flags(dNode,
            "als_rear,flash_en-gpio", 0, &flags);
    if (data->pin_flash_en < 0) {
        ALS_err("%s - get pin_flash_en error\n", __func__);
    }
#endif

#endif 	/* CONFIG_OF */
    data->als_pinctrl = devm_pinctrl_get(dev);

    if (IS_ERR_OR_NULL(data->als_pinctrl)) {
        ALS_err("%s - get pinctrl(%li) error\n",
                __func__, PTR_ERR(data->als_pinctrl));
        data->als_pinctrl = NULL;
        return -EINVAL;
    }

    data->pins_sleep =
        pinctrl_lookup_state(data->als_pinctrl, "sleep");
    if (IS_ERR_OR_NULL(data->pins_sleep)) {
        ALS_err("%s - get pins_sleep(%li) error\n",
                __func__, PTR_ERR(data->pins_sleep));
        devm_pinctrl_put(data->als_pinctrl);
        data->pins_sleep = NULL;
        return -EINVAL;
    }

    data->pins_idle =
        pinctrl_lookup_state(data->als_pinctrl, "idle");
    if (IS_ERR_OR_NULL(data->pins_idle)) {
        ALS_err("%s - get pins_idle(%li) error\n",
                __func__, PTR_ERR(data->pins_idle));

        devm_pinctrl_put(data->als_pinctrl);
        data->pins_idle = NULL;
        return -EINVAL;
    }
#ifdef CONFIG_TCS3407 //CONFIG_OF
    if (of_property_read_u32(dNode, "als_rear,gain_max", &gain_max) == 0) {
        deviceRegisterDefinition[DEVREG_AGC_GAIN_MAX].resetValue = gain_max;

        ALS_dbg("%s - DEVREG_AGC_GAIN_MAX = 0x%x\n", __func__, gain_max);
    }
#endif
#ifdef CONFIG_AMS_OPTICAL_SENSOR_EOL_MODE
    if (of_property_read_u32_array(dNode, "als_rear,ir_spec",
                data->eol_ir_spec, ARRAY_SIZE(data->eol_ir_spec)) < 0) {
        ALS_err("%s - get ir_spec error\n", __func__);

        data->eol_ir_spec[0] = DEFAULT_IR_SPEC_MIN;
        data->eol_ir_spec[1] = DEFAULT_IR_SPEC_MAX;
        data->eol_ir_spec[2] = DEFAULT_IR_SPEC_MIN;
        data->eol_ir_spec[3] = DEFAULT_IR_SPEC_MAX;
    }
    ALS_dbg("%s - ir_spec = %d, %d, %d, %d\n", __func__,
            data->eol_ir_spec[0], data->eol_ir_spec[1], data->eol_ir_spec[2], data->eol_ir_spec[3]);

    if (of_property_read_u32_array(dNode, "als_rear,clear_spec",
                data->eol_clear_spec, ARRAY_SIZE(data->eol_clear_spec)) < 0) {
        ALS_err("%s - get clear_spec error\n", __func__);

        data->eol_clear_spec[0] = DEFAULT_IR_SPEC_MIN;
        data->eol_clear_spec[1] = DEFAULT_IR_SPEC_MAX;
        data->eol_clear_spec[2] = DEFAULT_IR_SPEC_MIN;
        data->eol_clear_spec[3] = DEFAULT_IR_SPEC_MAX;
    }
    ALS_dbg("%s - clear_spec = %d, %d, %d, %d\n", __func__,
            data->eol_clear_spec[0], data->eol_clear_spec[1], data->eol_clear_spec[2], data->eol_clear_spec[3]);

    if (of_property_read_u32_array(dNode, "als_rear,icratio_spec",
                data->eol_icratio_spec, ARRAY_SIZE(data->eol_icratio_spec)) < 0) {
        ALS_err("%s - get icratio_spec error\n", __func__);

        data->eol_icratio_spec[0] = DEFAULT_IC_SPEC_MIN;
        data->eol_icratio_spec[1] = DEFAULT_IC_SPEC_MIN;
        data->eol_icratio_spec[2] = DEFAULT_IC_SPEC_MIN;
        data->eol_icratio_spec[3] = DEFAULT_IC_SPEC_MIN;
    }
    ALS_dbg("%s - icratio_spec = %d, %d, %d, %d\n", __func__,
            data->eol_icratio_spec[0], data->eol_icratio_spec[1], data->eol_icratio_spec[2], data->eol_icratio_spec[3]);
#endif

    ALS_dbg("%s - done.\n", __func__);

    return 0;
}

static int tsl2510_setup_gpio(struct tsl2510_device_data *data)
{
    int errorno = -EIO;

    errorno = gpio_request(data->pin_als_int, "als_rear_int");
    if (errorno) {
        ALS_err("%s - failed to request als_int\n", __func__);
        return errorno;
    }

    errorno = gpio_direction_input(data->pin_als_int);
    if (errorno) {
        ALS_err("%s - failed to set als_int as input\n", __func__);
        goto err_gpio_direction_input;
    }
    data->dev_irq = gpio_to_irq(data->pin_als_int);

    if (data->pin_als_en >= 0) {
        errorno = gpio_request(data->pin_als_en, "als_rear_en");
        if (errorno) {
            ALS_err("%s - failed to request als_en\n", __func__);
            goto err_gpio_direction_input;
        }
    }

    goto done;

err_gpio_direction_input:
    gpio_free(data->pin_als_int);
done:
    return errorno;
}
#if 0
static int _2510_resetAllRegisters(AMS_PORT_portHndl *portHndl)
{
    int err = 0;
    /*
     *	ams_deviceRegister_t i;
     *
     *	for (i = DEVREG_ENABLE; i <= DEVREG_CFG1; i++) {
     *		ams_setByte(portHndl, i, deviceRegisterDefinition[i].resetValue);
     *	}
     *	for (i = DEVREG_STATUS; i < DEVREG_REG_MAX; i++) {
     *		ams_setByte(portHndl, i, deviceRegisterDefinition[i].resetValue);
     *	}
     */
    // To prevent SIDE EFFECT , below register should be written
    err = ams_setByte(portHndl, DEVREG_CFG6, deviceRegisterDefinition[DEVREG_CFG6].resetValue);
    if (err < 0) {
        ALS_err("%s - failed to set DEVREG_CFG6\n", __func__);
        return err;
    }
    err = ams_setByte(portHndl, DEVREG_AGC_GAIN_MAX, deviceRegisterDefinition[DEVREG_AGC_GAIN_MAX].resetValue);
    if (err < 0) {
        ALS_err("%s - failed to set DEVREG_AGC_GAIN_MAX\n", __func__);
        return err;
    }

    err = ams_setByte(portHndl, DEVREG_FD_CFG3, deviceRegisterDefinition[DEVREG_FD_CFG3].resetValue);
    if (err < 0) {
        ALS_err("%s - failed to set DEVREG_FD_CFG3\n", __func__);
        return err;
    }

    return err;
}
#endif
//#ifdef CONFIG_AMS_OPTICAL_SENSOR_ALS_CCB
static int _2510_alsInit(ams_deviceCtx_t *ctx, ams_calibrationData_t *calibrationData)
{
    int ret = 0;

    if (calibrationData == NULL) {
        ams_ccb_als_info_t infoData;

        ALS_info("%s - calibrationData is null\n", __func__);
        ccb_alsInfo(&infoData);
        ctx->ccbAlsCtx.initData.calibrationData.calibrationFactor = infoData.defaultCalibrationData.calibrationFactor;
        ctx->ccbAlsCtx.initData.calibrationData.Time_base = infoData.defaultCalibrationData.Time_base;
        ctx->ccbAlsCtx.initData.calibrationData.thresholdLow = infoData.defaultCalibrationData.thresholdLow;
        ctx->ccbAlsCtx.initData.calibrationData.thresholdHigh = infoData.defaultCalibrationData.thresholdHigh;
        ctx->ccbAlsCtx.initData.calibrationData.calibrationFactor = infoData.defaultCalibrationData.calibrationFactor;
    }
    else {
        ALS_info("%s - calibrationData is non-null\n", __func__);
        //ctx->ccbAlsCtx.initData.calibrationData.luxTarget = calibrationData->alsCalibrationLuxTarget;
        //ctx->ccbAlsCtx.initData.calibrationData.luxTargetError = calibrationData->alsCalibrationLuxTargetError;
        ctx->ccbAlsCtx.initData.calibrationData.calibrationFactor = calibrationData->alsCalibrationFactor;
        ctx->ccbAlsCtx.initData.calibrationData.Time_base = calibrationData->timeBase_us;
        ctx->ccbAlsCtx.initData.calibrationData.thresholdLow = calibrationData->alsThresholdLow;
        ctx->ccbAlsCtx.initData.calibrationData.thresholdHigh = calibrationData->alsThresholdHigh;
    }
    ctx->ccbAlsCtx.initData.calibrate = false;
    //ctx->ccbAlsCtx.initData.configData.gain = 64000;//AGAIN
    ctx->ccbAlsCtx.initData.configData.gain = 16000;//AGAIN
    ctx->ccbAlsCtx.initData.configData.uSecTime = AMS_ALS_ATIME; /*ALS Inegration time 50msec*/

    ctx->alwaysReadAls = false;
    ctx->alwaysReadFlicker = false;
    ctx->ccbAlsCtx.initData.autoGain = true; //AutoGainCtrol on
    ctx->ccbAlsCtx.initData.hysteresis = 0x02; /*Lower threshold for adata in AGC */
    return ret;
}

static bool ams_deviceGetAls(ams_deviceCtx_t *ctx, ams_apiAls_t *exportData)
{
    ams_ccb_als_result_t result;

    ccb_alsGetResult(ctx, &result);
    exportData->clear = result.clear;
    exportData->ir = result.ir;
    exportData->time_us = result.time_us;
    exportData->ClearGain = result.ClearGain;
    exportData->WBGain = result.WBGain;

    exportData->wideband    = result.wideband;
    exportData->rawClear = result.rawClear;
    exportData->rawWideband = result.rawWideband;
    return false;
}

static int _2510_handleAlsEvent(ams_deviceCtx_t *ctx)
{
    int ret = 0;
    ams_ccb_als_dataSet_t ccbAlsData;

    ccbAlsData.statusReg = ctx->shadowStatus1Reg;
    ccbAlsData.status2Reg = ctx->shadowStatus2Reg;
    ccbAlsData.alsstatusReg = ctx->shadowAlsStatusReg;
    ccbAlsData.alsstatus2Reg = ctx->shadowAlsStatus2Reg;

    ret = ccb_alsHandle(ctx, &ccbAlsData);

    return ret;
}

//#endif


static bool ams_deviceGetFlicker(ams_deviceCtx_t *ctx, ams_apiAlsFlicker_t *exportData)
{
    ams_flicker_ctx_t *flickerCtx = (ams_flicker_ctx_t *)&ctx->flickerCtx;

    exportData->mHz = flickerCtx->frequency;
    return false;
}

static int ams_deviceSoftReset(ams_deviceCtx_t *ctx)
{
    int err = 0;

    ALS_dbg("%s - Start\n", __func__);

    // Before S/W reset, the PON has to be asserted
    err = ams_setByte(ctx->portHndl, DEVREG_ENABLE, PON);
    if (err < 0) {
        ALS_err("%s - failed to set DEVREG_ENABLE\n", __func__);
        return err;
    }

    err = ams_setField(ctx->portHndl, DEVREG_CONTROL, HIGH, MASK_SOFT_RESET);
    if (err < 0) {
        ALS_err("%s - failed to set DEVREG_SOFT_RESET\n", __func__);
        return err;
    }
    // Need 1 msec delay
    usleep_range(1000, 1100);

    // Recover the previous enable setting
    err = ams_setByte(ctx->portHndl, DEVREG_ENABLE, ctx->shadowEnableReg);
    if (err < 0) {
        ALS_err("%s - failed to set DEVREG_ENABLE\n", __func__);
        return err;
    }

    return err;
}

static ams_deviceIdentifier_e ams_validateDevice(AMS_PORT_portHndl *portHndl)
{
    uint8_t chipId;
    uint8_t revId;
    uint8_t auxId;
    uint8_t i = 0;
    int err = 0;

    struct tsl2510_device_data *data = i2c_get_clientdata(portHndl);

    err = ams_getByte(portHndl, DEVREG_ID, &chipId);
    if (err < 0) {
        ALS_err("%s - failed to get DEVREG_ID\n", __func__);
        return AMS_UNKNOWN_DEVICE;
    }
    err = ams_getByte(portHndl, DEVREG_REVID, &revId);
    if (err < 0) {
        ALS_err("%s - failed to get DEVREG_REVID\n", __func__);
        return AMS_UNKNOWN_DEVICE;
    }
    err = ams_getByte(portHndl, DEVREG_AUXID, &auxId);
    if (err < 0) {
        ALS_err("%s - failed to get DEVREG_AUXID\n", __func__);
        return AMS_UNKNOWN_DEVICE;
    }
    ALS_dbg("%s - ID:0x%02x, revID:0x%02x, auxID:0x%02x\n", __func__, chipId, revId, auxId);

    if ((revId & 0x10) == 0x10)
        data->isTrimmed = 1;
    else
        data->isTrimmed = 0;

    do {
        if (((chipId & deviceIdentifier[i].deviceIdMask) ==
                    (deviceIdentifier[i].deviceId & deviceIdentifier[i].deviceIdMask)) &&
                ((revId & deviceIdentifier[i].deviceRefMask) >=
                 (deviceIdentifier[i].deviceRef & deviceIdentifier[i].deviceRefMask))) {
            return deviceIdentifier[i].device;
        }
        i++;
    } while (deviceIdentifier[i].device != AMS_LAST_DEVICE);

    return AMS_UNKNOWN_DEVICE;
}

static int ams_deviceInit(ams_deviceCtx_t *ctx, AMS_PORT_portHndl *portHndl, ams_calibrationData_t *calibrationData)
{
    int ret = 0;

    ctx->portHndl = portHndl;
    ctx->mode = MODE_OFF;
    ctx->systemCalibrationData = calibrationData;
    ctx->deviceId = ams_validateDevice(ctx->portHndl);
    ctx->shadowEnableReg = deviceRegisterDefinition[DEVREG_ENABLE].resetValue;
    ctx->agc = true;

    ret = ams_deviceSoftReset(ctx);
    if (ret < 0) {
        ALS_err("%s - failed to ams_deviceSoftReset\n", __func__);
        return ret;
    }

    AMS_SET_SAMPLE_TIME(AMS_SAMPLING_TIME, ret);
    if (ret < 0) {
        ALS_err("%s - failed to Sample time \n", __func__);
        return ret;
    }

    if (ctx->agc)
    {

        // AGC ASAT MODE
        AMS_AGC_ASAT_MODE(HIGH, ret);
        if (ret < 0) {
            ALS_err("%s - failed to set AGC ASAT MODE\n", __func__);
            return ret;
        }
        // AGC PREDIC  MODE
        AMS_AGC_PREDICT_MODE(HIGH, ret);
        if (ret < 0) {
            ALS_err("%s - failed to set AGC PREDIC  MODE\n", __func__);
            return ret;
        }

        AMS_AGC_ENABLE(ret);
        if (ret < 0) {
            ALS_err("%s - failed to set AGC ENABLE\n", __func__);
            return ret;
        }


        // SET AGC MAX GAIN
        AMS_SET_AGC_MAX_GAIN(AMS_AGC_MAX_GAIN, ret);
        if (ret < 0) {
            ALS_err("%s - failed to set SET AGC MAX GAIN\n", __func__);
            return ret;
        }

        // AGC Number of samples
        AMS_SET_AGC_NR_SAMPLES(AMS_AGC_NUM_SAMPLES, ret);
        if (ret < 0) {
            ALS_err("%s - failed to set AGC Number of samples\n", __func__);
            return ret;
        }
        //ams_setByte(ctx->portHndl, DEVREG_AGC_NR_SAMPLES_LO, 0x13);

    }

#ifdef CONFIG_AMS_OPTICAL_SENSOR_ALS_CCB
    ret = _2510_alsInit(ctx, calibrationData);
    if (ret < 0) {
        ALS_err("%s - failed to _2510_alsInit\n", __func__);
        return ret;
    }
#endif

    return ret;
}

static bool ams_getDeviceInfo(ams_deviceInfo_t *info, ams_deviceIdentifier_e deviceId)
{
    memset(info, 0, sizeof(ams_deviceInfo_t));

    info->defaultCalibrationData.timeBase_us = AMS_USEC_PER_TICK;
    info->numberOfSubSensors = 0;
    info->memorySize = sizeof(ams_deviceCtx_t);

    switch (deviceId) {
        case AMS_TSL2510:
        case AMS_TSL2510_UNTRIM:
            info->deviceModel = "TSL2510";
            break;
            //	case AMS_TCS3408:
            //	case AMS_TCS3408_UNTRIM:
            //		info->deviceModel = "TCS3408";
            //		break;
        default:
            info->deviceModel = "UNKNOWN";
            break;
    }

    memcpy(info->defaultCalibrationData.deviceName, info->deviceModel, sizeof(info->defaultCalibrationData.deviceName));
    info->deviceName = "ALS/PRX/FLKR";
    info->driverVersion = "Alpha";
#ifdef CONFIG_AMS_OPTICAL_SENSOR_ALS_CCB
    {
        /* TODO */
        ams_ccb_als_info_t infoData;

        ccb_alsInfo(&infoData);
        info->tableSubSensors[info->numberOfSubSensors] = AMS_AMBIENT_SENSOR;
        info->numberOfSubSensors++;

        info->alsSensor.driverName = infoData.algName;
        info->alsSensor.adcBits = 8;
        info->alsSensor.maxPolRate = 50;
        info->alsSensor.activeCurrent_uA = 100;
        info->alsSensor.standbyCurrent_uA = 5;
        info->alsSensor.rangeMax = 1;
        info->alsSensor.rangeMin = 0;

        info->defaultCalibrationData.alsCalibrationFactor = infoData.defaultCalibrationData.calibrationFactor;
        //		info->defaultCalibrationData.alsCalibrationLuxTarget = infoData.defaultCalibrationData.luxTarget;
        //		info->defaultCalibrationData.alsCalibrationLuxTargetError = infoData.defaultCalibrationData.luxTargetError;
#if defined(CONFIG_AMS_ALS_CRWBI) || defined(CONFIG_AMS_ALS_CRGBW)
        info->tableSubSensors[info->numberOfSubSensors] = AMS_WIDEBAND_ALS_SENSOR;
        info->numberOfSubSensors++;
#endif
    }
#endif
    return false;
}


static void fifo_work_cb(struct work_struct *work)
{
    struct fifo_chip *fifo = NULL;

    if (work) {
        fifo = container_of(work, struct fifo_chip, work);
    }
}

static int tsl2510_init_fifo(struct tsl2510_device_data *data)
{
    static struct fifo_chip fifo;

    memset(&fifo, 0, sizeof(struct fifo_chip));

    //fifo.callbacks.irq = fifo_irq;
    //data->fifo = &fifo.work;
    fifo.data = data;
    init_waitqueue_head(&data->fifo_wait);
    //if (init_fifo_device_tree(chip)) {
    //fifo_reset(chip);
    INIT_WORK(&fifo.work, fifo_work_cb);
    INIT_KFIFO(ams_fifo);
    //enable_fifo(chip, FIFO_OFF);
    //if (sysfs_create_groups(&chip->input->dev.kobj, fifo_groups)) {
    //    dev_err(&chip->input->dev, "Error creating sysfs attribute group.\n");
    //}
    //}
    return 0;
}

int tsl2510_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int err = -ENODEV;
    struct device *dev = &client->dev;
    static struct tsl2510_device_data *data;
    struct amsdriver_i2c_platform_data *pdata = dev->platform_data;
    ams_deviceInfo_t amsDeviceInfo;
    ams_deviceIdentifier_e deviceId;

    ALS_dbg("%s - start\n", __func__);
    printk(KERN_ERR "\nams_tsl2510: tsl2510_probe() client->irq= %d\n", client->irq);

    /* check to make sure that the adapter supports I2C */
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        ALS_err("%s - I2C_FUNC_I2C not supported\n", __func__);
        return -ENODEV;
    }
    /* allocate some memory for the device */
    data = devm_kzalloc(dev, sizeof(struct tsl2510_device_data), GFP_KERNEL);
    if (data == NULL) {
        ALS_err("%s - couldn't allocate device data memory\n", __func__);
        return -ENOMEM;
    }

    data->flicker_data = devm_kzalloc(dev, sizeof(int16_t)*AMS_FFT_SIZE, GFP_KERNEL);
    if (data == NULL) {
        ALS_err("%s - couldn't allocate device flicker_data memory\n", __func__);
        return -ENOMEM;
    }

    tsl2510_data = data;
    tsl2510_init_var(data);

    if (!pdata) {
        pdata = devm_kzalloc(dev, sizeof(struct amsdriver_i2c_platform_data),
                GFP_KERNEL);
        if (pdata == NULL) {
            ALS_err("%s - couldn't allocate device pdata memory\n", __func__);
            goto err_malloc_pdata;
        }
#ifdef CONFIG_OF
        if (of_match_device(tsl2510_match_table, &client->dev))
            pdata->of_node = client->dev.of_node;
#endif
    }

    data->client = client;
    data->miscdev.minor = MISC_DYNAMIC_MINOR;
    data->miscdev.name = MODULE_NAME_ALS;
    data->miscdev.fops = &tsl2510_fops;
    data->miscdev.mode = S_IRUGO;
    data->pdata = pdata;
    i2c_set_clientdata(client, data);
    ALS_info("%s client = %p\n", __func__, client);

    err = misc_register(&data->miscdev);
    if (err < 0) {
        ALS_err("%s - failed to misc device register\n", __func__);
        goto err_misc_register;
    }
    mutex_init(&data->i2clock);
    mutex_init(&data->activelock);
    mutex_init(&data->suspendlock);
    mutex_init(&data->flickerdatalock);

    err = tsl2510_parse_dt(data);
    if (err < 0) {
        ALS_err("%s - failed to parse dt\n", __func__);
        err = -ENODEV;
        goto err_parse_dt;
    }
#ifndef AMS_BUILD
    err = tsl2510_setup_gpio(data);
    if (err) {
        ALS_err("%s - failed to setup gpio\n", __func__);
        goto err_setup_gpio;
    }
#else
    data->dev_irq = client->irq;
#endif
    err = tsl2510_power_ctrl(data, PWR_ON);
    if (err < 0) {
        ALS_err("%s - failed to power on ctrl\n", __func__);
        goto err_power_on;
    }
    if (data->client->addr == TSL2510_SLAVE_I2C_ADDR_REVID_V0) {
        ALS_dbg("%s - slave address is REVID_V0\n", __func__);
    }
    else if (data->client->addr == TSL2510_SLAVE_I2C_ADDR_REVID_V1) {
        ALS_dbg("%s - slave address is REVID_V1\n", __func__);
    }
    else {
        err = -EIO;
        ALS_err("%s - slave address error, 0x%02x\n", __func__, data->client->addr);
        goto err_init_fail;
    }

    /********************************************************************/
    /* Validate the appropriate ams device is available for this driver */
    /********************************************************************/
    deviceId = ams_validateDevice(data->client);

    if (deviceId == AMS_UNKNOWN_DEVICE) {
        ALS_err("%s - ams_validateDevice failed: AMS_UNKNOWN_DEVICE\n", __func__);
        err = -EIO;
        goto err_id_failed;
    }
    ALS_dbg("%s - deviceId: %d\n", __func__, deviceId);

    ams_getDeviceInfo(&amsDeviceInfo, deviceId);
    ALS_dbg("%s - name: %s, model: %s, driver ver:%s\n", __func__,
            amsDeviceInfo.deviceName, amsDeviceInfo.deviceModel, amsDeviceInfo.driverVersion);

    data->deviceCtx = devm_kzalloc(dev, amsDeviceInfo.memorySize, GFP_KERNEL);
    if (data->deviceCtx == NULL) {
        ALS_err("%s - couldn't allocate device deviceCtx memory\n", __func__);
        err = -ENOMEM;
        goto err_malloc_deviceCtx;
    }

    err = ams_deviceInit(data->deviceCtx, data->client, NULL);
    if (err < 0) {
        ALS_err("%s - ams_deviceInit failed.\n", __func__);
        goto err_id_failed;
    }
    else {
        ALS_dbg("%s - ams_amsDeviceInit ok\n", __func__);
    }

    /*
     * S-MUX Read/Write
     * 1  read configuration to ram Read smux configuration to RAM from smux chain
     * 2  write configuration from ram Write smux configuration from RAM to smux chain
     */
    //	ams_smux_set(data->deviceCtx);

    data->sensor_mode = 0;
    data->als_input_dev = input_allocate_device();
    if (!data->als_input_dev) {
        ALS_err("%s - could not allocate input device\n", __func__);
        err = -EIO;
        goto err_input_allocate_device;
    }
    data->als_input_dev->name = MODULE_NAME_ALS;
    data->als_input_dev->id.bustype = BUS_I2C;
    input_set_drvdata(data->als_input_dev, data);
    input_set_capability(data->als_input_dev, EV_REL, REL_X);
    input_set_capability(data->als_input_dev, EV_REL, REL_Y);
    input_set_capability(data->als_input_dev, EV_REL, REL_Z);
    input_set_capability(data->als_input_dev, EV_REL, REL_RX);
    input_set_capability(data->als_input_dev, EV_REL, REL_RY);
    input_set_capability(data->als_input_dev, EV_REL, REL_RZ);
    input_set_capability(data->als_input_dev, EV_REL, REL_MISC);
    input_set_capability(data->als_input_dev, EV_ABS, ABS_X);
    input_set_capability(data->als_input_dev, EV_ABS, ABS_Y);
    input_set_capability(data->als_input_dev, EV_ABS, ABS_Z);

    err = input_register_device(data->als_input_dev);
    if (err < 0) {
        input_free_device(data->als_input_dev);
        ALS_err("%s - could not register input device\n", __func__);
        goto err_input_register_device;
    }
#ifndef AMS_BUILD
    err = sensors_create_symlink(data->als_input_dev);
#endif

#ifndef AMS_BUILD
    if (err < 0) {
        ALS_err("%s - could not create_symlink\n", __func__);
        goto err_sensors_create_symlink;
    }
#endif
    err = sysfs_create_group(&data->als_input_dev->dev.kobj,
            &als_attribute_group);
    if (err) {
        ALS_err("%s - could not create sysfs group\n", __func__);
        goto err_sysfs_create_group;
    }
#ifndef AMS_BUILD
    err = sensors_register(&data->sensor_dev, data, tsl2510_sensor_attrs,
            MODULE_NAME_ALS);
#endif

    if (err) {
        ALS_err("%s - cound not register als_sensor(%d).\n", __func__, err);
        goto als_sensor_register_failed;
    }

    err = tsl2510_setup_irq(data);
    if (err) {
        ALS_err("%s - could not setup dev_irq\n", __func__);
        goto err_setup_irq;
    }

#ifdef CONFIG_AMS_OPTICAL_SENSOR_POLLING
    INIT_WORK(&data->work_light, tsl2510_work_func_light);

    hrtimer_init(&data->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    //data->light_poll_delay = ns_to_ktime(64 * NSEC_PER_MSEC);  //60 msec Polling time
    // data->light_poll_delay = ns_to_ktime(32 * NSEC_PER_MSEC);  //20 msec Polling time
    data->light_poll_delay = ns_to_ktime(200 * NSEC_PER_MSEC);  //als only mode need to 200 msec Polling time
    data->timer.function = tsl2510_timer_func;

    data->wq = create_singlethread_workqueue("tsl2510_wq");
    if (!data->wq) {
        //ret = -ENOMEM;
        ALS_err("%s: could not create workqueue\n", __func__);
        // HAVE TO GO TO ERROR CASED
        //goto init_failed;
    }
#endif  //CONFIG_AMS_OPTICAL_SENSOR_POLLING


    err = tsl2510_init_fifo(data);

    if (err) {
        ALS_err("%s - error tsl2510_init_fifo\n", __func__);
    }

    tsl2510_irq_set_state(data, PWR_ON); /* For flushing interrupt request */
    err = tsl2510_power_ctrl(data, PWR_OFF);
    if (err < 0) {
        ALS_err("%s - failed to power off ctrl\n", __func__);
        goto dev_set_drvdata_failed;
    }

    tsl2510_irq_set_state(data, PWR_OFF);
    ALS_dbg("%s - success\n", __func__);
    goto done;

dev_set_drvdata_failed:
    free_irq(data->dev_irq, data);
err_setup_irq:
#ifndef AMS_BUILD
    sensors_unregister(data->sensor_dev, tsl2510_sensor_attrs);
#endif
als_sensor_register_failed:
    sysfs_remove_group(&data->als_input_dev->dev.kobj,
            &als_attribute_group);
err_sysfs_create_group:
#ifndef AMS_BUILD
    sensors_remove_symlink(data->als_input_dev);
#endif
err_sensors_create_symlink:
    input_unregister_device(data->als_input_dev);
err_input_register_device:
err_input_allocate_device:
err_id_failed:
    //	devm_kfree(data->deviceCtx);
err_malloc_deviceCtx:
err_init_fail:
    tsl2510_power_ctrl(data, PWR_OFF);
err_power_on:
    gpio_free(data->pin_als_int);
    if (data->pin_als_en >= 0)
        gpio_free(data->pin_als_en);
err_setup_gpio:
err_parse_dt:
    //	devm_kfree(pdata);
err_malloc_pdata:

#ifdef CONFIG_AMS_OPTICAL_SENSOR_EOL_MODE
    if (data->als_pinctrl) {
        devm_pinctrl_put(data->als_pinctrl);
        data->als_pinctrl = NULL;
    }
    if (data->pins_idle)
        data->pins_idle = NULL;
    if (data->pins_sleep)
        data->pins_sleep = NULL;
#endif
    mutex_destroy(&data->i2clock);
    mutex_destroy(&data->activelock);
    mutex_destroy(&data->suspendlock);
    mutex_destroy(&data->flickerdatalock);
    misc_deregister(&data->miscdev);
err_misc_register:
    //	devm_kfree(data);
    ALS_err("%s failed\n", __func__);
done:
    return err;
}

int tsl2510_remove(struct i2c_client *client)
{
    struct tsl2510_device_data *data = i2c_get_clientdata(client);
    //struct fifo_chip *fifo;
    ALS_dbg("%s - start\n", __func__);
    tsl2510_power_ctrl(data, PWR_OFF);
    if (data->power_type == VDD_REGULATOR)
        regulator_put(data->regulator_vdd_1p8);
    if (data->i2c_1p8 == true)
        regulator_put(data->regulator_i2c_1p8);
#ifndef AMS_BUILD
    sensors_unregister(data->sensor_dev, tsl2510_sensor_attrs);
#endif
    sysfs_remove_group(&data->als_input_dev->dev.kobj,
            &als_attribute_group);
#ifndef AMS_BUILD
    sensors_remove_symlink(data->als_input_dev);
#endif
    input_unregister_device(data->als_input_dev);

#ifdef CONFIG_AMS_OPTICAL_SENSOR_EOL_MODE
    if (data->als_pinctrl) {
        devm_pinctrl_put(data->als_pinctrl);
        data->als_pinctrl = NULL;
    }
    if (data->pins_idle)
        data->pins_idle = NULL;
    if (data->pins_sleep)
        data->pins_sleep = NULL;
#endif
    disable_irq(data->dev_irq);
    free_irq(data->dev_irq, data);
    gpio_free(data->pin_als_int);
    if (data->pin_als_en >= 0)
        gpio_free(data->pin_als_en);
    mutex_destroy(&data->i2clock);
    mutex_destroy(&data->activelock);
    mutex_destroy(&data->suspendlock);
    mutex_destroy(&data->flickerdatalock);
    misc_deregister(&data->miscdev);

    //	devm_kfree(data->deviceCtx);
    //	devm_kfree(data->pdata);
    //	devm_kfree(data);
    i2c_set_clientdata(client, NULL);


    //fifo = &data->fifo;
    //cancel_work_sync(&fifo->work);

    data = NULL;
    return 0;
}

static void tsl2510_shutdown(struct i2c_client *client)
{
    ALS_dbg("%s - start\n", __func__);
}

void tsl2510_pin_control(struct tsl2510_device_data *data, bool pin_set)
{
#ifdef CONFIG_AMS_OPTICAL_SENSOR_EOL_MODE
    int status = 0;
    if (!data->als_pinctrl) {
        ALS_err("%s - als_pinctrl is null\n", __func__);
        return;
    }
    if (pin_set) {
        if (!IS_ERR_OR_NULL(data->pins_idle)) {
            status = pinctrl_select_state(data->als_pinctrl,
                    data->pins_idle);
            if (status)
                ALS_err("%s - can't set pin default state\n",
                        __func__);
            ALS_info("%s idle\n", __func__);
        }
    }
    else {
        if (!IS_ERR_OR_NULL(data->pins_sleep)) {
            status = pinctrl_select_state(data->als_pinctrl,
                    data->pins_sleep);
            if (status)
                ALS_err("%s - can't set pin sleep state\n",
                        __func__);
            ALS_info("%s sleep\n", __func__);
        }
    }
#endif
}

static int tsl2510_suspend(struct device *dev)
{
    struct tsl2510_device_data *data = dev_get_drvdata(dev);
    int err = 0;

    ALS_dbg("%s - %d\n", __func__, data->regulator_state);

    while (data->regulator_state) {
	err = tsl2510_stop(data);
	if (err < 0) {
	    ALS_dbg("%s - err in stop", __func__);
	    break;
	}
	data->suspend_cnt++;
    }

    mutex_lock(&data->suspendlock);

    data->pm_state = PM_SUSPEND;
    tsl2510_pin_control(data, false);

    mutex_unlock(&data->suspendlock);
    ALS_dbg("%s - suspend done", __func__);

    return err;
}

static int tsl2510_resume(struct device *dev)
{
    struct tsl2510_device_data *data = dev_get_drvdata(dev);
    int err = 0;

    ALS_dbg("%s - %d\n", __func__, data->suspend_cnt);

    mutex_lock(&data->suspendlock);

    tsl2510_pin_control(data, true);
    data->pm_state = PM_RESUME;

    mutex_unlock(&data->suspendlock);

    while(data->suspend_cnt) {
	err = tsl2510_start(data);
	if (err < 0) {
	    ALS_dbg("%s - err in start", __func__);
	    break;
	}
	data->suspend_cnt--;
    }
 
    ALS_dbg("%s - resume done", __func__);
    return err;
}

static const struct dev_pm_ops tsl2510_pm_ops = {
    .suspend = tsl2510_suspend,
    .resume = tsl2510_resume
};

static const struct i2c_device_id tsl2510_idtable[] = {
    { "tsl2510", 0 },
    { }
};
/* descriptor of the tsl2510 I2C driver */
static struct i2c_driver tsl2510_driver = {
    .driver = {
        .name = "tsl2510",
        .owner = THIS_MODULE,
        .pm = &tsl2510_pm_ops,
        .of_match_table = tsl2510_match_table,
    },
    .probe = tsl2510_probe,
    .remove = tsl2510_remove,
    .shutdown = tsl2510_shutdown,
    .id_table = tsl2510_idtable,
};

/* initialization and exit functions */
static int __init tsl2510_init(void)
{
    int rc;
#ifndef AMS_BUILD
    if (!lpcharge)
        return i2c_add_driver(&tsl2510_driver);
    else
        return 0;
#else

    rc = i2c_add_driver(&tsl2510_driver);
#endif
    return rc;
}

static void __exit tsl2510_exit(void)
{
    i2c_del_driver(&tsl2510_driver);
}

module_init(tsl2510_init);
module_exit(tsl2510_exit);

MODULE_AUTHOR("Samsung Electronics");
MODULE_DESCRIPTION("TSL2510 ALS Driver");
MODULE_LICENSE("GPL");
