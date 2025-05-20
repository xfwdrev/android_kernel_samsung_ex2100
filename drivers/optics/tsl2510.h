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

 /*! \file
  * \brief Device driver for monitoring ambient light intensity in (lux)
  * proximity detection (prox), and Beam functionality within the
  * AMS TMX49xx family of devices.
  */

#ifndef __AMS_TSL2510_H
#define __AMS_TSL2510_H

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/input.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/interrupt.h>
#include <linux/leds.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#include <linux/spinlock.h>
#include <linux/gpio.h>
#include <linux/kthread.h>
#include <linux/sched.h>
#include <linux/time.h>
#include <linux/timer.h>
#include <linux/file.h>
#include <linux/syscalls.h>
#ifdef CONFIG_AMS_OPTICAL_SENSOR_EOL_MODE
#include <linux/pinctrl/consumer.h>
#include <linux/pm_qos.h>
#endif

#ifdef CONFIG_OF
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#endif

#define HEADER_VERSION		"14"

#define CONFIG_AMS_ALS_CRGBW
#define CONFIG_AMS_OPTICAL_SENSOR_ALS
#define CONFIG_AMS_OPTICAL_SENSOR_ALS_CCB
#define CONFIG_AMS_OPTICAL_SENSOR_FLICKER
#define CONFIG_AMS_OPTICAL_SENSOR_2510
//#define CONFIG_AMS_OPTICAL_SENSOR_LOAD_FIFO
#define CONFIG_AMS_OPTICAL_SENSOR_POLLING
#define HIGH    0xFF
#define LOW     0x00

#if !defined(CONFIG_ALS_CAL_TARGET)
#define CONFIG_ALS_CAL_TARGET          300 /* lux */
#endif
#if !defined(CONFIG_ALS_CAL_TARGET_TOLERANCE)
#define CONFIG_ALS_CAL_TARGET_TOLERANCE  15 /* lux */
#endif

#define AMS_LUX_AVERAGE_COUNT    8

enum fft_size {
    FFT_128 = 128,
    FFT_256 = 256,
    FFT_512 = 512,
    FFT_1024 = 1024,
    FFT_2048 = 2048,
};

#define AMS_SAMPLING_TIME					(781)  /* usec */
//#define AMS_SAMPLING_TIME					(1000)  /* usec */
//#define AMS_SAMPLING_TIME					(500)  /* usec */
//#define AMS_SAMPLING_TIME					(250)  /* usec */


// FLICKER SETTING VALUES
#define AMS_FLICKER_NUM_SAMPLES         (512)
#define AMS_CLR_WIDE_FLICKER_NUM_SAMPLES (128) // Clear & wide = 128 samples ,  128 * 2byte * 2ch = 512byte
//#define AMS_FLICKER_NUM_SAMPLES         (128)

#define AMS_FLICKER_THR_LVL                   (270)
//#define AMS_FLICKER_THR_LVL                   (128)

//#define AMS_FFT_SIZE FFT_512
#define AMS_FFT_SIZE FFT_128

typedef enum _amsAlsAdaptive {
    ADAPTIVE_ALS_NO_REQUEST,
    ADAPTIVE_ALS_TIME_INC_REQUEST,
    ADAPTIVE_ALS_TIME_DEC_REQUEST,
    ADAPTIVE_ALS_GAIN_INC_REQUEST,
    ADAPTIVE_ALS_GAIN_DEC_REQUEST
} amsAlsAdaptive_t;

typedef enum _amsAlsStatus {
    ALS_STATUS_IRQ = (1 << 0),
    ALS_STATUS_RDY = (1 << 1),
    ALS_STATUS_OVFL = (1 << 2)
} amsAlsStatus_t;

typedef struct _alsData {
    uint32_t clearADC;
    uint32_t widebandADC;
} alsData_t;

typedef struct _amsAlsCalibration {
    uint32_t Time_base; /* in uSec */
    uint32_t adcMaxCount;
    uint16_t calibrationFactor; /* default 1000 */
    uint8_t thresholdLow;
    uint8_t thresholdHigh;
    int32_t Wbc;
} amsAlsCalibration_t;

typedef struct _amsAlsInitData {
    bool adaptive;
    bool irRejection;
    uint32_t time_us;
    uint32_t gain;
    amsAlsCalibration_t calibration;
} amsAlsInitData_t;

typedef struct _amsALSConf {
    uint32_t time_us;
    uint32_t gain;
    uint8_t thresholdLow;
    uint8_t thresholdHigh;
} amsAlsConf_t;

typedef struct _amsAlsDataSet {
    alsData_t *datasetArray;
    uint64_t timeStamp;
    uint8_t status;
} amsAlsDataSet_t;

typedef struct _amsAlsResult {
    uint32_t irrClear;
    uint32_t irrRed;
    uint32_t irrGreen;
    uint32_t irrBlue;
    uint32_t IR;
    uint32_t irrWideband;
    uint32_t mLux;
    uint32_t mMaxLux;
    uint32_t mLux_ave;
    uint32_t CCT;
    amsAlsAdaptive_t adaptive;
    uint32_t rawClear;
    uint32_t rawRed;
    uint32_t rawGreen;
    uint32_t rawBlue;
    uint32_t rawWideband;
} amsAlsResult_t;

typedef struct _amsAlsContext {
    uint64_t lastTimeStamp;
    uint32_t ave_lux[AMS_LUX_AVERAGE_COUNT];
    uint32_t ave_lux_index;
    uint32_t cpl;
    uint32_t uvir_cpl;
    uint32_t time_us;
    amsAlsCalibration_t calibration;
    amsAlsResult_t results;
    bool adaptive;
    uint16_t saturation;
    uint32_t ClearGain; // Clear Channel GAIN
    uint32_t WBGain;// WideBand Channel GAIN

    uint32_t previousGain;
    uint32_t previousLux;
    bool notStableMeasurement;
} amsAlsContext_t;

typedef struct _amsAlsAlgInfo {
    char *algName;
    uint16_t contextMemSize;
    uint16_t scratchMemSize;
    amsAlsCalibration_t calibrationData;
    int(*initAlg)(amsAlsContext_t *ctx, amsAlsInitData_t *initData);
    int(*processData)(amsAlsContext_t *ctx, amsAlsDataSet_t *inputData);
    int(*getResult)(amsAlsContext_t *ctx, amsAlsResult_t *outData);
    int(*setConfig)(amsAlsContext_t *ctx, amsAlsConf_t *inputData);
    int(*getConfig)(amsAlsContext_t *ctx, amsAlsConf_t *outputData);
} amsAlsAlgoInfo_t;

typedef struct {
    uint32_t calibrationFactor;
    uint32_t Time_base; /* in uSec */
    uint32_t adcMaxCount;
    uint8_t thresholdLow;
    uint8_t thresholdHigh;
    int32_t Wbc;
} ams_ccb_als_calibration_t;

typedef struct {
    uint32_t uSecTime;
    uint32_t gain;
    uint8_t threshold;
} ams_ccb_als_config_t;

typedef struct {
    bool calibrate;
    bool autoGain;
    uint8_t hysteresis;
    uint16_t sampleRate;
    ams_ccb_als_config_t configData;
    ams_ccb_als_calibration_t calibrationData;
} ams_ccb_als_init_t;

typedef enum {
    AMS_CCB_ALS_INIT,
    AMS_CCB_ALS_RGB,
    AMS_CCB_ALS_AUTOGAIN,
    AMS_CCB_ALS_CALIBRATION_INIT,
    AMS_CCB_ALS_CALIBRATION_COLLECT_DATA,
    AMS_CCB_ALS_CALIBRATION_CHECK,
    AMS_CCB_ALS_LAST_STATE
} ams_ccb_als_state_t;

typedef struct {
    char *algName;
    uint16_t contextMemSize;
    uint16_t scratchMemSize;
    ams_ccb_als_calibration_t defaultCalibrationData;
} ams_ccb_als_info_t;

typedef struct {
    ams_ccb_als_state_t state;
    amsAlsContext_t ctxAlgAls;
    ams_ccb_als_init_t initData;
    uint16_t bufferCounter;
    uint32_t shadowAiltReg;
    uint32_t shadowAihtReg;
} ams_ccb_als_ctx_t;

typedef struct {
    uint8_t  statusReg; // STATUS
    uint8_t  status2Reg; // STATUS2
    uint8_t  alsstatusReg; // ALS_STATUS
    uint8_t  alsstatus2Reg; // ALS_STATUS2
} ams_ccb_als_dataSet_t;

typedef struct {
    uint32_t ir;
    uint32_t clear;
    uint32_t wideband;
    uint32_t rawClear;
    uint32_t rawWideband;
    uint32_t time_us;
    uint32_t ClearGain;
    uint32_t WBGain;
} ams_ccb_als_result_t;

#define PWR_ON		1
#define PWR_OFF		0
#define PM_RESUME	1
#define PM_SUSPEND	0
#define NAME_LEN		32

#define CONFIG_AMS_LITTLE_ENDIAN 1
#ifdef CONFIG_AMS_LITTLE_ENDIAN
#define AMS_ENDIAN_1    0
#define AMS_ENDIAN_2    8
#else
#define AMS_ENDIAN_2    0
#define AMS_ENDIAN_1    8
#endif

#define AMS_PORT_portHndl   struct i2c_client

typedef struct {
    bool     nearBy;
    uint16_t proximity;
} ams_apiPrx_t;

typedef enum {
    NOT_VALID,
    PRESENT,
    ABSENT
} ternary;

typedef struct {
//    ternary freq100Hz;
//    ternary freq120Hz;

    int32_t mHz;
} ams_apiAlsFlicker_t;

typedef struct {
    uint32_t ir;
    uint32_t clear;
    uint32_t wideband;
    uint32_t rawClear;
    uint32_t rawWideband;
    uint32_t time_us;
    uint32_t ClearGain;
    uint32_t WBGain;
    uint32_t gain;

} ams_apiAls_t;

#ifdef __cplusplus
extern "C" {
#endif

#define AMS_USEC_PER_TICK			(1389) // 1.388889usec
    //#define ACTUAL_USEC(x)	(((x + AMS_USEC_PER_TICK / 2) / AMS_USEC_PER_TICK) * AMS_USEC_PER_TICK)
    //#define AMS_ALS_USEC_TO_REG(x)		(256 - (x / AMS_USEC_PER_TICK))
#define AMS_DEFAULT_REPORTTIME_US	(1000000) /* Max 8 seconds */
#define AMS_PRX_PGLD_TO_REG(x)		((x-4)/8)

#ifndef UINT_MAX_VALUE
#define UINT_MAX_VALUE      (-1)
#endif

#define AMS_CALIBRATION_DONE                (-1)
#define AMS_CALIBRATION_DONE_BUT_FAILED     (-2)

    typedef enum _deviceIdentifier_e {
        AMS_UNKNOWN_DEVICE,
        AMS_TSL2510,
        AMS_TSL2510_UNTRIM,
        AMS_LAST_DEVICE
    } ams_deviceIdentifier_e;

/*TSL2510 */
/*0x92 ID(0x5C), 0x91 REVID(0x00)*/
#define AMS_DEVICE_ID       0x5C
#define AMS_DEVICE_ID_MASK  0xFF
#define AMS_REV_ID         0x00
#define AMS_REV_ID_UNTRIM   0x00
#define AMS_REV_ID_MASK     0x1F


#define AMS_PRX_PERS_TO_REG(x)		(x << 4)
#define AMS_PRX_REG_TO_PERS(x)		(x >> 4)
#define AMS_PRX_CURRENT_TO_REG(mA)	((((mA) > 257) ? 127 : (((mA) - 4) >> 1)) << 0)
#define AMS_ALS_PERS_TO_REG(x)		(x << 0)
#define AMS_ALS_REG_TO_PERS(x)		(x >> 0)

    typedef enum _deviceRegisters {
        DEVREG_MOD_CHANNEL_CTRL,//0x40
        DEVREG_ENABLE,//0x80
        DEVREG_MEAS_MODE0,//0x81
        DEVREG_MEAS_MODE1,//0x82
        DEVREG_SAMPLE_TIME0,//0x83
        DEVREG_SAMPLE_TIME1,//0x84
        DEVREG_ALS_NR_SAMPLES0,//0x85
        DEVREG_ALS_NR_SAMPLES1,//0x86
        DEVREG_FD_NR_SAMPLES0,//0x87
        DEVREG_FD_NR_SAMPLES1,//0x88
        DEVREG_WTIME,//0x89
        DEVREG_AILT0,//0x8A
        DEVREG_AILT1,//0x8B
        DEVREG_AILT2,//0x8C
        DEVREG_AIHT0,//0x8D
        DEVREG_AIHT1,//0x8E
        DEVREG_AIHT2,//0x8F
        DEVREG_AUXID,//0x90
        DEVREG_REVID,//0x91
        DEVREG_ID,//0x92
        DEVREG_STATUS,//0x93
        DEVREG_ALS_STATUS,//0x94
        DEVREG_ALS_DATAL0,//0x95
        DEVREG_ALS_DATAH0,//0x96
        DEVREG_ALS_DATAL1,//0x97
        DEVREG_ALS_DATAH1,//0x98
        DEVREG_ALS_DATAL2,//0x99
        DEVREG_ALS_DATAH2,//0x9A
        DEVREG_ALS_STATUS2,//0x9B
        DEVREG_ALS_STATUS3,//0x9C
        DEVREG_STATUS2,//0x9D
        DEVREG_STATUS3,//0x9E
        DEVREG_STATUS4,//0x9F
        DEVREG_STATUS5,//0xA0
        DEVREG_CFG0,//0xA1
        DEVREG_CFG1,//0xA2
        DEVREG_CFG2,//0xA3
        DEVREG_CFG3,//0xA4
        DEVREG_CFG4,//0xA5
        DEVREG_CFG5,//0xA6
        DEVREG_CFG6,//0xA7
        DEVREG_CFG7,//0xA8
        DEVREG_CFG8,//0xA9
        DEVREG_CFG9,//0xAA
        DEVREG_AGC_NR_SAMPLES_LO,//0xAC
        DEVREG_AGC_NR_SAMPLES_HI,//0xAD
        DEVREG_TRIGGER_MODE,//0xAE
        DEVREG_CONTROL,//0xB1
        DEVREG_INTENAB,//0xBA
        DEVREG_SIEN,//0xBB
        DEVREG_MOD_COMP_CFG1,//0xCE
        DEVREG_MEAS_SEQR_FD_0,//0xCF
        DEVREG_MEAS_SEQR_ALS_FD_1,//0xD0
        DEVREG_MEAS_SEQR_APERS_AND_VSYNC_WAIT,//0xD1
        DEVREG_MEAS_SEQR_RESIDUAL_0,//0xD2
        DEVREG_MEAS_SEQR_RESIDUAL_1_AND_WAIT,//0xD3
        DEVREG_MEAS_SEQR_STEP0_MOD_GAINX_L,//0xD4
        DEVREG_MEAS_SEQR_STEP1_MOD_GAINX_L,//0xD6
        DEVREG_MEAS_SEQR_STEP2_MOD_GAINX_L,//0xD8
        DEVREG_MEAS_SEQR_STEP3_MOD_GAINX_L,//0xDA
        DEVREG_MEAS_SEQR_STEP0_MOD_PHDX_SMUX_L,//0xDC
        DEVREG_MEAS_SEQR_STEP0_MOD_PHDX_SMUX_H,//0xDD
        DEVREG_MEAS_SEQR_STEP1_MOD_PHDX_SMUX_L,//0xDE
        DEVREG_MEAS_SEQR_STEP1_MOD_PHDX_SMUX_H,//0xDF
        DEVREG_MEAS_SEQR_STEP2_MOD_PHDX_SMUX_L,//0xE0
        DEVREG_MEAS_SEQR_STEP2_MOD_PHDX_SMUX_H,//0xE1
        DEVREG_MEAS_SEQR_STEP3_MOD_PHDX_SMUX_L,//0xE2
        DEVREG_MEAS_SEQR_STEP3_MOD_PHDX_SMUX_H,//0xE3
        DEVREG_MOD_CALIB_CFG0,//0xE4
        DEVREG_MOD_CALIB_CFG2,//0xE6
        DEVREG_VSYNC_PERIOD_L,//0xF2
        DEVREG_VSYNC_PERIOD_H,//0xF3
        DEVREG_VSYNC_PERIOD_TARGET_L,//0xF4
        DEVREG_VSYNC_PERIOD_TARGET_H,//0xF5
        DEVREG_VSYNC_CONTROL,//0xF6
        DEVREG_VSYNC_CFG,//0xF7
        DEVREG_VSYNC_GPIO_INT,//0xF8
        DEVREG_MOD_FIFO_DATA_CFG0,//0xF9
        DEVREG_MOD_FIFO_DATA_CFG1,//0xFA
        DEVREG_MOD_FIFO_DATA_CFG2,//0xFB
        DEVREG_FIFO_THR,//0xFC
        DEVREG_FIFO_LEVEL,//0xFD
        DEVREG_FIFO_STATUS0,//0xFE
        DEVREG_FIFO_DATA,//0xFF
        DEVREG_REG_MAX
    } ams_deviceRegister_t;

    typedef enum _2510_regOptions {
        PON = 0x01, /* register 0x80 */
        AEN = 0x02,
        /* reserved: 0x20 */
        FDEN = 0x40,

        /* STATUS REG */
        SINT = 0x01, /* register 0x93 */
        FINT = 0x04,
        AINT = 0x08,
        MINT = 0x80,   /*aligned to STATUS2, check analog or digital saturation */
        AMS_ALL_INT = SINT + FINT + AINT + MINT,

        /* STATUS2 REG */
        ALS_DATA_VALID = 0x40, /* register 0x9D */

        /* ALS_STATUS REG */
        ALS_DATA1_SCALED_STATUS = 0x02, /* register 0x94 */
        ALS_DATA0_SCALED_STATUS = 0x04,
        ALS_DATA1_ANALOG_SAT_STATUS = 0x10,
        ALS_DATA0_ANALOG_SAT_STATUS = 0x20,

#if 0
        FDSAT_DIGITAL = 0x01, /* register 0xA3 */
        FDSAT_ANALOG = 0x02,
        ASAT_ANALOG = 0x08,
        ASAT_DIGITAL = 0x10,
        AVALID = 0x40,
        PVALID0 = 0x80,

        PSAT_AMB = 0x01, /* register 0xA4 */
        PSAT_REFL = 0x02,
        PSAT_ADC = 0x04,

        PINT0_PILT = 0x01, /* register 0xA5 */
        PINT0_PIHT = 0x02,
        PINT1_PILT = 0x04,
        PINT1_PIHT = 0x08,

        /* ambient, prox threshold/hysteresis bits in 0xA4-A5:  not used (yet?) */

        IBUSY = 0x01, /* register 0xA6 */
        SINT_IRBEAM = 0x02,
        SINT_SMUX = 0x04,
        SINT_FD = 0x08,
        SINT_ALS_MAN_AZ = 0x10,
        SINT_AUX = 0x20,

        INIT_BUSY = 0x01, /* register 0xA7 */

        RAM_BANK_0 = 0x00, /* register 0xA9 */
        RAM_BANK_1 = 0x01,
        RAM_BANK_2 = 0x02,
        RAM_BANK_3 = 0x03,
        ALS_TRIG_LONG = 0x04,
        PRX_TRIG_LONG = 0x08,
        REG_BANK = 0x10,
        LOWPWR_IDLE = 0x20,
        PRX_OFFSET2X = 0x40,
        PGOFF_HIRES = 0x80,

        AGAIN_1_HALF = 0x00, /* register 0xAA */
        AGAIN_1 = 0x01,
        AGAIN_2 = 0x02,
        AGAIN_4 = 0x03,
        AGAIN_8 = 0x04,
        AGAIN_16 = 0x05,
        AGAIN_32 = 0x06,
        AGAIN_64 = 0x07,
        AGAIN_128 = 0x08,
        AGAIN_256 = 0x09,
        ALS_TRIG_FAST = 0x40,
        S4S_MODE = 0x80,

        HXTALK_MODE1 = 0x20, /* register 0xAC */

        SMUX_CMD_ROM_INIT = 0x00, /* register 0xAF */
        SMUX_CMD_READ = 0x08,
        SMUX_CMD_WRITE = 0x10,
        SMUX_CMD_ARRAY_MODE = 0x18,

        SWAP_PROX_ALS5 = 0x01, /* register 0xB1 */
        ALS_AGC_ENABLE = 0x04,
        FD_AGC_ENABLE = 0x08,
        PROX_BEFORE_EACH_ALS = 0x10,

        SIEN_AUX = 0x08, /* register 0xB2 */
        SIEN_SMUX = 0x10,
        SIEN_ALS_MAN_AZ = 0x20,
        SIEN_FD = 0x40,
        SIEN_IRBEAM = 0x80,

        FD_PERS_ALWAYS = 0x00, /* register 0xB3 */
        FD_PERS_1 = 0x01,
        FD_PERS_2 = 0x02,
        FD_PERS_4 = 0x03,
        FD_PERS_8 = 0x04,
        FD_PERS_16 = 0x05,
        FD_PERS_32 = 0x06,
        FD_PERS_64 = 0x07,

        TRIGGER_APF_ALIGN = 0x10, /* register 0xB4 */
        PRX_TRIGGER_FAST = 0x20,
        PINT_DIRECT = 0x40,
        AINT_DIRECT = 0x80,

        PROX_FILTER_1 = 0x00, /* register 0xB8 */
        PROX_FILTER_2 = 0x01,
        PROX_FILTER_4 = 0x02,
        PROX_FILTER_8 = 0x03,
        HXTALK_MODE2 = 0x80,

        PGAIN_1 = 0x00, /* register 0xBB */
        PGAIN_2 = 0x01,
        PGAIN_4 = 0x02,
        PGAIN_8 = 0x03,

        PPLEN_4uS = 0x00, /* register 0xBC */
        PPLEN_8uS = 0x01,
        PPLEN_16uS = 0x02,
        PPLEN_32uS = 0x03,

        FD_COMPARE_32_32NDS = 0x00, /* register 0xD7 */
        FD_COMPARE_24_32NDS = 0x01,
        FD_COMPARE_16_32NDS = 0x02,
        FD_COMPARE_12_32NDS = 0x03,
        FD_COMPARE_8_32NDS = 0x04,
        FD_COMPARE_6_32NDS = 0x05,
        FD_COMPARE_4_32NDS = 0x06,
        FD_COMPARE_3_32NDS = 0x07,
        FD_SAMPLES_8 = 0x00,
        FD_SAMPLES_16 = 0x08,
        FD_SAMPLES_32 = 0x10,
        FD_SAMPLES_64 = 0x18,
        FD_SAMPLES_128 = 0x20,
        FD_SAMPLES_256 = 0x28,
        FD_SAMPLES_512 = 0x30,
        FD_SAMPLES_1024 = 0x38,

        FD_GAIN_1_HALF = 0x00, /* register 0xDA */
        FD_GAIN_1 = 0x08,
        FD_GAIN_2 = 0x10,
        FD_GAIN_4 = 0x18,
        FD_GAIN_8 = 0x20,
        FD_GAIN_16 = 0x28,
        FD_GAIN_32 = 0x30,
        FD_GAIN_64 = 0x38,
        FD_GAIN_128 = 0x40,
        FD_GAIN_256 = 0x48,

        FD_100HZ_FLICKER = 0x01, /* register 0xDB */
        FD_120HZ_FLICKER = 0x02,
        FD_100HZ_VALID = 0x04,
        FD_120HZ_VALID = 0x08,
        FD_SAT_DETECTED = 0x10,
        FD_MEAS_VALID = 0x20,

        ISTART_MOBEAM = 0x01, /* register 0xE8 */
        ISTART_REMCON = 0x02,

        START_OFFSET_CALIB = 0x01, /* register 0xEA */

        DCAVG_AUTO_BSLN = 0x80, /* register 0xEB */
        DCAVG_AUTO_OFFSET_ADJUST = 0x40,
        BINSRCH_SKIP = 0x08,

        BASELINE_ADJUSTED = 0x04, /* register 0xEE */
        OFFSET_ADJUSTED = 0x02,
        CALIB_FINISHED = 0x01,

#endif
        /* INTENAB register  */
        SIEN = 0x01,  /*aligned to STATUS3 *//* register 0xBA */
        FIEN = 0x04,
        AIEN = 0x08,
        MIEN = 0x80, /*aligned to STATUS2 */
        AMS_ALL_IENS = (AIEN + FIEN + MIEN + SIEN),



        LAST_IN_ENUM_LIST
    } ams_regOptions_t;

    typedef enum _2510_regMasks {
        MASK_AEN = 0x02,
        MASK_PEN = 0x04,
        MASK_FDEN = 0x40,

        MASK_WTIME = 0xFF, /* register 0x89 */

        MASK_MOD_FIFO_FD_GAIN_WRITE_ENABLE = 0x20, /* register 0x82[5] */
        MASK_MOD_FIFO_FD_END_MARKER_WRITE_ENABLE = 0x80, /* register 0x82[7] */


        MASK_SINT = 0x01, /* register 0x93 */
        MASK_FINT = 0x04,
        MASK_AINT = 0x08,
        MASK_MINT = 0x80,

        MASK_ALS_INT_ALL = MASK_AINT + MASK_FINT,


        MASK_ADATA = 0xFFFF, /* registers 0x95-0x98 */

           /* STATUS2 */
        MASK_ASAT_DIGITAL = 0x10,
        MASK_AVALID = 0x40,
        MASK_FDSAT_DIGITAL = 0x08,
        MASK_MOD1_SAT_ANALOG = 0x02,
        MASK_MOD0_SAT_ANALOG = 0x01,

        /* INTENAB register  */
        MASK_SIEN = 0x01,
        MASK_FIEN = 0x04,
        MASK_AIEN = 0x08,
        MASK_MIEN = 0x80,

        MASK_SOFT_RESET = 0x08, /* register 0xB1 */

        MASK_APERS = 0x0F, /* register 0xA6 */
        MASK_AGAIN0 = 0x0F, /* register 0xD4[3:0] */
        MASK_AGAIN1 = 0xF0, /* register 0xD4[7:4] */

        MASK_AINT_DIRECT = 0x80, /*register -0xA3*/
        MASK_FIFO_THR_0 = 0x01, /*register -0xA3*/

        MASK_MOD_DIVIDER_SELECT = 0x03, /*register -0xA8*/


        MASK_AGC_ASAT = 0xF0, /* register 0xDF */
        MASK_AGC_PREDICT = 0xF0, /* register 0xE1 */

        MASK_MOD_CALIB_NTH_ITERATION_AGC_ENABLE = 0x20, /* register 0xE6 */
        MASK_MAX_MOD_GAIN = 0xF0, /* register 0xA9[7:4] */

        MASK_FIFO_CLR = 0x02, /* register 0xB1[1]*/

        MASK_ALS_SAMPLE_TIME = 0x07FF, /*register 0x83 and 0x84*/


        MASK_FIFO_OVERFLOW = 0x80, /*register 0xFE*/
        MASK_FIFO_UNDERFLOW = 0x40, /*register 0xFE*/


        MASK_MOD_ALS_FIFO_DATA0_WRITE_ENABLE = 0x80, /*register 0xF9*/
        MASK_MOD_FD_FIFO_DATA0_COMPRESSION_ENABLE = 0x20, /*register 0xF9*/


        MASK_MOD_ALS_FIFO_DATA1_WRITE_ENABLE = 0x80, /*register 0xFA*/
        MASK_MOD_FD_FIFO_DATA1_COMPRESSION_ENABLE = 0x20, /*register 0xFA*/

        MASK_MOD_ALS_FIFO_DATA2_WRITE_ENABLE = 0x80, /*register 0xFB*/
        MASK_MOD_FD_FIFO_DATA2_COMPRESSION_ENABLE = 0x20, /*register 0xFB*/


#if 0

        MASK_AGAIN = 0x1F, /* register 0xAA */

        MASK_HXTALK_MODE1 = 0x20, /* register 0xAC */

        MASK_SMUX_CMD = 0x18, /* register 0xAF */

        MASK_AUTOGAIN = 0x04, /* register 0xB1 */
        MASK_PROX_BEFORE_EACH_ALS = 0x10,

        MASK_SIEN_FD = 0x40, /* register 0xB2 */
        MASK_SIEN_IRBEAM = 0x80,

        MASK_FD_PERS = 0x07, /* register 0xB3 */
        MASK_AGC_HYST_LOW = 0x30,
        MASK_AGC_HYST_HIGH = 0xC0,

        MASK_PRX_TRIGGER_FAST = 0x20, /* register 0xB4 */
        MASK_PINT_DIRECT = 0x40,
        MASK_AINT_DIRECT = 0x80,

        MASK_PROX_FILTER = 0x03, /* register 0xB8 */
        MASK_HXTALK_MODE2 = 0x80,

        MASK_PLDRIVE0 = 0x7f, /* register 0xB9 */

        MASK_PPERS = 0xF0,

        MASK_PPULSE = 0x3F, /* register 0xBC */
        MASK_PPLEN = 0xC0,

        MASK_PGAIN = 0x03, /* register 0xBB */

        MASK_PLDRIVE = 0x7F, /* Registers 0xB9, 0xBA */

        MASK_FD_TIME_MSBits = 0x07, /* Register 0xDA */

        MASK_100HZ_FLICKER = 0x05, /* Register 0xDB */
        MASK_120HZ_FLICKER = 0x0A,
        MASK_CLEAR_FLICKER_STATUS = 0x3C,
        MASK_FLICKER_VALID = 0x2C,

        MASK_SLEW = 0x10, /* register E0 */
        MASK_ISQZT = 0x07,

        MASK_OFFSET_CALIB = 0x01, /* register 0xEA */

        MASK_DCAVG_AUTO_BSLN = 0x80, /* register 0xEB */
        MASK_BINSRCH_SKIP = 0x08,

        MASK_PROX_AUTO_OFFSET_ADJUST = 0x40, /* register 0xEC */
        MASK_PXAVG_AUTO_BSLN = 0x08, /* register 0xEC */

        MASK_BINSRCH_TARGET = 0xE0, /* register 0xED */

        MASK_SIEN = 0x01, /* register 0xF9 */
        MASK_CIEN = 0x02,
        MASK_FIEN = 0x04,
        MASK_AIEN = 0x08,
        MASK_PIEN0 = 0x10,
        MASK_PIEN1 = 0x20,
        MASK_PSIEN = 0x40,
        MASK_ASIEN_FDSIEN = 0x80,

        MASK_AGC_HYST = 0x30,
        MASK_AGC_LOW_HYST = 0x30,
        MASK_AGC_HIGH_HYST = 0xC0,

        MASK_SOFT_RESET = 0x04, /* register 0xF3 */
#endif
        MASK_LAST_IN_ENUMLIST
    } ams_regMask_t;


#define AMS_ENABLE_ALS_EX()         {ctx->shadowEnableReg |= (AEN); \
    ams_setByte(ctx->portHndl, DEVREG_ENABLE, ctx->shadowEnableReg); \
    ams_setField(ctx->portHndl, DEVREG_INTENAB, HIGH, (MASK_AIEN | MASK_FIEN));\
    }
#define AMS_DISABLE_ALS_EX()        {ctx->shadowEnableReg &= ~(AEN); \
    ams_setByte(ctx->portHndl, DEVREG_ENABLE, ctx->shadowEnableReg); \
    ams_setField(ctx->portHndl, DEVREG_INTENAB, LOW, (MASK_AIEN | MASK_FIEN));\
    }

    typedef struct _deviceRegisterTable {
        uint8_t address;
        uint8_t resetValue;
    } deviceRegisterTable_t;

    typedef enum _2510_config_options {
        AMS_CONFIG_ENABLE,
        AMS_CONFIG_THRESHOLD,
        AMS_CONFIG_OPTION_LAST
    } deviceConfigOptions_t;

    typedef enum _2510_mode {
        MODE_OFF = (0),
        MODE_ALS_LUX = (1 << 0),
        MODE_ALS_RGB = (1 << 1),
        MODE_ALS_CT = (1 << 2),
        MODE_ALS_WIDEBAND = (1 << 3),
        MODE_ALS_ALL = (MODE_ALS_LUX | MODE_ALS_RGB | MODE_ALS_CT | MODE_ALS_WIDEBAND),
        MODE_FLICKER = (1 << 4), /* is independent of ALS in this model */
        MODE_PROX = (1 << 5),
        MODE_IRBEAM = (1 << 6),
        MODE_UNKNOWN    /* must be in last position */
    } ams_mode_t;

    typedef enum _2510_configureFeature {
        AMS_CONFIG_PROX,
        AMS_CONFIG_ALS_LUX,
        AMS_CONFIG_ALS_RGB,
        AMS_CONFIG_ALS_CT,
        AMS_CONFIG_ALS_WIDEBAND,
        AMS_CONFIG_FLICKER,
        AMS_CONFIG_MOBEAM,
        AMS_CONFIG_REMCON,
        AMS_CONFIG_FEATURE_LAST
    } ams_configureFeature_t;

    typedef struct _calibrationData {
        uint32_t timeBase_us;
        uint32_t adcMaxCount;
        uint8_t alsThresholdHigh; /* in % */
        uint8_t alsThresholdLow;  /* in % */
        uint16_t alsCalibrationFactor;        /* multiplicative factor default 1000 */
        char deviceName[8];
        int32_t alsCoefC;
        int32_t alsCoefR;
        int32_t alsCoefG;
        int32_t alsCoefB;
        int16_t alsDfg;
        uint16_t alsCctOffset;
        uint16_t alsCctCoef;
        int32_t Wbc;
    } ams_calibrationData_t;

    typedef struct _flickerParams {
        uint32_t samplePeriod_us;
        uint16_t gain;
        uint8_t compare;
        uint8_t statusReg;
        uint16_t fifolvl;
        uint32_t overflow;
        int32_t frequency;
        int flicker_data_cnt;
        uint16_t flicker_data[AMS_FFT_SIZE];
        ams_apiAlsFlicker_t lastValid;
        uint8_t data_ready;
        uint8_t als_data_ready;
    } ams_flicker_ctx_t;

    typedef struct _2510Context {
        ams_deviceIdentifier_e deviceId;
        uint64_t timeStamp;
        AMS_PORT_portHndl *portHndl;
        ams_mode_t mode;
#ifdef AMS_PHY_SUPPORT_SHADOW
        uint8_t shadow[DEVREG_REG_MAX];
#endif
        // ams_ccb_proximity_ctx_t ccbProxCtx;
        ams_ccb_als_ctx_t ccbAlsCtx;
        // ams_ccb_irBeam_ctx_t ccbIrBeamCtx;
        ams_flicker_ctx_t flickerCtx;
        ams_calibrationData_t *systemCalibrationData;

        bool alwaysReadAls;			/* read ADATA every ams_deviceEventHandler call regardless of xINT bits */
        bool alwaysReadProx;		/* ditto PDATA */
        bool alwaysReadFlicker;

        bool agc;   // Auto gain control for als and flicker
        u8 sensor_mode;

        uint32_t updateAvailable;
        uint8_t shadowEnableReg;
        uint8_t shadowIntenabReg;
        uint8_t shadowStatus1Reg;     // STATUS
        uint8_t shadowStatus2Reg;    // STATUS2
        uint8_t shadowAlsStatusReg;  // ALS_STATUS
        uint8_t shadowAlsStatus2Reg;  // ALS_STATUS2
        uint8_t shadowFIFOStatusReg;  // FifoStatus
        uint16_t clear_average_fifo;
        uint16_t wideband_average_fifo;
        bool has_fifo_fd_end_marker;
        bool has_fifo_fd_gain;
        bool has_fifo_fd_checksum;
        uint8_t fifo_mod0_gain;
        uint8_t fifo_mod1_gain;
        uint16_t fifo_checksum;
        uint32_t clear_average;
        uint32_t wideband_average;
        uint16_t flicker_num_samples;
        bool hamming_status;
    } ams_deviceCtx_t;

    typedef enum _sensorType {
        AMS_NO_SENSOR_AVAILABLE,
        AMS_AMBIENT_SENSOR,
        AMS_FLICKER_SENSOR,
        AMS_PROXIMITY_SENSOR,
        AMS_WIDEBAND_ALS_SENSOR,
        AMS_SW_FLICKER_SENSOR,
        AMS_LAST_SENSOR,
        AMS_ALS_RGB_GAIN_CHANGED
    } ams_sensorType_t;

    typedef struct _sensorInfo {
        uint32_t standbyCurrent_uA;
        uint32_t activeCurrent_uA;
        uint32_t rangeMin;
        uint32_t rangeMax;
        char *driverName;
        uint8_t maxPolRate;
        uint8_t adcBits;
    } ams_SensorInfo_t;

    typedef struct _deviceInfo {
        uint32_t	memorySize;
        ams_calibrationData_t defaultCalibrationData;
        ams_SensorInfo_t proxSensor;
        ams_SensorInfo_t alsSensor;
        ams_SensorInfo_t mobeamSensor;
        ams_SensorInfo_t remconSensor;
        ams_sensorType_t tableSubSensors[10];
        uint8_t numberOfSubSensors;
        char *driverVersion;
        char *deviceModel;
        char *deviceName;
    } ams_deviceInfo_t;


#define ALS_DBG

#ifndef ALS_dbg
#ifdef ALS_DBG
#define ALS_dbg(format, arg...)		\
                printk(KERN_DEBUG "ALS_dbg : "format, ##arg)
#define ALS_err(format, arg...)		\
                printk(KERN_DEBUG "ALS_err : "format, ##arg)
#else
#define ALS_dbg(format, arg...)		{if (als_debug)\
                printk(KERN_DEBUG "ALS_dbg : "format, ##arg);\
                    }
#define ALS_err(format, arg...)		{if (als_debug)\
                printk(KERN_DEBUG "ALS_err : "format, ##arg);\
                    }
#endif
#endif

#ifndef ALS_info
#ifdef ALS_INFO
#define ALS_info(format, arg...)	\
                printk(KERN_INFO "ALS_info : "format, ##arg)
#else
#define ALS_info(format, arg...)	{if (als_info)\
                printk(KERN_INFO "ALS_info : "format, ##arg);\
                    }
#endif
#endif

    enum {
        DEBUG_REG_STATUS = 1,
        DEBUG_VAR,
    };

    struct mode_count {
        s32 hrm_cnt;
        s32 amb_cnt;
        s32 prox_cnt;
        s32 sdk_cnt;
        s32 cgm_cnt;
        s32 unkn_cnt;
    };

    enum platform_pwr_state {
        POWER_ON,
        POWER_OFF,
        POWER_STANDBY,
    };

    struct tsl2510_parameters {
        /* Common */
        u8 persist;
        /* ALS / Color */
        u8 als_gain;
        u8 als_auto_gain;
        u16 als_deltaP;
        u8 als_time;
    };

    // Must match definition in ../arch file
    struct amsdriver_i2c_platform_data {
        /* The following callback for power events received and handled by the driver.
         * Currently only for SUSPEND and RESUME
         */
        int(*platform_power)(struct device *dev, enum platform_pwr_state state);
        int(*platform_init)(void);
        void(*platform_teardown)(struct device *dev);
        bool haveCalibrationData;
        char const *als_name;
        struct tsl2510_parameters parameters;
#ifdef CONFIG_OF
        struct device_node  *of_node;
#endif
    };

#ifdef CONFIG_AMS_OPTICAL_SENSOR_EOL_MODE
    enum {
        EOL_STATE_INIT = -1,
        EOL_STATE_100,
        EOL_STATE_120,
        EOL_STATE_DONE
    };

    enum {
        EOL_TORCH,
        EOL_FLASH
    };
#endif

    enum {
        VDD_GPIO,
        VDD_REGULATOR
    };

    struct tsl2510_device_data {
        struct i2c_client *client;
        struct amsdriver_i2c_platform_data *pdata;
        int in_suspend;
        int wake_irq;
        int irq_pending;
        int suspend_cnt;
        int power_type;
        bool unpowered;
        u8 device_index;
        void *deviceCtx;

        struct device *dev;
        struct device *sensor_dev;
        struct input_dev *als_input_dev;
        struct mutex i2clock;
        struct mutex activelock;
        struct mutex suspendlock;
        struct mutex flickerdatalock;
        struct miscdevice miscdev;
        struct pinctrl *als_pinctrl;
        struct pinctrl_state *pins_sleep;
        struct pinctrl_state *pins_idle;
        struct regulator *regulator_vdd_1p8;
        struct regulator *regulator_i2c_1p8;
        wait_queue_head_t fifo_wait;
        bool i2c_1p8;
        u8 enabled;
        u8 sensor_mode;
        u32 sampling_period_ns;
        u32 fifo_thr;
        u8 regulator_state;
        s32 pin_als_int;
        s32 pin_als_en;
        s32 dev_irq;
        u8 irq_state;
        u32 reg_read_buf;
        u8 debug_mode;
        struct mode_count mode_cnt;
        bool pm_state;
        int isTrimmed;

        u8 part_type;
        u32 i2c_err_cnt;
        u32 user_ir_data;
        u32 user_flicker_data;
        bool saturation;
#ifdef CONFIG_AMS_OPTICAL_SENSOR_EOL_MODE
        char *eol_result;
        u8 eol_enable;
        u8 eol_result_status;
        s16 eol_flash_type;
        s16 eol_state;
        u32 eol_count;
        u32 eol_awb;
        u32 eol_clear;
        u32 eol_wideband;
        u32 eol_flicker;
        u8 eol_flicker_count;
        u8 eol_flicker_pass_cnt[2];
        u32 eol_flicker_awb[6][4];
        u32 eol_pulse_duty[2];
        u32 eol_pulse_count;
        u32 eol_ir_spec[4];
        u32 eol_clear_spec[4];
        u32 eol_icratio_spec[4];
        s32 pin_torch_en;
        s32 pin_flash_en;
#endif
        u16 awb_sample_cnt;
        int16_t  *flicker_data;
        int flicker_data_cnt;
        u8 fifodata[256];
        //#ifdef CONFIG_AMS_OPTICAL_SENSOR_POLLING
        struct hrtimer timer;
        ktime_t light_poll_delay;
        struct workqueue_struct *wq;
        struct work_struct work_light;
        //#endif

    };



    struct fifo_chip {
        //    struct feature_callbacks callbacks;
        struct work_struct work;
        uint8_t channels;
        uint8_t map;
        int16_t level;
        uint32_t k_len;
        uint32_t overflow;
        uint32_t koverflow;
        uint8_t max;
        uint16_t threshold;
        //    enum fifo_mode mode;
        struct tsl2510_device_data *data;
    };

#define AMSDRIVER_ALS_ENABLE 1
#define AMSDRIVER_ALS_DISABLE 0
#define AMSDRIVER_FLICKER_ENABLE 1
#define AMSDRIVER_FLICKER_DISABLE 0


    extern int sensors_create_symlink(struct input_dev *inputdev);
    extern void sensors_remove_symlink(struct input_dev *inputdev);
    extern int sensors_register(struct device **dev, void *drvdata,
    struct device_attribute *attributes[], char *name);
    extern void sensors_unregister(struct device *dev,
    struct device_attribute *attributes[]);

    extern unsigned int lpcharge;

#endif /* __AMS_TSL2510_H */
