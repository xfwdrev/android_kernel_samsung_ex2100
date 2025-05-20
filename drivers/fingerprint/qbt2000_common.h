#ifndef _UAPI_QBT2000_H_
#define _UAPI_QBT2000_H_

#include <linux/ktime.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/pm.h>
#include <linux/mutex.h>
#include <linux/atomic.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/kfifo.h>
#include <linux/poll.h>
#include <linux/pinctrl/consumer.h>
#include <linux/gpio.h>
#include <linux/err.h>
#include <linux/spi/spidev.h>
#include <linux/regulator/consumer.h>
#include <linux/pinctrl/consumer.h>
#include "../pinctrl/core.h"
#include "fingerprint_common.h"

#undef DISABLED_GPIO_PROTECTION

#define VENDOR "QCOM"
#define CHIP_ID "QBT2000"
#define MAX_NAME_SIZE 32
#define SPI_CLOCK_MAX 35000000

#define QBT2000_DEV "qbt2000"
#define MAX_FW_EVENTS 128
#define FW_MAX_IPC_MSG_DATA_SIZE 0x500
#define IPC_MSG_ID_CBGE_REQUIRED 29
#define IPC_MSG_ID_FINGER_ON_SENSOR 55
#define IPC_MSG_ID_FINGER_OFF_SENSOR 56
#define QBT2000_WAKELOCK_HOLD_TIME 500

#define MINOR_NUM_FD 0
#define MINOR_NUM_IPC 1

#define FINGER_DOWN_GPIO_STATE 1
#define FINGER_LEAVE_GPIO_STATE 0

enum qbt2000_commands {
	QBT2000_POWER_CONTROL = 21,
	QBT2000_ENABLE_SPI_CLOCK = 22,
	QBT2000_DISABLE_SPI_CLOCK = 23,
	QBT2000_ENABLE_IPC = 24,
	QBT2000_DISABLE_IPC = 25,
	QBT2000_ENABLE_WUHB = 26,
	QBT2000_DISABLE_WUHB = 27,
	QBT2000_CPU_SPEEDUP = 28,
	QBT2000_SET_SENSOR_TYPE = 29,
	QBT2000_SET_LOCKSCREEN = 30,
	QBT2000_SENSOR_RESET = 31,
	QBT2000_SENSOR_TEST = 32,
	QBT2000_NOISE_REQUEST_STOP = 33,
	QBT2000_NOISE_REQUEST_START = 34,
	QBT2000_NOISE_STATUS_GET = 35,
	QBT2000_NOISE_I2C_RESULT_GET = 36,
	QBT2000_NOISE_REQUEST_STATUS = 37,
	QBT2000_GET_MODELINFO = 38,
	QBT2000_IS_WHUB_CONNECTED = 105,
};

#define QBT2000_SENSORTEST_DONE		0x0000 // do nothing or test done
#define QBT2000_SENSORTEST_BGECAL	0x0001 // begin the BGECAL
#define QBT2000_SENSORTEST_NORMALSCAN	0x0002 // begin the normalscan
#define QBT2000_SENSORTEST_SNR		0x0004 // begin the snr
#define QBT2000_SENSORTEST_CAPTURE	0x0008 // begin the image capture. it also needs liveness capture.

/*
 * enum qbt2000_fw_event -
 *      enumeration of firmware events
 * @FW_EVENT_FINGER_DOWN - finger down detected
 * @FW_EVENT_FINGER_UP - finger up detected
 * @FW_EVENT_INDICATION - an indication IPC from the firmware is pending
 */
enum qbt2000_fw_event {
	FW_EVENT_FINGER_DOWN = 1,
	FW_EVENT_FINGER_UP = 2,
	FW_EVENT_CBGE_REQUIRED = 3,
};

struct finger_detect_gpio {
	int gpio;
	int active_low;
	int irq;
	struct work_struct work;
	int last_gpio_state;
};

struct fw_event_desc {
	enum qbt2000_fw_event ev;
};

struct fw_ipc_info {
	int gpio;
	int irq;
};

struct qbt2000_drvdata {
	struct class *qbt2000_class;
	struct cdev qbt2000_fd_cdev;
	struct cdev qbt2000_ipc_cdev;
	struct device *dev;
	struct device *fp_device;
	atomic_t fd_available;
	atomic_t ipc_available;
	struct mutex mutex;
	struct mutex ioctl_mutex;
	struct mutex fd_events_mutex;
	struct mutex ipc_events_mutex;
	struct fw_ipc_info fw_ipc;
	struct finger_detect_gpio fd_gpio;
	DECLARE_KFIFO(fd_events, struct fw_event_desc, MAX_FW_EVENTS);
	DECLARE_KFIFO(ipc_events, struct fw_event_desc, MAX_FW_EVENTS);
	wait_queue_head_t read_wait_queue_fd;
	wait_queue_head_t read_wait_queue_ipc;

	int ldogpio;
	int sensortype;
	int cbge_count;
	int wuhb_count;
	int reset_count;
	int now_state;
	bool enabled_ipc;
	bool enabled_wuhb;
	bool enabled_ldo;
	bool tz_mode;
	const char *model_info;
	const char *chipid;
	struct pinctrl *p;
	struct pinctrl_state *pins_poweron;
	struct pinctrl_state *pins_poweroff;
	const char *sensor_position;
	const char *btp_vdd;
	struct regulator *regulator_1p8;
	struct wakeup_source *fp_signal_lock;
	struct spi_clk_setting *clk_setting;
	struct boosting_config *boosting;
	struct debug_logger *logger;
};

#ifdef CONFIG_BATTERY_SAMSUNG
extern unsigned int lpcharge;
#endif

struct debug_logger *g_logger;

#endif /* _UAPI_QBT2000_H_ */
