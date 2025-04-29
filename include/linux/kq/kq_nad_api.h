#ifndef __KQ_NAD_API_H__
#define __KQ_NAD_API_H__

#define KQ_NAD_API_MPARAM_MAX_DELIMITER 2
#define KQ_NAD_API_MPARAM_MAX_LEN 50
#define KQ_NAD_API_SUCCESS 0
#define KQ_NAD_API_FAIL 1

struct kq_nad_api {
	char name[30];
};

static struct kq_nad_api kq_nad_api_pin_list[] = {
	{ "MAINCAM_SCL_1P8" },
	{ "MAINCAM_SDA_1P8" },
	{ "UWCAM_SCL_1P8" },
	{ "UWCAM_SDA_1P8" },
	{ "W_AF_EEP_T10X_EEP_SCL_1P8" },
	{ "W_AF_EEP_T10X_EEP_SDA_1P8" },
	{ "TELEX10_SCL_1P8" },
	{ "TELEX10_SDA_1P8" },
	{ "TELEX3_SCL_1P8" },
	{ "TELEX3_SDA_1P8" },
	{ "UW_AF_EEP_SCL_1P8" },
	{ "UW_AF_EEP_SDA_1P8" },
	{ "VTCAM_SCL_1P8" },
	{ "VTCAM_SDA_1P8" },
	{ "VTCAM_AF_EEP_SCL_1P8" },
	{ "VTCAM_AF_EEP_SDA_1P8" },
	{ "WT3X_OIS_T10X_AF_OIS_SCL_1P8" },
	{ "WT3X_OIS_T10X_AF_OIS_SCL_1P8" },
};
#endif
