#ifndef _AW86014_H_
#define _AW86014_H_

#define AF_DRVNAME		"AW86014AF_DRV"

/* Debug Info Control */
//#define AW_AF_DEBUG
#ifdef AW_AF_DEBUG
#define AW_LOG_INF(format, args...) \
	pr_info(AF_DRVNAME " [%s] " format, __func__, ##args)
#else
#define AW_LOG_INF(format, args...)
#endif
#define AW_LOG_ERR(format, args...) \
	pr_err(AF_DRVNAME " [%s] " format, __func__, ##args)

/* Advance Mode */
#define AW_ADVANCE_VRC
#define AW_ADVANCE_VRC_MODE	0x88 /* VRC3.5 */
#define AW_ADVANCE_RPESC	0x00
#define AW_ADVANCE_VRCT		0x00

/* Register */
#define REG_VCM_MSB		0x03
#define REG_VCM_LSB		0x04
/* Init Position */
#define AW_INIT_POS_H		0x00
#define AW_INIT_POS_L		0x00

#endif
