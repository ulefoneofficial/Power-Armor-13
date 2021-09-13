/*
 * Copyright (C) 2017 MediaTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See http://www.gnu.org/licenses/gpl-2.0.html for more details.
 */

extern void mt_usbhost_connect(void);
extern void mt_usbhost_disconnect(void);
extern void mt_vbus_on(void);
//prize add by lipengpeng 20210309 start
#if defined (CONFIG_PRIZE_REVERE_CHARGING_MODE)
extern void mt_vbus_reverse_on(void);
extern int usb_otg_set_reverse_vbus(int is_on);
extern void mt_vbus_reverse_on_limited_current(void);
extern void mt_vbus_reverse_off_limited_current(void);
extern int usb_otg_set_reverse_vbus_limited_current(int is_on);
#endif
//prize add by lipengpeng 20210309 start
extern void mt_vbus_off(void);
extern int usb_otg_set_vbus(int is_on);
extern void mt_usb_connect(void);
extern void mt_usb_disconnect(void);
#ifndef CONFIG_TCPC_CLASS
#ifdef CONFIG_DUAL_ROLE_USB_INTF
extern void mt_usb_dual_role_to_none(void);
extern void mt_usb_dual_role_to_device(void);
extern void mt_usb_dual_role_to_host(void);
extern int mt_usb_dual_role_init(struct device *dev);
#endif
#endif
