#include <linux/syscalls.h>
#include <linux/types.h>
#include <linux/version.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/i2c.h>
#include <linux/vmalloc.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
//#include <mach/irqs.h>
#include <linux/jiffies.h>
#include <linux/version.h>
#include <linux/types.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/kernel.h>
//#include <linux/rtpm_prio.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/vmalloc.h>

#include <prize_exergen.h>
#define FM78100B_DRIVER_NAME "exergen-fm78100b"
#define FM78100B_CHIP_ID 0x24
