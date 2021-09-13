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
#define FM78100_DRIVER_NAME "exergen-fm78100"

const uint8_t init_register_array_fm78100[][2] = {
	
{0x01, 0x07}, //  temper sensor enable, 0x03 dis
{0x02, 0x33}, //  
{0x03, 0x83}, //  
{0x04, 0x80}, //  
{0x05, 0x80}, // 
{0x06, 0x50}, //  
{0x07, 0x07}, //   
{0x09, 0x03}, //  power down 0x03 ,power on 0x02
{0x11, 0x07}, // 
{0x12, 0x7f}, //  
{0x13, 0x00}, //  
{0x14, 0x00}, //  
{0x15, 0x00}, //  
{0x16, 0x00}, //  
{0xc0, 0x86}, //  
{0xc1, 0x2f}, //  
{0xc2, 0x40}, //  
{0xc4, 0x11}, //default 11  ff
{0xc3, 0x77}, //default 77  ff 
{0x0a, 0x00},//ff  00
{0x0b, 0x80},//ff  40
{0x0c, 0x00},//0f  00
};
#define INIT_ARRAY_SIZE_FM78100 (sizeof(init_register_array_fm78100)/sizeof(init_register_array_fm78100[0]))
