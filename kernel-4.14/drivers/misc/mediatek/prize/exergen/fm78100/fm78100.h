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
#define   FACTORY_TEST   1
#define   TEST_MAX_CNT  10	//64
#define   TEST_AVE_CNT 4	//48

typedef struct {
	int32_t ntc_temp;
	int32_t mems_temp;
	int32_t body_temp;
} fm78100_results_t;

int fm78100_chip_init(void);
void fm78100_chip_reset(void);

bool fm78100_alg_reset(void);
void fm78100_alg_send_data(int32_t ch1_raw_data, int32_t ch2_raw_data);
int32_t fm78100_alg_get_ntc_raw(void);
int32_t fm78100_alg_get_mems_raw(void);
void fm78100_chip_disable(void);
void fm78100_chip_enable(void);

void fm78100_int_handle(void);
void fm78100_alg_config(void);
void fm78100_sort(int8_t length);
void fm78100_cali(void);
void fm78100_mems_cali(int index,int memstp);
void fm78100_mems_cali_enable(bool en);
float fm78100_calc_bobytemp(float ntc_temp_f,float mems_temp_f);
void fm78100_ntc_cali(void);
uint8_t fm78100_chip_id(void);

fm78100_results_t fm78100_alg_get_tp_results(void);
bool fm78100_alg_open(void);
int32_t fm78100_read_ch1(void);
int32_t fm78100_read_ch2(void);

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
