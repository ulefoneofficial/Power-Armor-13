

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>

#include <linux/poll.h>
#include <linux/fb.h>
#include <linux/slab.h>
#include <linux/miscdevice.h>
#include <linux/delay.h>

#include<linux/timer.h> 
#include<linux/jiffies.h>

//#include "lcm_drv.h"
#include "hardware_info.h"
#include "prize_custom_memory.h"
#include <linux/fs.h>

#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/fb.h>

#define DEBUG_ON		0
#define HW_PRINT(fmt,arg...)           printk("[HW_INFO] "fmt"\n",##arg)
#define HW_ERROR(fmt,arg...)          printk("[HW_INFO] ERROR:"fmt"\n",##arg)
#define HW_DEBUG(fmt,arg...)          do{\
	if(DEBUG_ON)\
		printk("[HW_INFO] [%d]"fmt"\n",__LINE__, ##arg);\
		}while(0)


#if defined(CONFIG_MTK_AUXADC)
extern int IMM_get_adc_channel_num(char *channel_name, int len);
extern int IMM_GetOneChannelValue_Cali(int Channel, int *voltage);
#endif
#if defined(CONFIG_MEDIATEK_MT6577_AUXADC)
extern int lcm_auxadc_get_lcm_v(void);
#endif

int len = 0;
static struct notifier_block fb_nb;

struct class *hardware_info_class;


struct hardware_info current_lcm_info =
{
	"unknow","unknow","unknow","unknow",
};
struct hardware_info current_camera_info[5] = 
{
	{"unknow","unknow","unknow","unknow"},
	{"unknow","unknow","unknow","unknow"},
	{"unknow","unknow","unknow","unknow"},
	{"unknow","unknow","unknow","unknow"},
	{"unknow","unknow","unknow","unknow"},
};
struct hardware_info current_tp_info =
{
	"unknow","unknow","unknow","unknow",
};
//prize-wangyongsheng-2021330-for nfc start 
struct hardware_info current_nfc_info =
{
	"unknow","unknow","unknow","unknow",
};
//prize-wangyongsheng-2021330-for nfc end

struct hardware_info current_alsps_info =
{
	"unknow","unknow","unknow","unknow",
};

struct hardware_info current_gsensor_info =
{
	"unknow","unknow","unknow","unknow",
};
//prize-wangyongsheng-2021330-for gyroscope start 
struct hardware_info current_gyroscope_info =
{
	"unknow","unknow","unknow","unknow",
};
//prize-wangyongsheng-2021330-for gyroscope end 	
struct hardware_info current_msensor_info =
{
	"unknow","unknow","unknow","unknow",
};
struct hardware_info current_barosensor_info =
{
	"unknow","unknow","unknow","unknow",
};
//prize add by lipengpeng 20210820 start 
struct hardware_info current_wireless_info =
{
	"unknow","unknow","unknow","unknow",
};
//prize add by lipengpeng 20210820 end 

struct hardware_info current_fingerprint_info =
{
	"unknow","unknow","unknow","unknow",
};
#if defined(CONFIG_PRIZE_HARDWARE_INFO_BAT)
struct hardware_info current_battery_info =
{
	"unknow","unknow","unknow","unknow","unknow",
};
EXPORT_SYMBOL_GPL(current_battery_info);
#endif
struct hardware_info current_coulo_info =
{
	"unknow","unknow","unknow","unknow",
};
/* prize addded by wangmengdong for hardware info, ufs life, 20210206,start */
struct hardware_info current_mmc_info =
{
	"unknow","unknow","unknow","unknow",
};
/* prize addded by wangmengdong for hardware info, ufs life, 20210206,end */

EXPORT_SYMBOL_GPL(current_lcm_info);
EXPORT_SYMBOL_GPL(current_camera_info);
EXPORT_SYMBOL_GPL(current_tp_info);
//prize-wangyongsheng-20210330-start
EXPORT_SYMBOL_GPL(current_nfc_info);
//prize-wangyongsheng-20210330-end
EXPORT_SYMBOL_GPL(current_alsps_info);
EXPORT_SYMBOL_GPL(current_gsensor_info);
//prize-wangyongsheng-2021330-for gyroscope start 
EXPORT_SYMBOL_GPL(current_gyroscope_info);
//prize-wangyongsheng-2021330-for gyroscope end 
EXPORT_SYMBOL_GPL(current_msensor_info);
EXPORT_SYMBOL_GPL(current_barosensor_info);
EXPORT_SYMBOL_GPL(current_fingerprint_info);
//mt_battery_meter.h
static void dev_get_current_lcm_info(char *buf)
{
    char *p = buf;
#if defined(CONFIG_DRM_MEDIATEK)||defined(CONFIG_MTK_AUXADC)
	int ret = 0;
#endif
#if defined(CONFIG_MEDIATEK_MT6577_AUXADC)||defined(CONFIG_SC27XX_ADC)||defined(CONFIG_MTK_AUXADC)
	int lcm_volt = 0;
#endif
#if defined(CONFIG_MTK_AUXADC)
	int lcm_volt_ch = 0;
#endif
#if defined(CONFIG_DRM_MEDIATEK)
	struct device_node *dsi_node, *remote_node = NULL, *endpoint = NULL;
	const char *compatible_str = NULL;
#endif

	HW_PRINT("hardware_info_lcm");

	if(strcmp(current_lcm_info.chip,"unknow") == 0){
	#if defined(CONFIG_DRM_MEDIATEK)
		dsi_node = of_find_compatible_node(NULL,NULL,"mediatek,dsi0");
		if (!IS_ERR_OR_NULL(dsi_node)){
			endpoint = of_graph_get_next_endpoint(dsi_node, NULL);
			if (endpoint) {
				remote_node = of_graph_get_remote_port_parent(endpoint);
				if (!remote_node) {
					HW_PRINT("No panel connected");
					//return -ENODEV;
				}else{
					ret = of_property_read_string(remote_node, "compatible", &compatible_str);
					if (!ret){
						snprintf(&current_lcm_info.chip[0], sizeof(current_lcm_info.chip), compatible_str);
					}
				}
			}
		}else{
			HW_PRINT("get dsi0 node fail");
		}
	#else
	 	return;
	#endif
	}


	p += sprintf(p, "[LCM]:\n");
	p += sprintf(p, "  chip:%s\n", current_lcm_info.chip);
	if(strcmp(current_lcm_info.id,"unknow") != 0){
		p += sprintf(p, "  id:%s\n", current_lcm_info.id);
	}
	if(strcmp(current_lcm_info.vendor,"unknow") != 0){
		p += sprintf(p, "  vendor:%s\n",current_lcm_info.vendor);
	}
	p += sprintf(p, "  more:%s", current_lcm_info.more);

#if defined(CONFIG_MTK_AUXADC)
	lcm_volt_ch = IMM_get_adc_channel_num("ADC_LCM_VOLTAGE",strlen("ADC_LCM_VOLTAGE"));
	if (lcm_volt_ch >= 0){
		ret = IMM_GetOneChannelValue_Cali(lcm_volt_ch, &lcm_volt);
		if (!ret) {
			p += sprintf(p, "  %du", lcm_volt);
		}
	}
#endif
#if defined(CONFIG_MEDIATEK_MT6577_AUXADC)||defined(CONFIG_SC27XX_ADC)
	lcm_volt = lcm_auxadc_get_lcm_v();
	if (lcm_volt >= 0)//prize-Solve LCM information display - 19m error-pengzhipeng-20210625
	p += sprintf(p, "  %dm", lcm_volt);
#endif
	
	p += sprintf(p, "\n");

	len += (p - buf);
	HW_PRINT("%s",buf);
}
static void dev_get_current_camera_info(char *buf)
{
     char *p = buf;
	 HW_PRINT("dev_get_current_camera_info");
	 
	 
	 if(strcmp(current_camera_info[0].chip,"unknow") != 0)
	 {	 	   
	 	p += sprintf(p, "\n[Main Camera]:\n");	
	 	p += sprintf(p, "  chip:%s\n", current_camera_info[0].chip);	
	 	p += sprintf(p, "  id:%s\n", current_camera_info[0].id);	
	 	p += sprintf(p, "  vendor:%s\n",current_camera_info[0].vendor);		
	 	p += sprintf(p, "  more:%s\n", current_camera_info[0].more);
	 }

	 if(strcmp(current_camera_info[1].chip,"unknow") != 0)
	 {
		p += sprintf(p, "\n[Sub Camera]:\n");	
	 	p += sprintf(p, "  chip:%s\n", current_camera_info[1].chip);	
	 	p += sprintf(p, "  id:%s\n", current_camera_info[1].id);	
	 	p += sprintf(p, "  vendor:%s\n",current_camera_info[1].vendor);		
	 	p += sprintf(p, "  more:%s\n", current_camera_info[1].more);
	 }

	 if(strcmp(current_camera_info[2].chip,"unknow") != 0)
	 {
		p += sprintf(p, "\n[Main2 Camera]:\n");	
	 	p += sprintf(p, "  chip:%s\n", current_camera_info[2].chip);	
	 	p += sprintf(p, "  id:%s\n", current_camera_info[2].id);	
	 	p += sprintf(p, "  vendor:%s\n",current_camera_info[2].vendor);		
	 	p += sprintf(p, "  more:%s\n", current_camera_info[2].more);
	 }
	 
	 if(strcmp(current_camera_info[3].chip,"unknow") != 0)
	 {
		p += sprintf(p, "\n[Main3 Camera]:\n");	
	 	p += sprintf(p, "  chip:%s\n", current_camera_info[3].chip);	
	 	p += sprintf(p, "  id:%s\n", current_camera_info[3].id);	
	 	p += sprintf(p, "  vendor:%s\n",current_camera_info[3].vendor);		
	 	p += sprintf(p, "  more:%s\n", current_camera_info[3].more);
	 }
	 if(strcmp(current_camera_info[4].chip,"unknow") != 0)
	 {
		p += sprintf(p, "\n[Main4 Camera]:\n");	
	 	p += sprintf(p, "  chip:%s\n", current_camera_info[4].chip);	
	 	p += sprintf(p, "  id:%s\n", current_camera_info[4].id);	
	 	p += sprintf(p, "  vendor:%s\n",current_camera_info[4].vendor);		
	 	p += sprintf(p, "  more:%s\n", current_camera_info[4].more);
	 }
	 len += (p - buf);  
	 HW_PRINT("%s",buf);
}
static void  dev_get_current_tp_info(char *buf)
{	

    char *p = buf;	
	HW_PRINT("dev_get_current_tp_info");
	if(strcmp(current_tp_info.chip,"unknow") == 0)
	 	return ;
 
	    
	 p += sprintf(p, "\n[Touch Panel]:\n");	
	 p += sprintf(p, "  chip:%s\n", current_tp_info.chip);	
	 p += sprintf(p, "  id:%s\n", current_tp_info.id);	
	 p += sprintf(p, "  vendor:%s\n",current_tp_info.vendor);		
	 p += sprintf(p, "  more:%s\n", current_tp_info.more);

	 len += (p - buf);  
	 HW_PRINT("%s",buf);
	
}
//prize-wangyongsheng-2021330-for nfc start 
static void  dev_get_current_nfc_info(char *buf)
{	

    char *p = buf;	
	HW_PRINT("dev_get_current_nfc_info");
	if(strcmp(current_nfc_info.chip,"unknow") == 0)
	 	return ;
 
	    
	 p += sprintf(p, "\n[Nfc]:\n");	
	 p += sprintf(p, "  chip:%s\n", current_nfc_info.chip);	
	 p += sprintf(p, "  id:%s\n", current_nfc_info.id);	
	 p += sprintf(p, "  vendor:%s\n",current_nfc_info.vendor);		
	 p += sprintf(p, "  more:%s\n", current_nfc_info.more);

	 len += (p - buf);  
	 HW_PRINT("%s",buf);
	
}
//prize-wangyongsheng-2021330-for nfc start 
static void dev_get_current_alsps_info(char *buf)
{
    
	 char *p = buf;	  
	HW_PRINT("dev_get_current_alsps_info");
	if(strcmp(current_alsps_info.chip,"unknow") == 0)
	 	return ;	
 
	 p += sprintf(p, "\n[ALS/PS]:\n");	
	 p += sprintf(p, "  chip:%s\n", current_alsps_info.chip);	
	 p += sprintf(p, "  id:%s\n", current_alsps_info.id);	
	 p += sprintf(p, "  vendor:%s\n",current_alsps_info.vendor);		
	 p += sprintf(p, "  more:%s\n", current_alsps_info.more);

	 len += (p - buf);  
	 HW_PRINT("%s",buf);
}

static void dev_get_current_gsensor_info(char *buf)
{
    char *p = buf;	  
	HW_PRINT("dev_get_current_gsensor_info");
	if(strcmp(current_gsensor_info.chip,"unknow") == 0)
	 	return ;		

	  
	 p += sprintf(p, "\n[G-sensor]:\n");	
	 p += sprintf(p, "  chip:%s\n", current_gsensor_info.chip);	
	 p += sprintf(p, "  id:%s\n", current_gsensor_info.id);	
	 p += sprintf(p, "  vendor:%s\n",current_gsensor_info.vendor);		
	 p += sprintf(p, "  more:%s\n", current_gsensor_info.more);

	 len += (p - buf);  
	 HW_PRINT("%s",buf);
}
//prize-wangyongsheng-2021330-for gyroscope start 
static void dev_get_current_gyroscope_info(char *buf)
{
    char *p = buf;	  
	HW_PRINT("dev_get_current_gyroscope_info");
	if(strcmp(current_gyroscope_info.chip,"unknow") == 0)
	 	return ;		

	  
	 p += sprintf(p, "\n[Gyroscope]:\n");	
	 p += sprintf(p, "  chip:%s\n", current_gyroscope_info.chip);	
	 p += sprintf(p, "  id:%s\n", current_gyroscope_info.id);	
	 p += sprintf(p, "  vendor:%s\n",current_gyroscope_info.vendor);		
	 p += sprintf(p, "  more:%s\n", current_gyroscope_info.more);

	 len += (p - buf);  
	 HW_PRINT("%s",buf);
}
//prize-wangyongsheng-2021330-for gyroscope end 
static void dev_get_current_msensor_info(char *buf)
{
      
	 char *p = buf;	
	HW_PRINT("dev_get_current_msensor_info");
	if(strcmp(current_msensor_info.chip,"unknow") == 0)
	 	return ;		
   
	 p += sprintf(p, "\n[M-sensor]:\n");	
	 p += sprintf(p, "  chip:%s\n", current_msensor_info.chip);	
	 p += sprintf(p, "  id:%s\n", current_msensor_info.id);	
	 p += sprintf(p, "  vendor:%s\n",current_msensor_info.vendor);		
	 p += sprintf(p, "  more:%s\n", current_msensor_info.more);

	 len += (p - buf);  
	 HW_PRINT("%s",buf);
}
static void dev_get_current_barosensor_info(char *buf)
{
      
	 char *p = buf;	
	HW_PRINT("dev_get_current_barosensor_info");
	if(strcmp(current_barosensor_info.chip,"unknow") == 0)
	 	return ;		
   
	 p += sprintf(p, "\n[BARO-sensor]:\n");	
	 p += sprintf(p, "  chip:%s\n", current_barosensor_info.chip);	
	 p += sprintf(p, "  id:%s\n", current_barosensor_info.id);	
	 p += sprintf(p, "  vendor:%s\n",current_barosensor_info.vendor);		
	 p += sprintf(p, "  more:%s\n", current_barosensor_info.more);

	 len += (p - buf);  
	 HW_PRINT("%s",buf);
}

static void dev_get_current_fingerprint_info(char *buf)
{
     char *p = buf;
	HW_PRINT("dev_get_current_fingerprint_info");
	if(strcmp(current_fingerprint_info.chip,"unknow") == 0)
		return ;

	
	 p += sprintf(p, "\n[Fingerprint]:\n");
	 p += sprintf(p, "  chip:%s\n", current_fingerprint_info.chip);
	 p += sprintf(p, "  id:%s\n", current_fingerprint_info.id);
	 p += sprintf(p, "  vendor:%s\n",current_fingerprint_info.vendor);
	 p += sprintf(p, "  more:%s\n", current_fingerprint_info.more);

	 len += (p - buf);
	 HW_PRINT("%s",buf);
}
#if defined(CONFIG_PRIZE_HARDWARE_INFO_BAT)
static void dev_get_current_battery_info(char *buf)
{
     char *p = buf;
	HW_PRINT("dev_get_current_battery_info");
	//if(strcmp(current_battery_info.chip,"unknow") == 0)
	//	return ;

	
	 p += sprintf(p, "\n[battery]:\n");
	 p += sprintf(p, " batt_vendor:%s\n",current_battery_info.batt_versions);
	 p += sprintf(p, " Q_MAX_POS_50:%s\n",current_battery_info.Q_MAX_50);
	 p += sprintf(p, " Q_MAX_POS_25:%s\n",current_battery_info.Q_MAX_25);
	 p += sprintf(p, " Q_MAX_POS_0:%s\n",current_battery_info.Q_MAX_0);
	 p += sprintf(p, " Q_MAX_NEG_10:%s\n",current_battery_info.Q_MAX_10);
	

	 len += (p - buf);
	 HW_PRINT("%s",buf);
}
#endif

static void dev_get_current_coulo_info(char *buf)
{
     char *p = buf;
	HW_PRINT("dev_get_current_coulo_info");
	if(strcmp(current_coulo_info.chip,"unknow") == 0)
		return ;

	
	 p += sprintf(p, "\n[coulo]:\n");
	 p += sprintf(p, "  chip:%s\n", current_coulo_info.chip);
	 p += sprintf(p, "  id:%s\n", current_coulo_info.id);
	 p += sprintf(p, "  vendor:%s\n",current_coulo_info.vendor);
	 p += sprintf(p, "  more:%s\n", current_coulo_info.more);

	 len += (p - buf);
	 HW_PRINT("%s",buf);
}

#ifdef CONFIG_PRIZE_CTS
extern int sec_schip_enabled(void);
static void dev_get_efuse_status(char *buf)
{
     char *p = buf;
	HW_PRINT("dev_get_efuse_status");
	//if(strcmp(current_battery_info.chip,"unknow") == 0)
	//	return ;
	
	 p += sprintf(p, "\n[EFUSE]:\n");

	if(sec_schip_enabled())
		p += sprintf(p, " Status: eFuse blown!\n");
	else
		p += sprintf(p, " Status: eFuse not blown!\n");

	 len += (p - buf);
	 HW_PRINT("%s",buf);
}
#endif

//prize add by lipengpeng 20210820 start current_wireless_info
#if defined(CONFIG_PRIZE_MT5725_SUPPORT_15W)
static void dev_get_wireless_version(char *buf)
{
     char *p = buf;
	HW_PRINT("dev_get_wireless_version");
	if(strcmp(current_wireless_info.chip,"unknow") == 0)
		return ;

	
	 p += sprintf(p, "\n[wireless]:\n");
	 p += sprintf(p, "  chip:%s\n", current_wireless_info.chip);
	 p += sprintf(p, "  id:%s\n", current_wireless_info.id);
	 p += sprintf(p, "  vendor:%s\n",current_wireless_info.vendor);
	 p += sprintf(p, "  more:%s\n", current_wireless_info.more);

	 len += (p - buf);
	 HW_PRINT("%s",buf);
}
#endif
//prize add by lipengpeng 20210820 end  

static ssize_t hardware_info_store(struct device *dev, struct device_attribute *attr,const char *buf, size_t size)
{
	HW_PRINT("hardware_info_store buf:%s,size:%d=======",buf,(int)size);
	
	return size;
	
}

extern char *saved_command_line;
int get_lpddr_emmc_used_index(void)
{
     char *ptr;
     int lpddr_index=0;
     ptr=strstr(saved_command_line,"lpddr_used_index=");
     if(ptr==NULL)
	     return -1;
     ptr+=strlen("lpddr_used_index=");
     lpddr_index=simple_strtol(ptr,NULL,10);
     return lpddr_index;
} 

static void dev_get_current_flash_lpddr_index_info(char *buf)
{
    
	 char *p = buf;	
	 int flash_lpddr_index =-1;  
/* prize addded by wangmengdong for hardware info, ufs life, 20210206,start */
#if defined(CONFIG_MTK_UFS_SUPPORT)
	const char *life_time;
	const char *pre_eol_info;
	char *path = "/memory";
	struct device_node *dt_node;
	int LifeEstA,LifeEstB,PreEOL;
	int ret;
		dt_node = of_find_node_by_path(path);
	if (!dt_node) {
		HW_PRINT("of_find_node_by_path fail \n");
	}
	ret = of_property_read_string(dt_node, "life_time", &life_time);
	ret = sscanf(life_time,"%x %x",&LifeEstA,&LifeEstB);
	if (ret != 2) {
		HW_PRINT("read all health_status fail \n");
	}
	HW_PRINT("of_property_read_string LifeEstA=%x LifeEstB=%x\n",LifeEstA,LifeEstB);
	
	ret = of_property_read_string(dt_node, "pre_eol_info", &pre_eol_info);
	ret = sscanf(pre_eol_info,"%x",&PreEOL);
	if (ret != 1) {
		HW_PRINT("read all health_status fail \n");
	}
	HW_PRINT("of_property_read_string  PreEOL=%x \n",PreEOL);

#endif
/* prize addded by wangmengdong for hardware info, ufs life, 20210206,end */
	HW_PRINT("dev_get_flash_info");
	
//	if(strcmp(current_alsps_info.chip,"unknow") == 0)
	 //	return ;	
     flash_lpddr_index=get_lpddr_emmc_used_index();
	 p += sprintf(p, "\n[flash]:\n");	
	 p += sprintf(p, " %s\n",Cust_emmc_support[flash_lpddr_index]);
/* prize addded by wangmengdong for hardware info, ufs life, 20210206,start */
#if defined(CONFIG_MTK_UFS_SUPPORT)
	p += sprintf(p, "LifeEstA=0x%02x LifeEstB=0x%02x\n",LifeEstA,LifeEstB);
	 p += sprintf(p, "EOL=0x%02X\n", PreEOL);
#else
	 p += sprintf(p, "LifeEstA=%s LifeEstB=%s\n",current_mmc_info.chip,current_mmc_info.vendor);
	 p += sprintf(p, "EOL=%s\n",current_mmc_info.id);
#endif
/* prize addded by wangmengdong for hardware info, ufs life, 20210206,end */
	 	
	 len += (p - buf);  
	 HW_PRINT("%s",buf);
}

static void dev_get_AudioParam_version_info(char *buf)
{

	   char *p = buf;
	   struct file *fp = NULL;
	   mm_segment_t fs;
	   loff_t pos;
	   char databuf[100]={0};
	   int ret = -1;
	   
	   HW_PRINT("hardware_info_store hello enter\n");
	   fp = filp_open("/vendor/etc/audio_param/AudioParamVersionInfo.txt", O_RDONLY, 0664);
	   if (IS_ERR(fp)){
			HW_ERROR("open AudioParamVersionInfo.txt file error\n");
			return;
	   }
	   fs = get_fs();
	   set_fs(KERNEL_DS);
	   pos =0;
	   ret = vfs_read(fp, databuf, sizeof(databuf), &pos);
	   HW_PRINT("hardware_info_store read ret: %d\n",ret);
	   filp_close(fp,NULL);
	   set_fs(fs);
	   
	   p += sprintf(p, "\n[AudioParamVersionInfo]:\n");
	   p += sprintf(p, "%s\n", databuf);
	   
	   len += (p - buf);
	   HW_PRINT("%s",buf);
}

static ssize_t hardware_info_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	len = 0;
	HW_PRINT("hardware_info_show=======");
	dev_get_current_lcm_info(buf + len);

	dev_get_current_camera_info(buf + len);

	dev_get_current_tp_info(buf + len);
	
	dev_get_current_nfc_info(buf + len);

	dev_get_current_alsps_info(buf + len);

	dev_get_current_gsensor_info(buf + len);
	
	dev_get_current_gyroscope_info(buf + len);	

	dev_get_current_msensor_info(buf + len);
	dev_get_current_barosensor_info(buf + len);

	dev_get_current_fingerprint_info(buf + len);
	dev_get_current_coulo_info(buf + len);
#if defined(CONFIG_PRIZE_HARDWARE_INFO_BAT)
	dev_get_current_battery_info(buf + len);
#endif
#ifdef CONFIG_PRIZE_CTS	
	dev_get_efuse_status(buf + len);
#endif

//prize add by lipengpeng 20210820 start 
#if defined(CONFIG_PRIZE_MT5725_SUPPORT_15W)
	dev_get_wireless_version(buf + len);
#endif
//prize add by lipengpeng 20210820 end 

	dev_get_current_flash_lpddr_index_info(buf + len);

	dev_get_AudioParam_version_info(buf + len);

	return len;

}

static DEVICE_ATTR(hw_info_read, 0664, hardware_info_show, hardware_info_store);

//prize add by lipengpeng 20200315 start
#ifdef CONFIG_PRIZE_TYPEC_POSITIVE_NEGATIVE
extern int otgdetection;
static ssize_t otgtypec_detection_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	len = 0;
	HW_PRINT("otgtypec_detection_show=======otgdetection=%d, len=%d\n",otgdetection,len);
	
    return sprintf(buf, "%d\n", otgdetection);

}

static ssize_t otgtypec_detection_store(struct device *dev, struct device_attribute *attr,const char *buf, size_t size)
{
	HW_PRINT("otgtypec_detection_store buf:%s,size:%d=======",buf,(int)size);
	
	return size;
	
}
static DEVICE_ATTR(otg_detection_read, 0664, otgtypec_detection_show, otgtypec_detection_store);
//prize add by lipengpeng 20200315 end


//prize add by lipengpeng 20200324 start
extern int typeccharge_det;
static ssize_t typec_charge_detection_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	len = 0;
	HW_PRINT("otgtypec_detection_show=======typeccharge_det=%d, len=%d\n",typeccharge_det,len);
	
    return sprintf(buf, "%d\n", typeccharge_det);

}

static ssize_t typec_charge_detection_store(struct device *dev, struct device_attribute *attr,const char *buf, size_t size)
{
	HW_PRINT("otgtypec_detection_store buf:%s,size:%d=======",buf,(int)size);
	
	return size;
	
}
static DEVICE_ATTR(typec_charge_det, 0664, typec_charge_detection_show, typec_charge_detection_store);
//prize add by lipengpeng 20200324 end
#endif
static int fb_notifier_callback(struct notifier_block *nb, unsigned long event, void *data){
	struct fb_event *fb_event = data;
	struct fb_var_screeninfo *scr_info = NULL;

	if (event != FB_EVENT_FB_REGISTERED){
		return 0;
	}

	if (fb_event->info){
		scr_info = &fb_event->info->var;
		snprintf(&current_lcm_info.more[0], sizeof(current_lcm_info.more),
				"%dx%d", scr_info->xres, scr_info->yres);
	}

	return 0;
}

static int __init hardware_info_dev_init(void) {

	struct device *hardware_info_dev;
	hardware_info_class = class_create(THIS_MODULE, "hw_info");
	
	if (IS_ERR(hardware_info_class)) {
		HW_ERROR("Failed to create class(hardware_info)!");
		return PTR_ERR(hardware_info_class);
	}

	hardware_info_dev = device_create(hardware_info_class, NULL, 0, NULL, "hw_info_data");
	if (IS_ERR(hardware_info_dev))
		HW_ERROR("Failed to create hardware_info_dev device");
	
	if (device_create_file(hardware_info_dev, &dev_attr_hw_info_read) < 0)
		HW_ERROR("Failed to create device file(%s)!",dev_attr_hw_info_read.attr.name);	
//prize add by lipengpeng 20200315 start
#ifdef CONFIG_PRIZE_TYPEC_POSITIVE_NEGATIVE
	if (device_create_file(hardware_info_dev, &dev_attr_otg_detection_read) < 0)
		HW_ERROR("Failed to create device file(%s)!",dev_attr_otg_detection_read.attr.name);
//prize add by lipengpeng 20200315 end


//prize add by lipengpeng 20200324 start
	if (device_create_file(hardware_info_dev, &dev_attr_typec_charge_det) < 0)
		HW_ERROR("Failed to create device file(%s)!",dev_attr_typec_charge_det.attr.name);
//prize add by lipengpeng 20200324 end
#endif	
	fb_nb.notifier_call = fb_notifier_callback;
	if (fb_register_client(&fb_nb)){
		HW_ERROR("register fb client fail\n");
	}

	HW_PRINT("hardware_info initialized ok ");
	return 0;
}

static void __exit hardware_info_dev_exit(void) 
{
	class_destroy(hardware_info_class);
}

subsys_initcall_sync(hardware_info_dev_init);
module_exit(hardware_info_dev_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("lixuefeng <lixuefeng@boruizhiheng.com>");
MODULE_DESCRIPTION("show hardware info Driver");

