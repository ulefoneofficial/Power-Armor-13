#ifndef __TM1621D_H__
#define __TM1621D_H__

extern int TM1621D_DEBUG_ENABLE;

#define TM_DBG(format, args...) do { \
	if (TM1621D_DEBUG_ENABLE) \
	{\
		printk(KERN_WARNING format, ##args);\
	} \
} while (0)

#ifndef uchar
#define uchar (unsigned char)				 //数据类型宏定义
#endif

#ifndef uint
#define uint (unsigned int)				 //数据类型宏定义
#endif


/******************TM1621D模块命令定义*********************/
#define SYSDIS   0x00	 //关系统振荡器和LCD偏压发生器	
#define SYSEN    0x02    //打开系统振荡器	

#define LCDOFF   0x04     //关LCD偏压
#define LCDON    0x06     //开LCD偏压
							 
#define RC       0x30     //内部RC振荡	

#define BIAS     0x52     //1/3偏压 4公共口	




#endif

