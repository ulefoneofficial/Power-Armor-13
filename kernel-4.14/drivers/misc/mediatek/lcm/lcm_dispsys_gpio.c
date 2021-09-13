#include <linux/string.h>
#include <linux/wait.h>
#include <linux/platform_device.h>
#include <linux/pinctrl/consumer.h>
#include <linux/of_gpio.h>
#include <asm-generic/gpio.h>

#include "lcm_drv.h"
#include "ddp_irq.h"

#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/mm_types.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/gpio.h>

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/regulator/consumer.h>
#include <linux/clk.h>
#endif


static struct pinctrl *dsi_pinctrl1;
static struct pinctrl_state *pins_default, *lcm_rest_en0, *lcm_rest_en1, *lcm_power_dm_en0,*lcm_power_dm_en1,
          *lcm_power_dp_en0,*lcm_power_dp_en1,*lcm_power_enp_en0,*lcm_power_enp_en1,*lcm_power_enn_en0,*lcm_power_enn_en1;
		  
int mt_dsi_get_gpio_info(struct platform_device *pdev)
{
	int ret;

	pr_err("%s\n", __func__);
	dsi_pinctrl1 = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(dsi_pinctrl1)) {
		ret = PTR_ERR(dsi_pinctrl1);
		dev_err(&pdev->dev, "Cannot find lcm pinctrl1!\n");
             return -1;
	}
	pins_default = pinctrl_lookup_state(dsi_pinctrl1, "default");
	if (IS_ERR(pins_default)) {
		ret = PTR_ERR(pins_default);
		dev_err(&pdev->dev, "Cannot find lcm pinctrl default %d!\n", ret);
	}
	lcm_rest_en0 = pinctrl_lookup_state(dsi_pinctrl1, "lcm_rest_en0");
	if (IS_ERR(lcm_rest_en0)) {
		ret = PTR_ERR(lcm_rest_en0);
		dev_err(&pdev->dev, "Cannot find lcm pinctrl lcm_rest_en0!\n");
	}
	lcm_rest_en1 = pinctrl_lookup_state(dsi_pinctrl1, "lcm_rest_en1");
	if (IS_ERR(lcm_rest_en1)) {
		ret = PTR_ERR(lcm_rest_en1);
		dev_err(&pdev->dev, "Cannot find lcm pinctrl lcm_rest_en1!\n");
	}

      lcm_power_dm_en0 = pinctrl_lookup_state(dsi_pinctrl1, "lcm_power_dm_en0");
      if (IS_ERR(lcm_power_dm_en0)) {
          ret = PTR_ERR(lcm_power_dm_en0);
          dev_err(&pdev->dev, "Cannot find lcm pinctrl lcm_power_dm_en0!\n");
      }
      lcm_power_dm_en1 = pinctrl_lookup_state(dsi_pinctrl1, "lcm_power_dm_en1");
      if (IS_ERR(lcm_power_dm_en1)) {
          ret = PTR_ERR(lcm_power_dm_en1);
          dev_err(&pdev->dev, "Cannot find lcm pinctrl lcm_power_dm_en1!\n");
      }

      lcm_power_dp_en0 = pinctrl_lookup_state(dsi_pinctrl1, "lcm_power_dp_en0");
      if (IS_ERR(lcm_power_dp_en0)) {
          ret = PTR_ERR(lcm_power_dp_en0);
          dev_err(&pdev->dev, "Cannot find lcm pinctrl lcm_power_dp_en0!\n");
      }
      lcm_power_dp_en1 = pinctrl_lookup_state(dsi_pinctrl1, "lcm_power_dp_en1");
      if (IS_ERR(lcm_power_dp_en1)) {
          ret = PTR_ERR(lcm_power_dp_en1);
          dev_err(&pdev->dev, "Cannot find lcm pinctrl lcm_power_dp_en1!\n");
      }

      lcm_power_enp_en0 = pinctrl_lookup_state(dsi_pinctrl1, "lcm_power_enp_en0");
      if (IS_ERR(lcm_power_enp_en0)) {
          ret = PTR_ERR(lcm_power_enp_en0);
          dev_err(&pdev->dev, "Cannot find lcm pinctrl lcm_power_enp_en0!\n");
      }
      lcm_power_enp_en1 = pinctrl_lookup_state(dsi_pinctrl1, "lcm_power_enp_en1");
      if (IS_ERR(lcm_power_enp_en1)) {
          ret = PTR_ERR(lcm_power_enp_en1);
          dev_err(&pdev->dev, "Cannot find lcm pinctrl lcm_power_enp_en1!\n");
      }
 
      lcm_power_enn_en0 = pinctrl_lookup_state(dsi_pinctrl1, "lcm_power_enn_en0");
      if (IS_ERR(lcm_power_enn_en0)) {
          ret = PTR_ERR(lcm_power_enn_en0);
          dev_err(&pdev->dev, "Cannot find lcm pinctrl lcm_power_enn_en0!\n");
      }
      lcm_power_enn_en1 = pinctrl_lookup_state(dsi_pinctrl1, "lcm_power_enn_en1");
      if (IS_ERR(lcm_power_enn_en1)) {
          ret = PTR_ERR(lcm_power_enn_en1);
          dev_err(&pdev->dev, "Cannot find lcm pinctrl lcm_power_enn_en1!\n");
      }    
	  
	  //pinctrl_select_state(dsi_pinctrl1, lcm_power_enn_en0); 
	  //pinctrl_select_state(dsi_pinctrl1, lcm_power_enp_en0); 
	    
      return 0;

}

int mt_dsi_pinctrl_set(unsigned int pin , unsigned int level)
{
    int ret =-1;

	switch(pin)
	{
	case LCM_RESET_PIN_NO: 	
           if(IS_ERR(lcm_rest_en0)||IS_ERR(lcm_rest_en1))
           {
                pr_err( "err: lcm_rest_en0 or lcm_rest_en1 is error!!!");
                return ret;
           }   
           else
            break;
	case LCM_POWER_DM_NO: 	
        if( IS_ERR(lcm_power_dm_en0)|| IS_ERR(lcm_power_dm_en1))
        {
             pr_err("err: led1_green_gpio_en0 or led1_green_gpio_en1 is error!!!");
             return ret;
        }   
        else
         break;
      case LCM_POWER_DP_NO:   
          if( IS_ERR(lcm_power_dp_en0)|| IS_ERR(lcm_power_dp_en1))
          {
               pr_err("err: lcm_power_dp_en0 or lcm_power_dp_en1 is error!!!");
               return ret;
          }   
          else
           break;

      case LCM_POWER_ENP_NO:   
          if( IS_ERR(lcm_power_enp_en0)|| IS_ERR(lcm_power_enp_en1))
          {
               pr_err("err: lcm_power_enp_en0 or lcm_power_enp_en1 is error!!!");
               return ret;
          }   
          else
           break;

      case LCM_POWER_ENN_NO:   
          if( IS_ERR(lcm_power_enn_en0)|| IS_ERR(lcm_power_enn_en1))
          {
               pr_err("err: lcm_power_enn_en0 or lcm_power_enn_en1 is error!!!");
               return ret;
          }   
          else
           break;

	default:
          {
              pr_err("err: leds_gpio_output pin[%d] is error!!, drv need to check!", pin);
              return -1;
           }
	}
      

	switch(pin)
	{
		case LCM_RESET_PIN_NO: 	pinctrl_select_state(dsi_pinctrl1, level ? lcm_rest_en1      : lcm_rest_en0);      break; 
		case LCM_POWER_DP_NO: 	pinctrl_select_state(dsi_pinctrl1, level ? lcm_power_dp_en1  : lcm_power_dp_en0);  break; 
		case LCM_POWER_DM_NO:   pinctrl_select_state(dsi_pinctrl1, level ? lcm_power_dm_en1  : lcm_power_dm_en0);  break; 
		case LCM_POWER_ENP_NO:  pinctrl_select_state(dsi_pinctrl1, level ? lcm_power_enp_en1 : lcm_power_enp_en0); break; 
		case LCM_POWER_ENN_NO:  pinctrl_select_state(dsi_pinctrl1, level ? lcm_power_enn_en1 : lcm_power_enn_en0); break; 	
		default: ;
	}
      return 0;

}
EXPORT_SYMBOL_GPL(mt_dsi_pinctrl_set);

static int lcm_gpio_probe(struct platform_device *dev)
{
	pr_err("%s\n", __func__);
	mt_dsi_get_gpio_info(dev);
	return 0;
}
static const struct of_device_id lcm_gpio_of_ids[] = {
	{.compatible = "prize,dispsys_gpio",},
	{}
};

static struct platform_driver lcm_gpio_driver = {
	.driver = {
		   .name = "prize_lcm",
	#ifdef CONFIG_OF
		   .of_match_table = lcm_gpio_of_ids,
	#endif
		   },
	.probe = lcm_gpio_probe,
};


static int __init lcm_gpio_init(void)
{
	pr_err("%s\n", __func__);
	if (platform_driver_register(&lcm_gpio_driver) != 0) {
		pr_err("unable to register LCM GPIO driver.\n");
		return -1;
	}
	return 0;
}

static void __exit lcm_gpio_exit(void)
{
	pr_err("%s\n", __func__);
	platform_driver_unregister(&lcm_gpio_driver);

}
module_init(lcm_gpio_init);
module_exit(lcm_gpio_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("prize LCM GPIO driver");
MODULE_AUTHOR("Pan fei<panfei@szprize.com>");
