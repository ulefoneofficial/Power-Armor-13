// SPDX-License-Identifier: GPL-2.0
//
// Copyright (C) 2018 MediaTek Inc.

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/i2c.h>
/* alsa sound header */
#include <sound/soc.h>
#include <sound/pcm_params.h>

#if defined(CONFIG_SND_SOC_MTK_AUDIO_DSP)
#include "audio_task.h"
#include "../audio_dsp/mtk-dsp-common_define.h"
#include "audio_messenger_ipi.h"
#endif

#include "mtk-sp-common.h"
#include "mtk-sp-spk-amp.h"
#if defined(CONFIG_SND_SOC_RT5509)
#include "../../codecs/rt5509.h"
#endif
#ifdef CONFIG_SND_SOC_MT6660
#include "../../codecs/mt6660.h"
#endif /* CONFIG_SND_SOC_MT6660 */

#ifdef CONFIG_SND_SOC_TFA9874
#include "../../codecs/tfa98xx/inc/tfa98xx_ext.h"
#endif

#ifdef CONFIG_SND_SOC_AW87339
#include "aw87339.h"
#endif

/* prize added by chenjiaxi, add aw87519, 20200918-start */
#ifdef CONFIG_SND_SOC_AW87519
#include "aw87519_audio.h"
#endif
/* prize added by chenjiaxi, add aw87519, 20200918-end */
/* prize modified by lifenfen, add awinic smartpa aw8898, 20200103 begin */
#ifdef CONFIG_SND_SOC_AW8898
#include "../../codecs/aw8898/aw8898.h"
#endif
/* prize modified by lifenfen, add awinic smartpa aw8898, 20200103 end */
/* prize modified by wyq, add awinic smartpa aw881xx, 20200103 begin */
#ifdef CONFIG_SND_SMARTPA_AW881XX
#include "../../codecs/aw881xx/aw881xx.h"
#elif defined(CONFIG_SND_SMARTPA_AW88194A_NEW)||defined(CONFIG_SND_SMARTPA_AW881XX_V2)
extern int aw881xx_i2c_probe(struct i2c_client *i2c,
	const struct i2c_device_id *id);
extern int aw881xx_i2c_remove(struct i2c_client *i2c);
#endif
/* prize modified by wyq, add awinic smartpa aw881xx, 20200103 end */
#define MTK_SPK_NAME "Speaker Codec"
#define MTK_SPK_REF_NAME "Speaker Codec Ref"
static unsigned int mtk_spk_type;
static int mtk_spk_i2s_out, mtk_spk_i2s_in;
static struct mtk_spk_i2c_ctrl mtk_spk_list[MTK_SPK_TYPE_NUM] = {
	[MTK_SPK_NOT_SMARTPA] = {
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
	},
#if defined(CONFIG_SND_SOC_RT5509)
	[MTK_SPK_RICHTEK_RT5509] = {
		.i2c_probe = rt5509_i2c_probe,
		.i2c_remove = rt5509_i2c_remove,
		.i2c_shutdown = rt5509_i2c_shutdown,
		.codec_dai_name = "rt5509-aif1",
		.codec_name = "RT5509_MT_0",
	},
#endif
#ifdef CONFIG_SND_SOC_MT6660
	[MTK_SPK_MEDIATEK_MT6660] = {
		.i2c_probe = mt6660_i2c_probe,
		.i2c_remove = mt6660_i2c_remove,
		.codec_dai_name = "mt6660-aif",
		.codec_name = "MT6660_MT_0",
	},
#endif /* CONFIG_SND_SOC_MT6660 */

#ifdef CONFIG_SND_SOC_TFA9874
	[MTK_SPK_NXP_TFA98XX] = {
		.i2c_probe = tfa98xx_i2c_probe,
		.i2c_remove = tfa98xx_i2c_remove,
		.codec_dai_name = "tfa98xx-aif",
		.codec_name = "tfa98xx",
	},
#endif /* CONFIG_SND_SOC_MT6660 */
/* prize modified by lifenfen, add awinic smartpa aw8898, 20200103 begin */
#ifdef CONFIG_SND_SOC_AW8898
        [MTK_SPK_AWINIC_AW8898] = {
                .i2c_probe = aw8898_i2c_probe,
                .i2c_remove = aw8898_i2c_remove,
                .codec_dai_name = "aw8898-aif",
                .codec_name = "aw8898_smartpa",
        },
#endif /* CONFIG_SND_SOC_AW8898 */
/* prize modified by lifenfen, add awinic smartpa aw8898, 20200103 end */
/* prize modified by wyq, add awinic smartpa aw881xx, 20200103 begin */
#if defined(CONFIG_SND_SMARTPA_AW881XX) || defined(CONFIG_SND_SMARTPA_AW88194A_NEW)||defined(CONFIG_SND_SMARTPA_AW881XX_V2)
        [MTK_SPK_AWINIC_AW881XX] = {
                .i2c_probe = aw881xx_i2c_probe,
                .i2c_remove = aw881xx_i2c_remove,
		#if defined(CONFIG_SND_SOC_AW881XX_STEREO)
                .codec_dai_name = "aw881xx-aif-l",
                .codec_name = "aw881xx_smartpa_l",
		#else
                .codec_dai_name = "aw881xx-aif",
                .codec_name = "aw881xx_smartpa",
		#endif
        },
#endif /* CONFIG_SND_SMARTPA_AW881XX */
/* prize modified by wyq, add awinic smartpa aw881xx, 20200103 end */
};

/* prize modified by huarui, add awinic smartpa aw881xx, 20200103 begin */
#if defined(CONFIG_SND_SOC_AW881XX_STEREO)
static struct snd_soc_dai_link_component aw881xx_stereo_codecs[] = {
	{
		.name = "aw881xx_smartpa_l",
		.dai_name = "aw881xx-aif-l",
	},
	{
		.name = "aw881xx_smartpa_r",
		.dai_name = "aw881xx-aif-r",
	},
};
#endif
/* prize modified by huarui, add awinic smartpa aw881xx, 20200103 end */

static int mtk_spk_i2c_probe(struct i2c_client *client,
			     const struct i2c_device_id *id)
{
	int i, ret = 0;

	dev_info(&client->dev, "%s()\n", __func__);

	mtk_spk_type = MTK_SPK_NOT_SMARTPA;
	for (i = 0; i < MTK_SPK_TYPE_NUM; i++) {
		if (!mtk_spk_list[i].i2c_probe)
			continue;

		ret = mtk_spk_list[i].i2c_probe(client, id);
		if (ret)
			continue;

		mtk_spk_type = i;
		break;
	}

	return ret;
}

static int mtk_spk_i2c_remove(struct i2c_client *client)
{
	dev_info(&client->dev, "%s()\n", __func__);

	if (mtk_spk_list[mtk_spk_type].i2c_remove)
		mtk_spk_list[mtk_spk_type].i2c_remove(client);

	return 0;
}

static void mtk_spk_i2c_shutdown(struct i2c_client *client)
{
	dev_info(&client->dev, "%s()\n", __func__);

	if (mtk_spk_list[mtk_spk_type].i2c_shutdown)
		mtk_spk_list[mtk_spk_type].i2c_shutdown(client);
}

int mtk_spk_get_type(void)
{
	return mtk_spk_type;
}
EXPORT_SYMBOL(mtk_spk_get_type);

int mtk_spk_get_i2s_out_type(void)
{
	return mtk_spk_i2s_out;
}
EXPORT_SYMBOL(mtk_spk_get_i2s_out_type);

int mtk_spk_get_i2s_in_type(void)
{
	return mtk_spk_i2s_in;
}
EXPORT_SYMBOL(mtk_spk_get_i2s_in_type);

int mtk_ext_spk_get_status(void)
{
#ifdef CONFIG_SND_SOC_AW87339
	return aw87339_spk_status_get();
#else
	return 0;
#endif
}
EXPORT_SYMBOL(mtk_ext_spk_get_status);

void mtk_ext_spk_enable(int enable)
{
#ifdef CONFIG_SND_SOC_AW87339
	aw87339_spk_enable_set(enable);
#endif
/* prize added by chenjiaxi, add aw87519, 20200918-start */
#ifdef CONFIG_SND_SOC_AW87519
    aw87519_audio_off();
    usleep_range(1*1000, 20*1000);
    printk("Set aw87329 k class speaker\n");
    aw87519_audio_kspk();
    msleep(25);
#endif
/* prize added by chenjiaxi, add aw87519, 20200918-end */
}
EXPORT_SYMBOL(mtk_ext_spk_enable);

int mtk_spk_update_info(struct snd_soc_card *card,
			struct platform_device *pdev,
			int *spk_out_dai_link_idx, int *spk_ref_dai_link_idx,
			const struct snd_soc_ops *i2s_ops)
{
	int ret, i, mck_num;
	struct snd_soc_dai_link *dai_link;
	int i2s_out_dai_link_idx = -1;
	int i2s_in_dai_link_idx = -1;

	/* get spk i2s out number */
	ret = of_property_read_u32(pdev->dev.of_node,
				   "mtk_spk_i2s_out", &mtk_spk_i2s_out);
	if (ret) {
		mtk_spk_i2s_out = MTK_SPK_I2S_3;
		dev_err(&pdev->dev,
			"%s(), get mtk_spk_i2s_out fail, use defalut i2s3\n",
			__func__);
	}

	/* get spk i2s in number */
	ret = of_property_read_u32(pdev->dev.of_node,
				   "mtk_spk_i2s_in", &mtk_spk_i2s_in);
	if (ret) {
		mtk_spk_i2s_in = MTK_SPK_I2S_0;
		dev_err(&pdev->dev,
			"%s(), get mtk_spk_i2s_in fail, use defalut i2s0\n",
			 __func__);
	}

	if (mtk_spk_i2s_out > MTK_SPK_I2S_TYPE_NUM ||
	    mtk_spk_i2s_in > MTK_SPK_I2S_TYPE_NUM) {
		dev_err(&pdev->dev, "%s(), get mtk spk i2s fail\n",
			__func__);
		return -ENODEV;
	}

	/* get spk i2s mck number */
	ret = of_property_read_u32(pdev->dev.of_node,
				   "mtk_spk_i2s_mck", &mck_num);
	if (ret) {
		mck_num = MTK_SPK_I2S_TYPE_INVALID;
		dev_warn(&pdev->dev, "%s(), mtk_spk_i2s_mck no use\n",
			 __func__);
	}

	/* find dai link of i2s in and i2s out */
	for (i = 0; i < card->num_links; i++) {
		dai_link = &card->dai_link[i];

		if (i2s_out_dai_link_idx < 0 &&
		    strcmp(dai_link->cpu_dai_name, "I2S1") == 0 &&
		    mtk_spk_i2s_out == MTK_SPK_I2S_1) {
			i2s_out_dai_link_idx = i;
		} else if (i2s_out_dai_link_idx < 0 &&
			   strcmp(dai_link->cpu_dai_name, "I2S3") == 0 &&
			   mtk_spk_i2s_out == MTK_SPK_I2S_3) {
			i2s_out_dai_link_idx = i;
		} else if (i2s_out_dai_link_idx < 0 &&
			   strcmp(dai_link->cpu_dai_name, "I2S5") == 0 &&
			   mtk_spk_i2s_out == MTK_SPK_I2S_5) {
			i2s_out_dai_link_idx = i;
		}

		if (i2s_in_dai_link_idx < 0 &&
		    strcmp(dai_link->cpu_dai_name, "I2S0") == 0 &&
		    (mtk_spk_i2s_in == MTK_SPK_I2S_0 ||
		     mtk_spk_i2s_in == MTK_SPK_TINYCONN_I2S_0)) {
			i2s_in_dai_link_idx = i;
		} else if (i2s_in_dai_link_idx < 0 &&
			   strcmp(dai_link->cpu_dai_name, "I2S2") == 0 &&
			   (mtk_spk_i2s_in == MTK_SPK_I2S_2 ||
			    mtk_spk_i2s_in == MTK_SPK_TINYCONN_I2S_2)) {
			i2s_in_dai_link_idx = i;
		}

		if (i2s_out_dai_link_idx >= 0 && i2s_in_dai_link_idx >= 0)
			break;
	}

	if (i2s_out_dai_link_idx < 0 || i2s_in_dai_link_idx < 0) {
		dev_err(&pdev->dev,
			"%s(), i2s cpu dai name error, i2s_out_dai_link_idx = %d, i2s_in_dai_link_idx = %d",
			__func__, i2s_out_dai_link_idx, i2s_in_dai_link_idx);
		return -ENODEV;
	}

	*spk_out_dai_link_idx = i2s_out_dai_link_idx;
	*spk_ref_dai_link_idx = i2s_in_dai_link_idx;

	if (mtk_spk_type != MTK_SPK_NOT_SMARTPA) {
		dai_link = &card->dai_link[i2s_out_dai_link_idx];
		dai_link->codec_name = NULL;
		dai_link->codec_dai_name = NULL;
		if (mck_num == mtk_spk_i2s_out)
			dai_link->ops = i2s_ops;

		dai_link = &card->dai_link[i2s_in_dai_link_idx];
		dai_link->codec_name = NULL;
		dai_link->codec_dai_name = NULL;
		if (mck_num == mtk_spk_i2s_in)
			dai_link->ops = i2s_ops;
	}

	dev_info(&pdev->dev,
		 "%s(), mtk_spk_type %d, spk_out_dai_link_idx %d, spk_out_dai_link_idx %d, mck: %d\n",
		 __func__,
		 mtk_spk_type, *spk_ref_dai_link_idx,
		 *spk_out_dai_link_idx, mck_num);

	return 0;
}
EXPORT_SYMBOL(mtk_spk_update_info);

int mtk_spk_update_dai_link(struct snd_soc_card *card,
			    struct platform_device *pdev,
			    const struct snd_soc_ops *i2s_ops)
{
	int ret, i;
	int spk_ref_dai_link_idx = -1;
	int spk_dai_link_idx = -1;
	int i2s_mck;
	struct snd_soc_dai_link *dai_link;

	dev_info(&pdev->dev, "%s(), mtk_spk_type %d\n",
		 __func__, mtk_spk_type);

	/* get spk i2s out number */
	ret = of_property_read_u32(pdev->dev.of_node,
				   "mtk_spk_i2s_out", &mtk_spk_i2s_out);
	if (ret) {
		mtk_spk_i2s_out = MTK_SPK_I2S_3;
		dev_err(&pdev->dev,
			"%s(), get mtk_spk_i2s_out fail, use defalut i2s3\n",
			__func__);
	}

	/* get spk i2s in number */
	ret = of_property_read_u32(pdev->dev.of_node,
				   "mtk_spk_i2s_in", &mtk_spk_i2s_in);
	if (ret) {
		mtk_spk_i2s_in = MTK_SPK_I2S_0;
		dev_err(&pdev->dev,
			"%s(), get mtk_spk_i2s_in fail, use defalut i2s0\n",
			 __func__);
	}

	/* get spk i2s mck number */
	ret = of_property_read_u32(pdev->dev.of_node,
				   "mtk_spk_i2s_mck", &i2s_mck);
	if (ret) {
		i2s_mck = MTK_SPK_I2S_TYPE_INVALID;
		dev_warn(&pdev->dev, "%s(), mtk_spk_i2s_mck no use\n",
			 __func__);
	}

	dev_info(&pdev->dev,
		 "%s(), mtk_spk_type %d, i2s in %d, i2s out %d\n",
		 __func__, mtk_spk_type, mtk_spk_i2s_in, mtk_spk_i2s_out);

	if (mtk_spk_i2s_out > MTK_SPK_I2S_TYPE_NUM ||
	    mtk_spk_i2s_in > MTK_SPK_I2S_TYPE_NUM) {
		dev_err(&pdev->dev, "%s(), get mtk spk i2s fail\n",
			__func__);
		return -ENODEV;
	}

	if (mtk_spk_type == MTK_SPK_NOT_SMARTPA) {
		dev_info(&pdev->dev, "%s(), no need to update dailink\n",
			 __func__);
		return 0;
	}

	/* find dai link of i2s in and i2s out */
	for (i = 0; i < card->num_links; i++) {
		dai_link = &card->dai_link[i];

		if (spk_dai_link_idx < 0 &&
		    strcmp(dai_link->cpu_dai_name, "I2S1") == 0 &&
		    mtk_spk_i2s_out == MTK_SPK_I2S_1) {
			spk_dai_link_idx = i;
		} else if (spk_dai_link_idx < 0 &&
			   strcmp(dai_link->cpu_dai_name, "I2S3") == 0 &&
			   mtk_spk_i2s_out == MTK_SPK_I2S_3) {
			spk_dai_link_idx = i;
		} else if (spk_dai_link_idx < 0 &&
			   strcmp(dai_link->cpu_dai_name, "I2S5") == 0 &&
			   mtk_spk_i2s_out == MTK_SPK_I2S_5) {
			spk_dai_link_idx = i;
		}

		if (spk_ref_dai_link_idx < 0 &&
		    strcmp(dai_link->cpu_dai_name, "I2S0") == 0 &&
		    (mtk_spk_i2s_in == MTK_SPK_I2S_0 ||
		     mtk_spk_i2s_in == MTK_SPK_TINYCONN_I2S_0)) {
			spk_ref_dai_link_idx = i;
		} else if (spk_ref_dai_link_idx < 0 &&
			   strcmp(dai_link->cpu_dai_name, "I2S2") == 0 &&
			   (mtk_spk_i2s_in == MTK_SPK_I2S_2 ||
			    mtk_spk_i2s_in == MTK_SPK_TINYCONN_I2S_2)) {
			spk_ref_dai_link_idx = i;
		}

		if (spk_dai_link_idx >= 0 && spk_ref_dai_link_idx >= 0)
			break;
	}

	if (spk_dai_link_idx < 0 || spk_ref_dai_link_idx < 0) {
		dev_err(&pdev->dev,
			"%s(), i2s cpu dai name error, spk_dai_link_idx = %d, spk_ref_dai_link_idx = %d",
			__func__, spk_dai_link_idx, spk_ref_dai_link_idx);
		return -ENODEV;
	}

	/* update spk codec dai name and codec name */
	dai_link = &card->dai_link[spk_dai_link_idx];
	dai_link->name = MTK_SPK_NAME;
/* prize modified by huarui, add awinic smartpa aw881xx, 20200103 begin */
#if defined(CONFIG_SND_SOC_AW881XX_STEREO)
	dai_link->num_codecs = 2;
	dai_link->codecs = aw881xx_stereo_codecs;
	dai_link->codec_dai_name = NULL;
	dai_link->codec_name = NULL;
#else
	dai_link->codec_dai_name =
		mtk_spk_list[mtk_spk_type].codec_dai_name;
	dai_link->codec_name =
		mtk_spk_list[mtk_spk_type].codec_name;
#endif
/* prize modified by huarui, add awinic smartpa aw881xx, 20200103 end */
	dai_link->ignore_pmdown_time = 1;
	if (i2s_mck == mtk_spk_i2s_out)
		dai_link->ops = i2s_ops;

	dev_info(&pdev->dev,
		 "%s(), %s, codec dai name = %s, codec name = %s, cpu dai name: %s\n",
		 __func__, dai_link->name,
		 dai_link->codec_dai_name,
		 dai_link->codec_name,
		 dai_link->cpu_dai_name);

	dai_link = &card->dai_link[spk_ref_dai_link_idx];
	dai_link->name = MTK_SPK_REF_NAME;
	dai_link->codec_dai_name =
		mtk_spk_list[mtk_spk_type].codec_dai_name;
	dai_link->codec_name =
		mtk_spk_list[mtk_spk_type].codec_name;
	dai_link->ignore_pmdown_time = 1;
	if (i2s_mck == mtk_spk_i2s_in)
		dai_link->ops = i2s_ops;

	dev_info(&pdev->dev,
		 "%s(), %s, codec dai name = %s, codec name = %s, cpu dai name: %s\n",
		 __func__, dai_link->name,
		 dai_link->codec_dai_name,
		 dai_link->codec_name,
		 dai_link->cpu_dai_name);


	return 0;
}
EXPORT_SYMBOL(mtk_spk_update_dai_link);

int mtk_spk_send_ipi_buf_to_dsp(void *data_buffer, uint32_t data_size)
{
	int result = 0;
#if defined(CONFIG_SND_SOC_MTK_AUDIO_DSP)
	struct ipi_msg_t ipi_msg;
	int task_scene;

	memset((void *)&ipi_msg, 0, sizeof(struct ipi_msg_t));
	task_scene = mtk_get_speech_status() ?
		     TASK_SCENE_CALL_FINAL : TASK_SCENE_AUDPLAYBACK;

	result = audio_send_ipi_buf_to_dsp(&ipi_msg, task_scene,
					   AUDIO_DSP_TASK_AURISYS_SET_BUF,
					   data_buffer, data_size);
#endif
	return result;
}
EXPORT_SYMBOL(mtk_spk_send_ipi_buf_to_dsp);

int mtk_spk_recv_ipi_buf_from_dsp(int8_t *buffer,
				  int16_t size,
				  uint32_t *buf_len)
{
	int result = 0;
#if defined(CONFIG_SND_SOC_MTK_AUDIO_DSP)
	struct ipi_msg_t ipi_msg;
	int task_scene;

	memset((void *)&ipi_msg, 0, sizeof(struct ipi_msg_t));
	task_scene = mtk_get_speech_status() ?
		     TASK_SCENE_CALL_FINAL : TASK_SCENE_AUDPLAYBACK;

	result = audio_recv_ipi_buf_from_dsp(&ipi_msg,
					     task_scene,
					     AUDIO_DSP_TASK_AURISYS_GET_BUF,
					     buffer, size, buf_len);
#endif
	return result;
}
EXPORT_SYMBOL(mtk_spk_recv_ipi_buf_from_dsp);

static const struct i2c_device_id mtk_spk_i2c_id[] = {
	{ "tfa98xx", 0},
	{ "speaker_amp", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, mtk_spk_i2c_id);

#ifdef CONFIG_OF
static const struct of_device_id mtk_spk_match_table[] = {
	{.compatible = "nxp,tfa98xx",},
	{.compatible = "mediatek,speaker_amp",},
	{},
};
MODULE_DEVICE_TABLE(of, mtk_spk_match_table);
#endif /* #ifdef CONFIG_OF */

static struct i2c_driver mtk_spk_i2c_driver = {
	.driver = {
		.name = "speaker_amp",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(mtk_spk_match_table),
	},
	.probe = mtk_spk_i2c_probe,
	.remove = mtk_spk_i2c_remove,
	.shutdown = mtk_spk_i2c_shutdown,
	.id_table = mtk_spk_i2c_id,
};

module_i2c_driver(mtk_spk_i2c_driver);

MODULE_DESCRIPTION("Mediatek speaker amp register driver");
MODULE_AUTHOR("Shane Chien <shane.chien@mediatek.com>");
MODULE_LICENSE("GPL v2");
