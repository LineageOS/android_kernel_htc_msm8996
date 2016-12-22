/*
 *  sound/soc/codecs/rt5503.c
 *  Driver to Richtek RT5503 HPAMP IC
 *
 *  Copyright (C) 2015 Richtek Technology Corp.
 *  cy_huang <cy_huang@richtek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/version.h>
#include <linux/i2c.h>
#include <linux/pm_runtime.h>
#include <linux/delay.h>
#include <linux/of_gpio.h>
#include <linux/input.h>
#include <linux/qdsp6v2/apr.h> //HTC_AUD
/* alsa sound header */
#include <sound/soc.h>
#include <sound/tlv.h>
#include <sound/pcm_params.h>

#include <linux/misc/rt-regmap.h>
#include <linux/i2c/rt5503.h>

static int rt5503_block_read(
	void *client, u32 reg, int bytes, void *dest)
{
#if RT5503_SIMULATE_DEVICE
	struct rt5503_chip *chip = i2c_get_clientdata(client);
	int offset = 0, ret = 0;

	offset = rt5503_calculate_offset(reg);
	if (offset < 0) {
		dev_err(chip->dev, "%s: unknown register 0x%02x\n", __func__,
			ret);
		ret = -EINVAL;
	} else
		memcpy(dest, chip->sim + offset, bytes);
	return ret;
#else
	return i2c_smbus_read_i2c_block_data(client, reg, bytes, dest);
#endif /* #if RT5503_SIMULATE_DEVICE */
}

static int rt5503_block_write(void *client, u32 reg,
	int bytes, const void *src)
{
#if RT5503_SIMULATE_DEVICE
	struct rt5503_chip *chip = i2c_get_clientdata(client);
	int offset = 0, ret = 0;

	offset = rt5503_calculate_offset(reg);
	if (offset < 0) {
		dev_err(chip->dev, "%s: unknown register 0x%02x\n", __func__,
			ret);
		ret = -EINVAL;
	} else
		memcpy(chip->sim + offset, src, bytes);
	return ret;
#else
	return i2c_smbus_write_i2c_block_data(client, reg, bytes, src);
#endif /* #if RT5503_SIMULATE_DEVICE */
}

static struct rt_regmap_fops rt5503_regmap_ops = {
	.read_device = rt5503_block_read,
	.write_device = rt5503_block_write,
};

/* Global read/write function */
static int rt5503_reg_block_read(struct i2c_client *i2c, u32 reg,
			   int bytes, void *data)
{
	struct rt5503_chip *chip = i2c_get_clientdata(i2c);
	int ret = 0;
#ifdef CONFIG_RT_REGMAP
	ret = rt_regmap_block_read(chip->rd, reg, bytes, data);
#else
	down(&chip->io_semaphore);
	ret = rt5503_block_read(chip->i2c, reg, bytes, data);
	up(&chip->io_semaphore);
#endif /* #ifdef CONFIG_RT_REGMAP */
	return ret;
}

static int rt5503_update_bits(struct i2c_client *i2c, u32 reg,
			   u32 mask, u32 data, int bytes)
{
	struct rt5503_chip *chip = i2c_get_clientdata(i2c);
#ifdef CONFIG_RT_REGMAP
	struct rt_reg_data rrd;

	return rt_regmap_update_bits(chip->rd, &rrd, reg, mask, data);
#else
	u32 read_data = 0;
	u8* p_data = (u8 *)&read_data;
	int i = 0, j = 0, ret = 0;

	down(&chip->io_semaphore);
	ret = rt5503_block_read(chip->i2c, reg, bytes, &read_data);
	if (ret < 0)
		goto err_bits;
	j = (bytes / 2);
	for (i = 0; i < j; i++)
		swap(p_data[i], p_data[bytes - i]);
	ret = rt5503_block_write(chip->i2c, reg, bytes, &read_data);
	if (ret < 0)
		goto err_bits;
err_bits:
	up(&chip->io_semaphore);
	return ret;
#endif /* #ifdef CONFIG_RT_REGMAP */
}

static int rt5503_set_bits(struct i2c_client *i2c, u32 reg, u8 mask)
{
	return rt5503_update_bits(i2c, reg, mask, mask, 1);
}

static int rt5503_clr_bits(struct i2c_client *i2c, u32 reg, u8 mask)
{
	return rt5503_update_bits(i2c, reg, mask, 0, 1);
}

static unsigned int rt5503_io_read(struct snd_soc_codec *codec,
	 unsigned int reg)
{
	struct rt5503_chip *chip = snd_soc_codec_get_drvdata(codec);
	int ret = 0;
#ifdef CONFIG_RT_REGMAP
	struct rt_reg_data rrd = {0};

	dev_info(codec->dev, "%s: reg %02x\n", __func__, reg);
	ret = rt_regmap_reg_read(chip->rd, &rrd, reg);
	return (ret < 0 ? ret : rrd.rt_data.data_u32);
#else
	u8 data = 0;

	down(&chip->io_semaphore);
	ret = rt5503_block_read(chip->i2c, reg, 1, &data);
	up(&chip->io_semaphore);
	return (ret < 0 ? ret : data);
#endif /* #ifdef CONFIG_RT_REGMAP */
}

static int rt5503_io_write(struct snd_soc_codec *codec,
	unsigned int reg, unsigned int data)
{
	struct rt5503_chip *chip = snd_soc_codec_get_drvdata(codec);
#ifdef CONFIG_RT_REGMAP
	struct rt_reg_data rrd = {0};

	dev_info(codec->dev, "%s: reg %02x data %02x\n", __func__, reg, data);
	return rt_regmap_reg_write(chip->rd, &rrd, reg, data);
#else
	int ret = 0;

	down(&chip->io_semaphore);
	ret = rt5503_block_write(chip->i2c, reg, 1, &data);
	up(&chip->io_semaphore);
	return ret;
#endif /* #ifdef CONFIG_RT_REGMAP */
}

static inline int rt5503_power_on(struct rt5503_chip *chip, bool en)
{
	int ret = 0;

	dev_info(chip->dev, "%s: en %d\n", __func__, en);
	if (en)
		ret = rt5503_set_bits(chip->i2c, RT5503_REG_CHIPEN,
				RT5503_CHIPEN_MASK);
	else
		ret = rt5503_clr_bits(chip->i2c, RT5503_REG_CHIPEN,
				RT5503_CHIPEN_MASK);
	mdelay(1);
	return ret;
}

static int rt5503_set_bias_level(struct snd_soc_codec *codec,
	enum snd_soc_bias_level level)
{
	struct rt5503_chip *chip = snd_soc_codec_get_drvdata(codec);
	int ret = 0;

	switch (level) {
	case SND_SOC_BIAS_ON:
	case SND_SOC_BIAS_PREPARE:
		codec->dapm.bias_level = level;
		break;
	case SND_SOC_BIAS_STANDBY:
		ret = rt5503_power_on(chip, true);
		if (ret >= 0)
			codec->dapm.bias_level = level;
		break;
	case SND_SOC_BIAS_OFF:
		ret = rt5503_power_on(chip, false);
		if (ret >= 0)
			codec->dapm.bias_level = level;
		break;
	default:
		ret = -EINVAL;
	}
	return ret;
}

static int rt5503_codec_probe(struct snd_soc_codec *codec)
{
	int ret = 0;

	/* SW_EN, AVDD_SEL, HP_HP */
	ret = snd_soc_update_bits(codec, RT5503_REG_CHIPEN, 0x31, 0x31);
	if (ret < 0) {
		dev_err(codec->dev, "chip io error\n");
		goto err_out_probe;
	}
	/* clear chopper freq */
	ret = snd_soc_update_bits(codec, RT5503_REG_DACCFG2, 0x07, 0);
	if (ret < 0) {
		dev_err(codec->dev, "chip io error\n");
		goto err_out_probe;
	}
	ret = snd_soc_write(codec, RT5503_REG_OCOTCFG1, 0x02);
	if (ret < 0) {
		dev_err(codec->dev, "chip io error\n");
		goto err_out_probe;
	}
	ret = snd_soc_write(codec, RT5503_REG_CDACCFG1, 0x50);
	if (ret < 0) {
		dev_err(codec->dev, "chip io error\n");
		goto err_out_probe;
	}
	ret = snd_soc_write(codec, RT5503_REG_CDACCFG2, 0x80);
	if (ret < 0) {
		dev_err(codec->dev, "chip io error\n");
		goto err_out_probe;
	}
	ret = snd_soc_update_bits(codec, RT5503_REG_DACCFG2, 0x08, 0x08);
	if (ret < 0) {
		dev_err(codec->dev, "chip io error\n");
		goto err_out_probe;
	}
	ret = snd_soc_write(codec, RT5503_REG_PLLCFG1, 0x38);
	if (ret < 0) {
		dev_err(codec->dev, "chip io error\n");
		goto err_out_probe;
	}
	ret = snd_soc_write(codec, RT5503_REG_BOOSTCFG2, 0x14);
	if (ret < 0) {
		dev_err(codec->dev, "chip io error\n");
		goto err_out_probe;
	}
	ret = snd_soc_write(codec, RT5503_REG_BOOSTCFG3, 0x0b);
	if (ret < 0) {
		dev_err(codec->dev, "chip io error\n");
		goto err_out_probe;
	}
	ret = snd_soc_write(codec, RT5503_REG_IRRAMP, 0xe0);
	if (ret < 0) {
		dev_err(codec->dev, "chip io error\n");
		goto err_out_probe;
	}
	ret = snd_soc_write(codec, RT5503_REG_CFG3, 0x90);
	if (ret < 0) {
		dev_err(codec->dev, "chip io error\n");
		goto err_out_probe;
	}
	ret = snd_soc_write(codec, RT5503_REG_CPSRSEL, 0x81);
	if (ret < 0) {
		dev_err(codec->dev, "chip io error\n");
		goto err_out_probe;
	}
	ret = snd_soc_write(codec, RT5503_REG_CPNONOVLSEL, 0x65);
	if (ret < 0) {
		dev_err(codec->dev, "chip io error\n");
		goto err_out_probe;
	}
	ret = snd_soc_write(codec, RT5503_REG_VBBCFG1, 0x8f);
	if (ret < 0) {
		dev_err(codec->dev, "chip io error\n");
		goto err_out_probe;
	}
	ret = snd_soc_write(codec, RT5503_REG_HODSEL, 0x03);
	if (ret < 0) {
		dev_err(codec->dev, "chip io error\n");
		goto err_out_probe;
	}
	ret = snd_soc_write(codec, RT5503_REG_HPVSSCFG1, 0xc0);
	if (ret < 0) {
		dev_err(codec->dev, "chip io error\n");
		goto err_out_probe;
	}
	ret = snd_soc_write(codec, RT5503_REG_DACCFG3, 0x40);
	if (ret < 0) {
		dev_err(codec->dev, "chip io error\n");
		goto err_out_probe;
	}
	ret = snd_soc_write(codec, RT5503_REG_BBMODSEL, 0x60);
	if (ret < 0) {
		dev_err(codec->dev, "chip io error\n");
		goto err_out_probe;
	}
	return rt5503_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
err_out_probe:
	return ret;
}

static int rt5503_codec_remove(struct snd_soc_codec *codec)
{
	return rt5503_set_bias_level(codec, SND_SOC_BIAS_OFF);
}

#ifdef CONFIG_PM
static int rt5503_codec_suspend(struct snd_soc_codec *codec)
{
	return rt5503_set_bias_level(codec, SND_SOC_BIAS_OFF);
}

static int rt5503_codec_resume(struct snd_soc_codec *codec)
{
	return rt5503_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
}
#else
#define rt5503_codec_suspend NULL
#define rt5503_codec_resume NULL
#endif /* #ifdef CONFIG_PM */

static int rt5503_bbcp_event(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	int ret = 0;

	pr_info("%s: event %d\n", __func__, event);
	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		ret = snd_soc_update_bits(w->codec, RT5503_REG_BLOCKEN,
			RT5503_BBEN_MASK | RT5503_CPEN_MASK,
			RT5503_BBEN_MASK | RT5503_CPEN_MASK);
		mdelay(10);
		break;
	case SND_SOC_DAPM_POST_PMD:
		ret = snd_soc_update_bits(w->codec, RT5503_REG_BLOCKEN,
			RT5503_BBEN_MASK | RT5503_CPEN_MASK,
			~(RT5503_BBEN_MASK | RT5503_CPEN_MASK));
		break;
	default:
		break;
	}
	return ret;
}

static int rt5503_hpamp_event(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	int ret = 0;

	pr_info("%s: event %d\n", __func__, event);
	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		ret = snd_soc_update_bits(w->codec, RT5503_REG_CHIPEN,
			RT5503_HPENL_MASK | RT5503_HPENR_MASK,
			RT5503_HPENL_MASK | RT5503_HPENR_MASK);
		if (ret < 0)
			goto out_hpamp_event;
		mdelay(6);
		ret = snd_soc_write(w->codec, RT5503_REG_VBBDESSEL, 0x0c);
		if (ret < 0)
			goto out_hpamp_event;
		mdelay(10);
		ret = snd_soc_write(w->codec, RT5503_REG_VBBDESSEL, 0x1c);
		if (ret < 0)
			goto out_hpamp_event;
		mdelay(10);
		ret = snd_soc_write(w->codec, RT5503_REG_VBBDESSEL, 0x2c);
		if (ret < 0)
			goto out_hpamp_event;
		mdelay(10);
		ret = snd_soc_write(w->codec, RT5503_REG_VBBDESSEL, 0x3c);
		if (ret < 0)
			goto out_hpamp_event;
		mdelay(10);
		ret = snd_soc_write(w->codec, RT5503_REG_VBBDESSEL, 0x60);
		if (ret < 0)
			goto out_hpamp_event;
		mdelay(10);
		ret = snd_soc_write(w->codec, RT5503_REG_VBBDESSEL, 0x68);
		if (ret < 0)
			goto out_hpamp_event;
		mdelay(10);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		ret = snd_soc_update_bits(w->codec, RT5503_REG_CHIPEN,
			RT5503_HPENL_MASK | RT5503_HPENR_MASK,
			~(RT5503_HPENL_MASK | RT5503_HPENR_MASK));
		if (ret < 0)
			goto out_hpamp_event;
		mdelay(6);
		ret = snd_soc_write(w->codec, RT5503_REG_VBBDESSEL, 0x0c);
		if (ret < 0)
			goto out_hpamp_event;
		break;
	default:
		break;
	}
out_hpamp_event:
	return ret;
}

static int rt5503_pll_event(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	int ret = 0;

	pr_info("%s: event %d\n", __func__, event);
	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		ret = snd_soc_update_bits(w->codec, RT5503_REG_BLOCKEN,
			RT5503_PLLEN_MASK, RT5503_PLLEN_MASK);
		break;
	case SND_SOC_DAPM_POST_PMD:
		ret = snd_soc_update_bits(w->codec, RT5503_REG_BLOCKEN,
			RT5503_PLLEN_MASK, ~(RT5503_PLLEN_MASK));
		break;
	default:
		break;
	}
	return ret;
}

static const struct snd_soc_dapm_widget rt5503_dapm_widgets[] = {
	SND_SOC_DAPM_AIF_IN("AIF1DACDAT", "AIF1 Playback", 0,
		SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_OUT("AIF1ADCDAT", "AIF1 Capture", 0,
		SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_DAC("LDAC", NULL, RT5503_REG_DACCFG2,
		RT5503_DACLEN_SHFT, 1),
	SND_SOC_DAPM_DAC("RDAC", NULL, RT5503_REG_DACCFG2,
		RT5503_DACREN_SHFT, 1),
	SND_SOC_DAPM_ADC("ADC", NULL, RT5503_REG_ADCCFG1,
		RT5503_ADCEN_SHFT, 0),
	SND_SOC_DAPM_OUT_DRV_E("HPAMP", SND_SOC_NOPM, 0, 0, NULL, 0,
		rt5503_hpamp_event,
		SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_SUPPLY("BBandCP", SND_SOC_NOPM, 0, 0, rt5503_bbcp_event,
		SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_SUPPLY("PLL", SND_SOC_NOPM, 0, 0, rt5503_pll_event,
		SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
	SND_SOC_DAPM_OUTPUT("OUTL"),
	SND_SOC_DAPM_OUTPUT("OUTR"),
	SND_SOC_DAPM_INPUT("INP"),
	SND_SOC_DAPM_INPUT("INN"),
};

static const struct snd_soc_dapm_route rt5503_dapm_routes[] = {
	/* ADC Path */
	{ "AIF1ADCDAT", NULL, "ADC"},
	{ "ADC", NULL, "INP"},
	{ "ADC", NULL, "INN"},
	{ "ADC", NULL, "PLL"},
	/* DAC Path */
	{ "LDAC", NULL, "AIF1DACDAT"},
	{ "RDAC", NULL, "AIF1DACDAT"},
	{ "LDAC", NULL, "PLL"},
	{ "RDAC", NULL, "PLL"},
	{ "HPAMP", NULL, "LDAC"},
	{ "HPAMP", NULL, "RDAC"},
	{ "HPAMP", NULL, "BBandCP"},
	{ "OUTL", NULL, "HPAMP"},
	{ "OUTR", NULL, "HPAMP"},
};

static const DECLARE_TLV_DB_SCALE(dacdigvol_tlv, -9600, 5, 0);
static const DECLARE_TLV_DB_SCALE(adcdigvol_tlv, -9600, 5, 0);
static const char *rt5503_onoff_text[] = { "Off", "On"};
static const struct soc_enum rt5503_enum[] = {
	SOC_ENUM_SINGLE(RT5503_REG_MICCFG1, 5,
		ARRAY_SIZE(rt5503_onoff_text), rt5503_onoff_text),
};
static const struct snd_kcontrol_new rt5503_controls[] = {
	SOC_SINGLE_TLV("DAC VOL", RT5503_REG_DIGVOL, 0, 200, 0, dacdigvol_tlv),
	SOC_SINGLE_TLV("ADC VOL", RT5503_REG_ADCDIGVOL, 0, 240, 0, adcdigvol_tlv),
	SOC_ENUM("MicBias", rt5503_enum[0]),
};

static const struct snd_soc_codec_driver rt5503_codec_drv = {
	.probe = rt5503_codec_probe,
	.remove = rt5503_codec_remove,
	.suspend = rt5503_codec_suspend,
	.resume = rt5503_codec_resume,

	.controls = rt5503_controls,
	.num_controls = ARRAY_SIZE(rt5503_controls),
	.dapm_widgets = rt5503_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(rt5503_dapm_widgets),
	.dapm_routes = rt5503_dapm_routes,
	.num_dapm_routes = ARRAY_SIZE(rt5503_dapm_routes),

	.set_bias_level = rt5503_set_bias_level,
	/* codec io */
	.read = rt5503_io_read,
	.write = rt5503_io_write,
};

enum {
	RT5503_PLLS_BCK,
	RT5503_PLLS_ADCBCK,
	RT5503_PLLS_MCLK,
	RT5503_PLLS_BYPASS,
	RT5503_PLLS_MAX,
};

static inline int rt5503_aif_set_pll(struct snd_soc_dai *dai, int pll_id, int source,
	unsigned int freq_in, unsigned int freq_out)
{
	u8 data = RT5503_PLLS_ADCBCK;

	dev_info(dai->dev, "%s: playback %d\n", __func__, dai->playback_active);
	if (dai->playback_active)
		data = RT5503_PLLS_BCK;
	return snd_soc_update_bits(dai->codec, RT5503_REG_CFG1,
			RT5503_PLLSEL_MASK, data << RT5503_PLLSEL_SHFT);
}

static int rt5503_aif_digital_mute(struct snd_soc_dai *dai, int mute)
{
	dev_info(dai->dev, "%s: mute %d\n", __func__, mute);
	return snd_soc_update_bits(dai->codec, RT5503_REG_CFG1,
		RT5503_MUTE_MASK, mute ? RT5503_MUTE_MASK : ~RT5503_MUTE_MASK);
}

static int rt5503_aif_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	u8 regval = 0;
	int ret = 0;

	dev_info(dai->dev, "%s: fmt:%d\n", __func__, fmt);
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK ) {
	case SND_SOC_DAIFMT_I2S:
		regval |= (RT5503_AUDFMT_I2S << RT5503_AUDFMT_SHFT);
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		regval |= (RT5503_AUDFMT_RIGHTJ << RT5503_AUDFMT_SHFT);
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		regval |= (RT5503_AUDFMT_LEFTJ << RT5503_AUDFMT_SHFT);
		break;
	case SND_SOC_DAIFMT_DSP_A:
		regval |= (RT5503_DSPMODE_A << RT5503_DSPMODE_SHFT);
	case SND_SOC_DAIFMT_DSP_B:
		regval |= (RT5503_DSPMODE_B << RT5503_DSPMODE_SHFT);
		regval |= (RT5503_AUDFMT_DSPMODE << RT5503_AUDFMT_SHFT);
		break;
	default:
		break;
	}
	ret = snd_soc_update_bits(dai->codec, RT5503_REG_DACAUDFMT,
			RT5503_DSPMODE_MASK | RT5503_AUDFMT_MASK, regval);
	if (ret < 0) {
		dev_err(dai->dev, "config dac audfmt error\n");
		goto out_set_dacfmt;
	}
	ret = snd_soc_update_bits(dai->codec, RT5503_REG_ADCAUDFMT,
			RT5503_DSPMODE_MASK | RT5503_AUDFMT_MASK, regval);
out_set_dacfmt:
	return ret;
}

static int rt5503_aif_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *hw_params, struct snd_soc_dai *dai)
{
	int capture = (substream->stream == SNDRV_PCM_STREAM_CAPTURE);
	u32 regaddr = 0;
	/* 0 for sr and bckfs, 1 for audbits */
	u8 regval[2] = {0};
	int ret = 0;

	dev_info(dai->dev, "%s\n", __func__);
	dev_info(dai->dev, "format 0x%08x\n", params_format(hw_params));
	switch (params_format(hw_params)) {
	case SNDRV_PCM_FORMAT_S16:
	case SNDRV_PCM_FORMAT_U16:
		regval[0] |= (RT5503_BCKMODE_32FS << RT5503_BCKMODE_SHFT);
		regval[1] |= (RT5503_AUDBIT_16 << RT5503_AUDBIT_SHFT);
		break;
	case SNDRV_PCM_FORMAT_S18_3LE:
	case SNDRV_PCM_FORMAT_U18_3LE:
	case SNDRV_PCM_FORMAT_S18_3BE:
	case SNDRV_PCM_FORMAT_U18_3BE:
		regval[0] |= (RT5503_BCKMODE_48FS << RT5503_BCKMODE_SHFT);
		regval[1] |= (RT5503_AUDBIT_18 << RT5503_AUDBIT_SHFT);
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
	case SNDRV_PCM_FORMAT_U20_3LE:
	case SNDRV_PCM_FORMAT_S20_3BE:
	case SNDRV_PCM_FORMAT_U20_3BE:
		regval[0] |= (RT5503_BCKMODE_48FS << RT5503_BCKMODE_SHFT);
		regval[1] |= (RT5503_AUDBIT_20 << RT5503_AUDBIT_SHFT);
		break;
	case SNDRV_PCM_FORMAT_S24_3LE:
	case SNDRV_PCM_FORMAT_S24_3BE:
	case SNDRV_PCM_FORMAT_U24_3LE:
	case SNDRV_PCM_FORMAT_U24_3BE:
		regval[0] |= (RT5503_BCKMODE_48FS << RT5503_BCKMODE_SHFT);
		regval[1] |= (RT5503_AUDBIT_24 << RT5503_AUDBIT_SHFT);
		break;
	case SNDRV_PCM_FORMAT_S24:
	case SNDRV_PCM_FORMAT_U24:
		regval[0] |= (RT5503_BCKMODE_64FS << RT5503_BCKMODE_SHFT);
		regval[1] |= (RT5503_AUDBIT_24 << RT5503_AUDBIT_SHFT);
		break;
	default:
		ret = -EINVAL;
		goto out_hw_params;
	}
	dev_info(dai->dev, "rate %d\n", params_rate(hw_params));
	switch (params_rate(hw_params)) {
	case 8000:
		regval[0] |= (RT5503_SRMODE_8K << RT5503_SRMODE_SHFT);
		break;
	case 11025:
	case 12000:
		regval[0] |= (RT5503_SRMODE_12K << RT5503_SRMODE_SHFT);
		break;
	case 16000:
		regval[0] |= (RT5503_SRMODE_16K << RT5503_SRMODE_SHFT);
		break;
	case 22050:
	case 24000:
		regval[0] |= (RT5503_SRMODE_24K << RT5503_SRMODE_SHFT);
		break;
	case 32000:
		regval[0] |= (RT5503_SRMODE_32K << RT5503_SRMODE_SHFT);
		break;
	case 44100:
	case 48000:
		regval[0] |= (RT5503_SRMODE_48K << RT5503_SRMODE_SHFT);
		break;
	case 88200:
	case 96000:
		regval[0] |= (RT5503_SRMODE_96K << RT5503_SRMODE_SHFT);
		break;
	case 176400:
	case 192000:
		regval[0] |= (RT5503_SRMODE_192K << RT5503_SRMODE_SHFT);
		break;
	case 352800:
	case 384000:
		regval[0] |= (RT5503_SRMODE_384K << RT5503_SRMODE_SHFT);
		break;
	default:
		ret = -EINVAL;
		goto out_hw_params;
	}
	regaddr = capture ? RT5503_REG_ADCCFG2 : RT5503_REG_DACCFG1;
	ret = snd_soc_update_bits(dai->codec, regaddr,
			RT5503_BCKMODE_MASK | RT5503_SRMODE_MASK, regval[0]);
	if (ret < 0) {
		dev_err(dai->dev, "configure bck and sr fail\n");
		goto out_hw_params;
	}
	regaddr = capture ? RT5503_REG_ADCAUDFMT : RT5503_REG_DACAUDFMT;
	ret = snd_soc_update_bits(dai->codec, regaddr,
			RT5503_AUDBIT_MASK, regval[1]);
	if (ret < 0) {
		dev_err(dai->dev, "configure audbit fail\n");
		goto out_hw_params;
	}
	ret = rt5503_aif_set_pll(dai, 0, 0, 0, 0);
	if (ret < 0)
		dev_err(dai->dev, "pll config fail\n");
out_hw_params:
	return ret;
}

static int rt5503_aif_startup(struct snd_pcm_substream *substream,
	struct snd_soc_dai *dai)
{
	dev_info(dai->dev, "%s\n", __func__);
	return 0;
}

static void rt5503_aif_shutdown(struct snd_pcm_substream *substream,
	struct snd_soc_dai *dai)
{
	dev_info(dai->dev, "%s\n", __func__);
}

static int rt5503_aif_trigger(struct snd_pcm_substream *substream,
	int cmd, struct snd_soc_dai *dai)
{
	int capture = (substream->stream == SNDRV_PCM_STREAM_CAPTURE);

	dev_info(dai->dev, "%s: cmd=%d\n", __func__, cmd);
	dev_info(dai->dev, "%s: %c\n", __func__, capture ? 'c' : 'p');
	return 0;
}

static const struct snd_soc_dai_ops rt5503_dai_ops = {
	.set_fmt = rt5503_aif_set_fmt,
	.hw_params = rt5503_aif_hw_params,
	.digital_mute = rt5503_aif_digital_mute,
	.startup = rt5503_aif_startup,
	.shutdown = rt5503_aif_shutdown,
	.trigger = rt5503_aif_trigger,
};

#define RT5503_RATES SNDRV_PCM_RATE_8000_192000
#define RT5503_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S18_3LE | \
	SNDRV_PCM_FMTBIT_S20_3LE | SNDRV_PCM_FMTBIT_S24_LE)

static struct snd_soc_dai_driver rt5503_i2s_dais[] = {
	{
		.name = "rt5503-aif1",
		.playback = {
			.stream_name = "AIF1 Playback",
			.channels_min = 1,
			.channels_max = 2,
			.rates = RT5503_RATES,
			.formats = RT5503_FORMATS,
		},
		.capture = {
			.stream_name = "AIF1 Capture",
			.channels_min = 1,
			.channels_max = 2,
			.rates = RT5503_RATES,
			.formats = RT5503_FORMATS,
		},
		.ops = &rt5503_dai_ops,
	},
};

static inline int rt5503_codec_register(struct rt5503_chip *chip)
{
	return snd_soc_register_codec(chip->dev, &rt5503_codec_drv,
		rt5503_i2s_dais, ARRAY_SIZE(rt5503_i2s_dais));
}

static inline int rt5503_codec_unregister(struct rt5503_chip *chip)
{
	snd_soc_unregister_codec(chip->dev);
	return 0;
}

static inline void rt5503_report_headset_type(struct rt5503_chip *chip,
	u8 cur_headset_stat)
{
	switch(cur_headset_stat) {
	case RT_STEREO_HEADPHONE:
		dev_info(chip->dev, "headphone insert\n");
		input_report_switch(chip->input, SW_HEADPHONE_INSERT, 1);
		input_sync(chip->input);
		break;
	case RT_MONO_HEADSET:
	case RT_STEREO_HEADSET:
		dev_info(chip->dev, "headset insert\n");
		input_report_switch(chip->input, SW_HEADPHONE_INSERT, 1);
		input_report_switch(chip->input, SW_MICROPHONE_INSERT, 1);
		input_sync(chip->input);
		break;
	case RT_HEADSET_NONE:
	default:
		dev_info(chip->dev, "none plugin\n");
		input_report_switch(chip->input, SW_HEADPHONE_INSERT, 0);
		input_report_switch(chip->input, SW_MICROPHONE_INSERT, 0);
		input_sync(chip->input);
	}
	/* replace the old value */
	down(&chip->var_semaphore);
	chip->headset_stat = cur_headset_stat;
	up(&chip->var_semaphore);
}

static void rt5503_hdet_irq_dwork(struct work_struct *work)
{
	struct rt5503_chip *chip = (struct rt5503_chip *)container_of(work,
		struct rt5503_chip, dwork.work);
	u8 cur_headset_stat = RT_HEADSET_NONE;
	u8 cur_headset_in = 0;
	u32 read_data = 0;
	int ret = 0;

	dev_info(chip->dev, "%s\n", __func__);
	cur_headset_in = (chip->pdata->hdet_active_level ==
		gpio_get_value(chip->pdata->hdet_gpio)) ? 1 : 0;
	if (!(cur_headset_in ^ chip->headset_in))
		goto out_irq_dwork;
	if (cur_headset_in == chip->pdata->hdet_active_level) {
		/* turn on pll for micdetadc */
		ret = rt5503_set_bits(chip->i2c, RT5503_REG_BLOCKEN,
			RT5503_PLLEN_MASK);
		if (ret < 0) {
			dev_err(chip->dev, "set PLL enable fail\n");
			goto out_irq_dwork;
		}
		ret = rt5503_set_bits(chip->i2c, RT5503_REG_MICCFG1,
			RT5503_MICBEN_MASK);
		if (ret < 0) {
			dev_err(chip->dev, "micbias enable fail\n");
			goto out_irq_dwork;
		}
		/* micbias delay */
		mdelay(2);
		/* set mic_is_deb_en = 0, mic_is_en = 1 */
		ret = rt5503_update_bits(chip->i2c, RT5503_REG_MICCFG1,
			RT5503_MICISEN_MASK | RT5503_MICISDEBEN_MASK,
			RT5503_MICISEN_MASK, 1);
		if (ret < 0) {
			dev_err(chip->dev, "set mic_is function fail\n");
			goto out_irq_dwork;
		}
		/* micdetadc read delay */
		mdelay(3);
		ret = rt5503_reg_block_read(chip->i2c, RT5503_REG_MICDETADCOUT, 2,
			&read_data);
		if (ret < 0) {
			dev_err(chip->dev, "read micdetadc fail\n");
			goto out_irq_dwork;
		}
		ret = RT5503_MICISADC_RESOL * RT5503_MICDET_PERCENT / 100;
		if (read_data > ret)
			cur_headset_stat = RT_STEREO_HEADSET;
		else
			cur_headset_stat = RT_STEREO_HEADPHONE;
		/* turn off pll */
		ret = rt5503_clr_bits(chip->i2c, RT5503_REG_BLOCKEN,
			RT5503_PLLEN_MASK);
		if (ret < 0) {
			dev_err(chip->dev, "set PLL enable fail\n");
			goto out_irq_dwork;
		}
		if (cur_headset_stat == RT_STEREO_HEADPHONE) {
			/* turn off micbias for headphone, for hs, keep on */
			ret = rt5503_clr_bits(chip->i2c, RT5503_REG_MICCFG1,
				RT5503_MICBEN_MASK);
			if (ret < 0) {
				dev_err(chip->dev, "micbias disable fail\n");
				goto out_irq_dwork;
			}
		}
		/* re-check */
		ret = (chip->pdata->hdet_active_level ==
			gpio_get_value(chip->pdata->hdet_gpio)) ? 1 : 0;
		if (ret != cur_headset_in)
			goto out_irq_dwork;
	} else {
		/* whatever the headset is, just turn off micbias anyway */
		ret = rt5503_clr_bits(chip->i2c, RT5503_REG_MICCFG1,
			RT5503_MICBEN_MASK);
		if (ret < 0) {
			dev_err(chip->dev, "micbias disable fail\n");
			goto out_irq_dwork;
		}
	}
	chip->headset_in = cur_headset_in;
	rt5503_report_headset_type(chip, cur_headset_stat);
out_irq_dwork:
	pm_relax(chip->dev);
}

static irqreturn_t rt5503_hdet_irq_handler(int irq, void *param)
{
	struct rt5503_chip *chip = (struct rt5503_chip *)param;

	pm_stay_awake(chip->dev);
	cancel_delayed_work_sync(&chip->dwork);
	schedule_delayed_work(&chip->dwork,
		msecs_to_jiffies(RT5503_HDET_DEBOUNCE));
	return IRQ_HANDLED;
}

static inline int rt5503_register_input_device(struct rt5503_chip *chip)
{
	struct input_dev *input;
	int ret = 0;

	input = input_allocate_device();
	if (!input) {
		dev_err(chip->dev, "alloc input dev fail\n");
		return -ENOMEM;
	}
	input->name = "RT5503 Headset Detection";
	input->phys = dev_name(chip->dev);
	input->id.bustype = BUS_I2C;
	/* capability for Headphone and Microhpone event */
	input_set_capability(input, EV_SW, SW_HEADPHONE_INSERT);
	input_set_capability(input, EV_SW, SW_MICROPHONE_INSERT);
	ret = input_register_device(input);
	if (ret < 0) {
		dev_err(chip->dev, "register input device fail\n");
		goto err_input_register;
	}
	INIT_DELAYED_WORK(&chip->dwork, rt5503_hdet_irq_dwork);
	chip->input = input;
	return 0;
err_input_register:
	input_free_device(input);
	return ret;
}

static int rt5503_handle_pdata(struct rt5503_chip *chip)
{
	int ret = 0;

	chip->irq = -1;
	if (chip->pdata->hdet_gpio >= 0) {
		ret = rt5503_register_input_device(chip);
		if (ret < 0) {
			dev_err(chip->dev, "register input fail\n");
			goto err_input;
		}
		ret = gpio_request_one(chip->pdata->hdet_gpio, GPIOF_IN,
				"rt5503_hdet");
		if (ret < 0) {
			dev_err(chip->dev, "gpio request fail\n");
			goto err_request_gpio;
		}
		chip->irq = gpio_to_irq(chip->pdata->hdet_gpio);
		if (chip->irq < 0) {
			dev_err(chip->dev, "get gpio irq fail\n");
			ret = -EINVAL;
			goto err_gpio_irq;
		}
		ret = devm_request_threaded_irq(chip->dev, chip->irq, NULL,
			rt5503_hdet_irq_handler,IRQF_TRIGGER_RISING |
			IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
			"rt5503_hdet", chip);
		if (ret < 0) {
			dev_err(chip->dev, "request irq fail\n");
			goto err_request_irq;
		}
		device_init_wakeup(chip->dev, true);
		/* check for the initial state */
		rt5503_hdet_irq_handler(chip->irq, chip);
	}
	return 0;
err_request_irq:
err_gpio_irq:
	gpio_free(chip->pdata->hdet_gpio);
err_request_gpio:
	cancel_delayed_work_sync(&chip->dwork);
	input_unregister_device(chip->input);
err_input:
	return ret;
}

static inline int rt5503_i2c_initreg(struct rt5503_chip *chip)
{
	/* mute the HPL and HPR */
	return rt5503_set_bits(chip->i2c, RT5503_REG_CFG1, RT5503_MUTE_MASK);
}

static int rt5503_sw_reset(struct rt5503_chip *chip)
{
//HTC_AUD_START
//TODO: sw_reset has issue...
#if 1
	pr_err("%s: skip sw reset...\n", __func__);
	return 0;
#else
	int ret = 0;
	u8 data = 0;

	/* check revision id */
	ret = rt5503_block_read(chip->i2c, RT5503_REG_VERID, 1, &data);
	if (ret < 0)
		return ret;
	/* revision A -> 0, else increment by 1 */
	dev_info(chip->dev, "chip revision -> %d\n", data);
	if (data <= 0)
		return ret;
	dev_err(chip->dev, "%s\n", __func__);
	ret = rt5503_block_read(chip->i2c, RT5503_REG_CHIPEN, 1, &data);
	if (ret < 0)
		return ret;
	data |= RT5503_SWRST_MASK;
	ret = rt5503_block_write(chip->i2c, RT5503_REG_CHIPEN, 1, &data);
	mdelay(30);
	return ret;
#endif
//HTC_AUD_END
}

static inline int _rt5503_power_on(struct rt5503_chip *chip, bool en)
{
	u8 data = RT5503_CHIPEN_MASK;

	dev_info(chip->dev, "%s: en %d\n", __func__, en);
	return rt5503_block_write(chip->i2c, RT5503_REG_CHIPEN, 1, &data);
}

#ifdef CONFIG_OF
static int rt5503_parse_dt(struct device *dev, struct rt5503_pdata *pdata)
{
	pdata ->hdet_gpio = of_get_named_gpio(dev->of_node, "rt,hdet_gpio", 0);
	if (of_property_read_bool(dev->of_node, "rt,hdet_active_low") < 0)
		pdata->hdet_active_level = 0;
	else
		pdata->hdet_active_level = 1;

	return 0;
}
#else
static int rt5503_parse_dt(struct device *dev, struct rt5503_pdata *pdata)
{
	return 0;
}
#endif /* #ifdef CONFIG_OF */

static int rt5503_i2c_probe(struct i2c_client *client,
			    const struct i2c_device_id *id)
{
	struct rt5503_pdata *pdata = client->dev.platform_data;
	struct rt5503_chip *chip;
	int ret = 0;

//HTC_AUD_START
	if(apr_get_q6_state() == APR_SUBSYS_DOWN) {
		return -EPROBE_DEFER;
	}
//HTC_AUD_END

	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev, sizeof(*pdata), GFP_KERNEL);
		if (!pdata) {
			dev_err(&client->dev, "Failed to allocate memory\n");
			return -ENOMEM;
		}
		ret = rt5503_parse_dt(&client->dev, pdata);
		if (ret < 0)
			goto err_parse_dt;
		client->dev.platform_data = pdata;
	} else {
		if (!pdata) {
			dev_err(&client->dev, "Failed, no pdata specified\n");
			return -EINVAL;
		}
	}

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip) {
		dev_err(&client->dev, "Failed, on memory allocation\n");
		goto err_parse_dt;
	}
	chip->i2c = client;
	chip->dev = &client->dev;
	chip->pdata = pdata;
	i2c_set_clientdata(client, chip);
#if RT5503_SIMULATE_DEVICE
	ret = rt5503_calculate_total_size();
	chip->sim = devm_kzalloc(&client->dev, ret, GFP_KERNEL);
	if (!chip->sim) {
		ret = -ENOMEM;
		goto err_simulate;
	}
#endif /* #if RT5503_SIMULATE_DEVICE */

	sema_init(&chip->io_semaphore, 1);
#ifdef CONFIG_PM_RUNTIME
	pm_runtime_set_active(chip->dev);
	pm_runtime_enable(chip->dev);
#else
	atomic_set(&chip->power_count, 1);
#endif /* #ifdef CONFIG_PM_RUNTIME */

	ret = _rt5503_power_on(chip, true);
	if (ret < 0) {
		dev_err(chip->dev, "power on fail\n");
		goto err_pm_init;
	}

	/* do software reset at default */
	ret = rt5503_sw_reset(chip);
	if (ret < 0) {
		dev_err(chip->dev, "sw_reset fail\n");
		goto err_sw_reset;
	}

	/* register RegMAP */
	chip->rd = rt5503_regmap_register(
		&rt5503_regmap_ops, &client->dev, (void *)client, chip);
	if (!chip->rd) {
		dev_err(chip->dev, "create regmap device fail\n");
		ret = -EINVAL;
		goto err_regmap;
	}
	ret = rt5503_i2c_initreg(chip);
	if (ret < 0) {
		dev_err(chip->dev, "init_reg fail\n");
		goto err_initreg;
	}
	ret = rt5503_handle_pdata(chip);
	if (ret < 0) {
		dev_err(chip->dev, "init_pdata fail\n");
		goto err_pdata;
	}
	ret = rt5503_power_on(chip, false);
	if (ret < 0) {
		dev_err(chip->dev, "power off fail\n");
		goto err_put_sync;
	}
	ret = rt5503_codec_register(chip);
	if (ret < 0) {
		dev_err(chip->dev, "codec register fail\n");
		goto err_put_sync;
	}
	dev_info(&client->dev, "successfully driver probed\n");
	return 0;
err_put_sync:
err_pdata:
err_initreg:
#ifdef CONFIG_RT_REGMAP
	rt_regmap_device_unregister(chip->rd);
#endif /* #ifdef CONFIG_RT_REGMAP */
err_regmap:
err_sw_reset:
	_rt5503_power_on(chip, false);
err_pm_init:
#ifdef CONFIG_PM_RUNTIME
	pm_runtime_disable(chip->dev);
	pm_runtime_set_suspended(chip->dev);
#else
	atomic_dec(&chip->power_count);
#endif /* #ifdef CONFIG_PM_RUNTIME */
#if RT5503_SIMULATE_DEVICE
	devm_kfree(chip->dev, chip->sim);
err_simulate:
#endif /* #if RT5503_SIMULATE_DEVICE */
	devm_kfree(&client->dev, chip);
err_parse_dt:
	if (client->dev.of_node)
		devm_kfree(&client->dev, pdata);
	return ret;
}

static int rt5503_i2c_remove(struct i2c_client *client)
{
	struct rt5503_chip *chip = i2c_get_clientdata(client);

	if (chip->input){
		devm_free_irq(chip->dev, chip->irq, chip);
		cancel_delayed_work_sync(&chip->dwork);
		input_unregister_device(chip->input);
		gpio_free(chip->pdata->hdet_gpio);
	}
	rt5503_codec_unregister(chip);
#ifdef CONFIG_RT_REGMAP
	rt_regmap_device_unregister(chip->rd);
#endif /* #ifdef CONFIG_RT_REGMAP */
#ifdef CONFIG_PM_RUNTIME
	pm_runtime_disable(chip->dev);
	pm_runtime_set_suspended(chip->dev);
#else
	atomic_set(&chip->power_count, 0);
#endif /* #ifdef CONFIG_PM_RUNTIME */
	_rt5503_power_on(chip, false);
#if RT5503_SIMULATE_DEVICE
	devm_kfree(chip->dev, chip->sim);
#endif /* #if RT5503_SIMULATE_DEVICE */
	devm_kfree(chip->dev, chip->pdata);
	chip->pdata = client->dev.platform_data = NULL;
	dev_info(&client->dev, "driver removed\n");
	return 0;
}

#ifdef CONFIG_PM
static int rt5503_i2c_suspend(struct device *dev)
{
	struct rt5503_chip *chip = dev_get_drvdata(dev);

	if (device_may_wakeup(dev))
		enable_irq_wake(chip->irq);
	return 0;
}

static int rt5503_i2c_resume(struct device *dev)
{
	struct rt5503_chip *chip = dev_get_drvdata(dev);

	if (device_may_wakeup(dev))
		disable_irq_wake(chip->irq);
	return 0;
}

static int rt5503_i2c_runtime_suspend(struct device *dev)
{
	dev_info(dev, "%s\n", __func__);
	return 0;
}

static int rt5503_i2c_runtime_resume(struct device *dev)
{
	dev_info(dev, "%s\n", __func__);
	return 0;
}

static int rt5503_i2c_runtime_idle(struct device *dev)
{
	/* dummy function */
	return 0;
}

static const struct dev_pm_ops rt5503_i2c_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(rt5503_i2c_suspend, rt5503_i2c_resume)
	SET_RUNTIME_PM_OPS(rt5503_i2c_runtime_suspend,
			   rt5503_i2c_runtime_resume,
			   rt5503_i2c_runtime_idle)
};
#define prt5503_i2c_pm_ops (&rt5503_i2c_pm_ops)
#else
#define prt5503_i2c_pm_ops (NULL)
#endif /* #ifdef CONFIG_PM */

static const struct i2c_device_id rt5503_i2c_id[] = {
	{RT5503_DEVICE_NAME, 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, rt5503_i2c_id);

#ifdef CONFIG_OF
static const struct of_device_id rt5503_match_table[] = {
	{.compatible = "richtek,rt5503",},
	{},
};
MODULE_DEVICE_TABLE(of, rt5503_match_table);
#endif /* #ifdef CONFIG_OF */

static struct i2c_driver rt5503_i2c_driver = {
	.driver = {
		.name = RT5503_DEVICE_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(rt5503_match_table),
		.pm = prt5503_i2c_pm_ops,
	},
	.probe = rt5503_i2c_probe,
	.remove = rt5503_i2c_remove,
	.id_table = rt5503_i2c_id,
};

module_i2c_driver(rt5503_i2c_driver);

MODULE_AUTHOR("CY_Huang <cy_huang@richtek.com>");
MODULE_DESCRIPTION("RT5503 HP AMP Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(RT5503_DRV_VER);
