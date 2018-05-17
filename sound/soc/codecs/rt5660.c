/*
 * rt5660.c  --  RT5660 ALSA SoC audio codec driver
 *
 * Copyright 2012 Realtek Semiconductor Corp.
 * Author: Bard Liao <bardliao@realtek.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>
#include <dt-bindings/gpio/gpio.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/rockchip/cru.h>
#include <linux/rockchip/grf.h>

//#define USE_DMIC

/* #define RTK_IOCTL */
#ifdef RTK_IOCTL
#if defined(CONFIG_SND_HWDEP) || defined(CONFIG_SND_HWDEP_MODULE)
#include "rt_codec_ioctl.h"
#include "rt5660_ioctl.h"
#endif
#endif

#include "rt5660.h"

#define VERSION "0.0.1 alsa 1.0.25"

struct rt5660_init_reg {
	u8 reg;
	u16 val;
};

static struct rt5660_init_reg init_list[] = {
	{ RT5660_GEN_CTRL1	, 0x0801 },
	{ RT5660_PRIV_INDEX	, 0x003d },
	{ RT5660_PRIV_DATA	, 0x3600 },

	/* Playback */
	{ RT5660_STO_DAC_MIXER	, 0x0202 },
	/* SPK */
	{ RT5660_SPK_MIXER	, 0x001a },
	{ RT5660_SPO_MIXER	, 0xe800 },
	{ RT5660_SPK_VOL	, 0x8800 },
	/* LOUT */
	{ RT5660_LOUT_VOL	, 0x8888 },
	{ RT5660_OUT_L1_MIXER	, 0x01fe },
	{ RT5660_OUT_R1_MIXER	, 0x01fe },
	{ RT5660_LOUT_MIXER	, 0xa000 },

	/* Record */
#ifdef USE_DMIC
	/* DMIC */
	{ RT5660_STO1_ADC_MIXER	, 0x4040 },
#else
	/* AMIC */
	{ RT5660_STO1_ADC_MIXER	, 0x2020 },
	{ RT5660_REC_L2_MIXER	, 0x007f },
	{ RT5660_REC_R2_MIXER	, 0x007f },
#endif
	{ RT5660_LOUT_AMP_CTRL	, 0x0011 },
};
#define RT5660_INIT_REG_LEN ARRAY_SIZE(init_list)

#ifdef ALC_DRC_FUNC
static struct rt5660_init_reg alc_drc_list[] = {
	{ RT5660_DRC_AGC_CTRL1	, 0x0000 },
};
#define RT5660_ALC_DRC_REG_LEN ARRAY_SIZE(alc_drc_list)
#endif

static int rt5660_reg_init(struct snd_soc_codec *codec)
{
	int i;

	for (i = 0; i < RT5660_INIT_REG_LEN; i++)
		snd_soc_write(codec, init_list[i].reg, init_list[i].val);
#ifdef ALC_DRC_FUNC
	for (i = 0; i < RT5660_ALC_DRC_REG_LEN; i++)
		snd_soc_write(codec, alc_drc_list[i].reg, alc_drc_list[i].val);
#endif

	return 0;
}

static int rt5660_index_sync(struct snd_soc_codec *codec)
{
	int i;

	for (i = 0; i < RT5660_INIT_REG_LEN; i++)
		if (RT5660_PRIV_INDEX == init_list[i].reg ||
			RT5660_PRIV_DATA == init_list[i].reg)
			snd_soc_write(codec, init_list[i].reg,
					init_list[i].val);
	return 0;
}

static const u16 rt5660_reg[RT5660_VENDOR_ID2 + 1] = {
	[RT5660_SPK_VOL] = 0xc800,
	[RT5660_LOUT_VOL] = 0xc8c8,
	[RT5660_IN1_IN2] = 0x1010,
	[RT5660_IN3_IN4] = 0x1010,
	[RT5660_DAC1_DIG_VOL] = 0xafaf,
	[RT5660_STO1_ADC_DIG_VOL] = 0xafaf,//0x2f2f,
	[RT5660_STO1_ADC_MIXER] = 0x6060,
	[RT5660_AD_DA_MIXER] = 0x8080,
	[RT5660_STO_DAC_MIXER] = 0x4242,
	[RT5660_REC_L2_MIXER] = 0x007f,
	[RT5660_REC_R2_MIXER] = 0x007f,
	[RT5660_LOUT_MIXER] = 0xe000,
	[RT5660_SPK_MIXER] = 0x003e,
	[RT5660_SPO_MIXER] = 0xf800,
	[RT5660_SPO_CLSD_RATIO] = 0x0004,
	[RT5660_OUT_L1_MIXER] = 0x01ff,
	[RT5660_OUT_R1_MIXER] = 0x01ff,
	[RT5660_PWR_ANLG1] = 0x00c0,
	[RT5660_I2S1_SDP] = 0x8000,
	[RT5660_ADDA_CLK1] = 0x7000,
	[RT5660_ADDA_CLK2] = 0x3c00,
	[RT5660_DMIC_CTRL1] = 0x2800,
	[RT5660_CLSD_AMP_OC_CTRL] = 0x0228,
	[RT5660_CLSD_AMP_CTRL] = 0xa000,
	[RT5660_MICBIAS] = 0x3000,
	[RT5660_CLSD_OUT_CTRL1] = 0x0059,
	[RT5660_CLSD_OUT_CTRL2] = 0x0001,
	[RT5660_DIPOLE_MIC_CTRL1] = 0x5c80,
	[RT5660_DIPOLE_MIC_CTRL2] = 0x0146,
	[RT5660_DIPOLE_MIC_CTRL3] = 0x1f1f,
	[RT5660_DIPOLE_MIC_CTRL4] = 0x78c6,
	[RT5660_DIPOLE_MIC_CTRL5] = 0xe5ec,
	[RT5660_DIPOLE_MIC_CTRL6] = 0xba61,
	[RT5660_DIPOLE_MIC_CTRL7] = 0x3c78,
	[RT5660_DIPOLE_MIC_CTRL8] = 0x8ae2,
	[RT5660_DIPOLE_MIC_CTRL9] = 0xe5ec,
	[RT5660_DIPOLE_MIC_CTRL10] = 0xc600,
	[RT5660_DIPOLE_MIC_CTRL11] = 0xba61,
	[RT5660_DIPOLE_MIC_CTRL12] = 0x17ed,
	[RT5660_EQ_CTRL1] = 0x2080,
	[RT5660_DRC_AGC_CTRL1] = 0x001f,
	[RT5660_DRC_AGC_CTRL2] = 0x020c,
	[RT5660_DRC_AGC_CTRL3] = 0x1f00,
	[RT5660_DRC_AGC_CTRL5] = 0x4000,
	[RT5660_WIND_FILTER_CTRL1] = 0xa220,
	[RT5660_SV_ZCD1] = 0x0809,
	[RT5660_DRC1_LM_CTRL1] = 0x8000,
	[RT5660_DRC1_LM_CTRL2] = 0x0200,
	[RT5660_DRC2_LM_CTRL1] = 0x8000,
	[RT5660_DRC2_LM_CTRL2] = 0x0200,
	[RT5660_DRC2_CTRL1] = 0x001f,
	[RT5660_DRC2_CTRL2] = 0x020c,
	[RT5660_DRC2_CTRL3] = 0x1f00,
	[RT5660_DRC2_CTRL5] = 0x4000,
	[RT5660_ALC_PGA_CTRL1] = 0x00a6,
	[RT5660_ALC_PGA_CTRL2] = 0x04c3,
	[RT5660_ALC_PGA_CTRL3] = 0x27c8,
	[RT5660_ALC_PGA_CTRL4] = 0xbf50,
	[RT5660_ALC_PGA_CTRL5] = 0x0045,
	[RT5660_ALC_PGA_CTRL6] = 0x0007,
};

static int rt5660_reset(struct snd_soc_codec *codec)
{
	return snd_soc_write(codec, RT5660_RESET, 0);
}

/**
 * rt5660_index_write - Write private register.
 * @codec: SoC audio codec device.
 * @reg: Private register index.
 * @value: Private register Data.
 *
 * Modify private register for advanced setting. It can be written through
 * private index (0x6a) and data (0x6c) register.
 *
 * Returns 0 for success or negative error code.
 */
static int rt5660_index_write(struct snd_soc_codec *codec,
		unsigned int reg, unsigned int value)
{
	int ret;

	ret = snd_soc_write(codec, RT5660_PRIV_INDEX, reg);
	if (ret < 0) {
		dev_err(codec->dev, "Failed to set private addr: %d\n", ret);
		goto err;
	}
	ret = snd_soc_write(codec, RT5660_PRIV_DATA, value);
	if (ret < 0) {
		dev_err(codec->dev, "Failed to set private value: %d\n", ret);
		goto err;
	}
	return 0;

err:
	return ret;
}

/**
 * rt5660_index_read - Read private register.
 * @codec: SoC audio codec device.
 * @reg: Private register index.
 *
 * Read advanced setting from private register. It can be read through
 * private index (0x6a) and data (0x6c) register.
 *
 * Returns private register value or negative error code.
 */
static unsigned int rt5660_index_read(
	struct snd_soc_codec *codec, unsigned int reg)
{
	int ret;

	ret = snd_soc_write(codec, RT5660_PRIV_INDEX, reg);
	if (ret < 0) {
		dev_err(codec->dev, "Failed to set private addr: %d\n", ret);
		return ret;
	}
	return snd_soc_read(codec, RT5660_PRIV_DATA);
}

/**
 * rt5660_index_update_bits - update private register bits
 * @codec: audio codec
 * @reg: Private register index.
 * @mask: register mask
 * @value: new value
 *
 * Writes new register value.
 *
 * Returns 1 for change, 0 for no change, or negative error code.
 */
static int rt5660_index_update_bits(struct snd_soc_codec *codec,
	unsigned int reg, unsigned int mask, unsigned int value)
{
	unsigned int old, new;
	int change, ret;

	ret = rt5660_index_read(codec, reg);
	if (ret < 0) {
		dev_err(codec->dev, "Failed to read private reg: %d\n", ret);
		goto err;
	}

	old = ret;
	new = (old & ~mask) | (value & mask);
	change = old != new;
	if (change) {
		ret = rt5660_index_write(codec, reg, new);
		if (ret < 0) {
			dev_err(codec->dev,
				"Failed to write private reg: %d\n", ret);
			goto err;
		}
	}
	return change;

err:
	return ret;
}

static int rt5660_volatile_register(
	struct snd_soc_codec *codec, unsigned int reg)
{
	switch (reg) {
	case RT5660_RESET:
	case RT5660_PRIV_DATA:
	case RT5660_EQ_CTRL1:
	case RT5660_IRQ_CTRL2:
	case RT5660_INT_IRQ_ST:
	case RT5660_VENDOR_ID:
	case RT5660_VENDOR_ID1:
	case RT5660_VENDOR_ID2:
		return 1;
	default:
		return 0;
	}
}

static int rt5660_readable_register(
	struct snd_soc_codec *codec, unsigned int reg)
{
	switch (reg) {
	case RT5660_RESET:
	case RT5660_SPK_VOL:
	case RT5660_LOUT_VOL:
	case RT5660_IN1_IN2:
	case RT5660_IN3_IN4:
	case RT5660_DAC1_DIG_VOL:
	case RT5660_STO1_ADC_DIG_VOL:
	case RT5660_ADC_BST_VOL1:
	case RT5660_STO1_ADC_MIXER:
	case RT5660_AD_DA_MIXER:
	case RT5660_STO_DAC_MIXER:
	case RT5660_DIG_INF1_DATA:
	case RT5660_REC_L1_MIXER:
	case RT5660_REC_L2_MIXER:
	case RT5660_REC_R1_MIXER:
	case RT5660_REC_R2_MIXER:
	case RT5660_LOUT_MIXER:
	case RT5660_SPK_MIXER:
	case RT5660_SPO_MIXER:
	case RT5660_SPO_CLSD_RATIO:
	case RT5660_OUT_L_GAIN1:
	case RT5660_OUT_L_GAIN2:
	case RT5660_OUT_L1_MIXER:
	case RT5660_OUT_R_GAIN1:
	case RT5660_OUT_R_GAIN2:
	case RT5660_OUT_R1_MIXER:
	case RT5660_PWR_DIG1:
	case RT5660_PWR_DIG2:
	case RT5660_PWR_ANLG1:
	case RT5660_PWR_ANLG2:
	case RT5660_PWR_MIXER:
	case RT5660_PWR_VOL:
	case RT5660_PRIV_INDEX:
	case RT5660_PRIV_DATA:
	case RT5660_I2S1_SDP:
	case RT5660_ADDA_CLK1:
	case RT5660_ADDA_CLK2:
	case RT5660_DMIC_CTRL1:
	case RT5660_GLB_CLK:
	case RT5660_PLL_CTRL1:
	case RT5660_PLL_CTRL2:
	case RT5660_CLSD_AMP_OC_CTRL:
	case RT5660_CLSD_AMP_CTRL:
	case RT5660_LOUT_AMP_CTRL:
	case RT5660_SPK_AMP_SPKVDD:
	case RT5660_MICBIAS:
	case RT5660_CLSD_OUT_CTRL1:
	case RT5660_CLSD_OUT_CTRL2:
	case RT5660_DIPOLE_MIC_CTRL1:
	case RT5660_DIPOLE_MIC_CTRL2:
	case RT5660_DIPOLE_MIC_CTRL3:
	case RT5660_DIPOLE_MIC_CTRL4:
	case RT5660_DIPOLE_MIC_CTRL5:
	case RT5660_DIPOLE_MIC_CTRL6:
	case RT5660_DIPOLE_MIC_CTRL7:
	case RT5660_DIPOLE_MIC_CTRL8:
	case RT5660_DIPOLE_MIC_CTRL9:
	case RT5660_DIPOLE_MIC_CTRL10:
	case RT5660_DIPOLE_MIC_CTRL11:
	case RT5660_DIPOLE_MIC_CTRL12:
	case RT5660_EQ_CTRL1:
	case RT5660_EQ_CTRL2:
	case RT5660_DRC_AGC_CTRL1:
	case RT5660_DRC_AGC_CTRL2:
	case RT5660_DRC_AGC_CTRL3:
	case RT5660_DRC_AGC_CTRL4:
	case RT5660_DRC_AGC_CTRL5:
	case RT5660_JD_CTRL:
	case RT5660_IRQ_CTRL1:
	case RT5660_IRQ_CTRL2:
	case RT5660_INT_IRQ_ST:
	case RT5660_GPIO_CTRL1:
	case RT5660_GPIO_CTRL2:
	case RT5660_WIND_FILTER_CTRL1:
	case RT5660_SV_ZCD1:
	case RT5660_SV_ZCD2:
	case RT5660_DRC1_LM_CTRL1:
	case RT5660_DRC1_LM_CTRL2:
	case RT5660_DRC2_LM_CTRL1:
	case RT5660_DRC2_LM_CTRL2:
	case RT5660_DRC2_CTRL1:
	case RT5660_DRC2_CTRL2:
	case RT5660_DRC2_CTRL3:
	case RT5660_DRC2_CTRL4:
	case RT5660_DRC2_CTRL5:
	case RT5660_ALC_PGA_CTRL1:
	case RT5660_ALC_PGA_CTRL2:
	case RT5660_ALC_PGA_CTRL3:
	case RT5660_ALC_PGA_CTRL4:
	case RT5660_ALC_PGA_CTRL5:
	case RT5660_ALC_PGA_CTRL6:
	case RT5660_GEN_CTRL1:
	case RT5660_GEN_CTRL2:
	case RT5660_GEN_CTRL3:
	case RT5660_VENDOR_ID:
	case RT5660_VENDOR_ID1:
	case RT5660_VENDOR_ID2:
		return 1;
	default:
		return 0;
	}
}

static const DECLARE_TLV_DB_SCALE(out_vol_tlv, -4650, 150, 0);
static const DECLARE_TLV_DB_SCALE(dac_vol_tlv, -65625, 375, 0);
static const DECLARE_TLV_DB_SCALE(in_vol_tlv, -3450, 150, 0);
static const DECLARE_TLV_DB_SCALE(adc_vol_tlv, -17625, 375, 0);
static const DECLARE_TLV_DB_SCALE(adc_bst_tlv, 0, 1200, 0);

/* {0, +20, +24, +30, +35, +40, +44, +50, +52} dB */
static unsigned int bst_tlv[] = {
	TLV_DB_RANGE_HEAD(7),
	0, 0, TLV_DB_SCALE_ITEM(0, 0, 0),
	1, 1, TLV_DB_SCALE_ITEM(2000, 0, 0),
	2, 2, TLV_DB_SCALE_ITEM(2400, 0, 0),
	3, 5, TLV_DB_SCALE_ITEM(3000, 500, 0),
	6, 6, TLV_DB_SCALE_ITEM(4400, 0, 0),
	7, 7, TLV_DB_SCALE_ITEM(5000, 0, 0),
	8, 8, TLV_DB_SCALE_ITEM(5200, 0, 0),
};

/* IN1/IN2/IN3/IN4 Input Type */
static const char * const rt5660_input_mode[] = {
	"Single ended", "Differential"
};

static const SOC_ENUM_SINGLE_DECL(
	rt5660_in1_mode_enum, RT5660_IN1_IN2,
	RT5660_IN_SFT1, rt5660_input_mode);

static const SOC_ENUM_SINGLE_DECL(
	rt5660_in2_mode_enum, RT5660_IN1_IN2,
	RT5660_IN_SFT2, rt5660_input_mode);

static const SOC_ENUM_SINGLE_DECL(
	rt5660_in3_mode_enum, RT5660_IN3_IN4,
	RT5660_IN_SFT3, rt5660_input_mode);

static const SOC_ENUM_SINGLE_DECL(
	rt5660_in4_mode_enum, RT5660_IN3_IN4,
	RT5660_IN_SFT4, rt5660_input_mode);

static const struct snd_kcontrol_new rt5660_snd_controls[] = {
	/* Speaker Output Volume */
	SOC_SINGLE("Speaker Playback Switch", RT5660_SPK_VOL,
		RT5660_L_MUTE_SFT, 1, 1),
	SOC_SINGLE("Speaker Channel Switch", RT5660_SPK_VOL,
		RT5660_VOL_L_SFT, 1, 1),
	SOC_SINGLE_TLV("Speaker Playback Volume", RT5660_SPK_VOL,
		RT5660_L_VOL_SFT, 39, 1, out_vol_tlv),
	/* OUTPUT Control */
	SOC_DOUBLE("OUT Playback Switch", RT5660_LOUT_VOL,
		RT5660_L_MUTE_SFT, RT5660_R_MUTE_SFT, 1, 1),
	SOC_DOUBLE("OUT Channel Switch", RT5660_LOUT_VOL,
		RT5660_VOL_L_SFT, RT5660_VOL_R_SFT, 1, 1),
	SOC_DOUBLE_TLV("OUT Playback Volume", RT5660_LOUT_VOL,
		RT5660_L_VOL_SFT, RT5660_R_VOL_SFT, 39, 1, out_vol_tlv),
	/* DAC Digital Volume */
	SOC_DOUBLE_TLV("DAC1 Playback Volume", RT5660_DAC1_DIG_VOL,
		RT5660_L_VOL_SFT, RT5660_R_VOL_SFT, 175, 0, dac_vol_tlv),
	/* IN1/IN2/IN3/IN4 Control */
	SOC_ENUM("IN1 Mode Control",  rt5660_in1_mode_enum),
	SOC_SINGLE_TLV("IN1 Boost", RT5660_IN1_IN2,
		RT5660_BST_SFT1, 8, 0, bst_tlv),
	SOC_ENUM("IN2 Mode Control", rt5660_in2_mode_enum),
	SOC_SINGLE_TLV("IN2 Boost", RT5660_IN1_IN2,
		RT5660_BST_SFT2, 8, 0, bst_tlv),
	SOC_ENUM("IN3 Mode Control",  rt5660_in3_mode_enum),
	SOC_SINGLE_TLV("IN3 Boost", RT5660_IN3_IN4,
		RT5660_BST_SFT3, 8, 0, bst_tlv),
	SOC_ENUM("IN4 Mode Control", rt5660_in4_mode_enum),
	SOC_SINGLE_TLV("IN4 Boost", RT5660_IN3_IN4,
		RT5660_BST_SFT4, 8, 0, bst_tlv),
	/* ADC Digital Volume Control */
	SOC_DOUBLE("ADC Capture Switch", RT5660_STO1_ADC_DIG_VOL,
		RT5660_L_MUTE_SFT, RT5660_R_MUTE_SFT, 1, 1),
	SOC_DOUBLE_TLV("ADC Capture Volume", RT5660_STO1_ADC_DIG_VOL,
			RT5660_L_VOL_SFT, RT5660_R_VOL_SFT,
			127, 0, adc_vol_tlv),
	/* ADC Boost Volume Control */
	SOC_DOUBLE_TLV("STO1 ADC Boost Gain", RT5660_ADC_BST_VOL1,
			RT5660_STO1_ADC_L_BST_SFT, RT5660_STO1_ADC_R_BST_SFT,
			3, 0, adc_bst_tlv),
};

/**
 * set_dmic_clk - Set parameter of dmic.
 *
 * @w: DAPM widget.
 * @kcontrol: The kcontrol of this widget.
 * @event: Event id.
 *
 * Choose dmic clock between 1MHz and 3MHz.
 * It is better for clock to approximate 3MHz.
 */
static int set_dmic_clk(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;
	struct rt5660_priv *rt5660 = snd_soc_codec_get_drvdata(codec);
	int div[] = {2, 3, 4, 6, 8, 12};
	int idx = -EINVAL, i;
	int rate, red, bound, temp;

	rate = rt5660->lrck[rt5660->aif_pu] << 8;
	/* red = 3000000 * 12; */
	red = 2000000 * 12;
	for (i = 0; i < ARRAY_SIZE(div); i++) {
		bound = div[i] * 2000000;
		if (rate > bound)
			continue;
		temp = bound - rate;
		if (temp < red) {
			red = temp;
			idx = i;
		}
	}

	if (idx < 0)
		dev_err(codec->dev, "Failed to set DMIC clock\n");
	else
		snd_soc_update_bits(codec, RT5660_DMIC_CTRL1,
			RT5660_DMIC_CLK_MASK, idx << RT5660_DMIC_CLK_SFT);
	return idx;
}

static int check_sysclk1_source(struct snd_soc_dapm_widget *source,
			 struct snd_soc_dapm_widget *sink)
{
	unsigned int val;

	val = snd_soc_read(source->codec, RT5660_GLB_CLK);
	val &= RT5660_SCLK_SRC_MASK;
	if (val == RT5660_SCLK_SRC_PLL1)
		return 1;
	else
		return 0;
}

/* Digital Mixer */
static const struct snd_kcontrol_new rt5660_sto1_adc_l_mix[] = {
	SOC_DAPM_SINGLE("ADC1 Switch", RT5660_STO1_ADC_MIXER,
			RT5660_M_ADC_L1_SFT, 1, 1),
	SOC_DAPM_SINGLE("ADC2 Switch", RT5660_STO1_ADC_MIXER,
			RT5660_M_ADC_L2_SFT, 1, 1),
};

static const struct snd_kcontrol_new rt5660_sto1_adc_r_mix[] = {
	SOC_DAPM_SINGLE("ADC1 Switch", RT5660_STO1_ADC_MIXER,
			RT5660_M_ADC_R1_SFT, 1, 1),
	SOC_DAPM_SINGLE("ADC2 Switch", RT5660_STO1_ADC_MIXER,
			RT5660_M_ADC_R2_SFT, 1, 1),
};

static const struct snd_kcontrol_new rt5660_dac_l_mix[] = {
	SOC_DAPM_SINGLE("Stereo ADC Switch", RT5660_AD_DA_MIXER,
			RT5660_M_ADCMIX_L_SFT, 1, 1),
	SOC_DAPM_SINGLE("DAC1 Switch", RT5660_AD_DA_MIXER,
			RT5660_M_DAC1_L_SFT, 1, 1),
};

static const struct snd_kcontrol_new rt5660_dac_r_mix[] = {
	SOC_DAPM_SINGLE("Stereo ADC Switch", RT5660_AD_DA_MIXER,
			RT5660_M_ADCMIX_R_SFT, 1, 1),
	SOC_DAPM_SINGLE("DAC1 Switch", RT5660_AD_DA_MIXER,
			RT5660_M_DAC1_R_SFT, 1, 1),
};

static const struct snd_kcontrol_new rt5660_sto_dac_l_mix[] = {
	SOC_DAPM_SINGLE("DAC L1 Switch", RT5660_STO_DAC_MIXER,
			RT5660_M_DAC_L1_SFT, 1, 1),
	SOC_DAPM_SINGLE("DAC R1 Switch", RT5660_STO_DAC_MIXER,
			RT5660_M_DAC_R1_STO_L_SFT, 1, 1),
};

static const struct snd_kcontrol_new rt5660_sto_dac_r_mix[] = {
	SOC_DAPM_SINGLE("DAC R1 Switch", RT5660_STO_DAC_MIXER,
			RT5660_M_DAC_R1_SFT, 1, 1),
	SOC_DAPM_SINGLE("DAC L1 Switch", RT5660_STO_DAC_MIXER,
			RT5660_M_DAC_L1_STO_R_SFT, 1, 1),
};

/* Analog Input Mixer */
static const struct snd_kcontrol_new rt5660_rec_l_mix[] = {
	SOC_DAPM_SINGLE("BST4 Switch", RT5660_REC_L2_MIXER,
			RT5660_M_BST4_RM_L_SFT, 1, 1),
	SOC_DAPM_SINGLE("BST3 Switch", RT5660_REC_L2_MIXER,
			RT5660_M_BST3_RM_L_SFT, 1, 1),
	SOC_DAPM_SINGLE("BST2 Switch", RT5660_REC_L2_MIXER,
			RT5660_M_BST2_RM_L_SFT, 1, 1),
	SOC_DAPM_SINGLE("BST1 Switch", RT5660_REC_L2_MIXER,
			RT5660_M_BST1_RM_L_SFT, 1, 1),
	SOC_DAPM_SINGLE("OUT MIXL Switch", RT5660_REC_L2_MIXER,
			RT5660_M_OM_L_RM_L_SFT, 1, 1),
};

static const struct snd_kcontrol_new rt5660_rec_r_mix[] = {
	SOC_DAPM_SINGLE("BST4 Switch", RT5660_REC_R2_MIXER,
			RT5660_M_BST4_RM_R_SFT, 1, 1),
	SOC_DAPM_SINGLE("BST3 Switch", RT5660_REC_R2_MIXER,
			RT5660_M_BST3_RM_R_SFT, 1, 1),
	SOC_DAPM_SINGLE("BST2 Switch", RT5660_REC_R2_MIXER,
			RT5660_M_BST2_RM_R_SFT, 1, 1),
	SOC_DAPM_SINGLE("BST1 Switch", RT5660_REC_R2_MIXER,
			RT5660_M_BST1_RM_R_SFT, 1, 1),
	SOC_DAPM_SINGLE("OUT MIXR Switch", RT5660_REC_R2_MIXER,
			RT5660_M_OM_R_RM_R_SFT, 1, 1),
};

static const struct snd_kcontrol_new rt5660_spk_mix[] = {
	SOC_DAPM_SINGLE("BST3 Switch", RT5660_SPK_MIXER,
			RT5660_M_BST3_SM_SFT, 1, 1),
	SOC_DAPM_SINGLE("BST1 Switch", RT5660_SPK_MIXER,
			RT5660_M_BST1_SM_SFT, 1, 1),
	SOC_DAPM_SINGLE("DACL Switch", RT5660_SPK_MIXER,
			RT5660_M_DACL_SM_SFT, 1, 1),
	SOC_DAPM_SINGLE("DACR Switch", RT5660_SPK_MIXER,
			RT5660_M_DACR_SM_SFT, 1, 1),
	SOC_DAPM_SINGLE("OUTMIXL Switch", RT5660_SPK_MIXER,
			RT5660_M_OM_L_SM_SFT, 1, 1),
};

static const struct snd_kcontrol_new rt5660_out_l_mix[] = {
	SOC_DAPM_SINGLE("BST3 Switch", RT5660_OUT_L1_MIXER,
			RT5660_M_BST3_OM_L_SFT, 1, 1),
	SOC_DAPM_SINGLE("BST2 Switch", RT5660_OUT_L1_MIXER,
			RT5660_M_BST2_OM_L_SFT, 1, 1),
	SOC_DAPM_SINGLE("BST1 Switch", RT5660_OUT_L1_MIXER,
			RT5660_M_BST1_OM_L_SFT, 1, 1),
	SOC_DAPM_SINGLE("RECMIXL Switch", RT5660_OUT_L1_MIXER,
			RT5660_M_RM_L_OM_L_SFT, 1, 1),
	SOC_DAPM_SINGLE("DACR Switch", RT5660_OUT_L1_MIXER,
			RT5660_M_DAC_R_OM_L_SFT, 1, 1),
	SOC_DAPM_SINGLE("DACL Switch", RT5660_OUT_L1_MIXER,
			RT5660_M_DAC_L_OM_L_SFT, 1, 1),
};

static const struct snd_kcontrol_new rt5660_out_r_mix[] = {
	SOC_DAPM_SINGLE("BST4 Switch", RT5660_OUT_R1_MIXER,
			RT5660_M_BST4_OM_R_SFT, 1, 1),
	SOC_DAPM_SINGLE("BST2 Switch", RT5660_OUT_R1_MIXER,
			RT5660_M_BST2_OM_R_SFT, 1, 1),
	SOC_DAPM_SINGLE("BST1 Switch", RT5660_OUT_R1_MIXER,
			RT5660_M_BST1_OM_R_SFT, 1, 1),
	SOC_DAPM_SINGLE("RECMIXR Switch", RT5660_OUT_R1_MIXER,
			RT5660_M_RM_R_OM_R_SFT, 1, 1),
	SOC_DAPM_SINGLE("DACR Switch", RT5660_OUT_R1_MIXER,
			RT5660_M_DAC_R_OM_R_SFT, 1, 1),
	SOC_DAPM_SINGLE("DACL Switch", RT5660_OUT_R1_MIXER,
			RT5660_M_DAC_L_OM_R_SFT, 1, 1),
};

static const struct snd_kcontrol_new rt5660_spo_mix[] = {
	SOC_DAPM_SINGLE("DACR Switch", RT5660_SPO_MIXER,
			RT5660_M_DAC_R_SPM_SFT, 1, 1),
	SOC_DAPM_SINGLE("DACL Switch", RT5660_SPO_MIXER,
			RT5660_M_DAC_L_SPM_SFT, 1, 1),
	SOC_DAPM_SINGLE("SPKVOL Switch", RT5660_SPO_MIXER,
			RT5660_M_SV_SPM_SFT, 1, 1),
	SOC_DAPM_SINGLE("BST1 Switch", RT5660_SPO_MIXER,
			RT5660_M_BST1_SPM_SFT, 1, 1),
};

static const struct snd_kcontrol_new rt5660_lout_mix[] = {
	SOC_DAPM_SINGLE("DAC Switch", RT5660_LOUT_MIXER,
			RT5660_M_DAC1_LM_SFT, 1, 1),
	SOC_DAPM_SINGLE("OUTMIX Switch", RT5660_LOUT_MIXER,
			RT5660_M_LOVOL_LM_SFT, 1, 1),
};

static const struct snd_kcontrol_new spk_vol_control =
	SOC_DAPM_SINGLE("Switch", RT5660_SPK_VOL,
		RT5660_VOL_L_SFT, 1, 1);

static const struct snd_kcontrol_new lout_l_vol_control =
	SOC_DAPM_SINGLE("Switch", RT5660_LOUT_VOL,
		RT5660_VOL_L_SFT, 1, 1);

static const struct snd_kcontrol_new lout_r_vol_control =
	SOC_DAPM_SINGLE("Switch", RT5660_LOUT_VOL,
		RT5660_VOL_R_SFT, 1, 1);

static int rt5660_adc_clk_event(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		rt5660_index_update_bits(codec,
			RT5660_CHOP_DAC_ADC, 0x1000, 0x1000);
		break;

	case SND_SOC_DAPM_POST_PMD:
		rt5660_index_update_bits(codec,
			RT5660_CHOP_DAC_ADC, 0x1000, 0x0000);
		break;

	default:
		return 0;
	}

	return 0;
}

static int rt5660_sto1_adcl_event(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		snd_soc_update_bits(codec, RT5660_STO1_ADC_DIG_VOL,
			RT5660_L_MUTE, 0);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		snd_soc_update_bits(codec, RT5660_STO1_ADC_DIG_VOL,
			RT5660_L_MUTE,
			RT5660_L_MUTE);
		break;

	default:
		return 0;
	}

	return 0;
}

static int rt5660_sto1_adcr_event(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		snd_soc_update_bits(codec, RT5660_STO1_ADC_DIG_VOL,
			RT5660_R_MUTE, 0);
		break;
	case SND_SOC_DAPM_PRE_PMD:
		snd_soc_update_bits(codec, RT5660_STO1_ADC_DIG_VOL,
			RT5660_R_MUTE,
			RT5660_R_MUTE);
		break;

	default:
		return 0;
	}

	return 0;
}
static int rt5660_spk_event(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		snd_soc_update_bits(codec, RT5660_PWR_DIG1,
			RT5660_PWR_CLS_D, RT5660_PWR_CLS_D);
		snd_soc_update_bits(codec, RT5660_SPK_VOL,
			RT5660_L_MUTE, 0);
	break;

	case SND_SOC_DAPM_PRE_PMD:
		snd_soc_update_bits(codec, RT5660_SPK_VOL,
			RT5660_L_MUTE, RT5660_L_MUTE);
		snd_soc_update_bits(codec, RT5660_PWR_DIG1,
			RT5660_PWR_CLS_D, 0);
		break;

	default:
		return 0;
	}

	return 0;
}

static int rt5660_lout_event(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;
	struct rt5660_priv *rt5660 = snd_soc_codec_get_drvdata(codec);

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		snd_soc_update_bits(codec, RT5660_PWR_ANLG1,
			RT5660_PWR_HP_L | RT5660_PWR_HP_R | RT5660_PWR_HA,
			RT5660_PWR_HP_L | RT5660_PWR_HP_R | RT5660_PWR_HA);
		/*snd_soc_update_bits(codec, RT5660_LOUT_AMP_CTRL,
			RT5660_LOUT_CO_MASK | RT5660_LOUT_CB_MASK,
			RT5660_LOUT_CO_EN | RT5660_LOUT_CB_PU);*/
		snd_soc_update_bits(codec, RT5660_LOUT_VOL,
			RT5660_L_MUTE | RT5660_R_MUTE, 0);
		mdelay(50);
		gpio_direction_output(rt5660->amp_mute_gpio, rt5660->amp_mute_gpio_active);
		break;

	case SND_SOC_DAPM_PRE_PMD:
		gpio_direction_output(rt5660->amp_mute_gpio, !rt5660->amp_mute_gpio_active);
		mdelay(50);
		snd_soc_update_bits(codec, RT5660_LOUT_VOL,
			RT5660_L_MUTE | RT5660_R_MUTE,
			RT5660_L_MUTE | RT5660_R_MUTE);
		/*snd_soc_update_bits(codec, RT5660_LOUT_AMP_CTRL,
			RT5660_LOUT_CO_MASK | RT5660_LOUT_CB_MASK,
			RT5660_LOUT_CO_DIS | RT5660_LOUT_CB_PD);*/
		snd_soc_update_bits(codec, RT5660_PWR_ANLG1,
			RT5660_PWR_HP_L | RT5660_PWR_HP_R | RT5660_PWR_HA, 0);
		break;

	default:
		return 0;
	}

	return 0;
}

static int rt5660_set_dmic_event(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		snd_soc_update_bits(codec, RT5660_GPIO_CTRL1,
			RT5660_GP1_PIN_MASK,
			RT5660_GP1_PIN_DMIC1_SCL);
		snd_soc_update_bits(codec, RT5660_GPIO_CTRL2,
			RT5660_GP1_PF_MASK, RT5660_GP1_PF_OUT);
		snd_soc_update_bits(codec, RT5660_DMIC_CTRL1,
			RT5660_DMIC_1L_LH_MASK | RT5660_DMIC_1R_LH_MASK |
			RT5660_SEL_DMIC_DATA_MASK,
			RT5660_DMIC_1L_LH_FALLING | RT5660_DMIC_1R_LH_RISING |
			RT5660_SEL_DMIC_DATA_IN1P);
	default:
		return 0;
	}

	return 0;
}

static int rt5660_dac_l_event(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		break;

	case SND_SOC_DAPM_PRE_PMD:
		break;

	default:
		return 0;
	}

	return 0;
}

static int rt5660_dac_r_event(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		break;

	case SND_SOC_DAPM_PRE_PMD:
		break;

	default:
		return 0;
	}

	return 0;
}

static int rt5660_post_event(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	return 0;
}

static int rt5660_pre_event(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	return 0;
}

static const struct snd_soc_dapm_widget rt5660_dapm_widgets[] = {
	SND_SOC_DAPM_SUPPLY("LDO2", RT5660_PWR_ANLG1,
		RT5660_PWR_LDO2_BIT, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("PLL1", RT5660_PWR_ANLG2,
		RT5660_PWR_PLL_BIT, 0, NULL, 0),

	/* Input Side */
	/* micbias */
	SND_SOC_DAPM_MICBIAS("micbias1", RT5660_PWR_ANLG2,
			RT5660_PWR_MB1_BIT, 0),
	SND_SOC_DAPM_MICBIAS("micbias2", RT5660_PWR_ANLG2,
			RT5660_PWR_MB2_BIT, 0),
	/* Input Lines */
	SND_SOC_DAPM_INPUT("DMIC L1"),
	SND_SOC_DAPM_INPUT("DMIC R1"),

	SND_SOC_DAPM_INPUT("IN1P"),
	SND_SOC_DAPM_INPUT("IN1N"),
	SND_SOC_DAPM_INPUT("IN2P"),
	SND_SOC_DAPM_INPUT("IN2N"),
	SND_SOC_DAPM_INPUT("IN3P"),
	SND_SOC_DAPM_INPUT("IN3N"),
	SND_SOC_DAPM_INPUT("IN4P"),
	SND_SOC_DAPM_INPUT("IN4N"),

	SND_SOC_DAPM_SUPPLY("DMIC CLK", SND_SOC_NOPM, 0, 0,
		set_dmic_clk, SND_SOC_DAPM_PRE_PMU),
	SND_SOC_DAPM_SUPPLY("DMIC Power", RT5660_DMIC_CTRL1,
		RT5660_DMIC_1_EN_SFT, 0, rt5660_set_dmic_event,
		SND_SOC_DAPM_PRE_PMU),

	/* Boost */
	SND_SOC_DAPM_PGA("BST1", RT5660_PWR_ANLG2, RT5660_PWR_BST1_BIT, 0,
		NULL, 0),
	SND_SOC_DAPM_PGA("BST2", RT5660_PWR_ANLG2, RT5660_PWR_BST2_BIT, 0,
		NULL, 0),
	SND_SOC_DAPM_PGA("BST3", RT5660_PWR_ANLG2, RT5660_PWR_BST3_BIT, 0,
		NULL, 0),
	SND_SOC_DAPM_PGA("BST4", RT5660_PWR_ANLG2, RT5660_PWR_BST4_BIT, 0,
		NULL, 0),

	/* REC Mixer */
	SND_SOC_DAPM_MIXER("RECMIXL", RT5660_PWR_MIXER, RT5660_PWR_RM_L_BIT,
			0, rt5660_rec_l_mix, ARRAY_SIZE(rt5660_rec_l_mix)),
	SND_SOC_DAPM_MIXER("RECMIXR", RT5660_PWR_MIXER, RT5660_PWR_RM_R_BIT,
			0, rt5660_rec_r_mix, ARRAY_SIZE(rt5660_rec_r_mix)),
	/* ADCs */
	SND_SOC_DAPM_ADC("ADC L", NULL, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_ADC("ADC R", NULL, SND_SOC_NOPM, 0, 0),

	SND_SOC_DAPM_SUPPLY("ADC L power", RT5660_PWR_DIG1,
			RT5660_PWR_ADC_L_BIT, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("ADC R power", RT5660_PWR_DIG1,
			RT5660_PWR_ADC_R_BIT, 0, NULL, 0),
	SND_SOC_DAPM_SUPPLY("ADC clock", SND_SOC_NOPM,
			0, 0, rt5660_adc_clk_event,
			SND_SOC_DAPM_POST_PMD |
			SND_SOC_DAPM_POST_PMU),

	/* ADC Mixer */
	SND_SOC_DAPM_SUPPLY("adc stereo1 filter", RT5660_PWR_DIG2,
		RT5660_PWR_ADC_S1F_BIT, 0, NULL, 0),
	SND_SOC_DAPM_MIXER_E("Sto1 ADC MIXL", SND_SOC_NOPM, 0, 0,
		rt5660_sto1_adc_l_mix, ARRAY_SIZE(rt5660_sto1_adc_l_mix),
		rt5660_sto1_adcl_event,	SND_SOC_DAPM_PRE_PMD |
		SND_SOC_DAPM_POST_PMU),
	SND_SOC_DAPM_MIXER_E("Sto1 ADC MIXR", SND_SOC_NOPM, 0, 0,
		rt5660_sto1_adc_r_mix, ARRAY_SIZE(rt5660_sto1_adc_r_mix),
		rt5660_sto1_adcr_event, SND_SOC_DAPM_PRE_PMD |
		SND_SOC_DAPM_POST_PMU),

	/* ADC PGA */
	SND_SOC_DAPM_PGA("Stereo1 ADC MIXL", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_PGA("Stereo1 ADC MIXR", SND_SOC_NOPM, 0, 0, NULL, 0),

	/* Digital Interface */
	SND_SOC_DAPM_SUPPLY("I2S1", RT5660_PWR_DIG1,
		RT5660_PWR_I2S1_BIT, 0, NULL, 0),
	SND_SOC_DAPM_PGA("IF1 DAC", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_PGA("IF1 DAC L", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_PGA("IF1 DAC R", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_PGA("IF1 ADC", SND_SOC_NOPM, 0, 0, NULL, 0),

	/* Audio Interface */
	SND_SOC_DAPM_AIF_IN("AIF1RX", "AIF1 Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_OUT("AIF1TX", "AIF1 Capture", 0, SND_SOC_NOPM, 0, 0),

	/* Output Side */
	/* DAC mixer before sound effect  */
	SND_SOC_DAPM_MIXER_E("DAC1 MIXL", SND_SOC_NOPM, 0, 0,
		rt5660_dac_l_mix, ARRAY_SIZE(rt5660_dac_l_mix),
		rt5660_dac_l_event, SND_SOC_DAPM_PRE_PMD |
		SND_SOC_DAPM_POST_PMU),
	SND_SOC_DAPM_MIXER_E("DAC1 MIXR", SND_SOC_NOPM, 0, 0,
		rt5660_dac_r_mix, ARRAY_SIZE(rt5660_dac_r_mix),
		rt5660_dac_r_event, SND_SOC_DAPM_PRE_PMD |
		SND_SOC_DAPM_POST_PMU),

	/* DAC Mixer */
	SND_SOC_DAPM_SUPPLY("dac stereo1 filter", RT5660_PWR_DIG2,
		RT5660_PWR_DAC_S1F_BIT, 0, NULL, 0),
	SND_SOC_DAPM_MIXER("Stereo DAC MIXL", SND_SOC_NOPM, 0, 0,
		rt5660_sto_dac_l_mix, ARRAY_SIZE(rt5660_sto_dac_l_mix)),
	SND_SOC_DAPM_MIXER("Stereo DAC MIXR", SND_SOC_NOPM, 0, 0,
		rt5660_sto_dac_r_mix, ARRAY_SIZE(rt5660_sto_dac_r_mix)),

	/* DACs */
	SND_SOC_DAPM_DAC("DAC L1", NULL, RT5660_PWR_DIG1,
			RT5660_PWR_DAC_L1_BIT, 0),
	SND_SOC_DAPM_DAC("DAC R1", NULL, RT5660_PWR_DIG1,
			RT5660_PWR_DAC_R1_BIT, 0),
	/* OUT Mixer */
	SND_SOC_DAPM_MIXER("SPK MIX", RT5660_PWR_MIXER, RT5660_PWR_SM_BIT,
		0, rt5660_spk_mix, ARRAY_SIZE(rt5660_spk_mix)),
	SND_SOC_DAPM_MIXER("OUT MIXL", RT5660_PWR_MIXER, RT5660_PWR_OM_L_BIT,
		0, rt5660_out_l_mix, ARRAY_SIZE(rt5660_out_l_mix)),
	SND_SOC_DAPM_MIXER("OUT MIXR", RT5660_PWR_MIXER, RT5660_PWR_OM_R_BIT,
		0, rt5660_out_r_mix, ARRAY_SIZE(rt5660_out_r_mix)),
	/* Ouput Volume */
	SND_SOC_DAPM_SWITCH("SPKVOL", RT5660_PWR_VOL,
		RT5660_PWR_SV_BIT, 0, &spk_vol_control),
	SND_SOC_DAPM_PGA("DAC 1", SND_SOC_NOPM,
		0, 0, NULL, 0),
	SND_SOC_DAPM_PGA("LOUTVOL", SND_SOC_NOPM,
		0, 0, NULL, 0),
	SND_SOC_DAPM_SWITCH("LOUTVOL L", SND_SOC_NOPM,
		RT5660_PWR_LV_L_BIT, 0, &lout_l_vol_control),
	SND_SOC_DAPM_SWITCH("LOUTVOL R", SND_SOC_NOPM,
		RT5660_PWR_LV_R_BIT, 0, &lout_r_vol_control),

	/* HPO/LOUT/Mono Mixer */
	SND_SOC_DAPM_MIXER("SPO MIX", SND_SOC_NOPM, 0,
		0, rt5660_spo_mix, ARRAY_SIZE(rt5660_spo_mix)),
	SND_SOC_DAPM_MIXER("LOUT MIX", SND_SOC_NOPM, 0, 0,
		rt5660_lout_mix, ARRAY_SIZE(rt5660_lout_mix)),


	SND_SOC_DAPM_PGA_S("LOUT amp", 1, SND_SOC_NOPM,
		0, 0, rt5660_lout_event,
		SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMU),
	SND_SOC_DAPM_PGA_S("SPK amp", 2, SND_SOC_NOPM,
		0, 0, rt5660_spk_event,
		SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMU),

	/* Output Lines */
	SND_SOC_DAPM_OUTPUT("LOUTL"),
	SND_SOC_DAPM_OUTPUT("LOUTR"),
	SND_SOC_DAPM_OUTPUT("SPO"),

	SND_SOC_DAPM_POST("DAPM_POST", rt5660_post_event),
	SND_SOC_DAPM_PRE("DAPM_PRE", rt5660_pre_event),
};

static const struct snd_soc_dapm_route rt5660_dapm_routes[] = {
	{ "BST1", NULL, "IN1P" },
	{ "BST1", NULL, "IN1N" },
	{ "BST2", NULL, "IN2P" },
	{ "BST2", NULL, "IN2N" },
	{ "BST3", NULL, "IN3P" },
	{ "BST3", NULL, "IN3N" },
	{ "BST4", NULL, "IN4P" },
	{ "BST4", NULL, "IN4N" },

	{ "BST1", NULL, "LDO2" },
	{ "BST2", NULL, "LDO2" },
	{ "BST3", NULL, "LDO2" },
	{ "BST4", NULL, "LDO2" },

	{ "RECMIXL", "BST4 Switch", "BST4" },
	{ "RECMIXL", "BST3 Switch", "BST3" },
	{ "RECMIXL", "BST2 Switch", "BST2" },
	{ "RECMIXL", "BST1 Switch", "BST1" },
	{ "RECMIXL", "OUT MIXL Switch", "OUT MIXL" },

	{ "RECMIXR", "BST4 Switch", "BST4" },
	{ "RECMIXR", "BST3 Switch", "BST3" },
	{ "RECMIXR", "BST2 Switch", "BST2" },
	{ "RECMIXR", "BST1 Switch", "BST1" },
	{ "RECMIXR", "OUT MIXR Switch", "OUT MIXR" },

	{ "ADC L", NULL, "RECMIXL" },
	{ "ADC L", NULL, "ADC L power" },
	{ "ADC L", NULL, "ADC clock" },
	{ "ADC R", NULL, "RECMIXR" },
	{ "ADC R", NULL, "ADC R power" },
	{ "ADC R", NULL, "ADC clock" },

	{"DMIC L1", NULL, "DMIC CLK"},
	{"DMIC L1", NULL, "DMIC Power"},
	{"DMIC R1", NULL, "DMIC CLK"},
	{"DMIC R1", NULL, "DMIC Power"},

	{ "Sto1 ADC MIXL", "ADC1 Switch", "ADC L" },
	{ "Sto1 ADC MIXL", "ADC2 Switch", "DMIC L1" },
	{ "Sto1 ADC MIXR", "ADC1 Switch", "ADC R" },
	{ "Sto1 ADC MIXR", "ADC2 Switch", "DMIC R1" },

	{ "Stereo1 ADC MIXL", NULL, "Sto1 ADC MIXL" },
	{ "Stereo1 ADC MIXL", NULL, "adc stereo1 filter" },
	{ "adc stereo1 filter", NULL, "PLL1", check_sysclk1_source },

	{ "Stereo1 ADC MIXR", NULL, "Sto1 ADC MIXR" },
	{ "Stereo1 ADC MIXR", NULL, "adc stereo1 filter" },
	{ "adc stereo1 filter", NULL, "PLL1", check_sysclk1_source },

	{ "IF1 ADC", NULL, "Stereo1 ADC MIXL" },
	{ "IF1 ADC", NULL, "Stereo1 ADC MIXR" },
	{ "IF1 ADC", NULL, "I2S1" },
	{ "AIF1TX", NULL, "IF1 ADC" },

	{ "IF1 DAC", NULL, "AIF1RX" },
	{ "IF1 DAC", NULL, "I2S1" },

	{ "IF1 DAC L", NULL, "IF1 DAC" },
	{ "IF1 DAC R", NULL, "IF1 DAC" },

	{ "DAC1 MIXL", "Stereo ADC Switch", "Stereo1 ADC MIXL" },
	{ "DAC1 MIXL", "DAC1 Switch", "IF1 DAC L" },
	{ "DAC1 MIXR", "Stereo ADC Switch", "Stereo1 ADC MIXR" },
	{ "DAC1 MIXR", "DAC1 Switch", "IF1 DAC R" },

	{ "Stereo DAC MIXL", "DAC L1 Switch", "DAC1 MIXL" },
	{ "Stereo DAC MIXL", "DAC R1 Switch", "DAC1 MIXR" },
	{ "Stereo DAC MIXL", NULL, "dac stereo1 filter" },
	{ "Stereo DAC MIXR", "DAC R1 Switch", "DAC1 MIXR" },
	{ "Stereo DAC MIXR", "DAC L1 Switch", "DAC1 MIXL" },
	{ "Stereo DAC MIXR", NULL, "dac stereo1 filter" },

	{ "DAC L1", NULL, "Stereo DAC MIXL" },
	{ "DAC L1", NULL, "PLL1", check_sysclk1_source },
	{ "DAC R1", NULL, "Stereo DAC MIXR" },
	{ "DAC R1", NULL, "PLL1", check_sysclk1_source },

	{ "SPK MIX", "BST3 Switch", "BST3" },
	{ "SPK MIX", "BST1 Switch", "BST1" },
	{ "SPK MIX", "DACL Switch", "DAC L1" },
	{ "SPK MIX", "DACR Switch", "DAC R1" },
	{ "SPK MIX", "OUTMIXL Switch", "OUT MIXL" },

	{ "OUT MIXL", "BST3 Switch", "BST3" },
	{ "OUT MIXL", "BST2 Switch", "BST2" },
	{ "OUT MIXL", "BST1 Switch", "BST1" },
	{ "OUT MIXL", "RECMIXL Switch", "RECMIXL" },
	{ "OUT MIXL", "DACR Switch", "DAC R1" },
	{ "OUT MIXL", "DACL Switch", "DAC L1" },

	{ "OUT MIXR", "BST4 Switch", "BST4" },
	{ "OUT MIXR", "BST2 Switch", "BST2" },
	{ "OUT MIXR", "BST1 Switch", "BST1" },
	{ "OUT MIXR", "RECMIXR Switch", "RECMIXR" },
	{ "OUT MIXR", "DACR Switch", "DAC R1" },
	{ "OUT MIXR", "DACL Switch", "DAC L1" },

	{ "SPO MIX", "DACR Switch", "DAC R1" },
	{ "SPO MIX", "DACL Switch", "DAC L1" },
	{ "SPO MIX", "SPKVOL Switch", "SPKVOL" },
	{ "SPO MIX", "BST1 Switch", "BST1" },

	{ "SPKVOL", "Switch", "SPK MIX" },
	{ "LOUTVOL L", "Switch", "OUT MIXL" },
	{ "LOUTVOL R", "Switch", "OUT MIXR" },

	{ "LOUTVOL", NULL, "LOUTVOL L" },
	{ "LOUTVOL", NULL, "LOUTVOL R" },

	{ "DAC 1", NULL, "DAC L1" },
	{ "DAC 1", NULL, "DAC R1" },

	{ "LOUT MIX", "DAC Switch", "DAC 1" },
	{ "LOUT MIX", "OUTMIX Switch", "LOUTVOL" },

	{ "LOUT amp", NULL, "LOUT MIX" },
	{ "LOUTL", NULL, "LOUT amp" },
	{ "LOUTR", NULL, "LOUT amp" },

	{ "SPK amp", NULL, "SPO MIX" },
	{ "SPO", NULL, "SPK amp" },
};

static int get_clk_info(int sclk, int rate)
{
	int i, pd[] = {1, 2, 3, 4, 6, 8, 12, 16};

	if (sclk <= 0 || rate <= 0)
		return -EINVAL;

	rate = rate << 8;
	for (i = 0; i < ARRAY_SIZE(pd); i++)
		if (sclk == rate * pd[i])
			return i;

	return -EINVAL;
}

static int rt5660_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;
	struct rt5660_priv *rt5660 = snd_soc_codec_get_drvdata(codec);
	unsigned int val_len = 0, val_clk, mask_clk;
	int pre_div, bclk_ms, frame_size;

	rt5660->lrck[dai->id] = params_rate(params);
	pre_div = get_clk_info(rt5660->sysclk, rt5660->lrck[dai->id]);
	if (pre_div < 0) {
		dev_err(codec->dev, "Unsupported clock setting\n");
		return -EINVAL;
	}
	frame_size = snd_soc_params_to_frame_size(params);
	if (frame_size < 0) {
		dev_err(codec->dev, "Unsupported frame size: %d\n", frame_size);
		return -EINVAL;
	}
	bclk_ms = frame_size > 32 ? 1 : 0;
	rt5660->bclk[dai->id] = rt5660->lrck[dai->id] * (32 << bclk_ms);

	dev_dbg(dai->dev, "bclk is %dHz and lrck is %dHz\n",
		rt5660->bclk[dai->id], rt5660->lrck[dai->id]);
	dev_dbg(dai->dev, "bclk_ms is %d and pre_div is %d for iis %d\n",
				bclk_ms, pre_div, dai->id);

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		val_len |= RT5660_I2S_DL_20;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		val_len |= RT5660_I2S_DL_24;
		break;
	case SNDRV_PCM_FORMAT_S8:
		val_len |= RT5660_I2S_DL_8;
		break;
	default:
		return -EINVAL;
	}
	switch (dai->id) {
	case RT5660_AIF1:
		mask_clk = RT5660_I2S_BCLK_MS1_MASK | RT5660_I2S_PD1_MASK;
		val_clk = bclk_ms << RT5660_I2S_BCLK_MS1_SFT |
			pre_div << RT5660_I2S_PD1_SFT;
		snd_soc_update_bits(codec, RT5660_I2S1_SDP,
			RT5660_I2S_DL_MASK, val_len);
		snd_soc_update_bits(codec, RT5660_ADDA_CLK1, mask_clk, val_clk);
		break;
	default:
		dev_err(codec->dev, "Invalid dai->id: %d\n", dai->id);
		return -EINVAL;
	}

	return 0;
}

static int rt5660_prepare(struct snd_pcm_substream *substream,
				struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;
	struct rt5660_priv *rt5660 = snd_soc_codec_get_drvdata(codec);

	rt5660->aif_pu = dai->id;
	return 0;
}

static int rt5660_set_dai_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct snd_soc_codec *codec = dai->codec;
	struct rt5660_priv *rt5660 = snd_soc_codec_get_drvdata(codec);
	unsigned int reg_val = 0;

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		rt5660->master[dai->id] = 1;
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		reg_val |= RT5660_I2S_MS_S;
		rt5660->master[dai->id] = 0;
		break;
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		break;
	case SND_SOC_DAIFMT_IB_NF:
		reg_val |= RT5660_I2S_BP_INV;
		break;
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		reg_val |= RT5660_I2S_DF_LEFT;
		break;
	case SND_SOC_DAIFMT_DSP_A:
		reg_val |= RT5660_I2S_DF_PCM_A;
		break;
	case SND_SOC_DAIFMT_DSP_B:
		reg_val |= RT5660_I2S_DF_PCM_B;
		break;
	default:
		return -EINVAL;
	}
	switch (dai->id) {
	case RT5660_AIF1:
		snd_soc_update_bits(codec, RT5660_I2S1_SDP,
			RT5660_I2S_MS_MASK | RT5660_I2S_BP_MASK |
			RT5660_I2S_DF_MASK, reg_val);
		break;
	default:
		dev_err(codec->dev, "Invalid dai->id: %d\n", dai->id);
		return -EINVAL;
	}
	return 0;
}

static int rt5660_set_dai_sysclk(struct snd_soc_dai *dai,
		int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = dai->codec;
	struct rt5660_priv *rt5660 = snd_soc_codec_get_drvdata(codec);
	unsigned int reg_val = 0;

	if (freq == rt5660->sysclk && clk_id == rt5660->sysclk_src)
		return 0;

	switch (clk_id) {
	case RT5660_SCLK_S_MCLK:
		reg_val |= RT5660_SCLK_SRC_MCLK;
		break;
	case RT5660_SCLK_S_PLL1:
		reg_val |= RT5660_SCLK_SRC_PLL1;
		break;
	case RT5660_SCLK_S_RCCLK:
		reg_val |= RT5660_SCLK_SRC_RCCLK;
		break;
	default:
		dev_err(codec->dev, "Invalid clock id (%d)\n", clk_id);
		return -EINVAL;
	}
	snd_soc_update_bits(codec, RT5660_GLB_CLK,
		RT5660_SCLK_SRC_MASK, reg_val);
	rt5660->sysclk = freq;
	rt5660->sysclk_src = clk_id;

	dev_dbg(dai->dev, "Sysclk is %dHz and clock id is %d\n", freq, clk_id);

	return 0;
}

/**
 * rt5660_pll_calc - Calcualte PLL M/N/K code.
 * @freq_in: external clock provided to codec.
 * @freq_out: target clock which codec works on.
 * @pll_code: Pointer to structure with M, N, K and bypass flag.
 *
 * Calcualte M/N/K code to configure PLL for codec. And K is assigned to 2
 * which make calculation more efficiently.
 *
 * Returns 0 for success or negative error code.
 */
static int rt5660_pll_calc(const unsigned int freq_in,
	const unsigned int freq_out, struct rt5660_pll_code *pll_code)
{
	int max_n = RT5660_PLL_N_MAX, max_m = RT5660_PLL_M_MAX;
	int k, n = 0, m = 0, red, n_t, m_t, pll_out, in_t, out_t;
	int red_t = abs(freq_out - freq_in);
	bool bypass = false;

	if (RT5660_PLL_INP_MAX < freq_in || RT5660_PLL_INP_MIN > freq_in)
		return -EINVAL;

	k = 100000000 / freq_out - 2;
	if (k > RT5660_PLL_K_MAX)
		k = RT5660_PLL_K_MAX;
	for (n_t = 0; n_t <= max_n; n_t++) {
		in_t = freq_in / (k + 2);
		pll_out = freq_out / (n_t + 2);
		if (in_t < 0)
			continue;
		if (in_t == pll_out) {
			bypass = true;
			n = n_t;
			goto code_find;
		}
		red = abs(in_t - pll_out);
		if (red < red_t) {
			bypass = true;
			n = n_t;
			m = m_t;
			if (red == 0)
				goto code_find;
			red_t = red;
		}
		for (m_t = 0; m_t <= max_m; m_t++) {
			out_t = in_t / (m_t + 2);
			red = abs(out_t - pll_out);
			if (red < red_t) {
				bypass = false;
				n = n_t;
				m = m_t;
				if (red == 0)
					goto code_find;
				red_t = red;
			}
		}
	}
	pr_debug("Only get approximation about PLL\n");

code_find:

	pll_code->m_bp = bypass;
	pll_code->m_code = m;
	pll_code->n_code = n;
	pll_code->k_code = k;
	return 0;
}

static int rt5660_set_dai_pll(struct snd_soc_dai *dai, int pll_id, int source,
			unsigned int freq_in, unsigned int freq_out)
{
	struct snd_soc_codec *codec = dai->codec;
	struct rt5660_priv *rt5660 = snd_soc_codec_get_drvdata(codec);
	struct rt5660_pll_code pll_code;
	int ret;

	if (source == rt5660->pll_src && freq_in == rt5660->pll_in &&
	    freq_out == rt5660->pll_out)
		return 0;

	if (!freq_in || !freq_out) {
		dev_dbg(codec->dev, "PLL disabled\n");

		rt5660->pll_in = 0;
		rt5660->pll_out = 0;
		snd_soc_update_bits(codec, RT5660_GLB_CLK,
			RT5660_SCLK_SRC_MASK, RT5660_SCLK_SRC_MCLK);
		return 0;
	}

	switch (source) {
	case RT5660_PLL1_S_MCLK:
		snd_soc_update_bits(codec, RT5660_GLB_CLK,
			RT5660_PLL1_SRC_MASK, RT5660_PLL1_SRC_MCLK);
		break;
	case RT5660_PLL1_S_BCLK1:
		snd_soc_update_bits(codec, RT5660_GLB_CLK,
			RT5660_PLL1_SRC_MASK, RT5660_PLL1_SRC_BCLK1);
		break;
	case RT5660_PLL1_S_RCCLK:
		snd_soc_update_bits(codec, RT5660_GLB_CLK,
			RT5660_PLL1_SRC_MASK, RT5660_PLL1_SRC_RCCLK);
		break;
	default:
		dev_err(codec->dev, "Unknown PLL source %d\n", source);
		return -EINVAL;
	}

	ret = rt5660_pll_calc(freq_in, freq_out, &pll_code);
	if (ret < 0) {
		dev_err(codec->dev, "Unsupport input clock %d\n", freq_in);
		return ret;
	}

	dev_dbg(codec->dev, "bypass=%d m=%d n=%d k=%d\n",
		pll_code.m_bp, (pll_code.m_bp ? 0 : pll_code.m_code),
		pll_code.n_code, pll_code.k_code);

	snd_soc_write(codec, RT5660_PLL_CTRL1,
		pll_code.n_code << RT5660_PLL_N_SFT | pll_code.k_code);
	snd_soc_write(codec, RT5660_PLL_CTRL2,
		(pll_code.m_bp ? 0 : pll_code.m_code) << RT5660_PLL_M_SFT |
		pll_code.m_bp << RT5660_PLL_M_BP_SFT);

	rt5660->pll_in = freq_in;
	rt5660->pll_out = freq_out;
	rt5660->pll_src = source;

	return 0;
}

/**
 * rt5660_index_show - Dump private registers.
 * @dev: codec device.
 * @attr: device attribute.
 * @buf: buffer for display.
 *
 * To show non-zero values of all private registers.
 *
 * Returns buffer length.
 */
static ssize_t rt5660_index_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct rt5660_priv *rt5660 = i2c_get_clientdata(client);
	struct snd_soc_codec *codec = rt5660->codec;
	unsigned int val;
	int cnt = 0, i;

	cnt += sprintf(buf, "RT5660 index register\n");
	for (i = 0; i < 0xff; i++) {
		if (cnt + RT5660_REG_DISP_LEN >= PAGE_SIZE)
			break;
		val = rt5660_index_read(codec, i);
		if (!val)
			continue;
		cnt += snprintf(buf + cnt, RT5660_REG_DISP_LEN,
				"%02x: %04x\n", i, val);
	}

	if (cnt >= PAGE_SIZE)
		cnt = PAGE_SIZE - 1;

	return cnt;
}

static ssize_t rt5660_index_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct rt5660_priv *rt5660 = i2c_get_clientdata(client);
	struct snd_soc_codec *codec = rt5660->codec;
	unsigned int val = 0, addr = 0;
	int i;

	for (i = 0; i < count; i++) {
		if (*(buf + i) <= '9' && *(buf + i) >= '0')
			addr = (addr << 4) | (*(buf + i)-'0');
		else if (*(buf + i) <= 'f' && *(buf + i) >= 'a')
			addr = (addr << 4) | ((*(buf + i) - 'a') + 0xa);
		else if (*(buf + i) <= 'F' && *(buf + i) >= 'A')
			addr = (addr << 4) | ((*(buf + i)-'A') + 0xa);
		else
			break;
	}

	for (i = i + 1; i < count; i++) {
		if (*(buf + i) <= '9' && *(buf + i) >= '0')
			val = (val << 4) | (*(buf + i) - '0');
		else if (*(buf + i) <= 'f' && *(buf + i) >= 'a')
			val = (val << 4) | ((*(buf + i) - 'a') + 0xa);
		else if (*(buf + i) <= 'F' && *(buf + i) >= 'A')
			val = (val << 4) | ((*(buf + i) - 'A') + 0xa);
		else
			break;
	}
	pr_debug("addr=0x%x val=0x%x\n", addr, val);
	if (addr > RT5660_VENDOR_ID2 || val > 0xffff || val < 0)
		return count;

	if (i == count)
		pr_info("0x%02x = 0x%04x\n", addr,
			rt5660_index_read(codec, addr));
	else
		rt5660_index_write(codec, addr, val);


	return count;
}
static DEVICE_ATTR(index_reg, 0666, rt5660_index_show, rt5660_index_store);

static ssize_t rt5660_codec_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct rt5660_priv *rt5660 = i2c_get_clientdata(client);
	struct snd_soc_codec *codec = rt5660->codec;
	unsigned int val;
	int cnt = 0, i;

	for (i = 0; i <= RT5660_VENDOR_ID2; i++) {
		if (cnt + RT5660_REG_DISP_LEN >= PAGE_SIZE)
			break;
		val = snd_soc_read(codec, i);
		if (!val)
			continue;
		cnt += snprintf(buf + cnt, RT5660_REG_DISP_LEN,
				"#rng%02x  #rv%04x  #rd0\n", i, val);
	}

	if (cnt >= PAGE_SIZE)
		cnt = PAGE_SIZE - 1;

	return cnt;
}

static ssize_t rt5660_codec_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct rt5660_priv *rt5660 = i2c_get_clientdata(client);
	struct snd_soc_codec *codec = rt5660->codec;
	unsigned int val = 0, addr = 0;
	int i;

	pr_debug("register \"%s\" count=%d\n", buf, count);
	for (i = 0; i < count; i++) {
		if (*(buf + i) <= '9' && *(buf + i) >= '0')
			addr = (addr << 4) | (*(buf + i) - '0');
		else if (*(buf + i) <= 'f' && *(buf + i) >= 'a')
			addr = (addr << 4) | ((*(buf + i)-'a') + 0xa);
		else if (*(buf + i) <= 'F' && *(buf + i) >= 'A')
			addr = (addr << 4) | ((*(buf + i)-'A') + 0xa);
		else
			break;
	}

	for (i = i + 1; i < count; i++) {
		if (*(buf + i) <= '9' && *(buf + i) >= '0')
			val = (val << 4) | (*(buf + i)-'0');
		else if (*(buf + i) <= 'f' && *(buf + i) >= 'a')
			val = (val << 4) | ((*(buf + i)-'a') + 0xa);
		else if (*(buf + i) <= 'F' && *(buf + i) >= 'A')
			val = (val << 4) | ((*(buf + i)-'A') + 0xa);
		else
			break;
	}
	pr_debug("addr=0x%x val=0x%x\n", addr, val);
	if (addr > RT5660_VENDOR_ID2 || val > 0xffff || val < 0)
		return count;

	if (i == count)
		pr_info("0x%02x = 0x%04x\n", addr, snd_soc_read(codec, addr));
	else
		snd_soc_write(codec, addr, val);


	return count;
}

static DEVICE_ATTR(codec_reg, 0666, rt5660_codec_show, rt5660_codec_store);

static int rt5660_set_bias_level(struct snd_soc_codec *codec,
			enum snd_soc_bias_level level)
{
	switch (level) {
	case SND_SOC_BIAS_ON:
		break;

	case SND_SOC_BIAS_PREPARE:
		break;

	case SND_SOC_BIAS_STANDBY:
		if (SND_SOC_BIAS_OFF == codec->dapm.bias_level) {
			snd_soc_update_bits(codec, RT5660_PWR_ANLG1,
				RT5660_PWR_VREF1 | RT5660_PWR_MB |
				RT5660_PWR_BG | RT5660_PWR_VREF2,
				RT5660_PWR_VREF1 | RT5660_PWR_MB |
				RT5660_PWR_BG | RT5660_PWR_VREF2);
			mdelay(10);
			snd_soc_update_bits(codec, RT5660_PWR_ANLG1,
				RT5660_PWR_FV1 | RT5660_PWR_FV2,
				RT5660_PWR_FV1 | RT5660_PWR_FV2);
			codec->cache_only = false;
			codec->cache_sync = 1;
			snd_soc_cache_sync(codec);
			rt5660_index_sync(codec);
		}
		break;

	case SND_SOC_BIAS_OFF:
		snd_soc_write(codec, RT5660_PWR_DIG1, 0x0000);
		snd_soc_write(codec, RT5660_PWR_DIG2, 0x0000);
		snd_soc_write(codec, RT5660_PWR_VOL, 0x0000);
		snd_soc_write(codec, RT5660_PWR_MIXER, 0x0000);
		snd_soc_write(codec, RT5660_PWR_ANLG1, 0x0000);
		snd_soc_write(codec, RT5660_PWR_ANLG2, 0x0000);
		break;

	default:
		break;
	}
	codec->dapm.bias_level = level;

	return 0;
}

static int rt5660_probe(struct snd_soc_codec *codec)
{
	struct rt5660_priv *rt5660 = snd_soc_codec_get_drvdata(codec);
#ifdef RTK_IOCTL
#if defined(CONFIG_SND_HWDEP) || defined(CONFIG_SND_HWDEP_MODULE)
	struct rt_codec_ops *ioctl_ops = rt_codec_get_ioctl_ops();
#endif
#endif
	int ret;

	pr_info("Codec driver version %s\n", VERSION);

	ret = snd_soc_codec_set_cache_io(codec, 8, 16, SND_SOC_I2C);
	if (ret != 0) {
		dev_err(codec->dev, "Failed to set cache I/O: %d\n", ret);
		return ret;
	}

	rt5660_reset(codec);
	snd_soc_update_bits(codec, RT5660_PWR_ANLG1,
		RT5660_PWR_VREF1 | RT5660_PWR_MB |
		RT5660_PWR_BG | RT5660_PWR_VREF2,
		RT5660_PWR_VREF1 | RT5660_PWR_MB |
		RT5660_PWR_BG | RT5660_PWR_VREF2);
	mdelay(10);
	snd_soc_update_bits(codec, RT5660_PWR_ANLG1,
		RT5660_PWR_FV1 | RT5660_PWR_FV2,
		RT5660_PWR_FV1 | RT5660_PWR_FV2);

	rt5660_reg_init(codec);

	codec->dapm.bias_level = SND_SOC_BIAS_OFF;
	rt5660->codec = codec;

	snd_soc_add_codec_controls(codec, rt5660_snd_controls,
			ARRAY_SIZE(rt5660_snd_controls));
	snd_soc_dapm_new_controls(&codec->dapm, rt5660_dapm_widgets,
			ARRAY_SIZE(rt5660_dapm_widgets));
	snd_soc_dapm_add_routes(&codec->dapm, rt5660_dapm_routes,
			ARRAY_SIZE(rt5660_dapm_routes));

#ifdef RTK_IOCTL
#if defined(CONFIG_SND_HWDEP) || defined(CONFIG_SND_HWDEP_MODULE)
	ioctl_ops->index_write = rt5660_index_write;
	ioctl_ops->index_read = rt5660_index_read;
	ioctl_ops->index_update_bits = rt5660_index_update_bits;
	ioctl_ops->ioctl_common = rt5660_ioctl_common;
	realtek_ce_init_hwdep(codec);
#endif
#endif

	ret = device_create_file(codec->dev, &dev_attr_index_reg);
	if (ret != 0) {
		dev_err(codec->dev,
			"Failed to create index_reg sysfs files: %d\n", ret);
		return ret;
	}

	ret = device_create_file(codec->dev, &dev_attr_codec_reg);
	if (ret != 0) {
		dev_err(codec->dev,
			"Failed to create codex_reg sysfs files: %d\n", ret);
		return ret;
	}
	mdelay(200);

	return 0;
}

static int rt5660_remove(struct snd_soc_codec *codec)
{
	rt5660_set_bias_level(codec, SND_SOC_BIAS_OFF);
	return 0;
}

#ifdef CONFIG_PM
static int rt5660_suspend(struct snd_soc_codec *codec)
{
	rt5660_set_bias_level(codec, SND_SOC_BIAS_OFF);
	return 0;
}

static int rt5660_resume(struct snd_soc_codec *codec)
{
	rt5660_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
	return 0;
}
#else
#define rt5660_suspend NULL
#define rt5660_resume NULL
#endif

#define RT5660_STEREO_RATES SNDRV_PCM_RATE_8000_96000
#define RT5660_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE | \
			SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S8)

struct snd_soc_dai_ops rt5660_aif_dai_ops = {
	.hw_params = rt5660_hw_params,
	.prepare = rt5660_prepare,
	.set_fmt = rt5660_set_dai_fmt,
	.set_sysclk = rt5660_set_dai_sysclk,
	.set_pll = rt5660_set_dai_pll,
};

struct snd_soc_dai_driver rt5660_dai[] = {
	{
		.name = "rt5660-aif1",
		.id = RT5660_AIF1,
		.playback = {
			.stream_name = "AIF1 Playback",
			.channels_min = 1,
			.channels_max = 2,
			.rates = RT5660_STEREO_RATES,
			.formats = RT5660_FORMATS,
		},
		.capture = {
			.stream_name = "AIF1 Capture",
			.channels_min = 1,
			.channels_max = 2,
			.rates = RT5660_STEREO_RATES,
			.formats = RT5660_FORMATS,
		},
		.ops = &rt5660_aif_dai_ops,
	},
};

static struct snd_soc_codec_driver soc_codec_dev_rt5660 = {
	.probe = rt5660_probe,
	.remove = rt5660_remove,
	.suspend = rt5660_suspend,
	.resume = rt5660_resume,
	.idle_bias_off = true,
	.set_bias_level = rt5660_set_bias_level,
	.reg_cache_size = RT5660_VENDOR_ID2 + 1,
	.reg_word_size = sizeof(u16),
	.reg_cache_default = rt5660_reg,
	.volatile_register = rt5660_volatile_register,
	.readable_register = rt5660_readable_register,
	.reg_cache_step = 1,
};

static const struct i2c_device_id rt5660_i2c_id[] = {
	{ "rt5660", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, rt5660_i2c_id);

#ifdef CONFIG_OF
static int rt5660_parse_dt_property(struct device *dev,
				  struct rt5660_priv *rt5660)
{
	struct device_node *node = dev->of_node;
	int ret;
	int delay_time;
	enum of_gpio_flags flags;

	if (!node)
		return -ENODEV;

	rt5660->codec_spkvdd_gpio = of_get_named_gpio_flags(node, "codec-spkvdd-gpio", 0, &flags);
	if (gpio_is_valid(rt5660->codec_spkvdd_gpio)) {
		ret = devm_gpio_request(dev, rt5660->codec_spkvdd_gpio, "codec-spkvdd-gpio");
		if(ret){
			dev_err(dev,"codec_spkvdd_gpio request ERROR:%d\n",ret);
			return ret;
		}
		rt5660->codec_spkvdd_gpio_active = (flags == GPIO_ACTIVE_HIGH)? 1:0;
		ret = gpio_direction_output(rt5660->codec_spkvdd_gpio, rt5660->codec_spkvdd_gpio_active);
		if(ret){
			dev_err(dev,"codec_spkvdd_gpio set ERROR:%d\n",ret);
			return ret;
		}
		mdelay(5);
	} else {
		dev_err(dev,"Can not read property codec-spkvdd-gpio\n");
	}

	rt5660->codec_avdd_gpio = of_get_named_gpio_flags(node, "codec-avdd-gpio", 0, &flags);
	if (gpio_is_valid(rt5660->codec_avdd_gpio)) {
		ret = devm_gpio_request(dev, rt5660->codec_avdd_gpio, "codec-avdd-gpio");
		if(ret){
			dev_err(dev,"codec_avdd_gpio request ERROR:%d\n",ret);
			return ret;
		}
		rt5660->codec_avdd_gpio_active = (flags == GPIO_ACTIVE_HIGH)? 1:0;
		ret = gpio_direction_output(rt5660->codec_avdd_gpio, rt5660->codec_avdd_gpio_active);
		if(ret){
			dev_err(dev,"codec_avdd_gpio set ERROR:%d\n",ret);
			return ret;
		}

		if(of_property_read_u32(node,"codec-avdd-delay",&delay_time))
		{
			dev_err(dev,"codec-avdd-delay time get error,use default delay time\n");
			delay_time = 5;
		}
		dev_dbg(dev,"codec-avdd-delay delay time=%d\n",delay_time);
		mdelay(delay_time);
	} else {
		dev_err(dev,"Can not read property codec-avdd-gpio\n");
	}

	rt5660->codec_micvdd_gpio = of_get_named_gpio_flags(node, "codec-micvdd-gpio", 0, &flags);
	if (gpio_is_valid(rt5660->codec_micvdd_gpio)) {
		ret = devm_gpio_request(dev, rt5660->codec_micvdd_gpio, "codec-micvdd-gpio");
		if(ret){
			dev_err(dev,"codec_micvdd_gpio request ERROR:%d\n",ret);
			return ret;
		}
		rt5660->codec_micvdd_gpio_active = (flags == GPIO_ACTIVE_HIGH)? 1:0;
		ret = gpio_direction_output(rt5660->codec_micvdd_gpio, rt5660->codec_micvdd_gpio_active);
		if(ret){
			dev_err(dev,"codec_micvdd_gpio set ERROR:%d\n",ret);
			return ret;
		}
	} else {
		dev_err(dev,"Can not read property codec-micvdd-gpio\n");
	}

	rt5660->amp_shutdown_gpio = of_get_named_gpio_flags(node, "amp-shutdown-gpio", 0, &flags);
	if (gpio_is_valid(rt5660->amp_shutdown_gpio)) {
		ret = devm_gpio_request(dev, rt5660->amp_shutdown_gpio, "amp-shutdown-gpio");
		if(ret){
			dev_err(dev,"amp_shutdown_gpio request ERROR:%d\n",ret);
			return ret;
		}
		rt5660->amp_shutdown_gpio_active = (flags == GPIO_ACTIVE_HIGH)? 1:0;
		//always enable
		ret = gpio_direction_output(rt5660->amp_shutdown_gpio, rt5660->amp_shutdown_gpio_active);
		if(ret){
			dev_err(dev,"amp_shutdown_gpio set ERROR:%d\n",ret);
			return ret;
		}
	} else {
		dev_err(dev,"Can not read property amp-shutdown-gpio\n");
	}

	rt5660->amp_mute_gpio = of_get_named_gpio_flags(node, "amp-mute-gpio", 0, &flags);
	if (gpio_is_valid(rt5660->amp_mute_gpio)) {
		ret = devm_gpio_request(dev, rt5660->amp_mute_gpio, "amp-mute-gpio");
		if(ret){
			dev_err(dev,"amp_mute_gpio request ERROR:%d\n",ret);
			return ret;
		}
		rt5660->amp_mute_gpio_active = (flags == GPIO_ACTIVE_HIGH)? 1:0;
		ret = gpio_direction_output(rt5660->amp_mute_gpio, !rt5660->amp_mute_gpio_active);
		if(ret){
			dev_err(dev,"amp_mute_gpio set ERROR:%d\n",ret);
			return ret;
		}
	} else {
		dev_err(dev,"Can not read property amp-mute-gpio\n");
	}

	return 0;
}
#else
static int rt5660_parse_dt_property(struct device *dev,
				  struct rt5660_priv *rt5660)
{
	return -ENOSYS;
}
#endif

static int rt5660_i2c_probe(struct i2c_client *i2c,
		    const struct i2c_device_id *id)
{
	struct rt5660_priv *rt5660;
	int ret;

	rt5660 = kzalloc(sizeof(struct rt5660_priv), GFP_KERNEL);
	if (NULL == rt5660)
		return -ENOMEM;

	ret = rt5660_parse_dt_property(&i2c->dev, rt5660);
	if (ret < 0) {
		dev_err(&i2c->dev,"parse device tree property error: %d\n", ret);
		return ret;
	}

	i2c_set_clientdata(i2c, rt5660);

	ret = snd_soc_register_codec(&i2c->dev, &soc_codec_dev_rt5660,
			rt5660_dai, ARRAY_SIZE(rt5660_dai));
	if (ret < 0)
		kfree(rt5660);

	return ret;
}

static int rt5660_i2c_remove(struct i2c_client *i2c)
{
	snd_soc_unregister_codec(&i2c->dev);
	kfree(i2c_get_clientdata(i2c));
	return 0;
}

void rt5660_i2c_shutdown(struct i2c_client *client)
{
	struct rt5660_priv *rt5660 = i2c_get_clientdata(client);
	struct snd_soc_codec *codec = rt5660->codec;

	//disable i2s mclk & bclk
	writel_relaxed(0x90009, RK_CRU_VIRT+RK3288_CRU_CLKGATES_CON(4));
	//To meet poweroff sequence
	mdelay(200);
	if (codec != NULL)
		rt5660_set_bias_level(codec, SND_SOC_BIAS_OFF);

	if(gpio_is_valid(rt5660->codec_micvdd_gpio))
		gpio_direction_output(rt5660->codec_micvdd_gpio, !rt5660->codec_micvdd_gpio_active);
	mdelay(40);
	if(gpio_is_valid(rt5660->codec_avdd_gpio))
		gpio_direction_output(rt5660->codec_avdd_gpio, !rt5660->codec_avdd_gpio_active);
	mdelay(5);
	if(gpio_is_valid(rt5660->codec_spkvdd_gpio))
		gpio_direction_output(rt5660->codec_spkvdd_gpio, !rt5660->codec_spkvdd_gpio_active);
}

struct i2c_driver rt5660_i2c_driver = {
	.driver = {
		.name = "rt5660",
		.owner = THIS_MODULE,
	},
	.probe = rt5660_i2c_probe,
	.remove   = rt5660_i2c_remove,
	.shutdown = rt5660_i2c_shutdown,
	.id_table = rt5660_i2c_id,
};

static int __init rt5660_modinit(void)
{
	return i2c_add_driver(&rt5660_i2c_driver);
}
module_init(rt5660_modinit);

static void __exit rt5660_modexit(void)
{
	i2c_del_driver(&rt5660_i2c_driver);
}
module_exit(rt5660_modexit);

MODULE_DESCRIPTION("ASoC RT5660 driver");
MODULE_AUTHOR("Bard Liao <bardliao@realtek.com>");
MODULE_LICENSE("GPL");
