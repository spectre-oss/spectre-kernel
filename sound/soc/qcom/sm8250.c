// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2020, Linaro Limited

#include <dt-bindings/sound/qcom,q6afe.h>
#include <linux/clk.h>
#include <linux/module.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/string.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <linux/soundwire/sdw.h>
#include <sound/jack.h>
#include <linux/input-event-codes.h>
#include "qdsp6/q6afe.h"
#include "common.h"
#include "usb_offload_utils.h"
#include "sdw.h"

#define MI2S_BCLK_RATE		1536000
/* AOSP sunfish: 8 slots × 32-bit × 48 kHz on tert TDM (TLMM 53–56). */
#define TDM_BCLK_RATE		12288000
#define TDM_SLOTS		8
#define TDM_SLOT_WIDTH		32
/* sound/cs35l41.h CS35L41_CLKID_SCLK — avoid pulling irqreturn_t into this file */
#define CS35L41_CLKID_SCLK	0

struct sm8250_snd_data {
	bool stream_prepared[AFE_PORT_MAX];
	struct snd_soc_card *card;
	struct snd_soc_jack jack;
	struct snd_soc_jack usb_offload_jack;
	bool usb_offload_jack_setup;
	struct snd_soc_jack dp_jack;
	bool jack_setup;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_sleep;
	struct pinctrl_state *pins_active;
	struct clk *lpaif_sec;
	struct clk *lpaif_ter;
};

static int sm8250_snd_init(struct snd_soc_pcm_runtime *rtd)
{
	struct sm8250_snd_data *data = snd_soc_card_get_drvdata(rtd->card);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);

	switch (cpu_dai->id) {
	case DISPLAY_PORT_RX:
		return qcom_snd_dp_jack_setup(rtd, &data->dp_jack, 0);
	case USB_RX:
		return qcom_snd_usb_offload_jack_setup(rtd, &data->usb_offload_jack,
						       &data->usb_offload_jack_setup);
	case RX_CODEC_DMA_RX_0:
		/*
		 * The RX macro digital gain goes up to +40 dB. Userspace that
		 * maps full volume to the top of the range (PipeWire, PulseAudio)
		 * would clip there, so stop the interpolator gains at 0 dB.
		 */
		snd_soc_limit_volume(rtd->card, "RX_RX0 Digital Volume", 84);
		snd_soc_limit_volume(rtd->card, "RX_RX1 Digital Volume", 84);
		snd_soc_limit_volume(rtd->card, "RX_RX2 Digital Volume", 84);
		fallthrough;
	default:
		return qcom_snd_wcd_jack_setup(rtd, &data->jack, &data->jack_setup);
	}
}

static void sm8250_snd_exit(struct snd_soc_pcm_runtime *rtd)
{
	struct sm8250_snd_data *data = snd_soc_card_get_drvdata(rtd->card);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);

	if (cpu_dai->id == USB_RX)
		qcom_snd_usb_offload_jack_remove(rtd,
						 &data->usb_offload_jack_setup);

}

static int sm8250_be_hw_params_fixup(struct snd_soc_pcm_runtime *rtd,
				     struct snd_pcm_hw_params *params)
{
	struct snd_interval *rate = hw_param_interval(params,
					SNDRV_PCM_HW_PARAM_RATE);
	struct snd_interval *channels = hw_param_interval(params,
					SNDRV_PCM_HW_PARAM_CHANNELS);
	struct snd_mask *fmt = hw_param_mask(params, SNDRV_PCM_HW_PARAM_FORMAT);

	rate->min = rate->max = 48000;
	/* 1ch AFE map 0/65535 + ADM 0x10327. 2ch map 0/4 produced a WAV. */
	channels->min = channels->max = 2;
	snd_mask_none(fmt);
	/*
	 * AOSP SEC TDM is S32_LE. CS35 DAI dump still S16/S24, so keep
	 * those too; q6tdm_hw_params promotes bit_width up to slot 32.
	 */
	if (snd_soc_rtd_to_cpu(rtd, 0)->id == SECONDARY_TDM_RX_0 ||
	    snd_soc_rtd_to_cpu(rtd, 0)->id == TERTIARY_TDM_TX_0) {
		snd_mask_set_format(fmt, SNDRV_PCM_FORMAT_S32_LE);
		snd_mask_set_format(fmt, SNDRV_PCM_FORMAT_S24_LE);
		snd_mask_set_format(fmt, SNDRV_PCM_FORMAT_S16_LE);
	} else
		snd_mask_set_format(fmt, SNDRV_PCM_FORMAT_S16_LE);

	return 0;
}

static int sm8250_tdm_snd_hw_params(struct snd_pcm_substream *substream,
				    struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	struct snd_soc_dai *codec_dai;
	/* Live 0/1 made S32 hw_params EINVAL. 4.14 AFE map is byte offsets. */
	unsigned int afe_slots[2] = {0, 4};
	/*
	 * S32 hw_params still fails. S16 2ch lands in slot 0; R on slot 1
	 * is silent. Both CS35 RX slot 0. Mute L Digital for bottom-only.
	 */
	unsigned int codec_slots[2] = {0, 0};
	unsigned int slot;
	int i, ret;

	ret = snd_soc_dai_set_tdm_slot(cpu_dai, 0, 0x3, TDM_SLOTS,
				       TDM_SLOT_WIDTH);
	if (ret < 0) {
		dev_err(rtd->dev, "TDM CPU slots failed: %d\n", ret);
		return ret;
	}

	ret = snd_soc_dai_set_channel_map(cpu_dai, 0, NULL,
					  params_channels(params), afe_slots);
	if (ret < 0) {
		dev_err(rtd->dev, "TDM CPU map failed: %d\n", ret);
		return ret;
	}

	for_each_rtd_codec_dais(rtd, i, codec_dai) {
		slot = 0;
		if (codec_dai->component->name_prefix &&
		    !strcmp(codec_dai->component->name_prefix, "R"))
			slot = 1;

		ret = snd_soc_dai_set_sysclk(codec_dai, CS35L41_CLKID_SCLK,
					     TDM_BCLK_RATE, SND_SOC_CLOCK_IN);
		if (ret < 0) {
			dev_err(codec_dai->dev, "CS35 DAI sysclk: %d\n", ret);
			return ret;
		}

		ret = snd_soc_component_set_sysclk(codec_dai->component,
						   CS35L41_CLKID_SCLK, 0,
						   TDM_BCLK_RATE,
						   SND_SOC_CLOCK_IN);
		if (ret < 0) {
			dev_err(codec_dai->dev, "CS35 PLL sysclk: %d\n", ret);
			return ret;
		}

		ret = snd_soc_dai_set_channel_map(codec_dai, 0, NULL, 1,
						  &codec_slots[slot]);
		if (ret < 0) {
			dev_err(codec_dai->dev, "CS35 RX slot: %d\n", ret);
			return ret;
		}
	}

	return 0;
}

static int sm8250_tdm_tx_hw_params(struct snd_pcm_substream *substream,
				   struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	struct snd_soc_dai *codec_dai;
	unsigned int afe_slots[2] = {8, 0};
	unsigned int nch = params_channels(params);
	int i, ret;

	if (nch == 1)
		afe_slots[0] = 8;

	ret = snd_soc_dai_set_tdm_slot(cpu_dai, 0x5, 0, TDM_SLOTS,
				       TDM_SLOT_WIDTH);
	if (ret < 0) {
		dev_err(rtd->dev, "TDM TX CPU slots: %d\n", ret);
		return ret;
	}

	ret = snd_soc_dai_set_channel_map(cpu_dai, nch, afe_slots, 0, NULL);
	if (ret < 0) {
		dev_err(rtd->dev, "TDM TX CPU map: %d\n", ret);
		return ret;
	}

	for_each_rtd_codec_dais(rtd, i, codec_dai) {
		ret = snd_soc_dai_set_tdm_slot(codec_dai, 0x3, 0, TDM_SLOTS,
					       TDM_SLOT_WIDTH);
		if (ret < 0)
			dev_err(codec_dai->dev, "rt5514 TDM slots: %d\n", ret);
		/* No MCLK on sunfish. PLL from TDM BCLK. clk_id 1 = PLL1, src 1 = BCLK. */
		ret = snd_soc_dai_set_pll(codec_dai, 0, 1, TDM_BCLK_RATE,
					  TDM_BCLK_RATE);
		if (ret < 0)
			dev_err(codec_dai->dev, "rt5514 pll BCLK: %d\n", ret);
		ret = snd_soc_dai_set_sysclk(codec_dai, 1, TDM_BCLK_RATE,
					     SND_SOC_CLOCK_IN);
		if (ret < 0)
			dev_err(codec_dai->dev, "rt5514 sysclk PLL: %d\n", ret);
	}

	return 0;
}

static int sm8250_snd_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);

	switch (cpu_dai->id) {
	case SECONDARY_TDM_RX_0:
		return sm8250_tdm_snd_hw_params(substream, params);
	case TERTIARY_TDM_TX_0:
		return sm8250_tdm_tx_hw_params(substream, params);
	case TERTIARY_MI2S_RX:
	case SECONDARY_MI2S_RX: {
		struct snd_soc_dai *codec_dai;
		unsigned int rx_slots[2] = {0, 1};
		unsigned int slot;
		int i, ret;

		for_each_rtd_codec_dais(rtd, i, codec_dai) {
			slot = 0;
			if (codec_dai->component->name_prefix &&
			    !strcmp(codec_dai->component->name_prefix, "R"))
				slot = 1;
			ret = snd_soc_dai_set_channel_map(codec_dai, 0, NULL, 1,
							  &rx_slots[slot]);
			if (ret < 0)
				return ret;
		}
		return 0;
	}
	default:
		return 0;
	}
}


/*
 * The LPAIF IBIT clocks come from q6afe-clocks, which is torn down and
 * re-created on every ADSP restart. A pointer cached at card probe dangles
 * after the first recycle (Oops in clk_core_set_rate_nolock from arecord),
 * so look the clock up per stream and drop it at shutdown.
 */
static struct clk *sm8250_ibit_get(struct snd_soc_pcm_runtime *rtd,
				   const char *name)
{
	struct clk *clk = clk_get(rtd->card->dev, name);

	if (IS_ERR(clk)) {
		if (PTR_ERR(clk) != -ENOENT)
			dev_warn(rtd->dev, "%s: %ld\n", name, PTR_ERR(clk));
		return NULL;
	}
	return clk;
}

static void sm8250_ibit_put(struct clk **clkp)
{
	if (*clkp) {
		clk_disable_unprepare(*clkp);
		clk_put(*clkp);
		*clkp = NULL;
	}
}

static int sm8250_snd_startup(struct snd_pcm_substream *substream)
{
	unsigned int fmt = SND_SOC_DAIFMT_BP_FP;
	unsigned int codec_dai_fmt = SND_SOC_DAIFMT_BC_FC;
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	struct snd_soc_dai *codec_dai;
	int i, ret;

	switch (cpu_dai->id) {
	case PRIMARY_MI2S_RX:
		codec_dai_fmt |= SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_I2S;
		snd_soc_dai_set_sysclk(cpu_dai,
			Q6AFE_LPASS_CLK_ID_PRI_MI2S_IBIT,
			MI2S_BCLK_RATE, SNDRV_PCM_STREAM_PLAYBACK);
		snd_soc_dai_set_fmt(cpu_dai, fmt);
		snd_soc_dai_set_fmt(snd_soc_rtd_to_codec(rtd, 0), codec_dai_fmt);
		break;
	case SECONDARY_MI2S_RX:
		/* SoC is ASP clock master. CS35 is slave (no MCLK on the amp). */
		codec_dai_fmt |= SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_I2S;
		snd_soc_dai_set_fmt(cpu_dai, fmt);
		snd_soc_dai_set_sysclk(cpu_dai, LPAIF_BIT_CLK,
				       MI2S_BCLK_RATE, SNDRV_PCM_STREAM_PLAYBACK);
		snd_soc_dai_set_sysclk(cpu_dai,
			Q6AFE_LPASS_CLK_ID_SEC_MI2S_IBIT,
			MI2S_BCLK_RATE, SNDRV_PCM_STREAM_PLAYBACK);
		for_each_rtd_codec_dais(rtd, i, codec_dai) {
			ret = snd_soc_dai_set_fmt(codec_dai, codec_dai_fmt);
			if (ret < 0)
				return ret;
		}
		break;
	case TERTIARY_MI2S_RX:
		codec_dai_fmt |= SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_I2S;
		snd_soc_dai_set_sysclk(cpu_dai,
			Q6AFE_LPASS_CLK_ID_TER_MI2S_IBIT,
			MI2S_BCLK_RATE, SNDRV_PCM_STREAM_PLAYBACK);
		snd_soc_dai_set_fmt(cpu_dai, fmt);
		for_each_rtd_codec_dais(rtd, i, codec_dai) {
			ret = snd_soc_dai_set_fmt(codec_dai, codec_dai_fmt);
			if (ret < 0)
				return ret;
			ret = snd_soc_dai_set_sysclk(codec_dai, CS35L41_CLKID_SCLK,
						     MI2S_BCLK_RATE, SND_SOC_CLOCK_IN);
			if (ret < 0)
				return ret;
			ret = snd_soc_component_set_sysclk(codec_dai->component,
							   CS35L41_CLKID_SCLK, 0,
							   MI2S_BCLK_RATE,
							   SND_SOC_CLOCK_IN);
			if (ret < 0)
				return ret;
		}
		break;
	case QUINARY_MI2S_RX:
		codec_dai_fmt |= SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_I2S;
		snd_soc_dai_set_sysclk(cpu_dai,
			Q6AFE_LPASS_CLK_ID_QUI_MI2S_IBIT,
			MI2S_BCLK_RATE, SNDRV_PCM_STREAM_PLAYBACK);
		snd_soc_dai_set_fmt(cpu_dai, fmt);
		snd_soc_dai_set_fmt(snd_soc_rtd_to_codec(rtd, 0), codec_dai_fmt);
		break;
	case SECONDARY_TDM_RX_0:
		codec_dai_fmt |= SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_DSP_A;
		/* SoC drives SCK (BP_FP). Amp has no MCLK — do not make it master. */
		snd_soc_dai_set_fmt(cpu_dai, fmt);
		/*
		 * AOSP cdc-pinctrl is active before afe_set_lpass_clock.
		 * IBIT while gpio8 is still sleep (input) left SCK DC after OE.
		 */
		{
			struct sm8250_snd_data *pdata =
				snd_soc_card_get_drvdata(rtd->card);

			if (pdata->pins_active) {
				ret = pinctrl_select_state(pdata->pinctrl,
							   pdata->pins_active);
				if (ret)
					dev_err(rtd->dev,
						"LPI active before IBIT: %d\n",
						ret);
				else
					dev_info(rtd->dev,
						 "LPI pins active before IBIT\n");
			}
			pdata->lpaif_sec = sm8250_ibit_get(rtd, "lpaif-sec-ibit");
			if (pdata->lpaif_sec) {
				ret = clk_set_rate(pdata->lpaif_sec,
						   TDM_BCLK_RATE);
				if (ret)
					dev_err(rtd->dev,
						"lpaif-sec-ibit rate %u: %d\n",
						TDM_BCLK_RATE, ret);
				ret = clk_prepare_enable(pdata->lpaif_sec);
				if (ret)
					dev_err(rtd->dev, "lpaif-sec-ibit: %d\n", ret);
				else
					dev_info(rtd->dev,
						 "lpaif-sec-ibit on @ %lu\n",
						 clk_get_rate(pdata->lpaif_sec));
			}
		}
		ret = snd_soc_dai_set_sysclk(cpu_dai,
			Q6AFE_LPASS_CLK_ID_SEC_TDM_IBIT,
			TDM_BCLK_RATE, SNDRV_PCM_STREAM_PLAYBACK);
		if (ret < 0)
			dev_err(rtd->dev, "SEC TDM IBIT clk: %d (continue)\n",
				ret);
		else
			dev_info(rtd->dev, "SEC TDM IBIT 0x202 only @ %u\n",
				 TDM_BCLK_RATE);
		for_each_rtd_codec_dais(rtd, i, codec_dai) {
			ret = snd_soc_dai_set_fmt(codec_dai, codec_dai_fmt);
			if (ret < 0) {
				dev_err(rtd->dev, "CS35 TDM fmt: %d\n", ret);
				return ret;
			}
		}
		break;
	case TERTIARY_TDM_TX_0:
		/* AFE qcom,tdm-data-delay = 1 pairs with DSP_A. */
		codec_dai_fmt |= SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_DSP_A;
		snd_soc_dai_set_fmt(cpu_dai, fmt);
		{
			struct sm8250_snd_data *pdata =
				snd_soc_card_get_drvdata(rtd->card);

			if (pdata->pins_active) {
				ret = pinctrl_select_state(pdata->pinctrl,
							   pdata->pins_active);
				if (ret)
					dev_err(rtd->dev,
						"TER TLMM active before IBIT: %d\n",
						ret);
				else
					dev_info(rtd->dev,
						 "TER TLMM pins active before IBIT\n");
			}
			pdata->lpaif_ter = sm8250_ibit_get(rtd, "lpaif-ter-ibit");
			if (pdata->lpaif_ter) {
				ret = clk_set_rate(pdata->lpaif_ter,
						   TDM_BCLK_RATE);
				if (ret)
					dev_err(rtd->dev,
						"lpaif-ter-ibit rate: %d\n",
						ret);
				ret = clk_prepare_enable(pdata->lpaif_ter);
				if (ret)
					dev_err(rtd->dev,
						"lpaif-ter-ibit: %d\n", ret);
				else
					dev_info(rtd->dev,
						 "lpaif-ter-ibit on @ %lu\n",
						 clk_get_rate(pdata->lpaif_ter));
			}
		}
		ret = snd_soc_dai_set_sysclk(cpu_dai,
			Q6AFE_LPASS_CLK_ID_TER_TDM_IBIT,
			TDM_BCLK_RATE, SNDRV_PCM_STREAM_CAPTURE);
		if (ret < 0)
			dev_err(rtd->dev, "TER TDM IBIT: %d (continue)\n", ret);
		for_each_rtd_codec_dais(rtd, i, codec_dai) {
			ret = snd_soc_dai_set_fmt(codec_dai, codec_dai_fmt);
			if (ret < 0)
				dev_err(rtd->dev, "rt5514 fmt: %d\n", ret);
		}
		break;
	case SENARY_MI2S_RX:
		codec_dai_fmt |= SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_I2S;
		snd_soc_dai_set_sysclk(cpu_dai,
			Q6AFE_LPASS_CLK_ID_SEN_MI2S_IBIT,
			MI2S_BCLK_RATE, SNDRV_PCM_STREAM_PLAYBACK);
		snd_soc_dai_set_fmt(cpu_dai, fmt);
		snd_soc_dai_set_fmt(codec_dai, codec_dai_fmt);
		break;
	case LPI_MI2S_RX_0:
		codec_dai_fmt |= SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_I2S;
		snd_soc_dai_set_sysclk(cpu_dai,
			Q6AFE_LPASS_CLK_ID_INT0_MI2S_IBIT,
			MI2S_BCLK_RATE, SNDRV_PCM_STREAM_PLAYBACK);
		snd_soc_dai_set_fmt(cpu_dai, fmt);
		snd_soc_dai_set_fmt(codec_dai, codec_dai_fmt);
		break;
	case LPI_MI2S_TX_3:
		codec_dai_fmt |= SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_I2S;
		snd_soc_dai_set_sysclk(cpu_dai,
			Q6AFE_LPASS_CLK_ID_INT3_MI2S_IBIT,
			MI2S_BCLK_RATE, SNDRV_PCM_STREAM_CAPTURE);
		snd_soc_dai_set_fmt(cpu_dai, fmt);
		snd_soc_dai_set_fmt(codec_dai, codec_dai_fmt);
		break;
	default:
		break;
	}

	/* SEC TDM already selected active before IBIT. MI2S still after. */
	if (cpu_dai->id == SECONDARY_MI2S_RX) {
		struct sm8250_snd_data *pdata =
			snd_soc_card_get_drvdata(rtd->card);

		pdata->lpaif_sec = sm8250_ibit_get(rtd, "lpaif-sec-ibit");
		if (pdata->lpaif_sec) {
			ret = clk_prepare_enable(pdata->lpaif_sec);
			if (ret)
				dev_err(rtd->dev, "LPAIF SEC IBIT: %d\n", ret);
			else
				dev_info(rtd->dev, "LPAIF SEC IBIT on\n");
		}
		if (pdata->pins_active) {
			ret = pinctrl_select_state(pdata->pinctrl,
						   pdata->pins_active);
			if (ret)
				dev_err(rtd->dev, "LPI active pins: %d\n", ret);
			else
				dev_info(rtd->dev, "LPI pins active after IBIT\n");
		}
	}

	return qcom_snd_sdw_startup(substream);
}

static void sm8250_snd_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct sm8250_snd_data *pdata = snd_soc_card_get_drvdata(rtd->card);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);

	if (cpu_dai->id == SECONDARY_MI2S_RX ||
	    cpu_dai->id == SECONDARY_TDM_RX_0) {
		if (pdata->pins_sleep)
			pinctrl_select_state(pdata->pinctrl, pdata->pins_sleep);
		sm8250_ibit_put(&pdata->lpaif_sec);
	}
	if (cpu_dai->id == TERTIARY_TDM_TX_0) {
		if (pdata->pins_sleep)
			pinctrl_select_state(pdata->pinctrl, pdata->pins_sleep);
		sm8250_ibit_put(&pdata->lpaif_ter);
	}
	qcom_snd_sdw_shutdown(substream);
}

static int sm8250_snd_prepare(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);
	struct sm8250_snd_data *data = snd_soc_card_get_drvdata(rtd->card);

	return qcom_snd_sdw_prepare(substream, &data->stream_prepared[cpu_dai->id]);
}

static int sm8250_snd_hw_free(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct sm8250_snd_data *data = snd_soc_card_get_drvdata(rtd->card);
	struct snd_soc_dai *cpu_dai = snd_soc_rtd_to_cpu(rtd, 0);

	return qcom_snd_sdw_hw_free(substream, &data->stream_prepared[cpu_dai->id]);
}

static const struct snd_soc_ops sm8250_be_ops = {
	.startup = sm8250_snd_startup,
	.shutdown = sm8250_snd_shutdown,
	.hw_params = sm8250_snd_hw_params,
	.hw_free = sm8250_snd_hw_free,
	.prepare = sm8250_snd_prepare,
};

static void sm8250_add_be_ops(struct snd_soc_card *card)
{
	struct snd_soc_dai_link *link;
	int i;

	for_each_card_prelinks(card, i, link) {
		if (link->no_pcm == 1) {
			link->init = sm8250_snd_init;
			link->exit = sm8250_snd_exit;
			link->be_hw_params_fixup = sm8250_be_hw_params_fixup;
			link->ops = &sm8250_be_ops;
		}
	}
}

static int sm8250_platform_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card;
	struct sm8250_snd_data *data;
	struct device *dev = &pdev->dev;
	int ret;

	card = devm_kzalloc_obj(dev, *card);
	if (!card)
		return -ENOMEM;

	card->owner = THIS_MODULE;
	/* Allocate the private data */
	data = devm_kzalloc_obj(dev, *data);
	if (!data)
		return -ENOMEM;

	card->dev = dev;
	dev_set_drvdata(dev, card);
	snd_soc_card_set_drvdata(card, data);

	/* lpaif-{sec,ter}-ibit are looked up per stream (sm8250_ibit_get). */

	data->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(data->pinctrl)) {
		if (PTR_ERR(data->pinctrl) != -ENODEV)
			return PTR_ERR(data->pinctrl);
		data->pinctrl = NULL;
	} else {
		data->pins_sleep = pinctrl_lookup_state(data->pinctrl, "sleep");
		data->pins_active = pinctrl_lookup_state(data->pinctrl, "active");
		if (IS_ERR(data->pins_sleep) || IS_ERR(data->pins_active)) {
			data->pins_sleep = NULL;
			data->pins_active = NULL;
		} else {
			pinctrl_select_state(data->pinctrl, data->pins_sleep);
		}
	}

	ret = qcom_snd_parse_of(card);
	if (ret)
		return ret;

	card->driver_name = of_device_get_match_data(dev);
	sm8250_add_be_ops(card);
	return devm_snd_soc_register_card(dev, card);
}

static const struct of_device_id snd_sm8250_dt_match[] = {
	{ .compatible = "fairphone,fp4-sndcard", .data = "sm7225" },
	{ .compatible = "fairphone,fp5-sndcard", .data = "qcm6490" },
	{ .compatible = "motorola,dubai-sndcard", .data = "sm7325" },
	{ .compatible = "qcom,qrb2210-sndcard", .data = "qcm2290" },
	{ .compatible = "qcom,qrb4210-rb2-sndcard", .data = "sm4250" },
	{ .compatible = "qcom,qrb5165-rb5-sndcard", .data = "sm8250" },
	{ .compatible = "qcom,sdm660-sndcard", .data = "sdm660" },
	{ .compatible = "qcom,sm8250-sndcard", .data = "sm8250" },
	{}
};

MODULE_DEVICE_TABLE(of, snd_sm8250_dt_match);

static struct platform_driver snd_sm8250_driver = {
	.probe  = sm8250_platform_probe,
	.driver = {
		.name = "snd-sm8250",
		.of_match_table = snd_sm8250_dt_match,
	},
};
module_platform_driver(snd_sm8250_driver);
MODULE_AUTHOR("Srinivas Kandagatla <srinivas.kandagatla@linaro.org");
MODULE_DESCRIPTION("SM8250 ASoC Machine Driver");
MODULE_LICENSE("GPL");
