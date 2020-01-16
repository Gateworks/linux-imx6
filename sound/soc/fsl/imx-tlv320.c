/*
 * Copyright 2012 Freescale Semiconductor, Inc.
 * Copyright 2012 Linaro Ltd.
 *
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/i2c.h>
#include <linux/clk.h>
#include <linux/notifier.h>
#include <linux/of_gpio.h>
#include <sound/jack.h>
#include <sound/soc.h>

#include "../codecs/tlv320aic23.h"
#include "../codecs/ts3a227e.h"
#include "imx-audmux.h"

#define DAI_NAME_SIZE	32

struct imx_tlv320_data {
	struct snd_soc_dai_link dai;
	struct snd_soc_card card;
	char codec_dai_name[DAI_NAME_SIZE];
	char platform_name[DAI_NAME_SIZE];
	struct clk *codec_clk;
	unsigned int clk_frequency;
	int gpio_spkr_en;
};

static struct snd_soc_jack headset_jack;

static struct snd_soc_jack_pin imx_headset_jack_pins[] = {
	{
		.pin = "Headphone Jack",
		.mask = SND_JACK_HEADPHONE,
	},
	{
		.pin = "Mic Jack",
		.mask = SND_JACK_MICROPHONE,
	},
};

static struct snd_soc_aux_dev imx_tlv320_headset_dev = {
	.name = "Headset Chip",
	.codec_name = "ts3a227e.2-003b",
};

static int imx_headset_jack_event(struct notifier_block *nb,
		unsigned long event, void *data)
{
	struct snd_soc_jack *jack = (struct snd_soc_jack *)data;
	struct snd_soc_dapm_context *dapm = &jack->codec->dapm;
	struct device *dev = jack->codec->card->dev;

	/* Change audio routing according to headphone in/out */
	if (event & SND_JACK_HEADPHONE) {
		dev_info(dev, "headphone in\n");
		snd_soc_dapm_enable_pin(dapm, "Headphone Jack");
		snd_soc_dapm_disable_pin(dapm, "Int Spk");
		snd_soc_dapm_sync(dapm);
	} else {
		dev_info(dev, "headphone out\n");
		snd_soc_dapm_enable_pin(dapm, "Int Spk");
		snd_soc_dapm_disable_pin(dapm, "Headphone Jack");
		snd_soc_dapm_sync(dapm);
	}

	return 0;
}

static int imx_tlv320_spk(struct snd_soc_dapm_widget *w,
			  struct snd_kcontrol *ctrl, int event)
{
	struct snd_soc_dapm_context *dapm = w->dapm;
	struct snd_soc_card *card = dapm->card;
	struct imx_tlv320_data *imx_tlv320 = snd_soc_card_get_drvdata(card);

	if (!gpio_is_valid(imx_tlv320->gpio_spkr_en))
		return 0;

	gpio_set_value_cansleep(imx_tlv320->gpio_spkr_en,
				SND_SOC_DAPM_EVENT_ON(event));

	return 0;
}

static struct notifier_block imx_headset_jack_nb = {
	.notifier_call = imx_headset_jack_event,
};

static int imx_tlv320_dai_init(struct snd_soc_pcm_runtime *rtd)
{
	struct imx_tlv320_data *data = snd_soc_card_get_drvdata(rtd->card);
	struct device *dev = rtd->card->dev;
	struct snd_soc_dapm_context *dapm = &rtd->codec->dapm;
	int ret;

	ret = snd_soc_dai_set_sysclk(rtd->codec_dai, 0,
				     data->clk_frequency, SND_SOC_CLOCK_IN);
	if (ret) {
		dev_err(dev, "could not set codec driver clock params\n");
		return ret;
	}

	/* not connected */
	snd_soc_dapm_nc_pin(dapm, "MONO_LOUT");
	snd_soc_dapm_nc_pin(dapm, "HPLCOM");
	snd_soc_dapm_nc_pin(dapm, "HPRCOM");
	snd_soc_dapm_nc_pin(dapm, "Left Line Out");
	snd_soc_dapm_nc_pin(dapm, "Right Line Out");

	/* leave mic bias on for headset button detect while no stream active */
	dapm->idle_bias_off = false;

	return 0;
}

static const struct snd_soc_dapm_widget imx_tlv320_dapm_widgets[] = {
	SND_SOC_DAPM_MIC("Mic Jack", NULL),
	SND_SOC_DAPM_LINE("Line In", NULL),
	SND_SOC_DAPM_HP("Headphone Jack", NULL),
	SND_SOC_DAPM_SPK("Int Spk", imx_tlv320_spk),
};

static int imx_tlv320_audmux_config(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	int int_port, ext_port;
	int ret;

	ret = of_property_read_u32(np, "mux-int-port", &int_port);
	if (ret) {
		dev_err(&pdev->dev, "mux-int-port missing or invalid\n");
		return ret;
	}
	ret = of_property_read_u32(np, "mux-ext-port", &ext_port);
	if (ret) {
		dev_err(&pdev->dev, "mux-ext-port missing or invalid\n");
		return ret;
	}

	/*
	 * The port numbering in the hardware manual starts at 1, while
	 * the audmux API expects it starts at 0.
	 */
	int_port--;
	ext_port--;
	ret = imx_audmux_v2_configure_port(int_port,
			IMX_AUDMUX_V2_PTCR_SYN |
			IMX_AUDMUX_V2_PTCR_TFSEL(ext_port) |
			IMX_AUDMUX_V2_PTCR_TCSEL(ext_port) |
			IMX_AUDMUX_V2_PTCR_TFSDIR |
			IMX_AUDMUX_V2_PTCR_TCLKDIR,
			IMX_AUDMUX_V2_PDCR_RXDSEL(ext_port));
	if (ret) {
		dev_err(&pdev->dev, "audmux internal port setup failed\n");
		return ret;
	}
	ret = imx_audmux_v2_configure_port(ext_port,
			IMX_AUDMUX_V2_PTCR_SYN,
			IMX_AUDMUX_V2_PDCR_RXDSEL(int_port));
	if (ret) {
		dev_err(&pdev->dev, "audmux external port setup failed\n");
		return ret;
	}

	return 0;
}

static int imx_tlv320_late_probe(struct snd_soc_card *card)
{
	struct snd_soc_codec *codec = card->rtd[0].codec;
	int ret;

	ret = snd_soc_jack_new(codec, "Headset Jack",
			       SND_JACK_HEADPHONE |
			       SND_JACK_MICROPHONE |
			       SND_JACK_BTN_0 | SND_JACK_BTN_1 |
			       SND_JACK_BTN_2 | SND_JACK_BTN_3,
			       &headset_jack);
	if (ret) {
		dev_err(card->dev, "could not register headset jack detect\n");
		return ret;
	}
	ret = snd_soc_jack_add_pins(&headset_jack,
		ARRAY_SIZE(imx_headset_jack_pins), imx_headset_jack_pins);
	if (ret) {
		dev_err(card->dev, "could not register headset jack pins\n");
		return ret;
	}

	snd_soc_jack_notifier_register(&headset_jack, &imx_headset_jack_nb);

	ret = ts3a227e_enable_jack_detect(card, &headset_jack);
	if (ret) {
		dev_err(card->dev, "could not enable jack detect\n");
		return ret;
	}

	return 0;
}

static int imx_tlv320_probe(struct platform_device *pdev)
{
	struct device_node *cpu_np, *codec_np;
	struct platform_device *cpu_pdev;
	struct i2c_client *codec_dev;
	struct imx_tlv320_data *data = NULL;
	int ret;

	cpu_np = of_parse_phandle(pdev->dev.of_node, "cpu-dai", 0);
	codec_np = of_parse_phandle(pdev->dev.of_node, "audio-codec", 0);
	if (!cpu_np || !codec_np) {
		dev_err(&pdev->dev, "phandle missing or invalid\n");
		ret = -EINVAL;
		goto fail;
	}

	if (strstr(cpu_np->name, "ssi")) {
		ret = imx_tlv320_audmux_config(pdev);
		if (ret)
			goto fail;
	}

	cpu_pdev = of_find_device_by_node(cpu_np);
	if (!cpu_pdev) {
		dev_err(&pdev->dev, "failed to find SSI platform device\n");
		ret = -EPROBE_DEFER;
		goto fail;
	}
	codec_dev = of_find_i2c_device_by_node(codec_np);
	if (!codec_dev) {
		dev_err(&pdev->dev, "failed to find codec platform device\n");
		return -EPROBE_DEFER;
	}

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data) {
		ret = -ENOMEM;
		goto fail;
	}
	data->gpio_spkr_en = of_get_named_gpio(pdev->dev.of_node, "spkr-en-gpios", 0);
	if (gpio_is_valid(data->gpio_spkr_en)) {
		ret = devm_gpio_request_one(&pdev->dev, data->gpio_spkr_en,
					    GPIOF_OUT_INIT_LOW, "spkr_en");
		if (ret) {
			dev_err(&pdev->dev, "cannot get spkr_en gpio\n");
			goto fail;
		}
	}

	data->codec_clk = clk_get(&codec_dev->dev, NULL);
	if (IS_ERR(data->codec_clk)) {
		ret = PTR_ERR(data->codec_clk);
		dev_err(&pdev->dev, "failed to find clock\n");
		goto fail;
	}

	ret = clk_prepare_enable(data->codec_clk);
	if (ret) {
		dev_err(&pdev->dev, "Error enabling clock: %d\n", ret);
		goto fail;
	}
	data->clk_frequency = clk_get_rate(data->codec_clk);

	data->dai.name = "tlv320aic3x";
	data->dai.stream_name = "TLV320AIC3X";
	data->dai.codec_dai_name = "tlv320aic3x-hifi";
	data->dai.codec_of_node = codec_np;
	data->dai.cpu_of_node = cpu_np;
	data->dai.platform_of_node = cpu_np;
	data->dai.init = &imx_tlv320_dai_init;
	data->dai.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
			    SND_SOC_DAIFMT_CBM_CFM;

	data->card.dev = &pdev->dev;
	ret = snd_soc_of_parse_card_name(&data->card, "model");
	if (ret)
		goto fail;
	ret = snd_soc_of_parse_audio_routing(&data->card, "audio-routing");
	if (ret)
		goto fail;
	data->card.num_links = 1;
	data->card.owner = THIS_MODULE;
	data->card.dai_link = &data->dai;
	data->card.dapm_widgets = imx_tlv320_dapm_widgets;
	data->card.num_dapm_widgets = ARRAY_SIZE(imx_tlv320_dapm_widgets);
	data->card.aux_dev = &imx_tlv320_headset_dev;
	data->card.num_aux_devs = 1;
	data->card.late_probe = imx_tlv320_late_probe;

	platform_set_drvdata(pdev, &data->card);
	snd_soc_card_set_drvdata(&data->card, data);

	ret = devm_snd_soc_register_card(&pdev->dev, &data->card);
	if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n", ret);
		goto fail;
	}

	of_node_put(cpu_np);
	of_node_put(codec_np);

	return 0;

fail:
	if (data && !IS_ERR(data->codec_clk))
		clk_put(data->codec_clk);
	if (cpu_np)
		of_node_put(cpu_np);
	if (codec_np)
		of_node_put(codec_np);

	return ret;
}

static int imx_tlv320_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);
	struct imx_tlv320_data *data = snd_soc_card_get_drvdata(card);

	snd_soc_jack_notifier_unregister(&headset_jack, &imx_headset_jack_nb);
	clk_put(data->codec_clk);

	return 0;
}

static const struct of_device_id imx_tlv320_dt_ids[] = {
	{ .compatible = "fsl,imx-audio-tlv320", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, imx_tlv320_dt_ids);

static struct platform_driver imx_tlv320_driver = {
	.driver = {
		.name = "imx-tlv320",
		.owner = THIS_MODULE,
		.pm = &snd_soc_pm_ops,
		.of_match_table = imx_tlv320_dt_ids,
	},
	.probe = imx_tlv320_probe,
	.remove = imx_tlv320_remove,
};
module_platform_driver(imx_tlv320_driver);

MODULE_AUTHOR("Starterkit <info@starterkit.ru>");
MODULE_DESCRIPTION("imx with TLV320AIC23B codec ASoC machine driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:imx-tlv320");
