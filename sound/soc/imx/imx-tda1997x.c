/*
 * sound/soc/imx/imx-tda1997x.c - SoC audio for i.MX boards with
 *         tda1997x HDMI receiver
 * based off source/soc/imx/imx-sgtl5000.c
 *
 * Copyright 2013 Tim Harvey, Gateworks Corporation <tharvey@gateworks.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 *
 */
#define DEBUG
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/gpio.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include <asm/mach-types.h>
#include <mach/audmux.h>

#include "imx-ssi.h"
#include <linux/fsl_devices.h>
#include <linux/mfd/tda1997x-core.h>

static struct imx_tda1997x_priv {
	int sysclk;
	int hw;
	struct platform_device *pdev;
} card_priv;

static int tda1997x_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	u32 dai_format;
	int ret;
	unsigned int channels = params_channels(params);

	printk("%s freq=%d channels=%d format=%d period_size=%d periods=%d bufsize=%d bufbytes=%d\n", __func__, params_rate(params), channels, params_format(params), params_period_size(params), params_periods(params), params_buffer_size(params), params_buffer_bytes(params));

	/* there is nothing the codec can or should do regarding format or samplerate
	 * as those are dependent on the input stream.  However, perhaps somewhere a
	 * call to tda1997x_get_audiofmt should be done to ensure the format/rate
	 * asked for is compatible with the current stream?  */

	/* soc/cpu dai params
	 */

	/* TODO: The SSI driver should figure this out for us */
	switch (channels) {
	case 2:
		snd_soc_dai_set_tdm_slot(cpu_dai, 0xfffffffc, 0xfffffffc, 2, 0);
		break;
	case 1:
		snd_soc_dai_set_tdm_slot(cpu_dai, 0xfffffffe, 0xfffffffe, 1, 0);
		break;
	default:
		return -EINVAL;
	}

	/* set cpu DAI configuration */
	dai_format = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_IF |
		SND_SOC_DAIFMT_CBM_CFM;
	ret = snd_soc_dai_set_fmt(cpu_dai, dai_format);
	if (ret < 0)
		return ret;

	return 0;
}

static struct snd_soc_ops imx_tda1997x_ops = {
	.hw_params = tda1997x_params,
};

/* imx digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link imx_tda1997x_dai_link = {
		.name		= "IMX HDMI RX",
		.stream_name	= "IMX HDMI RX",
		.codec_dai_name	= "tda1997x",
		.codec_name	= "tda1997x_codec.0",
#if 0
		.cpu_dai_name	= "imx-ssi.2",
		.platform_name	= "imx-pcm-audio.2",
#else
 // need to change registration for this
		.cpu_dai_name	= "imx-ssi.0",
		.platform_name	= "imx-pcm-audio.0",
#endif
		.ops = &imx_tda1997x_ops,
};

static struct snd_soc_card snd_soc_card_imx_tda1997x = {
	.name		= "tda1997x-audio",
	.dai_link	= &imx_tda1997x_dai_link,
	.num_links	= 1,
};

static struct platform_device *imx_tda1997x_snd_device;

static int imx_audmux_config(int slave, int master)
{
	unsigned int ptcr, pdcr;
	pr_debug("%s tda1997x slave=%d master=%d\n", __func__, slave, master);
	slave = slave - 1;
	master = master - 1;

	ptcr = MXC_AUDMUX_V2_PTCR_SYN |
		MXC_AUDMUX_V2_PTCR_TFSDIR |
		MXC_AUDMUX_V2_PTCR_TFSEL(master) |
		MXC_AUDMUX_V2_PTCR_TCLKDIR |
		MXC_AUDMUX_V2_PTCR_TCSEL(master);
	ptcr |= (1<<30); /* use RXFS from port instead of TXFS */
	ptcr |= (1<<25); /* use RXC fro port instead of TXC */
	pdcr = MXC_AUDMUX_V2_PDCR_RXDSEL(master);
	mxc_audmux_v2_configure_port(slave, ptcr, pdcr);

	ptcr = MXC_AUDMUX_V2_PTCR_SYN;
	pdcr = MXC_AUDMUX_V2_PDCR_RXDSEL(slave);
	mxc_audmux_v2_configure_port(master, ptcr, pdcr);

	return 0;
}

static int __devinit imx_tda1997x_probe(struct platform_device *pdev)
{
	struct mxc_audio_platform_data *plat = pdev->dev.platform_data;

	pr_debug("%s: src_port=%d ext_port=%d\n", __func__,
		plat->src_port, plat->ext_port);
	card_priv.pdev = pdev;
	card_priv.sysclk = plat->sysclk;
	imx_audmux_config(plat->src_port, plat->ext_port);
	if (plat->init && plat->init())
		return -EINVAL;

	return 0;
}

static int imx_tda1997x_remove(struct platform_device *pdev)
{
	struct mxc_audio_platform_data *plat = pdev->dev.platform_data;

	pr_debug("%s\n", __func__);
	if (plat->finit)
		plat->finit();

	return 0;
}

static struct platform_driver imx_tda1997x_audio_driver = {
	.probe = imx_tda1997x_probe,
	.remove = imx_tda1997x_remove,
	.driver = {
		   .name = "imx-tda1997x",
		   },
};

static int __init imx_tda1997x_init(void)
{
	int ret;

	pr_debug("%s\n", __func__);
	/* register platform driver which configures the i.MX6 audmux
	 * per platform data
	 */
	ret = platform_driver_register(&imx_tda1997x_audio_driver);
	if (ret)
		return -ENOMEM;
	imx_tda1997x_snd_device = platform_device_alloc("soc-audio", 6);
	if (!imx_tda1997x_snd_device) {
		pr_err("%s - failed platform_device_alloc\n", __func__);
		return -ENOMEM;
	}

	/* register sound card which provides the mapping between codec and dai */
	platform_set_drvdata(imx_tda1997x_snd_device, &snd_soc_card_imx_tda1997x);

	ret = platform_device_add(imx_tda1997x_snd_device);
	if (ret) {
		pr_err("ASoC HDMI RX: Platform device allocation failed\n");
		platform_device_put(imx_tda1997x_snd_device);
	}

	return ret;
}

static void __exit imx_tda1997x_exit(void)
{
	pr_debug("%s\n", __func__);
	platform_driver_unregister(&imx_tda1997x_audio_driver);
	platform_device_unregister(imx_tda1997x_snd_device);
}

module_init(imx_tda1997x_init);
module_exit(imx_tda1997x_exit);

MODULE_AUTHOR("Tim Harvey <tharvey@gateworks.com>");
MODULE_DESCRIPTION("IMX TDA1997X RX ASoC driver");
MODULE_LICENSE("GPL");
