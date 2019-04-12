/* SPDX-License-Identifier: GPL-2.0
 *
 * Copyright (C) 2018 Gateworks Corporation
 *
 * This driver registers a Linux Watchdog for the GSC
 */
#include <linux/mfd/gsc.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/watchdog.h>

#define WDT_DEFAULT_TIMEOUT	60

struct gsc_wdt {
	struct watchdog_device wdt_dev;
	struct gsc_dev *gsc;
};

static int gsc_wdt_start(struct watchdog_device *wdd)
{
	struct gsc_wdt *wdt = watchdog_get_drvdata(wdd);
	unsigned int reg = BIT(GSC_CTRL_1_WDT_ENABLE);
	int ret;

	/* clear first as regmap_update_bits will not write if no change */
	ret = regmap_update_bits(wdt->gsc->regmap, GSC_CTRL_1, reg, 0);
	if (ret)
		return ret;
	return regmap_update_bits(wdt->gsc->regmap, GSC_CTRL_1, reg, reg);
}

static int gsc_wdt_stop(struct watchdog_device *wdd)
{
	struct gsc_wdt *wdt = watchdog_get_drvdata(wdd);

	dev_dbg(wdd->parent, "%s\n", __func__);

	return regmap_update_bits(wdt->gsc->regmap, GSC_CTRL_1,
				  BIT(GSC_CTRL_1_WDT_ENABLE), 0);
}

static int gsc_wdt_set_timeout(struct watchdog_device *wdd,
			       unsigned int timeout)
{
	struct gsc_wdt *wdt = watchdog_get_drvdata(wdd);
	unsigned int long_sel = 0;

	dev_dbg(wdd->parent, "%s: %d\n", __func__, timeout);

	/* round second resolution up to 30s granularity */
	timeout = roundup(timeout, 30);
	if (timeout < 30)
		timeout = 30;
	else if (timeout > 60)
		timeout = 60;
	switch (timeout) {
	case 60:
		long_sel = BIT(GSC_CTRL_1_WDT_TIME);
	case 30:
		regmap_update_bits(wdt->gsc->regmap, GSC_CTRL_1,
				   BIT(GSC_CTRL_1_WDT_TIME),
				   (long_sel << GSC_CTRL_1_WDT_TIME));
		wdd->timeout = timeout;
		return 0;
	}

	return -EINVAL;
}

static const struct watchdog_info gsc_wdt_info = {
	/*
	 * WDIOF_MAGICCLOSE is not set because this driver never
	 * enables or disables the watchdog
	 */
	.options = WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING,
	.identity = "GSC Watchdog"
};

static const struct watchdog_ops gsc_wdt_ops = {
	.owner		= THIS_MODULE,
	.start		= gsc_wdt_start,
	.stop		= gsc_wdt_stop,
	.set_timeout	= gsc_wdt_set_timeout,
};

static int gsc_wdt_probe(struct platform_device *pdev)
{
	struct gsc_dev *gsc = dev_get_drvdata(pdev->dev.parent);
	struct device *dev = &pdev->dev;
	struct gsc_wdt *wdt;
	int ret;
	unsigned int reg;

	if (!gsc) {
		dev_err(dev, "defering probe waiting for GSC core\n");
		return -EPROBE_DEFER;
	}

	wdt = devm_kzalloc(dev, sizeof(*wdt), GFP_KERNEL);
	if (!wdt)
		return -ENOMEM;

	/* ensure GSC fw supports WD functionality */
	if (gsc->fwver < 44) {
		dev_err(dev, "fw v44 or newer required for wdt function\n");
		return -EINVAL;
	}

	/* ensure WD bit enabled */
	if (regmap_read(gsc->regmap, GSC_CTRL_1, &reg))
		return -EIO;
	if (!(reg & BIT(GSC_CTRL_1_WDT_ENABLE))) {
		dev_err(dev, "not enabled - must be manually enabled\n");
		return -EINVAL;
	}

	platform_set_drvdata(pdev, wdt);

	wdt->gsc = gsc;
	wdt->wdt_dev.info = &gsc_wdt_info;
	wdt->wdt_dev.ops = &gsc_wdt_ops;
	wdt->wdt_dev.min_timeout = 30;
	wdt->wdt_dev.max_timeout = 60;
	wdt->wdt_dev.parent = dev;

	watchdog_set_nowayout(&wdt->wdt_dev, 1);
	watchdog_init_timeout(&wdt->wdt_dev, WDT_DEFAULT_TIMEOUT, dev);

	watchdog_set_drvdata(&wdt->wdt_dev, wdt);
	ret = devm_watchdog_register_device(dev, &wdt->wdt_dev);
	if (ret)
		return ret;

	dev_info(dev, "watchdog driver (timeout=%d sec)\n",
		 wdt->wdt_dev.timeout);

	return 0;
}

static const struct of_device_id gsc_wdt_dt_ids[] = {
	{ .compatible = "gw,gsc-watchdog", },
	{}
};

static struct platform_driver gsc_wdt_driver = {
	.probe		= gsc_wdt_probe,
	.driver		= {
		.name	= "gsc-wdt",
		.of_match_table = gsc_wdt_dt_ids,
	},
};

module_platform_driver(gsc_wdt_driver);

MODULE_AUTHOR("Tim Harvey <tharvey@gateworks.com>");
MODULE_DESCRIPTION("Gateworks System Controller Watchdog driver");
MODULE_LICENSE("GPL v2");
