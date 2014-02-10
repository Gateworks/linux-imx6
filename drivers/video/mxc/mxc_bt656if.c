/*
 * Copyright (C) 2013 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */
#define DEBUG

#include <linux/init.h>
#include <linux/ipu.h>
#include <linux/mxcfb.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>

#include "mxc_dispdrv.h"

struct mxc_bt656if_data {
	struct platform_device *pdev;
	struct mxc_dispdrv_handle *disp_bt656if;
};

struct mxc_bt656_platform_data {
	u32 default_ifmt;
	u32 ipu_id;
	u32 disp_id;
};

#define DISPDRV_BT656	"bt656"

/*
 * left_margin: used for field0 vStart width in lines
 *
 * right_margin: used for field0 vEnd width in lines
 *
 * up_margin: used for field1 vStart width in lines
 *
 * down_margin: used for field1 vEnd width in lines
 *
 * hsync_len: EAV Code + Blanking Video + SAV Code (in pixel clock count)
 *         For BT656 NTSC, it is 4 + 67*4 + 4 = 276.
 *         For BT1120 NTSC, it is 4 + 67*2 + 4 = 142.
 *         For BT656 PAL, it is 4 + 70*4 + 4 = 288.
 *         For BT1120 PAL, it is 4 + 70*2 + 4 = 148.
 *
 * vsync_len: not used, set to 1
 */
static struct fb_videomode bt656if_modedb[] = {
	{
	 /* NTSC Interlaced output */
	 "BT656-NTSC", 60, 720, 480, 37037,
	 19, 3,
	 20, 3,
	 276, 1,
	 FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
	 FB_VMODE_INTERLACED,
	 FB_MODE_IS_DETAILED,},
	{
	 /* PAL Interlaced output */
	 "BT656-PAL", 50, 720, 576, 37037,
	 22, 2,
	 23, 2,
	 288, 1,
	 FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
	 FB_VMODE_INTERLACED,
	 FB_MODE_IS_DETAILED,},
	{
	 /* NTSC Interlaced output */
	 "BT1120-NTSC", 30, 720, 480, 74074,
	 19, 3,
	 20, 3,
	 142, 1,
	 FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
	 FB_VMODE_INTERLACED,
	 FB_MODE_IS_DETAILED,},
	{
	 /* PAL Interlaced output */
	 "BT1120-PAL", 25, 720, 576, 74074,
	 22, 2,
	 23, 2,
	 148, 1,
	 FB_SYNC_HOR_HIGH_ACT | FB_SYNC_VERT_HIGH_ACT,
	 FB_VMODE_INTERLACED,
	 FB_MODE_IS_DETAILED,},
};
static int bt656if_modedb_sz = ARRAY_SIZE(bt656if_modedb);

static int bt656if_init(struct mxc_dispdrv_handle *disp,
	struct mxc_dispdrv_setting *setting)
{
	int ret, i;
	struct mxc_bt656if_data *bt656if = mxc_dispdrv_getdata(disp);
	struct mxc_bt656_platform_data *plat_data
			= bt656if->pdev->dev.platform_data;
	struct fb_videomode *modedb = bt656if_modedb;
	int modedb_sz = bt656if_modedb_sz;

	/* use platform defined ipu/di */
	setting->dev_id = plat_data->ipu_id;
	setting->disp_id = plat_data->disp_id;

	ret = fb_find_mode(&setting->fbi->var, setting->fbi, setting->dft_mode_str,
				modedb, modedb_sz, NULL, setting->default_bpp);
	if (!ret) {
		fb_videomode_to_var(&setting->fbi->var, &modedb[0]);
		setting->if_fmt = plat_data->default_ifmt;
	}
	dev_dbg(&bt656if->pdev->dev, "%s: IPU%d_DISP%d %c%c%c%c\n", __func__,
		setting->dev_id + 1, setting->disp_id,
		(setting->if_fmt >> 0) & 0xff,
		(setting->if_fmt >> 8) & 0xff,
		(setting->if_fmt >> 16) & 0xff,
		(setting->if_fmt >> 24) & 0xff);

	INIT_LIST_HEAD(&setting->fbi->modelist);
	for (i = 0; i < modedb_sz; i++) {
		fb_add_videomode(&modedb[i],
				&setting->fbi->modelist);
	}

	return ret;
}

static void bt656if_deinit(struct mxc_dispdrv_handle *disp)
{
	/*TODO*/
}

static struct mxc_dispdrv_driver bt656if_drv = {
	.name = DISPDRV_BT656,
	.init = bt656if_init,
	.deinit = bt656if_deinit,
};

static int bt656_get_of_property(struct platform_device *pdev,
		struct mxc_bt656_platform_data *plat_data)
{
	struct device_node *np = pdev->dev.of_node;
	int err;
	u32 ipu_id, disp_id;
	const char *default_ifmt;

	err = of_property_read_string(np, "default_ifmt", &default_ifmt);
	if (err) {
		dev_err(&pdev->dev, "get of property default_ifmt fail\n");
		return err;
	}
	err = of_property_read_u32(np, "ipu_id", &ipu_id);
	if (err) {
		dev_err(&pdev->dev, "get of property ipu_id fail\n");
		return err;
	}
	err = of_property_read_u32(np, "disp_id", &disp_id);
	if (err) {
		dev_err(&pdev->dev, "get of property disp_id fail\n");
		return err;
	}

	plat_data->ipu_id = ipu_id;
	plat_data->disp_id = disp_id;
	if (!strncmp(default_ifmt, "BT656", 5))
		plat_data->default_ifmt = IPU_PIX_FMT_BT656;
	else if (!strncmp(default_ifmt, "BT1120", 6))
		plat_data->default_ifmt = IPU_PIX_FMT_BT1120;
	else {
		dev_err(&pdev->dev, "err default_ifmt!\n");
		return -ENOENT;
	}

	return err;
}

static int mxc_bt656if_probe(struct platform_device *pdev)
{
	int ret;
	struct pinctrl *pinctrl;
	struct mxc_bt656if_data *bt656if;
	struct mxc_bt656_platform_data *plat_data;

	bt656if = devm_kzalloc(&pdev->dev, sizeof(struct mxc_bt656if_data),
			       GFP_KERNEL);
	if (!bt656if)
		return -ENOMEM;
	plat_data = devm_kzalloc(&pdev->dev,
				 sizeof(struct mxc_bt656_platform_data),
				 GFP_KERNEL);
	if (!plat_data)
		return -ENOMEM;
	pdev->dev.platform_data = plat_data;

	ret = bt656_get_of_property(pdev, plat_data);
	if (ret < 0) {
		dev_err(&pdev->dev, "get bt656 of property fail\n");
		return ret;
	}

	pinctrl = devm_pinctrl_get_select_default(&pdev->dev);
	if (IS_ERR(pinctrl)) {
		dev_err(&pdev->dev, "can't get/select pinctrl\n");
		return PTR_ERR(pinctrl);
	}

	bt656if->pdev = pdev;
	bt656if->disp_bt656if = mxc_dispdrv_register(&bt656if_drv);
	mxc_dispdrv_setdata(bt656if->disp_bt656if, bt656if);

	dev_set_drvdata(&pdev->dev, bt656if);

	return ret;
}

static int mxc_bt656if_remove(struct platform_device *pdev)
{
	struct mxc_bt656if_data *bt656if = dev_get_drvdata(&pdev->dev);

	mxc_dispdrv_puthandle(bt656if->disp_bt656if);
	mxc_dispdrv_unregister(bt656if->disp_bt656if);
	kfree(bt656if);
	return 0;
}

static const struct of_device_id imx_bt656_dt_ids[] = {
	{ .compatible = "fsl,bt656"},
	{ /* sentinel */ }
};
static struct platform_driver mxc_bt656if_driver = {
	.driver = {
		.name = "mxc_bt656if",
		.of_match_table = imx_bt656_dt_ids,
	},
	.probe = mxc_bt656if_probe,
	.remove = mxc_bt656if_remove,
};

static int __init mxc_bt656if_init(void)
{
	return platform_driver_register(&mxc_bt656if_driver);
}

static void __exit mxc_bt656if_exit(void)
{
	platform_driver_unregister(&mxc_bt656if_driver);
}

module_init(mxc_bt656if_init);
module_exit(mxc_bt656if_exit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("i.MX ipuv3 BT656 and BT1120 output driver");
MODULE_LICENSE("GPL");
