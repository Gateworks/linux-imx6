/*
 * Copyright (C) 1999 ARM Limited
 * Copyright (C) 2000 Deep Blue Solutions Ltd
 * Copyright 2006-2015 Freescale Semiconductor, Inc.
 * Copyright 2008 Juergen Beisert, kernel@pengutronix.de
 * Copyright 2009 Ilya Yanok, Emcraft Systems Ltd, yanok@emcraft.com
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
 */

#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/of_address.h>

#include <asm/system_misc.h>
#include <asm/proc-fns.h>
#include <asm/mach-types.h>
#include <asm/hardware/cache-l2x0.h>

#include "common.h"
#include "hardware.h"

#ifdef CONFIG_MXC_REBOOT_ANDROID_CMD
/* This function will set a bit on SNVS_LPGPR[7-8] bits to enter
 * special boot mode.  These bits will not clear by watchdog reset, so
 * it can be checked by bootloader to choose enter different mode.*/

#define ANDROID_RECOVERY_BOOT  (1 << 7)
#define ANDROID_FASTBOOT_BOOT  (1 << 8)

#define MX6_AIPS1_ARB_BASE_ADDR		0x02000000
#define MX6_ATZ1_BASE_ADDR			MX6_AIPS1_ARB_BASE_ADDR
#define MX6_AIPS1_OFF_BASE_ADDR		(MX6_ATZ1_BASE_ADDR + 0x80000)
#define MX6_SNVS_BASE_ADDR		(MX6_AIPS1_OFF_BASE_ADDR + 0x4C000)
#define MX6_SNVS_LPGPR				0x68
#define MX6_SNVS_SIZE				(1024*16)

#define MX7_AIPS1_ARB_BASE_ADDR		0x30000000
#define MX7_ATZ1_BASE_ADDR			MX7_AIPS1_ARB_BASE_ADDR
#define MX7_AIPS1_OFF_BASE_ADDR		(MX7_ATZ1_BASE_ADDR + 0x200000)
#define MX7_SNVS_BASE_ADDR		(MX7_AIPS1_OFF_BASE_ADDR + 0x170000)
#define MX7_SNVS_LPGPR				0x68
#define MX7_SNVS_SIZE				(1024*16)
void do_switch_recovery(void)
{
	u32 reg;
	void *addr;
	struct clk *snvs_root;
	if(cpu_is_imx6()){
		addr = ioremap(MX6_SNVS_BASE_ADDR, MX6_SNVS_SIZE);
		if (!addr) {
			pr_warn("SNVS ioremap failed!\n");
			return;
		}
		reg = __raw_readl(addr + MX6_SNVS_LPGPR);
		reg |= ANDROID_RECOVERY_BOOT;
		__raw_writel(reg, (addr + MX6_SNVS_LPGPR));
	}else{
		snvs_root = clk_get_sys("imx-snvs.0", "snvs");
		addr = ioremap(MX7_SNVS_BASE_ADDR, MX7_SNVS_SIZE);	
		if (!addr) {
			pr_warn("SNVS ioremap failed!\n");
			return;
		}
		clk_enable(snvs_root);
		reg = __raw_readl(addr + MX7_SNVS_LPGPR);
		reg |= ANDROID_RECOVERY_BOOT;
		__raw_writel(reg, (addr + MX7_SNVS_LPGPR));
		clk_disable(snvs_root);
	}
	iounmap(addr);
}

void do_switch_fastboot(void)
{
	u32 reg;
	void *addr;
	struct clk *snvs_root;
	if(cpu_is_imx6()){
		addr = ioremap(MX6_SNVS_BASE_ADDR, MX6_SNVS_SIZE);
		if (!addr) {
			pr_warn("SNVS ioremap failed!\n");
			return;
		}
		reg = __raw_readl(addr + MX6_SNVS_LPGPR);
		reg |= ANDROID_FASTBOOT_BOOT;
		__raw_writel(reg, addr + MX6_SNVS_LPGPR);
	}else{
		snvs_root = clk_get_sys("imx-snvs.0", "snvs");
		addr = ioremap(MX7_SNVS_BASE_ADDR, MX7_SNVS_SIZE);	
		if (!addr) {
			pr_warn("SNVS ioremap failed!\n");
			return;
		}
		clk_enable(snvs_root);
		reg = __raw_readl(addr + MX7_SNVS_LPGPR);
		reg |= ANDROID_FASTBOOT_BOOT;
		__raw_writel(reg, addr + MX7_SNVS_LPGPR);
		clk_disable(snvs_root);
	}
	iounmap(addr);
}
#endif

#ifdef CONFIG_CACHE_L2X0
void __init imx_init_l2cache(void)
{
	void __iomem *l2x0_base;
	struct device_node *np;
	unsigned int val, cache_id;

	np = of_find_compatible_node(NULL, NULL, "arm,pl310-cache");
	if (!np)
		goto out;

	l2x0_base = of_iomap(np, 0);
	if (!l2x0_base) {
		of_node_put(np);
		goto out;
	}

	/* Configure the L2 PREFETCH and POWER registers */
	/* Set prefetch offset with any value except 23 as per errata 765569 */
	val = readl_relaxed(l2x0_base + L2X0_PREFETCH_CTRL);
	val |= 0x7000000f;
	/*
	 * The L2 cache controller(PL310) version on the i.MX6D/Q is r3p1-50rel0
	 * The L2 cache controller(PL310) version on the i.MX6DL/SOLO/SL/SX/DQP
	 * is r3p2.
	 * But according to ARM PL310 errata: 752271
	 * ID: 752271: Double linefill feature can cause data corruption
	 * Fault Status: Present in: r3p0, r3p1, r3p1-50rel0. Fixed in r3p2
	 * Workaround: The only workaround to this erratum is to disable the
	 * double linefill feature. This is the default behavior.
	 */
	cache_id = readl_relaxed(l2x0_base + L2X0_CACHE_ID);
	if (((cache_id & L2X0_CACHE_ID_PART_MASK) == L2X0_CACHE_ID_PART_L310)
	    && ((cache_id & L2X0_CACHE_ID_RTL_MASK) < L2X0_CACHE_ID_RTL_R3P2))
		val &= ~(1 << 30);
	writel_relaxed(val, l2x0_base + L2X0_PREFETCH_CTRL);
	val = L2X0_DYNAMIC_CLK_GATING_EN | L2X0_STNDBY_MODE_EN;
	writel_relaxed(val, l2x0_base + L2X0_POWER_CTRL);

	iounmap(l2x0_base);
	of_node_put(np);

out:
	l2x0_of_init(0, ~0UL);
}
#endif
