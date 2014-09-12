/*
 * gsp platform support
 *
 * Copyright (C) 2014 Gateworks Coproration
 *
 * Author: Tim Harvey <tharvey@gateworks.com>
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _GSP_CONFIG_H
#define _GSP_CONFIG_H

struct gsp_sensor_info {
        const char* name;
        int reg;
};

struct gsp_platform_data {
	struct gsp_sensor_info *sensors;
	int nsensors;
	bool has_fan_controller;
};
#endif
