/*
 * ddbridge-hw.c: Digital Devices bridge hardware maps
 *
 * Copyright (C) 2010-2017 Digital Devices GmbH
 *                         Ralph Metzler <rjkm@metzlerbros.de>
 *                         Marcus Metzler <mocm@metzlerbros.de>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 only, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include "ddbridge.h"

/******************************************************************************/

static struct ddb_regset octopus_input = {
	.base = 0x200,
	.num  = 0x08,
	.size = 0x10,
};

static struct ddb_regset octopus_output = {
	.base = 0x280,
	.num  = 0x08,
	.size = 0x10,
};

static struct ddb_regset octopus_idma = {
	.base = 0x300,
	.num  = 0x08,
	.size = 0x10,
};

static struct ddb_regset octopus_idma_buf = {
	.base = 0x2000,
	.num  = 0x08,
	.size = 0x100,
};

static struct ddb_regset octopus_odma = {
	.base = 0x380,
	.num  = 0x04,
	.size = 0x10,
};

static struct ddb_regset octopus_odma_buf = {
	.base = 0x2800,
	.num  = 0x04,
	.size = 0x100,
};

static struct ddb_regset octopus_i2c = {
	.base = 0x80,
	.num  = 0x04,
	.size = 0x20,
};

static struct ddb_regset octopus_i2c_buf = {
	.base = 0x1000,
	.num  = 0x04,
	.size = 0x200,
};

/****************************************************************************/

static struct ddb_regmap octopus_map = {
	.irq_base_i2c = 0,
	.irq_base_idma = 8,
	.irq_base_odma = 16,
	.i2c = &octopus_i2c,
	.i2c_buf = &octopus_i2c_buf,
	.idma = &octopus_idma,
	.idma_buf = &octopus_idma_buf,
	.odma = &octopus_odma,
	.odma_buf = &octopus_odma_buf,
	.input = &octopus_input,
	.output = &octopus_output,
};

/****************************************************************************/

const struct ddb_info ddb_none = {
	.type     = DDB_NONE,
	.name     = "unknown Digital Devices PCIe card, install newer driver",
	.regmap   = &octopus_map,
};

const struct ddb_info ddb_octopus = {
	.type     = DDB_OCTOPUS,
	.name     = "Digital Devices Octopus DVB adapter",
	.regmap   = &octopus_map,
	.port_num = 4,
	.i2c_mask = 0x0f,
};

const struct ddb_info ddb_octopusv3 = {
	.type     = DDB_OCTOPUS,
	.name     = "Digital Devices Octopus V3 DVB adapter",
	.regmap   = &octopus_map,
	.port_num = 4,
	.i2c_mask = 0x0f,
};

const struct ddb_info ddb_octopus_le = {
	.type     = DDB_OCTOPUS,
	.name     = "Digital Devices Octopus LE DVB adapter",
	.regmap   = &octopus_map,
	.port_num = 2,
	.i2c_mask = 0x03,
};

const struct ddb_info ddb_octopus_oem = {
	.type     = DDB_OCTOPUS,
	.name     = "Digital Devices Octopus OEM",
	.regmap   = &octopus_map,
	.port_num = 4,
	.i2c_mask = 0x0f,
	.led_num  = 1,
	.fan_num  = 1,
	.temp_num = 1,
	.temp_bus = 0,
};

const struct ddb_info ddb_octopus_mini = {
	.type     = DDB_OCTOPUS,
	.name     = "Digital Devices Octopus Mini",
	.regmap   = &octopus_map,
	.port_num = 4,
	.i2c_mask = 0x0f,
};

const struct ddb_info ddb_v6 = {
	.type     = DDB_OCTOPUS,
	.name     = "Digital Devices Cine S2 V6 DVB adapter",
	.regmap   = &octopus_map,
	.port_num = 3,
	.i2c_mask = 0x07,
};

const struct ddb_info ddb_v6_5 = {
	.type     = DDB_OCTOPUS,
	.name     = "Digital Devices Cine S2 V6.5 DVB adapter",
	.regmap   = &octopus_map,
	.port_num = 4,
	.i2c_mask = 0x0f,
};

const struct ddb_info ddb_v7 = {
	.type     = DDB_OCTOPUS,
	.name     = "Digital Devices Cine S2 V7 DVB adapter",
	.regmap   = &octopus_map,
	.port_num = 4,
	.i2c_mask = 0x0f,
	.board_control   = 2,
	.board_control_2 = 4,
	.ts_quirks = TS_QUIRK_REVERSED,
};

const struct ddb_info ddb_v7a = {
	.type     = DDB_OCTOPUS,
	.name     = "Digital Devices Cine S2 V7 Advanced DVB adapter",
	.regmap   = &octopus_map,
	.port_num = 4,
	.i2c_mask = 0x0f,
	.board_control   = 2,
	.board_control_2 = 4,
	.ts_quirks = TS_QUIRK_REVERSED,
};

const struct ddb_info ddb_ctv7 = {
	.type     = DDB_OCTOPUS,
	.name     = "Digital Devices Cine CT V7 DVB adapter",
	.regmap   = &octopus_map,
	.port_num = 4,
	.i2c_mask = 0x0f,
	.board_control   = 3,
	.board_control_2 = 4,
};

const struct ddb_info ddb_satixS2v3 = {
	.type     = DDB_OCTOPUS,
	.name     = "Mystique SaTiX-S2 V3 DVB adapter",
	.regmap   = &octopus_map,
	.port_num = 3,
	.i2c_mask = 0x07,
};

const struct ddb_info ddb_ci = {
	.type     = DDB_OCTOPUS_CI,
	.name     = "Digital Devices Octopus CI",
	.regmap   = &octopus_map,
	.port_num = 4,
	.i2c_mask = 0x03,
};

const struct ddb_info ddb_cis = {
	.type     = DDB_OCTOPUS_CI,
	.name     = "Digital Devices Octopus CI single",
	.regmap   = &octopus_map,
	.port_num = 3,
	.i2c_mask = 0x03,
};

const struct ddb_info ddb_ci_s2_pro = {
	.type     = DDB_OCTOPUS_CI,
	.name     = "Digital Devices Octopus CI S2 Pro",
	.regmap   = &octopus_map,
	.port_num = 4,
	.i2c_mask = 0x01,
	.board_control   = 2,
	.board_control_2 = 4,
};

const struct ddb_info ddb_ci_s2_pro_a = {
	.type     = DDB_OCTOPUS_CI,
	.name     = "Digital Devices Octopus CI S2 Pro Advanced",
	.regmap   = &octopus_map,
	.port_num = 4,
	.i2c_mask = 0x01,
	.board_control   = 2,
	.board_control_2 = 4,
};

const struct ddb_info ddb_dvbct = {
	.type     = DDB_OCTOPUS,
	.name     = "Digital Devices DVBCT V6.1 DVB adapter",
	.regmap   = &octopus_map,
	.port_num = 3,
	.i2c_mask = 0x07,
};

/****************************************************************************/

const struct ddb_info ddb_ct2_8 = {
	.type     = DDB_OCTOPUS_MAX_CT,
	.name     = "Digital Devices MAX A8 CT2",
	.regmap   = &octopus_map,
	.port_num = 4,
	.i2c_mask = 0x0f,
	.board_control   = 0x0ff,
	.board_control_2 = 0xf00,
	.ts_quirks = TS_QUIRK_SERIAL,
	.tempmon_irq = 24,
};

const struct ddb_info ddb_c2t2_8 = {
	.type     = DDB_OCTOPUS_MAX_CT,
	.name     = "Digital Devices MAX A8 C2T2",
	.regmap   = &octopus_map,
	.port_num = 4,
	.i2c_mask = 0x0f,
	.board_control   = 0x0ff,
	.board_control_2 = 0xf00,
	.ts_quirks = TS_QUIRK_SERIAL,
	.tempmon_irq = 24,
};

const struct ddb_info ddb_isdbt_8 = {
	.type     = DDB_OCTOPUS_MAX_CT,
	.name     = "Digital Devices MAX A8 ISDBT",
	.regmap   = &octopus_map,
	.port_num = 4,
	.i2c_mask = 0x0f,
	.board_control   = 0x0ff,
	.board_control_2 = 0xf00,
	.ts_quirks = TS_QUIRK_SERIAL,
	.tempmon_irq = 24,
};

const struct ddb_info ddb_c2t2i_v0_8 = {
	.type     = DDB_OCTOPUS_MAX_CT,
	.name     = "Digital Devices MAX A8 C2T2I V0",
	.regmap   = &octopus_map,
	.port_num = 4,
	.i2c_mask = 0x0f,
	.board_control   = 0x0ff,
	.board_control_2 = 0xf00,
	.ts_quirks = TS_QUIRK_SERIAL | TS_QUIRK_ALT_OSC,
	.tempmon_irq = 24,
};

const struct ddb_info ddb_c2t2i_8 = {
	.type     = DDB_OCTOPUS_MAX_CT,
	.name     = "Digital Devices MAX A8 C2T2I",
	.regmap   = &octopus_map,
	.port_num = 4,
	.i2c_mask = 0x0f,
	.board_control   = 0x0ff,
	.board_control_2 = 0xf00,
	.ts_quirks = TS_QUIRK_SERIAL,
	.tempmon_irq = 24,
};

/****************************************************************************/

const struct ddb_info ddb_s2_48 = {
	.type     = DDB_OCTOPUS_MAX,
	.name     = "Digital Devices MAX S8 4/8",
	.regmap   = &octopus_map,
	.port_num = 4,
	.i2c_mask = 0x01,
	.board_control = 1,
	.tempmon_irq = 24,
};
