/*
 * ngene-cards.c: nGene PCIe bridge driver - card specific info
 *
 * Copyright (C) 2005-2007 Micronas
 *
 * Copyright (C) 2008-2009 Ralph Metzler <rjkm@metzlerbros.de>
 *                         Modifications for new nGene firmware,
 *                         support for EEPROM-copying,
 *                         support for new dual DVB-S2 card prototype
 *
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 only, as published by the Free Software Foundation.
 *
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * To obtain the license, point your browser to
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/pci_ids.h>
#include "compat.h"

#include "ngene.h"

/* demods/tuners */
#include "stv6110x.h"
#include "stv090x.h"
#include "lnbh24.h"
#include "lgdt330x.h"
#include "mt2131.h"
#include "tda18271c2dd.h"
#include "drxk.h"
#include "tda18212dd.h"
#include "stv0367dd.h"
#include "cxd2843.h"


enum DEMOD_TYPE {
	DMD_NONE = 0,
	DMD_STV0900,
	DMD_DRXK,
	DMD_STV0367,
	DMD_CXD28xx,
};


/****************************************************************************/
/* Demod/tuner attachment ***************************************************/
/****************************************************************************/

static int tuner_attach_stv6110(struct ngene_channel *chan)
{
	struct i2c_adapter *i2c;
	struct stv090x_config *feconf = (struct stv090x_config *)
		chan->dev->card_info->fe_config[chan->number];
	struct stv6110x_config *tunerconf = (struct stv6110x_config *)
		chan->dev->card_info->tuner_config[chan->number];
	struct stv6110x_devctl *ctl;

	/* tuner 1+2: i2c adapter #0, tuner 3+4: i2c adapter #1 */
	if (chan->number < 2)
		i2c = &chan->dev->channel[0].i2c_adapter;
	else
		i2c = &chan->dev->channel[1].i2c_adapter;

	ctl = dvb_attach(stv6110x_attach, chan->fe, tunerconf, i2c);
	if (ctl == NULL) {
		printk(KERN_ERR	DEVICE_NAME ": No STV6110X found!\n");
		return -ENODEV;
	}

	feconf->tuner_init          = ctl->tuner_init;
	feconf->tuner_sleep         = ctl->tuner_sleep;
	feconf->tuner_set_mode      = ctl->tuner_set_mode;
	feconf->tuner_set_frequency = ctl->tuner_set_frequency;
	feconf->tuner_get_frequency = ctl->tuner_get_frequency;
	feconf->tuner_set_bandwidth = ctl->tuner_set_bandwidth;
	feconf->tuner_get_bandwidth = ctl->tuner_get_bandwidth;
	feconf->tuner_set_bbgain    = ctl->tuner_set_bbgain;
	feconf->tuner_get_bbgain    = ctl->tuner_get_bbgain;
	feconf->tuner_set_refclk    = ctl->tuner_set_refclk;
	feconf->tuner_get_status    = ctl->tuner_get_status;

	return 0;
}

static int locked_gate_ctrl(struct dvb_frontend *fe, int enable)
{
	struct ngene_channel *chan = fe->sec_priv;
	int status;

	if (enable) {
		down(&chan->dev->pll_mutex);
		status = chan->gate_ctrl(fe, 1);
	} else {
		status = chan->gate_ctrl(fe, 0);
		up(&chan->dev->pll_mutex);
	}
	return status;
}

static int tuner_attach_tda18271(struct ngene_channel *chan)
{
	struct i2c_adapter *i2c;
	struct dvb_frontend *fe;

	i2c = &chan->dev->channel[0].i2c_adapter;
	if (chan->fe->ops.i2c_gate_ctrl)
		chan->fe->ops.i2c_gate_ctrl(chan->fe, 1);
	fe = dvb_attach(tda18271c2dd_attach, chan->fe, i2c, 0x60);
	if (chan->fe->ops.i2c_gate_ctrl)
		chan->fe->ops.i2c_gate_ctrl(chan->fe, 0);
	if (!fe) {
		printk(KERN_ERR "No TDA18271 found!\n");
		return -ENODEV;
	}

	return 0;
}

static int tuner_attach_tda18212dd(struct ngene_channel *chan)
{
	struct i2c_adapter *i2c;
	struct dvb_frontend *fe;
	u8 addr = (chan->number & 1) ? 0x63 : 0x60;

	if (chan->demod_type == DMD_CXD28xx && chan->number < 2)
		addr ^= 0x04;
	i2c = &chan->dev->channel[0].i2c_adapter;
	fe = dvb_attach(tda18212dd_attach, chan->fe, i2c, addr);
	if (!fe) {
		printk(KERN_ERR "No TDA18212 found!\n");
		return -ENODEV;
	}
	return 0;
}

static int tuner_attach_probe(struct ngene_channel *chan)
{
	switch(chan->demod_type)
	{
	case DMD_STV0900:
		return tuner_attach_stv6110(chan);

	case DMD_DRXK:
		return tuner_attach_tda18271(chan);

	case DMD_STV0367:
	case DMD_CXD28xx:
		return tuner_attach_tda18212dd(chan);

	default:
		pr_err("Unknown demod %x\n", chan->demod_type);
		break;
	}
	return -EINVAL;
}

static int demod_attach_stv0900(struct ngene_channel *chan)
{
	struct i2c_adapter *i2c;
	struct stv090x_config *feconf = (struct stv090x_config *)
		chan->dev->card_info->fe_config[chan->number];

	/* tuner 1+2: i2c adapter #0, tuner 3+4: i2c adapter #1 */
	/* Note: Both adapters share the same i2c bus, but the demod     */
	/*       driver requires that each demod has its own i2c adapter */
	if (chan->number < 2)
		i2c = &chan->dev->channel[0].i2c_adapter;
	else
		i2c = &chan->dev->channel[1].i2c_adapter;

	chan->fe = dvb_attach(stv090x_attach, feconf, i2c,
			(chan->number & 1) == 0 ? STV090x_DEMODULATOR_0
						: STV090x_DEMODULATOR_1);
	if (chan->fe == NULL) {
		printk(KERN_ERR	DEVICE_NAME ": No STV0900 found!\n");
		return -ENODEV;
	}

	/* store channel info */
	if (feconf->tuner_i2c_lock)
		chan->fe->analog_demod_priv = chan;

	if (!dvb_attach(lnbh24_attach, chan->fe, i2c, 0,
			0, chan->dev->card_info->lnb[chan->number])) {
		printk(KERN_ERR DEVICE_NAME ": No LNBH24 found!\n");
		dvb_frontend_detach(chan->fe);
		chan->fe = NULL;
		return -ENODEV;
	}

	return 0;
}

static void cineS2_tuner_i2c_lock(struct dvb_frontend *fe, int lock)
{
	struct ngene_channel *chan = fe->analog_demod_priv;

	if (lock)
		down(&chan->dev->pll_mutex);
	else
		up(&chan->dev->pll_mutex);
}

static int i2c_read(struct i2c_adapter *adapter, u8 adr, u8 *val)
{
	struct i2c_msg msgs[1] = {{.addr = adr,  .flags = I2C_M_RD,
				   .buf  = val,  .len   = 1 } };
	return (i2c_transfer(adapter, msgs, 1) == 1) ? 0 : -1;
}

static int i2c_read_regs(struct i2c_adapter *adapter,
			 u8 adr, u8 reg, u8 *val, u8 len)
{
	struct i2c_msg msgs[2] = {{.addr = adr,  .flags = 0,
				   .buf  = &reg, .len   = 1 },
				  {.addr = adr,  .flags = I2C_M_RD,
				   .buf  = val,  .len   = len } };
	return (i2c_transfer(adapter, msgs, 2) == 2) ? 0 : -1;
}

static int i2c_read_reg(struct i2c_adapter *adapter, u8 adr,
			u8 reg, u8 *val)
{
	struct i2c_msg msgs[2] = {{.addr = adr, .flags = 0,
				   .buf  = &reg, .len  = 1},
				  {.addr = adr, .flags = I2C_M_RD,
				   .buf  = val, .len   = 1} };
	return (i2c_transfer(adapter, msgs, 2) == 2) ? 0 : -1;
}

static int i2c_read_reg16(struct i2c_adapter *adapter, u8 adr,
			  u16 reg, u8 *val)
{
	u8 msg[2] = {reg>>8, reg&0xff};
	struct i2c_msg msgs[2] = {{.addr = adr, .flags = 0,
				   .buf  = msg, .len   = 2},
				  {.addr = adr, .flags = I2C_M_RD,
				   .buf  = val, .len   = 1} };
	return (i2c_transfer(adapter, msgs, 2) == 2) ? 0 : -1;
}

static int i2c_write_reg(struct i2c_adapter *adapter, u8 adr,
			 u8 reg, u8 val)
{
	u8 msg[2] = {reg, val};
	struct i2c_msg msgs[1] = {{.addr = adr,  .flags = 0,
                                   .buf  = msg,  .len   = 2 } };
        return (i2c_transfer(adapter, msgs, 1) == 1) ? 0 : -1;
}

static int port_has_stv0900(struct i2c_adapter *i2c, int port)
{
	u8 val;
	if (i2c_read_reg16(i2c, 0x68+port/2, 0xf100, &val) < 0)
		return 0;
	return 1;
}

static int port_has_drxk(struct i2c_adapter *i2c, int port)
{
	u8 val;

	if (i2c_read(i2c, 0x29+port, &val) < 0)
		return 0;
	return 1;
}

static int demod_attach_drxk(struct ngene_channel *chan,
			     struct i2c_adapter *i2c)
{
	struct drxk_config config;

	memset(&config, 0, sizeof(config));
	config.adr = 0x29 + (chan->number ^ 2);
	config.microcode_name = "drxk_a3.mc";

	chan->fe = dvb_attach(drxk_attach, &config, i2c);
	if (!chan->fe) {
		printk(KERN_ERR "No DRXK found!\n");
		return -ENODEV;
	}
	chan->fe->sec_priv = chan;
	chan->gate_ctrl = chan->fe->ops.i2c_gate_ctrl;
	chan->fe->ops.i2c_gate_ctrl = locked_gate_ctrl;
	return 0;
}

static int port_has_stv0367(struct i2c_adapter *i2c, int port)
{
	u8 val;

	if (i2c_read_reg16(i2c, 0x1c + (port ^ 1), 0xf000, &val) < 0)
		return 0;
	if (val != 0x60)
		return 0;
	return 1;
}

static int demod_attach_stv0367dd(struct ngene_channel *chan,
				  struct i2c_adapter *i2c)
{
	struct stv0367_cfg cfg;

	memset(&cfg, 0, sizeof cfg);
	cfg.adr = 0x1c + (chan->number ^ 1);

	chan->fe = dvb_attach(stv0367_attach, i2c, &cfg, &chan->fe2);
	if (!chan->fe) {
		printk(KERN_ERR "No stv0367 found!\n");
		return -ENODEV;
	}
	chan->fe->sec_priv = chan;
	chan->gate_ctrl = chan->fe->ops.i2c_gate_ctrl;
	chan->fe->ops.i2c_gate_ctrl = locked_gate_ctrl;
	return 0;
}

static int port_has_xo2(struct i2c_adapter *i2c, int port, u8 *id)
{
	u8 addr = (port < 2) ? 0x14 : 0x10;
	u8 val;
	u8 probe[1] = { 0x00 }, data[4];
	struct i2c_msg msgs[2] = {{ .addr = addr,  .flags = 0,
				    .buf  = probe, .len   = 1 },
				  { .addr = addr,  .flags = I2C_M_RD,
				    .buf  = data,  .len   = 4 } };
	val = i2c_transfer(i2c, msgs, 2);
	if (val != 2)
		return 0;

	if (data[0] != 'D' || data[1] != 'F')
		return 0;

	*id = data[2];
	return 1;
}

static int init_xo2(struct i2c_adapter *i2c, int port)
{
	u8 addr = (port < 2) ? 0x14 : 0x10;
	u8 val, data[2];
	int res;

	if (port & 1)
		return 0;

	res = i2c_read_regs(i2c, addr, 0x04, data, 2);
	if (res < 0)
		return res;

	if (data[0] != 0x01)  {
		pr_info("Invalid XO2\n");
		return -1;
	}

	i2c_read_reg(i2c, addr, 0x08, &val);
	if (val != 0) {
		i2c_write_reg(i2c, addr, 0x08, 0x00);
		msleep(100);
	}
	/* Enable tuner power, disable pll, reset demods */
	i2c_write_reg(i2c, addr, 0x08, 0x04);
	usleep_range(2000, 3000);
	/* Release demod resets */
	i2c_write_reg(i2c, addr, 0x08, 0x07);
	usleep_range(2000, 3000);
	/* Start XO2 PLL */
	i2c_write_reg(i2c, addr, 0x08, 0x87);

	return 0;
}

#define DDB_TUNER_XO2		16
#define DDB_TUNER_DVBS_STV0910	16
#define DDB_TUNER_DVBCT2_SONY	17
#define DDB_TUNER_ISDBT_SONY	18
#define DDB_TUNER_DVBC2T2_SONY	19
#define DDB_TUNER_ATSC_ST	20
#define DDB_TUNER_DVBC2T2_ST	21

static char *xo2names[] = {
	"DUAL DVB-S2",
	"DUAL DVB-C/T/T2",
	"DUAL DVB-ISDBT",
	"DUAL DVB-C/C2/T/T2",
	"DUAL ATSC",
	"DUAL DVB-C/C2/T/T2",
	"", ""
};

struct cxd2843_cfg cxd2843[] = {
	{ .adr = 0x68, },
	{ .adr = 0x69, },
	{ .adr = 0x6c, },
	{ .adr = 0x6d, },
};

static int demod_attach_cxd2843(struct ngene_channel *chan,
				struct i2c_adapter *i2c)
{
	chan->fe = dvb_attach(cxd2843_attach, i2c, cxd2843+chan->number);
	if (!chan->fe) {
		pr_err("No cxd2837/38/43 found!\n");
		return -ENODEV;
	}
	chan->fe->sec_priv = chan;
	chan->gate_ctrl = chan->fe->ops.i2c_gate_ctrl;
	chan->fe->ops.i2c_gate_ctrl = locked_gate_ctrl;
	return 0;
}

static int cineS2_probe(struct ngene_channel *chan)
{
	struct i2c_adapter *i2c;
	struct stv090x_config *fe_conf;
	u8 buf[3];
	struct i2c_msg i2c_msg = { .flags = 0, .buf = buf };
	int rc;
	u8 id;

	/* tuner 1+2: i2c adapter #0, tuner 3+4: i2c adapter #1 */
	if (chan->number < 2)
		i2c = &chan->dev->channel[0].i2c_adapter;
	else
		i2c = &chan->dev->channel[1].i2c_adapter;

	if (port_has_xo2(i2c, chan->number, &id)) {
		id >>= 2;
		if (id > 5) {
			pr_info("Channel %d: Unknown XO2 DuoFlex %u\n",
				chan->number, id);
			return -ENODEV;
		}
		init_xo2(i2c, chan->number);
		switch(DDB_TUNER_XO2 + id)
		{
		case DDB_TUNER_DVBCT2_SONY:
		case DDB_TUNER_ISDBT_SONY:
		case DDB_TUNER_DVBC2T2_SONY:
			chan->demod_type = DMD_CXD28xx;
			pr_info("Channel %d: %s\n", chan->number, xo2names[id]);
			demod_attach_cxd2843(chan, i2c);		
			break;
		default:
			pr_info("Channel %d: %s not supported yet\n",
				chan->number, xo2names[id]);
			return -ENODEV;
		}

	} else if (chan->dev->channel[0].demod_type != DMD_CXD28xx &&
		   port_has_stv0900(i2c, chan->number)) {
		chan->demod_type = DMD_STV0900;
		pr_info("Channel %d: STV0900\n", chan->number);
		fe_conf = chan->dev->card_info->fe_config[chan->number];
		/* demod found, attach it */
		rc = demod_attach_stv0900(chan);
		if (rc < 0 || chan->number < 2)
			return rc;

		/* demod #2: reprogram outputs DPN1 & DPN2 */
		i2c_msg.addr = fe_conf->address;
		i2c_msg.len = 3;
		buf[0] = 0xf1;
		switch (chan->number) {
		case 2:
			buf[1] = 0x5c;
			buf[2] = 0xc2;
			break;
		case 3:
			buf[1] = 0x61;
			buf[2] = 0xcc;
			break;
		default:
			return -ENODEV;
		}
		rc = i2c_transfer(i2c, &i2c_msg, 1);
		if (rc != 1) {
			printk(KERN_ERR DEVICE_NAME ": could not setup DPNx\n");
			return -EIO;
		}

	} else if (port_has_drxk(i2c, chan->number^2)) {
		chan->demod_type = DMD_DRXK;
		pr_info("Channel %d: DRXK\n", chan->number);
		demod_attach_drxk(chan, i2c);

	} else if (port_has_stv0367(i2c, chan->number)) {
		chan->demod_type = DMD_STV0367;
		pr_info("Channel %d: STV0367\n", chan->number);
		demod_attach_stv0367dd(chan, i2c);

	} else {
		printk(KERN_ERR "No demod found on chan %d\n", chan->number);
		return -ENODEV;
	}
	return 0;
}


static struct lgdt330x_config aver_m780 = {
	.demod_address = 0xb2 >> 1,
	.demod_chip    = LGDT3303,
	.serial_mpeg   = 0x00, /* PARALLEL */
	.clock_polarity_flip = 1,
};

static struct mt2131_config m780_tunerconfig = {
	0xc0 >> 1
};

/* A single func to attach the demo and tuner, rather than
 * use two sep funcs like the current design mandates.
 */
static int demod_attach_lg330x(struct ngene_channel *chan)
{
	chan->fe = dvb_attach(lgdt330x_attach, &aver_m780, &chan->i2c_adapter);
	if (chan->fe == NULL) {
		printk(KERN_ERR	DEVICE_NAME ": No LGDT330x found!\n");
		return -ENODEV;
	}

	dvb_attach(mt2131_attach, chan->fe, &chan->i2c_adapter,
		   &m780_tunerconfig, 0);

	return (chan->fe) ? 0 : -ENODEV;
}

/****************************************************************************/
/* Switch control (I2C gates, etc.) *****************************************/
/****************************************************************************/

static struct stv090x_config fe_cineS2 = {
	.device         = STV0900,
	.demod_mode     = STV090x_DUAL,
	.clk_mode       = STV090x_CLK_EXT,

	.xtal           = 27000000,
	.address        = 0x68,

	.ts1_mode       = STV090x_TSMODE_SERIAL_PUNCTURED,
	.ts2_mode       = STV090x_TSMODE_SERIAL_PUNCTURED,

	.repeater_level = STV090x_RPTLEVEL_16,

	.adc1_range	= STV090x_ADC_1Vpp,
	.adc2_range	= STV090x_ADC_1Vpp,

	.diseqc_envelope_mode = true,

	.tuner_i2c_lock = cineS2_tuner_i2c_lock,
};

static struct stv090x_config fe_cineS2_2 = {
	.device         = STV0900,
	.demod_mode     = STV090x_DUAL,
	.clk_mode       = STV090x_CLK_EXT,

	.xtal           = 27000000,
	.address        = 0x69,

	.ts1_mode       = STV090x_TSMODE_SERIAL_PUNCTURED,
	.ts2_mode       = STV090x_TSMODE_SERIAL_PUNCTURED,

	.repeater_level = STV090x_RPTLEVEL_16,

	.adc1_range	= STV090x_ADC_1Vpp,
	.adc2_range	= STV090x_ADC_1Vpp,

	.diseqc_envelope_mode = true,

	.tuner_i2c_lock = cineS2_tuner_i2c_lock,
};

static struct stv6110x_config tuner_cineS2_0 = {
	.addr	= 0x60,
	.refclk	= 27000000,
	.clk_div = 1,
};

static struct stv6110x_config tuner_cineS2_1 = {
	.addr	= 0x63,
	.refclk	= 27000000,
	.clk_div = 1,
};

static struct ngene_info ngene_info_cineS2 = {
	.type		= NGENE_SIDEWINDER,
	.name		= "Linux4Media cineS2 DVB-S2 Twin Tuner",
	.io_type	= {NGENE_IO_TSIN, NGENE_IO_TSIN},
	.demod_attach	= {demod_attach_stv0900, demod_attach_stv0900},
	.tuner_attach	= {tuner_attach_stv6110, tuner_attach_stv6110},
	.fe_config	= {&fe_cineS2, &fe_cineS2},
	.tuner_config	= {&tuner_cineS2_0, &tuner_cineS2_1},
	.lnb		= {0x0b, 0x08},
	.tsf		= {3, 3},
	.fw_version	= 18,
	.msi_supported	= true,
};

static struct ngene_info ngene_info_satixS2 = {
	.type		= NGENE_SIDEWINDER,
	.name		= "Mystique SaTiX-S2 Dual",
	.io_type	= {NGENE_IO_TSIN, NGENE_IO_TSIN},
	.demod_attach	= {demod_attach_stv0900, demod_attach_stv0900},
	.tuner_attach	= {tuner_attach_stv6110, tuner_attach_stv6110},
	.fe_config	= {&fe_cineS2, &fe_cineS2},
	.tuner_config	= {&tuner_cineS2_0, &tuner_cineS2_1},
	.lnb		= {0x0b, 0x08},
	.tsf		= {3, 3},
	.fw_version	= 18,
	.msi_supported	= true,
};

static struct ngene_info ngene_info_satixS2v2 = {
	.type		= NGENE_SIDEWINDER,
	.name		= "Mystique SaTiX-S2 Dual (v2)",
	.io_type	= {NGENE_IO_TSIN, NGENE_IO_TSIN, NGENE_IO_TSIN, NGENE_IO_TSIN,
			   NGENE_IO_TSOUT},
	.demod_attach	= {demod_attach_stv0900, demod_attach_stv0900, cineS2_probe, cineS2_probe},
	.tuner_attach	= {tuner_attach_stv6110, tuner_attach_stv6110, tuner_attach_probe, tuner_attach_probe},
	.fe_config	= {&fe_cineS2, &fe_cineS2, &fe_cineS2_2, &fe_cineS2_2},
	.tuner_config	= {&tuner_cineS2_0, &tuner_cineS2_1, &tuner_cineS2_0, &tuner_cineS2_1},
	.lnb		= {0x0a, 0x08, 0x0b, 0x09},
	.tsf		= {3, 3},
	.fw_version	= 18,
	.msi_supported	= true,
};

static struct ngene_info ngene_info_cineS2v5 = {
	.type		= NGENE_SIDEWINDER,
	.name		= "Linux4Media cineS2 DVB-S2 Twin Tuner (v5)",
	.io_type	= {NGENE_IO_TSIN, NGENE_IO_TSIN, NGENE_IO_TSIN, NGENE_IO_TSIN,
			   NGENE_IO_TSOUT},
	.demod_attach	= {demod_attach_stv0900, demod_attach_stv0900, cineS2_probe, cineS2_probe},
	.tuner_attach	= {tuner_attach_stv6110, tuner_attach_stv6110, tuner_attach_probe, tuner_attach_probe},
	.fe_config	= {&fe_cineS2, &fe_cineS2, &fe_cineS2_2, &fe_cineS2_2},
	.tuner_config	= {&tuner_cineS2_0, &tuner_cineS2_1, &tuner_cineS2_0, &tuner_cineS2_1},
	.lnb		= {0x0a, 0x08, 0x0b, 0x09},
	.tsf		= {3, 3},
	.fw_version	= 18,
	.msi_supported	= true,
};


static struct ngene_info ngene_info_duoFlex = {
	.type           = NGENE_SIDEWINDER,
	.name           = "Digital Devices DuoFlex PCIe or miniPCIe",
	.io_type        = {NGENE_IO_TSIN, NGENE_IO_TSIN, NGENE_IO_TSIN, NGENE_IO_TSIN,
			   NGENE_IO_TSOUT},
	.demod_attach   = {cineS2_probe, cineS2_probe, cineS2_probe, cineS2_probe},
	.tuner_attach   = {tuner_attach_probe, tuner_attach_probe, tuner_attach_probe, tuner_attach_probe},
	.fe_config      = {&fe_cineS2, &fe_cineS2, &fe_cineS2_2, &fe_cineS2_2},
	.tuner_config   = {&tuner_cineS2_0, &tuner_cineS2_1, &tuner_cineS2_0, &tuner_cineS2_1},
	.lnb            = {0x0a, 0x08, 0x0b, 0x09},
	.tsf            = {3, 3},
	.fw_version     = 18,
	.msi_supported	= true,
};

static struct ngene_info ngene_info_m780 = {
	.type           = NGENE_APP,
	.name           = "Aver M780 ATSC/QAM-B",

	/* Channel 0 is analog, which is currently unsupported */
	.io_type        = { NGENE_IO_NONE, NGENE_IO_TSIN },
	.demod_attach   = { NULL, demod_attach_lg330x },

	/* Ensure these are NULL else the frame will call them (as funcs) */
	.tuner_attach   = { 0, 0, 0, 0 },
	.fe_config      = { NULL, &aver_m780 },
	.avf            = { 0 },

	/* A custom electrical interface config for the demod to bridge */
	.tsf		= { 4, 4 },
	.fw_version	= 15,
};

/****************************************************************************/
/* PCI Subsystem ID *********************************************************/
/****************************************************************************/

#define NGENE_ID(_subvend, _subdev, _driverdata) { \
	.vendor = NGENE_VID, .device = NGENE_PID, \
	.subvendor = _subvend, .subdevice = _subdev, \
	.driver_data = (unsigned long) &_driverdata }

/****************************************************************************/

static const struct pci_device_id ngene_id_tbl[] = {
	NGENE_ID(0x18c3, 0xab04, ngene_info_cineS2),
	NGENE_ID(0x18c3, 0xab05, ngene_info_cineS2v5),
	NGENE_ID(0x18c3, 0xabc3, ngene_info_cineS2),
	NGENE_ID(0x18c3, 0xabc4, ngene_info_cineS2),
	NGENE_ID(0x18c3, 0xdb01, ngene_info_satixS2),
	NGENE_ID(0x18c3, 0xdb02, ngene_info_satixS2v2),
	NGENE_ID(0x18c3, 0xdd00, ngene_info_cineS2v5),
	NGENE_ID(0x18c3, 0xdd10, ngene_info_duoFlex),
	NGENE_ID(0x18c3, 0xdd20, ngene_info_duoFlex),
	NGENE_ID(0x1461, 0x062e, ngene_info_m780),
	{0}
};
MODULE_DEVICE_TABLE(pci, ngene_id_tbl);

/****************************************************************************/
/* Init/Exit ****************************************************************/
/****************************************************************************/

static pci_ers_result_t ngene_error_detected(struct pci_dev *dev,
					     enum pci_channel_state state)
{
	printk(KERN_ERR DEVICE_NAME ": PCI error\n");
	if (state == pci_channel_io_perm_failure)
		return PCI_ERS_RESULT_DISCONNECT;
	if (state == pci_channel_io_frozen)
		return PCI_ERS_RESULT_NEED_RESET;
	return PCI_ERS_RESULT_CAN_RECOVER;
}

static pci_ers_result_t ngene_slot_reset(struct pci_dev *dev)
{
	printk(KERN_INFO DEVICE_NAME ": slot reset\n");
	return 0;
}

static void ngene_resume(struct pci_dev *dev)
{
	printk(KERN_INFO DEVICE_NAME ": resume\n");
}

static struct pci_error_handlers ngene_errors = {
	.error_detected = ngene_error_detected,
	.slot_reset = ngene_slot_reset,
	.resume = ngene_resume,
};

static struct pci_driver ngene_pci_driver = {
	.name        = "ngene",
	.id_table    = ngene_id_tbl,
	.probe       = ngene_probe,
	.remove      = ngene_remove,
	.err_handler = &ngene_errors,
	.shutdown    = ngene_shutdown,
};

static __init int module_init_ngene(void)
{
	printk(KERN_INFO
	       "nGene PCIE bridge driver, Copyright (C) 2005-2007 Micronas\n");
	return pci_register_driver(&ngene_pci_driver);
}

static __exit void module_exit_ngene(void)
{
	pci_unregister_driver(&ngene_pci_driver);
}

module_init(module_init_ngene);
module_exit(module_exit_ngene);

MODULE_DESCRIPTION("nGene");
MODULE_AUTHOR("Micronas, Ralph Metzler, Manfred Voelkel");
MODULE_LICENSE("GPL");
