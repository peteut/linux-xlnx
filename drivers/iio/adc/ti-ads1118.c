// SPDX-License-Identifier: GPL-2.0-only
/*
 * iio/adc/ti-ads1118.c
 * Copyright (C) 2025 Spacetek Technology AG
 *
 * ads1118.c
 *
 */

#include "linux/iio/types.h"
#include <linux/bitfield.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/spi/spi.h>
#include <linux/regmap.h>
#include <linux/units.h>
#include <linux/args.h>

#include <linux/iio/iio.h>
#include <linux/iio/buffer.h>
#include <linux/regulator/consumer.h>
#include <linux/iio/sysfs.h>

/* Registers. */
enum ads1118_regs {
	REG_CONVERSION,
	REG_CONFIG,
};

/* Single-shot conversion start. */
#define SS_ENABLE 15
/* Input multiplexer configuration. */
#define MUX_MASK GENMASK(14, 12)
/* Programmable gain amplifier configuration. */
#define PGA_MASK GENMASK(11, 9)
/* Device operation mode. */
#define MODE_MASK GENMASK(8, 8)
enum { MODE_CONT, MODE_PWR_DOWN_AND_SS };
/* Data rate. */
#define DR_MASK GENMASK(7, 5)
/* Temperature sensor mode. */
#define TS_MODE 4
/* Pullup enable. */
#define PULL_UP_EN 3

enum ads1118_pga {
	PGA_PLUS_MINUS_6144_MV,
	PGA_PLUS_MINUS_4096_MV,
	PGA_PLUS_MINUS_2048_MV,
	PGA_PLUS_MINUS_1024_MV,
	PGA_PLUS_MINUS_512_MV,
	PGA_PLUS_MINUS_256_MV,
	PGA_MAX
};
#define PGA_DEFAULT PGA_PLUS_MINUS_2048_MV

static const unsigned int ads1118_pga_fsr_mv[] = {
	[PGA_PLUS_MINUS_6144_MV] = 6144, [PGA_PLUS_MINUS_4096_MV] = 4096,
	[PGA_PLUS_MINUS_2048_MV] = 2048, [PGA_PLUS_MINUS_1024_MV] = 1024,
	[PGA_PLUS_MINUS_512_MV] = 512,	 [PGA_PLUS_MINUS_256_MV] = 256
};
static_assert(ARRAY_SIZE(ads1118_pga_fsr_mv) == PGA_MAX);

// #define MODE_MASK GENMASK(8, 8)

// static const char *ads1118_mode_label[MODE_MAX] = {
// 	[MODE_CONT] = "continuous conversion mode",
// 	[MODE_PWR_DOWN] = "power-down and single-shot mode"
// };

enum ads1118_dr {
	DR_8_SPS,
	DR_16_SPS,
	DR_32_SPS,
	DR_64_SPS,
	DR_128_SPS,
	DR_250_SPS,
	DR_475_SPS,
	DR_860_SPS,
	DR_MAX
};
static_assert(DR_MAX == 8);
#define DR_DEFAULT DR_128_SPS

static const unsigned int ads1118_dr_sps[] = {
	[DR_8_SPS] = 8,	    [DR_16_SPS] = 16,	[DR_32_SPS] = 32,
	[DR_64_SPS] = 64,   [DR_128_SPS] = 128, [DR_250_SPS] = 250,
	[DR_475_SPS] = 475, [DR_860_SPS] = 860
};
static_assert(ARRAY_SIZE(ads1118_dr_sps) == DR_MAX);

enum ads1118_nop {
	NOP_INVALID_DATA,
	NOP_VALID_DATA,
};
#define NOP_MASK GENMASK(2, 1)

#define RESERVED 0

#define REG_CONFIG_RESET                                           \
	(FIELD_PREP_CONST(PGA_MASK, PGA_PLUS_MINUS_2048_MV) |      \
	 FIELD_PREP_CONST(MODE_MASK, MODE_PWR_DOWN_AND_SS) |       \
	 FIELD_PREP_CONST(DR_MASK, DR_128_SPS) | BIT(PULL_UP_EN) | \
	 FIELD_PREP_CONST(NOP_MASK, NOP_VALID_DATA) | BIT(RESERVED))
static_assert(REG_CONFIG_RESET == 0x058b);

static struct reg_default ads1118_reg_defaults[] = {
	{ REG_CONFIG, REG_CONFIG_RESET },
};

static const struct regmap_range ads1118_write_ranges[] = {
	regmap_reg_range(REG_CONFIG, REG_CONFIG),
};

static const struct regmap_access_table ads1118_reg_write_tbl = {
	.yes_ranges = ads1118_write_ranges,
	.n_yes_ranges = ARRAY_SIZE(ads1118_write_ranges),
};

static const struct regmap_range ads1118_read_ranges[] = {
	regmap_reg_range(REG_CONVERSION, REG_CONFIG),
};

static const struct regmap_access_table ads1118_reg_read_tbl = {
	.yes_ranges = ads1118_read_ranges,
	.n_yes_ranges = ARRAY_SIZE(ads1118_read_ranges),
};

static const struct regmap_range ads1118_volatile_ranges[] = {
	regmap_reg_range(REG_CONVERSION, REG_CONFIG),
};

static const struct regmap_access_table ads1118_reg_volatile_tbl = {
	.yes_ranges = ads1118_volatile_ranges,
	.n_yes_ranges = ARRAY_SIZE(ads1118_volatile_ranges),
};

static int ads1118_reg_read(void *context, unsigned int reg, unsigned int *val)
{
	struct spi_device *spi = context;
	unsigned int cmd = 0;
	int ret;

	switch (reg) {
	case REG_CONFIG:
		/* Ignore data, readback config. */
		ret = spi_write_then_read(spi, &cmd, 2, val, 2);
		break;
	case REG_CONVERSION:
		/* 16-bit data transmission cycle. */
		ret = spi_read(spi, val, 2);
		break;
	default:
		return -ERANGE;
	}

	return ret;
}

static const struct regmap_config ads1118_regmap_config = {
	.reg_bits = 8,
	.val_bits = 16,
	.max_register = REG_CONFIG,
	.wr_table = &ads1118_reg_write_tbl,
	.rd_table = &ads1118_reg_read_tbl,
	.volatile_table = &ads1118_reg_volatile_tbl,
	.reg_read = ads1118_reg_read,
	.write_flag_mask = FIELD_PREP_CONST(NOP_MASK, NOP_VALID_DATA),
	.reg_defaults = ads1118_reg_defaults,
	.num_reg_defaults = ARRAY_SIZE(ads1118_reg_defaults),
	.use_single_read = true,
	.use_single_write = true,
	.val_format_endian = REGMAP_ENDIAN_BIG,
	.can_sleep = true,
};

enum ads1118_scan_si {
	SI_AIN0_VS_AIN1,
	SI_AIN0_VS_AIN3,
	SI_AIN1_VS_AIN3,
	SI_AIN2_VS_AIN3,
	SI_AIN0,
	SI_AIN1,
	SI_AIN2,
	SI_AIN3,
	SI_MAX
};
static_assert(SI_MAX == 8);

#define ADS1118_VOLTAGE_CHAN_IIO(_chan, _si) \
	{ \
		.type = IIO_VOLTAGE, \
		.indexed = 1, \
		.channel = _chan, \
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) \
				      | BIT(IIO_CHAN_INFO_SCALE) \
				      | BIT(IIO_CHAN_INFO_SAMP_FREQ), \
		.scan_index = _si, \
		.scan_type = { \
			.sign = 's', \
			.realbits = 16, \
			.storagebits = 16, \
			.endianness = IIO_BE, \
		}, \
		.datasheet_name = __stringify(CONCATENATE(AIN, _chan)), \
}

#define ADS1118_VOLTAGE_CHAN_DIFF_IIO(_chan_p, _chan_n, _si) \
	{ \
		.type = IIO_VOLTAGE, \
		 .channel = _chan_p, \
		.channel2 = _chan_n, \
		.differential = 1, \
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) \
				      | BIT(IIO_CHAN_INFO_SCALE) \
				      | BIT(IIO_CHAN_INFO_SAMP_FREQ), \
		.scan_index = _si, \
		.scan_type = { \
			.sign = 's', \
			.realbits = 16, \
			.storagebits = 16, \
			.endianness = IIO_BE, \
		}, \
		.datasheet_name = __stringify( \
			CONCATENATE( \
				CONCATENATE(CONCATENATE(AIN, _chan_p), vs), \
				CONCATENATE(AIN, _chan_n) \
			) \
		), \
}

static const struct iio_chan_spec ads1118_channels[] = {
	ADS1118_VOLTAGE_CHAN_DIFF_IIO(0, 1, SI_AIN0_VS_AIN1),
	ADS1118_VOLTAGE_CHAN_DIFF_IIO(0, 3, SI_AIN0_VS_AIN3),
	ADS1118_VOLTAGE_CHAN_DIFF_IIO(1, 3, SI_AIN1_VS_AIN3),
	ADS1118_VOLTAGE_CHAN_DIFF_IIO(2, 3, SI_AIN2_VS_AIN3),
	ADS1118_VOLTAGE_CHAN_IIO(0, SI_AIN0),
	ADS1118_VOLTAGE_CHAN_IIO(1, SI_AIN1),
	ADS1118_VOLTAGE_CHAN_IIO(2, SI_AIN2),
	ADS1118_VOLTAGE_CHAN_IIO(3, SI_AIN3),
	{
		.type = IIO_TEMP,
		.modified = 1,
		.channel2 = IIO_MOD_TEMP_OBJECT,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				      BIT(IIO_CHAN_INFO_SCALE) |
				      BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.scan_type = { .sign = 's',
			       .realbits = 16,
			       .storagebits = 16,
			       .endianness = IIO_BE },
		.datasheet_name = "TS",
	}
};
static_assert(ARRAY_SIZE(ads1118_channels) == SI_MAX + 1);

struct ads1118_state {
	struct spi_device *spi;
	struct regmap *regmap;
	struct mutex lock;
	struct {
		const char *label;
		unsigned int pga;
		enum ads1118_dr data_rate;
	} channel_data[SI_MAX + 1];
};

static int ads1118_read_avail(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *chan, const int **val,
			      int *type, int *length, long mask)
{
	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		*type = IIO_VAL_INT;
		*val = ads1118_dr_sps;
		*length = ARRAY_SIZE(ads1118_dr_sps);
		return IIO_AVAIL_LIST;
	default:
		return -EINVAL;
	}
}

static int ads1118_read_single_value(struct iio_dev *indio_dev,
				     struct iio_chan_spec const *chan, int *val)
{
	struct ads1118_state *st = iio_priv(indio_dev);
	unsigned int cfg;
	unsigned int conversion_time_us;
	int ret;

	ret = iio_device_claim_direct_mode(indio_dev);
	if (!ret)
		return ret;

	if (chan->type == IIO_TEMP) {
	}
	switch (chan->type) {
	case IIO_TEMP:
		cfg = BIT(TS_MODE) | BIT(SS_ENABLE) |
		      FIELD_PREP_CONST(MODE_MASK, MODE_PWR_DOWN_AND_SS) |
		      FIELD_PREP(DR_MASK, st->channel_data[SI_MAX].data_rate) |
		      FIELD_PREP_CONST(NOP_MASK, NOP_VALID_DATA);
		ret = regmap_write(st->regmap, REG_CONFIG, cfg);
		if (!ret)
			goto err;

		conversion_time_us = DIV_ROUND_UP(
			MEGA,
			ads1118_dr_sps[st->channel_data[SI_MAX].data_rate]);
		usleep_range(conversion_time_us, conversion_time_us * 2);
		break;
	case IIO_VOLTAGE:
		cfg = BIT(SS_ENABLE) | FIELD_PREP(MUX_MASK, chan->scan_index) |
		      FIELD_PREP_CONST(MODE_MASK, MODE_PWR_DOWN_AND_SS) |
		      FIELD_PREP(DR_MASK,
				 st->channel_data[chan->scan_index].data_rate) |
		      FIELD_PREP(PGA_MASK,
				 st->channel_data[chan->scan_index].pga) |
		      FIELD_PREP_CONST(NOP_MASK, NOP_VALID_DATA);

		conversion_time_us = DIV_ROUND_UP(
			MEGA, ads1118_dr_sps[st->channel_data[chan->scan_index]
						     .data_rate]);
		break;
	default:
		ret = -EINVAL;
		goto err;
	}

	usleep_range(conversion_time_us, conversion_time_us * 2);
	ret = regmap_read(st->regmap, REG_CONVERSION, val);
err:
	iio_device_release_direct_mode(indio_dev);

	return ret;
}

static int ads1118_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan, int *val,
			    int *val2, long mask)
{
	struct ads1118_state *st = iio_priv(indio_dev);
	int ret;

	mutex_lock(&st->lock);
	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		ret = ads1118_read_single_value(indio_dev, chan, val);
		break;
	case IIO_CHAN_INFO_SAMP_FREQ:
		switch (chan->type) {
		case IIO_TEMP:
			*val = ads1118_dr_sps[DR_DEFAULT];
			ret = IIO_VAL_INT;
			break;
		case IIO_VOLTAGE:
			*val = ads1118_dr_sps[st->channel_data[chan->scan_index]
						      .data_rate];
			ret = IIO_VAL_INT;
			break;
		default:
			ret = -EINVAL;
		}
	default:
		ret = -EINVAL;
	}
	mutex_unlock(&st->lock);

	return ret;
}

static int ads1118_read_label(struct iio_dev *indio_dev,
			      const struct iio_chan_spec *chan, char *label)
{
	struct ads1118_state *st = iio_priv(indio_dev);
	char *name, *p;
	int ret;

	if (st->channel_data[chan->scan_index].label)
		return sysfs_emit(label, "%s\n",
				  st->channel_data[chan->scan_index].label);

	name = kstrdup(chan->datasheet_name, GFP_KERNEL);
	if (!name)
		return -ENOMEM;

	for (p = name; *p != '\0'; p++)
		*p = tolower(*p);

	ret = sysfs_emit(label, "%s\n", name);
	kfree(name);

	return ret;
}

int ads1118_debugfs_reg_access(struct iio_dev *indio_dev, unsigned reg,
			       unsigned writeval, unsigned *readval)
{
	struct ads1118_state *st = iio_priv(indio_dev);
	int ret;

	if (!readval)
		ret = regmap_write(st->regmap, reg, writeval);
	else
		ret = regmap_read(st->regmap, reg, readval);

	return ret;
}

static const struct iio_info ads1118_info = {
	.read_avail = ads1118_read_avail,
	.read_raw = ads1118_read_raw,
	.read_label = ads1118_read_label,
	.debugfs_reg_access = ads1118_debugfs_reg_access,
};

static int ads1118_chan_init(struct iio_dev *indio_dev)
{
	struct ads1118_state *st = iio_priv(indio_dev);
	unsigned int num_channels;
	struct fwnode_handle *child;
	struct iio_chan_spec *channels;
	size_t channel_i;

	num_channels = device_get_child_node_count(indio_dev->dev.parent);
	if (!num_channels) {
		dev_err(indio_dev->dev.parent, "No channel found\n");
	}
	/* Allocate for temp sensor too. */
	channels = devm_kcalloc(indio_dev->dev.parent, num_channels + 1,
				sizeof(struct iio_chan_spec), GFP_KERNEL);
	if (!channels)
		return -ENOMEM;

	channel_i = 0;
	device_for_each_child_node(indio_dev->dev.parent, child) {
		u32 val;
		const char *name;
		unsigned int reg;
		enum ads1118_pga pga = PGA_DEFAULT;
		enum ads1118_dr data_rate = DR_DEFAULT;
		size_t i;

		if (!fwnode_property_read_u32(child, "reg", &val)) {
			dev_err(indio_dev->dev.parent, "Invalid reg on %pfw\n",
				child);
			continue;
		}

		reg = val;
		if (reg >= ARRAY_SIZE(st->channel_data)) {
			dev_err(indio_dev->dev.parent,
				"Invalid channel index %d on %pfw\n", reg,
				child);
			continue;
		}

		/* label is optional */
		st->channel_data[reg].label =
			(!fwnode_property_read_string(child, "label", &name)) ?
				name :
				NULL;

		if (!fwnode_property_read_u32(child, "ti,gain", &val))
			pga = val;
		if (pga >= PGA_MAX) {
			dev_err(indio_dev->dev.parent, "Invalid gain on %pfw\n",
				child);
			fwnode_handle_put(child);
			return -EINVAL;
		}

		if (!fwnode_property_read_u32(child, "ti,datarate", &val)) {
			data_rate = val;
			if (data_rate >= DR_MAX) {
				dev_err(indio_dev->dev.parent,
					"Invalid data rate on %pfw\n", child);
				fwnode_handle_put(child);
				return -EINVAL;
			}
		}

		st->channel_data[reg].pga = pga;
		st->channel_data[reg].data_rate = data_rate;

		for (i = 0; i < ARRAY_SIZE(ads1118_channels); i++) {
			if (ads1118_channels[i].scan_index == reg)
				break;
		}

		memcpy(&channels[channel_i], &ads1118_channels[i],
		       sizeof(struct iio_chan_spec));

		channel_i++;
	}

	/* Append temperature sensor. */
	memcpy(&channels[channel_i], &ads1118_channels[SI_MAX],
	       sizeof(struct iio_chan_spec));
	channel_i++;
	st->channel_data[SI_MAX].data_rate = DR_DEFAULT;
	st->channel_data[SI_MAX].label = NULL;

	indio_dev->num_channels = channel_i;
	indio_dev->channels = channels;

	return 0;
}

static int ads1118_probe(struct spi_device *spi)
{
	const struct spi_device_id *id = spi_get_device_id(spi);
	struct iio_dev *indio_dev;
	struct ads1118_state *st;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev)
		return dev_err_probe(&spi->dev, -ENOMEM,
				     "Could not allocate iio device\n");

	st = iio_priv(indio_dev);
	st->spi = spi;

	indio_dev->name = id->name;
	iio_device_set_parent(indio_dev, &spi->dev);
	indio_dev->info = &ads1118_info;
	indio_dev->modes = INDIO_DIRECT_MODE;

	st->regmap = devm_regmap_init_spi(spi, &ads1118_regmap_config);
	if (IS_ERR(st->regmap))
		return dev_err_probe(&spi->dev, PTR_ERR(st->regmap),
				     "Could not initialise regmap\n");

	mutex_init(&st->lock);

	ret = ads1118_chan_init(indio_dev);
	if (ret)
		return dev_err_probe(indio_dev->dev.parent, ret,
				     "Could not initialise channels\n");

	return devm_iio_device_register(&spi->dev, indio_dev);
}

static const struct of_device_id ads1118_of_match[] = {
	{ .compatible = "ti,ads1118" },
	{},
};
MODULE_DEVICE_TABLE(of, ads1118_of_match);

static const struct spi_device_id ads1118_id[] = {
	{ .name = "ads1118" },
	{},
};
MODULE_DEVICE_TABLE(spi, ads1118_id);

static struct spi_driver ads1118_driver = {
	.driver = {
		.name = KBUILD_MODNAME,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(ads1118_of_match),
	},
	.probe = ads1118_probe,
	.id_table = ads1118_id,
};
module_spi_driver(ads1118_driver);

MODULE_AUTHOR("Alain Péteut <alain.peteut@spacetek.ch>");
MODULE_DESCRIPTION("ADS1118 ADC");
MODULE_LICENSE("GPL v2");
