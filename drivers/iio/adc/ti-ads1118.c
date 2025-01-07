// SPDX-License-Identifier: GPL-2.0-only
/*
 * iio/adc/ti-ads1158.c
 * Copyright (C) 2025 Spacetek Technology AG
 *
 * ads1158.c
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/spi/spi.h>
#include <linux/regmap.h>
#include <linux/units.h>

#include <linux/iio/iio.h>
#include <linux/iio/buffer.h>
#include <linux/regulator/consumer.h>
#include <linux/iio/sysfs.h>

/* Registers. */
enum ads1118_regs {
	REG_CONVERSION,
	REG_CONFIG,
};

/* Register fields. */
enum ads1118_regfields {
	/* Single-shot conversion start. */
	REGF_SS,
	/* Input multiplexer configuration. */
	REGF_MUX,
	/* Programmable gain amplifier configuration. */
	REGF_PGA,
	/* Devcie operation mode. */
	REGF_MODE,
	/* Data rate. */
	REGF_DR,
	/* Temperature sensor mode. */
	REGF_TS_MODE,
	/* Pullup enable. */
	REGF_PULL_UP_EN,
	REGF_MAX,
};

static const struct reg_field ads1118_regfields[] = {
	[REGF_SS] = REG_FIELD(REG_CONFIG, 15, 15),
	[REGF_MUX] = REG_FIELD(REG_CONFIG, 12, 14),
	[REGF_PGA] = REG_FIELD(REG_CONFIG, 9, 11),
	[REGF_MODE] = REG_FIELD(REG_CONFIG, 8, 8),
	[REGF_DR] = REG_FIELD(REG_CONFIG, 5, 7),
	[REGF_PULL_UP_EN] = REG_FIELD(REG_CONFIG, 3, 3),
};

enum ads1118_nop {
	NOP_INVALID_DATA_DO_NOT_UPDATE_CONFIG,
	NOP_VALID_DATA_UPDATE_CONFIG,
};

#define NOP_MASK GENMASK(2, 1)

enum ads1118_pga {
	PGA_PLUS_MINUS_6144_MV,
	PGA_PLUS_MINUS_4096_MV,
	PGA_PLUS_MINUS_2048_MV,
	PGA_PLUS_MINUS_1024_MV,
	PGA_PLUS_MINUS_512_MV,
	PGA_PLUS_MINUS_256_MV,
};

static struct reg_default ads1118_reg_defaults[] = {
	{ REG_CONFIG, 0x058b },
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
	regmap_reg_range(REG_CONVERSION, REG_CONVERSION),
};

static const struct regmap_access_table ads1118_reg_volatile_tbl = {
	.yes_ranges = ads1118_volatile_ranges,
	.n_yes_ranges = ARRAY_SIZE(ads1118_volatile_ranges),
};

static int ads1118_reg_write(void *context, unsigned int reg, unsigned int val)
{
	struct spi_device *spi = context;
	int ret;

	switch (reg) {
	case REG_CONFIG:
		val &= cpu_to_be16(~(__u16)NOP_MASK);
		val |= cpu_to_be16(
			FIELD_PREP(NOP_MASK, NOP_VALID_DATA_UPDATE_CONFIG));
		ret = spi_write(spi, &val, 2);
		break;
	default:
		return -ERANGE;
	}

	return ret;
}

static int ads1118_reg_read(void *context, unsigned int reg, unsigned int *val)
{
	struct spi_device *spi = context;
	u16 buf = 0;
	int ret;

	switch (reg) {
	case REG_CONFIG:
		/* Ignore data, readback config. */
		ret = spi_write_then_read(spi, &buf, 2, val, 2);
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
	.reg_write = ads1118_reg_write,
	.reg_defaults = ads1118_reg_defaults,
	.num_reg_defaults = ARRAY_SIZE(ads1118_reg_defaults),
	.cache_type = REGCACHE_FLAT,
	.use_single_read = true,
	.use_single_write = true,
	.val_format_endian = REGMAP_ENDIAN_BIG,
	.can_sleep = true,
};

struct ads1118_state {
	struct spi_device *spi;
	struct regmap *regmap;
	struct regmap_field *regfields[REGF_MAX];
};

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
	.debugfs_reg_access = ads1118_debugfs_reg_access,
};

static int ads1118_probe(struct spi_device *spi)
{
	const struct spi_device_id *id = spi_get_device_id(spi);
	const struct device_node *dn = dev_of_node(&spi->dev);
	struct iio_dev *indio_dev;
	struct ads1118_state *st;
	size_t i;
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

	st->regmap = devm_regmap_init_spi(spi, &ads1118_regmap_config);
	if (IS_ERR(st->regmap))
		return dev_err_probe(&spi->dev, PTR_ERR(st->regmap),
				     "Could not initialise regmap\n");

	for (i = 0; i < REGF_MAX; i++) {
		st->regfields[i] = devm_regmap_field_alloc(
				indio_dev->dev.parent, st->regmap,
				ads1118_regfields[i]);
		if (IS_ERR(st->regfields[i]))
			return dev_err_probe(
					indio_dev->dev.parent,
					PTR_ERR(st->regfields[i]),
					"Could not allocate register field %zu\n", i);
	}

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
MODULE_DESCRIPTION("ADS1158 ADC");
MODULE_LICENSE("GPL v2");
