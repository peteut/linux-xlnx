// SPDX-License-Identifier: GPL-2.0-only
/*
 * iio/adc/ti-ads1158.c
 * Copyright (C) 2024 Alain Péteut
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

/* Command byte */
enum ads1158_cmd {
	CHAN_DATA_DIRECT,
	CHAN_DATA_READ_CMD,
	REG_READ_CMD,
	REG_WRITE_CMD,
	PULSE_CONV_CMD,
	RESET_CMD = 0x6,
};
#define CMD_CMD_MASK GENMASK(7, 5)
#define CMD_MUL 4
#define CMD_ADDR_MASK GENMASK(3, 0)

/* Status byte. */
#define STATUS_NEW 7
#define STATUS_OVF 6
#define STATUS_SUPPLY 5
#define STATUS_CHID_MASK GENMASK(4, 0)

/* MUXSCH */
#define MUXSCH_AINP_MASK GENMASK(7, 4)
#define MUXSCH_AINN_MASK GENMASK(3, 0)

/* Temperature sensor coefficient. */
#define TEMP_COEFF_UV 563
/* Temperature sensor offset [uV]. */
#define TEMP_OFFSET_UV 168000
/* Temperature sensor offset [°C]. */
#define TEMP_OFFSET_DEGREE 25

/* Registers. */
enum ads1158_regs {
	REG_CONFIG_0 = 0x00,
	REG_CONFIG_1,
	REG_MUXSCH,
	REG_MUXDIF,
	REG_MUXSG_0,
	REG_MUXSG_1,
	REG_SYSRED,
	REG_GPIOC,
	REG_GPIOD,
	REG_ID,
	REG_MAX,
};

/* Register fields. */
enum ads1158_regfield {
	/* Mux mode. */
	REGF_MUXMOD,
	/* Clock output enable on CLKIO. */
	REGF_CLKENB,
	/* Delay after indexing. */
	REGF_DLY,
	/* Data rate. */
	REGF_DR,
	/* ID bit. */
	REGF_IS_ADS1158,
	REGF_MAX,
};

static const struct reg_field ads1158_regfields[] = {
	[REGF_MUXMOD] = REG_FIELD(REG_CONFIG_0, 5, 5),
	[REGF_CLKENB] = REG_FIELD(REG_CONFIG_0, 3, 3),
	[REGF_DLY] = REG_FIELD(REG_CONFIG_1, 4, 6),
	[REGF_DR] = REG_FIELD(REG_CONFIG_1, 0, 1),
	[REGF_IS_ADS1158] = REG_FIELD(REG_ID, 4, 4),
};

static const int ads1158_data_rate_average[] = { 64, 16, 4, 1 };

enum ads1158_scan_mode {
	SCAN_AUTO,
	SCAN_FIXED,
	SCAN_MAX,
};

/* SPS for a clock frequency of 16 MHz. */
static const int ads1158_data_rate[SCAN_MAX][4] = {
	[SCAN_AUTO] = { 1831, 6168, 15123, 23739 },
	[SCAN_FIXED] = { 1953, 7813, 31250, 125000 },
};

/* Channel switching delay for a clock frequency of 16 MHz */
static const int ads1158_delay_us[] = { 0, 8, 16, 32, 64, 128, 256, 384 };

static bool ads1158_volatile_register(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case REG_GPIOD:
	case REG_ID:
		return true;
	default:
		return false;
	}
}

static bool ads1158_readable_register(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case REG_CONFIG_0 ... REG_ID:
		return true;
	default:
		return false;
	}
}

static struct reg_default ads1158_reg_default[] = {
	{ REG_CONFIG_0, 0x0a }, { REG_CONFIG_1, 0x83 }, { REG_MUXSCH, 0x00 },
	{ REG_MUXDIF, 0x00 },	{ REG_MUXSG_0, 0xff },	{ REG_MUXSG_1, 0xff },
	{ REG_SYSRED, 0x00 },	{ REG_GPIOC, 0xff },	{ REG_GPIOD, 0x00 },
};

static const struct regmap_config ads1158_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.volatile_reg = ads1158_volatile_register,
	.max_register = REG_MAX - 1,
	.readable_reg = ads1158_readable_register,
	.read_flag_mask = FIELD_PREP_CONST(CMD_CMD_MASK, REG_READ_CMD),
	.write_flag_mask = FIELD_PREP_CONST(CMD_CMD_MASK, REG_WRITE_CMD),
	.use_single_read =
		true, /* H/W supports it, requires MUL flag to be set. */
	.use_single_write =
		true, /* H/W supports it, requires MUL flag to be set. */
	.reg_defaults = ads1158_reg_default,
	.num_reg_defaults = ARRAY_SIZE(ads1158_reg_default),
	.cache_type = REGCACHE_FLAT,
	.can_sleep = true,
};

#define ADS1158_VOLTAGE_CHAN_IIO(_chan, _si, _name) \
	{ \
		.type = IIO_VOLTAGE, \
		.indexed = 1, \
		.channel = _chan, \
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) \
				    | BIT(IIO_CHAN_INFO_SCALE) \
				    | BIT(IIO_CHAN_INFO_SAMP_FREQ) \
				    | BIT(IIO_CHAN_INFO_HARDWAREGAIN) \
				    | BIT(IIO_CHAN_INFO_CALIBSCALE), \
		.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_AVERAGE_RAW), \
		.info_mask_separate_available = BIT(IIO_CHAN_INFO_SAMP_FREQ), \
		.info_mask_shared_by_all_available = BIT(IIO_CHAN_INFO_AVERAGE_RAW), \
		.scan_index = _si, \
		.scan_type = { \
			.sign = 's', \
			.realbits = 16, \
			.storagebits = 16, \
			.endianness = IIO_BE, \
		}, \
		.datasheet_name = _name, \
	}

#define ADS1158_VOLTAGE_CHAN_DIFF_IIO(_chan_p, _chan_n, _si, _name) \
	{ \
		.type = IIO_VOLTAGE, \
		.indexed = 1, \
		.channel = _chan_p, \
		.channel2 = _chan_n, \
		.differential = 1, \
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) \
				    | BIT(IIO_CHAN_INFO_SCALE) \
				    | BIT(IIO_CHAN_INFO_SAMP_FREQ) \
				    | BIT(IIO_CHAN_INFO_HARDWAREGAIN) \
				    | BIT(IIO_CHAN_INFO_CALIBSCALE), \
		.info_mask_shared_by_all =  BIT(IIO_CHAN_INFO_AVERAGE_RAW), \
		.info_mask_separate_available = BIT(IIO_CHAN_INFO_SAMP_FREQ), \
		.info_mask_shared_by_all_available = BIT(IIO_CHAN_INFO_AVERAGE_RAW), \
		.scan_index = _si, \
		.scan_type = { \
			.sign = 's', \
			.realbits = 16, \
			.storagebits = 16, \
			.endianness = IIO_BE, \
		}, \
		.datasheet_name = _name, \
	}

#define ADS1158_TEMP_CHAN_IIO(_si, _name) \
	{ \
		.type = IIO_TEMP, \
		.modified = 1, \
		.channel2 = IIO_MOD_TEMP_OBJECT, \
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) \
				    | BIT(IIO_CHAN_INFO_OFFSET) \
				    | BIT(IIO_CHAN_INFO_SCALE) \
				    | BIT(IIO_CHAN_INFO_SAMP_FREQ) \
				    | BIT(IIO_CHAN_INFO_CALIBSCALE), \
		.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_AVERAGE_RAW), \
		.info_mask_separate_available = BIT(IIO_CHAN_INFO_SAMP_FREQ), \
		.info_mask_shared_by_all_available = BIT(IIO_CHAN_INFO_AVERAGE_RAW), \
		.scan_index = _si, \
		.scan_type = { \
			.sign = 's', \
			.realbits = 16, \
			.storagebits = 16, \
			.endianness = IIO_BE, \
		}, \
		.datasheet_name = _name, \
	}

enum ads1158_scan_si {
	SI_DIFF0 = 0,
	SI_DIFF1,
	SI_DIFF2,
	SI_DIFF3,
	SI_DIFF4,
	SI_DIFF5,
	SI_DIFF6,
	SI_DIFF7,
	SI_AIN0,
	SI_AIN1,
	SI_AIN2,
	SI_AIN3,
	SI_AIN4,
	SI_AIN5,
	SI_AIN6,
	SI_AIN7,
	SI_AIN8,
	SI_AIN9,
	SI_AIN10,
	SI_AIN11,
	SI_AIN12,
	SI_AIN13,
	SI_AIN14,
	SI_AIN15,
	SI_OFFSET,
	SI_VCC = 26,
	SI_TEMP,
	SI_GAIN,
	SI_REF,
	SI_MAX,
};

static const struct iio_chan_spec ads1158_channels[] = {
	ADS1158_VOLTAGE_CHAN_DIFF_IIO(0, 1, SI_DIFF0, "DIFF0"),
	ADS1158_VOLTAGE_CHAN_DIFF_IIO(2, 3, SI_DIFF1, "DIFF1"),
	ADS1158_VOLTAGE_CHAN_DIFF_IIO(4, 5, SI_DIFF2, "DIFF2"),
	ADS1158_VOLTAGE_CHAN_DIFF_IIO(6, 7, SI_DIFF3, "DIFF3"),
	ADS1158_VOLTAGE_CHAN_DIFF_IIO(8, 9, SI_DIFF4, "DIFF4"),
	ADS1158_VOLTAGE_CHAN_DIFF_IIO(10, 11, SI_DIFF5, "DIFF5"),
	ADS1158_VOLTAGE_CHAN_DIFF_IIO(12, 13, SI_DIFF6, "DIFF6"),
	ADS1158_VOLTAGE_CHAN_DIFF_IIO(14, 15, SI_DIFF7, "DIFF7"),
	ADS1158_VOLTAGE_CHAN_IIO(0, SI_AIN0, "AIN0"),
	ADS1158_VOLTAGE_CHAN_IIO(1, SI_AIN1, "AIN1"),
	ADS1158_VOLTAGE_CHAN_IIO(2, SI_AIN2, "AIN2"),
	ADS1158_VOLTAGE_CHAN_IIO(3, SI_AIN3, "AIN3"),
	ADS1158_VOLTAGE_CHAN_IIO(4, SI_AIN4, "AIN4"),
	ADS1158_VOLTAGE_CHAN_IIO(5, SI_AIN5, "AIN5"),
	ADS1158_VOLTAGE_CHAN_IIO(6, SI_AIN6, "AIN6"),
	ADS1158_VOLTAGE_CHAN_IIO(7, SI_AIN7, "AIN7"),
	ADS1158_VOLTAGE_CHAN_IIO(8, SI_AIN8, "AIN8"),
	ADS1158_VOLTAGE_CHAN_IIO(9, SI_AIN9, "AIN9"),
	ADS1158_VOLTAGE_CHAN_IIO(10, SI_AIN10, "AIN10"),
	ADS1158_VOLTAGE_CHAN_IIO(11, SI_AIN11, "AIN11"),
	ADS1158_VOLTAGE_CHAN_IIO(12, SI_AIN12, "AIN12"),
	ADS1158_VOLTAGE_CHAN_IIO(13, SI_AIN13, "AIN13"),
	ADS1158_VOLTAGE_CHAN_IIO(14, SI_AIN14, "AIN14"),
	ADS1158_VOLTAGE_CHAN_IIO(15, SI_AIN15, "AIN15"),
	ADS1158_VOLTAGE_CHAN_IIO(17, SI_VCC, "VCC"),
	ADS1158_TEMP_CHAN_IIO(SI_TEMP, "TEMP"),
	ADS1158_VOLTAGE_CHAN_IIO(18, SI_REF, "REF"),
};

enum ads1158_calibfactor {
	CALIB_INT,
	CALIB_MICRO,
	CALIB_MAX,
};

struct ads1158_state {
	struct spi_device *spi;
	struct regmap *regmap;
	struct regmap_field *regfields[REGF_MAX];
	struct regulator *vref;
	struct mutex lock;
	u32 clock_frequency;
	int data_rate[SCAN_MAX][ARRAY_SIZE(ads1158_data_rate[0])];
	int chan_switching_delay_us[ARRAY_SIZE(ads1158_delay_us)];
	int calibfactor[SI_MAX][CALIB_MAX];
	unsigned int switching_delay_setting[SI_MAX];
	const char *label[SI_MAX];
};

static int ads1158_read_data_rate(struct ads1158_state *st,
				  struct iio_chan_spec const *chan, int *val)
{
	unsigned int buf;
	int *data_rate =
		st->data_rate[chan->differential ? SCAN_FIXED : SCAN_AUTO];
	int ret;

	ret = regmap_field_read(st->regfields[REGF_DR], &buf);
	if (ret) {
		dev_err(&st->spi->dev, "Could not read data rate (%d)\n", ret);
		return ret;
	}

	*val = data_rate[buf];

	return IIO_VAL_INT;
}

static int ads1158_write_data_rate(struct ads1158_state *st,
				   struct iio_chan_spec const *chan, int val)
{
	size_t i;
	int *data_rate =
		st->data_rate[chan->differential ? SCAN_FIXED : SCAN_AUTO];
	int ret;

	for (i = 0; i < ARRAY_SIZE(st->data_rate[0]); i++)
		if (data_rate[i] == val)
			break;

	if (i == ARRAY_SIZE(st->data_rate[0]))
		return -EINVAL;

	ret = regmap_field_write(st->regfields[REGF_DR], i);
	if (ret) {
		dev_err(&st->spi->dev, "Cound not write data rate (%d)\n", ret);
		return ret;
	}

	return 0;
}

static int ads1158_read_average(struct ads1158_state *st, int *val)
{
	unsigned int buf;
	int ret;

	ret = regmap_field_read(st->regfields[REGF_DR], &buf);
	if (ret) {
		dev_err(&st->spi->dev, "Could not read data rate (%d)\n", ret);
		return ret;
	}
	*val = ads1158_data_rate_average[buf];

	return IIO_VAL_INT;
}

static int ads1158_write_average(struct ads1158_state *st, int val)
{
	size_t i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(ads1158_data_rate_average); i++)
		if (ads1158_data_rate_average[i] == val)
			break;

	if (i == ARRAY_SIZE(ads1158_data_rate_average))
		return -EINVAL;

	ret = regmap_field_write(st->regfields[REGF_DR], i);
	if (ret) {
		dev_err(&st->spi->dev, "Could not write data rate (%d)\n", ret);
		return ret;
	}

	return 0;
}

static int ads1158_get_ref_voltage_uv(struct ads1158_state *st)
{
	int vref_uv;

	vref_uv = regulator_get_voltage(st->vref);
	if (vref_uv < 0) {
		dev_err(&st->spi->dev, "Could not get voltage (%d)\n", vref_uv);
		return vref_uv;
	}
	return vref_uv;
}

static int ads1158_scale(struct ads1158_state *st,
			 struct iio_chan_spec const *chan, int *val, int *val2)
{
	int vref_uv;

	vref_uv = ads1158_get_ref_voltage_uv(st);
	if (vref_uv < 0)
		return vref_uv;

	switch (chan->scan_index) {
	case SI_VCC:
	case SI_REF:
		*val = MILLI;
		*val2 = 0xc00;
		return IIO_VAL_FRACTIONAL;
	case SI_DIFF0 ... SI_AIN15:
	case SI_OFFSET:
		*val = DIV_ROUND_CLOSEST(vref_uv, MICRO / MILLI);
		*val2 = 0x7800;
		return IIO_VAL_FRACTIONAL;
	case SI_TEMP:
		*val = vref_uv;
		*val2 = DIV_ROUND_CLOSEST(TEMP_COEFF_UV * 0x7800, MILLI);
		return IIO_VAL_FRACTIONAL;
	default:
		return -EINVAL;
	}
}

static int ads1158_offset(struct ads1158_state *st,
			  struct iio_chan_spec const *chan, int *val, int *val2)
{
	int lsb_deci, vref_uv;

	switch (chan->type) {
	case IIO_TEMP:
		vref_uv = ads1158_get_ref_voltage_uv(st);
		if (vref_uv < 0)
			return vref_uv;

		lsb_deci = DIV_ROUND_CLOSEST(vref_uv * DECI, 0x7800);
		*val = 0;
		*val -= DIV_ROUND_CLOSEST(TEMP_OFFSET_UV * DECI, lsb_deci);
		*val += DIV_ROUND_CLOSEST(
			TEMP_OFFSET_DEGREE * TEMP_COEFF_UV * DECI, lsb_deci);
		break;
	default:
		return -EINVAL;
	}

	return IIO_VAL_INT;
}

static int ads1158_read_avail(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *chan,
			      const int **vals, int *type, int *length,
			      long mask)
{
	struct ads1158_state *st = iio_priv(indio_dev);
	int *data_rate =
		st->data_rate[chan->differential ? SCAN_FIXED : SCAN_AUTO];

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		*type = IIO_VAL_INT;
		*vals = data_rate;
		*length = ARRAY_SIZE(st->data_rate[0]);
		return IIO_AVAIL_LIST;
	case IIO_CHAN_INFO_AVERAGE_RAW:
		*type = IIO_VAL_INT;
		*vals = ads1158_data_rate_average;
		*length = ARRAY_SIZE(ads1158_data_rate_average);
		return IIO_AVAIL_LIST;
	default:
		return -EINVAL;
	}
}

static int ads1158_configure_regs_single_value(struct ads1158_state *st,
					       struct iio_chan_spec const *chan)
{
	int ret;

	ret = regmap_field_write(st->regfields[REGF_MUXMOD],
				 chan->differential ? 1 : 0);
	if (ret)
		return ret;

	ret = regmap_field_write(st->regfields[REGF_DLY],
				 st->switching_delay_setting[chan->scan_index]);
	if (ret)
		return ret;

	if (chan->differential) {
		ret = regmap_set_bits(
			st->regmap, REG_MUXSCH,
			FIELD_PREP(MUXSCH_AINP_MASK, chan->channel) |
				FIELD_PREP(MUXSCH_AINN_MASK, chan->channel2));
		if (ret)
			return ret;
	} else {
		if ((chan->scan_index / 8) == 1)
			ret = regmap_write(st->regmap, REG_MUXSG_0,
					   BIT(chan->scan_index % 8));
		else
			ret = regmap_write(st->regmap, REG_MUXSG_0, 0);
		if (ret)
			return ret;

		if ((chan->scan_index / 8) == 2)
			ret = regmap_write(st->regmap, REG_MUXSG_1,
					   BIT(chan->scan_index % 8));
		else
			ret = regmap_write(st->regmap, REG_MUXSG_1, 0);
		if (ret)
			return ret;

		if ((chan->scan_index / 8) == 3)
			ret = regmap_write(st->regmap, REG_SYSRED,
					   BIT(chan->scan_index % 8));
		else
			ret = regmap_write(st->regmap, REG_SYSRED, 0);
		if (ret)
			return ret;
	}

	return 0;
}

static int ads1158_read_single_value(struct iio_dev *indio_dev,
				     struct iio_chan_spec const *chan, int *val)
{
	struct ads1158_state *st = iio_priv(indio_dev);
	unsigned int buf;
	u8 rx_buf[3];
	__be16 data;
	unsigned int conversion_time_us;
	int ret;

	ret = iio_device_claim_direct_mode(indio_dev);
	if (ret)
		return ret;
	ret = ads1158_configure_regs_single_value(st, chan);
	if (ret)
		goto release;

	buf = FIELD_PREP(CMD_CMD_MASK, PULSE_CONV_CMD);
	ret = spi_write(st->spi, &buf, 1);
	if (ret)
		goto release;

	ret = ads1158_read_data_rate(st, chan, &buf);
	if (ret < 0)
		goto release;

	conversion_time_us = DIV_ROUND_UP(USEC_PER_SEC, buf);
	ret = regmap_field_read(st->regfields[REGF_DLY], &buf);
	if (ret)
		goto release;

	conversion_time_us += st->chan_switching_delay_us[buf];
	usleep_range(conversion_time_us, conversion_time_us * 2);

	buf = FIELD_PREP(CMD_CMD_MASK, CHAN_DATA_READ_CMD) | BIT(CMD_MUL);
	ret = spi_write_then_read(st->spi, &buf, 1, rx_buf, 3);
	if (ret)
		goto release;

release:
	iio_device_release_direct_mode(indio_dev);
	if (ret)
		return ret;

	ret = regmap_field_test_bits(st->regfields[REGF_MUXMOD], 1);
	if (ret < 0)
		return ret;

	if (!ret) {
		/* Sanity check, only applicable in auto-scan mode. */
		if (rx_buf[0] & BIT(STATUS_OVF)) {
			dev_err(indio_dev->dev.parent,
				"Overflow detected (0x%02x)\n", rx_buf[0]);
			return -ERANGE;
		}
		if (rx_buf[0] & BIT(STATUS_SUPPLY)) {
			dev_err(indio_dev->dev.parent,
				"Supply out of range (0x%02x)\n", rx_buf[0]);
			return -ERANGE;
		}
		if ((rx_buf[0] & (BIT(STATUS_NEW) | STATUS_CHID_MASK)) !=
		    (BIT(STATUS_NEW) |
		     FIELD_PREP(STATUS_CHID_MASK, chan->scan_index))) {
			dev_err(indio_dev->dev.parent,
				"Invalid status byte (0x%02x)\n", rx_buf[0]);
			return -EIO;
		}
	}

	memcpy(&data, &rx_buf[1], 2);
	*val = (__s16)be16_to_cpu(data);

	return IIO_VAL_INT;
}

static int ads1158_read_hardwaregain(struct iio_dev *indio_dev, int *val,
				     int *val2)
{
	static const struct iio_chan_spec chan = {
		.scan_index = SI_GAIN,
	};
	int ret;

	ret = ads1158_read_single_value(indio_dev, &chan, val2);
	if (ret < 0)
		return ret;

	*val = 0x7800;

	return IIO_VAL_FRACTIONAL;
}

static int ads1158_read_calibscale(struct ads1158_state *st,
				   struct iio_chan_spec const *chan, int *val,
				   int *val2)
{
	*val = st->calibfactor[chan->scan_index][CALIB_INT];
	*val2 = st->calibfactor[chan->scan_index][CALIB_MICRO];

	return IIO_VAL_INT_PLUS_MICRO;
}

static int ads1158_write_calibscale(struct ads1158_state *st,
				    struct iio_chan_spec const *chan, int val,
				    int val2)
{
	if (val2 < 0 || val2 >= MICRO) {
		return -EINVAL;
	}

	st->calibfactor[chan->scan_index][CALIB_INT] = val;
	st->calibfactor[chan->scan_index][CALIB_MICRO] = val2;

	return 0;
}

static int ads1158_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan, int *val,
			    int *val2, long mask)
{
	struct ads1158_state *st = iio_priv(indio_dev);
	int ret;

	mutex_lock(&st->lock);
	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		ret = ads1158_read_single_value(indio_dev, chan, val);
		break;
	case IIO_CHAN_INFO_OFFSET:
		ret = ads1158_offset(st, chan, val, val2);
		break;
	case IIO_CHAN_INFO_SCALE:
		ret = ads1158_scale(st, chan, val, val2);
		break;
	case IIO_CHAN_INFO_SAMP_FREQ:
		ret = ads1158_read_data_rate(st, chan, val);
		break;
	case IIO_CHAN_INFO_AVERAGE_RAW:
		ret = ads1158_read_average(st, val);
		break;
	case IIO_CHAN_INFO_HARDWAREGAIN:
		ret = ads1158_read_hardwaregain(indio_dev, val, val2);
		break;
	case IIO_CHAN_INFO_CALIBSCALE:
		ret = ads1158_read_calibscale(st, chan, val, val2);
		break;
	default:
		ret = -EINVAL;
	}
	mutex_unlock(&st->lock);

	return ret;
}

static int ads1158_write_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan, int val,
			     int val2, long mask)
{
	struct ads1158_state *st = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		return ads1158_write_data_rate(st, chan, val);
	case IIO_CHAN_INFO_AVERAGE_RAW:
		return ads1158_write_average(st, val);
	case IIO_CHAN_INFO_CALIBSCALE:
		return ads1158_write_calibscale(st, chan, val, val2);
	default:
		return -EINVAL;
	}
}

static int ads1158_read_label(struct iio_dev *indio_dev,
			      const struct iio_chan_spec *chan, char *label)
{
	struct ads1158_state *st = iio_priv(indio_dev);
	int ret;
	char *name, *p;

	if (st->label[chan->scan_index])
		return sysfs_emit(label, "%s\n", st->label[chan->scan_index]);

	name = kstrdup(chan->datasheet_name, GFP_KERNEL);
	if (!name)
		return -ENOMEM;

	for (p = name; *p != '\0'; p++) {
		*p = tolower(*p);
	}

	ret = sysfs_emit(label, "%s\n", name);
	kfree(name);

	return ret;
}

static int ads1158_debugfs_reg_access(struct iio_dev *indio_dev, unsigned reg,
				      unsigned writeval, unsigned *readval)
{
	struct ads1158_state *st = iio_priv(indio_dev);
	int ret;

	mutex_lock(&st->lock);
	if (!readval)
		ret = regmap_write(st->regmap, reg, writeval);
	else
		ret = regmap_read(st->regmap, reg, readval);
	mutex_unlock(&st->lock);

	return ret;
}

static int ads1158_fwnode_xlate(struct iio_dev *indio_dev,
				const struct fwnode_reference_args *iiospec)
{
	int i;

	for (i = 0; i < indio_dev->num_channels; i++)
		if (indio_dev->channels[i].scan_index)
			return i;

	return -EINVAL;
}

static ssize_t channel_switching_delay_show(struct device *dev,
					    struct device_attribute *atttr,
					    char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct ads1158_state *st = iio_priv(indio_dev);
	unsigned int val;
	int ret;

	ret = regmap_field_read(st->regfields[REGF_DLY], &val);
	if (ret)
		return ret;

	return sysfs_emit(buf, "%d\n", st->chan_switching_delay_us[val]);
}

static ssize_t channel_switching_delay_store(struct device *dev,
					     struct device_attribute *attr,
					     const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct ads1158_state *st = iio_priv(indio_dev);
	long val;
	size_t i;
	int ret;

	ret = kstrtol(buf, 10, &val);
	if (ret)
		return ret;

	for (i = 0; i < ARRAY_SIZE(st->chan_switching_delay_us); i++)
		if (val == st->chan_switching_delay_us[i])
			break;

	if (i == ARRAY_SIZE(st->chan_switching_delay_us))
		return -EINVAL;

	mutex_lock(&st->lock);
	ret = regmap_field_write(st->regfields[REGF_DLY], i);
	mutex_unlock(&st->lock);

	if (ret)
		return ret;

	return len;
}

static IIO_DEVICE_ATTR_RW(channel_switching_delay, 0);

static ssize_t channel_switching_delay_available_show(
	struct device *dev, struct device_attribute *atttr, char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct ads1158_state *st = iio_priv(indio_dev);
	size_t i;
	size_t len = 0;

	for (i = 0; i < ARRAY_SIZE(st->chan_switching_delay_us); i++) {
		len += sprintf(buf + len, "%d ",
			       st->chan_switching_delay_us[i]);
	}
	len += sprintf(buf + len, "\n");

	return len;
}

static IIO_DEVICE_ATTR_RO(channel_switching_delay_available, 0);

static struct attribute *ads1158_attributes[] = {
	&iio_dev_attr_channel_switching_delay.dev_attr.attr,
	&iio_dev_attr_channel_switching_delay_available.dev_attr.attr,
	NULL,
};

static const struct attribute_group ads1158_attribute_group = {
	.attrs = ads1158_attributes,
};

static const struct iio_info ads1158_info = {
	.read_avail = ads1158_read_avail,
	.read_raw = ads1158_read_raw,
	.write_raw = ads1158_write_raw,
	.read_label = ads1158_read_label,
	.debugfs_reg_access = ads1158_debugfs_reg_access,
	.fwnode_xlate = ads1158_fwnode_xlate,
	.attrs = &ads1158_attribute_group,
};

static int ads1158_chan_init(struct iio_dev *indio_dev)
{
	struct ads1158_state *st = iio_priv(indio_dev);
	struct iio_chan_spec *channels;
	struct fwnode_handle *child;
	u32 reg, delay_us;
	const char *name;
	unsigned int num_channels;
	size_t i, channel_i;
	int ret;

	num_channels = device_get_child_node_count(indio_dev->dev.parent);
	if (!num_channels) {
		dev_err(indio_dev->dev.parent, "No channel found\n");
		return -ENODATA;
	}
	channels = devm_kcalloc(indio_dev->dev.parent, num_channels,
				sizeof(struct iio_chan_spec), GFP_KERNEL);
	if (!channels)
		return -ENOMEM;

	channel_i = 0;
	device_for_each_child_node(indio_dev->dev.parent, child) {
		ret = fwnode_property_read_u32(child, "reg", &reg);
		if (ret) {
			dev_err(indio_dev->dev.parent,
				"Missing channel index (%d)\n", ret);
			return ret;
		}

		ret = fwnode_property_read_string(child, "label", &name);
		/* label is optional */
		if (ret && ret != -EINVAL) {
			dev_err(indio_dev->dev.parent, "Invalid label (%d)\n",
				ret);
			break;
		}
		st->label[reg] = (!ret) ? name : NULL;

		ret = fwnode_property_read_u32(child, "settling-time-us",
					       &delay_us);
		/* settling-time-us is optional */
		if (ret && ret != -EINVAL) {
			dev_err(indio_dev->dev.parent,
				"Settling time, parsing failed (%d)\n", ret);
			break;
		}
		if (ret) {
			delay_us = 0;
			ret = 0;
		}

		for (i = 0; i < ARRAY_SIZE(st->chan_switching_delay_us); i++) {
			if (delay_us <= st->chan_switching_delay_us[i])
				break;
		}
		if (i == ARRAY_SIZE(st->chan_switching_delay_us)) {
			ret = -EINVAL;
			dev_err(indio_dev->dev.parent,
				"Invalid settling time (%d)\n", ret);
			break;
		}
		st->switching_delay_setting[reg] = i;

		for (i = 0; i < ARRAY_SIZE(ads1158_channels); i++) {
			if (ads1158_channels[i].scan_index == reg)
				break;
		}
		if (i == ARRAY_SIZE(ads1158_channels)) {
			ret = -EINVAL;
			dev_err(indio_dev->dev.parent,
				"Invalid channel index (%d)\n", ret);
			break;
		}
		memcpy(&channels[channel_i], &ads1158_channels[i],
		       sizeof(struct iio_chan_spec));

		channel_i++;
	}
	if (ret)
		return ret;

	indio_dev->num_channels = num_channels;
	indio_dev->channels = channels;

	return 0;
}

static int ads1158_probe(struct spi_device *spi)
{
	const struct spi_device_id *id = spi_get_device_id(spi);
	const struct device_node *dn = dev_of_node(&spi->dev);
	struct iio_dev *indio_dev;
	struct ads1158_state *st;
	size_t i;
	int ret;
	unsigned int buf;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev)
		return dev_err_probe(&spi->dev, -ENOMEM,
				     "Could not allocate iio device\n");

	st = iio_priv(indio_dev);
	st->spi = spi;

	indio_dev->name = id->name;
	iio_device_set_parent(indio_dev, &spi->dev);
	indio_dev->info = &ads1158_info;
	indio_dev->modes = INDIO_DIRECT_MODE;

	st->regmap = devm_regmap_init_spi(spi, &ads1158_regmap_config);
	if (IS_ERR(st->regmap))
		return dev_err_probe(&spi->dev, PTR_ERR(st->regmap),
				     "Could not initialise regmap\n");

	for (i = 0; i < REGF_MAX; i++) {
		st->regfields[i] = devm_regmap_field_alloc(
			indio_dev->dev.parent, st->regmap,
			ads1158_regfields[i]);
		if (IS_ERR(st->regfields[i]))
			return dev_err_probe(
				indio_dev->dev.parent,
				PTR_ERR(st->regfields[i]),
				"Could not allocate register field %zu\n", i);
	}

	mutex_init(&st->lock);

	ret = devm_regulator_get_enable(indio_dev->dev.parent, "vref");
	if (ret)
		return dev_err_probe(
			indio_dev->dev.parent, PTR_ERR(st->vref),
			"Could not get and enable regulator (%d)\n", ret);

	st->vref = devm_regulator_get(indio_dev->dev.parent, "vref");
	if (IS_ERR(st->vref))
		return dev_err_probe(indio_dev->dev.parent, PTR_ERR(st->vref),
				     "Could not get regulator (%d)\n", ret);

	buf = FIELD_PREP(CMD_CMD_MASK, RESET_CMD);
	ret = spi_write(spi, &buf, 1);
	if (ret)
		return dev_err_probe(indio_dev->dev.parent, ret,
				     "Could not reset the device (%d)\n", ret);

	/* assuming external 32.768 kHz crystal */
	st->clock_frequency = 15729000;
	ret = of_property_read_u32(dn, "clock-frequency", &st->clock_frequency);
	if (!ret) {
		dev_dbg(indio_dev->dev.parent,
			"Using default clock-frequency of %u Hz\n",
			st->clock_frequency);
	} else {
		dev_dbg(indio_dev->dev.parent,
			"Using clock-frequency of %u Hz\n",
			st->clock_frequency);
	}
	/* scale samling frequency */
	for (i = 0; i < ARRAY_SIZE(st->data_rate[0]); i++) {
		buf = ads1158_data_rate[SCAN_FIXED][i];
		buf *= (st->clock_frequency / KILO);
		buf /= (16000000 / KILO);
		st->data_rate[SCAN_FIXED][i] = buf;

		buf = ads1158_data_rate[SCAN_AUTO][i];
		buf *= (st->clock_frequency / KILO);
		buf /= (16000000 / KILO);
		st->data_rate[SCAN_AUTO][i] = buf;
	}
	/* scale switching delay */
	for (i = 0; i < ARRAY_SIZE(st->chan_switching_delay_us); i++) {
		buf = ads1158_delay_us[i];
		buf *= (16000000 / KILO);
		buf /= (st->clock_frequency / KILO);
		st->chan_switching_delay_us[i] = buf;
	}
	if (!of_property_read_bool(dn, "clock-output-enable")) {
		dev_dbg(indio_dev->dev.parent, "disable clock output\n");

		ret = regmap_field_write(st->regfields[REGF_CLKENB], 0);
		if (ret)
			return dev_err_probe(
				indio_dev->dev.parent, ret,
				"Could not write clock enable (%d)\n", ret);
	}

	/* init calibfactor to 1.0 */
	for (i = 0; i < ARRAY_SIZE(st->calibfactor); i++) {
		st->calibfactor[i][CALIB_INT] = 1;
		st->calibfactor[i][CALIB_MICRO] = 0;
	}

	ret = ads1158_chan_init(indio_dev);
	if (ret)
		return dev_err_probe(indio_dev->dev.parent, ret,
				     "Could not initialise channels (%d)\n",
				     ret);

	return devm_iio_device_register(&spi->dev, indio_dev);
}

static const struct of_device_id ads1158_of_match[] = {
	{ .compatible = "ti,ads1158" },
	{},
};
MODULE_DEVICE_TABLE(of, ads1158_of_match);

static const struct spi_device_id ads1158_id[] = {
	{ .name = "ads1158" },
	{},
};
MODULE_DEVICE_TABLE(spi, ads1158_id);

static struct spi_driver ads1158_driver = {
	.driver = {
		.name = KBUILD_MODNAME,
                .owner = THIS_MODULE,
		.of_match_table = of_match_ptr(ads1158_of_match),
	},
	.probe = ads1158_probe,
	.id_table = ads1158_id,
};
module_spi_driver(ads1158_driver);

MODULE_AUTHOR("Alain Péteut <alain.peteut@spacetek.ch>");
MODULE_DESCRIPTION("ADS1158 ADC");
MODULE_LICENSE("GPL v2");
