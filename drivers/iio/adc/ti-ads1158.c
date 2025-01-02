// SPDX-License-Identifier: GPL-2.0-only
/*
 * iio/adc/ti-ads1158.c
 * Copyright (C) 2024 Alain Péteut
 *
 * based on linux/drivers/iio/max1027.c
 * Copyright (C) 2014 Philippe Reynes
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

/* Command byte */
enum ads1158_cmd {
	CHAN_DATA_READ_CMD = 0x1,
	REG_READ_CMD,
	REG_WRITE_CMD,
	PULSE_CONV_CMD,
	RESET_CMD = 0x6,
};
#define ADS1158_CMD_MASK GENMASK(7, 5)
#define ADS1158_CMD_MUL 4
#define ADS1158_CMD_ADDR_MASK GENMASK(3, 0)

/* Status byte. */
#define ADS1158_STATUS_NEW 7
#define ADS1158_STATUS_OVF 6
#define ADS1158_STATUS_SUPPLY 5
#define ADS1158_STATUS_CHID_MASK GENMASK(4, 0)

/* MUXSCH */
#define ADS1158_AINP GENMASK(7, 4)
#define ADS1158_AINN GENMASK(3, 0)

/* Temperature sensor coefficient. */
#define ADS1158_TEMP_COEFF_UV 563
/* Temperature sensor offset [uV]. */
#define ADS1158_TEMP_OFFSET_UV 168000
/* Temperature sensor offset [°C]. */
#define ADS1158_TEMP_OFFSET_DEGREE 25

/* Registers. */
enum ads1158_regs {
	ADS1158_CONFIG_0 = 0x00,
	ADS1158_CONFIG_1,
	ADS1158_MUXSCH,
	ADS1158_MUXDIF,
	ADS1158_MUXSG_0,
	ADS1158_MUXSG_1,
	ADS1158_SYSRED,
	ADS1158_GPIOD,
	ADS1158_ID,
	ADS1158_REG_MAX,
};

/* Register fields. */
enum ads1158_regfield {
	/* Mux mode. */
	ADS1158_REGF_MUXMOD,
	/* Clock output enable on CLKIO. */
	ADS1158_REGF_CLKENB,
	/* Delay after indexing. */
	ADS1158_REGF_DLY,
	/* Data rate. */
	ADS1158_REGF_DR,
	/* ID bit. */
	ADS1158_REGF_IS_ADS1158,
	ADS1158_REGF_MAX,
};

static const struct reg_field ads1158_regfields[] = {
	[ADS1158_REGF_MUXMOD] = REG_FIELD(ADS1158_CONFIG_0, 5, 5),
	[ADS1158_REGF_CLKENB] = REG_FIELD(ADS1158_CONFIG_0, 3, 3),
	[ADS1158_REGF_DLY] = REG_FIELD(ADS1158_CONFIG_1, 4, 6),
	[ADS1158_REGF_DR] = REG_FIELD(ADS1158_CONFIG_1, 0, 1),
	[ADS1158_REGF_IS_ADS1158] = REG_FIELD(ADS1158_ID, 4, 4),
};

static const int ads1158_data_rate_average[] = { 64, 16, 4, 1 };
/* SPS for clock frequency of 16 MHz */
static const int ads1158_data_rate_auto_scan[] = { 1831, 6168, 15123, 23739 };
static const int ads1158_data_rate_fixed_chan[] = { 1953, 7813, 31250, 125000 };

static const int ads1158_delay_us[] = { 0, 8, 16, 32, 64, 128, 256, 384 };

struct ads1158_state {
	struct spi_device *spi;
	struct regmap *regmap;
	struct regmap_field *regfields[ADS1158_REGF_MAX];
	struct regulator *vref_p, *vref_n;
	struct mutex lock;

	u32 clock_frequency;
	int data_rate[ARRAY_SIZE(ads1158_data_rate_fixed_chan)];
};

static bool ads1158_volatile_register(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case ADS1158_GPIOD:
	case ADS1158_ID:
		return true;
	default:
		return false;
	}
}

static bool ads1158_readable_register(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case ADS1158_CONFIG_0:
	case ADS1158_CONFIG_1:
	case ADS1158_MUXSCH:
	case ADS1158_MUXDIF:
	case ADS1158_MUXSG_0:
	case ADS1158_MUXSG_1:
	case ADS1158_SYSRED:
	case ADS1158_GPIOD:
	case ADS1158_ID:
		return true;
	default:
		return false;
	}
}

static const struct regmap_config ads1158_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.volatile_reg = ads1158_volatile_register,
	.max_register = ADS1158_REG_MAX,
	.readable_reg = ads1158_readable_register,
	.read_flag_mask = FIELD_PREP_CONST(ADS1158_CMD_MASK, REG_READ_CMD),
	.write_flag_mask = FIELD_PREP_CONST(ADS1158_CMD_MASK, REG_WRITE_CMD),
};

#define ADS1158_VOLTAGE_CHAN_IIO(_chan, _si, _name) \
	{ \
		.type = IIO_VOLTAGE, \
		.indexed = 1, \
		.channel = _chan, \
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) \
				    | BIT(IIO_CHAN_INFO_SCALE), \
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_AVERAGE_RAW) \
					  | BIT(IIO_CHAN_INFO_SAMP_FREQ), \
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
				    | BIT(IIO_CHAN_INFO_SCALE), \
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_AVERAGE_RAW) \
					  | BIT(IIO_CHAN_INFO_SAMP_FREQ), \
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
				    | BIT(IIO_CHAN_INFO_SCALE), \
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_AVERAGE_RAW) \
					  | BIT(IIO_CHAN_INFO_SAMP_FREQ), \
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
	ADS1158_VOLTAGE_CHAN_IIO(16, SI_OFFSET, "OFFSET"),
	ADS1158_VOLTAGE_CHAN_IIO(17, SI_VCC, "VCC"),
	ADS1158_TEMP_CHAN_IIO(SI_TEMP, "TEMP"),
	ADS1158_VOLTAGE_CHAN_IIO(18, SI_REF, "REF"),
};

static int ads1158_read_data_rate(struct ads1158_state *st, int *val)
{
	unsigned int buf;
	int ret;

	ret = regmap_field_read(st->regfields[ADS1158_REGF_DR], &buf);
	if (ret) {
		dev_err(&st->spi->dev, "Could not read data rate (%d)\n", ret);
		return ret;
	}

	*val = st->data_rate[buf];

	return IIO_VAL_INT;
}

static int ads1158_write_data_rate(struct ads1158_state *st, int val)
{
	size_t i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(st->data_rate); i++)
		if (st->data_rate[i] == val)
			break;

	if (i == ARRAY_SIZE(st->data_rate))
		return -ERANGE;

	ret = regmap_field_write(st->regfields[ADS1158_REGF_DR], i);
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

	ret = regmap_field_read(st->regfields[ADS1158_REGF_DR], &buf);
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
		return -ERANGE;

	ret = regmap_field_write(st->regfields[ADS1158_REGF_DR], i);
	if (ret) {
		dev_err(&st->spi->dev, "Could not write data rate (%d)\n", ret);
		return ret;
	}

	return 0;
}

static int ads1158_get_ref_voltage_uv(struct ads1158_state *st)
{
	int vp_uv, vn_uv;

	vp_uv = regulator_get_voltage(st->vref_p);
	if (vp_uv < 0) {
		dev_err(&st->spi->dev, "Could not get voltage (%d)\n", vp_uv);
		return vp_uv;
	}
	if (!IS_ERR(st->vref_n)) {
		vn_uv = regulator_get_voltage(st->vref_n);
		if (vn_uv < 0) {
			dev_err(&st->spi->dev, "Could not get voltage (%d)\n",
				vn_uv);
			return vn_uv;
		}
	} else
		vn_uv = 0;

	return vp_uv - vn_uv;
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
		*val = DIV_ROUND_CLOSEST(vref_uv, MICRO / MILLI);
		*val2 = 0xc00;
		return IIO_VAL_FRACTIONAL;
	case SI_DIFF0:
	case SI_DIFF1:
	case SI_DIFF2:
	case SI_DIFF3:
	case SI_DIFF4:
	case SI_DIFF5:
	case SI_DIFF6:
	case SI_DIFF7:
	case SI_AIN0:
	case SI_AIN1:
	case SI_AIN2:
	case SI_AIN3:
	case SI_AIN4:
	case SI_AIN5:
	case SI_AIN6:
	case SI_AIN7:
	case SI_AIN8:
	case SI_AIN9:
	case SI_AIN10:
	case SI_AIN11:
	case SI_AIN12:
	case SI_AIN13:
	case SI_AIN14:
	case SI_AIN15:
		*val = DIV_ROUND_CLOSEST(vref_uv, MICRO / MILLI);
		*val2 = 0x7800;
		return IIO_VAL_FRACTIONAL;
	case SI_TEMP:
		*val = vref_uv;
		*val2 = DIV_ROUND_CLOSEST(ADS1158_TEMP_COEFF_UV * 0x7800, 1000);
		return IIO_VAL_FRACTIONAL;
	default:
		return -EINVAL;
	}
}

static int ads1158_offset(struct ads1158_state *st,
			  struct iio_chan_spec const *chan, int *val, int *val2)
{
	int lsb, vref_uv;

	switch (chan->type) {
	case IIO_TEMP:
		vref_uv = ads1158_get_ref_voltage_uv(st);
		if (vref_uv < 0)
			return vref_uv;

		lsb = DIV_ROUND_CLOSEST(vref_uv, 0x7800);
		*val = 0;
		*val -= DIV_ROUND_CLOSEST(ADS1158_TEMP_OFFSET_UV, lsb);
		*val += DIV_ROUND_CLOSEST(ADS1158_TEMP_OFFSET_DEGREE *
						  ADS1158_TEMP_COEFF_UV,
					  lsb);
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

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		*type = IIO_VAL_INT;
		*vals = st->data_rate;
		*length = ARRAY_SIZE(st->data_rate);
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

static int ads1158_write_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan, int val,
			     int val2, long mask)
{
	struct ads1158_state *st = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		return ads1158_write_data_rate(st, val);
	case IIO_CHAN_INFO_AVERAGE_RAW:
		return ads1158_write_average(st, val);
	default:
		return -EINVAL;
	}
}

static int ads1158_configure_regs_single_value(struct ads1158_state *st,
					       struct iio_chan_spec const *chan)
{
	int ret;

	ret = regmap_field_write(st->regfields[ADS1158_REGF_MUXMOD],
				 chan->differential ? 1 : 0);
	if (ret)
		return ret;

	if (chan->differential) {
		ret = regmap_set_bits(st->regmap, ADS1158_MUXSCH,
				      FIELD_PREP(ADS1158_AINP, chan->channel) |
					      FIELD_PREP(ADS1158_AINN,
							 chan->channel2));
		if (ret)
			return ret;
	} else {
		if ((chan->scan_index / 8) == 1) {
			ret = regmap_set_bits(st->regmap, ADS1158_MUXSG_0,
					      BIT(chan->scan_index % 8));
		} else {
			ret = regmap_write(st->regmap, ADS1158_MUXSG_0, 0);
		}
		if (ret)
			return ret;

		if ((chan->scan_index / 8) == 2) {
			ret = regmap_set_bits(st->regmap, ADS1158_MUXSG_1,
					      BIT(chan->scan_index % 8));
		} else {
			ret = regmap_write(st->regmap, ADS1158_MUXSG_1, 0);
		}
		if (ret)
			return ret;

		if ((chan->scan_index / 8) == 3) {
			ret = regmap_set_bits(st->regmap, ADS1158_SYSRED,
					      BIT(chan->scan_index % 8));
		} else {
			ret = regmap_write(st->regmap, ADS1158_SYSRED, 0);
		}
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
	__u16 data;
	unsigned int conversion_time_us;
	int ret;

	ret = iio_device_claim_direct_mode(indio_dev);
	if (ret)
		goto release;

	ret = ads1158_configure_regs_single_value(st, chan);
	if (ret)
		goto release;

	ret = ads1158_read_data_rate(st, &buf);
	if (ret < 0)
		goto release;

	conversion_time_us = DIV_ROUND_UP(USEC_PER_SEC, st->data_rate[buf]);
	usleep_range(conversion_time_us, conversion_time_us * 2);

	buf = FIELD_PREP(ADS1158_CMD_MASK, CHAN_DATA_READ_CMD) |
	      BIT(ADS1158_CMD_MUL);
	spi_write_then_read(st->spi, &buf, 1, rx_buf, 3);

release:
	iio_device_release_direct_mode(indio_dev);

	if (!ret)
		return ret;

	if ((rx_buf[0] &
	     ~(BIT(ADS1158_STATUS_OVF) | BIT(ADS1158_STATUS_SUPPLY))) !=
	    (BIT(ADS1158_STATUS_NEW) |
	     FIELD_PREP(ADS1158_STATUS_CHID_MASK, chan->scan_index))) {
		dev_err(indio_dev->dev.parent, "Invalid status byte (0x%02x)\n",
			rx_buf[0]);
		return -EIO;
	}
	if (rx_buf[0] & BIT(ADS1158_STATUS_OVF))
		dev_err(indio_dev->dev.parent, "Overflow detected (0x%02x)\n",
			rx_buf[0]);
	return -ERANGE;

	memcpy(&data, &rx_buf[1], 2);
	*val = be16_to_cpu(data);

	return IIO_VAL_INT;
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
		ret = ads1158_read_data_rate(st, val);
		break;
	case IIO_CHAN_INFO_AVERAGE_RAW:
		ret = ads1158_read_average(st, val);
		break;
	default:
		ret = -EINVAL;
	}
	mutex_unlock(&st->lock);

	return ret;
}

static int ads1158_read_label(struct iio_dev *iio_dev,
			      const struct iio_chan_spec *chan, char *label)
{
	int ret;
	char *name, *p;

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

static const struct iio_info ads1158_info = {
	.read_avail = ads1158_read_avail,
	.read_raw = ads1158_read_raw,
	.write_raw = ads1158_write_raw,
	.read_label = ads1158_read_label,
	.debugfs_reg_access = ads1158_debugfs_reg_access,
	.fwnode_xlate = ads1158_fwnode_xlate,
};

static int ads1158_probe(struct spi_device *spi)
{
	const struct spi_device_id *id = spi_get_device_id(spi);
	const struct device_node *dn = dev_of_node(&spi->dev);
	struct iio_dev *indio_dev;
	struct ads1158_state *st;
	u64 tmp;
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
	indio_dev->channels = ads1158_channels;
	indio_dev->num_channels = ARRAY_SIZE(ads1158_channels);

	st->regmap = devm_regmap_init_spi(spi, &ads1158_regmap_config);
	if (IS_ERR(st->regmap))
		return dev_err_probe(&spi->dev, PTR_ERR(st->regmap),
				     "Could not initialise regmap\n");

	for (i = 0; i < ADS1158_REGF_MAX; i++) {
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

	ret = devm_regulator_get_enable(indio_dev->dev.parent, "vref-p");
	if (ret)
		return dev_err_probe(
			indio_dev->dev.parent, PTR_ERR(st->vref_p),
			"Could not get and enable regulator (%d)\n", ret);

	st->vref_p = devm_regulator_get(indio_dev->dev.parent, "vref-p");
	if (IS_ERR(st->vref_p))
		return dev_err_probe(indio_dev->dev.parent, PTR_ERR(st->vref_p),
				     "Could not get regulator (%d)\n", ret);

	ret = devm_regulator_get_enable_optional(indio_dev->dev.parent,
						 "vref-n");
	if (ret < 0 && ret != -ENODEV)
		return dev_err_probe(indio_dev->dev.parent, ret,
				     "Could no get and enable regulator (%d)\n",
				     ret);

	st->vref_n =
		devm_regulator_get_optional(indio_dev->dev.parent, "vref-n");
	if (IS_ERR(st->vref_n)) {
		ret = PTR_ERR(st->vref_n);
		if (ret != -ENODEV)
			return dev_err_probe(
				indio_dev->dev.parent, ret,
				"Could not get and regulator (%d)\n", ret);
	}

	buf = FIELD_PREP(ADS1158_CMD_MASK, RESET_CMD);
	ret = spi_write(spi, &buf, 1);
	if (ret)
		return dev_err_probe(indio_dev->dev.parent, ret,
				     "Could not reset the device (%d)\n", ret);

	/* assuming external 32.768 kHz crystal */
	st->clock_frequency = 15729000;
	ret = of_property_read_u32(dn, "clock-frequency", &st->clock_frequency);
	if (!ret) {
		dev_dbg(indio_dev->dev.parent,
			"Using default clock-frequency of %ul Hz\n",
			st->clock_frequency);
	}
	/* scale samling frequency */
	for (i = 0; i < ARRAY_SIZE(st->data_rate); i++) {
		tmp = ads1158_data_rate_fixed_chan[i];
		tmp *= 16000000;
		div_u64(tmp, st->clock_frequency);
		st->data_rate[i] = tmp;
	}
	if (of_property_read_bool(dn, "clock-ouput-enable")) {
		ret = regmap_field_write(st->regfields[ADS1158_REGF_CLKENB], 1);
		if (ret)
			return dev_err_probe(
				indio_dev->dev.parent, ret,
				"Could not write clock enable (%d)\n", ret);
	}

	ret = regmap_field_test_bits(st->regfields[ADS1158_REGF_IS_ADS1158], 1);
	if (ret < 0)
		return dev_err_probe(indio_dev->dev.parent, -EIO,
				     "Could not read ID register (%d)\n", ret);
	if (ret == 0)
		return dev_err_probe(indio_dev->dev.parent, -EINVAL,
				     "Invalid ID\n");

	return devm_iio_device_register(&spi->dev, indio_dev);
}

static const struct of_device_id ads1158_of_match[] = {
	{ .compatible = "ti,ads1158" },
	{},
};
MODULE_DEVICE_TABLE(of, ads1158_of_match);

static struct spi_driver ads1158_driver = {
	.driver = {
		.name = KBUILD_MODNAME,
                .owner = THIS_MODULE,
		.of_match_table = of_match_ptr(ads1158_of_match),
	},
	.probe = ads1158_probe,
};
module_spi_driver(ads1158_driver);

MODULE_AUTHOR("Alain Péteut <alain.peteut@spacetek.ch>");
MODULE_DESCRIPTION("ADS1158 ADC");
MODULE_LICENSE("GPL v2");
