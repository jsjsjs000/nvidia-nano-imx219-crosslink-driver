/*
 * imx219_crosslink.c - imx219_crosslink parallel to CSI-2 driver
 */

#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/seq_file.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>

#include <media/camera_common.h>
#include <media/tegra_v4l2_camera.h>
#include <media/tegracam_core.h>
#include <media/imx219_crosslink.h>

/* imx219 sensor register address */
#define IMX219_CROSSLINK_MODEL_ID_ADDR_MSB		0x0000
#define IMX219_CROSSLINK_MODEL_ID_ADDR_LSB		0x0001

static const struct of_device_id imx219_crosslink_of_match[] = {
	{.compatible = "pco,imx219_crosslink",},
	{},
};

MODULE_DEVICE_TABLE(of, imx219_crosslink_of_match);

enum {
	IMX219_CROSSLINK_MODE_3264x2464_21FPS,
	IMX219_CROSSLINK_MODE_640x480_60FPS,
	IMX219_CROSSLINK_MODE_COMMON,
	IMX219_CROSSLINK_START_STREAM,
	IMX219_CROSSLINK_STOP_STREAM,
};

static const int imx219_crosslink_30_fr[] = {
	30,
};

static const int imx219_crosslink_60_fr[] = {
	60,
};

static const int imx219_crosslink_21_fr[] = {
	21,
};

static const struct camera_common_frmfmt imx219_crosslink_frmfmt[] = {
	{{3264, 2464}, imx219_crosslink_21_fr, 1, 0, IMX219_CROSSLINK_MODE_3264x2464_21FPS},
	{{640, 480}, imx219_crosslink_60_fr, 1, 0, IMX219_CROSSLINK_MODE_640x480_60FPS},
};

static const u32 ctrl_cid_list[] = {
	TEGRA_CAMERA_CID_GAIN,
	TEGRA_CAMERA_CID_EXPOSURE,
	TEGRA_CAMERA_CID_FRAME_RATE,
	TEGRA_CAMERA_CID_SENSOR_MODE_ID,
};

struct imx219_crosslink {
	struct i2c_client *i2c_client;
	struct v4l2_subdev *subdev;
	u16 fine_integ_time;
	u32 frame_length;
	struct camera_common_data *s_data;
	struct tegracam_device *tc_dev;
};

static const struct regmap_config sensor_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.cache_type = REGCACHE_RBTREE,
	.use_single_rw = true,
};

// static inline void imx219_crosslink_get_frame_length_regs(imx219_crosslink_reg * regs,
// 						u32 frame_length)
// {
// 	regs->addr = imx219_crosslink_FRAME_LENGTH_ADDR_MSB;
// 	regs->val = (frame_length >> 8) & 0xff;
// 	(regs + 1)->addr = imx219_crosslink_FRAME_LENGTH_ADDR_LSB;
// 	(regs + 1)->val = (frame_length) & 0xff;
// }

// static inline void imx219_crosslink_get_coarse_integ_time_regs(imx219_crosslink_reg * regs,
// 						     u32 coarse_time)
// {
// 	regs->addr = imx219_crosslink_COARSE_INTEG_TIME_ADDR_MSB;
// 	regs->val = (coarse_time >> 8) & 0xff;
// 	(regs + 1)->addr = imx219_crosslink_COARSE_INTEG_TIME_ADDR_LSB;
// 	(regs + 1)->val = (coarse_time) & 0xff;
// }

// static inline void imx219_crosslink_get_gain_reg(imx219_crosslink_reg * reg, u16 gain)
// {
// 	reg->addr = imx219_crosslink_ANALOG_GAIN_ADDR_MSB;
// 	reg->val = (gain >> imx219_crosslink_SHIFT_8_BITS) & imx219_crosslink_MASK_LSB_2_BITS;

// 	(reg + 1)->addr = imx219_crosslink_ANALOG_GAIN_ADDR_LSB;
// 	(reg + 1)->val = (gain) & imx219_crosslink_MASK_LSB_8_BITS;
// }

static inline int imx219_crosslink_read_reg(struct camera_common_data *s_data,
				  u16 addr, u8 * val)
{
	int err = 0;
	u32 reg_val = 0;

	err = regmap_read(s_data->regmap, addr, &reg_val);
	*val = reg_val & 0xff;

	return err;
}

static inline int imx219_crosslink_write_reg(struct camera_common_data *s_data,
				   u16 addr, u8 val)
{
	int err = 0;

	err = regmap_write(s_data->regmap, addr, val);
	if (err)
		dev_err(s_data->dev, "%s: i2c write failed, 0x%x = %x",
			__func__, addr, val);

	return err;
}

// static int imx219_crosslink_write_table(struct imx219_crosslink *priv, const imx219_crosslink_reg table[])
// {
// 	return regmap_util_write_table_8(priv->s_data->regmap, table, NULL, 0,
// 					 imx219_crosslink_TABLE_WAIT_MS,
// 					 imx219_crosslink_TABLE_END);
// }

static int imx219_crosslink_set_group_hold(struct tegracam_device *tc_dev, bool val)
{
// 	struct camera_common_data *s_data = tc_dev->s_data;
// 	struct device *dev = tc_dev->dev;
// 	int err;

// 	dev_dbg(dev, "%s: Setting group hold control to: %u\n", __func__, val);

// 	err = imx219_crosslink_write_reg(s_data, imx219_crosslink_GROUP_HOLD_ADDR, val);
// 	if (err) {
// 		dev_err(dev, "%s: Group hold control error\n", __func__);
// 		return err;
// 	}

	return 0;
}

// static int imx219_crosslink_get_fine_integ_time(struct imx219_crosslink *priv, u16 * fine_time)
// {
// 	struct camera_common_data *s_data = priv->s_data;
// 	int err = 0;
// 	u8 reg_val[2];

// 	err = imx219_crosslink_read_reg(s_data, imx219_crosslink_FINE_INTEG_TIME_ADDR_MSB,
// 			      &reg_val[0]);
// 	if (err)
// 		goto done;

// 	err = imx219_crosslink_read_reg(s_data, imx219_crosslink_FINE_INTEG_TIME_ADDR_LSB,
// 			      &reg_val[1]);
// 	if (err)
// 		goto done;

// 	*fine_time = (reg_val[0] << 8) | reg_val[1];

// done:
// 	return err;
// 	return 0;
// }

static int imx219_crosslink_set_gain(struct tegracam_device *tc_dev, s64 val)
{
// 	struct camera_common_data *s_data = tc_dev->s_data;
// 	struct device *dev = s_data->dev;
// 	const struct sensor_mode_properties *mode =
// 	    &s_data->sensor_props.sensor_modes[s_data->mode_prop_idx];
// 	int err = 0, i = 0;
// 	imx219_crosslink_reg gain_reg[2];
// 	s16 gain;

// 	dev_dbg(dev, "%s: Setting gain control to: %lld\n", __func__, val);

// 	if (val < mode->control_properties.min_gain_val)
// 		val = mode->control_properties.min_gain_val;
// 	else if (val > mode->control_properties.max_gain_val)
// 		val = mode->control_properties.max_gain_val;

// 	/* Gain Formula:
// 	   Gain = (imx219_crosslink_GAIN_C0 - (imx219_crosslink_GAIN_C0 * gain_factor / val))
// 	 */
// 	gain =
// 	    (s16) (imx219_crosslink_ANALOG_GAIN_C0 -
// 		   (mode->control_properties.gain_factor *
// 		    imx219_crosslink_ANALOG_GAIN_C0 / val));

// 	if (gain < imx219_crosslink_MIN_GAIN)
// 		gain = imx219_crosslink_MAX_GAIN;
// 	else if (gain > imx219_crosslink_MAX_GAIN)
// 		gain = imx219_crosslink_MAX_GAIN;

// 	dev_dbg(dev, "%s: val: %lld (/%d) [times], gain: %u\n",
// 		__func__, val, mode->control_properties.gain_factor, gain);

// 	imx219_crosslink_get_gain_reg(gain_reg, (u16) gain);

// 	for (i = 0; i < ARRAY_SIZE(gain_reg); i++) {
// 		err = imx219_crosslink_write_reg(s_data, gain_reg[i].addr,
// 				       gain_reg[i].val);
// 		if (err) {
// 			dev_err(dev, "%s: gain control error\n", __func__);
// 			break;
// 		}
// 	}

// 	return err;
	return 0;
}

static int imx219_crosslink_set_frame_rate(struct tegracam_device *tc_dev, s64 val)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct imx219_crosslink *priv = (struct imx219_crosslink *)tc_dev->priv;
	struct device *dev = tc_dev->dev;
	const struct sensor_mode_properties *mode =
	    &s_data->sensor_props.sensor_modes[s_data->mode_prop_idx];

	int err = 0;
	// imx219_crosslink_reg fl_regs[2];
	u32 frame_length;
	// int i;

	dev_dbg(dev, "%s: Setting framerate control to: %lld\n", __func__, val);

	frame_length = (u32) (mode->signal_properties.pixel_clock.val *
			      (u64) mode->control_properties.framerate_factor /
			      mode->image_properties.line_length / val);

	if (frame_length < IMX219_CROSSLINK_MIN_FRAME_LENGTH)
		frame_length = IMX219_CROSSLINK_MIN_FRAME_LENGTH;
	else if (frame_length > IMX219_CROSSLINK_MAX_FRAME_LENGTH)
		frame_length = IMX219_CROSSLINK_MAX_FRAME_LENGTH;

	dev_dbg(dev,
		"%s: val: %llde-6 [fps], frame_length: %u [lines]\n",
		__func__, val, frame_length);

	// imx219_crosslink_get_frame_length_regs(fl_regs, frame_length);
	// for (i = 0; i < 2; i++) {
	// 	err = imx219_crosslink_write_reg(s_data, fl_regs[i].addr, fl_regs[i].val);
	// 	if (err) {
	// 		dev_err(dev,
	// 			"%s: frame_length control error\n", __func__);
	// 		return err;
	// 	}
	// }

	priv->frame_length = frame_length;

	return err;
}

static int imx219_crosslink_set_exposure(struct tegracam_device *tc_dev, s64 val)
{
// 	struct camera_common_data *s_data = tc_dev->s_data;
// 	struct imx219_crosslink *priv = (struct imx219_crosslink *)tc_dev->priv;
// 	struct device *dev = tc_dev->dev;
// 	const struct sensor_mode_properties *mode =
// 	    &s_data->sensor_props.sensor_modes[s_data->mode_prop_idx];

// 	int err = 0;
// 	imx219_crosslink_reg ct_regs[2];
// 	const s32 max_coarse_time = priv->frame_length - imx219_crosslink_MAX_COARSE_DIFF;
// 	const s32 fine_integ_time_factor = priv->fine_integ_time *
// 	    mode->control_properties.exposure_factor /
// 	    mode->signal_properties.pixel_clock.val;
// 	u32 coarse_time;
// 	int i;

// 	dev_dbg(dev, "%s: Setting exposure control to: %lld\n", __func__, val);

// 	coarse_time = (val - fine_integ_time_factor)
// 	    * mode->signal_properties.pixel_clock.val
// 	    / mode->control_properties.exposure_factor
// 	    / mode->image_properties.line_length;

// 	if (coarse_time < imx219_crosslink_MIN_COARSE_EXPOSURE)
// 		coarse_time = imx219_crosslink_MIN_COARSE_EXPOSURE;
// 	else if (coarse_time > max_coarse_time) {
// 		coarse_time = max_coarse_time;
// 		dev_dbg(dev,
// 			"%s: exposure limited by frame_length: %d [lines]\n",
// 			__func__, max_coarse_time);
// 	}

// 	dev_dbg(dev, "%s: val: %lld [us], coarse_time: %d [lines]\n",
// 		__func__, val, coarse_time);

// 	imx219_crosslink_get_coarse_integ_time_regs(ct_regs, coarse_time);

// 	for (i = 0; i < 2; i++) {
// 		err = imx219_crosslink_write_reg(s_data, ct_regs[i].addr, ct_regs[i].val);
// 		if (err) {
// 			dev_dbg(dev,
// 				"%s: coarse_time control error\n", __func__);
// 			return err;
// 		}
// 	}

// 	return err;
	return 0;
}

static struct tegracam_ctrl_ops imx219_crosslink_ctrl_ops = {
	.numctrls = ARRAY_SIZE(ctrl_cid_list),
	.ctrl_cid_list = ctrl_cid_list,
	.set_gain = imx219_crosslink_set_gain,
	.set_exposure = imx219_crosslink_set_exposure,
	.set_frame_rate = imx219_crosslink_set_frame_rate,
	.set_group_hold = imx219_crosslink_set_group_hold,
};

static int imx219_crosslink_power_on(struct camera_common_data *s_data)
{
	int err = 0;
	struct camera_common_power_rail *pw = s_data->power;
	struct camera_common_pdata *pdata = s_data->pdata;
	struct device *dev = s_data->dev;

	dev_dbg(dev, "%s: power on\n", __func__);
	if (pdata && pdata->power_on) {
		err = pdata->power_on(pw);
		if (err)
			dev_err(dev, "%s failed.\n", __func__);
		else
			pw->state = SWITCH_ON;
		return err;
	}

	if (pw->reset_gpio) {
		if (gpio_cansleep(pw->reset_gpio))
			gpio_set_value_cansleep(pw->reset_gpio, 0);
		else
			gpio_set_value(pw->reset_gpio, 0);
	}

	if (unlikely(!(pw->avdd || pw->iovdd || pw->dvdd)))
		goto skip_power_seqn;

	usleep_range(10, 20);

	if (pw->avdd) {
		err = regulator_enable(pw->avdd);
		if (err)
			goto imx219_crosslink_avdd_fail;
	}

	if (pw->iovdd) {
		err = regulator_enable(pw->iovdd);
		if (err)
			goto imx219_crosslink_iovdd_fail;
	}

	if (pw->dvdd) {
		err = regulator_enable(pw->dvdd);
		if (err)
			goto imx219_crosslink_dvdd_fail;
	}

	usleep_range(10, 20);

skip_power_seqn:
	if (pw->reset_gpio) {
		if (gpio_cansleep(pw->reset_gpio))
			gpio_set_value_cansleep(pw->reset_gpio, 1);
		else
			gpio_set_value(pw->reset_gpio, 1);
	}

	/* Need to wait for t4 + t5 + t9 + t10 time as per the data sheet */
	/* t4 - 200us, t5 - 21.2ms, t9 - 1.2ms t10 - 270 ms */
	usleep_range(300000, 300100);

	pw->state = SWITCH_ON;

	return 0;

imx219_crosslink_dvdd_fail:
	regulator_disable(pw->iovdd);

imx219_crosslink_iovdd_fail:
	regulator_disable(pw->avdd);

imx219_crosslink_avdd_fail:
	dev_err(dev, "%s failed.\n", __func__);

	return -ENODEV;
}

static int imx219_crosslink_power_off(struct camera_common_data *s_data)
{
	int err = 0;
	struct camera_common_power_rail *pw = s_data->power;
	struct camera_common_pdata *pdata = s_data->pdata;
	struct device *dev = s_data->dev;

	dev_dbg(dev, "%s: power off\n", __func__);

	if (pdata && pdata->power_off) {
		err = pdata->power_off(pw);
		if (err) {
			dev_err(dev, "%s failed.\n", __func__);
			return err;
		}
	} else {
		if (pw->reset_gpio) {
			if (gpio_cansleep(pw->reset_gpio))
				gpio_set_value_cansleep(pw->reset_gpio, 0);
			else
				gpio_set_value(pw->reset_gpio, 0);
		}

		usleep_range(10, 10);

		if (pw->dvdd)
			regulator_disable(pw->dvdd);
		if (pw->iovdd)
			regulator_disable(pw->iovdd);
		if (pw->avdd)
			regulator_disable(pw->avdd);
	}

	pw->state = SWITCH_OFF;

	return 0;
}

static int imx219_crosslink_power_put(struct tegracam_device *tc_dev)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct camera_common_power_rail *pw = s_data->power;

	if (unlikely(!pw))
		return -EFAULT;

	if (likely(pw->dvdd))
		devm_regulator_put(pw->dvdd);

	if (likely(pw->avdd))
		devm_regulator_put(pw->avdd);

	if (likely(pw->iovdd))
		devm_regulator_put(pw->iovdd);

	pw->dvdd = NULL;
	pw->avdd = NULL;
	pw->iovdd = NULL;

	if (likely(pw->reset_gpio))
		gpio_free(pw->reset_gpio);

	return 0;
}

static int imx219_crosslink_power_get(struct tegracam_device *tc_dev)
{
	struct device *dev = tc_dev->dev;
	struct camera_common_data *s_data = tc_dev->s_data;
	struct camera_common_power_rail *pw = s_data->power;
	struct camera_common_pdata *pdata = s_data->pdata;
	struct clk *parent;
	int err = 0;

	if (!pdata) {
		dev_err(dev, "pdata missing\n");
		return -EFAULT;
	}

	/* Sensor MCLK (aka. INCK) */
	if (pdata->mclk_name) {
		pw->mclk = devm_clk_get(dev, pdata->mclk_name);
		if (IS_ERR(pw->mclk)) {
			dev_err(dev, "unable to get clock %s\n",
				pdata->mclk_name);
			return PTR_ERR(pw->mclk);
		}

		if (pdata->parentclk_name) {
			parent = devm_clk_get(dev, pdata->parentclk_name);
			if (IS_ERR(parent)) {
				dev_err(dev, "unable to get parent clock %s",
					pdata->parentclk_name);
			} else
				clk_set_parent(pw->mclk, parent);
		}
	}

	/* analog 2.8v */
	if (pdata->regulators.avdd)
		err |= camera_common_regulator_get(dev,
						   &pw->avdd,
						   pdata->regulators.avdd);
	/* IO 1.8v */
	if (pdata->regulators.iovdd)
		err |= camera_common_regulator_get(dev,
						   &pw->iovdd,
						   pdata->regulators.iovdd);
	/* dig 1.2v */
	if (pdata->regulators.dvdd)
		err |= camera_common_regulator_get(dev,
						   &pw->dvdd,
						   pdata->regulators.dvdd);
	if (err) {
		dev_err(dev, "%s: unable to get regulator(s)\n", __func__);
		goto done;
	}

	/* Reset or ENABLE GPIO */
	pw->reset_gpio = pdata->reset_gpio;
	err = gpio_request(pw->reset_gpio, "cam_reset_gpio");
	if (err < 0) {
		dev_err(dev, "%s: unable to request reset_gpio (%d)\n",
			__func__, err);
		goto done;
	}

done:
	pw->state = SWITCH_OFF;

	return err;
}

static struct camera_common_pdata *imx219_crosslink_parse_dt(struct tegracam_device
						   *tc_dev)
{
	struct device *dev = tc_dev->dev;
	struct device_node *np = dev->of_node;
	struct camera_common_pdata *board_priv_pdata;
	const struct of_device_id *match;
	struct camera_common_pdata *ret = NULL;
	int err = 0;
	int gpio;

	if (!np)
		return NULL;

	match = of_match_device(imx219_crosslink_of_match, dev);
	if (!match) {
		dev_err(dev, "Failed to find matching dt id\n");
		return NULL;
	}

	board_priv_pdata = devm_kzalloc(dev,
					sizeof(*board_priv_pdata), GFP_KERNEL);
	if (!board_priv_pdata)
		return NULL;

	gpio = of_get_named_gpio(np, "reset-gpios", 0);
	if (gpio < 0) {
		if (gpio == -EPROBE_DEFER)
			ret = ERR_PTR(-EPROBE_DEFER);
		dev_err(dev, "reset-gpios not found\n");
		goto error;
	}
	board_priv_pdata->reset_gpio = (unsigned int)gpio;

	err = of_property_read_string(np, "mclk", &board_priv_pdata->mclk_name);
	if (err)
		dev_dbg(dev, "mclk name not present, "
			"assume sensor driven externally\n");

	err = of_property_read_string(np, "avdd-reg",
				      &board_priv_pdata->regulators.avdd);
	err |= of_property_read_string(np, "iovdd-reg",
				       &board_priv_pdata->regulators.iovdd);
	err |= of_property_read_string(np, "dvdd-reg",
				       &board_priv_pdata->regulators.dvdd);
	if (err)
		dev_dbg(dev, "avdd, iovdd and/or dvdd reglrs. not present, "
			"assume sensor powered independently\n");

	board_priv_pdata->has_eeprom = of_property_read_bool(np, "has-eeprom");

	return board_priv_pdata;

error:
	devm_kfree(dev, board_priv_pdata);

	return ret;
}

static int imx219_crosslink_set_mode(struct tegracam_device *tc_dev)
{
	// struct imx219_crosslink *priv = (struct imx219_crosslink *)tegracam_get_privdata(tc_dev);
	// struct camera_common_data *s_data = tc_dev->s_data;

	// int err = 0;

	dev_dbg(tc_dev->dev, "%s:\n", __func__);

	// err = imx219_crosslink_write_table(priv, mode_table[IMX219_CROSSLINK_MODE_COMMON]);
	// if (err)
	// 	return err;

	// err = imx219_crosslink_write_table(priv, mode_table[s_data->mode]);
	// if (err)
	// 	return err;

	return 0;
}

static int imx219_crosslink_start_streaming(struct tegracam_device *tc_dev)
{
	// struct imx219_crosslink *priv = (struct imx219_crosslink *)tegracam_get_privdata(tc_dev);

	// dev_dbg(tc_dev->dev, "%s:\n", __func__);
	// return imx219_crosslink_write_table(priv, mode_table[IMX219_CROSSLINK_START_STREAM]);
	return 0;
}

static int imx219_crosslink_stop_streaming(struct tegracam_device *tc_dev)
{
	// int err;
	// struct imx219_crosslink *priv = (struct imx219_crosslink *)tegracam_get_privdata(tc_dev);

	// dev_dbg(tc_dev->dev, "%s:\n", __func__);
	// err = imx219_crosslink_write_table(priv, mode_table[IMX219_CROSSLINK_STOP_STREAM]);

	// return err;
	return 0;
}

static struct camera_common_sensor_ops imx219_crosslink_common_ops = {
	.numfrmfmts = ARRAY_SIZE(imx219_crosslink_frmfmt),
	.frmfmt_table = imx219_crosslink_frmfmt,
	.power_on = imx219_crosslink_power_on,
	.power_off = imx219_crosslink_power_off,
	.write_reg = imx219_crosslink_write_reg,
	.read_reg = imx219_crosslink_read_reg,
	.parse_dt = imx219_crosslink_parse_dt,
	.power_get = imx219_crosslink_power_get,
	.power_put = imx219_crosslink_power_put,
	.set_mode = imx219_crosslink_set_mode,
	.start_streaming = imx219_crosslink_start_streaming,
	.stop_streaming = imx219_crosslink_stop_streaming,
};

static int imx219_crosslink_board_setup(struct imx219_crosslink *priv)
{
	struct camera_common_data *s_data = priv->s_data;
	struct device *dev = s_data->dev;
	u8 reg_val[2];
	int err = 0;

	// Skip mclk enable as this camera has an internal oscillator

	err = imx219_crosslink_power_on(s_data);
	if (err) {
		dev_err(dev, "error during power on sensor (%d)\n", err);
		goto done;
	}

	/* Probe sensor model id registers */
	err = imx219_crosslink_read_reg(s_data, IMX219_CROSSLINK_MODEL_ID_ADDR_MSB, &reg_val[0]);
	if (err) {
		dev_err(dev, "%s: error during i2c read probe (%d)\n",
			__func__, err);
		goto err_reg_probe;
	}
	err = imx219_crosslink_read_reg(s_data, IMX219_CROSSLINK_MODEL_ID_ADDR_LSB, &reg_val[1]);
	if (err) {
		dev_err(dev, "%s: error during i2c read probe (%d)\n",
			__func__, err);
		goto err_reg_probe;
	}

	// if (!((reg_val[0] == 0x00) && reg_val[1] == 0x00))
	// 	dev_err(dev, "%s: invalid sensor model id: %x%x\n",
	// 		__func__, reg_val[0], reg_val[1]);

	/* Sensor fine integration time */
	// err = imx219_crosslink_get_fine_integ_time(priv, &priv->fine_integ_time);
	// if (err)
	// 	dev_err(dev, "%s: error querying sensor fine integ. time\n",
	// 		__func__);

	return 0;

err_reg_probe:
	imx219_crosslink_power_off(s_data);

done:
	return err;
}

static int imx219_crosslink_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	dev_dbg(&client->dev, "%s:\n", __func__);

	return 0;
}

static const struct v4l2_subdev_internal_ops imx219_crosslink_subdev_internal_ops = {
	.open = imx219_crosslink_open,
};

static int imx219_crosslink_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct tegracam_device *tc_dev;
	struct imx219_crosslink *priv;
	int err;

	dev_dbg(dev, "probing v4l2 sensor at addr 0x%0x\n", client->addr);

	if (!IS_ENABLED(CONFIG_OF) || !client->dev.of_node)
		return -EINVAL;

	priv = devm_kzalloc(dev, sizeof(struct imx219_crosslink), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	tc_dev = devm_kzalloc(dev, sizeof(struct tegracam_device), GFP_KERNEL);
	if (!tc_dev)
		return -ENOMEM;

	priv->i2c_client = tc_dev->client = client;
	tc_dev->dev = dev;
	strncpy(tc_dev->name, "imx219_crosslink", sizeof(tc_dev->name));
	tc_dev->dev_regmap_config = &sensor_regmap_config;
	tc_dev->sensor_ops = &imx219_crosslink_common_ops;
	tc_dev->v4l2sd_internal_ops = &imx219_crosslink_subdev_internal_ops;
	tc_dev->tcctrl_ops = &imx219_crosslink_ctrl_ops;

	err = tegracam_device_register(tc_dev);
	if (err) {
		dev_err(dev, "tegra camera driver registration failed\n");
		return err;
	}
	priv->tc_dev = tc_dev;
	priv->s_data = tc_dev->s_data;
	priv->subdev = &tc_dev->s_data->subdev;
	tegracam_set_privdata(tc_dev, (void *)priv);

	err = imx219_crosslink_board_setup(priv);
	if (err) {
		dev_err(dev, "board setup failed\n");
		return err;
	}

	err = tegracam_v4l2subdev_register(tc_dev, true);
	if (err) {
		dev_err(dev, "tegra camera subdev registration failed\n");
		return err;
	}

	dev_dbg(dev, "detected imx219_crosslink sensor\n");

	return 0;
}

static int imx219_crosslink_remove(struct i2c_client *client)
{
	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	struct imx219_crosslink *priv = (struct imx219_crosslink *)s_data->priv;

	tegracam_v4l2subdev_unregister(priv->tc_dev);
	tegracam_device_unregister(priv->tc_dev);

	return 0;
}

static const struct i2c_device_id imx219_crosslink_id[] = {
	{"imx219_crosslink", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, imx219_crosslink_id);

static struct i2c_driver imx219_crosslink_i2c_driver = {
	.driver = {
		   .name = "imx219_crosslink",
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(imx219_crosslink_of_match),
		   },
	.probe = imx219_crosslink_probe,
	.remove = imx219_crosslink_remove,
	.id_table = imx219_crosslink_id,
};

module_i2c_driver(imx219_crosslink_i2c_driver);

MODULE_DESCRIPTION("Media Controller driver for IMX219 Crosslink");
MODULE_AUTHOR("jarsulk, p2119, pco");
MODULE_LICENSE("GPL v2");
