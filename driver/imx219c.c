/*
 * imx219c.c - imx219c parallel to CSI-2 driver
 */

#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/seq_file.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>

#include <media/tegra_v4l2_camera.h>
#include <media/tegracam_core.h>
#include <media/imx219c.h>

// #include "../../../../nvidia/drivers/media/i2c/../platform/tegra/camera/camera_gpio.h"

/* imx219c sensor register address */
#define IMX219C_MODEL_ID_ADDR_MSB		0x0000
#define IMX219C_MODEL_ID_ADDR_LSB		0x0001

static const struct of_device_id imx219c_of_match[] = {
	{.compatible = "pco,imx219c",},
	{ },
};
MODULE_DEVICE_TABLE(of, imx219c_of_match);

static const u32 ctrl_cid_list[] = {
	TEGRA_CAMERA_CID_GAIN,
	TEGRA_CAMERA_CID_EXPOSURE,
	TEGRA_CAMERA_CID_FRAME_RATE,
	TEGRA_CAMERA_CID_SENSOR_MODE_ID,
};

struct imx219c {
	struct i2c_client		*i2c_client;
	struct v4l2_subdev		*subdev;
	u16				fine_integ_time;
	u32				frame_length;
	struct camera_common_data	*s_data;
	struct tegracam_device		*tc_dev;
};

static const struct regmap_config sensor_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.cache_type = REGCACHE_RBTREE,
	.use_single_rw = true,
};

static inline int imx219c_read_reg(struct camera_common_data *s_data,
	u16 addr, u8 *val)
{
	int err = 0;
	u32 reg_val = 0;

	err = regmap_read(s_data->regmap, addr, &reg_val);
	*val = reg_val & 0xff;

	return err;
}

static inline int imx219c_write_reg(struct camera_common_data *s_data,
	u16 addr, u8 val)
{
	int err = 0;

	err = regmap_write(s_data->regmap, addr, val);
	if (err)
		dev_err(s_data->dev, "%s: i2c write failed, 0x%x = %x",
			__func__, addr, val);

	return err;
}

static int imx219c_set_group_hold(struct tegracam_device *tc_dev, bool val)
{
	return 0;
}

static int imx219c_set_gain(struct tegracam_device *tc_dev, s64 val)
{
	return 0;
}

static int imx219c_set_frame_rate(struct tegracam_device *tc_dev, s64 val)
{
	return 0;
}

static int imx219c_set_exposure(struct tegracam_device *tc_dev, s64 val)
{
	return 0;
}

static struct tegracam_ctrl_ops imx219c_ctrl_ops = {
	.numctrls = ARRAY_SIZE(ctrl_cid_list),
	.ctrl_cid_list = ctrl_cid_list,
	.set_gain = imx219c_set_gain,
	.set_exposure = imx219c_set_exposure,
	.set_frame_rate = imx219c_set_frame_rate,
	.set_group_hold = imx219c_set_group_hold,
};

static int imx219c_power_on(struct camera_common_data *s_data)
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

// 	if (pw->reset_gpio) {
// 		if (gpio_cansleep(pw->reset_gpio))
// 			gpio_set_value_cansleep(pw->reset_gpio, 0);
// 		else
// 			gpio_set_value(pw->reset_gpio, 0);
// 	}

// 	usleep_range(10, 20);

// skip_power_seqn:
// 	if (pw->reset_gpio) {
// 		if (gpio_cansleep(pw->reset_gpio))
// 			gpio_set_value_cansleep(pw->reset_gpio, 1);
// 		else
// 			gpio_set_value(pw->reset_gpio, 1);
// 	}

	/* Need to wait for t4 + t5 + t9 time as per the data sheet */
	/* t4 - 200us, t5 - 21.2ms, t9 - 1.2ms */
	// usleep_range(23000, 23100);

	pw->state = SWITCH_ON;

	return 0;
}

static int imx219c_power_off(struct camera_common_data *s_data)
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
	// } else {
	// 	if (pw->reset_gpio) {
	// 		if (gpio_cansleep(pw->reset_gpio))
	// 			gpio_set_value_cansleep(pw->reset_gpio, 0);
	// 		else
	// 			gpio_set_value(pw->reset_gpio, 0);
	// 	}

		usleep_range(10, 10);
	}

	pw->state = SWITCH_OFF;

	return 0;
}

static int imx219c_power_put(struct tegracam_device *tc_dev)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct camera_common_power_rail *pw = s_data->power;

	if (unlikely(!pw))
		return -EFAULT;

	// if (likely(pw->reset_gpio))
	// 	gpio_free(pw->reset_gpio);

	return 0;
}

static int imx219c_power_get(struct tegracam_device *tc_dev)
{
	struct camera_common_data *s_data = tc_dev->s_data;
	struct camera_common_power_rail *pw = s_data->power;

	pw->state = SWITCH_OFF;

	return 0;
}

static struct camera_common_pdata *imx219c_parse_dt(
	struct tegracam_device *tc_dev)
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

	match = of_match_device(imx219c_of_match, dev);
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

	board_priv_pdata->has_eeprom =
		of_property_read_bool(np, "has-eeprom");

	return board_priv_pdata;

error:
	devm_kfree(dev, board_priv_pdata);

	return ret;
}

static int imx219c_set_mode(struct tegracam_device *tc_dev)
{
	// struct imx219c *priv = (struct imx219c *)tegracam_get_privdata(tc_dev);
	// struct camera_common_data *s_data = tc_dev->s_data;
	return 0;
}

static int imx219c_start_streaming(struct tegracam_device *tc_dev)
{
	return 0;
}

static int imx219c_stop_streaming(struct tegracam_device *tc_dev)
{
	return 0;
}

static struct camera_common_sensor_ops imx219c_common_ops = {
	.numfrmfmts = ARRAY_SIZE(imx219c_frmfmt),
	.frmfmt_table = imx219c_frmfmt,
	.power_on = imx219c_power_on,
	.power_off = imx219c_power_off,
	.write_reg = imx219c_write_reg,
	.read_reg = imx219c_read_reg,
	.parse_dt = imx219c_parse_dt,
	.power_get = imx219c_power_get,
	.power_put = imx219c_power_put,
	.set_mode = imx219c_set_mode,
	.start_streaming = imx219c_start_streaming,
	.stop_streaming = imx219c_stop_streaming,
};

static int imx219c_board_setup(struct imx219c *priv)
{
	struct camera_common_data *s_data = priv->s_data;
	struct camera_common_pdata *pdata = s_data->pdata;
	struct device *dev = s_data->dev;
	u8 reg_val[2];
	int err = 0;

	if (pdata->mclk_name) {
		err = camera_common_mclk_enable(s_data);
		if (err) {
			dev_err(dev, "error turning on mclk (%d)\n", err);
			goto done;
		}
	}

	err = imx219c_power_on(s_data);
	if (err) {
		dev_err(dev, "error during power on sensor (%d)\n", err);
		goto err_power_on;
	}

	/* Probe sensor model id registers */
	err = imx219c_read_reg(s_data, IMX219C_MODEL_ID_ADDR_MSB, &reg_val[0]);
	if (err) {
		dev_err(dev, "%s: error during i2c read probe (%d)\n",
			__func__, err);
		goto err_reg_probe;
	}
	err = imx219c_read_reg(s_data, IMX219C_MODEL_ID_ADDR_LSB, &reg_val[1]);
	if (err) {
		dev_err(dev, "%s: error during i2c read probe (%d)\n",
			__func__, err);
		goto err_reg_probe;
	}
	// if (!((reg_val[0] == 0x02) && reg_val[1] == 0x19))
	// 	dev_err(dev, "%s: invalid sensor model id: %x%x\n",
	// 		__func__, reg_val[0], reg_val[1]);

err_reg_probe:
	imx219c_power_off(s_data);

err_power_on:
	if (pdata->mclk_name)
		camera_common_mclk_disable(s_data);

done:
	return err;
}

static int imx219c_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	dev_dbg(&client->dev, "%s:\n", __func__);

	return 0;
}

static const struct v4l2_subdev_internal_ops imx219c_subdev_internal_ops = {
	.open = imx219c_open,
};

static int imx219c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct tegracam_device *tc_dev;
	struct imx219c *priv;
	int err;

	dev_dbg(dev, "probing v4l2 sensor at addr 0x%0x\n", client->addr);

	if (!IS_ENABLED(CONFIG_OF) || !client->dev.of_node)
		return -EINVAL;

	priv = devm_kzalloc(dev,
			sizeof(struct imx219c), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	tc_dev = devm_kzalloc(dev,
			sizeof(struct tegracam_device), GFP_KERNEL);
	if (!tc_dev)
		return -ENOMEM;

	priv->i2c_client = tc_dev->client = client;
	tc_dev->dev = dev;
	strncpy(tc_dev->name, "imx219c", sizeof(tc_dev->name));
	tc_dev->dev_regmap_config = &sensor_regmap_config;
	tc_dev->sensor_ops = &imx219c_common_ops;
	tc_dev->v4l2sd_internal_ops = &imx219c_subdev_internal_ops;
	tc_dev->tcctrl_ops = &imx219c_ctrl_ops;

	err = tegracam_device_register(tc_dev);
	if (err) {
		dev_err(dev, "tegra camera driver registration failed\n");
		return err;
	}
	priv->tc_dev = tc_dev;
	priv->s_data = tc_dev->s_data;
	priv->subdev = &tc_dev->s_data->subdev;
	tegracam_set_privdata(tc_dev, (void *)priv);

	err = imx219c_board_setup(priv);
	if (err) {
		tegracam_device_unregister(tc_dev);
		dev_err(dev, "board setup failed\n");
		return err;
	}

	err = tegracam_v4l2subdev_register(tc_dev, true);
	if (err) {
		dev_err(dev, "tegra camera subdev registration failed\n");
		return err;
	}

	dev_dbg(dev, "detected imx219c sensor\n");

	return 0;
}

static int imx219c_remove(struct i2c_client *client)
{
	struct camera_common_data *s_data = to_camera_common_data(&client->dev);
	struct imx219c *priv = (struct imx219c *)s_data->priv;

	tegracam_v4l2subdev_unregister(priv->tc_dev);
	tegracam_device_unregister(priv->tc_dev);

	return 0;
}

static const struct i2c_device_id imx219c_id[] = {
	{ "imx219c", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, imx219c_id);

static struct i2c_driver imx219c_i2c_driver = {
	.driver = {
		.name = "imx219c",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(imx219c_of_match),
	},
	.probe = imx219c_probe,
	.remove = imx219c_remove,
	.id_table = imx219c_id,
};
module_i2c_driver(imx219c_i2c_driver);

MODULE_DESCRIPTION("Media Controller driver for IMX219C Crosslink");
MODULE_AUTHOR("jarsulk, p2119, pco");
MODULE_LICENSE("GPL v2");
