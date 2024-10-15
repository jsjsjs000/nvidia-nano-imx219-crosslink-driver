#ifndef __IMX219C_H__
#define __IMX219C_H__

struct imx219c_power_rail {
	struct regulator *dvdd;
	struct regulator *avdd;
	struct regulator *iovdd;
	struct regulator *vdd_af;
};

enum {
	IMX219C_MODE_3264x2464_21FPS,
	IMX219_MODE_640x480_60FPS,

	IMX219C_MODE_COMMON,
	IMX219C_START_STREAM,
	IMX219C_STOP_STREAM,
};

static const int imx219c_21fps[] = {
	21,
};

static const int imx219c_60fps[] = {
	60,
};

/*
 * WARNING: frmfmt ordering need to match mode definition in
 * device tree!
 */
static const struct camera_common_frmfmt imx219c_frmfmt[] = {
	{{3264, 2464},	imx219c_21fps, 1, 0, IMX219C_MODE_3264x2464_21FPS},
	{{ 640,  480},	imx219c_60fps, 1, 0, IMX219_MODE_640x480_60FPS}
};

#endif  /* __IMX219C_H__ */
