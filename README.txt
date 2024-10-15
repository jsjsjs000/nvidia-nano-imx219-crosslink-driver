# -------------------- Device Tree backup --------------------
backup=~/nvidia-nano-imx219-crosslink-driver
mkdir -p $backup/dts/
cp ~/l4t-gcc/Linux_for_Tegra/source/public/hardware/nvidia/platform/t210/porg/kernel-dts/tegra210-p3448-all-p3449-0000-imx219-crosslink.dts $backup/dts/
cp ~/l4t-gcc/Linux_for_Tegra/source/public/hardware/nvidia/platform/t210/porg/kernel-dts/porg-platforms/tegra210-porg-camera-rbpcv5-dual-imx219-crosslink.dtsi $backup/dts/
cp ~/l4t-gcc/Linux_for_Tegra/source/public/hardware/nvidia/platform/t210/porg/kernel-dts/porg-platforms/tegra210-camera-rbpcv5-dual-imx219-crosslink.dtsi $backup/dts/

# -------------------- Driver backup --------------------
backup=~/nvidia-nano-imx219-crosslink-driver
mkdir -p $backup/driver/
cp ~/l4t-gcc/Linux_for_Tegra/source/public/kernel/kernel-4.9/drivers/media/i2c/imx219c.c $backup/driver/
cp ~/l4t-gcc/Linux_for_Tegra/source/public/kernel/nvidia/include/media/imx219c.h $backup/driver/


# -------------------- Device Tree restore --------------------
backup=~/nvidia-nano-imx219-crosslink-driver
cp $backup/dts/tegra210-p3448-all-p3449-0000-imx219-crosslink.dts     ~/l4t-gcc/Linux_for_Tegra/source/public/hardware/nvidia/platform/t210/porg/kernel-dts/
cp $backup/dts/tegra210-porg-camera-rbpcv5-dual-imx219-crosslink.dtsi ~/l4t-gcc/Linux_for_Tegra/source/public/hardware/nvidia/platform/t210/porg/kernel-dts/porg-platforms/
cp $backup/dts/tegra210-camera-rbpcv5-dual-imx219-crosslink.dtsi      ~/l4t-gcc/Linux_for_Tegra/source/public/hardware/nvidia/platform/t210/porg/kernel-dts/porg-platforms/

# -------------------- Driver restore --------------------
backup=~/nvidia-nano-imx219-crosslink-driver
cp $backup/driver/imx219c.c ~/l4t-gcc/Linux_for_Tegra/source/public/kernel/kernel-4.9/drivers/media/i2c/
cp $backup/driver/imx219c.h ~/l4t-gcc/Linux_for_Tegra/source/public/kernel/nvidia/include/media/


# -------------------- Backup patch --------------------
backup=~/nvidia-nano-tc358748-driver
mkdir -p $backup/patch/
cp ~/l4t-gcc/Linux_for_Tegra/source/public/kernel/kernel-4.9/0001-regmap-add-formats.patch $backup/patch/

# -------------------- Apply patch --------------------
cd kernel/kernel-4.9/
cp $backup/patch/0001-regmap-add-formats.patch .
git apply 0001-regmap-add-formats.patch
cd -


# -------------------- Device Tree - Kernel setup --------------------
code ~/l4t-gcc/Linux_for_Tegra/source/public/hardware/nvidia/platform/t210/porg/kernel-dts/Makefile
# ----------------------------------------
dtbo-$(CONFIG_ARCH_TEGRA_210_SOC) += tegra210-p3448-all-p3449-0000-imx219-crosslink.dtbo
# ----------------------------------------

code ~/l4t-gcc/Linux_for_Tegra/source/public/hardware/nvidia/platform/t210/porg/kernel-dts/tegra210-p3448-0000-p3449-0000-b00.dts
# ----------------------------------------
#include "porg-platforms/tegra210-porg-camera-rbpcv5-dual-imx219-crosslink.dtsi"
# ----------------------------------------

# -------------------- Driver - Kernel setup --------------------
code ~/l4t-gcc/Linux_for_Tegra/source/public/kernel/kernel-4.9/drivers/media/i2c/Makefile
# ----------------------------------------
obj-$(CONFIG_VIDEO_IMX219_CROSSLINK) += imx219c.o
# ----------------------------------------

code ~/l4t-gcc/Linux_for_Tegra/source/public/kernel/kernel-4.9/drivers/media/i2c/Kconfig
# ----------------------------------------
config VIDEO_IMX219_CROSSLINK
        tristate "IMX219_CROSSLINK camera sensor imitation on Crosslink"
	depends on I2C && VIDEO_V4L2 && VIDEO_V4L2_SUBDEV_API
	---help---
	  This driver supports IMX219 CROSSLINK camera sensor imitation on Crosslink

	  To compile this driver as a module, choose M here: the module
	  will be called imx219.
# ----------------------------------------

l4t  # set environments

l4t_menuconfig
# ----------------------------------------
/ imx219 = [n]  # reserve device on I2C address 0x10
/ imx219c = [m]
# ----------------------------------------

alias l4t_upload='scp build/arch/arm64/boot/dts/tegra210-p3448-all-p3449-0000-imx219-crosslink.dtbo root@192.168.3.12:/boot/ && scp build/arch/arm64/boot/dts/tegra210-p3448-0000-p3449-0000-b00.dtb root@192.168.3.12:/boot/ && scp build/arch/arm64/boot/Image root@192.168.3.12:/boot/ && scp build/drivers/media/i2c/imx219c.ko root@192.168.3.12:/lib/modules/4.9.253-tegra/kernel/drivers/media/i2c/'

clear; l4t_zImage && l4t_dtbs && l4t_modules && l4t_upload

ll build/arch/arm64/boot/dts/tegra210-p3448-all-p3449-0000-imx219-crosslink.dtbo
scp build/arch/arm64/boot/dts/tegra210-p3448-all-p3449-0000-imx219-crosslink.dtbo root@192.168.3.12:/boot/
ll build/arch/arm64/boot/dts/tegra210-p3448-0000-p3449-0000-b00.dtb
scp build/arch/arm64/boot/dts/tegra210-p3448-0000-p3449-0000-b00.dtb root@192.168.3.12:/boot/

ll build/arch/arm64/boot/Image
scp build/arch/arm64/boot/Image root@192.168.3.12:/boot/
ll build/drivers/media/i2c/imx219c.ko
scp build/drivers/media/i2c/imx219c.ko root@192.168.3.12:/lib/modules/4.9.253-tegra/kernel/drivers/media/i2c/

	# nVidia
depmod

dtsfilename=$(tr -d '\0' < "/proc/device-tree/nvidia,dtsfilename")
dtbfilename=$(basename -s .dts "$dtsfilename")

# echo $dtsfilename
# /dvs/git/dirty/git-master_linux/kernel/kernel-4.9/arch/arm64/boot/dts/../../../../../../hardware/nvidia/platform/t210/porg/kernel-dts/tegra210-p3448-0000-p3449-0000-b00.dts

echo $dtbfilename
# tegra210-p3448-0000-p3449-0000-b00

cp "/boot/$dtbfilename.dtb" "/boot/dtb/kernel_$dtbfilename.dtb"
cp "/boot/$dtbfilename.dtb" "/boot/kernel_$dtbfilename.dtb"

# equivalent of $ sudo /opt/nvidia/jetson-io/jetson-io.py
fdtoverlay -i "/boot/dtb/kernel_$dtbfilename.dtb" \
  -o "/boot/kernel_$dtbfilename-imx219-crosslink.dtb" \
  /boot/tegra210-p3448-all-p3449-0000-imx219-crosslink.dtbo

python3 - << PYTHON_EOF
import sys
sys.path.append('/opt/nvidia/jetson-io')
from Linux.extlinux import add_entry
add_entry('/boot/extlinux/extlinux.conf',
         'Jetson-IMX477-Crosslink', 'IMX477-CROSSLINK',
         '/boot/kernel_$dtbfilename-imx219-crosslink.dtb',
         True)
PYTHON_EOF

# nano /boot/extlinux/extlinux.conf

reboot

-------------------------------------------------------------------
-------------------------------------------------------------------
	# działa 640x480x60
code ~/l4t-gcc/Linux_for_Tegra/source/public/kernel/nvidia/drivers/media/i2c/imx219_mode_tbls.h
# ----------------------------------------
static const struct camera_common_frmfmt imx219_frmfmt[] = {
	// {{3264, 2464},	imx219_21fps, 1, 0, IMX219_MODE_640x480_60FPS},
	{{640, 480},	imx219_60fps, 1, 0, IMX219_MODE_640x480_60FPS}, // $$
# ----------------------------------------

DSIPLAY=:0.0 gst-launch-1.0 nvarguscamerasrc ! 'video/x-raw(memory:NVMM), width=640, height=480, format=(string)NV12, framerate=(fraction)60/1' ! fpsdisplaysink -v text-overlay=0 video-sink="nvoverlaysink overlay-x=100 overlay-y=100 overlay-w=640 overlay-h=480"

ls /dev/v4l-subdev*
#> /dev/v4l-subdev0  /dev/v4l-subdev1

ls /dev/v4l/by-path/platform-54080000.vi-video-index0
#> /dev/v4l/by-path/platform-54080000.vi-video-index0



-------------------------------------------------------------------
	# działa - moduł wkompilowany w Kernel
code ~/l4t-gcc/Linux_for_Tegra/source/public/hardware/nvidia/platform/t210/porg/kernel-dts/tegra210-p3448-0000-p3449-0000-b00.dts
# ----------------------------------------
#include "porg-platforms/tegra210-porg-camera-rbpcv5-dual-imx219-crosslink.dtsi"
// #include "porg-platforms/tegra210-porg-camera-rbpcv3-dual-imx477.dtsi"
// #include "porg-platforms/tegra210-porg-camera-rbpcv2-dual-imx219.dtsi"
# ----------------------------------------

nano /boot/extlinux/extlinux.conf
# ----------------------------------------
DEFAULT Jetson
LABEL Jetson
	MENU LABEL primary kernel Jetson
	LINUX /boot/Image
	FDT /boot/tegra210-p3448-0000-p3449-0000-b00.dtb
	INITRD /boot/initrd
	APPEND ${cbootargs} quiet root=/dev/mmcblk0p1 rw rootwait rootfstype=ext4 console=ttyS0,115$
# ----------------------------------------
