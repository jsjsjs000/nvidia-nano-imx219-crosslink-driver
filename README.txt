# -------------------- Device Tree backup --------------------
backup=~/nvidia-nano-imx219-crosslink
mkdir -p $backup/dts/
cp ~/l4t-gcc/Linux_for_Tegra/source/public/hardware/nvidia/platform/t210/porg/kernel-dts/tegra210-p3448-all-p3449-0000-imx219-crosslink.dts $backup/dts/
cp ~/l4t-gcc/Linux_for_Tegra/source/public/hardware/nvidia/platform/t210/porg/kernel-dts/porg-platforms/tegra210-porg-camera-rbpcv5-dual-imx219-crosslink.dtsi $backup/dts/
cp ~/l4t-gcc/Linux_for_Tegra/source/public/hardware/nvidia/platform/t210/porg/kernel-dts/porg-platforms/tegra210-camera-rbpcv5-dual-imx219-crosslink.dtsi $backup/dts/

# -------------------- Driver backup --------------------
backup=~/nvidia-nano-imx219-crosslink
mkdir -p $backup/driver/
cp ~/l4t-gcc/Linux_for_Tegra/source/public/kernel/kernel-4.9/drivers/media/i2c/imx219_crosslink.c $backup/driver/
cp ~/l4t-gcc/Linux_for_Tegra/source/public/kernel/nvidia/include/media/imx219_crosslink.h $backup/driver/

# -------------------- Device Tree restore --------------------
backup=~/nvidia-nano-imx219-crosslink
cp $backup/dts/tegra210-p3448-all-p3449-0000-imx219-crosslink.dts     ~/l4t-gcc/Linux_for_Tegra/source/public/hardware/nvidia/platform/t210/porg/kernel-dts/
cp $backup/dts/tegra210-porg-camera-rbpcv5-dual-imx219-crosslink.dtsi ~/l4t-gcc/Linux_for_Tegra/source/public/hardware/nvidia/platform/t210/porg/kernel-dts/porg-platforms/
cp $backup/dts/tegra210-camera-rbpcv5-dual-imx219-crosslink.dtsi      ~/l4t-gcc/Linux_for_Tegra/source/public/hardware/nvidia/platform/t210/porg/kernel-dts/porg-platforms/

# -------------------- Driver restore --------------------
backup=~/nvidia-nano-imx219-crosslink
cp $backup/driver/imx219_crosslink.c ~/l4t-gcc/Linux_for_Tegra/source/public/kernel/kernel-4.9/drivers/media/i2c/
cp $backup/driver/imx219_crosslink.h ~/l4t-gcc/Linux_for_Tegra/source/public/kernel/nvidia/include/media/


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
obj-$(CONFIG_VIDEO_IMX219_CROSSLINK) += imx219_crosslink.o
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
/ imx219_crosslink = [m]
# ----------------------------------------

c; l4t_zImage && l4t_dtbs && l4t_modules

ll build/arch/arm64/boot/dts/tegra210-p3448-all-p3449-0000-imx219-crosslink.dtbo
scp build/arch/arm64/boot/dts/tegra210-p3448-all-p3449-0000-imx219-crosslink.dtbo root@192.168.3.12:/boot/
ll build/arch/arm64/boot/dts/tegra210-p3448-0000-p3449-0000-b00.dtb
scp build/arch/arm64/boot/dts/tegra210-p3448-0000-p3449-0000-b00.dtb root@192.168.3.12:/boot/

ll build/arch/arm64/boot/Image
scp build/arch/arm64/boot/Image root@192.168.3.12:/boot/
ll build/drivers/media/i2c/imx219_crosslink.ko
scp build/drivers/media/i2c/imx219_crosslink.ko root@192.168.3.12:/lib/modules/4.9.253-tegra/kernel/drivers/media/i2c/

	# nVidia
depmod

dtsfilename=$(tr -d '\0' < "/proc/device-tree/nvidia,dtsfilename")
dtbfilename=$(basename -s .dts "$dtsfilename")

echo $dtsfilename
#/dvs/git/dirty/git-master_linux/kernel/kernel-4.9/arch/arm64/boot/dts/../../../../../../hardware/nvidia/platform/t210/porg/kernel-dts/tegra210-p3448-0000-p3449-0000-b00.dts

echo $dtbfilename 
# tegra210-p3448-0000-p3449-0000-b00

cp "/boot/$dtbfilename.dtb" "/boot/dtb/kernel_$dtbfilename.dtb"
cp "/boot/$dtbfilename.dtb" "/boot/kernel_$dtbfilename.dtb"

# equivalent of $ sudo /opt/nvidia/jetson-io/jetson-io.py
fdtoverlay -i "/boot/dtb/kernel_$dtbfilename.dtb" -o "/boot/kernel_$dtbfilename-imx219-crosslink.dtb" /boot/tegra210-p3448-all-p3449-0000-imx219-crosslink.dtbo

python3 - << PYTHON_EOF
import sys
sys.path.append('/opt/nvidia/jetson-io')
from Linux.extlinux import add_entry
add_entry('/boot/extlinux/extlinux.conf',
         'Jetson-IMX477-Crosslink', 'Custom Header Config: <IMX477-CROSSLINK>',
         '/boot/kernel_$dtbfilename-imx219-crosslink.dtb',
         True)
PYTHON_EOF

# nano /boot/extlinux/extlinux.conf

reboot
