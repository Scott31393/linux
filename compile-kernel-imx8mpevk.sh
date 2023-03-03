source env-imx8.sh

make imx8mp_evk_defconfig
make Image dtbs modules

sudo rm -r MODULES
mkdir MODULES
make -j16 modules_install CROSS_COMPILE=$CROSS_COMPILER ARCH=arm64 INSTALL_MOD_PATH=./MODULES


cp arch/arm64/boot/Image /tftp-folder/
cp arch/arm64/boot/dts/freescale/imx8mp-evk.dtb /tftp-folder/
cp arch/arm64/boot/dts/freescale/imx8mp-evk-isi0-ov5640.dtb /tftp-folder/imx8mp-evk-isi0-ov5640.dtbo
cp arch/arm64/boot/dts/freescale/imx8mp-evk-isp0-ov5640.dtb /tftp-folder/imx8mp-evk-isp0-ov5640.dtbo
cp arch/arm64/boot/dts/freescale/imx8mp-evk-isi0-alvium.dtb /tftp-folder/imx8mp-evk-isi0-alvium.dtbo
sudo cp -r MODULES/lib/ /targetfs/
