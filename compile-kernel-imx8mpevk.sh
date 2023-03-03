source env-imx8.sh

#make imx8mmc61_defconfig
make defconfig
make Image dtbs modules

sudo rm -r MODULES
mkdir MODULES
make -j16 modules_install CROSS_COMPILE=$CROSS_COMPILER ARCH=arm64 INSTALL_MOD_PATH=./MODULES


cp arch/arm64/boot/Image /tftp-folder/
cp arch/arm64/boot/dts/freescale/imx8mp-evk.dtb /tftp-folder/imx8mp-evk.dtb
sudo cp -r MODULES/lib/ /targetfs/
