export PATH=/opt/toolchain-rpi/gcc-linaro-7.5.0-2019.12-x86_64_aarch64-linux-gnu/bin:$PATH

make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- px30_evb_v11_20190507_defconfig
make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- -j$(nproc)

rm -rf MODULES
mkdir MODULES
make -j16 modules_install ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- INSTALL_MOD_PATH=./MODULES

cp arch/arm64/boot/Image /tftp-folder/
cp arch/arm64/boot/dts/rockchip/px30-evb.dtb /tftp-folder/
sudo cp -vr MODULES/lib/modules/ /targetfs/lib/

# cp u-boot.bin /media/tom/6ED8-61CA/kernel8.img