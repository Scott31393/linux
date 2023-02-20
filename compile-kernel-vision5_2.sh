export ARCH=riscv
export CROSS_COMPILE=/home/tom/work/starfive/VisionFive2/work/buildroot_initramfs/host/bin/riscv64-buildroot-linux-gnu-

make defconfig
make Image dtbs modules
make -j20

sudo rm -r MODULES
mkdir MODULES
make -j16 modules_install CROSS_COMPILE=$CROSS_COMPILER ARCH=riscv INSTALL_MOD_PATH=./MODULES


cp arch/riscv/boot/Image.gz /tftp-folder/
cp arch/riscv/boot/dts/starfive/jh7110-starfive-visionfive-2-v1.3b.dtb /tftp-folder/
sudo cp -r MODULES/lib/ /targetfs/
