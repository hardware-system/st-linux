#!/bin/bash

source /opt/st/stm32mp1/4.2.1-openstlinux-6.1-yocto-mickledore-mp1-v23.06.21/environment-setup-cortexa7t2hf-neon-vfpv4-ostl-linux-gnueabi

BUILD_DIR="$PWD/../build"
DEPLOY_DIR="$PWD/../deploy"

BUILD_DTS="$BUILD_DIR/arch/arm/boot/dts/stm32mp135d-ici.dtb"
BUILD_IMG="$BUILD_DIR/arch/arm/boot/uImage"

set -e
#set -x

if [ ! -e "$BUILD_DIR" ]; then
	    mkdir $BUILD_DIR
fi

if [ ! -e "$DEPLOY_DIR" ]; then
	    mkdir $DEPLOY_DIR
fi

if [[ "$CLEAN" == "clean" ]]; then
	echo "clean kernel"
	make distclean #清除编译
	exit 0
fi

if [ ! -f $BUILD_DIR/.config ]
then
	make ARCH=arm O="${BUILD_DIR}" multi_v7_defconfig fragment*.config
fi

make ARCH=arm uImage vmlinux dtbs LOADADDR=0xC2000040 O="${BUILD_DIR}" -j$(($(nproc)+1))

#编译内核模块
make ARCH=arm O="${BUILD_DIR}" modules -j$(($(nproc)+1))

#将编译好的模块安装到build目录，通过INSTALL_MOD_STRIP=1移除模块调试信息
make ARCH=arm O="${BUILD_DIR}" modules_install INSTALL_MOD_PATH="$BUILD_DIR" INSTALL_MOD_STRIP=1 -j$(($(nproc)+1))
#删除模块目录下的source目录
rm -rf "$BUILD_DIR/lib/modules/6.1.28/source"
#删除模块的目录下的build目录
rm -rf "$BUILD_DIR/lib/modules/6.1.28/build"

#拷贝uImage到$BUILD_DIR目录
cp $BUILD_DIR/arch/arm/boot/uImage "$DEPLOY_DIR"

#拷贝所有编译的设备树文件到当前的../build目录下
cp $BUILD_DIR/arch/arm/boot/dts/stm32mp135d-ici.dtb "$DEPLOY_DIR"
cp $BUILD_DIR/arch/arm/boot/dts/stm32mp131d-ici.dtb "$DEPLOY_DIR"

if [[ -d $DEPLOY_DIR/6.1.28 ]];
then
	rm -r $DEPLOY_DIR/6.1.28
fi
cp -r $BUILD_DIR/lib/modules/6.1.28 "$DEPLOY_DIR"

echo "编译完成请查看 "$DEPLOY_DIR" 目录"
