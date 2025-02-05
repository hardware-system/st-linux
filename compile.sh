#!/bin/bash

source /opt/st/stm32mp1/5.0.3-openstlinux-6.6-yocto-scarthgap-mpu-v24.11.06/environment-setup

BUILD_DIR="$PWD/build"
DEPLOY_DIR="$PWD/deploy"

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
make ARCH=arm O="${BUILD_DIR}" modules_install INSTALL_MOD_PATH="$BUILD_DIR" INSTALL_MOD_STRIP=1 -j$(($(nproc)+1)) > /dev/null
#删除模块目录下的source目录
rm -rf "$BUILD_DIR/lib/modules/6.6.48/source"
#删除模块的目录下的build目录
rm -rf "$BUILD_DIR/lib/modules/6.6.48/build"

#拷贝uImage到$BUILD_DIR目录
cp $BUILD_DIR/arch/arm/boot/uImage "$DEPLOY_DIR"

#拷贝所有编译的设备树文件到当前的../build目录下
cp $BUILD_DIR/arch/arm/boot/dts/st/stm32mp135d-ici.dtb "$DEPLOY_DIR"
cp $BUILD_DIR/arch/arm/boot/dts/st/stm32mp131d-ici.dtb "$DEPLOY_DIR"
cp $BUILD_DIR/arch/arm/boot/dts/st/stm32mp135d-ici-fgc1k.dtb "$DEPLOY_DIR"

if [[ -d $DEPLOY_DIR/6.6.48 ]];
then
	rm -r $DEPLOY_DIR/6.6.48
fi
cp -r $BUILD_DIR/lib/modules/6.6.48 "$DEPLOY_DIR"

echo "编译完成请查看 "$DEPLOY_DIR" 目录"
