#!/bin/bash

setup ()
{
    if [ x = "x$ANDROID_BUILD_TOP" ] ; then
        echo "Android build environment must be configured"
        exit 1
    fi
    . "$ANDROID_BUILD_TOP"/build/envsetup.sh

    KERNEL_DIR="$(dirname "$(readlink -f "$0")")"
    BUILD_DIR="$KERNEL_DIR/build"
    MODULES=("fs/cifs/cifs.ko" "fs/fuse/fuse.ko" "fs/nls/nls_utf8.ko")

    if [ x = "x$NO_CCACHE" ] && ccache -V &>/dev/null ; then
        CCACHE=ccache
        CCACHE_BASEDIR="$KERNEL_DIR"
        CCACHE_COMPRESS=1
        CCACHE_DIR="$BUILD_DIR/.ccache"
        export CCACHE_DIR CCACHE_COMPRESS CCACHE_BASEDIR
    else
        CCACHE=""
    fi

    CROSS_PREFIX="$ANDROID_BUILD_TOP/prebuilt/linux-x86/toolchain/arm-eabi-4.4.3/bin/arm-eabi-"
}

build ()
{
    local target=$1
    local module
    THREADS=`cat /proc/cpuinfo | grep processor | wc -l`
    make -j${THREADS} ARCH=arm ${target}_cm9_defconfig
    make -j${THREADS}
    cp "$KERNEL_DIR"/arch/arm/boot/zImage $ANDROID_BUILD_TOP/device/samsung/galaxytab/kernel
    for module in "${MODULES[@]}" ; do
        cp "$target_dir/$module" $ANDROID_BUILD_TOP/device/samsung/galaxytab/modules
    done
}
    
setup

if [ "$1" = clean ] ; then
    make clean
    exit 0
fi

target=$1

START=$(date +%s)
THREADS=`cat /proc/cpuinfo | grep processor | wc -l`
make -j${THREADS} ARCH=arm $target_cm9_defconfig
make -j${THREADS}
cp arch/arm/boot/zImage $ANDROID_BUILD_TOP/device/samsung/galaxytab/kernel

END=$(date +%s)
ELAPSED=$((END - START))
E_MIN=$((ELAPSED / 60))
E_SEC=$((ELAPSED - E_MIN * 60))
printf "Elapsed: "
[ $E_MIN != 0 ] && printf "%d min(s) " $E_MIN
printf "%d sec(s)\n" $E_SEC
