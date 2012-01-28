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
    echo "Building for $target"
    local target_dir="$BUILD_DIR/$target"
    local module
    rm -fr "$target_dir"
    mkdir -p "$target_dir/usr"
    cp "$KERNEL_DIR/usr/"*.list "$target_dir/usr"
    sed "s|usr/|$KERNEL_DIR/usr/|g" -i "$target_dir/usr/"*.list
    mka -C "$KERNEL_DIR" O="$target_dir" ${target}_cm9_defconfig HOSTCC="$CCACHE gcc"
    mka -C "$KERNEL_DIR" O="$target_dir" HOSTCC="$CCACHE gcc" CROSS_COMPILE="$CCACHE $CROSS_PREFIX" zImage modules
    # cp "$target_dir"/arch/arm/boot/zImage $ANDROID_BUILD_TOP/device/samsung/galaxytab/kernel-$target
    for module in "${MODULES[@]}" ; do
        cp "$target_dir/$module" $ANDROID_BUILD_TOP/device/samsung/galaxytab/modules
    done
}
    
setup

if [ "$1" = clean ] ; then
    rm -fr "$BUILD_DIR"/*
    exit 0
fi

P1_target=$1
targets=("$@")
if [ 0 = "${#targets[@]}" ] ; then
    targets=($P1_target)
fi

START=$(date +%s)

for target in "${targets[@]}" ; do 
    build $target
done

# Boot.img 
KERNEL_DIR="$(dirname "$(readlink -f "$0")")"
KERNEL_PATHS="$target_dir/arch/arm/boot/zImage"
RECOVERY_INITRD=$ANDROID_BUILD_TOP/out/target/product/galaxytab/recovery/root
KERNEL_INITRD=$ANDROID_BUILD_TOP/out/target/product/galaxytab/root

PACKAGE_BOOTIMG()
{
	if [ "$1" = "" ] || [ "$2" = "" ]  || [ "$3" = "" ] ; then
		ERROR_MSG="Error: PACKAGE_BOOTIMG - Missing args!"
		return 1
	fi
	if [ ! -f "$1" ] ; then
		ERROR_MSG="Error: PACKAGE_BOOTIMG - zImage does not exist!"
		return 2
	else
		local PATH=$PATH:$KERNEL_DIR/tools
		local KERNEL_INITRD="$2"
		local RECOVERY_INITRD="$3"
		echo "Creating ramdisk.img"
		mkbootfs $KERNEL_INITRD | minigzip > ramdisk-kernel.img
		echo "Creating ramdisk-recovery.img"
		mkbootfs $RECOVERY_INITRD > ramdisk-recovery.cpio
		minigzip < ramdisk-recovery.cpio > ramdisk-recovery.img
		if [ -f boot.img ] ; then
			echo "Deleting old boot.img"
			rm -f boot.img
		fi
		echo "Creating boot.img"
		./mkshbootimg.py boot.img $KERNEL_PATH ramdisk-kernel.img ramdisk-recovery.img
		echo "Cleaning up temp files:"
		echo "* rm -f ramdisk-kernel.img"
		rm -f ramdisk-kernel.img
		echo "* rm -f ramdisk-recovery.cpio"
		rm -f ramdisk-recovery.cpio
		echo "* rm -f ramdisk-recovery.img"
		rm -f ramdisk-recovery.img
	fi
	return 0
}

PACKAGE_BOOTIMG "$KERNEL_PATH" "$KERNEL_INITRD" "$RECOVERY_INITRD"
if [ $? != 0 ] ; then
	echo "$ERROR_MSG"
else
	echo "boot.img created successfully"
fi
exit

cp $target_dir/boot.img $ANDROID_BUILD_TOP//out/target/product/galaxytab/

END=$(date +%s)
ELAPSED=$((END - START))
E_MIN=$((ELAPSED / 60))
E_SEC=$((ELAPSED - E_MIN * 60))
printf "Elapsed: "
[ $E_MIN != 0 ] && printf "%d min(s) " $E_MIN
printf "%d sec(s)\n" $E_SEC
