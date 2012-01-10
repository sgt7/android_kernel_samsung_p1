#!/bin/bash
#
# Script to build Galaxy Tab Kernel
# 2012 Chirayu Desai 

# Common defines
txtrst='\e[0m'  # Color off
txtred='\e[0;31m' # Red
txtgrn='\e[0;32m' # Green
txtylw='\e[0;33m' # Yellow
txtblu='\e[0;34m' # Blue

echo -e "${txtblu}##########################################"
echo -e "${txtblu}#                                        #"
echo -e "${txtblu}#      GALAXYTAB KERNEL BUILDSCRIPT      #"
echo -e "${txtblu}#                                        #"
echo -e "${txtblu}##########################################"
echo -e "\r\n ${txtrst}"

# Starting Timer
START=$(date +%s)
DEVICE="$1"
THREADS=`cat /proc/cpuinfo | grep processor | wc -l`

case "$DEVICE" in
	clean)
		make clean
		exit
		;;
	p1|P1)
		DEFCONFIG=p1_cm9_defconfig
		;;
	p1c|P1C)
		DEFCONFIG=p1c_cm9_defconfig
		;;
	p1l|P1L)
		DEFCONFIG=p1l_cm9_defconfig
		;;
	p1n|P1N)
		DEFCONFIG=p1n_cm9_defconfig
		;;
	*)
		echo -e "${txtred}Usage: $0 device"
		echo -e "Example: ./build.sh p1"
		echo -e "Supported Devices: p1 p1c p1l p1n ${txtrst}"
		;;
esac

# The real build starts now
if [ ! "$1" = "" ] ; then
make -j$THREADS ARCH=arm $DEFCONFIG
make -j$THREADS
fi

# Make it into a boot.img
# ty nubecoder for this :)
# defines
KERNEL_DIR=`pwd`
KERNEL_PATH="./arch/arm/boot/zImage"
KERNEL_INITRD_DIR="../initramfs"
KERNEL_INITRD_GIT="https://github.com/sgt7/p1000-initramfs-cm9.git"

if [ ! -d $KERNEL_INITRD_DIR ]; then
	cd ..
	git clone $KERNEL_INITRD_GIT initramfs
	cd $KERNEL_DIR
fi

# .git is huge!
mv $KERNEL_INITRD_DIR/.git DONOTLOOKATME

# Function
function PACKAGE_BOOTIMG()
{
	if [ "$1" = "" ] || [ "$2" = "" ] ; then
		ERROR_MSG="Error: PACKAGE_BOOTIMG - Missing args!"
		return 2
	fi
	if [ ! -f "$1" ] ; then
		ERROR_MSG="Error: PACKAGE_BOOTIMG - zImage does not exist!"
		return 1
	else
		echo -e "${txtblu} Creating ramdisk.img"
		./tools/mkbootfs $KERNEL_INITRD_DIR | ./tools/minigzip > ramdisk.img
		if [ -f boot.img ] ; then
			echo "removing old boot.img"
			rm -f boot.img
		fi
		echo -e "${txtblu} Creating boot.img"
		./tools/mkshbootimg.py boot.img ./arch/arm/boot/zImage ramdisk.img
		echo -e "${txtblu} Cleaning up temp files:"
		rm -f ramdisk.img
		echo -e "${txtblu} Done!"
		return 0
	fi
}

# Main
if [ ! "$1" = "" ] ; then
PACKAGE_BOOTIMG "$KERNEL_PATH" "$KERNEL_INITRD_DIR"
if [ $? != 0 ] ; then
	echo -e "${txtred} $ERROR_MSG"
else
	echo -e "${txtblu} Boot.img created successfully...${txtrst}"
fi
fi

# move it back just in case
mv DONOTLOOKATME $KERNEL_INITRD_DIR/.git

# The end!
END=$(date +%s)
ELAPSED=$((END - START))
E_MIN=$((ELAPSED / 60))
E_SEC=$((ELAPSED - E_MIN * 60))
printf "Elapsed: "
[ $E_MIN != 0 ] && printf "%d min(s) " $E_MIN
printf "%d sec(s)\n" $E_SEC
