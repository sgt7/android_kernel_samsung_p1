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
	p1)
		DEFCONFIG=p1_cm9_defconfig
		;;
	p1c)
		DEFCONFIG=p1c_cm9_defconfig
		;;
	p1l)
		DEFCONFIG=p1l_cm9_defconfig
		;;
	p1n)
		DEFCONFIG=p1n_cm9_defconfig
		;;
	*)
		echo -e "${txtred}Usage: $0 device"
		echo -e "Example: ./build.sh p1"
		echo -e "Supported Devices: p1 p1c p1l p1n ${txtrst}"
		;;
esac

# Some defines
KERNEL_DIR=`pwd`
KERNEL_INITRD_DIR="../initramfs"
KERNEL_INITRD_GIT="https://github.com/sgt7/p1000-initramfs-cm9.git"

# Check if initramfs is present, if not, then clone it
if [ ! -d $KERNEL_INITRD_DIR ]; then
	cd ..
	git clone $KERNEL_INITRD_GIT initramfs
	cd $KERNEL_DIR
fi

# .git is huge!
mv $KERNEL_INITRD_DIR/.git DONOTLOOKATME


# The real build starts now
if [ ! "$1" = "" ] ; then
make -j$THREADS ARCH=arm $DEFCONFIG
make -j$THREADS
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
