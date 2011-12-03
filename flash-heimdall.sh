#cp Kernel/arch/arm/boot/zImage Source/zImage
#cd Source/
#./flash.sh

cp Kernel/arch/arm/boot/zImage .
heimdall flash --kernel zImage
