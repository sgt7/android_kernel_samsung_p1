#!/bin/bash

TREE="device/samsung/p1"

make clean mrproper
make ARCH=arm p1_cm9_defconfig

make -j8 modules
for M in `find . -name *.ko`
do
  cp "$M" ../../../"$TREE"/modules
done

make -j8
cp arch/arm/boot/zImage ../../../"$TREE"/kernel

build() {
  local target=$1
  make clean mrproper
  make ARCH=arm "$target"_cm9_defconfig
  make -j8
  cp arch/arm/boot/zImage ../../../"$TREE"/kernel-$target
}

targets=("$@")
if [ 0 = "${#targets[@]}" ] ; then
    targets=(p1c p1l p1n)
fi

for target in "${targets[@]}" ; do
  build $target
done
