#!/bin/sh

# created by juul

make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf-

make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- uImage

make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- targz-pkg

depmod -ae -F System.map -b tar-install -r 2.6.31-rt11-lab126 -n > modules.dep

sed -i 's/^kernel\//\/lib\/modules\/2.6.31-rt11-lab126\/kernel\//g' modules.dep

mkdir -p OUTPUT

cp arch/arm/boot/uImage OUTPUT/
cp linux-2.6.31-rt11-lab126.tar.gz OUTPUT/
cp modules.dep OUTPUT/

echo " "
echo "Build complete. Find the results in the OUTPUT dir."
echo "It should contain one uImage (the kernel),"
echo "one tar.gz containing the kernel modules"
echo "and one modules.dep file."
