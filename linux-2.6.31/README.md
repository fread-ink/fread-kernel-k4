This is a Linux kernel for the Amazon Kindle 4 non-touch.

It is based on the Amazon source code release for the Kindle 4, with some missing files added in from the Amazon source code release for the Kindle 5 and a few extra tweaks.

This will form part of the fread.ink operating system.

Instructions for how to cross compile will follow in hopefully not too long, but here is the brief version:

*Ensure you have an arm hard-float cross-compilation toolchain in your path like arm-linux-gnueabihf-*
*cp CONFIG_WORKING .config
*Edit the CONFIG_INITRAMFS_SOURCE line in .config to point to a valid initial ramdisk file in .cpio format
*Run ./build.sh


