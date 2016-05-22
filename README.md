Linux kernel for fread.ink for Kindle 4th generation non-touch.

WORK IN PROGRESS. Expect things to be broken.

It is based on the Amazon source code release for the Kindle 4, with some missing files added in from the Amazon source code release for the Kindle 5 and a few extra tweaks.

This will form part of the fread.ink operating system.

Instructions for how to cross compile will follow in hopefully not too long, but here is the brief version:

* Ensure you have an arm hard-float cross-compilation toolchain in your path like arm-linux-gnueabihf-*
* `cp ../config .config`
* Edit the CONFIG_INITRAMFS_SOURCE line in .config to point to a valid initial ramdisk file in .cpio format
* Run `./build.sh`

More complete build instructions can be found in the (fread-vagrant)[https://github.com/fread-ink/fread-vagrant] project.


# Disclaimer

Kindle is a registered trademarks of Amazon Inc. 

Amazon Inc is not in any way affiliated with fread nor this git project nor any of the authors of this project and neither fread nor this git project is in any way endorsed by Amazon Inc.
