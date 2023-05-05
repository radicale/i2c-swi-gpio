# i2c-swi-gpio

This kernel module allows you to interface to a SWI device such as the Microchip ATECC608B through the GPIO pins of a RK3328 based system.  It emulates an I2C adaptor so that existing software works out-of-the-box.

## Usage

You must specify the GPIO number through the _gpio_num_ module parameter.  For example:

```insmod i2c-swi-gpio.ko gpio_num=82```

If successful in communicating with the SWI device, a new i2c adapter should be added to the system.  The default emulated i2c address for the device is _0x60_, but this can be changed through the _i2c_addr_ module parameter.

## Building

The provided _Makefile_ looks for a _linux-rockchip_ within the parent folder of this repository.  Either create a simlink to your kernel sources or modify to _Makefile_ to your pleasing.  You can then simply build as you would a linux kernel with a valid configuration.  For example,

```make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu-```

## To Do

* Make the module more generic, perhaps through the kernel pinctrl interface.
* Make use of the kernel devicetree
