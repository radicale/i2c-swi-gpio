obj-m += i2c-swi-gpio.o
all:
	make -C ../linux-rockchip M=$(PWD) modules
clean:
	make -C ../linux-rockchip M=$(PWD) clean
