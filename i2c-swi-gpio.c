#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>

#define I2C_SWI_DEFAULT_ADDR    0x60

#define RK3328_GPIO_IOADDR      0xFF210000u
#define RK3328_GPIO_BANK_OFFSET 0x00010000u
#define RK3328_GPIO_IOMAP_SIZE  0x64
#define RK3328_GPIO_VAL_OFFSET  0x00
#define RK3328_GPIO_DIR_OFFSET  0x04
#define RK3328_GPIO_READ_OFFSET 0x50

#define IOBANK(gpio)            ((gpio) / 32u)
#define IOBIT(gpio)             ((gpio) % 32u)
#define IOBASE(gpio)            (IOBANK(gpio) * RK3328_GPIO_BANK_OFFSET + RK3328_GPIO_IOADDR)
#define IOMASK(gpio)            (1u << IOBIT(gpio))

#define GPIO_VAL_STATE(state)   ((state) = readl(base + RK3328_GPIO_VAL_OFFSET))
#define GPIO_DIR_STATE(state)   ((state) = readl(base + RK3328_GPIO_DIR_OFFSET))
#define GPIO_VAL_HIGH(state)    (writel((state) |  IOMASK(gpio_num), base + RK3328_GPIO_VAL_OFFSET))
#define GPIO_VAL_LOW(state)     (writel((state) & ~IOMASK(gpio_num), base + RK3328_GPIO_VAL_OFFSET))
#define GPIO_DIR_IN(state)      (writel((state) & ~IOMASK(gpio_num), base + RK3328_GPIO_DIR_OFFSET))
#define GPIO_DIR_OUT(state)     (writel((state) |  IOMASK(gpio_num), base + RK3328_GPIO_DIR_OFFSET))
#define GPIO_READ(val)          (val = (readl(base + RK3328_GPIO_READ_OFFSET) & IOMASK(gpio_num)))

static u8 i2c_addr = I2C_SWI_DEFAULT_ADDR;
module_param(i2c_addr, byte, S_IRUGO);
MODULE_PARM_DESC(i2c_addr, "I2C address");

static int gpio_num = -EINVAL;
module_param(gpio_num, int, S_IRUGO);
MODULE_PARM_DESC(gpio_num, "GPIO pin");

static void __iomem *base = NULL;

static u32 swi_func(struct i2c_adapter *adap) {
    return I2C_FUNC_I2C | I2C_FUNC_SMBUS_EMUL;
}

static void send_wake(void) {
    u32 iostate;

    GPIO_DIR_STATE(iostate);
    GPIO_DIR_OUT(iostate);
    udelay(60);
    GPIO_DIR_IN(iostate);
}

static void send_byte(u8 data) {
    u8 bit_mask = 1;
    u32 iostate;

    GPIO_DIR_STATE(iostate);

    /* Stop looping when the mask overflows */
    while (bit_mask) {
        /* Start bit */
        GPIO_DIR_OUT(iostate);
        udelay(4);

        /* Data bit */
        GPIO_DIR_IN(iostate);
        udelay(4);

        if (data & bit_mask) {
            udelay(4);
        }
        /* Couple more transitions if data bit is 0 */
        else {
            GPIO_DIR_OUT(iostate);
            udelay(4);
            GPIO_DIR_IN(iostate);
        }

        udelay(25);

        bit_mask <<= 1;
    }
}

static int recv_byte(u8 *data) {
    u8 bit_mask = 1;
    u8 result = 0;
    u32 iostate;
    int prev_val, val, transitions, usec;

    /* Ensure pin direction is "in" */
    GPIO_DIR_STATE(iostate);
    GPIO_DIR_IN(iostate);

    while (bit_mask) {
        /* Start */
        val = 1;
        for (usec = 80; usec >= 0; usec -= 2) {
            if (GPIO_READ(val) == 0)
                break;
            udelay(2);
        }

        /* Check if start condition was met */
        if (val)
            return -ETIMEDOUT;

        transitions = 0;
        prev_val = 0;
        for (usec = 30; usec >= 0; usec -= 2) {
            if (GPIO_READ(val) != prev_val) {
                prev_val = val;
                transitions++;
            }
            udelay(2);
        }

        switch (transitions) {
            case 1:
                result = result | bit_mask;
            case 3:
                break;
            default:
                return -ETIMEDOUT;
        }

        bit_mask <<= 1;
    }

    if (data != NULL)
        *data = result;

    return 0;
}

static int check_dev(void) {
    u32 status = 0;
    int i, err;

    send_wake();
    udelay(1500);
    send_byte(0x88);
    udelay(20);

    for (i = 0; i < sizeof(status); i++) {
        if ((err = recv_byte(((u8 *)&status) + i)) != 0) {
            return err;
        }
    }

    /* Status response packet after wake */
    if (status != 0x43331104)
        return -ENODEV;

    return 0;
}

static s32 swi_xfer(struct i2c_adapter *adap, struct i2c_msg *msgs, int num) {
    s32 err = -EOPNOTSUPP;
    int i, j, len;
    unsigned long flags;

    local_irq_save(flags);

    for (i = 0; i < num; i++) {
        /* Only handle our address or wake */
        if (msgs[i].addr != i2c_addr && msgs[i].addr != 0)
            return -ENODEV;

        /* Quick command, ignore */
        if (msgs[i].len == 0)
            continue;

        if (msgs[i].flags & I2C_M_RD) {
            /* Send transfer command */
            send_byte(0x88);
            udelay(20);

            /* First byte should hold the byte count */
            if ((err = recv_byte(&msgs[i].buf[0])) != 0)
                return err;

            if (msgs[i].buf[0] < 4 || msgs[i].buf[0] > 155)
                return -EIO;

            if (msgs[i].len >= msgs[i].buf[0]) {
                len = msgs[i].buf[0];
            } else {
                dev_dbg(&adap->dev, "Group count is larger than recv buffer.\n");
                len = msgs[i].len;
            }

            for (j = 1; j < len; j++) {
                if ((err = recv_byte(&msgs[i].buf[j])) != 0)
                    return err;
            }

            /* Read a minumum of 4 bytes */
            for (; j < 4; j++) {
                if ((err = recv_byte(NULL)) != 0)
                    return err;
            }

        } else {
            /* Check if this is a wake command*/
            if (msgs[i].addr == 0) {
                if (msgs[i].buf[0] == 0) {
                    send_wake();
                    continue;
                } else {
                    return -ENODEV;
                }
            }

            /* Sleep */
            if (msgs[i].buf[0] == 1) {
                send_byte(0xCC);
                continue;
            }

            /* Idle */
            if (msgs[i].buf[0] == 2) {
                send_byte(0xBB);
                continue;
            }

            /* Must now be a command or we don't support it */
            if (msgs[i].buf[0] != 3)
                return -EOPNOTSUPP;

            /* Some sanity checks on the message length */
            if (msgs[i].buf[1] < 4 || msgs[i].buf[1] > 155)
                return -EINVAL;

            /* Length should equal count + 1 */
            if (msgs[i].len != (msgs[i].buf[1] + 1))
                return -EINVAL;

            /* Send command sequence */
            send_byte(0x77);

            /* Send the group, skipping the word address */
            for (j = 1; j < msgs[i].len; j++) {
                send_byte(msgs[i].buf[j]);
            }
        }
    }

    local_irq_restore(flags);

    return 0;
}

static const struct i2c_algorithm swi_algorithm = {
    .functionality = swi_func,
    .master_xfer = swi_xfer,
};

static struct i2c_adapter swi_adapter = {
    .owner = THIS_MODULE,
    .class = I2C_CLASS_HWMON | I2C_CLASS_SPD,
    .algo = &swi_algorithm,
    .name = "I2C Adapter for SWI through GPIO Interface",
};

static int __init i2c_swi_dev_init(void) {
    int err = -ENODEV;
    u32 iostate;
    unsigned long flags;

    if (!gpio_is_valid(gpio_num)) {
        pr_err("Please specify a valid GPIO number.\n");
        err = -ENODEV;
        goto fail_gpio;
    }

    /* Reserve and setup GPIO even though we access IO registers directly */
    if ((err = gpio_request_one(gpio_num, GPIOF_OUT_INIT_HIGH | GPIOF_OPEN_DRAIN, "swi")) != 0) {
        pr_err("Unable to reserve GPIO resource.\n");
        goto fail_gpio;
    }

    if ((base = ioremap(IOBASE(gpio_num), RK3328_GPIO_IOMAP_SIZE)) == NULL) {
        pr_err("Unable to allocate IO Memory\n");
        err = -ENOMEM;
        goto fail_ioremap;
    }

    local_irq_save(flags);

    /* Configure as input to emulate open drain */
    GPIO_DIR_STATE(iostate);
    GPIO_DIR_IN(iostate);

    /* Set output bit to 0 so we only need to toggle pin direction */
    GPIO_VAL_STATE(iostate);
    GPIO_VAL_LOW(iostate);

    /* Check if device responds */
    err = check_dev();

    local_irq_restore(flags);

    if (err) {
        pr_err("Failed to communicate with device.\n");
        err = -ENODEV;
        goto fail_devcheck;
    }

    pr_info("Verified SWI device communication through GPIO%d\n", gpio_num);

    if ((err = i2c_add_adapter(&swi_adapter)) != 0) {
        pr_err("Failed to register I2C adapter.\n");
        goto fail_i2c;
    }

    pr_info("Emulating i2c adapter i2c-%d with device at address 0x%02X\n", swi_adapter.nr, i2c_addr);

    return 0;

fail_i2c:
fail_devcheck:
    iounmap(base);

fail_ioremap:
    gpio_free(gpio_num);

fail_gpio:
    return err;
}

static void __exit i2c_swi_dev_exit(void) {
    i2c_del_adapter(&swi_adapter);
    iounmap(base);
    gpio_free(gpio_num);
}

module_init(i2c_swi_dev_init);
module_exit(i2c_swi_dev_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("radical");
MODULE_DESCRIPTION("SWI device GPIO interface through I2C adapter emulation");
MODULE_VERSION("0.10");
