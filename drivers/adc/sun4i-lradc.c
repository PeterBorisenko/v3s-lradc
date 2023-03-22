// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Allwinner sunxi LRADC driver
 *
 * Copyright (C) 2021 Peter Borisenko <peter@awsmtek.com>
 */

/*
 * Allwinnner sunxi SoCs have a lradc which is specifically designed to have
 * various (tablet) keys (ie home, back, search, etc). attached to it using
 * a resistor network.
 *
 * This driver is utilizing LRADC as a general purpose ADC
 */

#warning "LRADC driver has not been properly tested!"

#include <linux/err.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/printk.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio/driver.h>

#define LRADC_CTRL      0x00
#define LRADC_INTC      0x04
#define LRADC_INTS      0x08
#define LRADC_DATA0     0x0c

/* LRADC_CTRL bits */
#define LRADC_CTRL_FIRST_CONVERT_DLY(x) ((x & 0xFF) << 24) /* 8 bits */
#define LRADC_CTRL_CONTINUE_TIME_SEL(x) ((x & 0x0F) << 16) /* 4 bits */
#define LRADC_CTRL_KEY_MODE_SEL(x)      ((x & 0x03) << 12) /* 2 bits */
#define LRADC_CTRL_LEVELA_B_CNT(x)      ((x & 0x0F) << 8)  /* 4 bits */
#define LRADC_CTRL_HOLD_KEY_EN(x)       ((x & 0x01) << 7)
#define LRADC_CTRL_HOLD_EN(x)           ((x & 0x01) << 6)
#define LRADC_CTRL_LEVELB_VOL(x)        ((x & 0x03) << 4)  /* 2 bits */
#define LRADC_CTRL_SAMPLE_RATE(x)       ((x & 0x03) << 2)  /* 2 bits */
#define LRADC_CTRL_ENABLE(x)            ((x & 0x01) << 0)

/* LRADC_INTC and LRADC_INTS bits */
#define LRADC_INT_CHAN0_KEYUP_IRQ       BIT(4)
#define LRADC_INT_CHAN0_ALRDY_HOLD_IRQ  BIT(3)
#define LRADC_INT_CHAN0_HOLD_IRQ        BIT(2)
#define LRADC_INT_CHAN0_KEYDOWN_IRQ     BIT(1)
#define LRADC_INT_CHAN0_DATA_IRQ        BIT(0)

/*
 * Set sample time to 4 ms / 250 Hz. No keypress detection enabled
 */
#define LRADC_ENABLE_SET    LRADC_CTRL_FIRST_CONVERT_DLY(0xf) \
                            | LRADC_CTRL_CONTINUE_TIME_SEL(0) \
                            | LRADC_CTRL_KEY_MODE_SEL(0) \
                            | LRADC_CTRL_LEVELA_B_CNT(0) \
                            | LRADC_CTRL_HOLD_KEY_EN(0) \
                            | LRADC_CTRL_HOLD_EN(0) \
                            | LRADC_CTRL_LEVELB_VOL(0) \
                            | LRADC_CTRL_SAMPLE_RATE(0) \
                            | LRADC_CTRL_ENABLE(1)

#define LRADC_DISABLE_SET   LRADC_CTRL_FIRST_CONVERT_DLY(0xf) \
                            | LRADC_CTRL_CONTINUE_TIME_SEL(0) \
                            | LRADC_CTRL_KEY_MODE_SEL(0) \
                            | LRADC_CTRL_LEVELA_B_CNT(0) \
                            | LRADC_CTRL_HOLD_KEY_EN(0) \
                            | LRADC_CTRL_HOLD_EN(0) \
                            | LRADC_CTRL_LEVELB_VOL(0) \
                            | LRADC_CTRL_SAMPLE_RATE(0) \
                            | LRADC_CTRL_ENABLE(0)

/* TODO Get it from DTS */
#define ADC_MUX_CHANNELS_NUM            8

struct sunxi_lradc_data {
    // struct device *dev;
    void __iomem *base;
    struct gpio_desc * mux0;
    struct gpio_desc * mux1;
    struct gpio_desc * mux2;
    struct regulator *vref_supply;
    const struct lradc_variant *variant;
    u8 currentChannel;
    u8 channelsEnabled;
    u32 vref;
};

/* struct lradc_variant - Describe sunxi-lradc variant
 * @divisor_numerator:      The numerator of lradc Vref internally divisor
 * @divisor_denominator:    The denominator of lradc Vref internally divisor
 */
struct lradc_variant {
    u8 divisor_numerator;
    u8 divisor_denominator;
};

static const struct lradc_variant lradc_variant_v3s = {
    .divisor_numerator = 3,
    .divisor_denominator = 4
};

static struct sunxi_lradc_data lradc;

// Kobject representation
// TODO:
// kobj:channel->(attr:select/attr:deselect)
// `echo 1 > channel/select` add channel 1 to adc mux sequencer and adds kobj:channel1
// kobj:channel1->(attr:value)
// `echo 1 > channel/deselect` removes channel 1 from adc mux sequencer and removes kobj:channel1

// Objects
struct channel_obj {
    struct kobject kobj;
    int value; // RO
    struct channel_obj * next;
};
#define to_channel_obj(x) container_of(x, struct channel_obj, kobj)

struct mux_obj {
    struct kobject kobj;
    int select; // WO
    int deselect; // WO
};
#define to_mux_obj(x) container_of(x, struct mux_obj, kobj)

static struct mux_obj *mux_obj= NULL;
static struct channel_obj *chan_obj_list[ADC_MUX_CHANNELS_NUM]= {0};

// Attributes
struct channel_attribute {
    struct attribute attr;
    ssize_t (*show)(struct channel_obj *adc, struct channel_attribute *attr, char *buf);
    ssize_t (*store)(struct channel_obj *adc, struct channel_attribute *attr, const char *buf, size_t count);
};
#define to_channel_attr(x) container_of(x, struct channel_attribute, attr)

struct mux_attribute {
    struct attribute attr;
    ssize_t (*show)(struct mux_obj *adc, struct mux_attribute *attr, char *buf);
    ssize_t (*store)(struct mux_obj *adc, struct mux_attribute *attr, const char *buf, size_t count);
};
#define to_mux_attr(x) container_of(x, struct mux_attribute, attr)

// Static declarations
static struct channel_obj *create_channel_obj(u8 num);
static void destroy_channel_obj(struct channel_obj *obj);

// Default handlers
static ssize_t channel_attr_show(struct kobject *kobj,
                 struct attribute *attr,
                 char *buf) {
    struct channel_attribute *attribute;
    struct channel_obj *adc;

    attribute = to_channel_attr(attr);
    adc = to_channel_obj(kobj);

    if (!attribute->show) {
        return -EIO;
    }

    return attribute->show(adc, attribute, buf);
}

static ssize_t channel_attr_store(struct kobject *kobj,
                  struct attribute *attr,
                  const char *buf, size_t len) {
    return -EIO;
}

static const struct sysfs_ops channel_sysfs_ops = {
    .show = channel_attr_show,
    .store = channel_attr_store,
};

static ssize_t mux_attr_show(struct kobject *kobj,
                 struct attribute *attr,
                 char *buf) {
    return -EIO;
}

static ssize_t mux_attr_store(struct kobject *kobj,
                  struct attribute *attr,
                  const char *buf, size_t len) {
    struct mux_attribute *attribute;
    struct mux_obj *obj;

    attribute = to_mux_attr(attr);
    obj = to_mux_obj(kobj);

    if (!attribute->store) {
        return -EIO;
    }

    return attribute->store(obj, attribute, buf, len);
}

static const struct sysfs_ops mux_sysfs_ops = {
    .show = mux_attr_show,
    .store = mux_attr_store,
};

// Custom handlers
static ssize_t channel_value_show(struct channel_obj *obj, struct channel_attribute *attr,
            char *buf) {
    ssize_t ret;
    ret= sprintf(buf, "%d\n", obj->value);
    return ret;
}

static ssize_t mux_select_store(struct mux_obj *obj, struct mux_attribute *attr,
               const char *buf, size_t count) {
    int var, ret;
    int error;

    ret = kstrtoint(buf, 10, &var);
    if (ret < 0) {
        return ret;
    }

    if (var < 0 || var >= ADC_MUX_CHANNELS_NUM) {
        return -EIO;
    }

    // Check if not selected yet
    if (chan_obj_list[var] != NULL) {
        printk(KERN_INFO "Channel %d is busy or didn't properly deselected\n", var);
        return -EIO;
    }

    if (lradc.channelsEnabled == 0) {
        error = regulator_enable(lradc.vref_supply);
        if (error) {
            return error;
        }
        writel(0xf, lradc.base + LRADC_INTS);
        writel(LRADC_INT_CHAN0_DATA_IRQ, lradc.base + LRADC_INTC);
        writel(LRADC_ENABLE_SET, lradc.base + LRADC_CTRL);
    }
    lradc.channelsEnabled+= 1;

    printk(KERN_INFO "Channels selected: %d\n", lradc.channelsEnabled);

    // Create Kobject
    chan_obj_list[var]= create_channel_obj(var);

    return count;
}

static ssize_t mux_deselect_store(struct mux_obj *obj, struct mux_attribute *attr,
               const char *buf, size_t count) {
    int var, ret;

    ret = kstrtoint(buf, 10, &var);
    if (ret < 0) {
        return ret;
    }

    if (var < 0 || var >= ADC_MUX_CHANNELS_NUM) {
        return -EIO;
    }

    // Check if selected
    if (chan_obj_list[var] == NULL) {
        printk(KERN_INFO "Channel %d is not selected\n", var);
        return -EIO;
    }

    // Destroy Kobject
    destroy_channel_obj(chan_obj_list[var]);
    chan_obj_list[var]= NULL;

    lradc.channelsEnabled-= 1;
    if (lradc.channelsEnabled == 0) {
        writel(0, lradc.base + LRADC_INTC);
        writel(LRADC_DISABLE_SET, lradc.base + LRADC_CTRL);
        regulator_disable(lradc.vref_supply);
    }

    printk(KERN_INFO "Channels selected: %d\n", lradc.channelsEnabled);

    return count;
}

static void channel_release(struct kobject *kobj) {
    struct channel_obj *adc;
    adc = to_channel_obj(kobj);
    kfree(adc);
}

static void mux_release(struct kobject *kobj) {
    struct mux_obj *adc;
    adc = to_mux_obj(kobj);
    kfree(adc);
}

static struct mux_attribute mux_attribute_deselect = __ATTR(deselect, 0664, NULL, mux_deselect_store);
static struct mux_attribute mux_attribute_select = __ATTR(select, 0664, NULL, mux_select_store);
static struct channel_attribute channel_attribute_value = __ATTR(value, 0664, channel_value_show, NULL);

/*
 * Create a group of attributes so that we can create and destroy them all
 * at once.
 */
static struct attribute *mux_default_attrs[] = {
    &mux_attribute_deselect.attr,
    &mux_attribute_select.attr,
    NULL,
};
ATTRIBUTE_GROUPS(mux_default);

static struct kobj_type lradc_ktype = {
    .sysfs_ops = &mux_sysfs_ops,
    .release = mux_release,
    .default_groups = mux_default_groups,
};

static struct attribute *channel_default_attrs[] = {
    &channel_attribute_value.attr,
    NULL,
};
ATTRIBUTE_GROUPS(channel_default);

static struct kobj_type channel_ktype = {
    .sysfs_ops = &channel_sysfs_ops,
    .release = channel_release,
    .default_groups = channel_default_groups,
};

static struct kset *lradc_kset;

static struct mux_obj *create_mux_obj(void) {
    struct mux_obj *obj;
    int retval;

    /* allocate the memory for the whole object */
    obj = kzalloc(sizeof(struct mux_obj), GFP_KERNEL);
    if (!obj) {
        return NULL;
    }

    obj->kobj.kset = lradc_kset;

    retval = kobject_init_and_add(&obj->kobj, &lradc_ktype, NULL, "%s", "mux");
    if (retval) {
        kobject_put(&obj->kobj);
        return NULL;
    }

    /*
     * We are always responsible for sending the uevent that the kobject
     * was added to the system.
     */
    kobject_uevent(&obj->kobj, KOBJ_ADD);

    return obj;
}

static struct channel_obj *create_channel_obj(u8 num) {
    struct channel_obj *obj;
    int retval;

    /* allocate the memory for the whole object */
    obj = kzalloc(sizeof(struct channel_obj), GFP_KERNEL);
    if (!obj) {
        return NULL;
    }

    obj->kobj.kset = lradc_kset;

    retval = kobject_init_and_add(&obj->kobj, &channel_ktype, NULL, "channel%d", num);
    if (retval) {
        kobject_put(&obj->kobj);
        return NULL;
    }

    /*
     * We are always responsible for sending the uevent that the kobject
     * was added to the system.
     */
    kobject_uevent(&obj->kobj, KOBJ_ADD);

    return obj;
}

static int __init lradc_sysfs_init(void) {
    /*
     * Create a kset with the name of "kset_example",
     * located under /sys/class/
     */
    lradc_kset = kset_create_and_add("lradc", NULL, kernel_kobj);
    if (!lradc_kset) {
        return -ENOMEM;
    }

    /*
     * Create three objects and register them with our kset
     */
    mux_obj = create_mux_obj();
    if (!mux_obj) {
        goto obj_error;
    }

    return 0;

obj_error:
    kset_unregister(lradc_kset);
    return -EINVAL;
}
static void destroy_channel_obj(struct channel_obj *obj) {
    kobject_put(&obj->kobj);
}

static void destroy_mux_obj(struct mux_obj *obj) {
    kobject_put(&obj->kobj);
}

static void __exit lradc_sysfs_exit(void) {
    u8 ch;
    for (ch= 0; ch < ADC_MUX_CHANNELS_NUM; ++ch) {
        if (chan_obj_list[ch]) {
            destroy_channel_obj(chan_obj_list[ch]);
            chan_obj_list[ch]= NULL;
        }
    }
    destroy_mux_obj(mux_obj);
    kset_unregister(lradc_kset);
}

/* DRIVER IMPLEMENTATION */

static long muxInit(struct device *dev) {
    lradc.mux0 = devm_gpiod_get_optional(dev, "lradc-mux-a0",
                            GPIOD_OUT_LOW);
    if (IS_ERR(lradc.mux0)) {
        return PTR_ERR(lradc.mux0);
    }
    lradc.mux1 = devm_gpiod_get_optional(dev, "lradc-mux-a1",
                            GPIOD_OUT_LOW);
    if (IS_ERR(lradc.mux1)) {
        return PTR_ERR(lradc.mux1);
    }
    lradc.mux2 = devm_gpiod_get_optional(dev, "lradc-mux-a2",
                            GPIOD_OUT_LOW);
    if (IS_ERR(lradc.mux2)) {
        return PTR_ERR(lradc.mux2);
    }

    return 0;
}

static void muxSelectNext(void) {

    u8 ch= ADC_MUX_CHANNELS_NUM;
    while (ch--) {
        lradc.currentChannel= (lradc.currentChannel + 1) % ADC_MUX_CHANNELS_NUM;
        if (chan_obj_list[lradc.currentChannel]) {
            gpiod_set_value(lradc.mux0, (lradc.currentChannel&0x01)?(1):(0));
            gpiod_set_value(lradc.mux1, (lradc.currentChannel&0x02)?(1):(0));
            gpiod_set_value(lradc.mux2, (lradc.currentChannel&0x04)?(1):(0));
            break;
        }
    }
}

static irqreturn_t sunxi_lradc_irq(int irq, void *dev_id) {
    u32 ints, val;
    int voltage;
    int currentChannel= lradc.currentChannel;

    ints  = readl(lradc.base + LRADC_INTS);
    writel(0x0f, lradc.base + LRADC_INTS);
    if ((ints & LRADC_INT_CHAN0_DATA_IRQ)) {
        writel(LRADC_DISABLE_SET, lradc.base + LRADC_CTRL);
        // Update MUX state
        muxSelectNext();
        // To prevent wrong measurement we have to disable a conversion to switch the MUX
        val = readl(lradc.base + LRADC_DATA0) & 0x3f;
            voltage = val;// * lradc.vref / 63;

            // Report value to sysfs
            if (chan_obj_list[currentChannel]) {
                chan_obj_list[currentChannel]->value= voltage;
            }
        writel(LRADC_ENABLE_SET, lradc.base + LRADC_CTRL);
    }

    return IRQ_HANDLED;
}

static int sunxi_lradc_probe(struct platform_device *pdev) {
    struct device *dev = &pdev->dev;
    int error;

    lradc.variant = of_device_get_match_data(dev);
    if (!lradc.variant) {
        dev_err(dev, "Missing sunxi-v3s-lradc variant\n");
        return -EINVAL;
    }

    lradc.vref_supply = devm_regulator_get(dev, "vref");
    if (IS_ERR(lradc.vref_supply)) {
        dev_err(dev, "devm_regulator_get error\n");
        return PTR_ERR(lradc.vref_supply);
    }

    lradc.base = devm_ioremap_resource(dev,
                  platform_get_resource(pdev, IORESOURCE_MEM, 0));
    if (IS_ERR(lradc.base)) {
        dev_err(dev, "devm_ioremap_resource error\n");
        return PTR_ERR(lradc.base);
    }

    if (0 != muxInit(dev)) {
        dev_err(dev, "muxInit error\n");
        return -EINVAL;
    }

    error = devm_request_irq(dev, platform_get_irq(pdev, 0),
                 sunxi_lradc_irq, 0,
                 "sunxi-lradc", &lradc);
    if (error) {
        dev_err(dev, "devm_request_irq error\n");
        return error;
    }

    error = lradc_sysfs_init();
    if (error) {
        dev_err(dev, "lradc_sysfs_init error\n");
        return error;
    }

    lradc.vref = regulator_get_voltage(lradc.vref_supply) *
              lradc.variant->divisor_numerator /
              lradc.variant->divisor_denominator;
    printk(KERN_INFO "LRADC driver vref: %d\n", lradc.vref);

    dev_info(dev, "LRADC driver is loaded\n");
    return error;
}

static void __exit lradc_exit(void) {
    writel(LRADC_CTRL_ENABLE(0), lradc.base + LRADC_CTRL);
    writel(0, lradc.base + LRADC_INTC);
    lradc_sysfs_exit();
    regulator_disable(lradc.vref_supply);
    printk(KERN_INFO "LRADC driver is exited\n");
}

static const struct of_device_id sunxi_lradc_of_match[] = {
    { .compatible= "allwinner,sunxi-v3s-lradc",
        .data= &lradc_variant_v3s },
    { /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, sunxi_lradc_of_match);

static struct platform_driver sunxi_lradc_driver = {
    .driver= {
        .name= "sunxi-lradc",
        .of_match_table= of_match_ptr(sunxi_lradc_of_match),
    },
    .probe= sunxi_lradc_probe,
};

builtin_platform_driver(sunxi_lradc_driver);

MODULE_DESCRIPTION("Allwinner sunxi LRADC driver");
MODULE_AUTHOR("Peter Borisenko <peter@awsmtek.com>");
MODULE_LICENSE("GPL");
