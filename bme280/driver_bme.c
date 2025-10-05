/*
 * driver_bme.c - BME280 kernel driver with full I2C and SPI support
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/rtmutex.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <linux/poll.h>
#include <linux/of.h>
#include <linux/pm.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/seqlock.h>
#include <linux/rcupdate.h>
#include <linux/kfifo.h>
#include <linux/bpf.h>
#include <linux/hrtimer.h>
#include <linux/dma-buf.h>
#include <linux/sched.h>
#include <linux/numa.h>
#include <linux/futex.h>
#include "lib_bme280.h"
#include "advanced_bme280.h"

#define DRIVER_NAME "bme280_kmod_full"
#define DEVICE_BASENAME "bme280"
#define MAX_DEVICES 16

#define BME280_IOC_MAGIC 'B'
#define BME280_IOCTL_GET_TEMPERATURE _IOR(BME280_IOC_MAGIC, 1, int32_t)
#define BME280_IOCTL_GET_PRESSURE    _IOR(BME280_IOC_MAGIC, 2, uint32_t)
#define BME280_IOCTL_GET_HUMIDITY    _IOR(BME280_IOC_MAGIC, 3, uint32_t)
#define BME280_IOCTL_RESET           _IO(BME280_IOC_MAGIC, 4)
#define BME280_IOCTL_GET_CHIPID      _IOR(BME280_IOC_MAGIC, 5, uint8_t)
#define BME280_IOCTL_FUTEX_TEST      _IO(BME280_IOC_MAGIC, 6)
#define BME280_IOCTL_RCU_UPDATE      _IOW(BME280_IOC_MAGIC, 7, uint32_t)
#define BME280_IOCTL_NUMA_ALLOC      _IO(BME280_IOC_MAGIC, 8)
#define BME280_IOCTL_QOS_SUBMIT      _IOW(BME280_IOC_MAGIC, 9, uint32_t)

/* Define BME280_REG_RESET if not in header */
#define BME280_REG_RESET 0xE0

struct bme_device {
    struct device *dev;
    struct cdev cdev;
    dev_t devt;
    struct class *class;
    struct rt_mutex lock;
    wait_queue_head_t read_wait;
    bool data_ready;
    atomic_t open_count;
    int id;
    struct i2c_client *i2c_client;
    struct spi_device *spi;
    struct bme280_calib *calib_rcu;
    int32_t t_fine;
    struct bme_shared_config shared_cfg;
    u8 osrs_t, osrs_p, osrs_h;
    u8 mode;
    u8 config_t_sb, config_filter;
    struct kfifo ring;
    struct bpf_map *ebpf_map;
    struct hrtimer timer;
    struct dma_buf *dma_buf;
    int futex_var;
    struct work_struct work;
};

static struct class *bme_class;
static struct bme_device *bme_devices[MAX_DEVICES];
static DEFINE_MUTEX(global_lock);

/* Prototype for file operations functions */
static int bme_open(struct inode *inode, struct file *file);
static int bme_release(struct inode *inode, struct file *file);
static ssize_t bme_read(struct file *file, char __user *buf, size_t count, loff_t *offset);
static unsigned int bme_poll(struct file *file, poll_table *wait);
static long bme_ioctl(struct file *file, unsigned int cmd, unsigned long arg);

/* Work and timer callbacks prototype */
static void bme_work_helper(struct work_struct *w);
static enum hrtimer_restart bme_hrtimer_cb(struct hrtimer *timer);

/* File operations structure */
static const struct file_operations bme_fops = {
    .owner = THIS_MODULE,
    .open = bme_open,
    .release = bme_release,
    .read = bme_read,
    .poll = bme_poll,
    .unlocked_ioctl = bme_ioctl,
};

static struct bme_device *bme_alloc_dev(int id)
{
    struct bme_device *bdev;
    int node = numa_node_id();

    bdev = kzalloc_node(sizeof(*bdev), GFP_KERNEL, node);
    if (!bdev) {
        pr_err("BME280: Failed to allocate device memory\n");
        return NULL;
    }

    bdev->calib_rcu = kzalloc_node(sizeof(*bdev->calib_rcu), GFP_KERNEL, node);
    if (!bdev->calib_rcu) {
        kfree(bdev);
        pr_err("BME280: Failed to allocate calibration memory\n");
        return NULL;
    }

    bdev->id = id;
    rt_mutex_init(&bdev->lock);
    init_waitqueue_head(&bdev->read_wait);
    atomic_set(&bdev->open_count, 0);
    seqlock_init(&bdev->shared_cfg.seqlock);
    bdev->data_ready = false;

    if (kfifo_alloc(&bdev->ring, 1024, GFP_KERNEL)) {
        kfree(bdev->calib_rcu);
        kfree(bdev);
        pr_err("BME280: Failed to allocate kfifo\n");
        return NULL;
    }

    hrtimer_init(&bdev->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    bdev->timer.function = bme_hrtimer_cb;

    INIT_WORK(&bdev->work, bme_work_helper);

    bdev->futex_var = 0;

    pr_info("BME280: Allocated device %d on NUMA node %d\n", id, node);
    return bdev;
}

static int bme_open(struct inode *inode, struct file *file)
{
    struct bme_device *bdev = container_of(inode->i_cdev, struct bme_device, cdev);
    file->private_data = bdev;
    atomic_inc(&bdev->open_count);
    pr_info("BME280: Device opened\n");
    return 0;
}

static int bme_release(struct inode *inode, struct file *file)
{
    struct bme_device *bdev = file->private_data;
    atomic_dec(&bdev->open_count);
    pr_info("BME280: Device closed\n");
    return 0;
}

static ssize_t bme_read(struct file *file, char __user *buf, size_t count, loff_t *offset)
{
    struct bme_device *bdev = file->private_data;
    char data[64];
    int len;
    unsigned long start_time = jiffies;

    rt_mutex_lock(&bdev->lock);
    if (wait_event_interruptible(bdev->read_wait, bdev->data_ready)) {
        rt_mutex_unlock(&bdev->lock);
        return -ERESTARTSYS;
    }
    bdev->data_ready = false;
    snprintf(data, sizeof(data), "Data ready for device %d\n", bdev->id);
    len = strlen(data);
    rt_mutex_unlock(&bdev->lock);

    if (len > count) len = count;
    if (copy_to_user(buf, data, len)) {
        bme_advanced_track_error(bdev);
        return -EFAULT;
    }

    bme_advanced_track_read(bdev, start_time);
    return len;
}

static unsigned int bme_poll(struct file *file, poll_table *wait)
{
    struct bme_device *bdev = file->private_data;
    unsigned int mask = 0;

    poll_wait(file, &bdev->read_wait, wait);
    if (bdev->data_ready) {
        mask |= POLLIN | POLLRDNORM;
    }
    return mask;
}

static long bme_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    struct bme_device *bdev = file->private_data;
    int ret = 0;
    int32_t temp;
    uint32_t press, hum;
    uint8_t chip_id;
    uint32_t deadline;
    unsigned long start_time = jiffies;

    rt_mutex_lock(&bdev->lock);

    switch (cmd) {
    case BME280_IOCTL_GET_TEMPERATURE:
        if (bdev->i2c_client) {
            ret = bme280_read_temperature_i2c(bdev->i2c_client, bdev->calib_rcu, &bdev->t_fine, &temp);
        } else {
            ret = bme280_read_temperature_spi(bdev->spi, bdev->calib_rcu, &bdev->t_fine, &temp);
        }
        if (ret == 0) {
            ret = copy_to_user((void __user *)arg, &temp, sizeof(temp)) ? -EFAULT : 0;
        }
        break;
    case BME280_IOCTL_GET_PRESSURE:
        if (bdev->i2c_client) {
            ret = bme280_read_pressure_i2c(bdev->i2c_client, bdev->calib_rcu, bdev->t_fine, &press);
        } else {
            ret = bme280_read_pressure_spi(bdev->spi, bdev->calib_rcu, bdev->t_fine, &press);
        }
        if (ret == 0) {
            ret = copy_to_user((void __user *)arg, &press, sizeof(press)) ? -EFAULT : 0;
        }
        break;
    case BME280_IOCTL_GET_HUMIDITY:
        if (bdev->i2c_client) {
            ret = bme280_read_humidity_i2c(bdev->i2c_client, bdev->calib_rcu, bdev->t_fine, &hum);
        } else {
            ret = bme280_read_humidity_spi(bdev->spi, bdev->calib_rcu, bdev->t_fine, &hum);
        }
        if (ret == 0) {
            ret = copy_to_user((void __user *)arg, &hum, sizeof(hum)) ? -EFAULT : 0;
        }
        break;
    case BME280_IOCTL_RESET:
        if (bdev->i2c_client) {
            ret = write_reg_i2c(bdev->i2c_client, BME280_REG_RESET, 0xB6);
        } else {
            ret = write_reg_spi(bdev->spi, BME280_REG_RESET, 0xB6);
        }
        break;
    case BME280_IOCTL_GET_CHIPID:
        if (bdev->i2c_client) {
            ret = bme280_read_chip_id_i2c(bdev->i2c_client, &chip_id);
        } else {
            ret = bme280_read_chip_id_spi(bdev->spi, &chip_id);
        }
        if (ret == 0) {
            ret = copy_to_user((void __user *)arg, &chip_id, sizeof(chip_id)) ? -EFAULT : 0;
        }
        break;
    case BME280_IOCTL_FUTEX_TEST:
        ret = bme_futex_helper(&bdev->futex_var, FUTEX_WAKE);
        break;
    case BME280_IOCTL_RCU_UPDATE:
        synchronize_rcu();
        break;
    case BME280_IOCTL_NUMA_ALLOC:
        bme_alloc_numa_buffer(1024);
        break;
    case BME280_IOCTL_QOS_SUBMIT:
        if (copy_from_user(&deadline, (void __user *)arg, sizeof(deadline))) {
            ret = -EFAULT;
        } else {
            bme_qos_helper(deadline);
        }
        break;
    default:
        ret = -ENOTTY;
    }

    rt_mutex_unlock(&bdev->lock);
    if (ret < 0) {
        bme_advanced_track_error(bdev);
    } else {
        bme_advanced_track_read(bdev, start_time);
    }
    return ret;
}

static void bme_work_helper(struct work_struct *w)
{
    struct bme_device *bdev = container_of(w, struct bme_device, work);
    pr_info("BME280: Work helper executed for device %d\n", bdev->id);
    bme_submit_work();
}

static enum hrtimer_restart bme_hrtimer_cb(struct hrtimer *timer)
{
    struct bme_device *bdev = container_of(timer, struct bme_device, timer);
    pr_info("BME280: HRTimer callback for device %d\n", bdev->id);
    bdev->data_ready = true;
    wake_up_interruptible(&bdev->read_wait);
    return HRTIMER_NORESTART;
}

static void bme_common_remove(struct bme_device *bdev)
{
    device_destroy(bme_class, bdev->devt);
    cdev_del(&bdev->cdev);
    bme_advanced_cleanup(bdev);
    kfifo_free(&bdev->ring);
    kfree(bdev->calib_rcu);
    kfree(bdev);
}

static int bme_i2c_probe(struct i2c_client *client)
{
    struct bme_device *bdev;
    int ret, idx;

    mutex_lock(&global_lock);
    for (idx = 0; idx < MAX_DEVICES; idx++) {
        if (!bme_devices[idx]) {
            break;
        }
    }
    if (idx == MAX_DEVICES) {
        mutex_unlock(&global_lock);
        pr_err("BME280: No available slot\n");
        return -ENOMEM;
    }

    bdev = bme_alloc_dev(idx);
    if (!bdev) {
        mutex_unlock(&global_lock);
        return -ENOMEM;
    }
    bdev->i2c_client = client;
    bdev->dev = &client->dev;
    dev_set_drvdata(bdev->dev, bdev);

    ret = alloc_chrdev_region(&bdev->devt, 0, 1, DEVICE_BASENAME);
    if (ret < 0) {
        kfree(bdev);
        mutex_unlock(&global_lock);
        return ret;
    }

    cdev_init(&bdev->cdev, &bme_fops);
    ret = cdev_add(&bdev->cdev, bdev->devt, 1);
    if (ret < 0) {
        unregister_chrdev_region(bdev->devt, 1);
        kfree(bdev);
        mutex_unlock(&global_lock);
        return ret;
    }

    device_create(bme_class, NULL, bdev->devt, NULL, DEVICE_BASENAME "%d", idx);

    ret = bme280_init_sensor_i2c(client, bdev->calib_rcu);
    if (ret) {
        cdev_del(&bdev->cdev);
        unregister_chrdev_region(bdev->devt, 1);
        device_destroy(bme_class, bdev->devt);
        kfree(bdev);
        mutex_unlock(&global_lock);
        return ret;
    }

    ret = bme_advanced_init(bdev);
    if (ret) {
        bme_common_remove(bdev);
        mutex_unlock(&global_lock);
        return ret;
    }

    bme_devices[idx] = bdev;
    mutex_unlock(&global_lock);
    pr_info("BME280: I2C device %d initialized\n", idx);
    return 0;
}

static void bme_i2c_remove(struct i2c_client *client)
{
    struct bme_device *bdev = dev_get_drvdata(&client->dev);
    int i;

    mutex_lock(&global_lock);
    for (i = 0; i < MAX_DEVICES; i++) {
        if (bme_devices[i] == bdev) {
            bme_common_remove(bdev);
            bme_devices[i] = NULL;
            break;
        }
    }
    mutex_unlock(&global_lock);
}

static int bme_spi_probe(struct spi_device *spi)
{
    struct bme_device *bdev;
    int ret, idx;

    mutex_lock(&global_lock);
    for (idx = 0; idx < MAX_DEVICES; idx++) {
        if (!bme_devices[idx]) {
            break;
        }
    }
    if (idx == MAX_DEVICES) {
        mutex_unlock(&global_lock);
        pr_err("BME280: No available slot\n");
        return -ENOMEM;
    }

    bdev = bme_alloc_dev(idx);
    if (!bdev) {
        mutex_unlock(&global_lock);
        return -ENOMEM;
    }
    bdev->spi = spi;
    bdev->dev = &spi->dev;
    dev_set_drvdata(bdev->dev, bdev);

    ret = alloc_chrdev_region(&bdev->devt, 0, 1, DEVICE_BASENAME);
    if (ret < 0) {
        kfree(bdev);
        mutex_unlock(&global_lock);
        return ret;
    }

    cdev_init(&bdev->cdev, &bme_fops);
    ret = cdev_add(&bdev->cdev, bdev->devt, 1);
    if (ret < 0) {
        unregister_chrdev_region(bdev->devt, 1);
        kfree(bdev);
        mutex_unlock(&global_lock);
        return ret;
    }

    device_create(bme_class, NULL, bdev->devt, NULL, DEVICE_BASENAME "%d", idx);

    ret = bme280_init_sensor_spi(spi, bdev->calib_rcu);
    if (ret) {
        cdev_del(&bdev->cdev);
        unregister_chrdev_region(bdev->devt, 1);
        device_destroy(bme_class, bdev->devt);
        kfree(bdev);
        mutex_unlock(&global_lock);
        return ret;
    }

    ret = bme_advanced_init(bdev);
    if (ret) {
        bme_common_remove(bdev);
        mutex_unlock(&global_lock);
        return ret;
    }

    bme_devices[idx] = bdev;
    mutex_unlock(&global_lock);
    pr_info("BME280: SPI device %d initialized\n", idx);
    return 0;
}

static void bme_spi_remove(struct spi_device *spi)
{
    struct bme_device *bdev = dev_get_drvdata(&spi->dev);
    int i;

    mutex_lock(&global_lock);
    for (i = 0; i < MAX_DEVICES; i++) {
        if (bme_devices[i] == bdev) {
            bme_common_remove(bdev);
            bme_devices[i] = NULL;
            break;
        }
    }
    mutex_unlock(&global_lock);
}

#ifdef CONFIG_PM
static int bme_pm_suspend(struct device *dev)
{
    struct bme_device *bdev = dev_get_drvdata(dev);
    if (!bdev) {
        return 0;
    }
    rt_mutex_lock(&bdev->lock);
    if (bdev->i2c_client) {
        bme280_set_mode_i2c(bdev->i2c_client, 0);
    } else {
        bme280_set_mode_spi(bdev->spi, 0);
    }
    rt_mutex_unlock(&bdev->lock);
    pr_info("BME280: Device suspended\n");
    return 0;
}

static int bme_pm_resume(struct device *dev)
{
    struct bme_device *bdev = dev_get_drvdata(dev);
    if (!bdev) {
        return 0;
    }
    rt_mutex_lock(&bdev->lock);
    if (bdev->i2c_client) {
        bme280_apply_settings_i2c(bdev->i2c_client, bdev->osrs_t, bdev->osrs_p, bdev->osrs_h, bdev->config_t_sb, bdev->config_filter, bdev->mode);
    } else {
        bme280_apply_settings_spi(bdev->spi, bdev->osrs_t, bdev->osrs_p, bdev->osrs_h, bdev->config_t_sb, bdev->config_filter, bdev->mode);
    }
    rt_mutex_unlock(&bdev->lock);
    pr_info("BME280: Device resumed\n");
    return 0;
}

static const struct dev_pm_ops bme_pm_ops = {
    .suspend = bme_pm_suspend,
    .resume = bme_pm_resume,
};
#endif

static const struct i2c_device_id bme_i2c_id[] = {
    { "bme280", 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, bme_i2c_id);

static const struct of_device_id bme_of_match[] = {
    { .compatible = "bosch,bme280" },
    { }
};
MODULE_DEVICE_TABLE(of, bme_of_match);

static struct i2c_driver bme_i2c_driver = {
    .driver = {
        .name = DRIVER_NAME,
        .of_match_table = bme_of_match,
#ifdef CONFIG_PM
        .pm = &bme_pm_ops,
#endif
    },
    .probe = bme_i2c_probe,
    .remove = bme_i2c_remove,
    .id_table = bme_i2c_id,
};

static const struct spi_device_id bme_spi_id[] = {
    { "bme280", 0 },
    { }
};
MODULE_DEVICE_TABLE(spi, bme_spi_id);

static struct spi_driver bme_spi_driver = {
    .driver = {
        .name = DRIVER_NAME "_spi",
        .of_match_table = bme_of_match,
#ifdef CONFIG_PM
        .pm = &bme_pm_ops,
#endif
    },
    .probe = bme_spi_probe,
    .remove = bme_spi_remove,
    .id_table = bme_spi_id,
};

static ssize_t config_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct bme_device *bdev = dev_get_drvdata(dev);
    u8 osrs_t;

    bme_read_shared_config(&bdev->shared_cfg, &osrs_t);
    return scnprintf(buf, PAGE_SIZE, "osrs_t=%u osrs_p=%u osrs_h=%u mode=%u t_sb=%u filter=%u\n",
                     bdev->osrs_t, bdev->osrs_p, bdev->osrs_h, bdev->mode, bdev->config_t_sb, bdev->config_filter);
}

static ssize_t config_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    struct bme_device *bdev = dev_get_drvdata(dev);
    u8 osrs_t, osrs_p, osrs_h, mode, t_sb, filter;
    int ret;

    ret = sscanf(buf, "osrs_t=%hhu osrs_p=%hhu osrs_h=%hhu mode=%hhu t_sb=%hhu filter=%hhu",
                 &osrs_t, &osrs_p, &osrs_h, &mode, &t_sb, &filter);
    if (ret != 6) {
        return -EINVAL;
    }

    rt_mutex_lock(&bdev->lock);
    bdev->osrs_t = osrs_t;
    bdev->osrs_p = osrs_p;
    bdev->osrs_h = osrs_h;
    bdev->mode = mode;
    bdev->config_t_sb = t_sb;
    bdev->config_filter = filter;

    ret = bme_update_shared_config(&bdev->shared_cfg, osrs_t);
    if (ret) {
        rt_mutex_unlock(&bdev->lock);
        return ret;
    }

    if (bdev->i2c_client) {
        ret = bme280_apply_settings_i2c(bdev->i2c_client, osrs_t, osrs_p, osrs_h, t_sb, filter, mode);
    } else {
        ret = bme280_apply_settings_spi(bdev->spi, osrs_t, osrs_p, osrs_h, t_sb, filter, mode);
    }
    rt_mutex_unlock(&bdev->lock);

    if (ret) {
        return ret;
    }
    return count;
}

static DEVICE_ATTR_RW(config);

static struct attribute *bme_attrs[] = {
    &dev_attr_config.attr,
    NULL
};

static const struct attribute_group bme_attr_group = {
    .attrs = bme_attrs,
};

static int __init bme_init(void)
{
    int ret;

    pr_info("BME280: Initializing module\n");

    bme_class = class_create("bme280_class");
    if (IS_ERR(bme_class)) {
        pr_err("BME280: Failed to create class: %ld\n", PTR_ERR(bme_class));
        return PTR_ERR(bme_class);
    }

    bme_class->dev_groups = (const struct attribute_group *[]) { &bme_attr_group, NULL };

    ret = i2c_add_driver(&bme_i2c_driver);
    if (ret) {
        pr_warn("BME280: Failed to register I2C driver: %d\n", ret);
    }

    ret = spi_register_driver(&bme_spi_driver);
    if (ret) {
        pr_warn("BME280: Failed to register SPI driver: %d\n", ret);
        i2c_del_driver(&bme_i2c_driver);
        class_destroy(bme_class);
        return ret;
    }

    pr_info("BME280: Full driver loaded\n");
    return 0;
}

static void __exit bme_exit(void)
{
    pr_info("BME280: Unloading module\n");

    i2c_del_driver(&bme_i2c_driver);
    spi_unregister_driver(&bme_spi_driver);

    class_destroy(bme_class);
    pr_info("BME280: Full driver unloaded\n");
}

module_init(bme_init);
module_exit(bme_exit);

MODULE_AUTHOR("Auto-generated full-featured BME280 driver with concurrency enhancements");
MODULE_DESCRIPTION("BME280 kernel driver with I2C, SPI, and advanced concurrency support");
MODULE_LICENSE("GPL v2");