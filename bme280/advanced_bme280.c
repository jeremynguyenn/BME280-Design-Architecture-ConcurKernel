/*
 * advanced_bme280.c - Advanced features for BME280 kernel driver
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/dmaengine.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/jiffies.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/rtmutex.h>
#include <linux/wait.h>
#include <linux/atomic.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/seqlock.h>
#include <linux/kfifo.h>
#include <linux/bpf.h>
#include <linux/hrtimer.h>
#include <linux/dma-buf.h>
#include <linux/workqueue.h>
#include "lib_bme280.h"
#include "advanced_bme280.h"  /* Include header for prototypes */

/* Định nghĩa struct bme_device để fix lỗi undefined type (copy từ driver_bme.c) */
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

#define BME280_IRQ_NAME "bme280_irq"
#define BME280_DEBUGFS_DIR "bme280"
#define MAX_DEVICES 16

struct bme_advanced {
    struct bme_device *bdev;
    struct dentry *debugfs_root;
    unsigned long read_count;
    unsigned long error_count;
    unsigned long total_latency; /* in jiffies */
    struct dma_chan *dma_chan;
    int irq;
};

static struct bme_advanced *advanced_data[MAX_DEVICES];

static int bme_stats_show(struct seq_file *s, void *unused)
{
    struct bme_advanced *adv = s->private;
    unsigned long avg_latency = adv->read_count ? adv->total_latency / adv->read_count : 0;

    seq_printf(s, "Read Count: %lu\n", adv->read_count);
    seq_printf(s, "Error Count: %lu\n", adv->error_count);
    seq_printf(s, "Average Latency (jiffies): %lu\n", avg_latency);
    return 0;
}

static int bme_stats_open(struct inode *inode, struct file *file)
{
    return single_open(file, bme_stats_show, inode->i_private);
}

static const struct file_operations bme_stats_fops = {
    .open = bme_stats_open,
    .read = seq_read,
    .llseek = seq_lseek,
    .release = single_release,
};

static irqreturn_t bme_irq_handler(int irq, void *dev_id)
{
    struct bme_advanced *adv = dev_id;
    struct bme_device *bdev = adv->bdev;

    pr_info("BME280: IRQ %d triggered for device %d\n", irq, bdev->id);
    bdev->data_ready = true;
    wake_up_interruptible(&bdev->read_wait);

    return IRQ_HANDLED;
}

static void bme_dma_callback(void *param)
{
    struct bme_advanced *adv = param;
    pr_info("BME280: DMA transfer completed for device %d\n", adv->bdev->id);
}

static int bme_init_dma(struct bme_advanced *adv)
{
    struct dma_chan *chan;
    struct dma_async_tx_descriptor *desc;
    dma_cookie_t cookie;

    chan = dma_request_chan(adv->bdev->dev, "bme280");
    if (IS_ERR(chan)) {
        dev_err(adv->bdev->dev, "BME280: Failed to request DMA channel: %ld\n", PTR_ERR(chan));  /* ADDED FOR ESSENTIAL FIX: dev_err */
        return PTR_ERR(chan);
    }
    adv->dma_chan = chan;

    desc = dmaengine_prep_slave_single(chan, 0, 0, DMA_DEV_TO_MEM, DMA_PREP_INTERRUPT);
    if (!desc) {
        dev_err(adv->bdev->dev, "BME280: Failed to prepare DMA descriptor\n");
        dma_release_channel(chan);
        return -ENOMEM;
    }

    desc->callback = bme_dma_callback;
    desc->callback_param = adv;
    cookie = dmaengine_submit(desc);
    if (dma_submit_error(cookie)) {
        dev_err(adv->bdev->dev, "BME280: DMA submission failed\n");
        dma_release_channel(chan);
        return -EIO;
    }

    dma_async_issue_pending(chan);
    pr_info("BME280: DMA initialized for device %d\n", adv->bdev->id);
    return 0;
}

static int bme_init_debugfs(struct bme_advanced *adv)
{
    struct dentry *dir;

    dir = debugfs_create_dir(BME280_DEBUGFS_DIR, NULL);
    if (IS_ERR(dir)) {
        dev_err(adv->bdev->dev, "BME280: Failed to create debugfs directory: %ld\n", PTR_ERR(dir));
        return PTR_ERR(dir);
    }
    adv->debugfs_root = dir;

    if (!debugfs_create_file("stats", 0444, dir, adv, &bme_stats_fops)) {
        dev_err(adv->bdev->dev, "BME280: Failed to create debugfs stats file\n");
        debugfs_remove_recursive(dir);
        return -ENOMEM;
    }

    pr_info("BME280: Debugfs initialized for device %d\n", adv->bdev->id);
    return 0;
}

int bme_advanced_init(struct bme_device *bdev)
{
    struct bme_advanced *adv;
    int ret, idx;

    for (idx = 0; idx < MAX_DEVICES; idx++) {
        if (!advanced_data[idx]) {
            break;
        }
    }
    if (idx == MAX_DEVICES) {
        dev_err(bdev->dev, "BME280: No available advanced slots\n");
        return -ENOMEM;
    }

    adv = kzalloc(sizeof(*adv), GFP_KERNEL);
    if (!adv) {
        dev_err(bdev->dev, "BME280: Failed to allocate advanced structure\n");
        return -ENOMEM;
    }
    adv->bdev = bdev;

    ret = bme_init_dma(adv);
    if (ret) {
        dev_err(bdev->dev, "BME280: DMA initialization failed: %d\n", ret);
        goto free_adv;
    }

    ret = bme_init_debugfs(adv);
    if (ret) {
        dev_err(bdev->dev, "BME280: Debugfs initialization failed: %d\n", ret);
        goto free_dma;
    }

    adv->irq = platform_get_irq(to_platform_device(bdev->dev), 0);
    if (adv->irq >= 0) {
        ret = request_irq(adv->irq, bme_irq_handler, IRQF_TRIGGER_RISING, BME280_IRQ_NAME, adv);
        if (ret) {
            dev_err(bdev->dev, "BME280: Failed to request IRQ %d: %d\n", adv->irq, ret);
            goto free_debugfs;
        }
        pr_info("BME280: IRQ %d registered for device %d\n", adv->irq, bdev->id);
    }

    advanced_data[idx] = adv;
    pr_info("BME280: Advanced features initialized for device %d\n", bdev->id);

    return 0;

free_debugfs:
    debugfs_remove_recursive(adv->debugfs_root);
free_dma:
    if (adv->dma_chan) {
        dma_release_channel(adv->dma_chan);
    }
free_adv:
    kfree(adv);
    return ret;
}

void bme_advanced_cleanup(struct bme_device *bdev)
{
    struct bme_advanced *adv = NULL;
    int i;

    for (i = 0; i < MAX_DEVICES; i++) {
        if (advanced_data[i] && advanced_data[i]->bdev == bdev) {
            adv = advanced_data[i];
            break;
        }
    }
    if (!adv) {
        return;
    }

    if (adv->irq >= 0) {
        free_irq(adv->irq, adv);
    }
    if (adv->dma_chan) {
        dma_release_channel(adv->dma_chan);
    }
    debugfs_remove_recursive(adv->debugfs_root);
    kfree(adv);
    advanced_data[i] = NULL;
    pr_info("BME280: Advanced features cleaned up for device %d\n", bdev->id);
}

void bme_advanced_track_read(struct bme_device *bdev, unsigned long start_time)
{
    struct bme_advanced *adv = NULL;
    int i;

    for (i = 0; i < MAX_DEVICES; i++) {
        if (advanced_data[i] && advanced_data[i]->bdev == bdev) {
            adv = advanced_data[i];
            break;
        }
    }
    if (adv) {
        adv->read_count++;
        adv->total_latency += jiffies - start_time;
    }
}

void bme_advanced_track_error(struct bme_device *bdev)
{
    struct bme_advanced *adv = NULL;
    int i;

    for (i = 0; i < MAX_DEVICES; i++) {
        if (advanced_data[i] && advanced_data[i]->bdev == bdev) {
            adv = advanced_data[i];
            break;
        }
    }
    if (adv) {
        adv->error_count++;
    }
}

MODULE_AUTHOR("Auto-generated advanced BME280 driver");
MODULE_DESCRIPTION("Advanced features for BME280 kernel driver");
MODULE_LICENSE("GPL v2");