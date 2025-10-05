/*
 * advanced_bme280.h - Header for advanced BME280 driver features
 *
 * Declares functions for advanced concurrency and kernel features
 * used in the BME280 driver, including DMA, IRQ, debugfs, and performance tracking.
 * Uses structures from lib_bme280.h to avoid redefinition.
 */

#ifndef __ADVANCED_BME280_H__
#define __ADVANCED_BME280_H__

#include <linux/seqlock.h>
#include <linux/jiffies.h>
#include <linux/device.h>
#include "lib_bme280.h"  // Include for bme_shared_config and other shared definitions

/* Forward declaration for bme_device to avoid circular includes */
struct bme_device;

/* Function declarations for advanced features */
int bme_advanced_init(struct bme_device *bdev);
void bme_advanced_cleanup(struct bme_device *bdev);
void bme_advanced_track_error(struct bme_device *bdev);
void bme_advanced_track_read(struct bme_device *bdev, unsigned long start_time);

/* ADDED FOR ESSENTIAL FIX: Forward decl for pm_ops if needed */
struct dev_pm_ops;

#endif /* __ADVANCED_BME280_H__ */