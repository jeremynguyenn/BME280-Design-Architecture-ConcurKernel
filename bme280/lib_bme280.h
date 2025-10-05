/*
 * lib_bme280.h - Helper declarations for BME280 sensor access for I2C and SPI
 */

#ifndef _LIB_BME280_H
#define _LIB_BME280_H

#ifndef __KERNEL__
#include <stdint.h>  /* For uint16_t, int16_t, etc. in user-space */
typedef uint8_t u8;
typedef uint16_t u16;
typedef int16_t s16;
typedef int32_t s32;
typedef uint32_t u32;
typedef int8_t s8;
#else
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/types.h>
#include <linux/seqlock.h>  /* For seqlock in shared config */
#include <linux/kfifo.h>    /* For lock-free ring buffers */
#include <linux/rcupdate.h> /* For RCU */
/* ADDED FOR ESSENTIAL FIX: IIO related includes */
#include <linux/iio/iio.h>  /* For IIO integration */
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger.h>
#endif

struct bme280_calib {
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;
    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;
    uint8_t  dig_H1;
    int16_t  dig_H2;
    uint8_t  dig_H3;
    int16_t  dig_H4;
    int16_t  dig_H5;
    int8_t   dig_H6;
};

struct bme_shared_config {
#ifdef __KERNEL__
    seqlock_t seqlock;
#endif
    u8 osrs_t;
    u8 osrs_p;
    u8 osrs_h;
    u8 mode;
    u8 config_t_sb;
    u8 config_filter;
};

#ifdef __KERNEL__
int read_regs_i2c(struct i2c_client *client, u8 reg, u8 *buf, int len);
int write_reg_i2c(struct i2c_client *client, u8 reg, u8 val);
int bme280_init_sensor_i2c(struct i2c_client *client, struct bme280_calib *calib);
int bme280_read_chip_id_i2c(struct i2c_client *client, uint8_t *id);
int bme280_read_temperature_i2c(struct i2c_client *client, struct bme280_calib *calib, int32_t *t_fine, int32_t *out_milli_c);
int bme280_read_pressure_i2c(struct i2c_client *client, struct bme280_calib *calib, int32_t t_fine, uint32_t *out_pa);
int bme280_read_humidity_i2c(struct i2c_client *client, struct bme280_calib *calib, int32_t t_fine, uint32_t *out_thousand_rh);
int bme280_apply_settings_i2c(struct i2c_client *client,
                              uint8_t osrs_t, uint8_t osrs_p, uint8_t osrs_h,
                              uint8_t t_sb, uint8_t filter, uint8_t mode);
int bme280_set_mode_i2c(struct i2c_client *client, uint8_t mode);

int read_regs_spi(struct spi_device *spi, u8 reg, u8 *buf, int len);
int write_reg_spi(struct spi_device *spi, u8 reg, u8 val);
int bme280_init_sensor_spi(struct spi_device *spi, struct bme280_calib *calib);
int bme280_read_chip_id_spi(struct spi_device *spi, uint8_t *id);
int bme280_read_temperature_spi(struct spi_device *spi, struct bme280_calib *calib, int32_t *t_fine, int32_t *out_milli_c);
int bme280_read_pressure_spi(struct spi_device *spi, struct bme280_calib *calib, int32_t t_fine, uint32_t *out_pa);
int bme280_read_humidity_spi(struct spi_device *spi, struct bme280_calib *calib, int32_t t_fine, uint32_t *out_thousand_rh);
int bme280_apply_settings_spi(struct spi_device *spi,
                              uint8_t osrs_t, uint8_t osrs_p, uint8_t osrs_h,
                              uint8_t t_sb, uint8_t filter, uint8_t mode);
int bme280_set_mode_spi(struct spi_device *spi, uint8_t mode);

int bme_read_shared_config(struct bme_shared_config *cfg, u8 *osrs_t);
int bme_update_shared_config(struct bme_shared_config *cfg, u8 osrs_t);  // Sửa để khớp: int và u8 osrs_t
void *bme_alloc_numa_buffer(size_t size);
int bme_futex_helper(int *futex_var, int op);
void bme_ring_helper(void);
void bme_ebpf_helper(void);
void bme_userfaultfd_helper(struct vm_area_struct *vma);
void bme_submit_work(void);
void bme_tsn_helper(void);
void bme_vfio_helper(void);
void bme_tsx_helper(void);
void bme_scheduler_helper(void);
void bme_irq_steering_helper(int irq);
void bme_energy_helper(void);
void bme_doorbell_helper(void);
void bme_qos_helper(uint32_t deadline);
void bme_crash_helper(void);
void bme_rcu_helper(void);
void bme_cache_helper(void);
void bme_page_table_helper(struct vm_area_struct *vma);
void bme_replay_helper(int val);
void bme_hetero_helper(void);
void bme_pmem_helper(void);
void bme_verify_helper(void);
#endif  /* __KERNEL__ */

#ifndef __KERNEL__
/* IOCTL definitions for user-space (guessed based on usage in bme280_tool.c) */
#define BME280_MAGIC 'b'

#define BME280_IOCTL_GET_TEMPERATURE _IOR(BME280_MAGIC, 1, int32_t)
#define BME280_IOCTL_GET_PRESSURE    _IOR(BME280_MAGIC, 2, uint32_t)
#define BME280_IOCTL_GET_HUMIDITY    _IOR(BME280_MAGIC, 3, uint32_t)
#define BME280_IOCTL_GET_CHIPID      _IOR(BME280_MAGIC, 4, uint8_t)
#define BME280_IOCTL_FUTEX_TEST      _IO(BME280_MAGIC, 5)
#define BME280_IOCTL_RCU_UPDATE      _IO(BME280_MAGIC, 6)
#define BME280_IOCTL_NUMA_ALLOC      _IO(BME280_MAGIC, 7)
#define BME280_IOCTL_QOS_SUBMIT      _IOW(BME280_MAGIC, 8, uint32_t)
/* Add more if needed */
#endif

#endif /* _LIB_BME280_H */