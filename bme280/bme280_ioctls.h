#ifndef BME280_IOCTLS_H
#define BME280_IOCTLS_H

#include <sys/ioctl.h>
#include <stdint.h>  /* for int32_t, uint32_t, uint8_t */

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

/* ADDED FOR ESSENTIAL FIX: Note for migration to IIO (no ioctl needed) */
/* In IIO mode, use sysfs instead of ioctl for readings */

#endif /* BME280_IOCTLS_H */