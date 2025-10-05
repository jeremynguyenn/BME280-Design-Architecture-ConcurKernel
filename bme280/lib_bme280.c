/*
 * lib_bme280.c - BME280 helper implementation for I2C and SPI
 * Provides functions to initialize, read, and configure the BME280 sensor.
 * Includes support for calibration data handling and concurrency mechanisms.
 */

#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/smp.h>
#include <linux/seqlock.h>  /* For seqlock in shared config */
#include "lib_bme280.h"
#include <linux/kfifo.h>    /* For lock-free ring buffers */
#include <linux/rcupdate.h> /* For RCU */

#define BME280_REG_ID            0xD0
#define BME280_REG_RESET         0xE0
#define BME280_REG_CTRL_HUM      0xF2
#define BME280_REG_STATUS        0xF3
#define BME280_REG_CTRL_MEAS     0xF4
#define BME280_REG_CONFIG        0xF5
#define BME280_REG_PRESS_MSB     0xF7
#define BME280_REG_TEMP_MSB      0xFA
#define BME280_REG_HUM_MSB       0xFD

int read_regs_i2c(struct i2c_client *client, u8 reg, u8 *buf, int len)
{
    int ret, retry = 3;  /* ADDED FOR ESSENTIAL FIX: Retry on error */
    while (retry--) {
        ret = i2c_smbus_read_i2c_block_data(client, reg, len, buf);
        if (ret < 0) {
            dev_err(&client->dev, "BME280: I2C read failed at reg 0x%02x: %d (retry %d)\n", reg, ret, retry);  /* ADDED: dev_err */
            msleep(10);  /* Delay before retry */
            continue;
        }
        if (ret != len) {
            dev_err(&client->dev, "BME280: I2C read incomplete at reg 0x%02x, expected %d, got %d\n", reg, len, ret);
            return -EIO;
        }
        smp_mb();
        return 0;
    }
    return ret;
}

int write_reg_i2c(struct i2c_client *client, u8 reg, u8 val)
{
    int ret, retry = 3;
    while (retry--) {
        ret = i2c_smbus_write_byte_data(client, reg, val);
        if (ret < 0) {
            dev_err(&client->dev, "BME280: I2C write failed at reg 0x%02x: %d (retry %d)\n", reg, ret, retry);
            msleep(10);
            continue;
        }
        smp_mb();
        return ret;
    }
    return ret;
}

int read_regs_spi(struct spi_device *spi, u8 reg, u8 *buf, int len)
{
    int ret, retry = 3;
    struct spi_transfer t[2];
    struct spi_message m;
    u8 cmd = reg | 0x80; 
    u8 *rxbuf;

    while (retry--) {
        rxbuf = kzalloc(len, GFP_KERNEL);
        if (!rxbuf) {
            dev_err(&spi->dev, "BME280: SPI read memory allocation failed\n");
            return -ENOMEM;
        }

        memset(t, 0, sizeof(t));
        spi_message_init(&m);

        t[0].tx_buf = &cmd;
        t[0].len = 1;
        spi_message_add_tail(&t[0], &m);

        t[1].rx_buf = rxbuf;
        t[1].len = len;
        spi_message_add_tail(&t[1], &m);

        ret = spi_sync(spi, &m);
        if (ret < 0) {
            dev_err(&spi->dev, "BME280: SPI read failed at reg 0x%02x: %d (retry %d)\n", reg, ret, retry);
            kfree(rxbuf);
            msleep(10);
            continue;
        }

        memcpy(buf, rxbuf, len);
        kfree(rxbuf);
        smp_mb();
        return 0;
    }
    return ret;
}

int write_reg_spi(struct spi_device *spi, u8 reg, u8 val)
{
    int ret, retry = 3;
    u8 tx[2];
    struct spi_transfer t = { .tx_buf = tx, .len = 2 };
    struct spi_message m;

    while (retry--) {
        tx[0] = reg & 0x7F; 
        tx[1] = val;
        spi_message_init(&m);
        spi_message_add_tail(&t, &m);

        ret = spi_sync(spi, &m);
        if (ret < 0) {
            dev_err(&spi->dev, "BME280: SPI write failed at reg 0x%02x: %d (retry %d)\n", reg, ret, retry);
            msleep(10);
            continue;
        }
        smp_mb();
        return ret;
    }
    return ret;
}

int bme280_init_sensor_i2c(struct i2c_client *client, struct bme280_calib *calib)
{
    int ret;
    u8 buf[26];

    ret = write_reg_i2c(client, BME280_REG_RESET, 0xB6);  // Reset value from datasheet
    if (ret) return ret;
    msleep(10);  // Wait for reset

    // Read calibration data
    ret = read_regs_i2c(client, 0x88, buf, 24);
    if (ret) return ret;
    calib->dig_T1 = (buf[1] << 8) | buf[0];
    calib->dig_T2 = (buf[3] << 8) | buf[2];
    calib->dig_T3 = (buf[5] << 8) | buf[4];
    calib->dig_P1 = (buf[7] << 8) | buf[6];
    calib->dig_P2 = (buf[9] << 8) | buf[8];
    calib->dig_P3 = (buf[11] << 8) | buf[10];
    calib->dig_P4 = (buf[13] << 8) | buf[12];
    calib->dig_P5 = (buf[15] << 8) | buf[14];
    calib->dig_P6 = (buf[17] << 8) | buf[16];
    calib->dig_P7 = (buf[19] << 8) | buf[18];
    calib->dig_P8 = (buf[21] << 8) | buf[20];
    calib->dig_P9 = (buf[23] << 8) | buf[22];

    ret = read_regs_i2c(client, 0xA1, &calib->dig_H1, 1);
    if (ret) return ret;

    ret = read_regs_i2c(client, 0xE1, buf, 7);
    if (ret) return ret;
    calib->dig_H2 = (buf[1] << 8) | buf[0];
    calib->dig_H3 = buf[2];
    calib->dig_H4 = (buf[3] << 4) | (buf[4] & 0x0F);
    calib->dig_H5 = (buf[5] << 4) | (buf[4] >> 4);
    calib->dig_H6 = buf[6];

    return 0;
}

int bme280_init_sensor_spi(struct spi_device *spi, struct bme280_calib *calib)
{
    int ret;
    u8 buf[26];

    ret = write_reg_spi(spi, BME280_REG_RESET, 0xB6);
    if (ret) return ret;
    msleep(10);

    ret = read_regs_spi(spi, 0x88, buf, 24);
    if (ret) return ret;
    calib->dig_T1 = (buf[1] << 8) | buf[0];
    calib->dig_T2 = (buf[3] << 8) | buf[2];
    calib->dig_T3 = (buf[5] << 8) | buf[4];
    calib->dig_P1 = (buf[7] << 8) | buf[6];
    calib->dig_P2 = (buf[9] << 8) | buf[8];
    calib->dig_P3 = (buf[11] << 8) | buf[10];
    calib->dig_P4 = (buf[13] << 8) | buf[12];
    calib->dig_P5 = (buf[15] << 8) | buf[14];
    calib->dig_P6 = (buf[17] << 8) | buf[16];
    calib->dig_P7 = (buf[19] << 8) | buf[18];
    calib->dig_P8 = (buf[21] << 8) | buf[20];
    calib->dig_P9 = (buf[23] << 8) | buf[22];

    ret = read_regs_spi(spi, 0xA1, &calib->dig_H1, 1);
    if (ret) return ret;

    ret = read_regs_spi(spi, 0xE1, buf, 7);
    if (ret) return ret;
    calib->dig_H2 = (buf[1] << 8) | buf[0];
    calib->dig_H3 = buf[2];
    calib->dig_H4 = (buf[3] << 4) | (buf[4] & 0x0F);
    calib->dig_H5 = (buf[5] << 4) | (buf[4] >> 4);
    calib->dig_H6 = buf[6];

    return 0;
}

int bme280_read_chip_id_i2c(struct i2c_client *client, uint8_t *id)
{
    int ret = read_regs_i2c(client, BME280_REG_ID, id, 1);
    if (ret) return ret;
    return 0;
}

int bme280_read_chip_id_spi(struct spi_device *spi, uint8_t *id)
{
    int ret = read_regs_spi(spi, BME280_REG_ID, id, 1);
    if (ret) return ret;
    return 0;
}

int bme280_read_temperature_i2c(struct i2c_client *client, struct bme280_calib *calib, int32_t *t_fine, int32_t *out_milli_c)
{
    int ret;
    u8 buf[3];
    int32_t adc_T, var1, var2;

    ret = read_regs_i2c(client, BME280_REG_TEMP_MSB, buf, 3);
    if (ret) return ret;
    adc_T = (buf[0] << 12) | (buf[1] << 4) | (buf[2] >> 4);

    var1 = ((((adc_T >> 3) - ((int32_t)calib->dig_T1 << 1))) * ((int32_t)calib->dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)calib->dig_T1)) * ((adc_T >> 4) - ((int32_t)calib->dig_T1))) >> 12) * ((int32_t)calib->dig_T3)) >> 14;
    *t_fine = var1 + var2;
    *out_milli_c = ((*t_fine * 5 + 128) >> 8) * 100;

    smp_mb();
    return 0;
}

int bme280_read_pressure_i2c(struct i2c_client *client, struct bme280_calib *calib, int32_t t_fine, uint32_t *out_pa)
{
    int ret;
    u8 buf[3];
    int64_t var1, var2, p;
    int32_t adc_P;

    ret = read_regs_i2c(client, BME280_REG_PRESS_MSB, buf, 3);
    if (ret) return ret;
    adc_P = (buf[0] << 12) | (buf[1] << 4) | (buf[2] >> 4);

    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)calib->dig_P6;
    var2 = var2 + ((var1 * (int64_t)calib->dig_P5) << 17);
    var2 = var2 + (((int64_t)calib->dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)calib->dig_P3) >> 8) + ((var1 * (int64_t)calib->dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)calib->dig_P1) >> 33;
    if (var1 == 0) {
        *out_pa = 0;
        return 0;
    }
    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)calib->dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)calib->dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)calib->dig_P7) << 4);
    *out_pa = (uint32_t)p;

    smp_mb();
    return 0;
}

int bme280_read_humidity_i2c(struct i2c_client *client, struct bme280_calib *calib, int32_t t_fine, uint32_t *out_thousand_rh)
{
    int ret;
    u8 buf[2];
    int32_t adc_H, v_x1_u32r;

    ret = read_regs_i2c(client, BME280_REG_HUM_MSB, buf, 2);
    if (ret) return ret;
    adc_H = (buf[0] << 8) | buf[1];

    v_x1_u32r = (t_fine - ((int32_t)76800));
    v_x1_u32r = (((((adc_H << 14) - (((int32_t)calib->dig_H4) << 20) - (((int32_t)calib->dig_H5) * v_x1_u32r)) +
                  ((int32_t)16384)) >> 15) * (((((((v_x1_u32r * ((int32_t)calib->dig_H6)) >> 10) *
                  (((v_x1_u32r * ((int32_t)calib->dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) *
                  ((int32_t)calib->dig_H2) + 8192) >> 14));
    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)calib->dig_H1)) >> 4));
    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
    *out_thousand_rh = (uint32_t)((v_x1_u32r >> 12) * 1000 / 1024);

    smp_mb();
    return 0;
}

int bme280_read_temperature_spi(struct spi_device *spi, struct bme280_calib *calib, int32_t *t_fine, int32_t *out_milli_c)
{
    int ret;
    u8 buf[3];
    int32_t adc_T, var1, var2;

    ret = read_regs_spi(spi, BME280_REG_TEMP_MSB, buf, 3);
    if (ret) return ret;
    adc_T = (buf[0] << 12) | (buf[1] << 4) | (buf[2] >> 4);

    var1 = ((((adc_T >> 3) - ((int32_t)calib->dig_T1 << 1))) * ((int32_t)calib->dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)calib->dig_T1)) * ((adc_T >> 4) - ((int32_t)calib->dig_T1)) ) >> 12) * ((int32_t)calib->dig_T3)) >> 14;
    *t_fine = var1 + var2;
    *out_milli_c = ((*t_fine * 5 + 128) >> 8) * 100;

    smp_mb();
    return 0;
}

int bme280_read_pressure_spi(struct spi_device *spi, struct bme280_calib *calib, int32_t t_fine, uint32_t *out_pa)
{
    int ret;
    u8 buf[3];
    int64_t var1, var2, p;
    int32_t adc_P;

    ret = read_regs_spi(spi, BME280_REG_PRESS_MSB, buf, 3);
    if (ret) return ret;
    adc_P = (buf[0] << 12) | (buf[1] << 4) | (buf[2] >> 4);

    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)calib->dig_P6;
    var2 = var2 + ((var1 * (int64_t)calib->dig_P5) << 17);
    var2 = var2 + (((int64_t)calib->dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)calib->dig_P3) >> 8) + ((var1 * (int32_t)calib->dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)calib->dig_P1) >> 33;
    if (var1 == 0) {
        *out_pa = 0;
        return 0;
    }
    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)calib->dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)calib->dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)calib->dig_P7) << 4);
    *out_pa = (uint32_t)p;

    smp_mb();
    return 0;
}

int bme280_read_humidity_spi(struct spi_device *spi, struct bme280_calib *calib, int32_t t_fine, uint32_t *out_thousand_rh)
{
    int ret;
    u8 buf[2];
    int32_t adc_H, v_x1_u32r;

    ret = read_regs_spi(spi, BME280_REG_HUM_MSB, buf, 2);
    if (ret) return ret;
    adc_H = (buf[0] << 8) | buf[1];

    v_x1_u32r = (t_fine - ((int32_t)76800));
    v_x1_u32r = (((((adc_H << 14) - (((int32_t)calib->dig_H4) << 20) - (((int32_t)calib->dig_H5) * v_x1_u32r)) +
                  ((int32_t)16384)) >> 15) * (((((((v_x1_u32r * ((int32_t)calib->dig_H6)) >> 10) *
                  (((v_x1_u32r * ((int32_t)calib->dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) *
                  ((int32_t)calib->dig_H2) + 8192) >> 14));
    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)calib->dig_H1)) >> 4));
    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
    *out_thousand_rh = (uint32_t)((v_x1_u32r >> 12) * 1000 / 1024);

    smp_mb();
    return 0;
}

int bme280_apply_settings_i2c(struct i2c_client *client,
                              uint8_t osrs_t, uint8_t osrs_p, uint8_t osrs_h,
                              uint8_t t_sb, uint8_t filter, uint8_t mode)
{
    int ret;

    ret = write_reg_i2c(client, BME280_REG_CTRL_HUM, osrs_h & 0x07);
    if (ret) return ret;

    ret = write_reg_i2c(client, BME280_REG_CTRL_MEAS, ((osrs_t & 0x07) << 5) | ((osrs_p & 0x07) << 2) | (mode & 0x03));
    if (ret) return ret;

    ret = write_reg_i2c(client, BME280_REG_CONFIG, ((t_sb & 0x07) << 5) | ((filter & 0x07) << 2));
    if (ret) return ret;

    smp_mb();
    return 0;
}

int bme280_apply_settings_spi(struct spi_device *spi,
                              uint8_t osrs_t, uint8_t osrs_p, uint8_t osrs_h,
                              uint8_t t_sb, uint8_t filter, uint8_t mode)
{
    int ret;

    ret = write_reg_spi(spi, BME280_REG_CTRL_HUM, osrs_h & 0x07);
    if (ret) return ret;

    ret = write_reg_spi(spi, BME280_REG_CTRL_MEAS, ((osrs_t & 0x07) << 5) | ((osrs_p & 0x07) << 2) | (mode & 0x03));
    if (ret) return ret;

    ret = write_reg_spi(spi, BME280_REG_CONFIG, ((t_sb & 0x07) << 5) | ((filter & 0x07) << 2));
    if (ret) return ret;

    smp_mb();
    return 0;
}

int bme280_set_mode_i2c(struct i2c_client *client, uint8_t mode)
{
    int ret;
    u8 ctrl_meas;

    ret = read_regs_i2c(client, BME280_REG_CTRL_MEAS, &ctrl_meas, 1);
    if (ret) return ret;

    ctrl_meas = (ctrl_meas & ~0x03) | (mode & 0x03);
    ret = write_reg_i2c(client, BME280_REG_CTRL_MEAS, ctrl_meas);
    if (ret) return ret;

    smp_mb();
    return 0;
}

int bme280_set_mode_spi(struct spi_device *spi, uint8_t mode)
{
    int ret;
    u8 ctrl_meas;

    ret = read_regs_spi(spi, BME280_REG_CTRL_MEAS, &ctrl_meas, 1);
    if (ret) return ret;

    ctrl_meas = (ctrl_meas & ~0x03) | (mode & 0x03);
    ret = write_reg_spi(spi, BME280_REG_CTRL_MEAS, ctrl_meas);
    if (ret) return ret;

    smp_mb();
    return 0;
}