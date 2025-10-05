/*
 * bme280_tool.c - User-space command-line tool for BME280 sensor
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/poll.h>
#include <time.h>
#include <getopt.h>
#include <errno.h>
//#include <linux/seqlock.h>
#include "lib_bme280.h"

#define DEVICE_PATH "/dev/bme2800"
#define SYSFS_PATH "/sys/class/bme280_class/bme2800/config"
#define LOG_FILE "bme280_log.csv"

/* ADDED FOR ESSENTIAL FIX: IIO sysfs paths for readings (if using IIO framework) */
#define IIO_TEMP_PATH "/sys/bus/iio/devices/iio:device0/in_temp_input"
#define IIO_PRESS_PATH "/sys/bus/iio/devices/iio:device0/in_pressure_input"
#define IIO_HUM_PATH "/sys/bus/iio/devices/iio:device0/in_humidityrelative_input"

void print_usage(const char *prog)
{
    printf("Usage: %s [options]\n", prog);
    printf("Options:\n");
    printf("  -r           Read sensor data (temperature, pressure, humidity)\n");
    printf("  -c CONFIG    Configure sensor (e.g., 'osrs_t=2 osrs_p=2 osrs_h=2 mode=3 t_sb=5 filter=0')\n");
    printf("  -l           Log sensor data to %s\n", LOG_FILE);
    printf("  -p SECONDS   Poll for data readiness for SECONDS seconds\n");
    printf("  -t           Test concurrency features (futex, RCU, NUMA, QoS)\n");
    printf("  -i           Read chip ID\n");
    printf("  -h           Show this help message\n");
}

int read_sensor_data(int fd, FILE *log_fp)
{
    int ret;
    int32_t temp;
    uint32_t press, hum;
    char buf[256];
    time_t now;
    struct tm *tm_info;

    /* ADDED FOR ESSENTIAL FIX: Fallback to IIO sysfs if ioctl fails (for IIO integration) */
    ret = ioctl(fd, BME280_IOCTL_GET_TEMPERATURE, &temp);
    if (ret < 0) {
        int sysfs_fd = open(IIO_TEMP_PATH, O_RDONLY);
        if (sysfs_fd >= 0) {
            ret = read(sysfs_fd, buf, sizeof(buf));
            temp = atoi(buf) * 1000;  // Adjust scale
            close(sysfs_fd);
        } else {
            perror("Failed to read temperature");
            return ret;
        }
    }
    /* Tương tự cho pressure và humidity */
    ret = ioctl(fd, BME280_IOCTL_GET_PRESSURE, &press);
    if (ret < 0) {
        int sysfs_fd = open(IIO_PRESS_PATH, O_RDONLY);
        if (sysfs_fd >= 0) {
            ret = read(sysfs_fd, buf, sizeof(buf));
            press = atoi(buf) * 100;  // Adjust scale
            close(sysfs_fd);
        } else {
            perror("Failed to read pressure");
            return ret;
        }
    }
    ret = ioctl(fd, BME280_IOCTL_GET_HUMIDITY, &hum);
    if (ret < 0) {
        int sysfs_fd = open(IIO_HUM_PATH, O_RDONLY);
        if (sysfs_fd >= 0) {
            ret = read(sysfs_fd, buf, sizeof(buf));
            hum = atoi(buf) * 1000;  // Adjust scale
            close(sysfs_fd);
        } else {
            perror("Failed to read humidity");
            return ret;
        }
    }

    printf("Temperature: %.2f C\n", temp / 1000.0);
    printf("Pressure: %.2f hPa\n", press / 100.0);
    printf("Humidity: %.2f%%\n", hum / 1000.0);

    if (log_fp) {
        time(&now);
        tm_info = localtime(&now);
        strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", tm_info);
        fprintf(log_fp, "%s,%.2f,%.2f,%.2f\n", buf, temp / 1000.0, press / 100.0, hum / 1000.0);
        fflush(log_fp);
    }

    return 0;
}

int configure_sensor(const char *config)
{
    int sysfs_fd = open(SYSFS_PATH, O_WRONLY);
    if (sysfs_fd < 0) {
        perror("Failed to open sysfs config");
        return -1;
    }
    int ret = write(sysfs_fd, config, strlen(config));
    if (ret < 0) {
        perror("Failed to write sysfs config");
        close(sysfs_fd);
        return ret;
    }
    printf("Configuration applied: %s\n", config);
    close(sysfs_fd);
    return 0;
}

int test_concurrency(int fd)
{
    int ret;
    uint32_t deadline = 1000; /* 1 second deadline for QoS */

    printf("Testing futex...\n");
    ret = ioctl(fd, BME280_IOCTL_FUTEX_TEST);
    if (ret < 0) {
        perror("Futex test failed");
        return ret;
    }
    printf("Futex test successful\n");

    printf("Testing RCU update...\n");
    ret = ioctl(fd, BME280_IOCTL_RCU_UPDATE);
    if (ret < 0) {
        perror("RCU update test failed");
        return ret;
    }
    printf("RCU update test successful\n");

    printf("Testing NUMA allocation...\n");
    ret = ioctl(fd, BME280_IOCTL_NUMA_ALLOC);
    if (ret < 0) {
        perror("NUMA allocation test failed");
        return ret;
    }
    printf("NUMA allocation test successful\n");

    printf("Testing QoS with deadline %u ms...\n", deadline);
    ret = ioctl(fd, BME280_IOCTL_QOS_SUBMIT, &deadline);
    if (ret < 0) {
        perror("QoS test failed");
        return ret;
    }
    printf("QoS test successful\n");

    return 0;
}

int poll_sensor(int fd, int timeout)
{
    struct pollfd pfd = { .fd = fd, .events = POLLIN };
    char buf[256];
    int ret;

    printf("Polling for %d seconds...\n", timeout);
    ret = poll(&pfd, 1, timeout * 1000);
    if (ret > 0 && (pfd.revents & POLLIN)) {
        ret = read(fd, buf, sizeof(buf));
        if (ret > 0) {
            buf[ret] = '\0';
            printf("Polled data: %s\n", buf);
        } else {
            perror("Failed to read polled data");
            return ret;
        }
    } else if (ret == 0) {
        printf("Poll timeout\n");
    } else {
        perror("Poll error");
        return ret;
    }
    return 0;
}

int read_chip_id(int fd)
{
    uint8_t chip_id;
    int ret = ioctl(fd, BME280_IOCTL_GET_CHIPID, &chip_id);
    if (ret < 0) {
        perror("Failed to read chip ID");
        return ret;
    }
    printf("BME280 Chip ID: 0x%02x\n", chip_id);
    return 0;
}

int main(int argc, char *argv[])
{
    int opt, fd, ret = 0;
    int do_read = 0, do_log = 0, do_test = 0, do_id = 0, poll_timeout = 0;
    const char *config = NULL;
    FILE *log_fp = NULL;

    while ((opt = getopt(argc, argv, "rc:lp:tih")) != -1) {
        switch (opt) {
        case 'r':
            do_read = 1;
            break;
        case 'c':
            config = optarg;
            break;
        case 'l':
            do_log = 1;
            break;
        case 'p':
            poll_timeout = atoi(optarg);
            break;
        case 't':
            do_test = 1;
            break;
        case 'i':
            do_id = 1;
            break;
        case 'h':
        default:
            print_usage(argv[0]);
            return 1;
        }
    }

    fd = open(DEVICE_PATH, O_RDWR);
    if (fd < 0) {
        perror("Failed to open device");
        return 1;
    }

    if (do_log) {
        log_fp = fopen(LOG_FILE, "a");
        if (!log_fp) {
            perror("Failed to open log file");
            close(fd);
            return 1;
        }
        fprintf(log_fp, "Timestamp,Temperature(C),Pressure(hPa),Humidity(%%)\n");
    }

    if (config) {
        ret = configure_sensor(config);
        if (ret < 0) {
            goto cleanup;
        }
    }

    if (do_id) {
        ret = read_chip_id(fd);
        if (ret < 0) {
            goto cleanup;
        }
    }

    if (do_read || do_log) {
        ret = read_sensor_data(fd, log_fp);
        if (ret < 0) {
            goto cleanup;
        }
    }

    if (poll_timeout > 0) {
        ret = poll_sensor(fd, poll_timeout);
        if (ret < 0) {
            goto cleanup;
        }
    }

    if (do_test) {
        ret = test_concurrency(fd);
        if (ret < 0) {
            goto cleanup;
        }
    }

cleanup:
    if (log_fp) {
        fclose(log_fp);
    }
    close(fd);
    return ret < 0 ? 1 : 0;
}