/*
 * concurrency_user.c - User-space program to interact with BME280 kernel driver
 */

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/poll.h>
#include <string.h>
#include <errno.h>
#include <linux/futex.h>
#include <sys/syscall.h>
#include <liburing.h>
#include "bme280_ioctls.h"

#define DEVICE_PATH "/dev/bme2800"
#define SYSFS_PATH "/sys/class/bme280_class/bme2800/config"

int main(void)
{
    int fd, ret;
    int32_t temp;
    uint32_t press, hum, deadline = 1000; /* Deadline in ms for QoS */
    uint8_t chip_id;
    char buf[256];
    struct pollfd pfd = {0};
    struct io_uring ring;
    struct io_uring_sqe *sqe;
    struct io_uring_cqe *cqe;

    fd = open(DEVICE_PATH, O_RDWR);
    if (fd < 0) {
        perror("Failed to open device");
        return 1;
    }

    /* ADDED FOR ESSENTIAL FIX: Retry on ioctl fail */
    int retry = 3;
    while (retry--) {
        ret = ioctl(fd, BME280_IOCTL_GET_CHIPID, &chip_id);
        if (ret < 0) {
            perror("Failed to read chip ID");
            sleep(1);
            continue;
        }
        break;
    }
    if (ret < 0) goto cleanup;
    printf("BME280 Chip ID: 0x%02x\n", chip_id);

    retry = 3;
    while (retry--) {
        ret = ioctl(fd, BME280_IOCTL_GET_TEMPERATURE, &temp);
        if (ret < 0) {
            perror("Failed to read temperature");
            sleep(1);
            continue;
        }
        break;
    }
    if (ret < 0) goto cleanup;
    printf("Temperature: %.2f C\n", temp / 1000.0);

    retry = 3;
    while (retry--) {
        ret = ioctl(fd, BME280_IOCTL_GET_PRESSURE, &press);
        if (ret < 0) {
            perror("Failed to read pressure");
            sleep(1);
            continue;
        }
        break;
    }
    if (ret < 0) goto cleanup;
    printf("Pressure: %.2f hPa\n", press / 100.0);

    retry = 3;
    while (retry--) {
        ret = ioctl(fd, BME280_IOCTL_GET_HUMIDITY, &hum);
        if (ret < 0) {
            perror("Failed to read humidity");
            sleep(1);
            continue;
        }
        break;
    }
    if (ret < 0) goto cleanup;
    printf("Humidity: %.2f%%\n", hum / 1000.0);

    ret = ioctl(fd, BME280_IOCTL_FUTEX_TEST);
    if (ret < 0) {
        perror("Failed to test futex");
        goto cleanup;
    }
    printf("Futex test successful\n");

    ret = ioctl(fd, BME280_IOCTL_RCU_UPDATE);
    if (ret < 0) {
        perror("Failed to test RCU update");
        goto cleanup;
    }
    printf("RCU update test successful\n");

    ret = ioctl(fd, BME280_IOCTL_NUMA_ALLOC);
    if (ret < 0) {
        perror("Failed to test NUMA allocation");
        goto cleanup;
    }
    printf("NUMA allocation test successful\n");

    ret = ioctl(fd, BME280_IOCTL_QOS_SUBMIT, &deadline);
    if (ret < 0) {
        perror("Failed to test QoS");
        goto cleanup;
    }
    printf("QoS test successful with deadline %u ms\n", deadline);

    pfd.fd = fd;
    pfd.events = POLLIN;
    ret = poll(&pfd, 1, 1000);
    if (ret > 0 && (pfd.revents & POLLIN)) {
        ret = read(fd, buf, sizeof(buf));
        if (ret > 0) {
            buf[ret] = '\0';
            printf("Polled data: %s\n", buf);
        } else {
            perror("Failed to read polled data");
        }
    } else {
        printf("Poll timeout or error: %d\n", ret);
    }

    ret = io_uring_queue_init(8, &ring, 0);
    if (ret) {
        perror("Failed to initialize io_uring");
        goto cleanup;
    }

    sqe = io_uring_get_sqe(&ring);
    if (!sqe) {
        perror("Failed to get io_uring SQE");
        io_uring_queue_exit(&ring);
        goto cleanup;
    }
    io_uring_prep_read(sqe, fd, buf, sizeof(buf), 0);
    ret = io_uring_submit(&ring);
    if (ret < 0) {
        perror("Failed to submit io_uring request");
        io_uring_queue_exit(&ring);
        goto cleanup;
    }

    ret = io_uring_wait_cqe(&ring, &cqe);
    if (ret < 0) {
        perror("Failed to wait for io_uring CQE");
        io_uring_queue_exit(&ring);
        goto cleanup;
    }
    if (cqe->res > 0) {
        buf[cqe->res] = '\0';
        printf("io_uring read: %s\n", buf);
    } else {
        printf("io_uring read error: %d\n", cqe->res);
    }
    io_uring_cqe_seen(&ring, cqe);
    io_uring_queue_exit(&ring);

    int sysfs_fd = open(SYSFS_PATH, O_WRONLY);
    if (sysfs_fd < 0) {
        perror("Failed to open sysfs config");
        goto cleanup;
    }
    const char *config = "osrs_t=2 osrs_p=2 osrs_h=2 mode=3 t_sb=5 filter=0\n";
    ret = write(sysfs_fd, config, strlen(config));
    if (ret < 0) {
        perror("Failed to write sysfs config");
        close(sysfs_fd);
        goto cleanup;
    }
    close(sysfs_fd);
    printf("Sysfs configuration updated\n");

cleanup:
    close(fd);
    return ret < 0 ? 1 : 0;
}