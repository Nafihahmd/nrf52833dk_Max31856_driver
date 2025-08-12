/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/sensor.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(max31855_example, LOG_LEVEL_INF);

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led1)

/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

#define SPIOP	SPI_WORD_SET(8) | SPI_TRANSFER_MSB	//SPI operation flag
struct spi_dt_spec spispec = SPI_DT_SPEC_GET(DT_NODELABEL(max31855), SPIOP, 0);

/* assumes you named the DT node: max31855: in your overlay (node-label = max31855;) */
#define MAX31855_NODE DT_NODELABEL(max31855)
static const struct device *max_dev = DEVICE_DT_GET(MAX31855_NODE);

/* call this when you want a fresh reading */
int read_max31855_once(double *tc_c, double *internal_c)
{
    int ret;

    if (!spi_is_ready_dt(&spispec)) {
        LOG_ERR("SPI device not ready");
        return -ENODEV;
    }

    uint8_t rx_buf[4] = {0};
    struct spi_buf rx = {
        .buf = rx_buf,
        .len = sizeof(rx_buf),
    };
    struct spi_buf_set rx_set = {
        .buffers = &rx,
        .count = 1,
    };

    /* MAX31855 outputs data on MISO; a simple read of 4 bytes is enough */
    ret = spi_read_dt(&spispec, &rx_set);
    if (ret) {
        LOG_ERR("spi_read_dt failed: %d", ret);
        return ret;
    }

    /* pack into 32-bit MSB-first */
    uint32_t raw = (uint32_t)rx_buf[0] << 24 |
                   (uint32_t)rx_buf[1] << 16 |
                   (uint32_t)rx_buf[2] << 8  |
                   (uint32_t)rx_buf[3];

    /* check fault flag (bit 16) */
    if (raw & (1u << 16)) {
        uint8_t fault = raw & 0x7u; /* bits D2..D0 */
        LOG_ERR("MAX31855 fault detected, code: %u", fault);
        /* You can decode:
           1 -> short to VCC, 2 -> short to GND, 4 -> open-circuit
           (check datasheet for exact mapping) */
        return -EIO;
    }

    /* Thermocouple temperature: bits 31..18 (14-bit signed), LSB = 0.25 C */
    int32_t tc_raw = (int32_t)(raw >> 18) & 0x3FFF; /* keep 14 bits */
    /* sign-extend 14-bit value to 32-bit signed */
    if (tc_raw & (1 << 13)) {
        tc_raw |= ~((1 << 14) - 1);
    }
    *tc_c = tc_raw * 0.25;

    /* Internal temp: bits 15..4 (12-bit signed), LSB = 0.0625 C */
    int32_t int_raw = (int32_t)((raw >> 4) & 0x0FFF);
    if (int_raw & (1 << 11)) {
        int_raw |= ~((1 << 12) - 1);
    }
    *internal_c = int_raw * 0.0625;

    return 0;
}

int main(void)
{
	int ret;
	bool led_state = true;

	if (!gpio_is_ready_dt(&led)) {
		return 0;
	}

	// ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	// if (ret < 0) {
	// 	return 0;
	// }
	// double tc_c, internal_c;
	
    int err;

    if (!device_is_ready(max_dev)) {
        LOG_ERR("MAX31855 device not ready");
        return;
    }

	while (1) {
		// ret = gpio_pin_toggle_dt(&led);
		// ret = read_max31855_once(&tc_c, &internal_c);
		if (ret < 0) {
			return 0;
		}
		
        /* ask driver to read from the hardware and store latest sample */
        err = sensor_sample_fetch(max_dev);
        if (err) {
            LOG_ERR("sensor_sample_fetch failed: %d", err);
            k_msleep(1000);
            continue;
        }
		 /* thermocouple temp (thermocouple channel) */
        struct sensor_value tc_val;
        err = sensor_channel_get(max_dev, SENSOR_CHAN_DIE_TEMP, &tc_val);
        if (err) {
            LOG_ERR("sensor_channel_get TEMP failed: %d", err);
        } else {
            double tc = sensor_value_to_double(&tc_val);
            LOG_INF("Thermocouple: %.2f °C", tc);
        }

        /* internal / ambient temp (chip die temperature) */
        struct sensor_value ambient_val;
        err = sensor_channel_get(max_dev, SENSOR_CHAN_AMBIENT_TEMP, &ambient_val);
        if (err) {
            LOG_ERR("sensor_channel_get AMBIENT failed: %d", err);
        } else {
            double amb = sensor_value_to_double(&ambient_val);
            LOG_INF("Internal ambient: %.2f °C", amb);
        }

		// led_state = !led_state;
		// printf("LED state: %s\n", led_state ? "ON" : "OFF");
		// printf("Thermocouple Temp: %.2f C\n", tc_c);
		// printf("Internal Temp: %.2f C\n", internal_c);
		k_msleep(SLEEP_TIME_MS);
	}
	return 0;
}
