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

/* assumes you named the DT node: max31855: in your overlay (node-label = max31855;) */
#define MAX31855_NODE DT_NODELABEL(max31856)
static const struct device *max_dev = DEVICE_DT_GET(MAX31855_NODE);

/* Custom sensor attributes for fault detection */
enum max31856_attribute {
    // MAX31856_ATTR_FAULT_OVUV = SENSOR_ATTR_PRIV_START,
    // MAX31856_ATTR_FAULT_OC,
    // MAX31856_ATTR_FAULT_TCRANGE,
    // MAX31856_ATTR_FAULT_CJRANGE,
    MAX31856_ATTR_FILTER_FREQ = SENSOR_ATTR_PRIV_START,
    MAX31856_ATTR_CJ_LOWER_THRESH,
    MAX31856_ATTR_CJ_UPPER_THRESH,
    MAX31856_ATTR_TC_LOWER_THRESH,
    MAX31856_ATTR_TC_UPPER_THRESH,
    MAX31856_ATTR_CJ_OFFSET,
    MAX31856_ATTR_FAULT_TYPE,
};
int main(void)
{	
    int err;
	/* thermocouple temp (thermocouple channel) */
	struct sensor_value tc_val;
	/* internal / ambient temp (chip die temperature) */
	struct sensor_value ambient_val;
    /* Sensor threshold value */
    struct sensor_value threshold;
    /* Fault status */
    struct sensor_value fault_status;

    if (!device_is_ready(max_dev)) {
        LOG_ERR("MAX31855 device not ready");
        return;
    }

    threshold.val1 = 30; // Set threshold to 100 degrees Celsius
    threshold.val2 = 0;   // No fractional part
    sensor_attr_set(max_dev, SENSOR_CHAN_ALL, MAX31856_ATTR_TC_UPPER_THRESH, &threshold);

	while (1) {
		
        /* ask driver to read from the hardware and store latest sample */
        err = sensor_sample_fetch(max_dev);
        if (err) {
            LOG_ERR("sensor_sample_fetch failed: %d", err);
            k_msleep(1000);
            continue;
        }

        err = sensor_channel_get(max_dev, SENSOR_CHAN_DIE_TEMP, &tc_val);
        if (err) {
            LOG_ERR("sensor_channel_get TEMP failed: %d", err);
        } else {
            double tc = sensor_value_to_double(&tc_val);
            LOG_INF("Thermocouple: %.2f °C", tc);
        }

        err = sensor_channel_get(max_dev, SENSOR_CHAN_AMBIENT_TEMP, &ambient_val);
        if (err) {
            LOG_ERR("sensor_channel_get AMBIENT failed: %d", err);
        } else {
            double amb = sensor_value_to_double(&ambient_val);
            LOG_INF("Cold junction: %.2f °C", amb);
        }

        /* Read fault status register */
        sensor_attr_get(max_dev, SENSOR_CHAN_ALL, MAX31856_ATTR_FAULT_TYPE, &fault_status);
        LOG_INF("Fault status: 0x%02x", fault_status.val1);

		k_msleep(SLEEP_TIME_MS);
	}
	return 0;
}
