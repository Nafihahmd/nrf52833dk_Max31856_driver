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
#include "max31856.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(max31856_app, LOG_LEVEL_INF);

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000

/* assumes you named the DT node: max31855: in your overlay (node-label = max31855;) */
#define MAX31855_NODE DT_NODELABEL(max31856)
static const struct device *max_dev = DEVICE_DT_GET(MAX31855_NODE);
// static const struct device *external_cj = DEVICE_DT_GET(DT_NODELABEL(temp));

#ifdef CONFIG_MAX31856_FAULT_TRIGGER
/* Fault trigger handler */
static void fault_trigger_handler(const struct device *dev,
                                 const struct sensor_trigger *trig)
{
    struct sensor_value fault_status;
    
    /* Read the fault status */
    sensor_attr_get(dev, SENSOR_CHAN_ALL, MAX31856_ATTR_FAULT_TYPE, &fault_status);
    
    LOG_DBG("Fault detected: 0x%02x\n", fault_status.val1);
    
    /* Check specific fault bits */
    if (fault_status.val1 & MAX31856_FAULT_OPEN) {
        LOG_ERR("  - Open circuit fault\n");
    }
    if (fault_status.val1 & MAX31856_FAULT_OVUV) {
        LOG_ERR("  - Overvoltage/undervoltage fault\n");
    }
    if( fault_status.val1 & MAX31856_FAULT_TCLOW) {
        LOG_ERR("  - Thermocouple low threshold fault\n");
    }
    if (fault_status.val1 & MAX31856_FAULT_TCHIGH) {
        LOG_ERR("  - Thermocouple high threshold fault\n");
    }
    if (fault_status.val1 & MAX31856_FAULT_CJLOW) {
        LOG_ERR("  - Cold junction low threshold fault\n");
    }
    if (fault_status.val1 & MAX31856_FAULT_CJHIGH) {
        LOG_ERR("  - Cold junction high threshold fault\n");
    }
    if (fault_status.val1 & MAX31856_FAULT_TCRANGE) {
        LOG_ERR("  - Thermocouple out of range\n");
    }
    if (fault_status.val1 & MAX31856_FAULT_CJRANGE) {
        LOG_ERR("  - Cold junction out of range\n");
    }
}
#endif
#ifdef CONFIG_MAX31856_DRDY_TRIGGER
/* Data ready trigger handler */
static void drdy_trigger_handler(const struct device *dev,
                                const struct sensor_trigger *trig)
{
    struct sensor_value thermocouple, cold_junction;
    
    /* Read the temperature values */
    sensor_channel_get(dev, SENSOR_CHAN_DIE_TEMP, &thermocouple);
    sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &cold_junction);
    
    LOG_INF("Data ready: Thermocouple: %d.%06d°C, Cold Junction: %d.%06d°C\n",
           thermocouple.val1, thermocouple.val2,
           cold_junction.val1, cold_junction.val2);
}
#endif

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
    /* External CJ value */
    // struct sensor_value external_cj_val;
    if (!device_is_ready(max_dev)) {
        LOG_ERR("MAX31855 device not ready");
        return -1;
    }
        
#ifdef CONFIG_MAX31856_DRDY_TRIGGER
    /* Configure drdy trigger */
    struct sensor_trigger drdy_trigger_cfg = {
        .type = MAX31856_TRIGGER_DATA_READY,
        .chan = SENSOR_CHAN_ALL,
    };
    /* Set up data ready trigger */
    sensor_trigger_set(max_dev, &drdy_trigger_cfg, drdy_trigger_handler);
#endif
#ifdef CONFIG_MAX31856_FAULT_TRIGGER
    /* sensor trigger*/
    struct sensor_trigger fault_trigger = {
        .type = MAX31856_TRIGGER_FAULT,
        .chan = SENSOR_CHAN_ALL,
    };
     /* Set up fault trigger */
    sensor_trigger_set(max_dev, &fault_trigger, fault_trigger_handler);
#endif

    k_msleep(100); /* Wait for sensor to stabilize */
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

        // err = sensor_sample_fetch(external_cj);
        // if (err) {
        //     LOG_ERR("sensor_sample_fetch_chan failed: %d", err);
        // } else {
        //     /* Read external cold junction temperature */
        //     sensor_channel_get(external_cj, SENSOR_CHAN_DIE_TEMP, &external_cj_val);
        //     double ext_temp = sensor_value_to_double(&external_cj_val);
        //     LOG_INF("*External Cold Junction: %.2f °C", ext_temp);
        // }

        /* Set external cold junction temperature */
        // err = sensor_attr_set(max_dev, SENSOR_CHAN_ALL, MAX31856_ATTR_CJ_TEMP, &external_cj_val);
        // if (err) {
        //     LOG_ERR("sensor_attr_set CJ_TEMP failed: %d", err);
        // } 

        /* Read thermocouple and cold junction temperatures */
        // err = sensor_channel_get(max_dev, SENSOR_CHAN_DIE_TEMP, &tc_val);
        // if (err) {
        //     LOG_ERR("sensor_channel_get TEMP failed: %d", err);
        // } else {
        //     double tc = sensor_value_to_double(&tc_val);
        //     LOG_INF("Thermocouple: %.2f °C", tc);
        // }

        // err = sensor_channel_get(max_dev, SENSOR_CHAN_AMBIENT_TEMP, &ambient_val);
        // if (err) {
        //     LOG_ERR("sensor_channel_get AMBIENT failed: %d", err);
        // } else {
        //     double amb = sensor_value_to_double(&ambient_val);
        //     LOG_INF("Cold junction: %.2f °C", amb);
        // }

        /* Read fault status register */
        // sensor_attr_get(max_dev, SENSOR_CHAN_ALL, MAX31856_ATTR_FAULT_TYPE, &fault_status);
        // LOG_INF("Fault status: 0x%02x", fault_status.val1);

		k_msleep(SLEEP_TIME_MS);
	}
	return 0;
}
