#define DT_DRV_COMPAT custom_max31856

#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/init.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/types.h>
#include <zephyr/device.h>
#include "max31856_regs.h"
#include "max31856.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(customMAX31856, CONFIG_SENSOR_LOG_LEVEL);

struct max31856_config {
    struct spi_dt_spec spi;
    uint8_t thermocouple_type;
    uint8_t averaging;
    bool filter_50hz;
    bool use_external_cj;
    uint8_t operating_mode;
    bool fault_mask;
#ifdef CONFIG_MAX31856_FAULT_TRIGGER
    struct gpio_dt_spec fault_gpio;
    struct gpio_dt_spec drdy_gpio;
#endif
};

struct max31856_data {
    int32_t thermocouple_temp;
    int32_t cold_junction_temp;
    uint8_t fault;
#ifdef CONFIG_MAX31856_FAULT_TRIGGER
    struct gpio_callback fault_cb;
    struct k_work fault_work;
    const struct sensor_trigger *fault_trigger;
    sensor_trigger_handler_t fault_handler;
    const struct device *dev;
#endif
};

/* Fixed SPI write function with proper timing */
static int max31856_write_reg(const struct device *dev, uint8_t reg, uint8_t val)
{
    const struct max31856_config *config = dev->config;
    uint8_t tx_buf[2] = {reg | MAX31856_RD_WR_BIT, val};
    // LOG_INF("Writing reg 0x%02x: 0x%02x", reg | MAX31856_RD_WR_BIT, val);
    
    const struct spi_buf buf = {
        .buf = tx_buf,
        .len = sizeof(tx_buf)
    };
    const struct spi_buf_set tx_set = {
        .buffers = &buf,
        .count = 1
    };
    
    int ret = spi_write_dt(&config->spi, &tx_set);
    
    return ret;
}

/* Fixed SPI read function */
static int max31856_read_reg(const struct device *dev, uint8_t reg, uint8_t *val)
{
    const struct max31856_config *config = dev->config;
    uint8_t tx_buf = reg & ~MAX31856_RD_WR_BIT;
    uint8_t rx_buf[2] = {0};
    
    const struct spi_buf tx_bufs[] = {
        {
            .buf = &tx_buf,
            .len = 1
        }
    };
    const struct spi_buf_set tx = {
        .buffers = tx_bufs,
        .count = 1
    };
    
    const struct spi_buf rx_bufs[] = {
        {
            .buf = NULL,  /* Dummy byte */
            .len = 1
        },
        {
            .buf = rx_buf,
            .len = 1
        }
    };
    const struct spi_buf_set rx = {
        .buffers = rx_bufs,
        .count = 2
    };
    
    int ret = spi_transceive_dt(&config->spi, &tx, &rx);
    
    if (ret == 0) {
        *val = rx_buf[0];
        // LOG_DBG("Read reg 0x%02x: 0x%02x", reg, *val);
    }
    return ret;
}

/* Read multiple registers in one transaction */
static int max31856_read_regs(const struct device *dev, uint8_t start_reg, uint8_t *vals, size_t count)
{
    const struct max31856_config *config = dev->config;
    uint8_t tx_buf = start_reg & ~MAX31856_RD_WR_BIT;
    uint8_t rx_buf[count + 1];
    
    const struct spi_buf tx_bufs[] = {
        {
            .buf = &tx_buf,
            .len = 1
        }
    };
    const struct spi_buf_set tx = {
        .buffers = tx_bufs,
        .count = 1
    };
    
    const struct spi_buf rx_bufs[] = {
        {
            .buf = NULL,  /* Dummy byte */
            .len = 1
        },
        {
            .buf = rx_buf,
            .len = count + 1
        }
    };
    const struct spi_buf_set rx = {
        .buffers = rx_bufs,
        .count = 2
    };
    
    int ret = spi_transceive_dt(&config->spi, &tx, &rx);
    
    if (ret == 0) {
        memcpy(vals, &rx_buf[1], count);
    }
    return ret;
}

#ifdef CONFIG_MAX31856_FAULT_TRIGGER
/* Fault work handler */
static void max31856_fault_work_handler(struct k_work *work)
{
    struct max31856_data *data = CONTAINER_OF(work, struct max31856_data, fault_work);
    
    if (data->fault_handler && data->fault_trigger) {
        data->fault_handler(data->dev, data->fault_trigger);
    }
}

/* Fault GPIO interrupt handler */
static void max31856_fault_handler(const struct device *gpio_dev, 
                                  struct gpio_callback *cb, uint32_t pins)
{
    struct max31856_data *data = CONTAINER_OF(cb, struct max31856_data, fault_cb);
    
    k_work_submit(&data->fault_work);
}

static int max31856_trigger_set(const struct device *dev,
                               const struct sensor_trigger *trig,
                               sensor_trigger_handler_t handler)
{
    struct max31856_data *data = dev->data;
    const struct max31856_config *config = dev->config;
    int ret = 0;
    LOG_INF("Setting trigger type %d (%d)\n", trig->type, MAX31856_TRIGGER_FAULT);
    if (trig->type == MAX31856_TRIGGER_FAULT) {
        data->fault_trigger = trig;
        data->fault_handler = handler;
        
        if (config->fault_gpio.port) {
            LOG_INF("Configuring fault GPIO interrupt\n");
            ret = gpio_pin_interrupt_configure_dt(&config->fault_gpio, 
                                                 GPIO_INT_EDGE_TO_ACTIVE);
        }
    }
    
    /* To-do: Implement data ready trigger if needed
    else if (trig->type == SENSOR_TRIG_DATA_READY) {
        data->drdy_trigger = trig;
        data->drdy_handler = handler;
        
        if (config->drdy_gpio.port) {
            ret = gpio_pin_interrupt_configure_dt(&config->drdy_gpio, 
                                                 GPIO_INT_EDGE_TO_ACTIVE);
        }
    }*/ 
    else  {
        return -ENOTSUP;
    }
    
    return ret;
}
/* Initialize trigger support */
static int max31856_init_trigger(const struct device *dev)
{
    struct max31856_data *data = dev->data;
    const struct max31856_config *config = dev->config;
    int ret;
    
    data->dev = dev;
    LOG_INF("Initializing trigger support\n");
        
      /* Initialize fault GPIO */
    if (config->fault_gpio.port) {
        if (!gpio_is_ready_dt(&config->fault_gpio)) {
            LOG_ERR("Fault GPIO device is not ready");
            return -ENODEV;
        }
        
        ret = gpio_pin_configure_dt(&config->fault_gpio, GPIO_INPUT);
        if (ret) {
            LOG_ERR("Failed to configure fault GPIO");
            return ret;
        }
        
        gpio_init_callback(&data->fault_cb, max31856_fault_handler, 
                          BIT(config->fault_gpio.pin));
        
        ret = gpio_add_callback(config->fault_gpio.port, &data->fault_cb);
        if (ret) {
            LOG_ERR("Failed to add fault callback");
            return ret;
        }
        
        k_work_init(&data->fault_work, max31856_fault_work_handler);
    }
    
    return 0;
}
#endif /* CONFIG_MAX31856_FAULT_TRIGGER */
static int max31856_init(const struct device *dev)
{
    const struct max31856_config *config = dev->config;
    uint8_t cr0, cr1;
    int ret;

    if (!spi_is_ready_dt(&config->spi)) {
        LOG_ERR("SPI bus is not ready");
        return -ENODEV;
    }

    /* Read current CR0 */
    ret = max31856_read_reg(dev, MAX31856_CR0_REG, &cr0);
    if (ret) {
        LOG_ERR("Failed to read CR0");
        return ret;
    }

    if (config->operating_mode == MAX31856_MODE_AUTO) {
        cr0 |= MAX31856_CR0_AUTOCONVERT;
    }

    /* Configure CR0 */
    cr0 &= ~(MAX31856_CR0_OCFAULT_MASK | MAX31856_CR0_FILTER_50HZ);
    cr0 |= MAX31856_CR0_OCFAULT;  /* Enable open circuit fault detection */
    cr0 |= MAX31856_CR0_AUTOCONVERT;  /* Enable auto-convert */

    if (config->filter_50hz) {
        cr0 |= MAX31856_CR0_FILTER_50HZ;
    }
    // cr0 = 0x81;
    // LOG_INF("CR0 config to be written: 0x%02x", cr0);
    ret = max31856_write_reg(dev, MAX31856_CR0_REG, cr0);
    if (ret) {
        LOG_ERR("Failed to write CR0");
        return ret;
    }

    /* Configure CR1 */
    ret = max31856_read_reg(dev, MAX31856_CR1_REG, &cr1);
    if (ret) {
        LOG_ERR("Failed to read CR1");
        return ret;
    }

    cr1 &= ~(MAX31856_TC_TYPE_MASK | MAX31856_AVERAGING_MASK);
    cr1 |= (config->thermocouple_type & MAX31856_TC_TYPE_MASK);
    cr1 |= (config->averaging << MAX31856_AVERAGING_SHIFT);
    // cr1 = 0x33;  // Set to default for testing
    // LOG_INF("CR1 config to be written: 0x%02x", cr1);
    ret = max31856_write_reg(dev, MAX31856_CR1_REG, cr1);
    if (ret) {
        LOG_ERR("Failed to write CR1");
        return ret;
    }

    /* Config Fault Mask Regitser */
    if (config->fault_mask) {
        uint8_t mask = 0x00;
        ret = max31856_write_reg(dev, MAX31856_MASK_REG, mask);
        if (ret) {
            LOG_ERR("Failed to write MASK register");
            return ret;
        }            
        #ifdef CONFIG_MAX31856_FAULT_TRIGGER
        /* Initialize trigger support */
        ret = max31856_init_trigger(dev);
        if (ret) {
            LOG_ERR("Failed to initialize triggers");
            return ret;
        }
        #endif
    }

    return 0;
}

static int max31856_sample_fetch(const struct device *dev,
                                enum sensor_channel chan)
{
    struct max31856_data *data = dev->data;
    const struct max31856_config *config = dev->config;
    uint8_t buf[3];
    int ret;
    
    // LOG_INF("Fetching sample from MAX31856\n");
    if (config->operating_mode == MAX31856_MODE_1SHOT) {
        /* Trigger conversion */
        uint8_t cr0;
        // LOG_INF("Triggering 1-shot conversion\n");
        ret = max31856_read_reg(dev, MAX31856_CR0_REG, &cr0);
        if (ret) return ret;
        cr0 |= MAX31856_CR0_1SHOT;
        max31856_write_reg(dev, MAX31856_CR0_REG, cr0);
        k_sleep(K_MSEC(100)); /* Wait for conversion */
    }
    
    /* Read status */
    ret = max31856_read_reg(dev, MAX31856_SR_REG, &data->fault);
    if (ret) return ret;
    
    /* Read thermocouple temperature */
    ret = max31856_read_reg(dev, MAX31856_LTCBH_REG, &buf[0]);
    ret |= max31856_read_reg(dev, MAX31856_LTCBM_REG, &buf[1]);
    ret |= max31856_read_reg(dev, MAX31856_LTCBL_REG, &buf[2]);
    if (ret) return ret;
    
    data->thermocouple_temp = ((int32_t)buf[0] << 16) | 
                             ((int32_t)buf[1] << 8) | 
                             buf[2];
    data->thermocouple_temp >>= 5; /* Remove unused bits */
    
    /* Sign extend from 19 bits */
    if (data->thermocouple_temp & BIT(18)) {
        data->thermocouple_temp |= 0xFFF80000;
    }
    
    /* Read cold junction temperature */
    ret = max31856_read_reg(dev, MAX31856_CJTH_REG, &buf[0]);
    ret |= max31856_read_reg(dev, MAX31856_CJTL_REG, &buf[1]);
    if (ret) return ret;
    
    data->cold_junction_temp = ((int16_t)buf[0] << 8) | buf[1];
    data->cold_junction_temp >>= 2; /* Remove unused bits */
    
    /* Sign extend from 14 bits */
    if (data->cold_junction_temp & BIT(13)) {
        data->cold_junction_temp |= 0xC000;
    }
    
    return 0;
}

static int max31856_channel_get(const struct device *dev,
                               enum sensor_channel chan,
                               struct sensor_value *val)
{
    struct max31856_data *data = dev->data;
    int64_t microtemp;
    
    /* Check for faults */
    if (data->fault & (MAX31856_FAULT_OVUV | MAX31856_FAULT_OPEN)) {
        LOG_ERR("%s fault detected", (data->fault & MAX31856_FAULT_OVUV) ? "Over/Under Voltage" : "Open Circuit");
        return -EIO;
    }

    switch (chan) {
    case SENSOR_CHAN_AMBIENT_TEMP:  /* Cold junction temperature */
        /* Convert to microdegrees (0.015625 C per LSB) */
        microtemp = (int64_t)data->cold_junction_temp * INTERNAL_RESOLUTION;
        val->val1 = microtemp / 1000000;
        val->val2 = microtemp % 1000000;
        break;
    
    case SENSOR_CHAN_DIE_TEMP:  /* Thermocouple temperature */
        /* Convert to microdegrees (0.0078125 C per LSB) */
        microtemp = (int64_t)data->thermocouple_temp * THERMOCOUPLE_RESOLUTION;
        microtemp /= 10;  /* Adjust scaling */
        val->val1 = microtemp / 1000000;
        val->val2 = microtemp % 1000000;
        break;
    
    default:
        return -ENOTSUP;
    }

    return 0;
}

/* Helper function to check a specific fault */
static int max31856_check_fault(const struct device *dev, uint8_t fault_bit, 
                               struct sensor_value *val)
{
    struct max31856_data *data = dev->data;
    uint8_t status;
    int ret;
    
    /* Read status register */
    ret = max31856_read_reg(dev, MAX31856_SR_REG, &status);
    if (ret) {
        return ret;
    }
    
    /* Check if the specific fault bit is set */
    val->val1 = (status & fault_bit) ? 1 : 0;
    val->val2 = 0;
    
    return 0;
}


/* Helper function to read 16-bit (cold junction) threshold register */
static int max31856_read_threshold(const struct device *dev, uint8_t reg_msb, 
                                  int16_t *value)
{
    uint8_t buf[2];
    int ret;
    
    ret = max31856_read_reg(dev, reg_msb, &buf[0]);
    if (ret) return ret;
    
    ret = max31856_read_reg(dev, reg_msb + 1, &buf[1]);
    if (ret) return ret;
    
    *value = (buf[0] << 8) | buf[1];
    return 0;
}

/* Helper function to write 16-bit (cold junction) threshold register */
static int max31856_write_threshold(const struct device *dev, uint8_t reg_msb, 
                                   int16_t value)
{
    int ret;
    
    ret = max31856_write_reg(dev, reg_msb, (value >> 8) & 0xFF);
    if (ret) return ret;
    
    return max31856_write_reg(dev, reg_msb + 1, value & 0xFF);
}

/* Helper function to read 19-bit thermocouple threshold */
static int max31856_read_tc_threshold(const struct device *dev, uint8_t reg_msb, 
                                     int32_t *value)
{
    uint8_t buf[3];
    int ret;
    
    ret = max31856_read_reg(dev, reg_msb, &buf[0]);
    if (ret) return ret;
    
    ret = max31856_read_reg(dev, reg_msb + 1, &buf[1]);
    if (ret) return ret;
    
    ret = max31856_read_reg(dev, reg_msb + 2, &buf[2]);
    if (ret) return ret;
    
    /* Combine bytes and shift to get 19-bit value */
    *value = ((int32_t)buf[0] << 16) | ((int32_t)buf[1] << 8) | buf[2];
    *value >>= 5; /* Remove 5 unused bits */
    
    /* Sign extend from 19 bits */
    if (*value & BIT(18)) {
        *value |= 0xFFF80000;
    }
    
    return 0;
}

/* Helper function to write 19-bit thermocouple threshold */
static int max31856_write_tc_threshold(const struct device *dev, uint8_t reg_msb, 
                                      int32_t value)
{
    int ret;
    
    /* Convert to 24-bit register format (19 bits in MSBs) */
    uint32_t reg_value = (value & 0x7FFFF) << 5;
    
    ret = max31856_write_reg(dev, reg_msb, (reg_value >> 16) & 0xFF);
    if (ret) return ret;
    
    ret = max31856_write_reg(dev, reg_msb + 1, (reg_value >> 8) & 0xFF);
    if (ret) return ret;
    
    return max31856_write_reg(dev, reg_msb + 2, reg_value & 0xFF);
}

/* Function to set cold junction temperature */
static int max31856_write_cold_junction_temp(const struct device *dev, float temperature)
{
    uint8_t buf[2];
    int16_t reg_value;
    int ret;
    
    /* Convert temperature to register units (0.015625°C per LSB) */
    reg_value = (int16_t)(temperature / 0.015625);
    
    /* Prepare register values according to the specified format */
    /* CJTH: Sign bit + 7 bits of integer part */
    buf[0] = (reg_value >> 6) & 0xFF;
    
    /* CJTL: 6 bits of fractional part (2^-1 to 2^-6) in bits 7-2, bits 1-0 are 0 */
    /* The fractional part is bits 5-0 of the 14-bit value, shifted left by 2 */
    buf[1] = (reg_value & 0x3F) << 2;
    
    /* Write to cold junction temperature registers */
    ret = max31856_write_reg(dev, MAX31856_CJTH_REG, buf[0]);
    if (ret) {
        LOG_ERR("Failed to write CJTH register");
        return ret;
    }
    
    ret = max31856_write_reg(dev, MAX31856_CJTL_REG, buf[1]);
    if (ret) {
        LOG_ERR("Failed to write CJTL register");
        return ret;
    }
    
    LOG_DBG("Set cold junction temperature: %.2f°C (CJTH: 0x%02x, CJTL: 0x%02x)",
            temperature, buf[0], buf[1]);
    
    return 0;
}

static int max31856_attr_set(const struct device *dev,
                            enum sensor_channel chan,
                            enum sensor_attribute attr,
                            const struct sensor_value *val)
{
    int ret = 0;
    int64_t microtemp;
    int16_t cj_value;
    int32_t tc_value;
    int8_t offset_value;
    
    switch ((int)attr) {
    case MAX31856_ATTR_CJ_LOWER_THRESH:
        /* Convert to register units (0.015625°C per LSB) */
        microtemp = (int64_t)val->val1 * 1000000 + val->val2;
        cj_value = microtemp / INTERNAL_RESOLUTION;
        ret = max31856_write_threshold(dev, MAX31856_CJLF_REG, cj_value);
        break;
        
    case MAX31856_ATTR_CJ_UPPER_THRESH:
        /* Convert to register units (0.015625°C per LSB) */
        microtemp = (int64_t)val->val1 * 1000000 + val->val2;
        cj_value = microtemp / INTERNAL_RESOLUTION;
        ret = max31856_write_threshold(dev, MAX31856_CJHF_REG, cj_value);
        break;
        
    case MAX31856_ATTR_TC_LOWER_THRESH:
        /* Convert to register units (0.0078125°C per LSB) */
        microtemp = (int64_t)val->val1 * 1000000 + val->val2;
        tc_value = (microtemp * 10) / THERMOCOUPLE_RESOLUTION;
        ret = max31856_write_tc_threshold(dev, MAX31856_LTLFTH_REG, tc_value);
        break;
        
    case MAX31856_ATTR_TC_UPPER_THRESH:
        /* Convert to register units (0.0078125°C per LSB) */
        microtemp = (int64_t)val->val1 * 1000000 + val->val2;
        tc_value = (microtemp * 10) / THERMOCOUPLE_RESOLUTION;
        ret = max31856_write_tc_threshold(dev, MAX31856_LTHFTH_REG, tc_value);
        break;
        
    case MAX31856_ATTR_CJ_OFFSET:
        /* Convert to register units (0.015625°C per LSB) */
        microtemp = (int64_t)val->val1 * 1000000 + val->val2;
        offset_value = microtemp / INTERNAL_RESOLUTION;
        ret = max31856_write_reg(dev, MAX31856_CJTO_REG, (uint8_t)offset_value);
        break;        
        
    case MAX31856_ATTR_CJ_TEMP:
        /* Set cold junction temperature */
        float temperature;        
        uint8_t buf[2];
        /* Ensure we are in the correct mode */
       ret = max31856_read_reg(dev, MAX31856_CR0_REG, &buf[0]);
       ret |= max31856_write_reg(dev, MAX31856_CR0_REG, buf[0] | MAX31856_CR0_CJ); /* Enable cold junction mode */
       if (ret) {
            LOG_ERR("Failed to enable cold junction mode");
            return ret;
       }
        
        /* Convert sensor_value to float temperature */
        microtemp = (int64_t)val->val1 * 1000000 + val->val2;
        temperature = (float)microtemp / 1000000.0f;
        
        /* Write to cold junction temperature registers */
        ret = max31856_write_cold_junction_temp(dev, temperature);
        if (ret) {
            LOG_ERR("Failed to write cold junction temperature");
            return ret;
        }
        // buf[0] = (cj_value >> 8) & 0xFF;
        // buf[1] = cj_value & 0xFF; 
        // /* Write cold junction temperature */
        // ret = max31856_write_reg(dev, MAX31856_CJTH_REG, &buf[0]);
        // ret |= max31856_write_reg(dev, MAX31856_CJTL_REG, &buf[1]);
        break;
        
    case MAX31856_ATTR_FAULT_TYPE:
        LOG_ERR("Fault type is read-only, cannot set");
        /* Read-only attribute */
        ret = -EINVAL;
        break;
        
    default:
        ret = -ENOTSUP;
        break;
    }
    
    return ret;
}

static int max31856_attr_get(const struct device *dev,
                            enum sensor_channel chan,
                            enum sensor_attribute attr,
                            struct sensor_value *val)
{
    int ret = 0;
    int16_t cj_value;
    int32_t tc_value;
    uint8_t reg_value;
    
    switch ((int)attr) {
    case MAX31856_ATTR_CJ_LOWER_THRESH:
        ret = max31856_read_threshold(dev, MAX31856_CJLF_REG, &cj_value);
        if (ret == 0) {
            /* Convert to microdegrees (0.015625°C per LSB) */
            int64_t microtemp = (int64_t)cj_value * INTERNAL_RESOLUTION;
            val->val1 = microtemp / 1000000;
            val->val2 = microtemp % 1000000;
        }
        break;
        
    case MAX31856_ATTR_CJ_UPPER_THRESH:
        ret = max31856_read_threshold(dev, MAX31856_CJHF_REG, &cj_value);
        if (ret == 0) {
            /* Convert to microdegrees (0.015625°C per LSB) */
            int64_t microtemp = (int64_t)cj_value * INTERNAL_RESOLUTION;
            val->val1 = microtemp / 1000000;
            val->val2 = microtemp % 1000000;
        }
        break;
        
    case MAX31856_ATTR_TC_LOWER_THRESH:
        ret = max31856_read_tc_threshold(dev, MAX31856_LTLFTH_REG, &tc_value);
        if (ret == 0) {
            /* Convert to microdegrees (0.0078125°C per LSB) */
            int64_t microtemp = (int64_t)tc_value * THERMOCOUPLE_RESOLUTION / 10;
            val->val1 = microtemp / 1000000;
            val->val2 = microtemp % 1000000;
        }
        break;
        
    case MAX31856_ATTR_TC_UPPER_THRESH:
        ret = max31856_read_tc_threshold(dev, MAX31856_LTHFTH_REG, &tc_value);
        if (ret == 0) {
            /* Convert to microdegrees (0.0078125°C per LSB) */
            int64_t microtemp = (int64_t)tc_value * THERMOCOUPLE_RESOLUTION / 10;
            val->val1 = microtemp / 1000000;
            val->val2 = microtemp % 1000000;
        }
        break;
        
    case MAX31856_ATTR_CJ_OFFSET:
        ret = max31856_read_reg(dev, MAX31856_CJTO_REG, &reg_value);
        if (ret == 0) {
            /* Convert to microdegrees (0.015625°C per LSB) */
            int64_t microtemp = (int64_t)(int8_t)reg_value * INTERNAL_RESOLUTION;
            val->val1 = microtemp / 1000000;
            val->val2 = microtemp % 1000000;
        }
        break;
        
    case MAX31856_ATTR_FAULT_TYPE:
        ret = max31856_read_reg(dev, MAX31856_SR_REG, &reg_value);
        if (ret == 0) {
            val->val1 = reg_value;
            val->val2 = 0;
        }
        break;
        
    default:
        ret = -ENOTSUP;
        break;
    }
    
    return ret;

}


static const struct sensor_driver_api max31856_api = {
    .sample_fetch = max31856_sample_fetch,
    .channel_get = max31856_channel_get,
    .attr_set = max31856_attr_set,
    .attr_get = max31856_attr_get,
#ifdef CONFIG_MAX31856_FAULT_TRIGGER
    .trigger_set = max31856_trigger_set,
#endif
};
#ifdef CONFIG_MAX31856_FAULT_TRIGGER
#define MAX31856_FAULT_GPIO_INIT(n) \
    .fault_gpio = GPIO_DT_SPEC_INST_GET_OR(n, fault_gpios, {0}),
#else
#define MAX31856_FAULT_GPIO_INIT(n)
#endif
#define MAX31856_INIT(n) \
    static struct max31856_data max31856_data_##n; \
    static const struct max31856_config max31856_config_##n = { \
        .spi = SPI_DT_SPEC_INST_GET(n, \
                    SPI_OP_MODE_MASTER | SPI_MODE_CPHA | SPI_WORD_SET(8), 0), \
        .thermocouple_type = DT_INST_PROP(n, thermocouple_type), \
        .averaging = DT_INST_PROP(n, averaging), \
         MAX31856_FAULT_GPIO_INIT(n) \
        .filter_50hz = DT_INST_PROP(n, filter_50hz), \
        .use_external_cj = DT_INST_PROP(n, use_external_cj), \
        .operating_mode = DT_INST_PROP(n, operating_mode), \
        .fault_mask = DT_INST_PROP(n, fault_mask), \
    }; \
    SENSOR_DEVICE_DT_INST_DEFINE(n, \
                  max31856_init, \
                  NULL, \
                  &max31856_data_##n, \
                  &max31856_config_##n, \
                  POST_KERNEL, \
                  CONFIG_SENSOR_INIT_PRIORITY, \
                  &max31856_api);

DT_INST_FOREACH_STATUS_OKAY(MAX31856_INIT)