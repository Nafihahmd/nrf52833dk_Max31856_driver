#define DT_DRV_COMPAT custom_max31856

#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/init.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>
#include <zephyr/types.h>
#include <zephyr/device.h>
#include "max31856_regs.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(customMAX31856, CONFIG_SENSOR_LOG_LEVEL);

struct max31856_config {
    struct spi_dt_spec spi;
    uint8_t thermocouple_type;
    uint8_t averaging;
    bool filter_50hz;
    bool use_external_cj;
    uint8_t operating_mode;
};

struct max31856_data {
    int32_t thermocouple_temp;
    int32_t cold_junction_temp;
    uint8_t fault;
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

    /* Disable auto-convert mode for configuration */
    cr0 &= ~MAX31856_CR0_AUTOCONVERT;
    ret = max31856_write_reg(dev, MAX31856_CR0_REG, cr0);
    if (ret) {
        LOG_ERR("Failed to write CR0");
        return ret;
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
        LOG_INF("Triggering 1-shot conversion\n");
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
        LOG_ERR("Sensor fault detected: 0x%02x", data->fault);
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

static int max31856_attr_set(const struct device *dev,
                            enum sensor_channel chan,
                            enum sensor_attribute attr,
                            const struct sensor_value *val)
{
    if (attr == SENSOR_ATTR_CALIBRATION && chan == SENSOR_CHAN_AMBIENT_TEMP) {
        /* Set cold-junction temperature */
        int64_t microtemp = (int64_t)val->val1 * 1000000 + val->val2;
        int16_t cj_value = microtemp / 15625; /* 0.015625°C per LSB */
        
        uint8_t buf[2] = {
            (cj_value >> 8) & 0xFF,
            cj_value & 0xFF
        };
        
        int ret = max31856_write_reg(dev, MAX31856_CJTH_REG, buf[0]);
        if (ret) return ret;
        return max31856_write_reg(dev, MAX31856_CJTL_REG, buf[1]);
    }
    return -ENOTSUP;
}

static const struct sensor_driver_api max31856_api = {
    .sample_fetch = max31856_sample_fetch,
    .channel_get = max31856_channel_get,
    .attr_set = max31856_attr_set,
};

#define MAX31856_INIT(n) \
    static struct max31856_data max31856_data_##n; \
    static const struct max31856_config max31856_config_##n = { \
        .spi = SPI_DT_SPEC_INST_GET(n, \
                    SPI_OP_MODE_MASTER | SPI_MODE_CPHA | SPI_WORD_SET(8), 0), \
        .thermocouple_type = DT_INST_PROP(n, thermocouple_type), \
        .averaging = DT_INST_PROP(n, averaging), \
        .filter_50hz = DT_INST_PROP(n, filter_50hz), \
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