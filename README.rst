MAX31856 Thermocouple-to-Digital Converter Driver Guide
=========================================================
Overview
********
This driver provides comprehensive support for the MAX31856 thermocouple-to-digital converter with advanced 
features including external cold junction compensation, fault detection, and trigger-based operation. 
The driver integrates with Zephyr's sensor subsystem and supports 
both internal and external temperature sensing.

Key Features
************
- Multiple Thermocouple Types: Support for B, E, J, K, N, R, S, T thermocouples
- Dual Temperature Reading: Thermocouple and cold junction temperatures
- External Cold Junction Compensation: Integration with external temperature sensors (e.g., SHT4x)
- Advanced Fault Detection: Open circuit, ~overvoltage/undervoltage~, threshold, and range faults
- Trigger Support: Data ready and fault triggers for interrupt-driven operation
- Configurable Operation: One-shot and auto-conversion modes
- Noise Filtering: 50Hz/60Hz noise rejection options

Device Tree Configuration
**************************
1. **Basic Configuration**

.. code-block:: DTS

   &spi0 {
      status = "okay";
      cs-gpios = <&gpio0 22 GPIO_ACTIVE_LOW>;

      max31856: max31856@0 {
         compatible = "maxim,max31856";
         reg = <0>;
         spi-max-frequency = <1000000>;
         thermocouple-type = <3>; /* K-type */
         averaging = <0>;         /* 1x averaging */
         filter-50hz;             /* Enable 50Hz filter */
         operating-mode = <0>;    /* Auto conversion mode */
      };
   };

2. **Advanced Configuration with External Sensor**

.. code-block:: DTS

   &i2c0 {
      status = "okay";
      sht4x: sht4x@44 {
         compatible = "sensirion,sht4x";
         reg = <0x44>;
      };
   };

   &spi0 {
      status = "okay";
      cs-gpios = <&gpio0 22 GPIO_ACTIVE_LOW>;

      max31856: max31856@0 {
         compatible = "maxim,max31856";
         reg = <0>;
         spi-max-frequency = <1000000>;
         thermocouple-type = <3>;   /* K-type */
         averaging = <3>;           /* 8x averaging */
         filter-50hz;
         operating-mode = <1>;      /* One-shot mode */
         use-external-cj;           /* Enable external CJ */
         cold-junction = <&sht4x>;  /* External sensor */
         fault-mask = <0x0F>;       /* Enable all fault detection */
         
         /* GPIOs for trigger support */
         drdy-gpios = <&gpio0 12 GPIO_ACTIVE_LOW>;
         fault-gpios = <&gpio0 13 GPIO_ACTIVE_LOW>;
      };
   };

3. **Kconfig Options**
Enable driver features through Kconfig:

.. code-block:: Kconfig

   # Enable MAX31856 driver
   CONFIG_MAX31856=y

   # Enable trigger support
   CONFIG_MAX31856_TRIGGER=y

   # Enable external cold junction support
   CONFIG_MAX31856_EXTERNAL_CJ=y

   # Enable specific fault types
   CONFIG_MAX31856_FAULT_DETECTION=y

API Usage
*********
1. **Basic Temperature Reading**

.. code-block:: C

   #include <zephyr/drivers/sensor.h>
   #include "max31856.h"

   const struct device *max_dev = DEVICE_DT_GET(DT_NODELABEL(max31856));

   void read_temperature(void)
   {
      struct sensor_value thermocouple, cold_junction;
      int err;
      
      /* Fetch sample from hardware */
      err = sensor_sample_fetch(max_dev);
      if (err) {
         printk("Sample fetch failed: %d\n", err);
         return;
      }
      
      /* Read temperatures */
      sensor_channel_get(max_dev, SENSOR_CHAN_TEMP, &thermocouple);
      sensor_channel_get(max_dev, SENSOR_CHAN_AMBIENT_TEMP, &cold_junction);
      
      printk("Thermocouple: %d.%06d °C\n", thermocouple.val1, thermocouple.val2);
      printk("Cold Junction: %d.%06d °C\n", cold_junction.val1, cold_junction.val2);
   }

2. **Setting Attributes**

.. code-block:: C

   /* Set Thermocouple thresholds */
   struct sensor_value fault_th = { .val1 = 100, .val2 = 0 }; // 100°C
   sensor_attr_set(max_dev, SENSOR_CHAN_ALL, MAX31856_ATTR_TC_UPPER_THRESH, &fault_th);
   
   fault_th.val1 = -20; // -20°C
   sensor_attr_set(max_dev, SENSOR_CHAN_ALL, MAX31856_ATTR_TC_LOWER_THRESH, &fault_th);
   
   /* Set averaging to 4x */
   struct sensor_value avg = { .val1 = 2, .val2 = 0 }; // 4x
   sensor_attr_set(max_dev, SENSOR_CHAN_ALL, MAX31856_ATTR_AVERAGING, &avg);

3. **Trigger Handling**

.. code-block:: C 
  
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
      const struct device *max_dev = DEVICE_DT_GET(DT_NODELABEL(max31856));
      int err;
      
      if (!device_is_ready(max_dev)) {
         printk("Device not ready\n");
         return -ENODEV;
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
   while (1) {
		
      /* ask driver to read from the hardware and store latest sample */
      err = sensor_sample_fetch(max_dev);
      if (err) {
         LOG_ERR("sensor_sample_fetch failed: %d", err);
         k_msleep(1000);
         continue;
      }
   }