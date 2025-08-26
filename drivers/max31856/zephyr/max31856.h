#ifndef MAX31856_H
#define MAX31856_H

#include <zephyr/drivers/sensor.h>

/* Custom sensor attributes for fault detection */
enum max31856_attribute {
    MAX31856_ATTR_FILTER_FREQ = SENSOR_ATTR_PRIV_START,
    MAX31856_ATTR_CJ_LOWER_THRESH,
    MAX31856_ATTR_CJ_UPPER_THRESH,
    MAX31856_ATTR_TC_LOWER_THRESH,
    MAX31856_ATTR_TC_UPPER_THRESH,
    MAX31856_ATTR_CJ_TEMP,
    MAX31856_ATTR_CJ_OFFSET,
    MAX31856_ATTR_FAULT_TYPE,
};

/* Custom trigger types */
enum max31856_trigger_type {
    MAX31856_TRIGGER_DATA_READY = SENSOR_TRIG_PRIV_START,
    MAX31856_TRIGGER_FAULT,
};


/* Fault Status Register (SR)x Register Bits */
#define MAX31856_FAULT_OPEN        BIT(0)
#define MAX31856_FAULT_OVUV        BIT(1)
#define MAX31856_FAULT_TCLOW       BIT(2)
#define MAX31856_FAULT_TCHIGH      BIT(3)
#define MAX31856_FAULT_CJLOW       BIT(4)
#define MAX31856_FAULT_CJHIGH      BIT(5)
#define MAX31856_FAULT_TCRANGE     BIT(6)
#define MAX31856_FAULT_CJRANGE     BIT(7)

#endif /* MAX31856_H */