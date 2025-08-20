/*
 * Copyright (c) 2019 Nordic Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef __MAX31856_REGS_H__
#define __MAX31856_REGS_H__
/* MAX31856 Register Definitions */
#define MAX31856_CR0_REG           0x00
#define MAX31856_CR1_REG           0x01
#define MAX31856_MASK_REG          0x02
#define MAX31856_CJHF_REG          0x03
#define MAX31856_CJLF_REG          0x04
#define MAX31856_LTHFTH_REG        0x05
#define MAX31856_LTHFTL_REG        0x06
#define MAX31856_LTLFTH_REG        0x07
#define MAX31856_LTLFTL_REG        0x08
#define MAX31856_CJTO_REG          0x09
#define MAX31856_CJTH_REG          0x0A
#define MAX31856_CJTL_REG          0x0B
#define MAX31856_LTCBH_REG         0x0C
#define MAX31856_LTCBM_REG         0x0D
#define MAX31856_LTCBL_REG         0x0E
#define MAX31856_SR_REG            0x0F

/* CR0 Register Bits */
#define MAX31856_CR0_AUTOCONVERT   BIT(7)
#define MAX31856_CR0_1SHOT         BIT(6)
#define MAX31856_CR0_OCFAULT       BIT(4)
#define MAX31856_CR0_OCFAULT_MASK  (BIT(4) | BIT(5))
#define MAX31856_CR0_FILTER_50HZ   BIT(0)

/* CR1 Register Bits */
#define MAX31856_AVERAGING_MASK    (BIT(4) | BIT(5) | BIT(6))
#define MAX31856_AVERAGING_SHIFT   4
#define MAX31856_TC_TYPE_MASK      (BIT(0) | BIT(1) | BIT(2) | BIT(3))

/* SR Register Bits */
#define MAX31856_FAULT_OVUV        BIT(1)
#define MAX31856_FAULT_OPEN        BIT(0)

/* Thermocouple Types */
#define THERMOCOUPLE_TYPE_B        0
#define THERMOCOUPLE_TYPE_E        1
#define THERMOCOUPLE_TYPE_J        2
#define THERMOCOUPLE_TYPE_K        3
#define THERMOCOUPLE_TYPE_N        4
#define THERMOCOUPLE_TYPE_R        5
#define THERMOCOUPLE_TYPE_S        6
#define THERMOCOUPLE_TYPE_T        7

/* Resolution constants */
#define THERMOCOUPLE_RESOLUTION    78125   /* 0.0078125 * 10000000 */
#define INTERNAL_RESOLUTION        15625   /* 0.015625 * 1000000 */

/* Register read/write bit */
#define MAX31856_RD_WR_BIT         BIT(7)

/* Operation modes */
#define MAX31856_MODE_AUTO         0
#define MAX31856_MODE_1SHOT        1

#endif /* __MAX31856_REGS_H__ */
