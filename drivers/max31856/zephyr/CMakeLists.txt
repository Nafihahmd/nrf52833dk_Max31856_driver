# SPDX-License-Identifier: Apache-2.0

if(CONFIG_CUSTOM_MAX31856)
  zephyr_include_directories(.)

  zephyr_library()
  zephyr_library_sources(
    max31856.c
    )
endif()
