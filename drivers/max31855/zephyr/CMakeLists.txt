# SPDX-License-Identifier: Apache-2.0

if(CONFIG_CUSTOM_MAX31855)
  zephyr_include_directories(.)

  zephyr_library()
  zephyr_library_sources(
    max31855.c
    )
endif()
