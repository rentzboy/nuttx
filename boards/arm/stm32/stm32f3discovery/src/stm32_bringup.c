/****************************************************************************
 * boards/arm/stm32/stm32f3discovery/src/stm32_bringup.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <syslog.h>
#include <nuttx/board.h>
#include <stm32.h>
#include "stm32f3discovery.h"
#include <sys/types.h>


//Pending revisar si puede ser útil
#ifdef CONFIG_INPUT_BUTTONS
#  include <nuttx/input/buttons.h>
#endif

//Pending revisar si puede ser útil
#ifdef CONFIG_USERLED
#  include <nuttx/leds/userled.h>
#endif

#ifdef CONFIG_I2C_DRIVER
#  include <stm32_i2c.h>
#endif

#ifdef CONFIG_SENSORS_LSM303DLHC_I2C
#  include "stm32_lsm303dlhc.h"
#endif

#ifdef CONFIG_USBMONITOR
#  include <nuttx/usb/usbmonitor.h>
#endif

#ifdef CONFIG_SENSORS_QENCODER
#include "board_qencoder.h"
#endif


/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#define HAVE_USBDEV     1
#define HAVE_USBMONITOR 1

/* Can't support USB device features if the STM32 USB peripheral is not
 * enabled.
 */

#ifndef CONFIG_STM32_USB
#  undef HAVE_USBDEV
#endif

/* Can't support USB device is USB device is not enabled */

#ifndef CONFIG_USBDEV
#  undef HAVE_USBDEV
#endif

/* Check if we should enable the USB monitor before starting NSH */

#ifndef CONFIG_USBMONITOR
#  undef HAVE_USBMONITOR
#endif

#ifndef HAVE_USBDEV
#  undef CONFIG_USBDEV_TRACE
#endif

#ifndef HAVE_USBHOST
#  undef CONFIG_USBHOST_TRACE
#endif

#if !defined(CONFIG_USBDEV_TRACE) && !defined(CONFIG_USBHOST_TRACE)
#  undef HAVE_USBMONITOR
#endif

#ifdef CONFIG_I2C_DRIVER
/**
 * @name stm32_i2c_register
 * @note This function definition would be better placed in stm32_i2c.c
 * but since there are many stm32_i2c.c files, it is "easy" to keep it here.
 * @brief Register an I2C bus
 *
 * This function is called from the board-specific logic to register
 * the I2C bus
 *
 * @param bus   The I2C bus number (1-4) to be registered.
 */
static void stm32_i2c_register(int bus)
{
  struct i2c_master_s *i2c;
  int ret;

  i2c = stm32_i2cbus_initialize(bus);
  if (i2c == NULL)
    {
      syslog(LOG_ERR, "ERROR: Failed to get I2C%d interface\n", bus);
    }
  else
    {
      ret = i2c_register(i2c, bus);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: Failed to register I2C%d driver: %d\n",
                 bus, ret);
          stm32_i2cbus_uninitialize(i2c);
        }
    }
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y :
 *     Called from board_late_initialize().
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y && CONFIG_BOARDCTL=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

int stm32_bringup(void)
{
  int ret = OK;

#ifdef HAVE_USBMONITOR
  /* Start the USB Monitor */

  ret = usbmonitor_start();
  if (ret != OK)
    {
      syslog(LOG_ERR, "ERROR: Failed to start USB monitor: %d\n", ret);
    }
#endif

#ifdef CONFIG_PWM
  /* Initialize PWM and register the PWM device. */

  ret = stm32_pwm_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_pwm_setup() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_SENSORS_QENCODER
  /* Initialize and register the qencoder driver */

  ret = board_qencoder_initialize(0, CONFIG_STM32F3DISCO_QETIMER);
  if (ret != OK)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to register the qencoder: %d\n",
             ret);
      return ret;
    }
#endif

//First: configurar el I2C driver
#ifdef CONFIG_I2C_DRIVER
  /* Register I2C drivers on behalf of the I2C tool */
  #ifdef CONFIG_STM32_I2C1
    stm32_i2c_register(1);
  #endif
  #ifdef CONFIG_STM32_I2C2
    stm32_i2c_register(2);
  #endif
  #ifdef CONFIG_STM32_I2C3
    stm32_i2c_register(3);
  #endif
#endif

//Second: configurar el sensor driver: creamos una instancia para cada I2Cx
#ifdef CONFIG_SENSORS_LSM303DLHC_I2C
  #ifdef CONFIG_STM32_I2C1
    ret = stm32_lsm303dlhc_initialize(1);
  #endif
  #ifdef CONFIG_STM32_I2C2
    ret = stm32_lsm303dlhc_initialize(2);
  #endif
  #ifdef CONFIG_STM32_I2C3
    ret = stm32_lsm303dlhc_initialize(3);
  #endif
  if (ret != OK)
  {
    syslog(LOG_ERR,
            "ERROR: Failed to register the LSM303DLHC sensor: %d\n",
            ret);
  }
  return ret;
#endif
}