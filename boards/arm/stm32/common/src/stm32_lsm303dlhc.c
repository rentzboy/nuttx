/****************************************************************************
 * boards/arm/stm32/common/src/stm32_lsm303dlhc.c
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
#include <stdio.h>
#include <debug.h>
#include "lsm303dlhc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/
/**
 * stm32_lsm303dlhc_initialize - Initialize the LSM303DLHC sensor using the I2C
 * interface of the specified bus
 *
 * @param bus The bus number of the I2C interface to use
 *
 * @return 0 on success, -1 on failure
 *
 * This function allocates memory for the LSM303DLHC device structure, initializes
 * the device addresses and frequency, and registers the device driver with the
 * system using the provided I2C interface.
 */
 int stm32_lsm303dlhc_initialize(int bus)
{
  struct i2c_master_s *i2c;
  int ret;
    sninfo("Initializing lsm303dlhc!\n");

  //Get the I2C interface registered for the bus  
  i2c = stm32_i2c_get_instance(bus);
  if (i2c == NULL)
    {
      snerror("ERROR: Failed to get an instance of I2C%d interface\n", bus);
      return -1;
    }
  else
    {
      ret = lsm303dlhc_register("/dev/lsm303dlhc", i2c);
      if (ret < 0)
        {
          snerror("ERROR: Failed to register the LSM303DLHC sensor with the I2C%d interface\n",
                 bus);

        }
    }
    return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/
