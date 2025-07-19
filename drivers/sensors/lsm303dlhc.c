/****************************************************************************
 * drivers/sensors/lsm303dlhc.c
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
#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/kmalloc.h>
#include "lsm303dlhc.h"

#include <nuttx/include/sys/types.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define LSM303DLHC_ACC_ADDRESS 0x19
#define LSM303DLHC_MAG_ADDRESS 0x1E

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct lsm303dlhc_dev_s
{
  FAR struct i2c_master_s *i2c;
  uint8_t acc_addr;
  uint8_t mag_addr;
  uint16_t frequency;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int lsm303dlhc_open(FAR struct file *filep);
static int lsm303dlhc_close(FAR struct file *filep);
static ssize_t lsm303dlhc_read(FAR struct file *filep, FAR char *buffer,
                              size_t buflen);
static ssize_t lsm303dlhc_write(FAR struct file *filep,
                                FAR const char *buffer, size_t buflen);
static int lsm303dlhc_ioctl(FAR struct file *filep, int cmd,
                            unsigned long arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_lsm303dlhc_fops =
{
  lsm303dlhc_open,
  lsm303dlhc_close,
  lsm303dlhc_read,
  lsm303dlhc_write,
  lsm303dlhc_ioctl
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lsm303dlhc_open
 *
 * Description:
 *   This function is called whenever the LSM303DLHC device is opened.
 *
 ****************************************************************************/

static int lsm303dlhc_open(FAR struct file *filep)
{
  /* TODO: Implement */

  return OK;
}

/****************************************************************************
 * Name: lsm303dlhc_close
 *
 * Description:
 *   This function is called whenever the LSM303DLHC device is closed.
 *
 ****************************************************************************/

static int lsm303dlhc_close(FAR struct file *filep)
{
  /* TODO: Implement */

  return OK;
}

/****************************************************************************
 * Name: lsm303dlhc_read
 *
 * Description:
 *   This function is called whenever data is read from the LSM303DLHC
 *   device.
 *
 ****************************************************************************/

static ssize_t lsm303dlhc_read(FAR struct file *filep, FAR char *buffer,
                               size_t buflen)
{
  /* TODO: Implement */

  return 0;
}

/****************************************************************************
 * Name: lsm303dlhc_write
 *
 * Description:
 *   This function is called whenever data is written to the LSM303DLHC
 *   device.
 *
 ****************************************************************************/

static ssize_t lsm303dlhc_write(FAR struct file *filep,
                                FAR const char *buffer, size_t buflen)
{
  /* TODO: Implement */

  return 0;
}

/****************************************************************************
 * Name: lsm303dlhc_ioctl
 *
 * Description:
 *   This function is called for ioctl commands on the LSM303DLHC device.
 *
 ****************************************************************************/

static int lsm303dlhc_ioctl(FAR struct file *filep, int cmd,
                            unsigned long arg)
{
  /* TODO: Implement */

  return -ENOSYS;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: lsm303dlhc_register
 *
 * Description:
 *   Register the LSM303DLHC device as a character driver.
 *
 * Input Parameters:
 *   devpath - The full path to the driver to register. E.g., "/dev/accel0".
 *   i2c     - An instance of the I2C interface to use to communicate with
 *             the LSM303DLHC.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int lsm303dlhc_register(FAR const char *devpath,
                        FAR struct i2c_master_s *i2c)
{
  FAR struct lsm303dlhc_dev_s *priv;
  int ret;
  UNUSED(devpath);

  DEBUGASSERT(devpath != NULL);
  DEBUGASSERT(i2c != NULL);

  /* Allocate a new driver instance */   

  priv = kmm_zalloc(sizeof(struct lsm303dlhc_dev_s));
  if (priv == NULL)
    {
      snerr("ERROR: Failed to allocate a new driver instance\n");
      return -ENOMEM;
    }

  /* Initialize the device structure */

  priv->i2c       = i2c;
  priv->acc_addr  = LSM303DLHC_ACC_ADDRESS;
  priv->mag_addr  = LSM303DLHC_MAG_ADDRESS;
  priv->frequency = CONFIG_LSM303DLHC_I2C_FREQUENCY;

  /* Register the character driver */

  ret = register_driver(devpath, &g_lsm303dlhc_fops, 0666, priv);
  if (ret < 0)
    {
      snerr("ERROR: Failed to register driver: %d\n", ret);
      kmm_free(priv);
    }

  return ret;
}
