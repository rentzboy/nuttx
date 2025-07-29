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
#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/kmalloc.h>
#include <nuttx/include/string.h>
#include <nuttx/sensors/lsm303dlhc.h>

#include <nuttx/include/sys/types.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

//TODO: PENDING MAGNETOMETER DRIVER
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

static int lsm303dlhc_acc_open(FAR struct file *filep);
static int lsm303dlhc_acc_close(FAR struct file *filep);
static ssize_t lsm303dlhc_read(FAR struct file *filep, FAR char *buffer,
                              size_t buflen);
static ssize_t lsm303dlhc_acc_write(FAR struct file *filep,
                                FAR const char *buffer, size_t buflen);
static int lsm303dlhc_acc_ioctl(FAR struct file *filep, int cmd,
                            unsigned long arg);
static int two_complement(int8_t *buffer, uint8_t buflen);
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
  struct lsm303dlhc_dev_s* dev;
	struct inode* node;
  ssize_t ret;
  char *str;

	node = filep->f_inode;
  dev = node->i_private;

  /* Setup the LSM303DLHC ACC */
  
  lsm303dlhc_write_register(dev, ACC_CTRL_REG1_A, ACC_SPEED_400_KHZ, "acc");
  up_mdelay(5);

  /* No need to return a fd (file descriptor) */
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
  struct lsm303dlhc_acc_data_s* data;
  struct lsm303dlhc_dev_s* dev;
	struct inode* node;
  ssize_t ret;
  int ret2;
  char *str;

  /* Sanity check */

    if (buflen < sizeof(struct lsm303dlhc_acc_data_s))
    {
      snerr("Expected buffer size is %zu\n", sizeof(struct lsm303dlhc_acc_data_s));
      return 0;
    }

	node = filep->f_inode;
  dev = node->i_private;

  str = strstr(node->i_name, "acc");

  ret = lsm303dlhc_i2c_read(dev, buffer, (uint8_t*)buflen, str == NULL ? "mag" : "acc");

  if (ret < 0)
  {
      snerr("ERROR: Failed to read from the LSM303DLHC\n");
  }

  /* Axis acceleration data. The value is expressed in two’s complement */

    ret2 = two_complement(buffer, sizeof(struct lsm303dlhc_acc_data_s));

    if (ret2 < 0)
    {
      snerr("ERROR: Failed to convert to two's complement\n");
    }
  return ret;
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

/* ---------------------- Auxiliary functions ---------------------- */
static int lsm303dlhc_i2c_read(struct lsm303dlhc_dev_s* dev, uint8_t *rbuffer, uint8_t length, const char *sensor)
{
  struct i2c_config_s i2c_config;
  uint8_t wbuffer[1];
  int ret;
  length = 6; //Acc registers, from 0x28 to 0x2D

  //TODO: Cast rbuffer to acc or mag struct

  if (strcmp(sensor, "acc") == 0)
  {
    dev->acc_addr = LSM303DLHC_ACC_ADDRESS;
  }
  else
  {
    dev->acc_addr = LSM303DLHC_MAG_ADDRESS;
  }
  
  i2c_config.frequency = CONFIG_LSM303DLHC_I2C_FREQUENCY;
  i2c_config.address = dev->acc_addr;
  i2c_config.addrlen = 7;

    /* In order to read multiple bytes, the MSB of the register address must be set to 1 in the address field. 
  In other words, SUB(7) must be equal to 1 while SUB(6-0) represents the address of the first register to be read. */

  /* //MSB must be 1, we are reading multiple bytes pag.20 sensor manual */
  wbuffer[0] = ACC_OUT_X_L_A | LSM303DLHC_AUTO_INCREMENT_BIT; 
  
  ret = i2c_writeread(dev->i2c, &i2c_config, wbuffer, 1, rbuffer, length);
  if (ret < 0)  
  {
      snerr("ERROR: Failed to read from the LSM303DLHC\n");
  }
  return ret;
}

static int lsm303dlhc_read_register(struct lsm303dlhc_dev_s* dev, const uint8_t regAddr, const char *sensor)
{
  struct i2c_config_s i2c_config;
  uint8_t regValue;
  int ret;

  if (strcmp(sensor, "acc") == 0)
  {
    dev->acc_addr = LSM303DLHC_ACC_ADDRESS;
  }
  else
  {
    dev->acc_addr = LSM303DLHC_MAG_ADDRESS;
  }

  i2c_config.frequency = CONFIG_LSM303DLHC_I2C_FREQUENCY;
  i2c_config.address = dev->acc_addr;
  i2c_config.addrlen = 7;

  ret = i2c_writeread(dev->i2c, &i2c_config, &regAddr, 1, &regValue, 1);
  if (ret < 0)  
  {
      snerr("ERROR: Failed to read from the LSM303DLHC\n");
  }
  return regValue;
}

static void lsm303dlhc_write_register(struct lsm303dlhc_dev_s* dev, const uint8_t regaddr, uint8_t regValue, const char *sensor)
{
  struct i2c_config_s i2c_config;
  uint8_t wbuffer[2];
  int ret;
  
  if (strcmp(sensor, "acc") == 0)
  {
    dev->acc_addr = LSM303DLHC_ACC_ADDRESS;
  }
  else
  {
    dev->acc_addr = LSM303DLHC_MAG_ADDRESS;
  }

  i2c_config.frequency = CONFIG_LSM303DLHC_I2C_FREQUENCY;
  i2c_config.address = dev->acc_addr;
  i2c_config.addrlen = 7;

  wbuffer[0] = regaddr;
  wbuffer[1] = regValue;

  ret = i2c_write(dev->i2c, &i2c_config, wbuffer, 2);
  if (ret < 0)  
  {
      snerr("ERROR: Failed to read from the LSM303DLHC\n");
  }
  return ret;
}

static int two_complement(int8_t *buffer, uint8_t buflen)
{
  /* No es necesaria, pero no la eliminamos */
  DEBUGASSERT(buflen > 6);

  struct lsm303dlhc_acc_data_s* acc_data;
  int8_t data[buflen];

  for (uint8_t i = 0; i < buflen; i++)
  {
    data[i] = *buffer++;
  }

  acc_data = kmm_malloc(sizeof(struct lsm303dlhc_acc_data_s));
  if (acc_data == NULL)
  {
    snerr("ERROR: Failed to allocate a new driver instance\n");
    return -ENOMEM;
  }
  
  acc_data->x_axis_acc = (int16_t)(data[0] | (data[1] << 8));
  acc_data->y_axis_acc = (int16_t)(data[2] | (data[3] << 8));
  acc_data->z_axis_acc = (int16_t)(data[4] | (data[5] << 8));

  buffer = acc_data;
  free(acc_data);

  return OK;
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

