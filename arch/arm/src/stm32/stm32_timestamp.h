/****************************************************************************
 * arch/arm/src/stm32/stm32_timestamp.h
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

#ifndef __ARCH_ARM_SRC_STM32_STM32_TIMESTAMP_H
#define __ARCH_ARM_SRC_STM32_STM32_TIMESTAMP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <signal.h>
#include <sys/types.h>

#include <nuttx/timers/rtc.h>

#ifdef CONFIG_RTC_DRIVER

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Architecture-specific RTC IOCTL commands *********************************/

/* These commands expose the STM32 RTC time-stamp function (RM0316, section
 * 27.3.13 "Time-stamp function") through the common upper-half RTC driver.
 * They are dispatched by the default: branch of rtc_ioctl() (see
 * drivers/timers/rtc.c) into the lower-half ioctl() method, so
 * CONFIG_RTC_IOCTL must be enabled for them to reach the STM32 driver.
 *
 * The time-stamp hardware captures the calendar into RTC_TSTR/RTC_TSDR and
 * the synchronous prescaler counter into RTC_TSSSR when the configured edge
 * is seen on the RTC_TS pin (or, optionally, on a tamper event).
 */

/* STM32_RTC_SET_TIMESTAMP configures and enables the time-stamp capture.
 *
 * Argument: A read-only reference to a struct stm32_setts_s describing the
 *           active edge and how the caller wants to be notified.
 */

#define STM32_RTC_SET_TIMESTAMP     _RTCIOC(RTC_USER_IOCBASE + 0)

/* STM32_RTC_CANCEL_TIMESTAMP disables the time-stamp capture (clears TSE
 * and TSIE in RTC_CR) and cancels any pending notification.
 *
 * Argument: None (ignored).
 */

#define STM32_RTC_CANCEL_TIMESTAMP  _RTCIOC(RTC_USER_IOCBASE + 1)

/* STM32_RTC_RD_TIMESTAMP reads back the captured time-stamp.
 *
 * Argument: A writeable reference to a struct stm32_rdts_s to receive the
 *           captured date/time and status.
 *
 * NOTE: this operation is destructive.  RTC_TSTR/RTC_TSDR/RTC_TSSSR are
 * only valid while TSF is set and are cleared when TSF is reset (RM0316
 * sections 27.6.12 - 27.6.14), so the driver clears TSF after reading them
 * and the capture cannot be read a second time.
 */

#define STM32_RTC_RD_TIMESTAMP      _RTCIOC(RTC_USER_IOCBASE + 2)

/* Time-stamp active edge ***************************************************/

/* Value of the TSEDGE bit (bit 3) of RTC_CR (RM0316 section 27.6.3).  TSE
 * must be cleared before TSEDGE is changed, otherwise a spurious TSF may be
 * set.
 */

#define STM32_RTC_TS_RISING_EDGE    0  /* Rising edge on RTC_TS captures */
#define STM32_RTC_TS_FALLING_EDGE   1  /* Falling edge on RTC_TS captures */

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Structure used with the STM32_RTC_SET_TIMESTAMP IOCTL command.
 *
 * Unlike RTC_SET_ALARM, an architecture-specific IOCTL is forwarded
 * verbatim by the upper half, which performs no notification bookkeeping of
 * its own.  The pid/event pair below is therefore consumed by the STM32
 * lower half, which posts the notification itself when the time-stamp
 * interrupt fires.
 */

struct stm32_setts_s
{
  uint8_t edge;              /* Active edge on RTC_TS: STM32_RTC_TS_* */
  bool tamper;               /* True: also capture on a tamper event.
                              * Sets TAMPTS (bit 7) in RTC_TAFCR. */
  pid_t pid;                 /* Task to be notified (0 = caller) */

  /* Describes how the task is to be notified.  Set sigev_notify to
   * SIGEV_NONE to capture silently, leaving TSIE clear and polling the
   * capture with STM32_RTC_RD_TIMESTAMP; any other value enables TSIE.
   */

  struct sigevent event;
};

/* Structure used with the STM32_RTC_RD_TIMESTAMP IOCTL command. */

struct stm32_rdts_s
{
  bool active;               /* Time-stamp capture is enabled (TSE) */
  bool valid;                /* A capture was pending (TSF was set).  The
                              * remaining fields are meaningless if false. */
  bool overflow;             /* TSOVF: a later event was lost because this
                              * capture had not been read yet.  Only
                              * meaningful when 'valid' is true. */

  /* The captured calendar.  RTC_TSTR supplies tm_hour/tm_min/tm_sec and
   * RTC_TSDR supplies tm_mday/tm_mon/tm_wday.
   *
   * CAUTION: the hardware does NOT capture the year - RTC_TSDR has no year
   * field (RM0316 section 27.6.13).  tm_year is DERIVED by the driver from
   * the current calendar (RTC_DR), decremented by one when the captured
   * month is later than the current month, i.e. when the capture predates a
   * new-year rollover.  tm_yday and tm_isdst are not captured and are
   * zeroed.
   */

  struct rtc_time time;

  /* Raw RTC_TSSSR SS[15:0]: the value of the synchronous prescaler counter
   * at the instant of the capture.  Left as the raw register value on
   * purpose: converting it needs the PREDIV_S actually programmed in
   * RTC_PRER, which depends on the selected clock source.  The caller
   * converts with the RM0316 section 27.6.10 formula:
   *
   *   second fraction = (PREDIV_S - subsec) / (PREDIV_S + 1)
   *
   * Note that subsec can be larger than PREDIV_S after a shift operation
   * (RTC_SHIFTR).  In that case the fraction above goes negative and the
   * correct time is one second less than 'time' indicates.
   */

  uint32_t subsec;
};

#endif /* CONFIG_RTC_DRIVER */
#endif /* __ARCH_ARM_SRC_STM32_STM32_TIMESTAMP_H */
