/****************************************************************************
 * arch/arm/src/stm32l4/stm32l4_pmstandby.c
 *
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: 2012, 2017 Gregory Nutt. All rights reserved.
 * SPDX-FileCopyrightText: 2015 Motorola Mobility LLC. All rights reserved.
 * SPDX-FileContributor: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>

#include "arm_internal.h"
#include "nvic.h"
#include "stm32l4_pwr.h"
#include "stm32l4_pm.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32l4_pmstandby
 *
 * Description:
 *   Enter STANDBY mode.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   On success, this function will not return (STANDBY mode can only be
 *   terminated with a reset event).  Otherwise, STANDBY mode did not occur
 *   and a negated errno value is returned to indicate the cause of the
 *   failure.
 *
 ****************************************************************************/

int stm32l4_pmstandby(void)
{
  uint32_t regval;

  /* Clear the Wake-Up Flags by setting the CWUFx bits in the power status
   * clear register
   */

  regval = PWR_SCR_CWUF1 | PWR_SCR_CWUF2 | PWR_SCR_CWUF3 |
           PWR_SCR_CWUF4 | PWR_SCR_CWUF5;
  putreg32(regval, STM32L4_PWR_SCR);

  /* Select Standby mode */

  regval  = getreg32(STM32L4_PWR_CR1);
  regval &= ~PWR_CR1_LPMS_MASK;
  regval |= PWR_CR1_LPMS_STANDBY;

  putreg32(regval, STM32L4_PWR_CR1);

  /* Set SLEEPDEEP bit of Cortex System Control Register */

  regval  = getreg32(NVIC_SYSCON);
  regval |= NVIC_SYSCON_SLEEPDEEP;
  putreg32(regval, NVIC_SYSCON);

  /* Sleep until the wakeup reset occurs */

  asm("wfi");
  return OK;  /* Won't get here */
}
