/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author              Notes
 * 2023-10-25     Raman Gopalan       Initial version
 */

#ifndef __DRV_SOFT_I2C_H__
#define __DRV_SOFT_I2C_H__

#include <rtthread.h>
#include <rthw.h>
#include <rtdevice.h>

#ifdef BSP_BOARD_MIZAR32B
// All Mizar32 peripherals seem to work OK at max freq:
// All derived frequencies (sdram, USB, tmr, VTMT, SPI, UARTs, PWM, MMC)
// compensate correctly for the different PBA freq and
// tmr.getmaxdelay() is still just over 0.5 sec (by a hair!).
#define REQ_CPU_FREQ      66000000
#define REQ_PBA_FREQ      16500000
#else
#define REQ_CPU_FREQ      60000000
#define REQ_PBA_FREQ      15000000
#endif

// I2C speed
enum
{
	PLATFORM_I2C_SPEED_SLOW = 100000,
	PLATFORM_I2C_SPEED_FAST = 400000
};

// I2C direction
enum
{
	PLATFORM_I2C_DIRECTION_TRANSMITTER,
	PLATFORM_I2C_DIRECTION_RECEIVER
};

#endif /* #ifndef __DRV_SOFT_I2C_H__ */