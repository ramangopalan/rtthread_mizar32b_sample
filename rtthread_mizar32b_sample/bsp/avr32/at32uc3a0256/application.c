/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author           Notes
 * 2010-03-30     Kyle             First version
 * 2023-10-25     Raman Gopalan    AT32UC3A: Access GPIO using RT's pin abstractions
 */

#include <rtthread.h>
#include <rtdevice.h>
#include "compiler.h"
#include "gpio.h"
//#include "usb-cdc.h"

/* Mizar32's built-in LED */
#define USER_LED_1    AVR32_PIN_PB29

extern void usb_device_task(void);
extern void UsbCdcFlush (void);
extern void avr32_usb_cdc_send( char data );

char thread_led1_stack[1024];
struct rt_thread thread_led1;
static void rt_thread_entry_led1(void* parameter)
{
    rt_pin_mode(USER_LED_1, PIN_MODE_OUTPUT);
	const char *buf = "SimpleMachines\r\n";
	int i;
    while (1)
    {
        rt_pin_write(USER_LED_1, 1);
        rt_thread_delay(RT_TICK_PER_SECOND / 2); /* sleep 0.5 second and switch to other thread */

        rt_pin_write(USER_LED_1, 0);
        rt_thread_delay(RT_TICK_PER_SECOND / 2);

		for (i = 0; i < 16; i++)
		  avr32_usb_cdc_send(buf[i]);
    }
}

char thread_usb_stack[2048];
struct rt_thread thread_usb;
static void rt_thread_usb(void* parameter)
{
	// rt_pin_mode(USER_LED_1, PIN_MODE_OUTPUT);
	while (1)
	{
		usb_device_task();
		UsbCdcFlush ();
		rt_thread_delay(RT_TICK_PER_SECOND / 10);
	}
}

int rt_application_init()
{
    /* create led1 thread */
    rt_thread_init(&thread_led1,
                   "led1",
                   rt_thread_entry_led1,
                   RT_NULL,
                   &thread_led1_stack[0],
                   sizeof(thread_led1_stack), 5, 5);
    rt_thread_startup(&thread_led1);
//#if (0)
    /* create USB thread */
    rt_thread_init(&thread_usb,
                   "usb",
                   rt_thread_usb,
                   RT_NULL,
                   &thread_usb_stack[0],
                   sizeof(thread_usb_stack), 5, 5);
    rt_thread_startup(&thread_usb);
//#endif
    return 0;
}
