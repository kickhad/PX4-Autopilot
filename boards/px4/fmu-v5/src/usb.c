/****************************************************************************
 *
 *   Copyright (C) 2016 PX4 Development Team. All rights reserved.
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
 * 3. Neither the name PX4 nor the names of its contributors may be
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

/**
 * @file usb.c
 *
 * Board-specific USB functions.
 */

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <px4_platform_common/px4_config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <debug.h>

#include <nuttx/usb/usbdev.h>
#include <nuttx/usb/usbdev_trace.h>

#include <arm_arch.h>
#include <chip.h>
#include <stm32_gpio.h>
#include <stm32_otg.h>
#include "board_config.h"

#include <nuttx/wqueue.h>

extern int sercon_main(int c, char **argv);
extern int serdis_main(int c, char **argv);

extern int mavlink_main(int c, char **argv);

static char *mavlink_start_argv[5] = {"mavlink", "start", "-d", "/dev/ttyACM0", NULL};
static char *mavlink_stop_argv[5] = {"mavlink", "stop", "-d", "/dev/ttyACM0", NULL};

static void mavlink_usb_start(void *arg)
{
	if (sercon_main(0, NULL) == EXIT_SUCCESS) {
		syslog(LOG_INFO, "MAVLINK USB START\n");

		// TODO:
		// ret = exec_builtin(cmd, (FAR char * const *)argv, redirfile, oflags);
		//char *mavlink_argv[5] = {"mavlink", "start", "-d", "/dev/ttyACM0", NULL};
		mavlink_main(4, (char **)mavlink_start_argv);
	}
}

static void mavlink_usb_stop(void *arg)
{
	syslog(LOG_INFO, "MAVLINK USB STOP\n");

	// TODO:
	// ret = exec_builtin(cmd, (FAR char * const *)argv, redirfile, oflags);
	//char *mavlink_argv[5] = {"mavlink", "stop", "-d", "/dev/ttyACM0", NULL};
	mavlink_main(4, (char **)mavlink_stop_argv);

	serdis_main(0, NULL);
}

static struct work_s usb_work;

static int usb_otgfs_vbus_event(int irq, void *context, void *arg)
{
	int value = stm32_gpioread(GPIO_OTGFS_VBUS);

	if (value == 1) {
		// USB connected
		work_queue(HPWORK, &usb_work, mavlink_usb_start, NULL, USEC2TICK(10000));

	} else if (value == 0) {
		// USB disconnected
		work_queue(HPWORK, &usb_work, mavlink_usb_stop, NULL, USEC2TICK(10000));
	}

	return 0;
}

/************************************************************************************
 * Name: stm32_usbinitialize
 *
 * Description:
 *   Called to setup USB-related GPIO pins for the PX4FMU board.
 *
 ************************************************************************************/

__EXPORT void stm32_usbinitialize(void)
{
	/* The OTG FS has an internal soft pull-up */

	/* Configure the OTG FS VBUS sensing GPIO, Power On, and Overcurrent GPIOs */

#ifdef CONFIG_STM32F7_OTGFS
	stm32_configgpio(GPIO_OTGFS_VBUS);
	stm32_gpiosetevent(GPIO_OTGFS_VBUS, true, true, true, usb_otgfs_vbus_event, NULL);
#endif
}

/************************************************************************************
 * Name:  stm32_usbsuspend
 *
 * Description:
 *   Board logic must provide the stm32_usbsuspend logic if the USBDEV driver is
 *   used.  This function is called whenever the USB enters or leaves suspend mode.
 *   This is an opportunity for the board logic to shutdown clocks, power, etc.
 *   while the USB is suspended.
 *
 ************************************************************************************/

__EXPORT void stm32_usbsuspend(FAR struct usbdev_s *dev, bool resume)
{
	uinfo("resume: %d\n", resume);
}
