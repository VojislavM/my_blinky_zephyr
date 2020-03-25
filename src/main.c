 
/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <nrf9160.h>
#include <zephyr.h>
#include <device.h>
#include <i2c.h>
#include <gpio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <drivers/gpio.h>
#include <sys/printk.h>

#define LED_PORT	DT_ALIAS_LED0_GPIOS_CONTROLLER
#define LED		DT_ALIAS_LED0_GPIOS_PIN

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS	1000

#ifdef CONFIG_SOC_NRF9160
#define I2C_DEV "I2C_3"
#else
#define I2C_DEV "I2C_1"
#endif

struct device * i2c_dev;
struct device * dev_led;
uint8_t WhoAmI = 0u;


void main(void)
{
	u32_t cnt = 0;
	uint8_t error = 0u;

	k_sleep(500);

	printk("Starting i2c scanner...\n");

	/*start and configure i2c*/
	i2c_dev = device_get_binding(I2C_DEV);
	if (!i2c_dev) 
	{
		printk("I2C: Device driver not found.\n");
		return;
	}
	i2c_configure(i2c_dev, I2C_SPEED_SET(I2C_SPEED_STANDARD));
	printk("Value of NRF_TWIM3_NS->PSEL.SCL: %ld \n",NRF_TWIM3_NS->PSEL.SCL);
	printk("Value of NRF_TWIM3_NS->PSEL.SDA: %ld \n",NRF_TWIM3_NS->PSEL.SDA);
	printk("Value of NRF_TWIM3_NS->FREQUENCY: %ld \n",NRF_TWIM3_NS->FREQUENCY);
	printk("26738688 -> 100k\n");

	/*start and configure led pin*/
	dev_led = device_get_binding(LED_PORT);
	/* Set LED pin as output */
	gpio_pin_configure(dev_led, LED, GPIO_DIR_OUT);

	/*search for i2c devices*/
	for (u8_t i = 4; i <= 0x77; i++) {
		struct i2c_msg msgs[1];
		u8_t dst = 1;

		/* Send the address to read from */
		msgs[0].buf = &dst;
		msgs[0].len = 1U;
		msgs[0].flags = I2C_MSG_WRITE | I2C_MSG_STOP;
		
		error = i2c_transfer(i2c_dev, &msgs[0], 1, i);
		if (error == 0) {
			printk("0x%2x FOUND\n", i);
		}
		else {
			//printk("error %d \n", error);
		}
		
		
	}

	/*blink led in loop*/
	while (1) {
		/* Set pin to HIGH/LOW every 1 second */
		gpio_pin_write(dev_led, LED, cnt % 2);
		cnt++;
		k_sleep(SLEEP_TIME_MS);
		printk("Hello World! %s\n", CONFIG_BOARD);
	}
}
