 
/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

//#include <nrf9160.h>
#include <stdio.h>
#include <zephyr.h>
#include <device.h>
#include <i2c.h>
#include <gpio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <drivers/sensor.h>
#include <drivers/gpio.h>
#include <sys/printk.h>

#include <sys/util.h>

#include "sensors_izo.h"

#define LED_PORT	DT_ALIAS_LED0_GPIOS_CONTROLLER
#define LED		DT_ALIAS_LED0_GPIOS_PIN


/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS	1000

struct device * dev_led;
uint8_t WhoAmI = 0u;

void main(void)
{
	uint8_t error = 0u;
	
	/*start and configure led pin*/
	dev_led = device_get_binding(LED_PORT);
	/* Set LED pin as output */
	gpio_pin_configure(dev_led, LED, GPIO_DIR_OUT);
	k_sleep(500); 
	
/************************************TEST1************************************/
/******************************LED and UART test******************************/
	/*blink led in loop*/
//	while (1) {
//	 	/* Set pin to HIGH/LOW every 1 second */
//	 	gpio_pin_write(dev_led, LED, cnt % 2);
//	 	cnt++;
//	 	k_sleep(SLEEP_TIME_MS);
//	 	printk("Hello World! %s\n", CONFIG_BOARD);
//	}
/*****************************************************************************/
/*****************************************************************************/


/************************************TEST2************************************/
/**********************************I2C SCAN***********************************/
	printk("Starting i2c scanner...\n");

	/*start and configure i2c*/
	i2c_init();
	printk("Value of NRF_TWIM3_NS->PSEL.SCL: %ld \n",NRF_TWIM2_NS->PSEL.SCL);
	printk("Value of NRF_TWIM3_NS->PSEL.SDA: %ld \n",NRF_TWIM2_NS->PSEL.SDA);
	printk("Value of NRF_TWIM3_NS->FREQUENCY: %ld \n",NRF_TWIM2_NS->FREQUENCY);
	printk("26738688 -> 100k\n");

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

	//accel init
	accel_whoami();
	//single_read_setup();
	accel_wake_up_free_fall_setup(0x10, 0x10, 0x10);
	struct accel_data data_of_accel;
    float data_of_si7060_temp = 0;
	uint8_t reg_test = 0;
	while(1){
        //accel read
		data_of_accel = accel_read_accel_values();
		printf("accel: %.6f, %.6f, %.6f\n\r", data_of_accel.x_axis, data_of_accel.y_axis, data_of_accel.z_axis);

        //temp read
        si7060_prepare();
        data_of_si7060_temp = si7060_read_temp();
        si7060_sleep();
        printf("temperature: %.4f degC\n\r", data_of_si7060_temp);

		k_sleep(1000); 
	}

/*****************************************************************************/
/*****************************************************************************/
}
