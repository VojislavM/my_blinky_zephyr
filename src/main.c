 
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
#include <drivers/sensor.h>
#include <sys/util.h>

#define LED_PORT	DT_ALIAS_LED0_GPIOS_CONTROLLER
#define LED		DT_ALIAS_LED0_GPIOS_PIN

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS	1000

// #ifdef CONFIG_SOC_NRF9160
 #define I2C_DEV "I2C_2"
// #else
// #define I2C_DEV "I2C_1"
// #endif

struct device * i2c_dev;
struct device * dev_led;
// uint8_t WhoAmI = 0u;

static void process_sample(struct device *dev)
{
	static unsigned int obs;
	struct sensor_value temp, hum;
	if (sensor_sample_fetch(dev) < 0) {
		printk("Sensor sample update error\n");
		return;
	}

	if (sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &temp) < 0) {
		printk("Cannot read HTS221 temperature channel\n");
		return;
	}

	if (sensor_channel_get(dev, SENSOR_CHAN_HUMIDITY, &hum) < 0) {
		printk("Cannot read HTS221 humidity channel\n");
		return;
	}

	++obs;
	printk("Observation:%u\n", obs);

	/* display temperature */
	printk("Temperature:%.1f C\n", sensor_value_to_double(&temp));

	/* display humidity */
	printk("Relative Humidity:%.1f%%\n",
	       sensor_value_to_double(&hum));
}

static void hts221_handler(struct device *dev,
			   struct sensor_trigger *trig)
{
	process_sample(dev);
}



void main(void)
{

	u32_t cnt = 0;
	uint8_t error = 0u;
	
	/*start and configure led pin*/
	dev_led = device_get_binding(LED_PORT);
	/* Set LED pin as output */
	gpio_pin_configure(dev_led, LED, GPIO_DIR_OUT);

	k_sleep(500);
	
/************************************TEST1************************************/
/******************************LED and UART test******************************/
	/*blink led in loop*/
	while (1) {
	 	/* Set pin to HIGH/LOW every 1 second */
	 	gpio_pin_write(dev_led, LED, cnt % 2);
	 	cnt++;
	 	k_sleep(SLEEP_TIME_MS);
	 	printk("Hello World! %s\n", CONFIG_BOARD);
	}
/*****************************************************************************/
/*****************************************************************************/

/************************************TEST2************************************/
/**********************************I2C SCAN***********************************/
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
/*****************************************************************************/
/*****************************************************************************/

/************************************TEST3************************************/
/**********************************I2C ACCEL**********************************/
	if (!init_accelerometer()) {
		return -1;
	}

	while (1) {
		printk("\033[1;1H");
		printk("\033[2J");

		double x_accel = 0;
		double y_accel = 0;
		double z_accel = 0;
		get_accelerometer_data(&x_accel, &y_accel, &z_accel);

		printk("Acceleration values:\n");
		printk("------------------------------------------------------------------------------------------------------\n");
		printf("X acceleration: %lf (m/s^2), Y acceleration: %lf (m/s^2), Z acceleration: %lf (m/s^2)\n", x_accel, y_accel, z_accel);
		printk("------------------------------------------------------------------------------------------------------");

		k_sleep(K_MSEC(500));
	}
/*****************************************************************************/
/*****************************************************************************/

	// return 0;

	/*******************************************/
	/*	struct device *dev = device_get_binding("HTS221");*/

/*	if (dev == NULL) {*/
/*		printk("Could not get HTS221 device\n");*/
/*		return;*/
/*	}*/

/*	if (IS_ENABLED(CONFIG_HTS221_TRIGGER)) {*/
/*		struct sensor_trigger trig = {*/
/*			.type = SENSOR_TRIG_DATA_READY,*/
/*			.chan = SENSOR_CHAN_ALL,*/
/*		};*/
/*		if (sensor_trigger_set(dev, &trig, hts221_handler) < 0) {*/
/*			printk("Cannot configure trigger\n");*/
/*			return;*/
/*		};*/
/*	}*/

/*	while (!IS_ENABLED(CONFIG_HTS221_TRIGGER)) {*/
/*		process_sample(dev);*/
/*		k_sleep(K_MSEC(2000));*/
/*	}*/
/*	k_sleep(K_FOREVER);*/

	// /*blink led in loop*/
	// while (1) {
	// 	/* Set pin to HIGH/LOW every 1 second */
	// 	gpio_pin_write(dev_led, LED, cnt % 2);
	// 	cnt++;
	// 	k_sleep(SLEEP_TIME_MS);
	// 	printk("Hello World! %s\n", CONFIG_BOARD);
	// }
}
