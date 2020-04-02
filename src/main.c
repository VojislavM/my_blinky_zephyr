 
/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

//#include <nrf9160.h>
#include <stdio.h>
#include <zephyr.h>
#include <device.h>
//#include <i2c.h>
#include <gpio.h>
//#include <string.h>
//#include <stdlib.h>
//#include <math.h>
#include <drivers/sensor.h>
#include <drivers/gpio.h>
//#include <sys/printk.h>
//#include "accelerometer.h"

//#include <sys/util.h>

#define LED_PORT	DT_ALIAS_LED0_GPIOS_CONTROLLER
#define LED		DT_ALIAS_LED0_GPIOS_PIN

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS	1000

// #ifdef CONFIG_SOC_NRF9160
// #define I2C_DEV "I2C_2"
// #else
// #define I2C_DEV "I2C_1"
// #endif

//struct device * i2c_dev;
struct device * dev_led;
// uint8_t WhoAmI = 0u;

// static void process_sample(struct device *dev)
// {
// 	static unsigned int obs;
// 	struct sensor_value temp, hum;
// 	if (sensor_sample_fetch(dev) < 0) {
// 		printk("Sensor sample update error\n");
// 		return;
// 	}

// 	if (sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &temp) < 0) {
// 		printk("Cannot read HTS221 temperature channel\n");
// 		return;
// 	}

// 	if (sensor_channel_get(dev, SENSOR_CHAN_HUMIDITY, &hum) < 0) {
// 		printk("Cannot read HTS221 humidity channel\n");
// 		return;
// 	}

// 	++obs;
// 	printk("Observation:%u\n", obs);

// 	/* display temperature */
// 	printk("Temperature:%.1f C\n", sensor_value_to_double(&temp));

// 	/* display humidity */
// 	printk("Relative Humidity:%.1f%%\n",
// 	       sensor_value_to_double(&hum));
// }

// static void hts221_handler(struct device *dev,
// 			   struct sensor_trigger *trig)
// {
// 	process_sample(dev);
// }

#ifdef CONFIG_LIS2DW12_TRIGGER
static int lis2dw12_trig_cnt;

static void lis2dw12_trigger_handler(struct device *dev,
				    struct sensor_trigger *trig)
{
	sensor_sample_fetch_chan(dev, SENSOR_CHAN_ACCEL_XYZ);
	lis2dw12_trig_cnt++;
}
#endif

static void lis2dw12_config(struct device *lis2dw12)
{
	struct sensor_value odr_attr, fs_attr;

	/* set LIS2DW12 accel/gyro sampling frequency to 100 Hz */
	odr_attr.val1 = 100;
	odr_attr.val2 = 0;

	if (sensor_attr_set(lis2dw12, SENSOR_CHAN_ACCEL_XYZ,
			    SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr) < 0) {
		printk("Cannot set sampling frequency for LIS2DW12 accel\n");
		return;
	}

	sensor_g_to_ms2(16, &fs_attr);

	if (sensor_attr_set(lis2dw12, SENSOR_CHAN_ACCEL_XYZ,
			    SENSOR_ATTR_FULL_SCALE, &fs_attr) < 0) {
		printk("Cannot set sampling frequency for LIS2DW12 gyro\n");
		return;
	}

#ifdef CONFIG_LIS2DW12_TRIGGER
	struct sensor_trigger trig;

	trig.type = SENSOR_TRIG_DATA_READY;
	trig.chan = SENSOR_CHAN_ACCEL_XYZ;
	sensor_trigger_set(lis2dw12, &trig, lis2dw12_trigger_handler);
#endif
}

static void fetch_and_display(struct device *sensor)
{
	static unsigned int count;
	struct sensor_value accel_x, accel_y, accel_z;
	const char *overrun = "";
	int rc = sensor_sample_fetch(sensor);
	//int rc = sensor_sample_fetch(sensor);

	++count;
	if (rc == -EBADMSG) {
		/* Sample overrun.  Ignore in polled mode. */
		if (IS_ENABLED(CONFIG_LIS2DW12_TRIGGER)) {
			overrun = "[OVERRUN] ";
		}
		rc = 0;
	}
	if (rc == 0) {
		//printk("sensor_channel_get");
		rc = sensor_channel_get(sensor, SENSOR_CHAN_ACCEL_X, &accel_x);
		rc = sensor_channel_get(sensor, SENSOR_CHAN_ACCEL_Y, &accel_y);
		rc = sensor_channel_get(sensor, SENSOR_CHAN_ACCEL_Z, &accel_z);
	}
	if (rc < 0) {
		printf("ERROR: Update failed: %d\n", rc);
	} else {
		printf("#%u @ %u ms: %sx %f , y %f , z %f\n",
		       count, k_uptime_get_32(), overrun,
		       sensor_value_to_double(&accel_x),
		       sensor_value_to_double(&accel_y),
		       sensor_value_to_double(&accel_z));
	}
}

void main(void)
{

	struct device *lis2dw12 = device_get_binding(DT_INST_0_ST_LIS2DW12_LABEL);

	if (!lis2dw12) {
		printf("Could not get LIS2DW12 device\n");
		return;
	}

	lis2dw12_config(lis2dw12);
	// u32_t cnt = 0;
	// uint8_t error = 0u;

	// int ret;
	// struct sensor_value temp_value;
	// struct sensor_value odr_attr;
	
	// /*start and configure led pin*/
	// dev_led = device_get_binding(LED_PORT);
	// /* Set LED pin as output */
	// gpio_pin_configure(dev_led, LED, GPIO_DIR_OUT);

	// k_sleep(500);

	// struct device *sensor = device_get_binding("LIS2DW12-ACCEL");
	// if (sensor == NULL) {
	// 	printf("Could not get %s device\n",
	// 	       "LIS2DH12-ACCEL");
	// 	return;
	// }

	// /* set accel/gyro sampling frequency to 104 Hz */
	// odr_attr.val1 = 104;
	// odr_attr.val2 = 0;

	// if (sensor_attr_set(sensor, SENSOR_CHAN_ACCEL_XYZ,
	// 		    SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr) < 0) {
	// 	printk("Cannot set sampling frequency for accelerometer.\n");
	// 	return;
	// }

	// while (true) {
	// 	fetch_and_display(sensor);
	// 	k_sleep(K_MSEC(2000));
	// }
	
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
	// printk("Starting i2c scanner...\n");

	// /*start and configure i2c*/
	// i2c_dev = device_get_binding(I2C_DEV);
	// if (!i2c_dev) 
	// {
	// 	printk("I2C: Device driver not found.\n");
	// 	return;
	// }
	// i2c_configure(i2c_dev, I2C_SPEED_SET(I2C_SPEED_STANDARD));
	// printk("Value of NRF_TWIM3_NS->PSEL.SCL: %ld \n",NRF_TWIM3_NS->PSEL.SCL);
	// printk("Value of NRF_TWIM3_NS->PSEL.SDA: %ld \n",NRF_TWIM3_NS->PSEL.SDA);
	// printk("Value of NRF_TWIM3_NS->FREQUENCY: %ld \n",NRF_TWIM3_NS->FREQUENCY);
	// printk("26738688 -> 100k\n");

	// /*search for i2c devices*/
	// for (u8_t i = 4; i <= 0x77; i++) {
	// 	struct i2c_msg msgs[1];
	// 	u8_t dst = 1;

	// 	/* Send the address to read from */
	// 	msgs[0].buf = &dst;
	// 	msgs[0].len = 1U;
	// 	msgs[0].flags = I2C_MSG_WRITE | I2C_MSG_STOP;
		
	// 	error = i2c_transfer(i2c_dev, &msgs[0], 1, i);
	// 	if (error == 0) {
	// 		printk("0x%2x FOUND\n", i);
	// 	}
	// 	else {
	// 		//printk("error %d \n", error);
	// 	}
		
		
	// }
/*****************************************************************************/
/*****************************************************************************/

/************************************TEST3************************************/
/**********************************I2C ACCEL**********************************/
	// if (!init_accelerometer()) {
	// 	return -1;
	// }

	// while (1) {
	// 	//printk("\033[1;1H");
	// 	//printk("\033[2J");

	// 	double x_accel = 0;
	// 	double y_accel = 0;
	// 	double z_accel = 0;
	// 	get_accelerometer_data(&x_accel, &y_accel, &z_accel);

	// 	printk("Acceleration values:\n");
	// 	printk("------------------------------------------------------------------------------------------------------\n");
	// 	printf("X acceleration: %lf (m/s^2), Y acceleration: %lf (m/s^2), Z acceleration: %lf (m/s^2)\n", x_accel, y_accel, z_accel);
	// 	printk("------------------------------------------------------------------------------------------------------");

	// 	k_sleep(K_MSEC(1500));
	// }
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
