 
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

#include "lis2dw12_reg.h"

#define LED_PORT	DT_ALIAS_LED0_GPIOS_CONTROLLER
#define LED		DT_ALIAS_LED0_GPIOS_PIN
#define ACCEL_DEV_ADDR 0x19
#define LIS2DW12_DRDY_AI_BIT                 (0x01 << 0)
//#define LIS2DW12_WHO_AM_I                    (0x0FU)
#define LIS2DW12_CTRL7                       (0x3FU)

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS	1000

// #ifdef CONFIG_SOC_NRF9160
#define I2C_DEV "I2C_2"
// #else
// #define I2C_DEV "I2C_1"
// #endif

struct accel_data {
    float x_axis;
    float y_axis;
    float z_axis;
};

struct device * i2c_dev;
struct device * dev_led;
uint8_t WhoAmI = 0u;

uint8_t read_reg(uint8_t reg);
void write_reg(uint8_t reg, uint8_t val);
bool accel_whoami(void);
uint8_t read_temp(void);
struct accel_data read_accel_values(void);
int16_t twos_comp_to_signed_int(uint16_t x);
float raw_to_mg_2g_range(int16_t x);
void single_read_setup(void);
void wake_up_free_fall_setup(uint8_t wake_up_thr, uint8_t wake_up_dur,uint8_t free_fall);

uint8_t i2c_init(){
    i2c_dev = device_get_binding(I2C_DEV);
    if (!i2c_dev) {
		printk("I2C_2 error\n");
        return -1;
	} 
	else{
		//i2c_configure(i2c_dev, I2C_SPEED_SET(I2C_SPEED_STANDARD));
		printk("I2C_2 Init OK\n");
        return 0;
    }
}

void main(void)
{
	uint8_t error = 0u;
	
	/*start and configure led pin*/
	dev_led = device_get_binding(LED_PORT);
	/* Set LED pin as output */
	gpio_pin_configure(dev_led, LED, GPIO_DIR_OUT);

	/*
	*  Initialize mems driver interface
	*/
	// stmdev_ctx_t dev_ctx;
	// lis2dw12_reg_t int_route;
	// dev_ctx.write_reg = platform_write;
	// dev_ctx.read_reg = platform_read;
	// dev_ctx.handle = i2c_dev;

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
	//wake_up_free_fall_setup(0x10, 0x10, 0x10);
	uint8_t temperature_accel = read_temp();
	printk("temp: %d C\n\r", (int)temperature_accel);
	struct accel_data data_of_accel;
	uint8_t reg_test = 0;
	while(1){
		//Start sensor with ODR 100Hz and in low-power mode 1 
		//write_reg(LIS2DW12_CTRL1, 0x10);
		//reg_test = read_reg(LIS2DW12_CTRL1);
		//printf("reg_read LIS2DW12_CTRL1:  ");
		//reg_test = read_reg(LIS2DW12_CTRL1);    
		//printf(" %#04x\n\r", reg_test);

		data_of_accel = read_accel_values();
		printf("voja: %.6f, %.6f, %.6f\n\r", data_of_accel.x_axis, data_of_accel.y_axis, data_of_accel.z_axis);
		printf("voja1: %#08x\n\r", reg_test);
		k_sleep(1000); 
	}

/*****************************************************************************/
/*****************************************************************************/
}

/* 
 * @brief Function checks WHO_AM_I register and checks its value.
 *
 * @note should be called before all other functions to check if LIS is
 *       connected.
 *
 * @return true if output of read_reg is expected value 
 */
bool accel_whoami()
{
	uint8_t reg = 0;
    // Comparing with expected value
	reg = read_reg(LIS2DW12_WHO_AM_I);
    if(0x44 == reg)
    {
		printf("I am accel %#08x\n", reg);
        return true;
    }
    else
    {
		printk("I am NOT accel\n");
        return false;
    }
}

uint8_t read_temp()
{
    return read_reg(LIS2DW12_OUT_T);
}

/* 
 * @brief Writes value to register over I2C 
 * 
 * @param reg
 * @param val 
 *
 */
void write_reg(uint8_t reg, uint8_t val)
{
	uint8_t temp[2];
	temp[0] = reg;
	temp[1] = val;
	//i2c_reg_write_byte(i2c_dev, ACCEL_DEV_ADDR, reg, val)
	//i2c_burst_write(i2c_dev, ACCEL_DEV_ADDR, reg, temp, 2)
	if (i2c_reg_write_byte(i2c_dev, ACCEL_DEV_ADDR, reg, val) != 0) {
			printk("Error on i2c_write()\n");
        } else {
			//printk("i2c_write: no error\r\n");
	}
}

/* 
 * @brief Reads value from register over I2C 
 * 
 * @param reg
 *
 * @return content of reg
 */
uint8_t read_reg(uint8_t reg)
{
	uint8_t read_data;
	//i2c_burst_read(i2c_dev, ACCEL_DEV_ADDR, reg, &read_data, 1);
	//i2c_reg_read_byte(i2c_dev, ACCEL_DEV_ADDR, reg, &read_data);
	if (i2c_reg_read_byte(i2c_dev, ACCEL_DEV_ADDR, reg, &read_data) != 0) {
		printk("Error on i2c_read()\n");
	} else {
		//printk("i2c_read: no error\r\n");
	}
    return read_data;
}

/* 
 * @brief Setup wakeup detection 
 *
 */
struct accel_data read_accel_values()
{
    // Demand data, after that we wait for DATA ready bit 
    write_reg(LIS2DW12_CTRL3, 0x03);

    for(int i=0;i<100;i++)
    {
        if(read_reg(LIS2DW12_STATUS) & LIS2DW12_DRDY_AI_BIT) 
        {
            // Data is ready
            break;
        }
        k_sleep(10);
    }


    struct accel_data data;
    uint8_t msb;
    uint8_t lsb;

    // For each axis read lower and upper register, concentate them,
    // convert them into signed decimal notation and map them to
    // appropriate range.
    // By default register address should be incremented automaticaly,
    // this is controled in CTRL2 but for some reason doesn't look like
    // it does.
    lsb = read_reg(LIS2DW12_OUT_X_L);  
    msb = read_reg(LIS2DW12_OUT_X_H);  
    data.x_axis = twos_comp_to_signed_int(((msb << 8) | lsb)); 

    lsb = read_reg(LIS2DW12_OUT_Y_L);  
    msb = read_reg(LIS2DW12_OUT_Y_H);  
    data.y_axis = twos_comp_to_signed_int(((msb << 8) | lsb)); 

    lsb = read_reg(LIS2DW12_OUT_Z_L);  
    msb = read_reg(LIS2DW12_OUT_Z_H);  
    data.z_axis = twos_comp_to_signed_int(((msb << 8) | lsb)); 

    data.x_axis = raw_to_mg_2g_range(data.x_axis); 
    data.y_axis = raw_to_mg_2g_range(data.y_axis); 
    data.z_axis = raw_to_mg_2g_range(data.z_axis); 

    return data;
}

/* 
 *  @brief Convert from two's complement number into 
 *      signed decimal number 
 *
 *  @param x, number to be converted
 *
 *  @return converted number  
 */
int16_t twos_comp_to_signed_int(uint16_t x)
{
    if( x & (0x01 << 16))
    {
        // Number is negative, clear MSB bit
        x &= ~(0x01 << 16);
        x = -x;
        return x;
    }
    else
    {
        // Number is positive
        return x;
    }
}

/* 
 * @brief Setup reading of X, Y and Z axis 
 *
 */
void single_read_setup()
{
    // Soft-reset, reset all control registers
    write_reg(LIS2DW12_CTRL2, 0x40);

    // Enable Block data unit which prevents continus updates of 
    // lower and upper registers. 
    write_reg(LIS2DW12_CTRL2, 0x08);

    // Set Full-scale to +/-2g
    write_reg(LIS2DW12_CTRL6, 0x00);

    // Enable single data conversion
    write_reg(LIS2DW12_CTRL3, 0x02);

    // Data-ready routed to INT1
    // write_reg(LIS2DW12_CTRL4_INT1_PAD_CTRL, 0x01);

    // Start up the sensor
    //Set ODR 50Hz, Single data mode, 14 bit resolution 
    write_reg(LIS2DW12_CTRL1, 0x49);
    
    // Settling time
    k_sleep(20);
    
    //Enable interrupts
    write_reg(LIS2DW12_CTRL7, 0x20);
}

/*
 *  @brief Convert raw accel values to milli G's mapped to +/- 2g range
 *
 *  @param x
 *
 *  @return mapped value
 */
float raw_to_mg_2g_range(int16_t x)
{
    return (x >> 2) * 0.244; 
}


/* 
 * @brief Setup wakeup and free fall detection 
 *
 */
void wake_up_free_fall_setup(uint8_t wake_up_thr, uint8_t wake_up_dur,uint8_t free_fall)
{
	uint8_t reg_test = 0x00;
    // Soft-reset, reset all control registers
    write_reg(LIS2DW12_CTRL2, 0x40);

    //Enable BDU
    write_reg(LIS2DW12_CTRL2, 0x08);    

    //Set full scale +- 2g 
    write_reg(LIS2DW12_CTRL6, 0x00);    

    //Enable wakeup and free fall detection interrupt
    write_reg(LIS2DW12_CTRL4_INT1_PAD_CTRL, 0x30);    

    // Programmed for 30 mm fall at 100Hz ODR
    write_reg(LIS2DW12_FREE_FALL, free_fall);
    write_reg(LIS2DW12_WAKE_UP_THS, wake_up_thr);    
    write_reg(LIS2DW12_WAKE_UP_DUR, wake_up_dur);    

    k_sleep(100); //Settling time

    //Start sensor with ODR 100Hz and in low-power mode 1 
    write_reg(LIS2DW12_CTRL1, 0x10);
	printf("reg_read LIS2DW12_CTRL1:  ");
	reg_test = read_reg(LIS2DW12_CTRL1);    
	printf(" %#04x\n\r", reg_test);
    //Enable interrupt function
    write_reg(LIS2DW12_CTRL7, 0x20);    


}
// static int32_t platform_write(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
// {
// 	int32_t ret;
// 	ret = i2c_burst_write(handle, ACCEL_DEV_ADDR, reg, bufp, len);
// 	if(0 != ret)
// 	{
// 		printk("i2c_burst_write_error");
// 	}
// 	return ret;
// }
// static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
// {
// 	int32_t ret;
// 	printk("platform_read: i2c_burst_read");
// 	ret = i2c_burst_read(handle, ACCEL_DEV_ADDR, reg, bufp, len);
// 	if(0 != ret)
// 	{
// 		printk("i2c_burst_read_error");
// 	}
// 	return ret;
// }
