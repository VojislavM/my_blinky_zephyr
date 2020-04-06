#include "lis2dw12_reg.h"

#include <stdio.h>
#include <zephyr.h>
#include <device.h>
#include <i2c.h>

#include "sensors_izo.h"

struct device * i2c_dev;

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
	reg = read_reg(ACCEL_DEV_ADDR, LIS2DW12_WHO_AM_I);
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

uint8_t accel_read_temp()
{
    return read_reg(ACCEL_DEV_ADDR, LIS2DW12_OUT_T);
}

/* 
 * @brief Writes value to register over I2C 
 * 
 * @param reg
 * @param val 
 *
 */
void write_reg(uint8_t address, uint8_t reg, uint8_t val)
{
	uint8_t temp[2];
	temp[0] = reg;
	temp[1] = val;
	//i2c_reg_write_byte(i2c_dev, ACCEL_DEV_ADDR, reg, val)
	//i2c_burst_write(i2c_dev, ACCEL_DEV_ADDR, reg, temp, 2)
	if (i2c_reg_write_byte(i2c_dev, address, reg, val) != 0) {
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
uint8_t read_reg(uint8_t address, uint8_t reg)
{
	uint8_t read_data;
	//i2c_burst_read(i2c_dev, ACCEL_DEV_ADDR, reg, &read_data, 1);
	//i2c_reg_read_byte(i2c_dev, ACCEL_DEV_ADDR, reg, &read_data);
	if (i2c_reg_read_byte(i2c_dev, address, reg, &read_data) != 0) {
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
struct accel_data accel_read_accel_values()
{
    // Demand data, after that we wait for DATA ready bit 
    write_reg(ACCEL_DEV_ADDR, LIS2DW12_CTRL3, 0x03);

    for(int i=0;i<100;i++)
    {
        if(read_reg(ACCEL_DEV_ADDR, LIS2DW12_STATUS) & LIS2DW12_DRDY_AI_BIT) 
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
    lsb = read_reg(ACCEL_DEV_ADDR, LIS2DW12_OUT_X_L);  
    msb = read_reg(ACCEL_DEV_ADDR, LIS2DW12_OUT_X_H);  
    data.x_axis = accel_twos_comp_to_signed_int(((msb << 8) | lsb)); 

    lsb = read_reg(ACCEL_DEV_ADDR, LIS2DW12_OUT_Y_L);  
    msb = read_reg(ACCEL_DEV_ADDR, LIS2DW12_OUT_Y_H);  
    data.y_axis = accel_twos_comp_to_signed_int(((msb << 8) | lsb)); 

    lsb = read_reg(ACCEL_DEV_ADDR, LIS2DW12_OUT_Z_L);  
    msb = read_reg(ACCEL_DEV_ADDR, LIS2DW12_OUT_Z_H);  
    data.z_axis = accel_twos_comp_to_signed_int(((msb << 8) | lsb)); 

    data.x_axis = accel_raw_to_mg_2g_range(data.x_axis); 
    data.y_axis = accel_raw_to_mg_2g_range(data.y_axis); 
    data.z_axis = accel_raw_to_mg_2g_range(data.z_axis); 

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
int16_t accel_twos_comp_to_signed_int(uint16_t x)
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
void accel_single_read_setup()
{
    // Soft-reset, reset all control registers
    write_reg(ACCEL_DEV_ADDR, LIS2DW12_CTRL2, 0x40);

    // Enable Block data unit which prevents continus updates of 
    // lower and upper registers. 
    write_reg(ACCEL_DEV_ADDR, LIS2DW12_CTRL2, 0x08);

    // Set Full-scale to +/-2g
    write_reg(ACCEL_DEV_ADDR, LIS2DW12_CTRL6, 0x00);

    // Enable single data conversion
    write_reg(ACCEL_DEV_ADDR, LIS2DW12_CTRL3, 0x02);

    // Data-ready routed to INT1
    // write_reg(LIS2DW12_CTRL4_INT1_PAD_CTRL, 0x01);

    // Start up the sensor
    //Set ODR 50Hz, Single data mode, 14 bit resolution 
    write_reg(ACCEL_DEV_ADDR, LIS2DW12_CTRL1, 0x49);
    
    // Settling time
    k_sleep(20);
    
    //Enable interrupts
    write_reg(ACCEL_DEV_ADDR, LIS2DW12_CTRL7, 0x20);
}

/*
 *  @brief Convert raw accel values to milli G's mapped to +/- 2g range
 *
 *  @param x
 *
 *  @return mapped value
 */
float accel_raw_to_mg_2g_range(int16_t x)
{
    return (x >> 2) * 0.244; 
}


/* 
 * @brief Setup wakeup and free fall detection 
 *
 */
void accel_wake_up_free_fall_setup(uint8_t wake_up_thr, uint8_t wake_up_dur,uint8_t free_fall)
{
	uint8_t reg_test = 0x00;
    // Soft-reset, reset all control registers
    write_reg(ACCEL_DEV_ADDR, LIS2DW12_CTRL2, 0x40);

    //Enable BDU
    write_reg(ACCEL_DEV_ADDR, LIS2DW12_CTRL2, 0x08);    

    //Set full scale +- 2g 
    write_reg(ACCEL_DEV_ADDR, LIS2DW12_CTRL6, 0x00);    

    //Enable wakeup and free fall detection interrupt
    write_reg(ACCEL_DEV_ADDR, LIS2DW12_CTRL4_INT1_PAD_CTRL, 0x30);    

    // Programmed for 30 mm fall at 100Hz ODR
    write_reg(ACCEL_DEV_ADDR, LIS2DW12_FREE_FALL, free_fall);
    write_reg(ACCEL_DEV_ADDR, LIS2DW12_WAKE_UP_THS, wake_up_thr);    
    write_reg(ACCEL_DEV_ADDR, LIS2DW12_WAKE_UP_DUR, wake_up_dur);    

    k_sleep(100); //Settling time

    //Start sensor with ODR 100Hz and in low-power mode 1 
    write_reg(ACCEL_DEV_ADDR, LIS2DW12_CTRL1, 0x10);
	//printf("reg_read LIS2DW12_CTRL1:  ");
	//reg_test = read_reg(LIS2DW12_CTRL1);    
	//printf(" %#04x\n\r", reg_test);
    //Enable interrupt function
    write_reg(ACCEL_DEV_ADDR, LIS2DW12_CTRL7, 0x20);    


}

/* 
 * @brief Setup wakeup and free fall detection 
 *
 */
void accel_setup_lowpower(void)
{
	uint8_t reg_test = 0x00;
    // Soft-reset, reset all control registers
    write_reg(ACCEL_DEV_ADDR, LIS2DW12_CTRL2, 0x40);

    //Enable BDU
    //write_reg(LIS2DW12_CTRL2, 0x08);    

    //Set full scale +- 2g 
    write_reg(ACCEL_DEV_ADDR, LIS2DW12_CTRL6, 0x00);    

    //Enable wakeup and free fall detection interrupt
    //write_reg(LIS2DW12_CTRL4_INT1_PAD_CTRL, 0x30);    

    // Programmed for 30 mm fall at 100Hz ODR
    //write_reg(LIS2DW12_FREE_FALL, free_fall);
    //write_reg(LIS2DW12_WAKE_UP_THS, wake_up_thr);    
    //write_reg(LIS2DW12_WAKE_UP_DUR, wake_up_dur);    

    k_sleep(100); //Settling time

    //Start sensor with ODR 100Hz and in low-power mode 1 
    write_reg(ACCEL_DEV_ADDR, LIS2DW12_CTRL1, 0x10);
	//printf("reg_read LIS2DW12_CTRL1:  ");
	//reg_test = read_reg(LIS2DW12_CTRL1);    
	//printf(" %#04x\n\r", reg_test);
    //Enable interrupt function
    //write_reg(LIS2DW12_CTRL7, 0x20);    
}

bool si7060_whoami(void)
{
	uint8_t reg = 0;
    // Comparing with expected value
	reg = read_reg(SI7060_DEV_ADDR, SI7060_REG_ID);
    uint8_t chipID = SI7060_HI_NIBBLE(reg);
    if(SI7060_CHIP_ID_VALUE == reg)
    {
		printf("I am si7060 %#08x\n", reg);
        return true;
    }
    else
    {
		printk("I am NOT si7060\n");
        return false;
    }
}

void si7060_prepare(void)
{
    uint8_t _ret;
    _ret = read_reg(SI7060_DEV_ADDR, SI7060_REG_MEASUREMENT);

    // Prepare Mesure
    write_reg(SI7060_DEV_ADDR, SI7060_REG_MEASUREMENT, 0x04);

    _ret = read_reg(SI7060_DEV_ADDR, SI7060_REG_MEASUREMENT);

    write_reg(SI7060_DEV_ADDR, SI7060_REG_POLARITY, 0x4E);
    write_reg(SI7060_DEV_ADDR, SI7060_REG_HYSTERESIS, 0x1C);
}

void si7060_sleep(void)
{
    // Prepare Mesure
    write_reg(SI7060_DEV_ADDR, SI7060_REG_MEASUREMENT, 0x01);
}

float si7060_read_temp(void)
{
    float _temp;
    uint8_t _Dspsigm;
    uint8_t _Dspsigl;
    uint8_t _ret;

    _ret = read_reg(SI7060_DEV_ADDR, SI7060_REG_DSPSIGM);
    _Dspsigm = (_ret&0x7F);

    _ret = read_reg(SI7060_DEV_ADDR, SI7060_REG_DSPSIGL);
    _Dspsigl = _ret;


    _temp = 55+ ((float)(256*_Dspsigm)+(float)(_Dspsigl-16384))/160;

    return _temp;
}
