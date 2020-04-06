#include <stdio.h>

#define ACCEL_DEV_ADDR 0x19
#define LIS2DW12_DRDY_AI_BIT                 (0x01 << 0)
//#define LIS2DW12_WHO_AM_I                    (0x0FU)
#define LIS2DW12_CTRL7                       (0x3FU)

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

extern struct device * i2c_dev;

uint8_t i2c_init(void);
uint8_t read_reg(uint8_t reg);
void write_reg(uint8_t reg, uint8_t val);
bool accel_whoami(void);
uint8_t accel_read_temp(void);
struct accel_data accel_read_accel_values(void);
int16_t accel_twos_comp_to_signed_int(uint16_t x);
float accel_raw_to_mg_2g_range(int16_t x);
void accel_single_read_setup(void);
void accel_wake_up_free_fall_setup(uint8_t wake_up_thr, uint8_t wake_up_dur,uint8_t free_fall);
void accel_setup_lowpower(void);