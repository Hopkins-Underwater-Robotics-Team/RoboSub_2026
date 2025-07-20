#include "driver_mpu6500_basic.h"

uint8_t res;
uint32_t i;
uint32_t times;
float g[3];
float dps[3];
float degrees;
mpu6500_address_t addr;

/* init */
addr = MPU6500_ADDRESS_AD0_LOW;
res = mpu6500_basic_init(MPU6500_INTERFACE_IIC, addr);
if (res != 0)
{
    return 1;
}

...
    
/* read all */
times = 3;
for (i = 0; i < times; i++)
{
    /* read */
    if (mpu6500_basic_read(g, dps) != 0)
    {
        (void)mpu6500_basic_deinit();

        return 1;
    }

    ...
        
    if (mpu6500_basic_read_temperature(&degrees) != 0)
    {
        (void)mpu6500_basic_deinit();

        return 1;
    }

    ...
        
    /* output */
    mpu6500_interface_debug_print("mpu6500: %d/%d.\n", i + 1, times);
    mpu6500_interface_debug_print("mpu6500: acc x is %0.2fg.\n", g[0]);
    mpu6500_interface_debug_print("mpu6500: acc y is %0.2fg.\n", g[1]);
    mpu6500_interface_debug_print("mpu6500: acc z is %0.2fg.\n", g[2]);
    mpu6500_interface_debug_print("mpu6500: gyro x is %0.2fdps.\n", dps[0]);
    mpu6500_interface_debug_print("mpu6500: gyro y is %0.2fdps.\n", dps[1]);
    mpu6500_interface_debug_print("mpu6500: gyro z is %0.2fdps.\n", dps[2]);
    mpu6500_interface_debug_print("mpu6500: temperature %0.2fC.\n", degrees);

    ...
        
    /* delay 1000 ms */
    mpu6500_interface_delay_ms(1000);

    ...
}

...
    
/* deinit */
(void)mpu6500_basic_deinit();

return 0;