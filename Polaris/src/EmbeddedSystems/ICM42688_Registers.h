    #include <Arduino.h>
    // BANK 0
    const uint8_t ACCEL_OUT = 0x1F;
    const uint8_t GYRO_OUT = 0x25;
    const uint8_t TEMP_OUT = 0x1D;

    const uint8_t ACCEL_CONFIG0 = 0x50;
    const uint8_t ACCEL_FS_SEL_2G = 0x80; // TODO: 0x60 in datasheet
    const uint8_t ACCEL_FS_SEL_4G = 0x60; // TODO: 0x40 in datasheet
    const uint8_t ACCEL_FS_SEL_8G = 0x40; // TODO: 0x20 in datasheet
    const uint8_t ACCEL_FS_SEL_16G = 0x20; // TODO: 0x00 in datasheet
    const uint8_t ACCEL_ODR_32KHZ = 0x01;
    const uint8_t ACCEL_ODR_16KHZ = 0x02;
    const uint8_t ACCEL_ODR_8KHZ = 0x03;
    const uint8_t ACCEL_ODR_4KHZ = 0x04;
    const uint8_t ACCEL_ODR_2KHZ = 0x05;
    const uint8_t ACCEL_ODR_1KHZ = 0x06;
    const uint8_t ACCEL_ODR_200HZ = 0x07;
    const uint8_t ACCEL_ODR_100HZ = 0x08;
    const uint8_t ACCEL_ODR_50HZ = 0x09;
    const uint8_t ACCEL_ODR_25HZ = 0x0A;
    const uint8_t ACCEL_ODR_12_5HZ = 0x0B;
    const uint8_t ACCEL_ODR_6_25HZ = 0x0C;
    const uint8_t ACCEL_ODR_3_125HZ = 0x0D;
    const uint8_t ACCEL_ODR_1_5625HZ = 0x0E;
    const uint8_t ACCEL_ODR_500HZ = 0x0F;

    const uint8_t GYRO_CONFIG0 = 0x4F;
    const uint8_t GYRO_FS_SEL_15_625DPS = 0xE0;
    const uint8_t GYRO_FS_SEL_31_25DPS = 0xC0;
    const uint8_t GYRO_FS_SEL_62_5DPS = 0xA0;
    const uint8_t GYRO_FS_SEL_125DPS = 0x80;
    const uint8_t GYRO_FS_SEL_250DPS = 0x60;
    const uint8_t GYRO_FS_SEL_500DPS = 0x40;
    const uint8_t GYRO_FS_SEL_1000DPS = 0x20;
    const uint8_t GYRO_FS_SEL_2000DPS = 0x00;
    const uint8_t GYRO_ODR_32KHZ = 0x01;
    const uint8_t GYRO_ODR_16KHZ = 0x02;
    const uint8_t GYRO_ODR_8KHZ = 0x03;
    const uint8_t GYRO_ODR_4KHZ = 0x04;
    const uint8_t GYRO_ODR_2KHZ = 0x05;
    const uint8_t GYRO_ODR_1KHZ = 0x06;
    const uint8_t GYRO_ODR_200HZ = 0x07;
    const uint8_t GYRO_ODR_100HZ = 0x08;
    const uint8_t GYRO_ODR_50HZ = 0x09;
    const uint8_t GYRO_ODR_25HZ = 0x0A;
    const uint8_t GYRO_ODR_12_5HZ = 0x0B;
    const uint8_t GYRO_ODR_500HZ = 0x0F;

    const uint8_t INT_CONFIG = 0x14;
    const uint8_t INT_HOLD_ANY = 0x08;
    const uint8_t INT_PULSE_100us = 0x03;
    const uint8_t INT_SOURCE0 = 0x65;
    const uint8_t RESET_DONE_INT1_EN = 0x10;
    const uint8_t UI_DRDY_INT1_EN = 0x10;
    const uint8_t INT_STATUS = 0x2D;

    const uint8_t DEVICE_CONFIG = 0x11;
    const uint8_t PWR_RESET = 0x80;
    const uint8_t INTF_CONFIG1 = 0x4D;
    const uint8_t CLOCK_SEL_PLL = 0x01;
    const uint8_t PWR_MGMT0 = 0x4E;
    const uint8_t SEN_ENABLE = 0x0F;

    const uint8_t WHO_AM_I = 0x75;
    const uint8_t FIFO_EN = 0x23;
    const uint8_t FIFO_TEMP_EN = 0x04;
    const uint8_t FIFO_GYRO = 0x02;
    const uint8_t FIFO_ACCEL = 0x01;
    const uint8_t FIFO_COUNT = 0x2E;
    const uint8_t FIFO_DATA = 0x30;

    const uint8_t BANK_SEL = 0x76;
    const uint8_t BANK0 = 0x00;
    const uint8_t BANK1 = 0x01;
    const uint8_t BANK2 = 0x02;
    const uint8_t BANK3 = 0x03;
    const uint8_t BANK4 = 0x04;

    // BANK 1
    const uint8_t GYRO_CONFIG_STATIC2 = 0x0B;
    const uint8_t GYRO_NF_ENABLE = 0x00;
    const uint8_t GYRO_NF_DISABLE = 0x01;
    const uint8_t GYRO_AAF_ENABLE = 0x00;
    const uint8_t GYRO_AAF_DISABLE = 0x02;

    // BANK 2
    const uint8_t ACCEL_CONFIG_STATIC2 = 0x03;
    const uint8_t ACCEL_AAF_ENABLE = 0x00;
    const uint8_t ACCEL_AAF_DISABLE = 0x01;

    // Bit Masks
    const uint8_t Bit0 = 0x01;
    const uint8_t Bit1 = 0x02;
    const uint8_t Bit2 = 0x04;
    const uint8_t Bit3 = 0x08;
    const uint8_t Bit4 = 0x10;
    const uint8_t Bit5 = 0x20;
    const uint8_t Bit6 = 0x40;
    const uint8_t Bit7 = 0x80;