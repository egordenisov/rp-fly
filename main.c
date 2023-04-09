#include "pico/stdlib.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ssd1306-driver.h"

#include "hardware/i2c.h"

ssd1306_ctx_t ssd1306_ctx;
uint8_t data[SSD1306_BUFSIZE];

uint32_t i2c_disp_write (uint8_t addr, const uint8_t* data, uint32_t size) {
    
    return i2c_write_blocking (i2c_default, addr, data, size, false);
}

void draw_x_y_z (float x, float y, float z, float temp, uint32_t ok, uint32_t count) {
    memset(*(ssd1306_ctx.buf), 0, SSD1306_BUFSIZE);


    char sbuf[16];
    sprintf(sbuf, "X=%.1f", x);
    ssd1306_string(&ssd1306_ctx, sbuf, 0, 0);

    sprintf(sbuf, "Y=%.1f", y);
    ssd1306_string(&ssd1306_ctx, sbuf,  0, 12);

    sprintf(sbuf, "Z=%.1f", z);
    ssd1306_string(&ssd1306_ctx, sbuf,  0, 23);

    sprintf(sbuf, "T=%.1f", temp);
    ssd1306_string(&ssd1306_ctx, sbuf, 64, 0);

    itoa(ok, sbuf, 10);
    ssd1306_string(&ssd1306_ctx, sbuf, 64, 12);

    itoa(count, sbuf, 10);
    ssd1306_string(&ssd1306_ctx, sbuf, 64, 23);

    ssd1306_render(&ssd1306_ctx);
}

static void mpu6050_reset() {
    // Two byte reset. First byte register, second byte data
    // There are a load more options to set up the device in different ways that could be added here
    uint8_t buf[] = {0x6B, 0x80};
    i2c_write_blocking(i2c_default, 0x68, buf, 2, false);
}

static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
    // For this particular device, we send the device the register we want to read
    // first, then subsequently read from the device. The register is auto incrementing
    // so we don't need to keep sending the register we want, just the first.

    uint8_t buffer[6];

    // Start reading acceleration registers from register 0x3B for 6 bytes
    uint8_t val = 0x3B;
    i2c_write_blocking(i2c_default, 0x68, &val, 1, true); // true to keep master control of bus
    i2c_read_blocking(i2c_default, 0x68, buffer, 6, false);

    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    // Now gyro data from reg 0x43 for 6 bytes
    // The register is auto incrementing on each read
    val = 0x43;
    i2c_write_blocking(i2c_default, 0x68, &val, 1, true);
    i2c_read_blocking(i2c_default, 0x68, buffer, 6, false);  // False - finished with bus

    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);;
    }

    // Now temperature from reg 0x41 for 2 bytes
    // The register is auto incrementing on each read
    val = 0x41;
    i2c_write_blocking(i2c_default, 0x68, &val, 1, true);
    i2c_read_blocking(i2c_default, 0x68, buffer, 2, false);  // False - finished with bus

    *temp = buffer[0] << 8 | buffer[1];
}

int main (){

    stdio_init_all();

    i2c_init(i2c_default, 100 * 1000);

    gpio_set_function(16, GPIO_FUNC_I2C);
    gpio_set_function(17, GPIO_FUNC_I2C);

    gpio_pull_up(16);
    gpio_pull_up(17);

    ssd1306_init_driver(&ssd1306_ctx, &data, i2c_disp_write);
    ssd1306_init(&ssd1306_ctx);

    mpu6050_reset();  
    sleep_ms(250);  
    {
        uint8_t val[] = {0x6B, 0x00};
        i2c_write_blocking(i2c_default, 0x68, val, 2, true);
    }

    int16_t acceleration[3], gyro[3], temp;
    acceleration[0] = 11111;
    acceleration[1] = 11111;
    acceleration[2] = 11111;
    temp = 222;

    uint32_t count = 0;

    while (1) {
        mpu6050_read_raw(acceleration, gyro, &temp);

        uint8_t val = 0x75;
        uint8_t buffer[3] = {0};
        i2c_write_blocking(i2c_default, 0x68, &val, 1, true);
        i2c_read_blocking(i2c_default, 0x68, buffer, 1, false); 

        count++;

        uint32_t ac_div = 8*2048 / 9.8;

        sleep_ms(5);
        draw_x_y_z((float)acceleration[0] / ac_div, (float)acceleration[1] / ac_div, (float)acceleration[2] / ac_div, ((float)temp / 340.0) + 36.53, buffer[0], count);
        sleep_ms(100);
    }
}