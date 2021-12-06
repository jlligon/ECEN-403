/* =========================================================================
The MIT License (MIT)

Copyright 2017 Natanael Josue Rabello. All rights reserved.
Edited by James Ligon for use with ESP32C3

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to
deal in the Software without restriction, including without limitation the
rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
IN THE SOFTWARE.
 ========================================================================= */

#include <stdio.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "I2Cbus.hpp"

#define BH1790GLC_MANUFACTURER_ID 0x0F   /*!<  Manufacturer ID                   */
#define BH1790GLC_PART_ID         0x10   /*!<  Part ID                           */
#define BH1790GLC_RESET           0x40   /*!<  SWRESET                           */
#define BH1790GLC_MEAS_CONTROL1   0x41   /*!<  Measurement setting Control       */
#define BH1790GLC_MEAS_CONTROL2   0x42   /*!<  Measurement setting Control       */
#define BH1790GLC_MEAS_START      0x43   /*!<  Start Measurement                 */
#define BH1790GLC_DATAOUT_LEDOFF_LSB 0x54   /*!<  Measurement Data LSB (LED OFF)    */
#define BH1790GLC_DATAOUT_LEDOFF_MSB 0x55   /*!<  Measurement Data MSB (LED OFF)    */
#define BH1790GLC_DATAOUT_LEDON_LSB  0x56   /*!<  Measurement Data LSB (LED ON)     */
#define BH1790GLC_DATAOUT_LEDON_MSB  0x57    /*!<  Measurement Data MSB (LED ON)     */

#define MEAS_CONTROL1_RDY_OSC_BLOCK_ACTIVE 1U << 7U    //!<  OSC block clock to internal block
#define MEAS_CONTROL1_LED_LIGHTING_FREQ_64HZ 0 << 2U
#define MEAS_CONTROL1_RCYCLE_32HZ_MODE 0b10 << 0U   //its 64hz mode, dont question it
#define MEAS_CONTROL2_LED_EN_0 0b00 << 6U  //both leds pulsing
#define MEAS_CONTROL2_LED_ON_TIME_0_6_MS_MODE 0 << 5U  //0.6 ms mode
#define MEAS_CONTROL2_LED_CURRENT_1_MA_MODE 0xC << 0U

#define MEAS_START_MEAS_ST_MEASUREMENT_START 1

/*
esp_err_t BH1790GLC_StartMeasurement(void) {
    uint8_t cmd[4] = {0};
    uint32_t errorCode;

    //write:
    cmd[0] = BH1790GLC_MEAS_CONTROL1;
    cmd[1] = (MEAS_CONTROL1_RDY_OSC_BLOCK_ACTIVE | MEAS_CONTROL1_LED_LIGHTING_FREQ_64HZ | MEAS_CONTROL1_RCYCLE_32HZ_MODE);
    cmd[2] = (MEAS_CONTROL2_LED_EN_0 | MEAS_CONTROL2_LED_ON_TIME_0_6_MS_MODE | MEAS_CONTROL2_LED_CURRENT_1_MA_MODE);
    cmd[3] = MEAS_START_MEAS_ST_MEASUREMENT_START;
    printf("attempting to write start measurement\n");
    errorCode = myI2C.writeBytes(0x5B, 0x41, cmd[0], 4);
    return errorCode;
}
*/

uint32_t myState = 0;
void changeData(void) {
    myState = 1;
}

extern "C" void app_main() {
    printf(">I2Cbus Example \n");
    fflush(stdout);

    I2C_t& myI2C = i2c0;  // i2c0 and i2c1 are the default objects
    
    ESP_ERROR_CHECK( myI2C.begin(GPIO_NUM_8, GPIO_NUM_9, 40000));
    myI2C.setTimeout(2*1000);
    myI2C.scanner();
    //up to here works
    uint8_t cmd = 0;
    uint8_t output = 0;
    uint32_t errorCode;

    cmd = 0x0E;
    //part ID
    printf("part ID cmd: %d\n", cmd);
    errorCode = myI2C.writeByte(0x5B, 0x10, cmd);
    printf("errorCode for write: %d\n", errorCode);
    errorCode = myI2C.readByte(0x5B, 0x10, &output);

    printf("errorCode for read: %d\n", errorCode);
    printf("output part ID (should be 13): %d\n", output);
    //reset
    printf("trying to reset\n");
    uint8_t command[2] = {0};
    
    command[0] = 0x40;
    command[1] = (1U << 7U);

    errorCode = myI2C.writeBytes(0x5B, 0x40, 2, &command[0]);
    if (errorCode == 0) {
        printf("reset successful\n");
    }
    
    
    printf("attempting to start measure \n");
    uint8_t measCmd[4] = {0};

    //write:
    measCmd[0] = BH1790GLC_MEAS_CONTROL1;
    printf("measCmd[1]: %x\n", measCmd[1]);
    //measCmd[1] = (MEAS_CONTROL1_RDY_OSC_BLOCK_ACTIVE | MEAS_CONTROL1_LED_LIGHTING_FREQ_64HZ | MEAS_CONTROL1_RCYCLE_32HZ_MODE);
    measCmd[1] = 0b10000010;
    printf("measCmd[1]: %d\n", measCmd[1]);
    //measCmd[2] = (MEAS_CONTROL2_LED_EN_0 | MEAS_CONTROL2_LED_ON_TIME_0_6_MS_MODE | MEAS_CONTROL2_LED_CURRENT_1_MA_MODE);
    measCmd[2] = 0x0C;
    measCmd[3] = MEAS_START_MEAS_ST_MEASUREMENT_START;
    printf("attempting to write start measurement\n");
    errorCode = myI2C.writeBytes(0x5B, 0x41, sizeof(measCmd)/sizeof(measCmd[0]), &measCmd[0]);
    printf("startMeasurement errorCode: %d\n", errorCode);
    
    
    for (int i=0; i<100; ++i) {
        //this is wait 32ms to wait measurement time I think
        vTaskDelay(32 / portTICK_RATE_MS);
        printf("iteration: %d\n", i);

        uint8_t dataOutCmd[4] = {0};
        uint16_t dataOut_LED_OFF;
        uint16_t dataOut_LED_ON;

        dataOutCmd[0] = BH1790GLC_DATAOUT_LEDOFF_LSB;
        errorCode = myI2C.writeByte(0x5B, 0x54, dataOutCmd[0]);
        printf("dataout LSB errorcode: %d\n", errorCode);
        errorCode = myI2C.readBytes(0x5B, 0x54, 4, &dataOutCmd[0]);
        printf("dataout read errorCode: %d\n", errorCode);

        dataOut_LED_OFF = dataOutCmd[1];
        dataOut_LED_OFF <<= 8U;
        dataOut_LED_OFF |= dataOutCmd[0];
        dataOut_LED_ON = dataOutCmd[3];
        dataOut_LED_ON <<= 8U;
        dataOut_LED_ON |= dataOutCmd[2];

        printf("dataOut_LED_OFF: %d\n", dataOut_LED_OFF);
        printf("dataOut_LED_ON: %d\n", dataOut_LED_ON);
    }
    
    fflush(stdout);
    /*
    uint8_t buffer[6];
    while (1) {
        myI2C.readBytes(0x68, 0x6F, 6, buffer);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    
    myI2C.close();
    vTaskDelay(portMAX_DELAY);
    */
}
