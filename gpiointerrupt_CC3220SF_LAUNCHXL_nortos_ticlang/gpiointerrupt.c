/*
 * Copyright (c) 2015-2020, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== gpiointerrupt.c ========
 *  Thermostat application for CC3220SF LaunchPad
 *  - Reads temperature from TMP116 sensor via I2C
 *  - Adjusts set-point temperature using buttons
 *  - Controls an LED to simulate a heater
 *  - Sends status over UART in the format: <AA,BB,S,CCCC>
 */

/*
 *  ======== gpiointerrupt.c ========
 *  Thermostat application for CC3220SF LaunchPad
 */

/*
 *  ======== gpiointerrupt.c ========
 *  Thermostat application for CC3220SF LaunchPad
 */

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdio.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/UART2.h>
#include <ti/drivers/Timer.h>

/* Driver configuration */
#include "ti_drivers_config.h"

/* Defines */
#define DISPLAY(x) UART2_write(uart, output, x, NULL)

/* Global Variables */
volatile uint32_t timerCount = 0;
volatile uint32_t seconds = 0;
volatile uint8_t TimerFlag = 0;

int16_t currentTemp = 25; // Initial room temperature
int16_t setPoint = 25;    // Initial set-point temperature
uint8_t heaterOn = 0;     // Heater status: 0 - off, 1 - on

/* UART Global Variables */
UART2_Handle uart;
char output[64];

/* I2C Global Variables */
I2C_Handle i2c;
uint8_t txBuffer[1];
uint8_t rxBuffer[2];
I2C_Transaction i2cTransaction;

/* Timer Handle */
Timer_Handle timer0; // Declaration of timer0

/* Function Prototypes */
void initUART(void);
void initI2C(void);
void initTimer(void);
void gpioButtonFxn0(uint_least8_t index);
void gpioButtonFxn1(uint_least8_t index);
void timerCallback(Timer_Handle myHandle, int_fast16_t status);
int16_t readTemp(void);

/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    /* Call driver init functions */
    GPIO_init();
    initUART();
    initI2C();
    initTimer();



    /* Install Button callback */
    GPIO_setCallback(CONFIG_GPIO_BUTTON_0, gpioButtonFxn0);
    GPIO_enableInt(CONFIG_GPIO_BUTTON_0);

    GPIO_setCallback(CONFIG_GPIO_BUTTON_1, gpioButtonFxn1);
    GPIO_enableInt(CONFIG_GPIO_BUTTON_1);

    /* Main loop */
    while (1)
    {
        if (TimerFlag)
        {
            TimerFlag = 0;
            timerCount++;

            /* Read temperature every 500ms */
            if (timerCount % 3 == 0) // 200ms * 3 = 600ms (approximate 500ms)
            {
                currentTemp = readTemp();
            }

            /* Update heater status and UART output every 1 second */
            if (timerCount % 5 == 0) // 200ms * 5 = 1 second
            {
                seconds++; // Increment seconds every 1 second

                /* Control heater */
                if (currentTemp < setPoint)
                {
                    heaterOn = 0;
                    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
                }
                else
                {
                    heaterOn = 1;
                    GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
                }

                /* Send status over UART */
                int len = snprintf(output, sizeof(output), "<%02d,%02d,%d,%04d>\n\r",
                                   currentTemp, setPoint, heaterOn, seconds);
                DISPLAY(len);

                /* Reset timerCount */
                timerCount = 0;
            }
        }
    }
}

/*
 *  ======== initUART ========
 *  Initialize the UART driver
 */
void initUART(void)
{
    UART2_Params uartParams;

    UART2_Params_init(&uartParams);
    uartParams.baudRate = 115200;

    uart = UART2_open(CONFIG_UART2_0, &uartParams);

    if (uart == NULL)
    {
        /* UART_open() failed */
        while (1)
            ;
    }
}

/*
 *  ======== initI2C ========
 *  Initialize the I2C driver and detect the temperature sensor
 */
void initI2C(void)
{
    int8_t i;
    bool found = false;
    I2C_Params i2cParams;

    /* Initialize I2C */
    I2C_init();

    /* Configure I2C parameters */
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;

    /* Open I2C */
    i2c = I2C_open(CONFIG_I2C_0, &i2cParams);
    if (i2c == NULL)
    {
        /* I2C_open() failed */
        int len = snprintf(output, sizeof(output), "Error Initializing I2C\n\r");
        DISPLAY(len);
        while (1)
            ;
    }

    /* Detect the temperature sensor */
    const struct
    {
        uint8_t address;
        uint8_t resultReg;
        char *id;
    } sensors[3] = {
        {0x48, 0x00, "11X"},
        {0x49, 0x00, "116"},
        {0x41, 0x00, "006"}};

    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 0;

    for (i = 0; i < 3; ++i)
    {
        i2cTransaction.targetAddress = sensors[i].address; // Updated to targetAddress
        txBuffer[0] = sensors[i].resultReg;

        if (I2C_transfer(i2c, &i2cTransaction))
        {
            /* Sensor found */
            found = true;
            int len = snprintf(output, sizeof(output), "Detected TMP%s at I2C address 0x%x\n\r",
                               sensors[i].id, sensors[i].address);
            DISPLAY(len);
            break;
        }
    }

    if (!found)
    {
        int len = snprintf(output, sizeof(output), "Temperature sensor not found\n\r");
        DISPLAY(len);
        while (1)
            ;
    }
}

/*
 *  ======== initTimer ========
 *  Initialize the Timer driver
 */
void initTimer(void)
{
    Timer_Params params;

    /* Initialize Timer */
    Timer_init();

    /* Configure Timer parameters */
    Timer_Params_init(&params);
    params.period = 200000; // 200 ms
    params.periodUnits = Timer_PERIOD_US;
    params.timerMode = Timer_CONTINUOUS_CALLBACK;
    params.timerCallback = timerCallback;

    /* Open Timer */
    timer0 = Timer_open(CONFIG_TIMER_0, &params);
    if (timer0 == NULL)
    {
        /* Failed to initialize timer */
        while (1)
            ;
    }

    /* Start Timer */
    if (Timer_start(timer0) == Timer_STATUS_ERROR)
    {
        /* Failed to start timer */
        while (1)
            ;
    }
}

/*
 *  ======== timerCallback ========
 *  Timer interrupt callback function
 */
void timerCallback(Timer_Handle myHandle, int_fast16_t status)
{
    TimerFlag = 1;
}

/*
 *  ======== gpioButtonFxn0 ========
 *  Callback function for the button to increase set-point
 */
void gpioButtonFxn0(uint_least8_t index)
{
    setPoint++;
    if (setPoint > 99)
        setPoint = 99;
}

/*
 *  ======== gpioButtonFxn1 ========
 *  Callback function for the button to decrease set-point
 */
void gpioButtonFxn1(uint_least8_t index)
{
    setPoint--;
    if (setPoint < 0)
        setPoint = 0;
}

/*
 *  ======== readTemp ========
 *  Read temperature from the sensor
 */
int16_t readTemp(void)
{
    int16_t temperature = 0;

    i2cTransaction.readCount = 2;
    i2cTransaction.writeCount = 1;
    txBuffer[0] = 0x00; // Register address to read temperature

    if (I2C_transfer(i2c, &i2cTransaction))
    {
        /* Convert raw data to temperature */
        temperature = (rxBuffer[0] << 8) | rxBuffer[1];
        temperature *= 0.0078125; // Sensor-specific conversion

        /* Sign extension for negative temperatures */
        if (rxBuffer[0] & 0x80)
        {
            temperature |= 0xF000;
        }
    }
    else
    {
        int len = snprintf(output, sizeof(output), "Error reading temperature sensor\n\r");
        DISPLAY(len);
    }

    return temperature;
}
