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
 *  Header Files
 */
/* Driver configuration */
#include "ti_drivers_config.h"

#include <ti/drivers/I2C.h>
#include <ti/drivers/UART2.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/Timer.h>
#include <stdio.h>
//optional
#include <stdint.h>
#include <stddef.h>
#include <string.h>



/*
 *
 * Constants
 *
 */

#define LED_ON  1
#define LED_OFF 0

// Map LED pins
#define CONFIG_GPIO_LED_RED  9
#define CONFIG_GPIO_LED_YELLOW  10
#define CONFIG_GPIO_LED_GREEN  11


//define sensor structure
static const struct {
    uint8_t address;
    uint8_t resultReg;
    char *id;
} sensors[3] = {
    { 0x48, 0x0000, "11X" },
    { 0x49, 0x0000, "116" },
    { 0x41, 0x0001, "006" }
};

/*
 *
 * GLOBAL VARIABLES
 *
 */

volatile unsigned int taskCounter = 0;
volatile unsigned char TimerFlag = 0;
volatile int setPoint = 22; // Default set-point
volatile unsigned int seconds = 0;

// Global I2C handle
I2C_Handle i2c;
I2C_Params i2cParams;

// Global UART handle
UART2_Handle uart;
UART2_Params uartParams;
char output[100]; //character for storing the UART data

// Global timer handle
Timer_Handle timer;
Timer_Params timerParams;

/*
 *
 *  FUNCTIONS
 *
 */

/*
 *  ======== gpioButtonFxn0 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_0.
 *
 *  Note: GPIO interrupts are cleared prior to invoking callbacks.
 */
void gpioButtonFxn0(uint_least8_t index)
{
    /* Update Set Points */
    //++ Set Point
    setPoint++;
    printf("[GPIO] Button 0 Pressed: Set Point = %d\n", setPoint);
}

/*
 *  ======== gpioButtonFxn1 ========
 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_1.
 *  This may not be used for all boards.
 *
 *  Note: GPIO interrupts are cleared prior to invoking callbacks.
 */
void gpioButtonFxn1(uint_least8_t index)
{
    /* Update Set Points */
    // Set Points
    setPoint--;
    printf("[GPIO] Button 1 Pressed: Set Point = %d\n", setPoint);
}

// Function definition
void initI2C(void) {
    // Initialize the I2C driver
    I2C_init();

    // Configure I2C parameters
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz; // Fast mode

    // Open I2C driver
    i2c = I2C_open(CONFIG_I2C_0, &i2cParams);
    if (i2c == NULL) {
        printf("[ERROR] Error initializing I2C\n");
        while (1); // Halt on error
    } else {
        printf("[I2C] I2C Initialized\n");
    }
}

void initUART2(void) {
//    // Initialize the UART driver
//    UART2_init();

    // Configure UART parameters
    UART2_Params_init(&uartParams);
    uartParams.writeMode = UART2_Mode_BLOCKING;
    uartParams.readMode = UART2_Mode_BLOCKING;
    uartParams.readReturnMode = UART2_ReadReturnMode_PARTIAL;;
    uartParams.baudRate = 115200; // Standard baud rate

    // Open UART driver
    uart = UART2_open(CONFIG_UART2_0, &uartParams);
    if (uart == NULL) {
        printf("[ERROR] Error initializing UART\n");
        while (1); // Halt on error
    } else {
        printf("[UART] UART Initialized\n");
    }
}

size_t bytesWritten;

void UART2SendData(const char *data) {
    size_t bytesWritten;
    if (UART2_write(uart, data, strlen(data), &bytesWritten) != UART2_STATUS_SUCCESS) {
        printf("[ERROR] UART Write Error\n");
    } else {
        printf("[UART] Bytes written: %zu\n", bytesWritten);
    }
}


void initGPIO(void) {
    // Initialize the GPIO driver
    GPIO_init();

    // Configure button inputs with pull-up resistors and falling-edge interrupts
    GPIO_setConfig(CONFIG_GPIO_BUTTON_0, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);
    GPIO_setConfig(CONFIG_GPIO_BUTTON_1, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

    // Configure LED outputs
    GPIO_setConfig(CONFIG_GPIO_LED_RED, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_LED_GREEN, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_LED_YELLOW, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);

    // Register button callback functions
    GPIO_setCallback(CONFIG_GPIO_BUTTON_0, gpioButtonFxn0);
    GPIO_setCallback(CONFIG_GPIO_BUTTON_1, gpioButtonFxn1);

    // Enable interrupts for buttons
    GPIO_enableInt(CONFIG_GPIO_BUTTON_0);
    GPIO_enableInt(CONFIG_GPIO_BUTTON_1);

    printf("[GPIO] GPIO Initialized\n");
}

// Timer callback
void timerCallback(Timer_Handle myHandle, int_fast16_t status) {
    TimerFlag = 1;
    taskCounter++;
    if (taskCounter % 5 == 0) seconds++; // Increment seconds every 1s
    printf("[TIMER] Timer Callback Triggered: taskCounter = %d, seconds = %d\n", taskCounter, seconds);
}

void initTimer(void) {
    // Initialize the Timer driver
    Timer_init();

    // Configure the Timer parameters
    Timer_Params_init(&timerParams);
    timerParams.period = 200000; // 200ms period (in microseconds)
    timerParams.periodUnits = Timer_PERIOD_US;
    timerParams.timerMode = Timer_CONTINUOUS_CALLBACK;
    timerParams.timerCallback = timerCallback; // Link to the callback function

    // Open the Timer driver
    timer = Timer_open(CONFIG_TIMER_0, &timerParams);
    if (timer == NULL) {
        printf("Error initializing Timer\n");
        while (1); // Halt on error
    }

    // Start the Timer
    if (Timer_start(timer) == Timer_STATUS_ERROR) {
        printf("Error starting Timer\n");
        while (1); // Halt on error
    }

    printf("[TIMER] Timer Initialized\n");
}

// Buffers for I2C communication
uint8_t txBuffer[1]; // Transmit buffer (to specify the register to read)
uint8_t rxBuffer[2]; // Receive buffer (to store temperature data)

// I2C transaction structure
I2C_Transaction i2cTransaction;

int16_t readTemp(void) {
    int i;
    int16_t temperature = -1;
    uint8_t found = 0;

    for (i = 0; i < 3; i++) {
        i2cTransaction.targetAddress = sensors[i].address;
        txBuffer[0] = sensors[i].resultReg;
        i2cTransaction.writeBuf = txBuffer;
        i2cTransaction.writeCount = 1;
        i2cTransaction.readBuf = rxBuffer;
        i2cTransaction.readCount = 2;

        printf("[I2C] Checking sensor: %s at address 0x%X\n", sensors[i].id, sensors[i].address);

        if (I2C_transfer(i2c, &i2cTransaction)) {
            printf("[I2C] Sensor %s detected\n", sensors[i].id);
            found = 1;
            break;
        } else {
            printf("[I2C] Sensor %s not found\n", sensors[i].id);
        }
    }

    if (found) {
        temperature = (rxBuffer[0] << 8) | rxBuffer[1];
        temperature *= 0.0078125; // Conversion based on datasheet
        if (rxBuffer[0] & 0x80) { // Handle 2's complement for negative values
            temperature |= 0xF000;
        }
        printf("[I2C] Temperature: %d\n", temperature);
    } else {
        printf("[I2C] No sensor detected\n");
    }

    return temperature;
}


// Task Scheduler
void taskScheduler(void) {
    //read temp
    int16_t temp = readTemp();

    if (taskCounter % 1 == 0) {
        // 200ms Task: Check buttons
        if (GPIO_read(CONFIG_GPIO_BUTTON_0)) setPoint++;
        if (GPIO_read(CONFIG_GPIO_BUTTON_1)) setPoint--;
        printf("[TASK] Button State Checked: setPoint = %d\n", setPoint);
    }
    if (taskCounter % 2 == 0) {
        // 500ms Task: Read temperature
        char output[100];
        snprintf(output, sizeof(output), "<%02d,%02d,%d,%04d>\n", temp, setPoint, temp < setPoint, seconds);
        UART2SendData(output);
    }
    if (taskCounter % 5 == 0) {
        // 1st Task: Update LED and send data
        GPIO_write(CONFIG_GPIO_LED_GREEN, temp < setPoint); // Turn on LED if temp < set-point
        printf("[TASK] LED Updated: Green LED State = %d\n", setPoint);
    }
}

/*
 *  ======== mainThread ========
 *  Main execution thread for the smart thermostat system.
 */
void *mainThread(void *arg0) {
    //initialize i
    int i = 0; //Sensor iteration variables

    // Initialize peripherals
    initUART2();  // Initialize UART for communication
    initI2C();   // Initialize I2C for temperature sensor
    initGPIO();  // Initialize GPIO for LED and buttons
    initTimer(); // Initialize Timer for task scheduling

    //configure buttons
    GPIO_setConfig(CONFIG_GPIO_BUTTON_0, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);
    GPIO_setConfig(CONFIG_GPIO_BUTTON_1, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

    //configure LEDS
    GPIO_write(CONFIG_GPIO_LED_RED, LED_OFF);
    GPIO_write(CONFIG_GPIO_LED_GREEN, LED_OFF);
    GPIO_write(CONFIG_GPIO_LED_YELLOW, LED_ON);

    printf("[MAIN] Entering Main Loop\n");

    // Main execution loop
    while (1) {
        if (TimerFlag) {          // Check if the timer flag is set
            taskScheduler();      // Execute scheduled tasks
            TimerFlag = 0;        // Reset the timer flag

            printf(output, "is this %s\n", sensors[i].id); //LOGGING, verify sensor output
        }
        // Optional: Add any low-priority tasks or debug logging here.
    }

    // Return null as the thread exits (shouldn't reach here in an embedded system)
    return NULL;
}
