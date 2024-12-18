/*

 *  ======== gpiointerrupt.c ========

 */


#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <stdbool.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/Timer.h>
#include <ti/drivers/UART2.h>

/* Driver configuration */
#include "ti_drivers_config.h"

#define DISPLAY(x) UART2_write(uart, &output, x, NULL);

#define MAX_PATTERN_LENGTH 11

#define CONFIG_GPIO_RED_LED  9
#define CONFIG_GPIO_YELLOW_LED  10


// UART Global Variables

char output[64];
int bytesToSend;

// I2C Global Variables

static const struct {
    uint8_t address;
    uint8_t resultReg;
    char *id;
    } sensors[3] = {
    { 0x48, 0x0000, "11X" },
    { 0x49, 0x0000, "116" },
    { 0x41, 0x0001, "006" }
};



uint8_t txBuffer[1];
uint8_t rxBuffer[2];
I2C_Transaction i2cTransaction;

// Driver Handles - Global variables

volatile int timerFlag = 0;
typedef enum {
    STATE_OFF,
    STATE_ON
} HeatState;

// Driver Handles - Global variables
UART2_Handle uart;
I2C_Handle i2c;
Timer_Handle timer0;

// Timer intervals (in milliseconds)
const int BUTTON_CHECK_INTERVAL = 200000;
const int TEMP_CHECK_INTERVAL = 300000;
const int REPORT_INTERVAL = 500000;


// state, interrupt, and elapsed time variables
volatile HeatState currentState = STATE_OFF;
volatile int button0Flag = 0;
volatile int button1Flag = 0;
volatile uint32_t seconds = 0;


// Global variables to store the temperature, set-point, heater status, and timer count

volatile int16_t ambient_temp = 25;      // Example initial temperature in Celsius
volatile int16_t set_point = 22;         // Example initial set-point temperature in Celsius
volatile bool heater_on = false;     // Heater status (false = off, true = on)
volatile unsigned int seconds_since_reset = 0;  // Counter for seconds since reset

// method declarations

void initUART(void);
void initI2C(void);
int16_t readTemp(void); // check the ambient temperature
void sendStateToUART(void); // report to server
void setTimerPeriod(int); // encapsulation of set timer period operations
void timerCallback(Timer_Handle, int_fast16_t); // timer callback
void updateStateDisplay(void); // update the appropriate LED setting
void setTempStateON(void); // set the state
void setTempStateOFF(void); // set the state
void showTempStateON(void);  // turn LED ON
void showTempStateOFF(void); // turn LED OFF
void initTimer(void); // initialize timer
void gpioButtonFxn0(uint_least8_t); // button interrupt callback
void gpioButtonFxn1(uint_least8_t); // button interrupt callback
void checkSetPointButtons(void); // check to see if the buttons have been pressed
void compareAmbientToDesiredTemp(void); // read the ambient temp and compare to set_pointm then setTempState
void init_system(void); // system initialization command to keep the main thread clean and readable

// Driver Handles - Global variables

I2C_Handle i2c;

// initUART

void initUART(void)
{
    UART2_Params uartParams;

    UART2_Params_init(&uartParams);
    uartParams.baudRate = 115200; // Standard for embedded systems

    uart = UART2_open(CONFIG_UART2_0, &uartParams);
    if (uart == NULL)
    {
        while (1) {
            DISPLAY(snprintf(output, 64, "UART2 initialization failed\n\r"));
        } // UART initialization failed
    }
}




// Make sure you call initUART() before calling this function.

void initI2C(void)

{
    int8_t i;
    int8_t found;
    I2C_Params  i2cParams;

    DISPLAY(snprintf(output, 64, "Initializing I2C Driver - "))

    // Init the driver
    I2C_init();

    // Configure the driver
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;

    // Open the driver
    i2c = I2C_open(CONFIG_I2C_0, &i2cParams);
    if (i2c == NULL)
    {
        DISPLAY(snprintf(output, 64, "Failed\n\r"))
        while (1);
    }



    DISPLAY(snprintf(output, 32, "Passed\n\r"))

    // Boards were shipped with different sensors.

    // Welcome to the world of embedded systems.

    // Try to determine which sensor we have.

    // Scan through the possible sensor addresses

    /* Common I2C transaction setup */

    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 0;

    found = false;

    for (i=0; i<3; ++i)
    {
        i2cTransaction.targetAddress = sensors[i].address;
        txBuffer[0] = sensors[i].resultReg;
        DISPLAY(snprintf(output, 64, "Is this %s? ", sensors[i].id))
        if (I2C_transfer(i2c, &i2cTransaction))

        {
            DISPLAY(snprintf(output, 64, "Found\n\r"))
            found = true;
            break;
        }
        DISPLAY(snprintf(output, 64, "No\n\r"))
    }

    if(found)

    {
        DISPLAY(snprintf(output, 64, "Detected TMP%s I2C address: %x\n\r", sensors[i].id, i2cTransaction.targetAddress))
    }
    else

    {
        DISPLAY(snprintf(output, 64, "Temperature sensor not found, contact professor\n\r"))
    }
}

// read ambient temperature from sensor
int16_t readTemp(void)
{
    int j;
    int16_t temperature = 0;

    i2cTransaction.readCount = 2;
    if (I2C_transfer(i2c, &i2cTransaction))
    {
        /*

         * Extract degrees C from the received data;

         * see TMP sensor datasheet

         */

        temperature = (rxBuffer[0] << 8) | (rxBuffer[1]);
        temperature *= 0.0078125;

        /*

         * If the MSB is set '1', then we have a 2's complement

         * negative value which needs to be sign extended

         */
        if (rxBuffer[0] & 0x80)
        {
            temperature |= 0xF000;
        }
    }
    else
    {
        DISPLAY(snprintf(output, 64, "Error reading temperature sensor (%d)\n\r",i2cTransaction.status))
        DISPLAY(snprintf(output, 64, "Please power cycle your board by unplugging USB and plugging back in.\n\r"))
    }
    return temperature;
 }





// write the current state variables to the UART
void sendStateToUART()
{
    DISPLAY(snprintf(output, 64, "<%02d, %02d, %d, %04d>\n\r", ambient_temp, set_point, currentState, seconds));
}
// encapsulation of call to Timer_setPeriod and all ancillary operations

// Note that this is the same encapsulation of timer code that I provided in module five

void setTimerPeriod(int DURATION)
{
    Timer_stop(timer0);
    timerFlag=0;
    Timer_setPeriod(timer0,Timer_PERIOD_US,DURATION);
    Timer_start(timer0);
    while(timerFlag==0){}
}



/* timer callback

 *

 */

void timerCallback(Timer_Handle myHandle, int_fast16_t status)

{
    timerFlag=1;
}



// set temp state ON

void setTempStateON(void)
{
    if (!heater_on) // Only log if the state is actually changing
    {
        heater_on = true; // Update heater status
        DISPLAY(snprintf(output, 64, "Heater turned ON\n\r"));
        updateStateDisplay(); // Update LED state
    }
    currentState = STATE_ON;
}




// set temp state OFF

void setTempStateOFF(void)
{
    if (heater_on) // Only log if the state is actually changing
    {
        heater_on = false; // Update heater status
        DISPLAY(snprintf(output, 64, "Heater turned OFF\n\r"));
        updateStateDisplay(); // Update LED state
    }
    currentState = STATE_OFF;
}




// show red LED ON to indicate heat is on

void showTempStateON() {
    GPIO_write(CONFIG_GPIO_RED_LED, 0);  // Turn on RED LED
}



// show red LED OFF to indicate heat is off

void showTempStateOFF() {
    GPIO_write(CONFIG_GPIO_RED_LED, 1);  // Turn off RED LED
}



// update LED state

void updateStateDisplay(void)

{
    if (currentState == STATE_ON) {
        showTempStateON();
    } else {
        showTempStateOFF();
    }
}



void initTimer(void)

{

    Timer_Params params;


    Timer_init();
    Timer_Params_init(&params);
    params.period = 5000;
    params.periodUnits = Timer_PERIOD_US;
    params.timerMode = Timer_CONTINUOUS_CALLBACK;
    params.timerCallback = timerCallback;
    timer0 = Timer_open(CONFIG_TIMER_0, &params);
    if (timer0 == NULL)
    {
        /* Failed to initialize timer */
        while (1) {}
    }

    if (Timer_start(timer0) == Timer_STATUS_ERROR) {
        /* Failed to start timer */
        while (1) {}
    }

}

/*

 *  ======== gpioButtonFxn0 ========

 *  Callback function for the GPIO interrupt on CONFIG_GPIO_BUTTON_0.

 *

 *  Note: GPIO interrupts are cleared prior to invoking callbacks.

 */

void gpioButtonFxn0(uint_least8_t index)

{
    button0Flag = 1;
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
    button1Flag = 1;
}



void checkSetPointButtons()
{
    if (button0Flag == 1)
    {
        set_point--; // Decrease temperature
        DISPLAY(snprintf(output, 64, "Set-point decreased to: %02d\n\r", set_point));
        button0Flag = 0; // Reset the flag
    }

    if (button1Flag == 1)
    {
        set_point++; // Increase temperature
        DISPLAY(snprintf(output, 64, "Set-point increased to: %02d\n\r", set_point));
        button1Flag = 0; // Reset the flag
    }
}


// compare ambient_temp to set point and adjust state accordingly

void compareAmbientToDesiredTemp()
{
    ambient_temp = readTemp(); // Read current ambient temperature
    if (ambient_temp < set_point)
    {
        setTempStateON(); // Turn heater ON
    }
    else
    {
        setTempStateOFF(); // Turn heater OFF
    }
}


// Function to initialize the system.

// Moving these operations out of the main thread keeps the task scheduler more readable

void init_system() {

    // call driver init functions

    GPIO_init();



    // very important code block that will turn on your temperature sensor

 #ifdef CONFIG_GPIO_TMP_EN
     GPIO_setConfig(CONFIG_GPIO_TMP_EN, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_HIGH);
     /* Allow the sensor to power on */
     sleep(1);
 #endif



    // Note that the rest of this method is the same as in our GPIO morse code project



    // Set LED pins as output

    //GPIO_setConfig(CONFIG_GPIO_GREEN_LED, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);

    GPIO_setConfig(CONFIG_GPIO_RED_LED, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_YELLOW_LED, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);

    // configure the user button pins
    GPIO_setConfig(CONFIG_GPIO_BUTTON_0, GPIO_CFG_IN_INT_FALLING | GPIO_CFG_INPUT);
    GPIO_setCallback(CONFIG_GPIO_BUTTON_0, gpioButtonFxn0);
    GPIO_enableInt(CONFIG_GPIO_BUTTON_0);

    /*  If more than one input pin is available for your device, interrupts

    *  will be enabled on CONFIG_GPIO_BUTTON1.

    */
   if (CONFIG_GPIO_BUTTON_0 != CONFIG_GPIO_BUTTON_1) {
       /* Configure BUTTON1 pin */
       GPIO_setConfig(CONFIG_GPIO_BUTTON_1, GPIO_CFG_IN_INT_FALLING | GPIO_CFG_INPUT);

       /* Install Button callback */
       GPIO_setCallback(CONFIG_GPIO_BUTTON_1, gpioButtonFxn1);
       GPIO_enableInt(CONFIG_GPIO_BUTTON_1);
   }

   /* Call remaining driver init functions */
   initUART();

   initI2C();
}



/*

 *  ======== mainThread ========

    The main thread will drive our task scheduler.

    Therefore, keep it light and readable

 */

void *mainThread(void *arg0)

{

    init_system(); // Initialize system
    initTimer();
    setTempStateOFF(); // initial state of heat

    DISPLAY(snprintf(output, 64, "Starting Task Scheduler\n\r"));

    while (1)
    {
        // Check buttons every 200ms
        setTimerPeriod(BUTTON_CHECK_INTERVAL);
        checkSetPointButtons();

        // Check and update temperature every 300ms
        setTimerPeriod(TEMP_CHECK_INTERVAL);
        compareAmbientToDesiredTemp();

        // Report state every 500ms
        setTimerPeriod(REPORT_INTERVAL);
        sendStateToUART();

        // Increment time counter
        seconds++;
    }
}
