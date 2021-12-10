#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <queue>
#include <Adafruit_SSD1306.h>
#include <STM32TimerInterrupt.h>
#include <MPU9250.h>
#include <BasicLinearAlgebra.h>
#include <CircularBuffer.h>

#include "DisplayMenu.h"
#include "globals.h"

/* Defines */ 

// Timer Interrupt Times
#define SCHEDULER_PERIOD_MS     1    
#define DEBOUNCE_PERIOD_MS      20

// Process Call Period
#define IMU_PERIOD_MS       100
#define MAGNET_PERIOD_MS    200
#define LCD_PERIOD_MS       100
#define BATTERY_PERIOD_MS   100

// Process Start Time Offset
#define IMU_OFFSET              5
#define MAGNET_OFFSET           20
#define LCD_OFFSET              40
#define BATTERY_READ_OFFSET     85

// Pin Definitions
#define PIN_BATTERY_VOLTAGE     PA5
#define PIN_BUTTON_LEFT         PA0
#define PIN_BUTTON_RIGHT        PA1
#define PIN_DEBUG               PA7

// Other Variables
#define BATTERY_MIN     1.2*3
#define BATTERY_MAX     4.55

typedef enum BUTTON
{
    LEFT_BUTTON = 0,
    RIGHT_BUTTON,
} BUTTON;

// State Definitions
typedef enum STATE
{
    IDLE = 0,
    INIT,
    UPDATE_LCD,
    READ_MAGNETOMETER,
    READ_IMU,
    READ_BATTERY,
    BUTTON_LEFT_PRESSED,
    BUTTON_RIGHT_PRESSED,
} STATE;

struct DebounceEvent
{
    uint32_t debounceTime;
    BUTTON button;
};

// GLOBALS
CircularBuffer<STATE, 32> STATE_QUEUE;
CircularBuffer<DebounceEvent, 32> DEBOUNCE_QUEUE;

volatile bool BUTTON_LEFT_DEBOUNCE = false;
volatile bool BUTTON_RIGHT_DEBOUNCE = false;

// IO Devices
TwoWire Wire2(PB7, PB6);
STM32TimerInterrupt schedulerTimer(TIM2);
STM32TimerInterrupt debounceTimer(TIM3);
DisplayMenu display;
MPU9250 IMU(Wire2, 0x68);
unsigned int TIMER2_COUNT_MS = 0;


// ISR Functions
void schedulerTimerISR(void);
void debounceTimerISR(void);
void buttonLeftISR();
void buttonRightISR();

// Helper Functions
void buttonHanlder(BUTTON button);

// State functions
void _READ_IMU();
void _READ_MAGNETOMETER();
void _UPDATE_LCD();
void _READ_BATTERY();

using namespace BLA;

/**
 * @brief Default setup function run at code start
 * @note IO initialization should occur here
 */
void setup() {

    // Setup GPIO
    pinMode(PIN_BUTTON_LEFT, INPUT_PULLUP);
    pinMode(PIN_BUTTON_RIGHT, INPUT_PULLUP);
    pinMode(PIN_BATTERY_VOLTAGE, INPUT);
    pinMode(PIN_DEBUG, OUTPUT);
    pinMode(PC13, OUTPUT);

    digitalWrite(PIN_DEBUG, LOW);

    // IO Interrupt Enables
    attachInterrupt(digitalPinToInterrupt(PIN_BUTTON_LEFT), buttonLeftISR, FALLING);
    attachInterrupt(digitalPinToInterrupt(PIN_BUTTON_RIGHT), buttonRightISR, FALLING);

    // Setup I2C
    Wire2.begin();
    //Serial.begin(115200);

    // Setup IMU
    IMU.begin();
    IMU.setAccelRange(MPU9250::ACCEL_RANGE_4G);
    IMU.setGyroRange(MPU9250::GYRO_RANGE_250DPS);
    IMU.setMagCalX(1.306, 0.956);
    IMU.setMagCalY(28.961, 1.086);
    IMU.setMagCalZ(-25.828, 0.998);
    
    // Setup Display
    display = DisplayMenu(128, 64, &Wire2, MENU_SPLASH);

    // Setup Timers
    schedulerTimer.setInterval(SCHEDULER_PERIOD_MS * 1000, schedulerTimerISR);
    debounceTimer.setInterval(DEBOUNCE_PERIOD_MS * 1000, debounceTimerISR);

    debounceTimer.disableTimer();
    schedulerTimer.disableTimer();

    // Draw Splash Screen
    display.draw();
    delay(1000);
    display.changeContext(MENU_COMPASS);
    
    // Enable Timer
    schedulerTimer.enableTimer();

}

/**
 * @brief   loop implements the state machine state handling.
 * 
 */
void loop() {

    digitalWrite(PC13, LOW);

    while(!STATE_QUEUE.isEmpty()) 
    {

        // select state function
        switch(STATE_QUEUE.first()) 
        {
            case READ_IMU:
                _READ_IMU();
                break;
            case READ_MAGNETOMETER:
                _READ_MAGNETOMETER();
                break;
            case UPDATE_LCD:
                _UPDATE_LCD();
                break;
            case READ_BATTERY:
                _READ_BATTERY();
                break;
            case BUTTON_LEFT_PRESSED:
                digitalWrite(PC13, HIGH);
                display.prevMenu();
                break;
            case BUTTON_RIGHT_PRESSED:
                digitalWrite(PC13, HIGH);
                display.nextMenu();
                break;
            case IDLE:
                break;
            default:
                break;
        }

        // remove executed state
        STATE_QUEUE.shift();

    }
}

/**
 * @brief schedulerTimerISR for the main scheduling schedulerTimer. Recurring states should be defined here
 * 
 */
void schedulerTimerISR(void) 
{

    TIMER2_COUNT_MS += SCHEDULER_PERIOD_MS;

    if( (TIMER2_COUNT_MS - IMU_OFFSET) % IMU_PERIOD_MS == 0 )
    {
        STATE_QUEUE.push(READ_IMU);
        return;
    } 
    else if( (TIMER2_COUNT_MS - MAGNET_OFFSET) % MAGNET_PERIOD_MS == 0 )
    {
        STATE_QUEUE.push(READ_MAGNETOMETER);
        return;
    }
    else if( (TIMER2_COUNT_MS - LCD_OFFSET) % LCD_PERIOD_MS == 0 )
    {
        STATE_QUEUE.push(UPDATE_LCD);
        return;
    }
    else if( (TIMER2_COUNT_MS - BATTERY_READ_OFFSET) % BATTERY_PERIOD_MS == 0 )
    {
        STATE_QUEUE.push(READ_BATTERY);
        return;
    }

}

/**
 * @brief   Executes when the debounce timer interrupts. Determines if a button 
 *          debounce is successful and queues the button press state.
 */
void debounceTimerISR()
{

    volatile DebounceEvent event = DEBOUNCE_QUEUE.first();

    // Check if button is pressed low
    volatile int buttonState;
    switch (event.button)
    {
        case LEFT_BUTTON:
            buttonState = digitalRead(PIN_BUTTON_LEFT);
            break;
        case RIGHT_BUTTON:            
            buttonState = digitalRead(PIN_BUTTON_RIGHT);
            break;
        default:
            buttonState = HIGH;
            break;
    }

    if (buttonState == LOW)
    {
        switch(event.button)
        {
            case LEFT_BUTTON:
                BUTTON_LEFT_DEBOUNCE = false;
                STATE_QUEUE.push(BUTTON_LEFT_PRESSED);
                break;
            case RIGHT_BUTTON:
                BUTTON_RIGHT_DEBOUNCE = false;
                STATE_QUEUE.push(BUTTON_RIGHT_PRESSED);
                break;
            default:
                break;
        }

        DEBOUNCE_QUEUE.shift();

        if (DEBOUNCE_QUEUE.isEmpty())
        {
            debounceTimer.disableTimer();
            debounceTimer.setTimerCount(0);
            return;
        }
        else
        {
            debounceTimer.restartTimer();
            debounceTimer.setTimerCount((DEBOUNCE_PERIOD_MS * 1000) - event.debounceTime);
        }
    }
    else
    {
        if (!DEBOUNCE_QUEUE.isEmpty())
        {
            DEBOUNCE_QUEUE.shift();
        } else {
            debounceTimer.disableTimer();
        }

        switch (event.button)
        {
            case LEFT_BUTTON:
                BUTTON_LEFT_DEBOUNCE = false;
                break;
            case RIGHT_BUTTON:            
                BUTTON_RIGHT_DEBOUNCE = false;
                break;
        }

    }

}

/**
 * @brief  Interrupt Handler for left button
 * 
 */
void buttonLeftISR()
{
    buttonHanlder(LEFT_BUTTON);
}

/**
 * @brief   Interrupt Handler for right button
 * 
 */
void buttonRightISR()
{
    buttonHanlder(RIGHT_BUTTON);
}

/**
 * @brief   Executed with button interrupts. Setups the debounce timer and debounce variables as needed
 * @param button    The button that trigged an interrupt request
 */
void buttonHanlder(BUTTON button)
{
    
    DebounceEvent event;
    event.debounceTime = (DEBOUNCE_PERIOD_MS * 1000) - debounceTimer.getTimerCount();
    event.button = button;

    // Check if button is currently debouncing
    volatile bool isButtonDebounced = false;
    switch(button)
    {
        case LEFT_BUTTON:
            isButtonDebounced = BUTTON_LEFT_DEBOUNCE;
            break;
        case RIGHT_BUTTON:
            isButtonDebounced = BUTTON_RIGHT_DEBOUNCE;
            break;
        default:
            isButtonDebounced = false;
            break;
    }

    if (isButtonDebounced == false)
    {

        // If no button is currently debouncing
        if (DEBOUNCE_QUEUE.isEmpty())
        {   
            debounceTimer.setTimerCount(0);
            debounceTimer.enableTimer();
        }   

        DEBOUNCE_QUEUE.push(event);

        // Set debounce state
        switch (button)
        {
            case LEFT_BUTTON:
                BUTTON_LEFT_DEBOUNCE = true;
                break;
            case RIGHT_BUTTON:
                BUTTON_RIGHT_DEBOUNCE = true;
                break;
            default:
                break;
        }

    }

}

/**
 * @brief state definition for READ_IMU state
 * @note reads from the IMU and calculates the Euler roll and pitch orientation based
 */
void _READ_IMU()
{

    IMU.readSensor();
    ACCEL_CONTEXT.accelY = IMU.getAccelX_mss();
    ACCEL_CONTEXT.accelX = IMU.getAccelY_mss();
    ACCEL_CONTEXT.accelZ = IMU.getAccelZ_mss();

    // Setup Vectors
    volatile float A[3] = {ACCEL_CONTEXT.accelX, ACCEL_CONTEXT.accelY, ACCEL_CONTEXT.accelZ};
    volatile float A_mag = sqrt(pow(A[0], 2) + pow(A[1], 2) + pow(A[2], 2));
    volatile float Ax = A[0] / A_mag;
    volatile float Ay = A[1] / A_mag;
    volatile float Az = A[2] / A_mag;

    // Calculate angle between down vector and accel
    g_OrientationEuler.roll =  acosf( (-Az) / sqrt(pow(Ay, 2) + pow(Az, 2)) );
    g_OrientationEuler.roll = (Ay > 0) ? g_OrientationEuler.roll : -g_OrientationEuler.roll;

    g_OrientationEuler.pitch = acosf( (-Az) / sqrt(pow(Ax, 2) + pow(Az, 2)) );
    g_OrientationEuler.pitch = (Ax > 0) ? g_OrientationEuler.pitch : -g_OrientationEuler.pitch;

    // g_OrientationEuler.roll =  acosf( (A[1]*G[1] + A[2]*G[2]) / ( sqrt(pow(A[1], 2) * pow(A[2], 2)) * sqrt(pow(G[1], 2) * pow(G[2], 2)) ));
    // g_OrientationEuler.pitch = acosf( (A[0]*G[0] + A[2]*G[2]) / ( sqrt(pow(A[0], 2) * pow(A[2], 2)) * sqrt(pow(G[0], 2) * pow(G[2], 2)) ));

}

/**
 * @brief state definition for READ_MAGNETOMETER state
 * @note reads the IMU and calculates the tilt corrected magnetometer vector.
 *       Also calculates the cardinal direction and heading for the compass
 */
void _READ_MAGNETOMETER()
{

    IMU.readSensor();

    // Measured Magnetometer Vector with orientation corrections
    Matrix<3, 1, Array<3, 1, float>> magnetometerMeasurement = {
        IMU.getMagY_uT(),   // X Axis
        IMU.getMagX_uT(),   // Y Axis
       -IMU.getMagZ_uT(),   // Z Axis
    };

    // Setup Rotation Matrix
    float phi = g_OrientationEuler.roll;
    float theta = g_OrientationEuler.pitch;
    Matrix<3, 3, Array<3, 3, float>> RotationMatrix = {
        cos(theta),             0,          sin(theta), 
        sin(phi)*sin(theta),    cos(phi),   -cos(theta)*sin(phi), 
        -cos(phi)*sin(theta),   sin(phi),   cos(phi)*cos(theta),
    };

    Matrix<3, 1, Array<3, 1, float>> magnetometerTiltCorrected = RotationMatrix*magnetometerMeasurement;

    // Record original measurement
    MAGNET_CONTEXT.magX = magnetometerMeasurement(0, 0);
    MAGNET_CONTEXT.magZ = magnetometerMeasurement(1, 0);
    MAGNET_CONTEXT.magY = magnetometerMeasurement(2, 0);
    MAGNET_CONTEXT.magnitude = sqrt(pow(MAGNET_CONTEXT.magX, 2) + pow(MAGNET_CONTEXT.magY, 2) + pow(MAGNET_CONTEXT.magZ, 2));

    // Record corrected
    MAGNET_CONTEXT.CmagX = magnetometerTiltCorrected(0, 0);
    MAGNET_CONTEXT.CmagY = magnetometerTiltCorrected(1, 0);
    MAGNET_CONTEXT.CmagZ = magnetometerTiltCorrected(2, 0);
    COMPASS_CONTEXT.heading = (180 / PI) * atan2f(MAGNET_CONTEXT.CmagY, MAGNET_CONTEXT.CmagX) + 270;
    
    // Convert Compass heading into cardinal direction
    if (COMPASS_CONTEXT.heading >=  337.5 || COMPASS_CONTEXT.heading < 22.5) {
        COMPASS_CONTEXT.cardinal = EAST;
    } else if (COMPASS_CONTEXT.heading >= 22.5 && COMPASS_CONTEXT.heading < 67.5) {
         COMPASS_CONTEXT.cardinal = SOUTH_EAST;
    } else if (COMPASS_CONTEXT.heading >= 67.5 && COMPASS_CONTEXT.heading < 112.5) {
        COMPASS_CONTEXT.cardinal = SOUTH;
    } else if (COMPASS_CONTEXT.heading >= 112.5 && COMPASS_CONTEXT.heading < 157.5) {
        COMPASS_CONTEXT.cardinal = SOUTH_WEST;
    } else if (COMPASS_CONTEXT.heading >= 157.5 && COMPASS_CONTEXT.heading < 202.5) {
        COMPASS_CONTEXT.cardinal = WEST;
    } else if (COMPASS_CONTEXT.heading >= 202.5 && COMPASS_CONTEXT.heading < 247.5) {
        COMPASS_CONTEXT.cardinal = NORTH_WEST;
    } else if (COMPASS_CONTEXT.heading >= 247.5 && COMPASS_CONTEXT.heading < 292.5) {
        COMPASS_CONTEXT.cardinal = NORTH;
    } else if (COMPASS_CONTEXT.heading >= 292.5 && COMPASS_CONTEXT.heading < 337.5) {
        COMPASS_CONTEXT.cardinal = NORTH_EAST;
    }



}

/**
 * @brief state definition for the UPDATE_LCD state
 */
void _UPDATE_LCD()
{
    display.draw();
}


/**
 * @brief state definition for the READ_BATTERY state
 * 
 */
void _READ_BATTERY()
{

    BATTERY_CONTEXT.raw = analogRead(PIN_BATTERY_VOLTAGE);
    BATTERY_CONTEXT.value = 2 * BATTERY_CONTEXT.raw * BATTERY_CONTEXT.factor; // Using a 1/2 voltage divider on the read pin
    BATTERY_CONTEXT.percentage = (BATTERY_CONTEXT.value - BATTERY_MIN) / (BATTERY_MAX - BATTERY_MIN);

}