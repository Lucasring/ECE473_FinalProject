#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <STM32TimerInterrupt.h>
#include <MPU9250.h>
#include <math.h>

#include "DisplayMenu.h"

// Defines
#define TIMER2_PERIOD_MS    1    

// Process Call Period
#define ACCEL_PERIOD_MS     100
#define MAGNET_PERIOD_MS    200
#define LCD_PERIOD_MS       100

// Process Start Time Offset
#define ACCEL_OFFSET        0
#define MAGNET_OFFSET       20
#define LCD_OFFSET          50

// State Definitions
typedef enum STATE
{
    INIT = 0,
    IDLE,
    READ_ACCEL,
    READ_MAGNETOMETER,
    UPDATE_LCD,
} STATE;

// GLOBALS
STATE CURRENT_STATE = INIT;

TwoWire Wire2(PB7, PB6);
STM32TimerInterrupt timer(TIM2);
DisplayMenu display;
MPU9250 IMU(Wire2, 0x68);

/**
 * @brief   Timer ISR for main process scheduling
 * @note    Runs every TIMER2_PERIOD_MS milliseconds
 */
void timerISR(void);
void _READ_ACCEL();
void _READ_MAGNETOMETER();
void _UPDATE_LCD();

void setup() {

    // Setup GPIO
    pinMode(PC13, OUTPUT);
    digitalWrite(PC13, LOW);

    // Setup I2C
    Wire2.begin();

    // Setup IMU
    IMU.begin();
    IMU.setAccelRange(MPU9250::ACCEL_RANGE_4G);
    IMU.setGyroRange(MPU9250::GYRO_RANGE_250DPS);

    // Setup Display
    display = DisplayMenu(128, 64, &Wire2, MENU_SPLASH);

    // Setup Timers
    timer.setInterval(TIMER2_PERIOD_MS * 1000, timerISR);

    // Draw Splash Screen
    display.draw();
    delay(1000);
    display.changeContext(MENU_ACCEL);
    
    // Enable Timer
    timer.enableTimer();

}

void loop() {

    //if(CURRENT_STATE == IDLE)   {
    //    digitalWrite(PC13, HIGH);
    //} else {
    //    digitalWrite(PC13, LOW);
    //}

    switch(CURRENT_STATE) 
    {
        case READ_ACCEL:
            _READ_ACCEL();
            CURRENT_STATE = IDLE;
            break;
        case READ_MAGNETOMETER:
            _READ_MAGNETOMETER();
            CURRENT_STATE = IDLE;
            break;
        case UPDATE_LCD:
            _UPDATE_LCD();
            CURRENT_STATE = IDLE;
            break;
        case IDLE:
            break;
    }

}

unsigned int TIMER2_COUNT_MS = 0;
void timerISR(void) 
{

    TIMER2_COUNT_MS += TIMER2_PERIOD_MS;

    if( (TIMER2_COUNT_MS - ACCEL_OFFSET) % ACCEL_PERIOD_MS == 0 )
    {
        CURRENT_STATE = READ_ACCEL;
        return;
    } 
    else if( (TIMER2_COUNT_MS - MAGNET_OFFSET) % MAGNET_PERIOD_MS == 0 )
    {
        CURRENT_STATE = READ_MAGNETOMETER;
        return;
    }
    else if( (TIMER2_COUNT_MS - LCD_OFFSET) % LCD_PERIOD_MS == 0 )
    {
        CURRENT_STATE = UPDATE_LCD;
        return;
    }

}

void _READ_ACCEL()
{
    IMU.readSensor();
    ACCEL_CONTEXT.accelX = IMU.getAccelX_mss();
    ACCEL_CONTEXT.accelY = IMU.getAccelY_mss();
    ACCEL_CONTEXT.accelZ = IMU.getAccelZ_mss();
}

void _READ_MAGNETOMETER()
{

    IMU.readSensor();
    MAGNET_CONTEXT.magZ = IMU.getMagZ_uT();
    MAGNET_CONTEXT.magX = IMU.getMagX_uT();
    MAGNET_CONTEXT.magY = IMU.getMagY_uT();

    GYRO_CONTEXT.pitch = -IMU.getGyroY_rads();
    GYRO_CONTEXT.roll = IMU.getGyroX_rads();
    GYRO_CONTEXT.yaw = -IMU.getGyroZ_rads();

    // Calculate X Heading
    COMPASS_CONTEXT.xh =  MAGNET_CONTEXT.magX*cos(GYRO_CONTEXT.pitch) +
                MAGNET_CONTEXT.magY*sin(GYRO_CONTEXT.roll)*sin(GYRO_CONTEXT.pitch) -
                MAGNET_CONTEXT.magZ*cos(GYRO_CONTEXT.roll)*sin(GYRO_CONTEXT.pitch);

    // Calculate Y Heading
    COMPASS_CONTEXT.yh =  MAGNET_CONTEXT.magY*cos(GYRO_CONTEXT.roll) + 
                MAGNET_CONTEXT.magZ*sin(GYRO_CONTEXT.roll);

    COMPASS_CONTEXT.heading = 180 * atan2(COMPASS_CONTEXT.yh, COMPASS_CONTEXT.xh) / PI;

}

void _UPDATE_LCD()
{
    display.draw();
}
