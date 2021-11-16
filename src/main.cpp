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
    display.changeContext(MENU_COMPASS);
    
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


    // Convert magnet vector into cardinal position
    float magnitude = 1; //sqrt(pow(MAGNET_CONTEXT.magX, 2) + pow(MAGNET_CONTEXT.magY, 2));
    float unitMagX = MAGNET_CONTEXT.magX;
    float unitMagY = MAGNET_CONTEXT.magY;
    float angle = atan2(unitMagY, unitMagX) * 180 / PI;

    MAGNET_CONTEXT.angle = (angle < 0) ? angle += 360 : angle;

    if(angle <= 180 && angle > 135) {
        MAGNET_CONTEXT.compass_position = WEST; 
    } else if(angle <= 135 && angle > 90) {
        MAGNET_CONTEXT.compass_position = NORTH_WEST;
    } else if(angle <= 90 && angle > 45) {
        MAGNET_CONTEXT.compass_position = NORTH;
    } else if(angle <= 45 && angle > 0) {
        MAGNET_CONTEXT.compass_position = NORTH_EAST;
    } else if(angle <= 0 && angle > -45) {
        MAGNET_CONTEXT.compass_position = EAST;
    } else if(angle <= -45 && angle > -90) {
        MAGNET_CONTEXT.compass_position = SOUTH_EAST;
    } else if(angle <= -90 && angle > -135) {
        MAGNET_CONTEXT.compass_position = SOUTH;
    } else if(angle <= -135 && angle > -180) {
        MAGNET_CONTEXT.compass_position = SOUTH_WEST;
    }

    //switch(index)
    //{
    //    case 0:
    //        MAGNET_CONTEXT.compass_position = WEST;
    //        break;
    //    case 1:
    //        MAGNET_CONTEXT.compass_position = SOUTH_WEST;
    //        break;
    //    case 2:
    //        MAGNET_CONTEXT.compass_position = SOUTH;
    //        break;
    //    case 3:
    //        MAGNET_CONTEXT.compass_position = SOUTH_EAST;
    //        break;
    //    case 4:
    //        MAGNET_CONTEXT.compass_position = EAST;
    //        break;
    //    case 5:
    //        MAGNET_CONTEXT.compass_position = NORTH_EAST;
    //        break;
    //    case 6:
    //        MAGNET_CONTEXT.compass_position = NORTH;
    //        break;
    //    case 7:
    //        MAGNET_CONTEXT.compass_position = NORTH_WEST;
    //        break;
    //}

    MAGNET_CONTEXT.compass_position = (MAGNET_CONTEXT.compass_position + 1) % 8;

}

void _UPDATE_LCD()
{
    display.draw();
}
