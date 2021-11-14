#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <STM32TimerInterrupt.h>

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
#define LCD_OFFSET          40

// State Definitions
typedef enum STATE
{
    INIT = 0,
    READ_ACCEL,
    READ_MAGNETOMETER,
    UPDATE_LCD,
} STATE;

// GLOBALS
STATE CURRENT_STATE = INIT;

TwoWire Wire2(PB7, PB6);
STM32TimerInterrupt timer(TIM2);
DisplayMenu display;

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

    switch(CURRENT_STATE) 
    {
        case READ_ACCEL:
            _READ_ACCEL();
            break;
        case READ_MAGNETOMETER:
            _READ_MAGNETOMETER();
            break;
        case UPDATE_LCD:
            _UPDATE_LCD();
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

}

void _READ_MAGNETOMETER()
{

}

void _UPDATE_LCD()
{
    display.draw();
}
