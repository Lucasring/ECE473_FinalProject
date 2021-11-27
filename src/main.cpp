#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <STM32TimerInterrupt.h>
#include <MPU9250.h>
#include <math.h>
#include <BasicLinearAlgebra.h>
#include <queue>

#include "DisplayMenu.h"
#include "globals.h"

/* Defines */ 
#define TIMER2_PERIOD_MS    1    

// Process Call Period
#define IMU_PERIOD_MS       100
#define MAGNET_PERIOD_MS    200
#define LCD_PERIOD_MS       100

// Process Start Time Offset
#define IMU_OFFSET          0
#define MAGNET_OFFSET       25
#define LCD_OFFSET          50

#define BUTTON_LEFT         PA0
#define BUTTON_RIGHT        PA1

// State Definitions
typedef enum STATE
{
    IDLE = 0,
    INIT,
    UPDATE_LCD,
    READ_MAGNETOMETER,
    READ_IMU,
    BUTTON_LEFT_PRESSED,
    BUTTON_RIGHT_PRESSED,
} STATE;

// GLOBALS
std::priority_queue<STATE> STATE_QUEUE;

// IO Devices
TwoWire Wire2(PB7, PB6);
STM32TimerInterrupt schedulerTimer(TIM2);
STM32TimerInterrupt debounceTimer(TIM3);
DisplayMenu display;
MPU9250 IMU(Wire2, 0x68);

/**
 * @brief   Timer ISR for main process scheduling
 * @note    Runs every TIMER2_PERIOD_MS milliseconds
 */
void schedulerTimerISR(void);
void buttonLeftISR();
void buttonRightISR();

void _READ_IMU();
void _READ_MAGNETOMETER();
void _UPDATE_LCD();


using namespace BLA;

/**
 * @brief Default setup function run at code start
 * @note IO initialization should occur here
 */
void setup() {

    // Setup GPIO
    pinMode(BUTTON_LEFT, INPUT_PULLUP);
    pinMode(BUTTON_RIGHT, INPUT_PULLUP);

    // IO Interrupt Enables
    attachInterrupt(digitalPinToInterrupt(BUTTON_LEFT), buttonLeftISR, FALLING);
    attachInterrupt(digitalPinToInterrupt(BUTTON_RIGHT), buttonRightISR, FALLING);

    // Setup I2C
    Wire2.begin();
    Serial.begin(115200);

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
    schedulerTimer.setInterval(TIMER2_PERIOD_MS * 1000, schedulerTimerISR);

    // Draw Splash Screen
    display.draw();
    delay(1000);
    display.changeContext(MENU_COMPASS);
    
    // Enable Timer
    schedulerTimer.enableTimer();

}

void loop() {

    while(!STATE_QUEUE.empty()) 
    {

        // select state function
        switch(STATE_QUEUE.top()) 
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
            case IDLE:
                break;
            default:
                break;
        }

        // remove executed state
        STATE_QUEUE.pop();

    }
}

/**
 * @brief schedulerTimerISR for the main scheduling schedulerTimer. Recurring states should be defined here
 * 
 */
unsigned int TIMER2_COUNT_MS = 0;
void schedulerTimerISR(void) 
{

    TIMER2_COUNT_MS += TIMER2_PERIOD_MS;

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

}

void buttonLeftISR()
{

}

void buttonRightISR()
{

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

//do {
//    Serial.println("Calibrating Mag...");
//    IMU.calibrateMag();
//    Serial.println("Calibration results:");
//    Serial.print("Bias X: ");
//    Serial.println(IMU.getMagBiasX_uT(), 3);
//    Serial.print("Bias Y: ");
//    Serial.println(IMU.getMagBiasY_uT(), 3);
//    Serial.print("Bias Z: ");
//    Serial.println(IMU.getMagBiasZ_uT(), 3);
//    Serial.print("Scale X: ");
//    Serial.println(IMU.getMagScaleFactorX(), 3);
//    Serial.print("Scale Y: ");
//    Serial.println(IMU.getMagScaleFactorY(), 3);
//    Serial.print("Scale Z: ");
//    Serial.println(IMU.getMagScaleFactorZ(), 3);
//} while(1);