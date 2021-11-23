#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <STM32TimerInterrupt.h>
#include <MPU9250.h>
#include <math.h>
#include <BasicLinearAlgebra.h>
#include "DisplayMenu.h"

// Defines
#define TIMER2_PERIOD_MS    1    

// Process Call Period
#define IMU_PERIOD_MS       100
#define MAGNET_PERIOD_MS    200
#define LCD_PERIOD_MS       100

// Process Start Time Offset
#define IMU_OFFSET          0
#define MAGNET_OFFSET       20
#define LCD_OFFSET          50

// State Definitions
typedef enum STATE
{
    INIT = 0,
    IDLE,
    READ_IMU,
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
void _READ_IMU();
void _READ_MAGNETOMETER();
void _UPDATE_LCD();

using namespace BLA;

void setup() {

    // Setup GPIO
    pinMode(PC13, OUTPUT);
    digitalWrite(PC13, LOW);

    // Setup I2C
    Wire2.begin();
    Serial.begin(115200);

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
        case READ_IMU:
            _READ_IMU();
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

    if( (TIMER2_COUNT_MS - IMU_OFFSET) % IMU_PERIOD_MS == 0 )
    {
        CURRENT_STATE = READ_IMU;
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

void _READ_IMU()
{

    IMU.readSensor();
    ACCEL_CONTEXT.accelY = IMU.getAccelX_mss();
    ACCEL_CONTEXT.accelX = IMU.getAccelY_mss();
    ACCEL_CONTEXT.accelZ = IMU.getAccelZ_mss();

    // Setup Current Normalized Accel Vector
    float V_mag = sqrt( pow(ACCEL_CONTEXT.accelX, 2) + pow(ACCEL_CONTEXT.accelY, 2) + pow(ACCEL_CONTEXT.accelZ, 2) );
    float V[3] = { ACCEL_CONTEXT.accelX / V_mag, ACCEL_CONTEXT.accelY / V_mag, ACCEL_CONTEXT.accelZ / V_mag };

    // Calculate Half angle vector
    float half_mag = sqrt( pow(V[0], 2) + pow(V[1], 2) + pow(V[2] - 1, 2) );
    float half[3] = { 
                        V[0] / half_mag, 
                        V[1] / half_mag, 
                        (V[2] - 1) / half_mag 
                    };

    // V crossproduct half
    float cross[3] = {0};
    cross[0] = -(V[1]*half[2] - V[2]*half[1]);
    cross[1] = -(V[2]*half[0] - V[0]*half[2]);
    cross[2] = -(V[0]*half[1] - V[1]*half[0]);

    // Calculate dot product
    float dot = V[0]*half[0] + V[1]*half[1] + V[2]*half[2];

    // Setup Quaternion
    float Q_mag = sqrt( pow(dot, 2) + pow(cross[0], 2) + pow(cross[1], 2) + pow(cross[2], 2) );
    float Q[4] = {
                dot / Q_mag,
                cross[0] / Q_mag,
                cross[1] / Q_mag,
                cross[2] / Q_mag
            };

    // Float Angles
    g_OrientationEuler.roll = atan2f( 2*(Q[0]*Q[1] + Q[2]*Q[3]), 1 - 2*( pow(Q[1], 2) + pow(Q[2], 2) ));
    g_OrientationEuler.pitch = asinf( 2*(Q[0]*Q[2] - Q[3]*Q[1]) );
    g_OrientationEuler.yaw = atan2f( 2*(Q[0]*Q[3] + Q[1]*Q[2]), 1 - 2*( pow(Q[2], 2) + pow(Q[3], 2) ));

    // Store Quaternion
    g_OrientationQuaternion.w       = Q[0];
    g_OrientationQuaternion.roll    = Q[1];
    g_OrientationQuaternion.pitch   = Q[2];
    g_OrientationQuaternion.yaw     = Q[3];

}


volatile Qauternion magQuat;
void _READ_MAGNETOMETER()
{

    IMU.readSensor();
    MAGNET_CONTEXT.magX = IMU.getMagY_uT();
    MAGNET_CONTEXT.magY = IMU.getMagX_uT();
    MAGNET_CONTEXT.magZ = -IMU.getMagZ_uT();

    //volatile float magnitude = sqrt( pow(MAGNET_CONTEXT.magX,2) + pow(MAGNET_CONTEXT.magY,2) + pow(MAGNET_CONTEXT.magZ,2));
    //MAGNET_CONTEXT.magX /= magnitude;
    //MAGNET_CONTEXT.magY /= magnitude;
    //MAGNET_CONTEXT.magZ /= magnitude;

    magQuat.w = g_OrientationQuaternion.w;
    magQuat.roll = g_OrientationQuaternion.roll;
    magQuat.pitch = g_OrientationQuaternion.pitch;
    magQuat.yaw = g_OrientationQuaternion.yaw;

    // Correct MagnetometerData to be in world space not local space
    // Rotates magnetometer vector using orientation quaternion from accelerometer relative to gravity

    Matrix<3, 1, Array<3, 1, float>> MagnetometerData;
    MagnetometerData = {MAGNET_CONTEXT.magX, MAGNET_CONTEXT.magY, MAGNET_CONTEXT.magZ};

    Matrix<3, 3, Array<3, 3, float>> RotationMatrix;
    RotationMatrix = {
                        1.0f - 2*(pow(magQuat.pitch, 2) + pow(magQuat.yaw, 2)),
                        2*(magQuat.roll * magQuat.pitch - magQuat.yaw * magQuat.w),
                        2*(magQuat.roll * magQuat.yaw + magQuat.pitch * magQuat.w),
                        2*(magQuat.roll * magQuat.pitch + magQuat.yaw * magQuat.w),
                        1.0f - 2*(pow(magQuat.roll, 2) + pow(magQuat.yaw, 2)),
                        2*(magQuat.pitch * magQuat.yaw - magQuat.roll * magQuat.w),
                        2*(magQuat.roll * magQuat.yaw - magQuat.pitch * magQuat.w),
                        2*(magQuat.pitch * magQuat.yaw + magQuat.roll * magQuat.w),
                        1.0f - 2*(pow(magQuat.roll, 2) + pow(magQuat.pitch, 2)),
                    };

    Matrix<3, 1, Array<3, 1, float>> correctedMagnetometer = RotationMatrix*MagnetometerData;

    //MAGNET_CONTEXT.magX = correctedMagnetometer(0, 0);
    //MAGNET_CONTEXT.magY = correctedMagnetometer(1, 0);
    //MAGNET_CONTEXT.magZ = correctedMagnetometer(2, 0);

    MAGNET_CONTEXT.magX = MAGNET_CONTEXT.magX * cos(g_OrientationEuler.pitch) + MAGNET_CONTEXT.magY * sin(g_OrientationEuler.roll) * sin(g_OrientationEuler.pitch) -  MAGNET_CONTEXT.magZ * sin(g_OrientationEuler.pitch) * sin(g_OrientationEuler.roll);
    MAGNET_CONTEXT.magY = MAGNET_CONTEXT.magY * cos(g_OrientationEuler.roll) + MAGNET_CONTEXT.magZ * sin(g_OrientationEuler.roll);
    MAGNET_CONTEXT.magZ = 0;

    // Process Into Compass Data
    COMPASS_CONTEXT.heading = 0; // (180 / PI) * atan2f(MAGNET_CONTEXT.magY, MAGNET_CONTEXT.magX);

    //Serial.print(MAGNET_CONTEXT.magX, 3);
    //Serial.print(", ");
    //Serial.print(MAGNET_CONTEXT.magY, 3);
    //Serial.print(", ");
    //Serial.print(MAGNET_CONTEXT.magZ, 3);
    //Serial.print(", ");
    //Serial.print(g_OrientationQuaternion.w, 3);
    //Serial.print(", ");
    //Serial.print(g_OrientationQuaternion.roll, 3);
    //Serial.print(", ");
    //Serial.print(g_OrientationQuaternion.pitch, 3);
    //Serial.print(", ");
    //Serial.println(g_OrientationQuaternion.yaw, 3);

}

void _UPDATE_LCD()
{
    display.draw();
}
