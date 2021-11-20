#pragma once

/* Context Structs */
typedef struct accelContext
{
    float accelX {0.0f};
    float accelY {0.0f};
    float accelZ {0.0f};
} accelContext;

typedef struct magnetContext
{
    float magX {0.0f};
    float magY {0.0f};
    float magZ {0.0f};
} magnetContext;

typedef struct gyroContext
{
    float pitch {0.0f}; // X
    float roll {0.0f};  // Y
    float yaw {0.0f};   // Z
} gyroContext;

typedef struct compassContext
{
    float heading {0.0f}; // Pointed Direction
    float xh;
    float yh;
    int cardinal;
} compassContext;

/* Externs */

extern accelContext     ACCEL_CONTEXT;
extern magnetContext    MAGNET_CONTEXT;
extern gyroContext      GYRO_CONTEXT;
extern compassContext   COMPASS_CONTEXT;