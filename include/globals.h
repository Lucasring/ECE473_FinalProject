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

typedef struct Qauternion
{
    float w {0.0f};
    float pitch {0.0f}; // X
    float roll {0.0f};  // Y
    float yaw {0.0f};   // Z
} Qauternion;

typedef struct EulerAngles
{
    float roll {0.0f};
    float pitch {0.0f};
    float yaw {0.0f};
} EulerAngles;

typedef struct compassContext
{
    float heading {0.0f}; // Pointed Direction
    float xh;
    float yh;
    int cardinal;
} compassContext;

/* Externs */

extern Qauternion       g_OrientationQuaternion;
extern EulerAngles      g_OrientationEuler;

extern accelContext     ACCEL_CONTEXT;
extern magnetContext    MAGNET_CONTEXT;
extern compassContext   COMPASS_CONTEXT;