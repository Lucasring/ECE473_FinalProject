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
    float CmagX {0.0f};
    float CmagY {0.0f};
    float CmagZ {0.0f};
    float magnitude {0.0f};
} magnetContext;

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
volatile extern EulerAngles      g_OrientationEuler;
volatile extern accelContext     ACCEL_CONTEXT;
volatile extern magnetContext    MAGNET_CONTEXT;
volatile extern compassContext   COMPASS_CONTEXT;