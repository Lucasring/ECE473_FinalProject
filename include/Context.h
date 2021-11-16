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
    float angle {0.0f};
    int compass_position = 0;
} magnetContext;

typedef struct gyroContext
{
    float pitch {0.0f};
    float yaw {0.0f};
    float roll {0.0f};
} gyroContext;

/* Externs */

extern accelContext     ACCEL_CONTEXT;
extern magnetContext    MAGNET_CONTEXT;
extern gryoContext      GRYO_CONTEXT;