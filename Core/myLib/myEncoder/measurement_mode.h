#ifndef MEASUREMENT_MODE_H
#define MEASUREMENT_MODE_H

// Measurement mode enumeration
typedef enum {
    MEASUREMENT_MODE_RPM = 0,
    MEASUREMENT_MODE_LENGTH = 1
} MeasurementMode_t;

// Speed display unit enumeration
typedef enum {
    SPEED_UNIT_RPM = 0,     // Rotations per minute
    SPEED_UNIT_M_MIN = 1    // Meters per minute
} SpeedDisplayUnit_t;

#endif // MEASUREMENT_MODE_H