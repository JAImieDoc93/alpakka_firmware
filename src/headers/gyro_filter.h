// SPDX-License-Identifier: GPL-2.0-only
// Adaptive One Euro gyro stabiliser (causal, per-axis, no buffering).
// Pure C with no Pico SDK dependencies so it can be unit tested on a host.

#pragma once
#include <stdbool.h>

typedef enum {
    GYRO_STAB_OFF = 0,
    GYRO_STAB_LOW_LATENCY = 1,
    GYRO_STAB_BALANCED = 2,
    GYRO_STAB_STRONG = 3,
    GYRO_STAB_MODES = 4,
} GyroStabMode;

// Default mode when none is stored in config (-DGYRO_STAB_DEFAULT=0/1/2/3).
#ifndef GYRO_STAB_DEFAULT
    #define GYRO_STAB_DEFAULT GYRO_STAB_OFF
#endif

typedef struct {
    float min_cutoff;  // Hz. Cutoff when the hand is still.
    float beta;        // Hz per deg/s. Cutoff increase with speed.
    float d_cutoff;    // Hz. Smoothing of the speed estimate driving beta.
    float bypass_dps;  // deg/s. Output fully raw at/above this speed (0 = no bypass).
    // Tremor gate (0 = classic One Euro). When > 0, speed is |low-passed signed
    // velocity| (tremor cancels, intent does not), minus threshold_dps and
    // minus tremor_k * measured tremor RMS, so only deliberate motion opens it.
    float tremor_k;
    float threshold_dps;
} GyroFilterParams;

typedef struct {
    float lag;    // Raw angle minus filtered angle (deg). Not yet emitted.
    float speed;  // Low-passed |velocity| (deg/s), or signed velocity (tremor gate).
    float tremor; // Tremor gate: mean square of velocity minus `speed` (deg/s)^2.
    float calm;   // Tremor gate: `tremor` as last seen during slow motion (< 30 deg/s).
    float hold;   // Tremor gate: seconds left with `tremor` frozen after fast motion.
    bool init;
} GyroFilterAxis;

const GyroFilterParams* gyro_filter_params(GyroStabMode mode);
void gyro_filter_reset(GyroFilterAxis *axis);
// x: angular velocity in deg/s. dt: seconds since previous sample.
float gyro_filter_step(GyroFilterAxis *axis, const GyroFilterParams *p, float x, float dt);
