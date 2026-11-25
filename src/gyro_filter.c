// SPDX-License-Identifier: GPL-2.0-only
// Adaptive One Euro gyro stabiliser. See docs/gyro_stabiliser.md.
//
// Classic One Euro filter applied to the camera ANGLE (integrated gyro), with
// the gyro reading itself used as the derivative (no finite differences):
//     fc = min_cutoff + beta * speed,  speed = |velocity| low-passed at d_cutoff
// Filtering the angle (not the velocity) guarantees the total rotation is
// preserved: whatever is held back is emitted once the hand slows down, so
// RWS / sensitivity calibration is unchanged. Only the angle lag is stored,
// so there is no float precision loss from an ever-growing absolute angle.
//
// Optional fast-motion bypass: between 75% and 100% of bypass_dps the output
// velocity crossfades linearly from filtered to raw. Any lag held at that
// moment stays pending (not dumped) and drains smoothly once below the band,
// so there is no jump entering or leaving bypass and rotation is preserved.
//
// Tremor gate (STRONG): the cutoff is driven by the low-passed SIGNED velocity,
// so oscillating tremor averages out while deliberate aim does not. The tremor
// level itself is measured (RMS of velocity around that average, capped per
// sample and frozen during/just after fast motion so flicks don't inflate it)
// and subtracted, so the gate adapts to how shaky the hand is. Result: heavy
// smoothing while holding still, near-raw response once real motion starts.

#include <math.h>
#include "gyro_filter.h"

static const GyroFilterParams params[GYRO_STAB_MODES] = {
    [GYRO_STAB_OFF]         = {0, 0, 0, 0, 0, 0},
    [GYRO_STAB_LOW_LATENCY] = {8.0f, 0.10f, 2.0f, 120.0f, 0, 0},
    [GYRO_STAB_BALANCED] = {5.0f, 0.08f, 1.0f, 120.0f, 0, 0},
    [GYRO_STAB_STRONG]   = {2.0f, 50.0f, 10.0f, 180.0f, 3.0f, 0},
};

#define TREMOR_HZ 2.0f       // Smoothing of the tremor level estimate.
#define TREMOR_CAP_DPS 30.0f // Per-sample clamp on the tremor contribution.
#define TREMOR_CALM_DPS 30.0f // Below this |x|, remember the tremor level as "calm".
#define TREMOR_FREEZE 0.5f   // Above this fraction of bypass_dps (a flick), restore
#define TREMOR_HOLD_S 0.15f  // the calm level and freeze it, until this long after.

const GyroFilterParams* gyro_filter_params(GyroStabMode mode) {
    if (mode >= GYRO_STAB_MODES) mode = GYRO_STAB_OFF;
    return &params[mode];
}

void gyro_filter_reset(GyroFilterAxis *axis) {
    axis->lag = 0;
    axis->speed = 0;
    axis->tremor = 0;
    axis->calm = 0;
    axis->hold = 0;
    axis->init = false;
}

// Exact EMA coefficient for a first-order low-pass at fc over dt.
static inline float lp_alpha(float fc, float dt) {
    float tau = 1.0f / (2.0f * (float)M_PI * fc);
    return dt / (dt + tau);
}

float gyro_filter_step(GyroFilterAxis *axis, const GyroFilterParams *p, float x, float dt) {
    if (!isfinite(x)) x = 0;
    if (!(dt > 0) || !isfinite(dt)) dt = 0.004f;
    if (dt < 0.0005f) dt = 0.0005f;
    if (dt > 0.05f) dt = 0.05f;
    float ax = fabsf(x);
    if (!axis->init) {
        // First sample after engage/reset: start from the current motion.
        axis->lag = 0;
        axis->speed = p->tremor_k > 0 ? x : ax;
        axis->tremor = 0;
        axis->calm = 0;
        axis->hold = 0;
        axis->init = true;
        return x;
    }
    float speed;
    if (p->tremor_k > 0) {
        axis->speed += lp_alpha(p->d_cutoff, dt) * (x - axis->speed);
        float hp = x - axis->speed;
        if (hp > TREMOR_CAP_DPS) hp = TREMOR_CAP_DPS;
        if (hp < -TREMOR_CAP_DPS) hp = -TREMOR_CAP_DPS;
        if (ax < TREMOR_CALM_DPS && axis->hold <= 0) axis->calm = axis->tremor;
        if (ax > TREMOR_FREEZE * p->bypass_dps) {
            if (axis->hold <= 0) axis->tremor = axis->calm;  // Undo the flick's run-up.
            axis->hold = TREMOR_HOLD_S;
        }
        if (axis->hold > 0) axis->hold -= dt;
        else axis->tremor += lp_alpha(TREMOR_HZ, dt) * (hp * hp - axis->tremor);
        speed = fabsf(axis->speed) - p->threshold_dps - p->tremor_k * sqrtf(axis->tremor);
        if (speed < 0) speed = 0;
    } else {
        axis->speed += lp_alpha(p->d_cutoff, dt) * (ax - axis->speed);
        speed = axis->speed;
    }
    float a = lp_alpha(p->min_cutoff + p->beta * speed, dt);
    float pending = axis->lag + x * dt;  // Raw angle minus last filtered angle.
    float step = a * pending;            // Filtered angle increment.
    if (p->bypass_dps > 0) {
        float w = (ax - 0.75f * p->bypass_dps) / (0.25f * p->bypass_dps);
        if (w > 0) step += (w > 1 ? 1 : w) * (x * dt - step);
    }
    axis->lag = pending - step;
    return step / dt;
}
