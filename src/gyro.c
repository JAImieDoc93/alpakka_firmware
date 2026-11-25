// SPDX-License-Identifier: GPL-2.0-only
// Copyright (C) 2022, Input Labs Oy.

#include <stdio.h>
#include <math.h>
#include <string.h>
#include "button.h"
#include "config.h"
#include "gyro.h"
#include "gyro_filter.h"
#include "common.h"
#include "hid.h"
#include "imu.h"
#include "pin.h"
#include "touch.h"
#include "vector.h"

double sensitivity_multiplier;

#define GYRO_DPS_PER_LSB 0.0175  // LSM6DS 500dps scale (125dps IMU is pre-divided by 4).
#define GYRO_STAB_DT_MIN 0.002f
#define GYRO_STAB_DT_MAX 0.008f
#define GYRO_STAB_GAP_RESET 0.020f  // s.

static GyroStabMode gyro_stab_mode() {
    return (GyroStabMode)(config_get_gyro_stab() - 1);  // Runtime (home menu) or build default.
}

// Filter raw gyro (calibrated counts) in deg/s. Returns false if disabled.
static bool gyro_stabilise(Gyro *self, Vector *g) {
    GyroStabMode mode = gyro_stab_mode();
    if (mode != self->stab_mode) self->stab_running = false;  // Mode changed at runtime.
    self->stab_mode = mode;
    if (mode == GYRO_STAB_OFF) return false;
    const GyroFilterParams *p = gyro_filter_params(mode);
    uint64_t now = time_us_64();
    float dt = (now - self->stab_last_us) / 1000000.0f;
    // Not called for a while (e.g. home profile held): treat as new engagement.
    if (dt > GYRO_STAB_GAP_RESET) self->stab_running = false;
    if (!self->stab_running) {
        // Fresh engagement: never carry state across gyro on/off.
        for(uint8_t i=0; i<3; i++) gyro_filter_reset(&self->stab[i]);
        self->stab_running = true;
        dt = CFG_TICK_INTERVAL_IN_US / 1000000.0f;
    }
    self->stab_last_us = now;
    if (dt < GYRO_STAB_DT_MIN) dt = GYRO_STAB_DT_MIN;
    if (dt > GYRO_STAB_DT_MAX) dt = GYRO_STAB_DT_MAX;
    g->x = gyro_filter_step(&self->stab[0], p, g->x * GYRO_DPS_PER_LSB, dt) / GYRO_DPS_PER_LSB;
    g->y = gyro_filter_step(&self->stab[1], p, g->y * GYRO_DPS_PER_LSB, dt) / GYRO_DPS_PER_LSB;
    g->z = gyro_filter_step(&self->stab[2], p, g->z * GYRO_DPS_PER_LSB, dt) / GYRO_DPS_PER_LSB;
    return true;
}

uint8_t world_init = 0;
Vector world_top;
Vector world_fw;
Vector world_right;
Vector accel_smooth;

void gyro_update_sensitivity() {
    uint8_t preset = config_get_mouse_sens_preset();
    sensitivity_multiplier = config_get_mouse_sens_value(preset);
}

void gyro_accel_correction() {
    Vector accel = imu_read_accel();
    // Convert to inverted unit value.
    accel.x /= -BIT_14;
    accel.y /= -BIT_14;
    accel.z /= -BIT_14;
    // Get a smoothed gravity vector.
    accel_smooth = vector_smooth(accel_smooth, accel, CFG_ACCEL_CORRECTION_SMOOTH);
    if (world_init < CFG_ACCEL_CORRECTION_SMOOTH) {
        // It the world space orientation is not fully initialized.
        world_top = vector_normalize(vector_invert(accel_smooth));
        world_fw = vector_cross_product(world_top, (Vector){1, 0, 0});
        world_right = vector_cross_product(world_fw, world_top);
        world_init++;
    } else {
        // Correction.
        float rate_fw = (world_right.z - accel_smooth.x) * CFG_ACCEL_CORRECTION_RATE;
        float rate_r = (world_fw.z - accel_smooth.y) * CFG_ACCEL_CORRECTION_RATE;
        Vector4 correction_fw = quaternion(world_fw, rate_fw);
        Vector4 correction_r = quaternion(world_right, -rate_r);
        Vector4 correction = qmultiply(correction_fw, correction_r);
        world_top = qrotate(correction, world_top);
        world_right = qrotate(correction, world_right);
        world_fw = vector_cross_product(world_top, world_right);
    }
}

void gyro_absolute_output(float value, uint8_t *actions, bool *pressed) {
    for(uint8_t i=0; i<4; i++) {
        uint8_t action = actions[i];
        if (hid_is_gamepad_axis(action)) {
            value = fabs(value);
            if      (action == GAMEPAD_AXIS_LX)     hid_gamepad_axis(LX,  value);
            else if (action == GAMEPAD_AXIS_LY)     hid_gamepad_axis(LY,  value);
            else if (action == GAMEPAD_AXIS_LZ)     hid_gamepad_axis(LZ,  value);
            else if (action == GAMEPAD_AXIS_RX)     hid_gamepad_axis(RX,  value);
            else if (action == GAMEPAD_AXIS_RY)     hid_gamepad_axis(RY,  value);
            else if (action == GAMEPAD_AXIS_RZ)     hid_gamepad_axis(RZ,  value);
            else if (action == GAMEPAD_AXIS_LX_NEG) hid_gamepad_axis(LX, -value);
            else if (action == GAMEPAD_AXIS_LY_NEG) hid_gamepad_axis(LY, -value);
            else if (action == GAMEPAD_AXIS_LZ_NEG) hid_gamepad_axis(LZ, -value);
            else if (action == GAMEPAD_AXIS_RX_NEG) hid_gamepad_axis(RX, -value);
            else if (action == GAMEPAD_AXIS_RY_NEG) hid_gamepad_axis(RY, -value);
            else if (action == GAMEPAD_AXIS_RZ_NEG) hid_gamepad_axis(RZ, -value);
        } else {
            if (!(*pressed) && value >= 0.5) {
                hid_press(action);
                if (i==3) *pressed = true;
            }
            else if (*pressed && value < 0.5) {
                hid_release(action);
                if (i==3) *pressed = false;
            }
        }
    }
}

void gyro_incremental_output(double value, uint8_t *actions) {
    for(uint8_t i=0; i<4; i++) {
        uint8_t action = actions[i];
        if      (action == MOUSE_X)     hid_mouse_move(value, 0);
        else if (action == MOUSE_Y)     hid_mouse_move(0, value);
        else if (action == MOUSE_X_NEG) hid_mouse_move(-value, 0);
        else if (action == MOUSE_Y_NEG) hid_mouse_move(0, -value);
    }
}

double hssnf(double t, double k, double x) {
    double a = x - (x * k);
    double b = 1 - (x * k * (1/t));
    return a / b;
}

void Gyro__report_absolute(Gyro *self) {
    // Accel-based correction.
    gyro_accel_correction();
    // Get data from gyros.
    Vector gyro = imu_read_gyro();
    static float sens = -BIT_18 * M_PI;
    // Rotate world space orientation.
    Vector4 rx = quaternion(world_right, gyro.y / sens);
    Vector4 ry = quaternion(world_fw, gyro.z / sens);
    Vector4 rz = quaternion(world_top, gyro.x / sens);
    static uint8_t i = 0;
    Vector4 r;
    if      (i==0) r = qmultiply(qmultiply(rx, ry), rz);
    else if (i==1) r = qmultiply(qmultiply(rz, rx), ry);
    else if (i==2) r = qmultiply(qmultiply(ry, rz), rx);
    else if (i==3) r = qmultiply(qmultiply(rx, rz), ry);
    else if (i==4) r = qmultiply(qmultiply(ry, rx), rz);
    else if (i==5) r = qmultiply(qmultiply(rz, ry), rx);
    i++;
    if (i>5) i = 0;
    world_top = qrotate(r, world_top);
    world_fw = qrotate(r, world_fw);
    world_right = vector_cross_product(world_fw, world_top);
    // Debug.
    bool debug = 0;
    if (debug) {
        hid_gamepad_axis(LX, world_top.x);
        hid_gamepad_axis(LY, -world_top.y);
        hid_gamepad_axis(RX, world_fw.x);
        hid_gamepad_axis(RY, -world_fw.y);
        return;
    }
    // Output calculation.
    float x = degrees(asin(-world_right.z)) / 90;
    float y = degrees(asin(-world_top.z)) / 90;
    float z = degrees(asin(world_fw.z)) / 90;
    if (fabs(x) > 0.5 && z < 0) x += -z * 2 * sign(x); // Steering lock.
    x = constrain(x * 1.1, -1, 1); // Additional saturation.
    x = ramp(x, self->absolute_x_min/90, self->absolute_x_max/90); // Adjust range.
    y = ramp(y, self->absolute_y_min/90, self->absolute_y_max/90); // Adjust range.
    // Output mapping.
    if (x >= 0) gyro_absolute_output( x, self->actions_x_pos, &(self->pressed_x_pos));
    else        gyro_absolute_output(-x, self->actions_x_neg, &(self->pressed_x_neg));
    if (y >= 0) gyro_absolute_output( y, self->actions_y_pos, &(self->pressed_y_pos));
    else        gyro_absolute_output(-y, self->actions_y_neg, &(self->pressed_y_neg));
    // printf("\r%6.1f %6.1f %6.1f", x*100, y*100, z*100);
}

void Gyro__report_incremental(Gyro *self) {
     // Read gyro values.
    Vector imu_gyro = imu_read_gyro();
    gyro_stabilise(self, &imu_gyro);
    double x = imu_gyro.x * CFG_GYRO_SENSITIVITY_X * sensitivity_multiplier;
    double y = imu_gyro.y * CFG_GYRO_SENSITIVITY_Y * sensitivity_multiplier;
    double z = imu_gyro.z * CFG_GYRO_SENSITIVITY_Z * sensitivity_multiplier;
    // Additional processing.
    double t = 1.0;
    double k = 0.5;
    if      (x > 0 && x <  t) x =  hssnf(t, k,  x);
    else if (x < 0 && x > -t) x = -hssnf(t, k, -x);
    if      (y > 0 && y <  t) y =  hssnf(t, k,  y);
    else if (y < 0 && y > -t) y = -hssnf(t, k, -y);
    if      (z > 0 && z <  t) z =  hssnf(t, k,  z);
    else if (z < 0 && z > -t) z = -hssnf(t, k, -z);
    // Report.
    if (x >= 0) gyro_incremental_output( x * self->sens_x, self->actions_x_pos);
    else        gyro_incremental_output(-x * self->sens_x, self->actions_x_neg);
    if (y >= 0) gyro_incremental_output( y * self->sens_y, self->actions_y_pos);
    else        gyro_incremental_output(-y * self->sens_y, self->actions_y_neg);
    if (z >= 0) gyro_incremental_output( z * self->sens_z, self->actions_z_pos);
    else        gyro_incremental_output(-z * self->sens_z, self->actions_z_neg);
}

bool Gyro__is_engaged(Gyro *self) {
    if (self->engage == PIN_NONE) return false;
    if (self->engage == PIN_TOUCH_IN) return touch_status();
    return self->engage_button.is_pressed(&(self->engage_button));
}

void Gyro__report(Gyro *self) {
    bool incremental = false;
    if (self->mode == GYRO_MODE_TOUCH_ON) {
        incremental = self->is_engaged(self);
    }
    else if (self->mode == GYRO_MODE_TOUCH_OFF) {
        incremental = !self->is_engaged(self);
    }
    else if (self->mode == GYRO_MODE_ALWAYS_ON) {
        incremental = true;
    }
    if (incremental) {
        self->report_incremental(self);
        return;
    }
    // Disengaged: drop stabiliser state, residual motion is never emitted.
    self->stab_running = false;
    if (self->mode == GYRO_MODE_AXIS_ABSOLUTE) self->report_absolute(self);
}

void Gyro__reset(Gyro *self) {
    world_init = 0;
    self->stab_running = false;
    self->stab_mode = GYRO_STAB_OFF;
    self->pressed_x_pos = false;
    self->pressed_y_pos = false;
    self->pressed_z_pos = false;
    self->pressed_x_neg = false;
    self->pressed_y_neg = false;
    self->pressed_z_neg = false;
}

void Gyro__config_x(Gyro *self, double min, double max, Actions neg, Actions pos, float sens) {
    self->absolute_x_min = min;
    self->absolute_x_max = max;
    memcpy(self->actions_x_neg, neg, ACTIONS_LEN);
    memcpy(self->actions_x_pos, pos, ACTIONS_LEN);
    self->sens_x = sens;
}

void Gyro__config_y(Gyro *self, double min, double max, Actions neg, Actions pos, float sens) {
    self->absolute_y_min = min;
    self->absolute_y_max = max;
    memcpy(self->actions_y_neg, neg, ACTIONS_LEN);
    memcpy(self->actions_y_pos, pos, ACTIONS_LEN);
    self->sens_y = sens;
}

void Gyro__config_z(Gyro *self, double min, double max, Actions neg, Actions pos, float sens) {
    self->absolute_z_min = min;
    self->absolute_z_max = max;
    memcpy(self->actions_z_neg, neg, ACTIONS_LEN);
    memcpy(self->actions_z_pos, pos, ACTIONS_LEN);
    self->sens_z = sens;
}

Gyro Gyro_ (
    GyroMode mode,
    uint8_t engage
) {
    Gyro gyro;
    gyro.is_engaged = Gyro__is_engaged;
    gyro.report = Gyro__report;
    gyro.report_incremental = Gyro__report_incremental;
    gyro.report_absolute = Gyro__report_absolute;
    gyro.reset = Gyro__reset;
    gyro.config_x = Gyro__config_x;
    gyro.config_y = Gyro__config_y;
    gyro.config_z = Gyro__config_z;
    gyro.mode = mode;
    gyro.engage = engage;
    if (engage != PIN_NONE && engage != PIN_TOUCH_IN) {
        Actions none = {0,};
        gyro.engage_button = Button_(engage, NORMAL, none, none, none);
    }
    memset(gyro.actions_x_pos, 0, ACTIONS_LEN);
    memset(gyro.actions_y_pos, 0, ACTIONS_LEN);
    memset(gyro.actions_z_pos, 0, ACTIONS_LEN);
    memset(gyro.actions_x_neg, 0, ACTIONS_LEN);
    memset(gyro.actions_y_neg, 0, ACTIONS_LEN);
    memset(gyro.actions_z_neg, 0, ACTIONS_LEN);
    gyro_update_sensitivity();
    gyro.reset(&gyro);
    return gyro;
}
