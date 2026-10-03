/*
 * drive.c
 *
 *  Created on: Jul 4, 2026
 *      Author: akiaki96
 */

#include "drive.h"
#include "motor.h"
#include "tim.h"
#include <math.h>

DriveData drive = { 0 };

// Only this controller owns the motor commands while mode 4 is moving.
static volatile struct {
    bool enabled, busy, turning, follow_wall;
    float origin, distance, start_speed, end_speed, speed, angle_gain;
    uint32_t turn_tick;
} slalom;

static float SlalomDistance(void) {
    return (motor.distance_mm_l + motor.distance_mm_r) * 0.5f;
}

void DriveSlalomStop(void) {
    uint32_t irq = __get_PRIMASK();
    __disable_irq();
    slalom.enabled = false;
    slalom.busy = false;
    drive.wall_following = false;
    drive.speed_mm_s = 0.0f;
    drive.accel_mm_s2 = 0.0f;
    drive.angle_speed_deg_s = 0.0f;
    drive.angle_accel_deg_s2 = 0.0f;
    MotorSetSpeedAcc(0.0f, 0.0f, 0.0f, 0.0f);
    MotorStop();
    __set_PRIMASK(irq);
}

static bool SlalomWait(uint32_t timeout_ms) {
    uint32_t start = HAL_GetTick();
    while (slalom.busy) {
        if ((uint32_t)(HAL_GetTick() - start) > timeout_ms) {
            DriveSlalomStop();
            printf("Slalom: motion timeout\n");
            return false;
        }
    }
    return true;
}

bool DriveSlalomStraight(float distance_mm, float end_speed, bool follow_wall) {
    if (!isfinite(distance_mm) || distance_mm <= 0.0f ||
            !isfinite(end_speed) || end_speed < 0.0f || end_speed > SLALOM_SPEED_MM_S ||
            (end_speed > 0.0f && end_speed < SLALOM_MIN_SPEED_MM_S)) {
        DriveSlalomStop();
        return false;
    }
    uint32_t irq = __get_PRIMASK();
    __disable_irq();
    bool starting = !slalom.enabled;
    if (starting) slalom.speed = SLALOM_MIN_SPEED_MM_S;
    slalom.start_speed = slalom.speed;
    slalom.end_speed = end_speed;
    slalom.distance = distance_mm;
    slalom.origin = starting ? 0.0f : SlalomDistance();
    slalom.turning = false;
    slalom.follow_wall = follow_wall;
    slalom.busy = true;
    slalom.enabled = true;
    drive.wall_following = false;
    if (starting) {
        MotorSetSpeedAcc(slalom.speed, slalom.speed, 0.0f, 0.0f);
        MotorUpdate();
        __HAL_TIM_SET_AUTORELOAD(&htim16, motor.arr_r);
        __HAL_TIM_SET_AUTORELOAD(&htim17, motor.arr_l);
        MotorStart();
    }
    __set_PRIMASK(irq);
    return SlalomWait((uint32_t)(distance_mm / SLALOM_MIN_SPEED_MM_S * 1000.0f) + 2000U);
}

bool DriveSlalom90(bool right) {
    float entry = right ? SLALOM_R_ENTRY_MM : SLALOM_L_ENTRY_MM;
    float exit = right ? SLALOM_R_EXIT_MM : SLALOM_L_EXIT_MM;
    float gain = right ? SLALOM_R_ANGLE_GAIN : SLALOM_L_ANGLE_GAIN;
    float peak = 90000.0f / (SLALOM_RAMP_MS + SLALOM_HOLD_MS);
    // The pulse distances are unsigned, so this controller requires both
    // wheels to remain forward and above the timer's minimum usable speed.
    if (!slalom.enabled || !isfinite(gain) || gain <= 0.0f ||
            SLALOM_RAMP_MS == 0U ||
            SLALOM_SPEED_MM_S - peak * gain * TREAD_MM * PI / 360.0f < SLALOM_MIN_SPEED_MM_S) {
        DriveSlalomStop();
        return false;
    }
    if (!DriveSlalomStraight(entry, SLALOM_SPEED_MM_S, false)) return false;
    uint32_t irq = __get_PRIMASK();
    __disable_irq();
    slalom.angle_gain = right ? -gain : gain;
    slalom.turn_tick = 0;
    slalom.turning = true;
    slalom.follow_wall = false;
    slalom.busy = true;
    __set_PRIMASK(irq);
    if (!SlalomWait(2U * SLALOM_RAMP_MS + SLALOM_HOLD_MS + 2000U)) return false;
    return DriveSlalomStraight(exit, SLALOM_SPEED_MM_S, false);
}

void DriveSlalomUpdate(void) {
    if (!slalom.enabled) return;
    float omega = 0.0f;
    if (slalom.busy && slalom.turning) {
        const uint32_t total = 2U * SLALOM_RAMP_MS + SLALOM_HOLD_MS;
        if (slalom.turn_tick >= total) {
            slalom.busy = false;
            slalom.turning = false;
        } else {
            // Midpoint samples give exactly 90 degrees of commanded area.
            float t = (float)slalom.turn_tick++ + 0.5f;
            float shape = 1.0f;
            if (t < SLALOM_RAMP_MS) shape = t / SLALOM_RAMP_MS;
            else if (t > SLALOM_RAMP_MS + SLALOM_HOLD_MS)
                shape = ((float)total - t) / SLALOM_RAMP_MS;
            omega = slalom.angle_gain * 90000.0f /
                    (SLALOM_RAMP_MS + SLALOM_HOLD_MS) * shape;
        }
    } else if (slalom.busy) {
        float fraction = (SlalomDistance() - slalom.origin) / slalom.distance;
        if (fraction >= 1.0f) {
            if (slalom.end_speed == 0.0f) {
                DriveSlalomStop();
                return;
            }
            slalom.speed = slalom.end_speed;
            slalom.busy = false;
        } else {
            if (fraction < 0.0f) fraction = 0.0f;
            float v0 = slalom.start_speed;
            float v1 = slalom.end_speed;
            slalom.speed = fmaxf(SLALOM_MIN_SPEED_MM_S,
                    sqrtf(v0 * v0 + (v1 * v1 - v0 * v0) * fraction));
        }
    }
    if (slalom.follow_wall && !slalom.turning) {
        int16_t left = (int16_t)sensor.ad_l - (int16_t)BASE_L;
        int16_t right = (int16_t)sensor.ad_r - (int16_t)BASE_R;
        // Positive omega turns left: steer away from the closer wall.
        if (CTRL_BASE_L < left) omega -= left * CTRL_CONT;
        if (CTRL_BASE_R < right) omega += right * CTRL_CONT;
        // Limit steering at low launch/braking speeds; never reverse a wheel.
        float limit = (slalom.speed - SLALOM_MIN_SPEED_MM_S) * 360.0f / (TREAD_MM * PI);
        omega = fmaxf(-limit, fminf(limit, omega));
    }
    float delta = omega * TREAD_MM * PI / 360.0f;
    MotorSetSpeedAcc(slalom.speed - delta, slalom.speed + delta, 0.0f, 0.0f);
    drive.speed_mm_s = slalom.speed;
    drive.accel_mm_s2 = 0.0f;
    drive.angle_speed_deg_s = omega;
    drive.angle_accel_deg_s2 = 0.0f;
}

void DriveUpdate(void) {
    if (slalom.enabled) return;
    // 機体の速度と加速度をモータの速度と加速度から計算する
    drive.speed_mm_s = (motor.speed_mm_s_l + motor.speed_mm_s_r) / 2.0f;
    drive.accel_mm_s2 = (motor.acc_mm_s2_l + motor.acc_mm_s2_r) / 2.0f;
    drive.angle_speed_deg_s = (motor.speed_mm_s_r - motor.speed_mm_s_l) / TREAD_MM * (180.0f / PI);
    drive.angle_accel_deg_s2 = (motor.acc_mm_s2_r - motor.acc_mm_s2_l) / TREAD_MM * (180.0f / PI);

    if (drive.wall_following) {
        int16_t dif_l = (int16_t)sensor.ad_l - (int16_t)BASE_L;
        int16_t dif_r = (int16_t)sensor.ad_r - (int16_t)BASE_R;
        float ctrl = 0.0f;
        // Positive angular speed turns left with TIM16=right, TIM17=left.
        if (CTRL_BASE_L < dif_l) {
            ctrl -= (float)dif_l * CTRL_CONT;
        }
        if (CTRL_BASE_R < dif_r) {
            ctrl += (float)dif_r * CTRL_CONT;
        }
        // Clear any previous correction when neither wall needs correction.
        DriveSetAngleSpeed(ctrl);
    }
}

void DriveSetSpeed(float speed) {
    // モータの速度を設定する
    MotorSetSpeedAcc(speed, speed, motor.acc_mm_s2_l, motor.acc_mm_s2_r);
    drive.speed_mm_s = speed; // 即座に並進速度を同期
}

void DriveSetAccel(float accel) {
    // モータの加速度を設定する
    MotorSetSpeedAcc(motor.speed_mm_s_l, motor.speed_mm_s_r, accel, accel);
    drive.accel_mm_s2 = accel; // 即座に並進加速度を同期
}

void DriveSetAngleSpeed(float angle_speed) {
    drive.angle_speed_deg_s = angle_speed; // 角速度を設定する
    // 角速度を設定する
    float speed_l = drive.speed_mm_s - (angle_speed * TREAD_MM / 2.0f) * (PI / 180.0f);
    float speed_r = drive.speed_mm_s + (angle_speed * TREAD_MM / 2.0f) * (PI / 180.0f);
    MotorSetSpeedAcc(speed_l, speed_r, motor.acc_mm_s2_l, motor.acc_mm_s2_r);
}

void DriveSetAngleAccel(float angle_accel) {
    drive.angle_accel_deg_s2 = angle_accel; // 即座に角加速度を同期
    // モータの角加速度を設定する
    float accel_l = drive.accel_mm_s2 - (angle_accel * TREAD_MM / 2.0f) * (PI / 180.0f);
    float accel_r = drive.accel_mm_s2 + (angle_accel * TREAD_MM / 2.0f) * (PI / 180.0f);
    MotorSetSpeedAcc(motor.speed_mm_s_l, motor.speed_mm_s_r, accel_l, accel_r);
}

/*
 * DriveHalfSectionAccel
 * 半区画加速走行を行う
 * 引数：なし
 * 戻り値：なし
*/
void DriveHalfSectionAccel(void) {
    drive.wall_following = true;
    DriveSetSpeed(0.f);
    DriveSetAccel(DEFAULT_ACCEL);
    MotorStart();
    while((motor.distance_mm_l < HALF_CELL_MM) || (motor.distance_mm_r < HALF_CELL_MM)) {
    }
    DriveSetSpeed(DEFAULT_SPEED);
    MotorStop();
}

/*
 * DriveHalfSectionBreak
 * 半区画減速走行を行う
 * 引数：なし
 * 戻り値：なし
*/
static void DriveBrakeDistance(float distance_mm) {
    drive.wall_following = true;
    DriveSetSpeed(DEFAULT_SPEED);
    DriveSetAccel(-(DEFAULT_SPEED * DEFAULT_SPEED) / (2.0f * distance_mm));
    MotorStart();
    while((motor.distance_mm_l < distance_mm) || (motor.distance_mm_r < distance_mm)) {
    }
    DriveSetSpeed(0.f);
    DriveSetAccel(0.f);
    MotorStop();
}

void DriveHalfSectionBreak(void) {
    DriveBrakeDistance(HALF_CELL_MM);
}

void DriveOneSection(void) {
    drive.wall_following = true;
    DriveSetSpeed(DEFAULT_SPEED);
    DriveSetAccel(0.f);
    MotorStart();
    while((motor.distance_mm_l < FULL_CELL_MM) || (motor.distance_mm_r < FULL_CELL_MM)) {
    }
    MotorStop();
}

void DriveRotateR90(void) {
    drive.wall_following = false; // 壁追従を無効化

    DriveSetSpeed(0.f);
    DriveSetAccel(0.f);
    MotorStart();
    DriveSetAngleSpeed(-DEFAULT_ROTATE_SPEED);

    while ((motor.distance_mm_l < ROTATE_90_MM) || (motor.distance_mm_r < ROTATE_90_MM)) {

    }

    DriveSetSpeed(0.f);
    DriveSetAccel(0.f);
    DriveSetAngleSpeed(0.f);
    MotorStop();
}

void DriveRotateL90(void) {
    drive.wall_following = false; // 壁追従を無効化

    DriveSetSpeed(0.f);
    DriveSetAccel(0.f);
    MotorStart();
    DriveSetAngleSpeed(DEFAULT_ROTATE_SPEED);

    while ((motor.distance_mm_l < ROTATE_90_MM) || (motor.distance_mm_r < ROTATE_90_MM)) {

    }

    DriveSetSpeed(0.f);
    DriveSetAccel(0.f);
    DriveSetAngleSpeed(0.f);
    MotorStop();
}

void DriveRotateR180(void) {
    drive.wall_following = false; // 壁追従を無効化

    DriveSetSpeed(0.f);
    DriveSetAccel(0.f);
    MotorStart();
    DriveSetAngleSpeed(-DEFAULT_ROTATE_SPEED);

    while ((motor.distance_mm_l < ROTATE_180_MM) || (motor.distance_mm_r < ROTATE_180_MM)) {

    }

    DriveSetSpeed(0.f);
    DriveSetAccel(0.f);
    DriveSetAngleSpeed(0.f);
    MotorStop();
}

void DriveRotateL180(void) {
    drive.wall_following = false; // 壁追従を無効化
    DriveSetSpeed(0.f);
    DriveSetAccel(0.f);
    MotorStart();
    DriveSetAngleSpeed(DEFAULT_ROTATE_SPEED);

    while ((motor.distance_mm_l < ROTATE_180_MM) || (motor.distance_mm_r < ROTATE_180_MM)) {

    }

    DriveSetSpeed(0.f);
    DriveSetAccel(0.f);
    DriveSetAngleSpeed(0.f);
    MotorStop();
}

void DriveTurnR90(void) {
    drive.wall_following = false; // 壁追従を無効化
    DriveBrakeDistance(HALF_CELL_MM + TURN_ENTRY_EXTRA_MM);
    DriveRotateR90();
    DriveHalfSectionAccel();
}

void DriveTurnL90(void) {
    drive.wall_following = false; // 壁追従を無効化
    DriveBrakeDistance(HALF_CELL_MM + TURN_ENTRY_EXTRA_MM);
    DriveRotateL90();
    DriveHalfSectionAccel();
}

void DriveTurn180(void) {
    drive.wall_following = false; // 壁追従を無効化
    DriveHalfSectionBreak();
    DriveRotateR180();
    DriveHalfSectionAccel();
}

void DriveSetPosition(void) {
	// 後壁への位置合わせ中は横壁追従で操舵しない。
	drive.wall_following = false;
    DriveSetAngleSpeed(0.f);
    DriveSetAngleAccel(0.f);
    DriveSetAccel(0.f);
    DriveSetSpeed(-DEFAULT_SPEED/4);
	MotorStart();
    while((motor.distance_mm_l < SETPOS_BACK_MM) || (motor.distance_mm_r < SETPOS_BACK_MM)) {
    }

    MotorStop();
    MotorStart();
    DriveSetSpeed(DEFAULT_SPEED/4);
    while ((motor.distance_mm_l < SETPOS_SET_MM) || (motor.distance_mm_r < SETPOS_SET_MM)) {
    }

    DriveSetSpeed(0.f);
    DriveSetAccel(0.f);
    MotorStop();
}
