/*
 * drive.h
 *
 *  Created on: Jul 4, 2026
 *      Author: akiaki96
 */

#ifndef INC_DRIVE_H_
#define INC_DRIVE_H_

#include "global.h"

typedef struct {
    volatile float speed_mm_s; // 機体の速度[mm/s]
    volatile float accel_mm_s2; // 機体の加速度[mm/s^2]
    volatile float angle_speed_deg_s; // 機体の角速度[deg/s]
    volatile float angle_accel_deg_s2; // 機体の角加速度[deg/s^2]

    volatile bool wall_following; // 壁追従中かどうかのフラグ
} DriveData;

#define BASE_L 960// 左センサの中央位置での基準値
#define BASE_R 550 // 右制御閾値

extern DriveData drive;

void DriveHalfSectionAccel(void);
void DriveHalfSectionBreak(void);
void DriveOneSection(void);
void DriveTurnR90(void);
void DriveTurnL90(void);
void DriveTurn180(void);
void DriveRotateR90(void);
void DriveRotateL90(void);
void DriveRotateR180(void);
void DriveRotateL180(void);

void DriveSetPosition(void);

void DriveSetSpeed(float speed);
void DriveSetAccel(float accel);
void DriveSetAngleSpeed(float angle_speed);
void DriveSetAngleAccel(float angle_accel);

void DriveUpdate(void);

// Continuous mode-4 primitives. Start automatically; end_speed == 0 stops.
bool DriveSlalomStraight(float distance_mm, float end_speed, bool follow_wall);
bool DriveSlalom90(bool right);
void DriveSlalomStop(void);
void DriveSlalomUpdate(void); // Called at 1 kHz, before MotorUpdate.

#endif /* INC_DRIVE_H_ */
