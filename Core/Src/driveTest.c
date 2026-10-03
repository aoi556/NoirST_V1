#include "driveTest.h"

#define DRIVE_TEST_STRAIGHT_SECTIONS 3U
#define DRIVE_TEST_WALL_FOLLOWING true

// Mode 7: drive continuously for the configured number of sections.
static void DriveTestStraight(void) {
    const float target_distance_mm = FULL_CELL_MM * DRIVE_TEST_STRAIGHT_SECTIONS;

    drive.wall_following = DRIVE_TEST_WALL_FOLLOWING;
    DriveSetSpeed(DEFAULT_SPEED);
    DriveSetAccel(0.0f);
    MotorStart();
    while ((motor.distance_mm_l < target_distance_mm) ||
           (motor.distance_mm_r < target_distance_mm)) {
    }
    DriveSetSpeed(0.0f);
    DriveSetAccel(0.0f);
    MotorStop();
}

static void DriveTestTurn(bool right) {
    printf("Turn test: 0 = original stop turn, 1 = single slalom (centre start)\n");
    int kind = select_mode(0);
    if (kind == 0) {
        if (right) DriveTurnR90();
        else DriveTurnL90();
        MotorStop();
    } else if (kind == 1) {
        // Place at a cell centre. Half-cell launch, one corner, half-cell stop.
        DriveSlalomStop();
        if (DriveSlalomStraight(HALF_CELL_MM, SLALOM_SPEED_MM_S, true) &&
                DriveSlalom90(right)) {
            DriveSlalomStraight(HALF_CELL_MM, 0.0f, true);
        }
        DriveSlalomStop();
    }
    HAL_Delay(100);
}

void DriveTest(void) {
    printf("Drive Test Mode\n");
    uint8_t mode = 0;
    MotorInit();
    MotorEnable();

    while (true) {
        mode = select_mode(mode); // モード選択関数を呼び出して、モードを更新
        switch (mode) {
            case 0:
                for (int8_t i = 0; i < 8; i++) {
                    DriveRotateR90();
                    HAL_Delay(100);
                }
                break;
            case 1:
                for (int8_t i = 0; i < 8; i++) {
                    DriveRotateL90();
                    HAL_Delay(100);
                }
                break;
            case 2:
                for (int8_t i = 0; i < 4; i++) {
                    DriveRotateR180();
                    HAL_Delay(100);
                }
                break;
            case 3:
                DriveSetPosition();
                HAL_Delay(100);
                break;
            case 4:
                DriveTestTurn(true);
                break;
            case 5:
                DriveTestTurn(false);
                break;
            case 6:
                DriveTurn180();
                MotorStop();
                HAL_Delay(100);
                break;
            case 7:
                DriveSetPosition();
                HAL_Delay(100);
                DriveTestStraight();
                HAL_Delay(100);
                break;
            default:
            break;
        }
    }
}
