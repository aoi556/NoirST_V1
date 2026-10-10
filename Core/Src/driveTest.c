#include "driveTest.h"
#include "tim.h"
#include <math.h>

#define DRIVE_TEST_STRAIGHT_SECTIONS 6U
#define DRIVE_TEST_WALL_FOLLOWING true

/* Foreground-only experiment. Neither exploration nor the ISR calls this. */
static bool FrontStopRead(float *fl_mm, float *fr_mm) {
    SensorDistance distances[SENSOR_COUNT];
    SensorReadDistances(distances);
    SensorDistance fl = distances[SENSOR_FL];
    SensorDistance fr = distances[SENSOR_FR];
    *fl_mm = fl.mm;
    *fr_mm = fr.mm;
    return fl.status == SENSOR_DISTANCE_OK && fr.status == SENSOR_DISTANCE_OK;
}

static void DriveTestFrontStop(void) {
    DriveSlalomStop(); // Stop PWM and clear previous motion/controller commands.
    float fl, fr;
    const char *reason = "target reached (nearer front sensor)";
    /* Reject edited parameters that exceed calibration or the 16-bit timer.
     * Fixed low speed is deliberate: no high-speed braking in this test. */
    float period = WHEEL_DIAMETER_MM * PI * 1000000.0f /
            (PULSE_PER_REV * FRONT_STOP_SPEED_MM_S);
    if (!isfinite(period) || period < 1.0f || period > 65535.0f ||
            !isfinite(FRONT_STOP_TARGET_MM) || FRONT_STOP_TARGET_MM < 40.0f ||
            FRONT_STOP_TARGET_MM > 80.0f || FRONT_STOP_SPEED_MM_S <= 0.0f ||
            FRONT_STOP_SPEED_MM_S > 50.0f ||
            !(FRONT_STOP_START_MARGIN_MM > 0.0f) ||
            !(FRONT_STOP_START_MAX_MM > FRONT_STOP_TARGET_MM + FRONT_STOP_START_MARGIN_MM) ||
            FRONT_STOP_START_MAX_MM > 90.0f ||
            !(FRONT_STOP_MAX_DIFFERENCE_MM > 0.0f) ||
            FRONT_STOP_MAX_DIFFERENCE_MM > 10.0f ||
            !(FRONT_STOP_MAX_TRAVEL_MM > 0.0f) || FRONT_STOP_MAX_TRAVEL_MM > 40.0f ||
            FRONT_STOP_TIMEOUT_MS == 0U || FRONT_STOP_TIMEOUT_MS > 2000U) {
        printf("Front stop: invalid test parameters\n");
        return;
    }
    printf("Front stop: target=%u mm, speed=%u mm/s; SW3 cancels\n",
            (unsigned)FRONT_STOP_TARGET_MM, (unsigned)FRONT_STOP_SPEED_MM_S);
    HAL_Delay(20); // Allow ordinary TIM6 acquisition while stationary.
    if (!FrontStopRead(&fl, &fr) ||
            fabsf(fl - fr) > FRONT_STOP_MAX_DIFFERENCE_MM ||
            fminf(fl, fr) <= FRONT_STOP_TARGET_MM + FRONT_STOP_START_MARGIN_MM ||
            fmaxf(fl, fr) > FRONT_STOP_START_MAX_MM) {
        printf("Front stop: start rejected; align wall within calibrated start range\n");
        SensorPrintDistances();
        return;
    }

    uint32_t start = HAL_GetTick();
    uint32_t irq = __get_PRIMASK();
    __disable_irq();
    MotorSetSpeedAcc(FRONT_STOP_SPEED_MM_S, FRONT_STOP_SPEED_MM_S, 0.0f, 0.0f);
    MotorUpdate();
    // MotorStart requires valid initial ARR; do not inherit the previous test.
    __HAL_TIM_SET_AUTORELOAD(&htim16, motor.arr_r);
    __HAL_TIM_SET_AUTORELOAD(&htim17, motor.arr_l);
    MotorStart();
    __set_PRIMASK(irq);

    for (;;) {
        if (HAL_GPIO_ReadPin(SW3_GPIO_Port, SW3_Pin) == GPIO_PIN_RESET) {
            reason = "cancelled";
            break;
        }
        if ((uint32_t)(HAL_GetTick() - start) >= FRONT_STOP_TIMEOUT_MS) {
            reason = "timeout";
            break;
        }
        if (motor.distance_mm_l >= FRONT_STOP_MAX_TRAVEL_MM ||
                motor.distance_mm_r >= FRONT_STOP_MAX_TRAVEL_MM) {
            reason = "travel limit";
            break;
        }
        if (!FrontStopRead(&fl, &fr)) {
            reason = "front sensor outside calibration";
            break;
        }
        if (fabsf(fl - fr) > FRONT_STOP_MAX_DIFFERENCE_MM) {
            reason = "front sensor disagreement";
            break;
        }
        // Do not average away a closer reading, or wait for repeated crossings.
        if (fminf(fl, fr) <= FRONT_STOP_TARGET_MM) break;
        HAL_Delay(1);
    }
    DriveSlalomStop(); // Stop before any blocking UART output, on every exit.
    printf("Front stop: %s\n", reason);
    SensorPrintDistances();
    // Consume cancellation so it cannot confirm another menu selection.
    while (HAL_GPIO_ReadPin(SW3_GPIO_Port, SW3_Pin) == GPIO_PIN_RESET) {
        HAL_Delay(1);
    }
    HAL_Delay(100);
}

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
    printf("Turn test: 0 = original stop turn, 1 = single slalom (left/rear wall alignment)\n");
    int kind = select_mode(0);
    if (kind == 0) {
        if (right) DriveTurnR90();
        else DriveTurnL90();
        MotorStop();
    } else if (kind == 1) {
        // Start facing forward with walls on the left and behind the robot.
        DriveSlalomStop();
        DriveSetStartPosition();
        // Left single-slalom test: disable wall correction on the approach too.
        if (DriveSlalomStraight(HALF_CELL_MM, SLALOM_SPEED_MM_S, right) &&
                DriveSlalom90(right)) {
            // Disable wall correction after the turn to check the final heading.
            DriveSlalomStraight(HALF_CELL_MM, 0.0f, false);
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
                printf("Position test: 0 = rear wall (original), 1 = front wall distance stop\n");
                switch (select_mode(0)) {
                    case 0: DriveSetPosition(); break;
                    case 1: DriveTestFrontStop(); break;
                    default: break;
                }
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
