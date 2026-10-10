/*
 * params.h
 *
 *  Created on: 2026/06/22
 *      Author: Karasawa
 */

#ifndef INC_PARAMS_H_
#define INC_PARAMS_H_

 /*============================================================
     各種定数（パラメータ）設定
 ============================================================*/
 /*------------------------------------------------------------
     走行系
 ------------------------------------------------------------*/
 //----数学定数----
#define PI 3.14159265358979323846f // 円周率
 //----機体定数----
#define PULSE_PER_REV      400.0f       // モータ1回転あたりのパルス数
#define WHEEL_DIAMETER_MM     52.f        // ホイール直径(mm)
 //----走行パルス関連----
#define MM_TO_PULSES(mm) ((unsigned)((mm) * PULSE_PER_REV / (WHEEL_DIAMETER_MM * PI) + 0.5f))
// Legacy pulse constants follow the same geometry as distance-based motion.
#define PULSE_SEC_HALF MM_TO_PULSES(HALF_CELL_MM)
#define PULSE_ROT_R90 MM_TO_PULSES(ROTATE_90_MM)
#define PULSE_ROT_L90 MM_TO_PULSES(ROTATE_90_MM)
#define PULSE_ROT_180 MM_TO_PULSES(ROTATE_180_MM)
#define PULSE_SETPOS_BACK MM_TO_PULSES(SETPOS_BACK_MM)
#define PULSE_SETPOS_SET MM_TO_PULSES(SETPOS_SET_MM)
//----テーブルカウンタ関連----
#define MAX_T_CNT PULSE_SEC_HALF // テーブルカウンタの最大値
#define MIN_T_CNT 0              // テーブルカウンタの最小値
//----タイマ関連----
#define DEFAULT_INTERVAL 3000 // デフォルトのインターバル

//----動作方向関連----
#define MT_FWD_L GPIO_PIN_SET    // CW/CCWで前に進む出力（左）
#define MT_BACK_L GPIO_PIN_RESET // CW/CCWで後ろに進む出力（左）
#define MT_FWD_R GPIO_PIN_RESET  // CW/CCWで前に進む出力（右）
#define MT_BACK_R GPIO_PIN_SET   // CW/CCWで後ろに進む出力（右）

/*------------------------------------------------------------
    センサ系
------------------------------------------------------------*/
//----壁判断閾値（しきい値）----

#define WALL_BASE_FR 258U
#define WALL_BASE_FL 456U
#define WALL_BASE_R  150U
#define WALL_BASE_L  350U

#define BATT_LOW_VOL 3204 * 0.88 // バッテリ電圧低下判定のしきい値
// 33kΩと10kΩの分圧抵抗を通してバッテリ電圧を取得している→ 11.1*(10/(10+33)/3.3)*4096=3204

//----制御閾値（しきい値）----
// Initial centres converted from previous raw references L=960 / R=550.
// L/R retain their independent calibration reference, not body-centre distance.
#define WALL_TARGET_L_MM 41.588235f
#define WALL_TARGET_R_MM 38.490196f
#define WALL_FOLLOW_MAX_MM 70.0f // Ignore distant/opening-edge readings for steering.
#define WALL_FOLLOW_DEADBAND_MM 1.0f
#define WALL_FOLLOW_KP_DEG_S_PER_MM 2.0f
#define WALL_FOLLOW_MAX_DEG_S 60.0f
#define WALL_FOLLOW_MIN_WHEEL_MM_S 30.0f

//----赤外線（赤色）LED発光待機時間（単位はマイクロ秒）
#define IR_WAIT_US 15

// Front-wall stopping experiment only (main mode 6 / test 3 / option 1).
#define FRONT_STOP_TARGET_MM 50.0f
#define FRONT_STOP_SPEED_MM_S 30.0f
#define FRONT_STOP_START_MARGIN_MM 5.0f
#define FRONT_STOP_START_MAX_MM 85.0f
#define FRONT_STOP_MAX_DIFFERENCE_MM 10.0f
#define FRONT_STOP_MAX_TRAVEL_MM 40.0f
#define FRONT_STOP_TIMEOUT_MS 2000U

/*------------------------------------------------------------
    探索系
------------------------------------------------------------*/
#define HALF_CELL_MM (90.f)
#define TURN_ENTRY_EXTRA_MM 3.5f // 通常の左右90度旋回前に追加で進む距離[mm]
#define FULL_CELL_MM (180.f)
#define ROTATE_90_MM (TREAD_MM * PI / 4.f) // 90度回転時の走行距離[mm]
#define ROTATE_180_MM (TREAD_MM * PI / 2.f) // 180度回転時の走行距離[mm]
#define SETPOS_BACK_MM (100.f) // 後ろ壁に当てるために下がる距離[mm]
#define SETPOS_SET_MM (40.f) // 後ろ壁から中央までの距離[mm]

#define TREAD_MM (80.f) // トレッド幅[mm]（左右の車輪の中心間距離）

#define DEFAULT_ACCEL ((DEFAULT_SPEED * DEFAULT_SPEED - 0*0) / (2.f * HALF_CELL_MM)) // デフォルト加速度[mm/s^2]
#define DEFAULT_SPEED 525.0f // デフォルト速度[mm/s]
#define DEFAULT_ROTATE_SPEED 225.0f // デフォルト回転速度[deg/s]

// モード4専用。スラローム Initial values require on-robot tuning.
#define SECOND_RUN_STRAIGHT_SPEED_MM_S 800.0f // Modes 3/4; turns retain their calibrated speeds.
#define SLALOM_SPEED_MM_S 500.0f
#define SLALOM_MIN_SPEED_MM_S 30.0f // Keep step periods within the 16-bit timer range.
#define SLALOM_TURN_ANGLE_DEG 90.0f // 旋回角度。加速・一定速・減速でそれぞれ1/3ずつ回る。
#define SLALOM_HOLD_MS 50U // 一定角速度の時間[ms]。加速・減速は各2倍。
#define SLALOM_RAMP_MS (2U * SLALOM_HOLD_MS)
// 最大角速度[°/s] = 旋回角度 × 1000 /（加速時間 + 一定角速度の時間）[ms]。

// 500 mm/s、100/50/100 ms：最大角速度600°/s。各区間で30°ずつ回る。
// 進入・脱出の直進を加えて、縦・横の移動量をそれぞれ90 mmにする。
#define SLALOM_R_ENTRY_MM 10.618f
#define SLALOM_R_EXIT_MM 14.619f
#define SLALOM_L_ENTRY_MM 12.618f
#define SLALOM_L_EXIT_MM 13.618f
#define SLALOM_R_ANGLE_GAIN 1.f
#define SLALOM_L_ANGLE_GAIN 1.f

//----ゴール座標----
// Mode 0: online Adachi exploration, independent of the saved-map run.
#define EXPLORE_SLALOM_SPEED_MM_S 500.0f
#define EXPLORE_SLALOM_KNOWN_STRAIGHT_SPEED_MM_S 600.0f // Mode 0 known-straight cruise after first goal.
#define EXPLORE_KNOWN_STRAIGHT_SPEED_MM_S 800.0f // Mode 2 known-straight cruise after first goal.
#define EXPLORE_SLALOM_TURN_ANGLE_DEG 90.0f
#define EXPLORE_SLALOM_HOLD_MS 50U // 500 mm/s: keep approximately the previous turn radius.
#define EXPLORE_SLALOM_RAMP_MS (2U * EXPLORE_SLALOM_HOLD_MS)
// Nominal entry/exit: 15.319 mm; retain the previous -3/-1 mm tuning offsets.
#define EXPLORE_SLALOM_R_ENTRY_MM 10.618f
#define EXPLORE_SLALOM_R_EXIT_MM 14.619f
#define EXPLORE_SLALOM_L_ENTRY_MM 12.618f
#define EXPLORE_SLALOM_L_EXIT_MM 13.618f
#define EXPLORE_SLALOM_R_ANGLE_GAIN 1.f
#define EXPLORE_SLALOM_L_ANGLE_GAIN 1.f

#define GOAL_X 7 // 7
#define GOAL_Y 7 // 7
//----迷路データ----
#define MAZE_SIZE 16 // 迷路のサイズ（16x16）
#define MAX_ROUTE 256 // 最大経路数

#endif /* INC_PARAMS_H_ */
