/*
 * sensor.c
 *
 *  Created on: 2026/06/17
 *      Author: Karasawa
 */


#include "global.h"
#include "sensor.h"
#include "adc.h"

typedef struct {
	uint16_t raw;
	uint16_t mm;
} SensorCalibrationPoint;

/* Measured with the existing acquisition/IR illumination configuration.
 * Strictly decreasing raw, increasing distance; no geometric offsets.
 * FR 20-30 mm: 12 counts/10 mm; FL 30-40 mm: 13 counts/10 mm.
 * FL 70-80 mm is even flatter: 6 counts/10 mm. Noise is amplified there. */
static const SensorCalibrationPoint calibration[SENSOR_COUNT][9] = {
	[SENSOR_L] = {{2599,10}, {1880,20}, {1286,30}, {946,40}, {740,50},
		{606,60}, {516,70}, {456,80}, {417,90}},
	[SENSOR_FL] = {{771,10}, {547,20}, {510,30}, {497,40}, {464,50},
		{412,60}, {336,70}, {330,80}, {298,90}},
	[SENSOR_FR] = {{2647,10}, {2528,20}, {2516,30}, {1715,40}, {1311,50},
		{1047,60}, {856,70}, {704,80}, {581,90}},
	[SENSOR_R] = {{950,10}, {798,20}, {670,30}, {555,40}, {453,50},
		{347,60}, {280,70}, {230,80}, {194,90}}
};

SensorDistance SensorRawToDistance(SensorId id, uint32_t raw) {
	SensorDistance result = {-1.0f, 0.0f, SENSOR_DISTANCE_INVALID};
	if ((unsigned)id >= SENSOR_COUNT) return result;
	const SensorCalibrationPoint *table = calibration[id];
	const unsigned last = sizeof(calibration[0]) / sizeof(calibration[0][0]) - 1U;
	if (raw > table[0].raw) {
		result.mm = table[0].mm;
		result.status = SENSOR_DISTANCE_ABOVE_RAW_RANGE;
	} else if (raw < table[last].raw) {
		result.mm = table[last].mm;
		result.status = SENSOR_DISTANCE_BELOW_RAW_RANGE;
	} else {
		for (unsigned i = 0; i < last; ++i) {
			if (raw >= table[i + 1].raw) {
				result.mm_per_count = (float)(table[i + 1].mm - table[i].mm)
					/ (float)(table[i].raw - table[i + 1].raw);
				result.mm = table[i].mm + (table[i].raw - raw) * result.mm_per_count;
				result.status = SENSOR_DISTANCE_OK;
				break;
			}
		}
	}
	return result;
}

void SensorReadDistances(SensorDistance result[SENSOR_COUNT]) {
	const volatile SensorData *live = &sensor;
	uint32_t raw[SENSOR_COUNT];
	uint32_t irq = __get_PRIMASK();
	__disable_irq();
	raw[SENSOR_L] = live->ad_l;
	raw[SENSOR_FL] = live->ad_fl;
	raw[SENSOR_FR] = live->ad_fr;
	raw[SENSOR_R] = live->ad_r;
	__set_PRIMASK(irq);
	for (unsigned i = 0; i < SENSOR_COUNT; ++i)
		result[i] = SensorRawToDistance((SensorId)i, raw[i]);
}

SensorWalls SensorReadWalls(void) {
	const volatile SensorData *live = &sensor;
	uint32_t irq = __get_PRIMASK();
	__disable_irq();
	uint32_t l = live->ad_l;
	uint32_t fl = live->ad_fl;
	uint32_t fr = live->ad_fr;
	uint32_t r = live->ad_r;
	__set_PRIMASK(irq);
	// Original strict thresholds; no distance-range gate on wall detection.
	SensorWalls walls = {
		.l = l > WALL_BASE_L,
		.f = fl > WALL_BASE_FL || fr > WALL_BASE_FR,
		.r = r > WALL_BASE_R
	};
	return walls;
}

void SensorPrintDistances(void) {
	static const char *const names[SENSOR_COUNT] = {"L", "FL", "FR", "R"};
	uint32_t raw[SENSOR_COUNT];
	uint32_t batt;
	/* Snapshot only: restore interrupt state before conversion or UART output.
	 * Volatile reads ensure fresh ISR-owned samples without changing SensorData. */
	const volatile SensorData *live = &sensor;
	uint32_t primask = __get_PRIMASK();
	__disable_irq();
	raw[SENSOR_L] = live->ad_l;
	raw[SENSOR_FL] = live->ad_fl;
	raw[SENSOR_FR] = live->ad_fr;
	raw[SENSOR_R] = live->ad_r;
	batt = live->ad_batt;
	__set_PRIMASK(primask);
	for (unsigned i = 0; i < SENSOR_COUNT; ++i) {
		SensorDistance d = SensorRawToDistance((SensorId)i, raw[i]);
		unsigned tenths = (unsigned)(d.mm * 10.0f + 0.5f);
		const char *note = "";
		if (d.status == SENSOR_DISTANCE_ABOVE_RAW_RANGE)
			note = " [out of calibration: raw high, clamped]";
		else if (d.status == SENSOR_DISTANCE_BELOW_RAW_RANGE)
			note = " [out of calibration: raw low, clamped/no wall possible]";
		/* Integer formatting avoids requiring newlib printf float support. */
		printf("%-2s : raw=%lu, distance=%u.%u mm%s\n",
			names[i], (unsigned long)raw[i], tenths / 10U, tenths % 10U, note);
	}
	printf("Batt: raw=%lu\n", (unsigned long)batt);
}

SensorData sensor = { 0 }; //センサーのAD値を格納する構造体
//+++++++++++++++++++++++++++++++++++++++++++++++
// SensorInit
// センサー情報の初期化を行う
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void SensorInit(void) {
	//センサーの初期化を行う
	sensor.ad_r = 0; //右壁センサーのAD値を初期化
	sensor.ad_fr = 0; //前壁右センサーのAD値を初期化
	sensor.ad_fl = 0; //前壁左センサーのAD値を初期化
	sensor.ad_l = 0; //左壁センサーのAD値を初期化
	sensor.ad_batt = 0; //バッテリー電圧のAD値を初期化
	sensor.low_vol_count = 0; //低電圧フラグまでのカウントを初期化
	sensor.wall_r = false; //右壁センサーの検知フラグを初期化
	sensor.wall_f = false; //前壁センサーの検知フラグを初期化
	sensor.wall_l = false; //左壁センサーの検知フラグを初期化
}
//+++++++++++++++++++++++++++++++++++++++++++++++
// GetWallSensorInfo
// 壁の有無を判定し、フラグを更新する
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void GetWallSensorInfo(void) {
	//壁センサーの検知フラグを更新する
	SensorWalls walls = SensorReadWalls();
	sensor.wall_r = walls.r;
	sensor.wall_f = walls.f;
	sensor.wall_l = walls.l;
	led_write(walls.r, walls.f, walls.l);
}
//+++++++++++++++++++++++++++++++++++++++++++++++
// SensorUpdate
// 各センサーのAD値取得とバッテリー監視を行う
// 引数：なし
// 戻り値：なし
//+++++++++++++++++++++++++++++++++++++++++++++++
void SensorUpdate(void) {
	//l,frのAD値を取得する
	HAL_ADC_Start(&hadc1); // Lの開始
	HAL_ADC_Start(&hadc2); // Rの開始
	HAL_ADC_PollForConversion(&hadc1, 10);
	HAL_ADC_PollForConversion(&hadc2, 10);
	sensor.ad_l = HAL_ADC_GetValue(&hadc1);
	sensor.ad_r = HAL_ADC_GetValue(&hadc2);

	HAL_ADC_Start(&hadc1); // FRの開始
	HAL_ADC_Start(&hadc2); // FLの開始
	HAL_ADC_PollForConversion(&hadc1, 10);
	HAL_ADC_PollForConversion(&hadc2, 10);
	sensor.ad_fr = HAL_ADC_GetValue(&hadc1);
	sensor.ad_fl = HAL_ADC_GetValue(&hadc2);

	HAL_ADC_Start(&hadc1); // Battの開始
	HAL_ADC_PollForConversion(&hadc1, 10);
	sensor.ad_batt = HAL_ADC_GetValue(&hadc1);

	//バッテリー電圧監視
	if (sensor.ad_batt < BATT_LOW_VOL) sensor.low_vol_count++;
	if (sensor.low_vol_count > 10000) {
		led_write_4(1); //LED4を点灯させる
	}
	else {
		led_write_4(0); //LED4を消灯させる
	}
}
