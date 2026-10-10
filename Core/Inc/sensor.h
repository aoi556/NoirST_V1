/*
 * sensor.h
 *
 *  Created on: 2026/06/17
 *      Author: Karasawa
 */

#ifndef INC_SENSOR_H_
#define INC_SENSOR_H_

#include <stdint.h>
#include <stdbool.h>

typedef enum {
	SENSOR_L, SENSOR_FL, SENSOR_FR, SENSOR_R, SENSOR_COUNT
} SensorId;

typedef enum {
	SENSOR_DISTANCE_OK,
	SENSOR_DISTANCE_ABOVE_RAW_RANGE, /* Clamped to 10 mm; not a measured distance. */
	SENSOR_DISTANCE_BELOW_RAW_RANGE, /* Clamped to 90 mm; may also mean no wall. */
	SENSOR_DISTANCE_INVALID
} SensorDistanceStatus;

typedef struct {
	float mm;
	/* Local interpolation sensitivity, NOT an accuracy guarantee.
	 * Zero outside calibration. At knots, uses the nearer-distance interval. */
	float mm_per_count;
	SensorDistanceStatus status;
} SensorDistance;

/* Pure conversion of one raw sample. No ADC access or change to sensor.
 * FL/FR: robot nose to front wall. L/R: supplied calibration reference.
 * Check status before using mm. Invalid ID returns mm=-1. */
SensorDistance SensorRawToDistance(SensorId id, uint32_t raw);
/* Snapshot all four readings, then convert outside the short critical section.
 * Safe from foreground and TIM6; result is indexed by SensorId. */
void SensorReadDistances(SensorDistance result[SENSOR_COUNT]);
typedef struct {
	bool l, f, r;
} SensorWalls;
/* Raw ADC wall detection shared by exploration and LEDs; independent of
 * the 10-90 mm calibration used for distance control. */
SensorWalls SensorReadWalls(void);
/* UART diagnostics; call from foreground only, never an interrupt. */
void SensorPrintDistances(void);

typedef struct {
	uint32_t ad_r, ad_fr, ad_fl, ad_l; //壁センサーのAD値
	uint32_t ad_batt; //バッテリー電圧のAD値
	uint16_t low_vol_count;				//低電圧フラグまでのカウント
	bool wall_r, wall_f, wall_l; //壁センサーの検知フラグ
} SensorData;

extern SensorData sensor;

void GetWallSensorInfo(void);

void SensorUpdate(void);


#endif /* INC_SENSOR_H_ */

