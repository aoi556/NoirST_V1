#ifndef INC_RUN_LOG_H_
#define INC_RUN_LOG_H_
#include <stdint.h>
enum { LOG_POSITION = 1, LOG_DIRECTION = 2, LOG_WALLS = 3 };
void RunLogStart(uint8_t mode);
void RunLogTick(void); /* TIM6 only, after SensorUpdate; never transmits. */
void RunLogEvent(uint8_t kind); /* Foreground only. */
void RunLogSlalomBegin(void);
void RunLogSlalomEnd(void);
void RunLogStopAndDump(void); /* Foreground, after motors are disabled. */
#endif
