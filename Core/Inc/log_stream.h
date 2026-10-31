#ifndef INC_LOG_STREAM_H_
#define INC_LOG_STREAM_H_
#include <stdint.h>
#include <stdbool.h>
void LogStreamStart(uint8_t mode);
void LogStreamTick(uint32_t ms, bool slalom);
void LogStreamEvent(uint32_t ms, uint8_t kind);
void LogStreamWork(void); /* PendSV only. */
void LogStreamStop(void); /* Foreground only, motors stopped. */
bool LogStreamSuppressDebug(void);
#endif
