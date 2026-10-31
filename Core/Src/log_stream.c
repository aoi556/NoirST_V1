#include "global.h"
#include "motor.h"
#include "search.h"
#include "usart.h"
#include "log_stream.h"

#if LOG_LIVE_EXPLORE_MS < 1 || LOG_LIVE_SLALOM_MS < 1 || LOG_LIVE_QUEUE < 2
#error Invalid live log settings
#endif
_Static_assert((LOG_LIVE_QUEUE & (LOG_LIVE_QUEUE - 1U)) == 0,
        "Queue capacity must be a power of two");
typedef struct {
    uint32_t ms;
    uint16_t raw[4];
    int16_t speed[2];
    int8_t x, y;
    uint8_t dir, walls, kind, event;
} LiveSample;
static LiveSample queue[LOG_LIVE_QUEUE];
static char tx[128]; /* Owned by UART until TxCplt; never a stack buffer. */
static volatile uint32_t head, tail, dropped, sent, errors, debug_chars;
static volatile bool enabled, busy;
static volatile bool resync;
static uint32_t due;
static bool last_slalom;

static void Wake(void) { SCB->ICSR = SCB_ICSR_PENDSVSET_Msk; }
static void Drop(void) {
    uint32_t irq = __get_PRIMASK();
    __disable_irq();
    if (dropped != UINT32_MAX) dropped++;
    __set_PRIMASK(irq);
}
bool LogStreamSuppressDebug(void) {
    if (!enabled) return false;
    if (debug_chars != UINT32_MAX) debug_chars++;
    return true;
}
static int16_t Speed(float v) {
    if (v != v) return 0;
    if (v > 32767.0f) return 32767;
    if (v < -32768.0f) return -32768;
    return (int16_t)v;
}
/* Producers are TIM6 and foreground event hooks. The short critical section
 * serialises them, but never includes formatting or UART operations. */
static void Push(uint32_t ms, uint8_t kind, uint8_t event) {
    uint32_t irq = __get_PRIMASK();
    __disable_irq();
    if (enabled) {
        if ((uint32_t)(head - tail) >= LOG_LIVE_QUEUE) Drop();
        else {
            LiveSample *s = &queue[head % LOG_LIVE_QUEUE];
            s->ms = ms;
            s->raw[0] = sensor.ad_l; s->raw[1] = sensor.ad_fl;
            s->raw[2] = sensor.ad_fr; s->raw[3] = sensor.ad_r;
            s->speed[0] = Speed(motor.speed_mm_s_l);
            s->speed[1] = Speed(motor.speed_mm_s_r);
            s->x = mouse.x; s->y = mouse.y; s->dir = mouse.dir;
            s->walls = mouse.section.north | (mouse.section.east << 1) |
                    (mouse.section.south << 2) | (mouse.section.west << 3);
            s->kind = kind; s->event = event;
            __DMB();
            head++;
        }
        Wake();
    }
    __set_PRIMASK(irq);
}
void LogStreamTick(uint32_t ms, bool slalom) {
    if (!enabled) return;
    uint32_t period = slalom ? LOG_LIVE_SLALOM_MS : LOG_LIVE_EXPLORE_MS;
    if (slalom != last_slalom) { due = period; last_slalom = slalom; }
    if (--due == 0) { due = period; Push(ms, slalom ? 'S' : 'E', 0); }
}
void LogStreamEvent(uint32_t ms, uint8_t kind) { Push(ms, 'U', kind); }
static char *Unsigned(char *p, uint32_t value) {
    char digits[10];
    unsigned n = 0;
    do { digits[n++] = '0' + value % 10U; value /= 10U; } while (value);
    while (n) *p++ = digits[--n];
    return p;
}
static char *Column(char *p, int32_t value) {
    *p++ = ',';
    if (value < 0) { *p++ = '-'; value = -value; }
    return Unsigned(p, (uint32_t)value);
}
void LogStreamWork(void) {
    if (!enabled || busy || head == tail) return;
    __DMB();
    LiveSample s = queue[tail % LOG_LIVE_QUEUE];
    __DMB();
    tail++;
    /* One bounded row per invocation; preemptible by all control interrupts. */
    /* Maximum row is 85 bytes (87 with error resynchronisation).
     * Avoid libc formatting in an exception: foreground printf may be
     * interrupted here, and newlib is not configured for reentrant calls. */
    char *p = tx;
    /* Keep recovery pending until the whole replacement row completes.
     * A HAL_BUSY/HAL_ERROR return here transmits no recovery bytes. */
    if (resync) { *p++ = '\r'; *p++ = '\n'; }
    *p++ = s.kind; *p++ = ','; p = Unsigned(p, s.ms);
    for (unsigned i = 0; i < 4; i++) p = Column(p, s.raw[i]);
    for (unsigned i = 0; i < 2; i++) p = Column(p, s.speed[i]);
    p = Column(p, s.x); p = Column(p, s.y);
    p = Column(p, s.dir); p = Column(p, s.walls); p = Column(p, s.event);
    *p++ = ','; p = Unsigned(p, dropped);
    *p++ = '\r'; *p++ = '\n';
    busy = true;
    if (HAL_UART_Transmit_IT(&huart2, (uint8_t *)tx, (uint16_t)(p - tx)) != HAL_OK) {
        busy = false; errors++; Drop();
        if (head != tail) Wake();
    }
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *uart) {
    if (uart == &huart2 && enabled && busy) {
        /* TC callback: the final LF has left the UART, not just TDR. */
        resync = false;
        busy = false;
        sent++;
        if (head != tail) Wake();
    }
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart) {
    if (uart == &huart2 && enabled) {
        /* IT TX only: the HAL abort path just disables TX interrupts. */
        HAL_UART_AbortTransmit(uart);
        if (busy) { Drop(); resync = true; }
        busy = false; errors++;
        if (head != tail) Wake();
    }
}
void LogStreamStop(void) {
    if (!enabled) return;
    /* No producer remains: RunLogStopAndDump disabled capture first.
     * Waiting is restricted to stopped foreground, with a bounded timeout. */
    uint32_t start = HAL_GetTick();
    Wake();
    while ((busy || head != tail) && (uint32_t)(HAL_GetTick() - start) < 100U) {}
    uint32_t irq = __get_PRIMASK();
    __disable_irq();
    /* ErrorCallback may already have aborted the last row and cleared busy.
     * In that case resync still requires a line break before LIVE_END. */
    bool truncated = busy || resync;
    if (busy) { HAL_UART_AbortTransmit(&huart2); Drop(); }
    while (tail != head) { tail++; Drop(); }
    enabled = busy = false;
    resync = false;
    __set_PRIMASK(irq);
    /* Foreground only: __io_putchar expands LF to CRLF and blocking HAL TX
     * drains any byte still in TDR/the shift register after an IT abort. */
    if (truncated) printf("\n");
    printf("# LIVE_END,sent=%lu,dropped=%lu,uart_errors=%lu,suppressed_debug_chars=%lu\n",
            (unsigned long)sent, (unsigned long)dropped, (unsigned long)errors,
            (unsigned long)debug_chars);
}
void LogStreamStart(uint8_t mode) {
    LogStreamStop();
    head = tail = dropped = sent = errors = debug_chars = 0;
    busy = resync = false; last_slalom = mode == 4;
    due = last_slalom ? LOG_LIVE_SLALOM_MS : LOG_LIVE_EXPLORE_MS;
    if (!LOG_LIVE_ENABLED) return;
    HAL_NVIC_SetPriority(PendSV_IRQn, 15, 0);
    printf("# LIVE_BEGIN,mode=%u,explore_ms=%lu,slalom_ms=%lu\n", mode,
            (unsigned long)LOG_LIVE_EXPLORE_MS, (unsigned long)LOG_LIVE_SLALOM_MS);
    printf("kind,time_ms,raw_l,raw_fl,raw_fr,raw_r,command_l_mm_s,command_r_mm_s,x,y,dir,walls,event,dropped\n");
    __DMB();
    enabled = true;
}
