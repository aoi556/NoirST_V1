#include "global.h"
#include "motor.h"
#include "drive.h"
#include "search.h"
#include "run_log.h"
#include "log_stream.h"

#if LOG_EXPLORE_PERIOD_MS < 1 || LOG_SLALOM_PERIOD_MS < 1
#error Log periods must be at least one control tick
#endif
#if LOG_EXPLORE_CAPACITY < 1 || LOG_SLALOM_CAPACITY < 1 || LOG_EVENT_CAPACITY < 1
#error Log capacities must be positive
#endif

/* Fixed point: integer mm/s commands, raw 12-bit ADC, unsigned pulse count
 * modulo 65536 (MotorStart resets it). No allocation or formatting in ISR. */
typedef struct {
    uint32_t ms;
    uint16_t raw[4];
    int16_t speed[2];
} FastSample;
typedef struct {
    FastSample fast;
    uint16_t pulse[2];
    int8_t x, y;
    uint8_t dir, walls;
} ExploreSample;
typedef struct {
    uint32_t ms;
    int8_t x, y;
    uint8_t dir, walls, kind;
} Event;
_Static_assert(sizeof(FastSample) == 16, "Fast sample layout");
_Static_assert(sizeof(ExploreSample) == 24, "Explore sample layout");
_Static_assert(sizeof(Event) == 12, "Event layout");
_Static_assert(sizeof(FastSample) * LOG_SLALOM_CAPACITY <= 4096,
        "Slalom log exceeds CCM RAM");

/* Startup need not initialise CCM: only slots below fast_count are read,
 * and every field of those slots is assigned before publication. */
static FastSample fast_log[LOG_SLALOM_CAPACITY] __attribute__((section(".ccmram")));
static ExploreSample explore_log[LOG_EXPLORE_CAPACITY];
static Event events[LOG_EVENT_CAPACITY];
static volatile bool active;
static volatile uint32_t ticks, explore_count, fast_count, event_count;
static volatile uint32_t explore_dropped, fast_dropped, event_dropped;
static volatile uint32_t explore_due, fast_due;
static uint8_t run_mode;
static volatile bool fast_active;
static volatile bool slalom_turning;
static volatile uint32_t turn_count, fast_start_ms, fast_end_ms;

static uint8_t Walls(void) {
    return mouse.section.north | (mouse.section.east << 1) |
            (mouse.section.south << 2) | (mouse.section.west << 3);
}
static int16_t Speed(float value) {
    if (value != value) return 0;
    if (value > 32767.0f) return 32767;
    if (value < -32768.0f) return -32768;
    return (int16_t)value;
}
static void Capture(FastSample *s) {
    s->ms = ticks;
    s->raw[0] = sensor.ad_l;
    s->raw[1] = sensor.ad_fl;
    s->raw[2] = sensor.ad_fr;
    s->raw[3] = sensor.ad_r;
    s->speed[0] = Speed(motor.speed_mm_s_l);
    s->speed[1] = Speed(motor.speed_mm_s_r);
}
void RunLogStart(uint8_t mode) {
    uint32_t irq = __get_PRIMASK();
    __disable_irq();
    active = false;
    __set_PRIMASK(irq);
    LogStreamStart(mode);
    __disable_irq();
    ticks = explore_count = fast_count = event_count = 0;
    explore_dropped = fast_dropped = event_dropped = 0;
    explore_due = LOG_EXPLORE_PERIOD_MS;
    fast_due = LOG_SLALOM_PERIOD_MS;
    run_mode = mode;
    fast_active = false;
    slalom_turning = false;
    turn_count = fast_start_ms = fast_end_ms = 0;
    active = true;
    __set_PRIMASK(irq);
    RunLogEvent(LOG_POSITION);
    RunLogEvent(LOG_DIRECTION);
}
void RunLogTick(void) {
    if (!active) return;
    ticks++;
    LogStreamTick(ticks, run_mode == 4 || run_mode == 8 || slalom_turning);
    if (--explore_due == 0) {
        explore_due = LOG_EXPLORE_PERIOD_MS;
        if (explore_count < LOG_EXPLORE_CAPACITY) {
            ExploreSample *s = &explore_log[explore_count++];
            Capture(&s->fast);
            s->pulse[0] = motor.pulse_l;
            s->pulse[1] = motor.pulse_r;
            s->x = mouse.x; s->y = mouse.y;
            s->dir = mouse.dir; s->walls = Walls();
        } else if (explore_dropped != UINT32_MAX) explore_dropped++;
    }
    if (fast_active && --fast_due == 0) {
        fast_due = LOG_SLALOM_PERIOD_MS;
        if (fast_count < LOG_SLALOM_CAPACITY) Capture(&fast_log[fast_count++]);
        else if (fast_dropped != UINT32_MAX) fast_dropped++;
    }
}
void RunLogSlalomBegin(void) {
    uint32_t irq = __get_PRIMASK();
    __disable_irq();
    slalom_turning = true;
    if (active && turn_count++ == LOG_SLALOM_TURN_INDEX) {
        fast_active = true;
        fast_due = LOG_SLALOM_PERIOD_MS;
        fast_start_ms = ticks;
    }
    __set_PRIMASK(irq);
}
void RunLogSlalomEnd(void) {
    uint32_t irq = __get_PRIMASK();
    __disable_irq();
    if (fast_active) fast_end_ms = ticks;
    fast_active = false;
    slalom_turning = false;
    __set_PRIMASK(irq);
}
void RunLogEvent(uint8_t kind) {
    uint32_t irq = __get_PRIMASK();
    __disable_irq();
    if (active) {
        LogStreamEvent(ticks, kind);
        if (event_count < LOG_EVENT_CAPACITY) {
            Event *e = &events[event_count++];
            e->ms = ticks; e->x = mouse.x; e->y = mouse.y;
            e->dir = mouse.dir; e->walls = Walls(); e->kind = kind;
        } else if (event_dropped != UINT32_MAX) event_dropped++;
    }
    __set_PRIMASK(irq);
}
static void PrintFast(const char *type, const FastSample *s) {
    printf("%s,%lu,%u,%u,%u,%u,%d,%d", type, (unsigned long)s->ms,
            s->raw[0], s->raw[1], s->raw[2], s->raw[3], s->speed[0], s->speed[1]);
}
void RunLogStopAndDump(void) {
    uint32_t irq = __get_PRIMASK();
    __disable_irq();
    active = false;
    if (fast_active) fast_end_ms = ticks;
    fast_active = false;
    __set_PRIMASK(irq);
    LogStreamStop();
    printf("# RUN_LOG,mode=%u,duration_ms=%lu,explore_period_ms=%lu,slalom_period_ms=%lu\n",
            run_mode, (unsigned long)ticks, (unsigned long)LOG_EXPLORE_PERIOD_MS,
            (unsigned long)LOG_SLALOM_PERIOD_MS);
    printf("# dropped_explore=%lu,dropped_slalom=%lu,dropped_event=%lu\n",
            (unsigned long)explore_dropped, (unsigned long)fast_dropped, (unsigned long)event_dropped);
    printf("# selected_turn=%lu,turns_seen=%lu,turn_start_ms=%lu,turn_end_ms=%lu\n",
            (unsigned long)LOG_SLALOM_TURN_INDEX, (unsigned long)turn_count,
            (unsigned long)fast_start_ms, (unsigned long)fast_end_ms);
    printf("type,time_ms,raw_l,raw_fl,raw_fr,raw_r,command_l_mm_s,command_r_mm_s,pulse_l,pulse_r,x,y,dir,walls,event\n");
    for (uint32_t i = 0; i < explore_count; i++) {
        const ExploreSample *s = &explore_log[i];
        PrintFast("explore", &s->fast);
        printf(",%u,%u,%d,%d,%u,%u,0\n", s->pulse[0], s->pulse[1], s->x, s->y, s->dir, s->walls);
    }
    for (uint32_t i = 0; i < fast_count; i++) {
        PrintFast("slalom", &fast_log[i]);
        printf(",,,,,,,\n");
    }
    for (uint32_t i = 0; i < event_count; i++) {
        const Event *e = &events[i];
        printf("event,%lu,,,,,,,,,%d,%d,%u,%u,%u\n", (unsigned long)e->ms,
                e->x, e->y, e->dir, e->walls, e->kind);
    }
    printf("# END_RUN_LOG\n");
}
