/* Host test of the 60-2 + cam pattern table. */
#include <stdio.h>
#include "pattern.h"

static int failures, checks;
#define CHECK(c) do { checks++; if (!(c)) { failures++; printf("FAIL line %d: %s\n", __LINE__, #c); } } while (0)

int main(void)
{
    /* Crank: 58 teeth per revolution, 116 per cycle, gap after tooth 57. */
    int rising = 0, prev = 0, longest_low = 0, low = 0;
    for (unsigned i = 0; i < 2 * STIM_STEPS; i++) {        /* two cycles to cover wrap */
        int c = stim_pattern((uint8_t)(i % STIM_STEPS), CAM_NONE, 0) & STIM_OUT_CRANK;
        if (i < STIM_STEPS && c && !prev) rising++;
        low = c ? 0 : low + 1;
        if (low > longest_low) longest_low = low;
        prev = c;
    }
    CHECK(rising == 116);
    CHECK(longest_low == 5);            /* 1 normal low + 2 missing teeth (4 steps) */
    CHECK(stim_pattern(0, CAM_NONE, 0) & STIM_OUT_CRANK);           /* tooth 0 */
    CHECK(!(stim_pattern(116, CAM_NONE, 0) & STIM_OUT_CRANK));      /* tooth 58 missing */
    CHECK(stim_pattern(120, CAM_NONE, 0) & STIM_OUT_CRANK);         /* rev 2 tooth 0 */

    /* Cam pulse: exactly 2 half-steps high, at cam_start, wrapping. */
    int hi = 0;
    for (unsigned i = 0; i < STIM_STEPS; i++) hi += !!(stim_pattern((uint8_t)i, CAM_PULSE, 239) & STIM_OUT_CAM);
    CHECK(hi == 2);
    CHECK(stim_pattern(239, CAM_PULSE, 239) & STIM_OUT_CAM);
    CHECK(stim_pattern(0, CAM_PULSE, 239) & STIM_OUT_CAM);

    /* Half-moon: 120 steps (360° crank) high. */
    hi = 0;
    for (unsigned i = 0; i < STIM_STEPS; i++) hi += !!(stim_pattern((uint8_t)i, CAM_HALFMOON, 40) & STIM_OUT_CAM);
    CHECK(hi == 120);
    CHECK(stim_pattern(40, CAM_HALFMOON, 40) & STIM_OUT_CAM);
    CHECK(!(stim_pattern(160, CAM_HALFMOON, 40) & STIM_OUT_CAM));

    CHECK(STIM_TICKS_PER_STEP(1000) == 1000);   /* 3° at 1000 rpm = 500 us */

    printf("pattern: %d checks, %d failures\n", checks, failures);
    return failures ? 1 : 0;
}
