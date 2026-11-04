#include "hal/io.h"
#include "board.h"

void io_init_safe(void)
{
    /* Drive low before switching to output so no pin ever glitches high.
     * Injectors and coils are active-high logic into low-side drivers:
     * low = injector closed, coil not dwelling. */
    INJ_1_3_PORT &= (uint8_t)~INJ_1_3_MASK;  INJ_1_3_DDR |= INJ_1_3_MASK;
    INJ_4_6_PORT &= (uint8_t)~INJ_4_6_MASK;  INJ_4_6_DDR |= INJ_4_6_MASK;
    IGN_1_3_PORT &= (uint8_t)~IGN_1_3_MASK;  IGN_1_3_DDR |= IGN_1_3_MASK;
    IGN_4_6_PORT &= (uint8_t)~IGN_4_6_MASK;  IGN_4_6_DDR |= IGN_4_6_MASK;

    IAC_OPEN_PORT  &= (uint8_t)~(1 << IAC_OPEN_BIT);  IAC_OPEN_DDR  |= (1 << IAC_OPEN_BIT);
    IAC_CLOSE_PORT &= (uint8_t)~(1 << IAC_CLOSE_BIT); IAC_CLOSE_DDR |= (1 << IAC_CLOSE_BIT);

    AUX_PORT = 0x00;  AUX_DDR = 0xFF;
    AUX2_PORT &= (uint8_t)~((1 << AUX2_PURGE) | (1 << AUX2_TI_OUT) | (1 << AUX2_KNOCK_HOLD));
    AUX2_DDR  |= (1 << AUX2_PURGE) | (1 << AUX2_TI_OUT) | (1 << AUX2_KNOCK_HOLD);

    /* Knock IC chip select idles high (inactive). PB0 must be an output for
     * the SPI peripheral to stay in master mode. */
    KNOCK_CS_PORT |= (1 << KNOCK_CS_BIT);
    KNOCK_CS_DDR  |= (1 << KNOCK_CS_BIT);

    /* Inputs: no internal pull-ups, signals are conditioned on the board. */
    TRIG_DDR  &= (uint8_t)~((1 << CRANK_BIT) | (1 << CAM_BIT));
    TRIG_PORT &= (uint8_t)~((1 << CRANK_BIT) | (1 << CAM_BIT));
    VSS_DDR   &= (uint8_t)~(1 << VSS_BIT);

    /* Disable digital input buffers on analogue pins (less noise, less power). */
    DIDR0 = 0xFF;                       /* ADC0..ADC7 */
    DIDR2 = (1 << ADC8D) | (1 << ADC9D);
}
