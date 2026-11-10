#include "knock/knock.h"
#include "knock/knock_core.h"

#include <avr/interrupt.h>
#include <avr/io.h>
#include <string.h>
#include <util/atomic.h>

#include "board.h"
#include "status.h"

/*
 * ---- TPIC8101 command encoding --------------------------------------------
 * VERIFY AGAINST THE TI DATASHEET BEFORE FIRST USE. These follow the command
 * layout commonly used for the TPIC8101 (8-bit commands, one per chip-select
 * frame, SPI mode 1, MSB first): the top bits select the register, the low
 * bits carry the code chosen in the tune (knockPrescaler/Band/Gain/IntTime,
 * see the datasheet tables). The 10-bit integrator result is clocked out on
 * SO during the two frames after INT/HOLD goes low; its byte order is the
 * tune setting knockReadOrder. Everything chip-specific is in this block.
 */
#define TPIC_PRESCALER(p) ((uint8_t)(0x40u | (((p) & 0x1Fu) << 1)))   /* bit 0 = 0: SO active */
#define TPIC_BAND(b)      ((uint8_t)(0x00u | ((b) & 0x3Fu)))
#define TPIC_GAIN(g)      ((uint8_t)(0x80u | ((g) & 0x3Fu)))
#define TPIC_INTTIME(t)   ((uint8_t)(0xC0u | ((t) & 0x1Fu)))
#define TPIC_CHANNEL(ch)  ((uint8_t)(0xE0u | ((ch) & 0x01u)))

/* SPI: mode 1 (CPOL 0, CPHA 1), master, f_cpu / 16 = 1 MHz, interrupt driven */
#define SPI_SCK  PB1
#define SPI_MOSI PB2

uint8_t knock_act[120];                 /* window table, read by the crank ISR */
volatile uint8_t knock_pending;         /* cylinder + 1 whose window just closed (| 0x80: not integrated) */
static volatile uint8_t chip_ch = 0xFF; /* sensor the chip is switched to (0xFF unknown) */
static volatile uint8_t armed;          /* windows run (enabled, configured, full sync) */

static knock_t kn;

/* SPI exchange (ISR side) */
enum { XFER_CONFIG = 0, XFER_RESULT = 1 };
static volatile uint8_t tx[6], rx[6], xn, xi, xbusy, xkind, xcyl;
static volatile uint8_t res_ready, res_cyl, res_b0, res_b1;
static volatile uint16_t res_skipped;

/* main-loop state */
static uint8_t configured;
static uint8_t cfg_sig[4];
static uint16_t win_tdc;
static uint8_t win_start, win_len, win_valid;
static uint8_t was_enabled;

static inline void hold_low(void)  { AUX2_PORT &= (uint8_t)~(1 << AUX2_KNOCK_HOLD); }
static inline void hold_high(void) { AUX2_PORT |= (1 << AUX2_KNOCK_HOLD); }
static inline void cs_low(void)    { KNOCK_CS_PORT &= (uint8_t)~(1 << KNOCK_CS_BIT); }
static inline void cs_high(void)   { KNOCK_CS_PORT |= (1 << KNOCK_CS_BIT); }

/* Start an exchange of n bytes from tx[]. Interrupts off or from an ISR. */
static void xfer_start(uint8_t kind, uint8_t n, uint8_t cyl)
{
    xbusy = 1;
    xkind = kind;
    xcyl = cyl;
    xn = n;
    xi = 0;
    cs_low();
    SPDR = tx[0];
}

ISR(SPI_STC_vect)
{
    cs_high();                          /* one command per chip-select frame */
    rx[xi] = SPDR;
    xi++;
    if (xi < xn) {
        cs_low();
        SPDR = tx[xi];
        return;
    }
    xbusy = 0;
    if (xkind == XFER_RESULT) {
        res_b0 = rx[0];
        res_b1 = rx[1];
        res_cyl = xcyl;
        res_ready = 1;
    }
}

/* Crank ISR: a tooth with a window edge, or the tooth after a close. */
void knock_tooth_events(uint8_t a)
{
    if (knock_pending) {
        uint8_t p = knock_pending;
        uint8_t cyl = (uint8_t)((p & 0x7Fu) - 1u);
        knock_pending = 0;
        if (!xbusy) {
            /* switch to the next firing cylinder's sensor; the bytes clocked
             * back are this window's level (thrown away if the window was
             * skipped because the chip was on the other sensor) */
            uint8_t ch = knock_channel(knock_next_cyl[cyl]);
            tx[0] = TPIC_CHANNEL(ch);
            tx[1] = TPIC_GAIN(tu_knockGain());
            xfer_start((p & 0x80u) ? XFER_CONFIG : XFER_RESULT, 2, cyl);
            chip_ch = ch;
        } else {
            res_skipped++;
        }
    }
    if (!armed) {
        hold_low();
        return;
    }
    if (a & KNOCK_OPEN) {
        /* integrate, unless the chip listens to the wrong sensor (first
         * window after arming, or a skipped channel switch) */
        if (chip_ch == knock_channel(a & KNOCK_CYL)) hold_high();
    } else if (a & KNOCK_CLOSE) {
        uint8_t integrated = (AUX2_PORT & (1 << AUX2_KNOCK_HOLD)) != 0;
        hold_low();                     /* hold: the chip digitises the window */
        knock_pending = (uint8_t)(((a & KNOCK_CYL) + 1u) | (integrated ? 0u : 0x80u));
    }
}

/* Sync lost or degraded: close any window, forget a pending read. */
void knock_stop(void)
{
    hold_low();
    knock_pending = 0;
}

void knock_init(void)
{
    knock_reset(&kn);
    memset(knock_act, 0, sizeof knock_act);
    hold_low();
    cs_high();
    DDRB |= (1 << SPI_SCK) | (1 << SPI_MOSI) | (1 << KNOCK_CS_BIT);
    SPCR = (1 << SPIE) | (1 << SPE) | (1 << MSTR) | (1 << CPHA) | (1 << SPR0);
    (void)SPSR;
    (void)SPDR;
}

void knock_poll(void)
{
    uint8_t ready, cyl = 0, b0 = 0, b1 = 0;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        ready = res_ready;
        if (ready) {
            cyl = res_cyl;
            b0 = res_b0;
            b1 = res_b1;
            res_ready = 0;
        }
    }
    if (ready) {
        knock_sample(&kn, cyl, knock_parse(b0, b1, tu_knockReadOrder()), (kn.state & KNOCK_ST_ACTIVE) != 0,
                     rt.rpm);
    }
}

void knock_update(uint16_t rpm, uint16_t air_mg, int16_t clt_x10, uint8_t full_sync, uint8_t cranking)
{
    uint8_t enabled = tu_knockEnable();

    /* chip configuration: at enable and whenever a chip setting changes */
    uint8_t sig[4] = { tu_knockPrescaler(), tu_knockBand(), tu_knockGain(), tu_knockIntTime() };
    if (!enabled) {
        configured = 0;
    } else if (!configured || memcmp(sig, cfg_sig, sizeof sig) != 0) {
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            if (!xbusy && !(AUX2_PORT & (1 << AUX2_KNOCK_HOLD)) && !knock_pending) {
                tx[0] = TPIC_PRESCALER(sig[0]);
                tx[1] = TPIC_BAND(sig[1]);
                tx[2] = TPIC_GAIN(sig[2]);
                tx[3] = TPIC_INTTIME(sig[3]);
                tx[4] = TPIC_CHANNEL(0);
                xfer_start(XFER_CONFIG, 5, 0);
                chip_ch = 0;
                memcpy(cfg_sig, sig, sizeof sig);
                configured = 1;
            }
        }
    }

    /* window table: rebuild when the angles change */
    uint16_t tdc = tu_tdcOffset();
    if (!win_valid || tdc != win_tdc || tu_knockWinStart() != win_start || tu_knockWinLen() != win_len) {
        uint8_t act[120];
        knock_build_windows(act, tdc, tu_knockWinStart(), tu_knockWinLen());
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            memcpy(knock_act, act, sizeof act);
            knock_stop();
        }
        win_tdc = tdc;
        win_start = tu_knockWinStart();
        win_len = tu_knockWinLen();
        win_valid = 1;
    }

    uint8_t run = enabled && configured && full_sync;
    if (!run) {
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            armed = 0;
            knock_stop();
        }
    } else {
        armed = 1;
    }

    uint8_t active = run && !cranking && rpm >= (uint16_t)tu_knockMinRpm() * 100u &&
                     rpm <= (uint16_t)tu_knockMaxRpm() * 100u && air_mg >= tu_knockMinLoad() &&
                     clt_x10 >= ((int16_t)tu_knockMinClt() - 40) * 10;
    if (enabled && !was_enabled) knock_reset(&kn);        /* fresh noise learning */
    was_enabled = enabled;
    knock_tick(&kn, enabled, active);

    for (uint8_t c = 0; c < 6; c++) {
        rt.knockLevel[c] = kn.level[c];
        rt.knockNoise[c] = (uint16_t)(kn.noise16[c] >> 4);
        rt.knockRetard[c] = knock_retard(&kn, c);
    }
    rt.knockCount = kn.count;
    rt.knockState = kn.state;
}

uint8_t knock_get_retard(uint8_t c)
{
    return knock_retard(&kn, c);
}
