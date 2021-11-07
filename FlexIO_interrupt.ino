// proof of concept of interrupt-driven FlexIO data transmit, with 8 bit parallel bus
// allows single-beat and multi-beat transfers, and adjusts number of beats for final burst if there is a remainder

#include <Arduino.h>
#include "FlexIO_t4.h"
#define SHIFTNUM 4 // number of shifters used (must be 1, 2, 4, or 8)
#define BYTES_PER_BEAT (sizeof(uint8_t))
#define BEATS_PER_SHIFTER (sizeof(uint32_t)/BYTES_PER_BEAT)
#define BYTES_PER_BURST (sizeof(uint32_t)*SHIFTNUM)
#define SHIFTER_IRQ (SHIFTNUM-1)
#define TIMER_IRQ 0
#define FLEXIO_BASE_CLOCK 120000000UL
#define SHIFT_CLOCK_DIVIDER 20
#define FLEXIO_ISR_PRIORITY 64 // interrupt is timing sensitive, so use relatively high priority (supersedes USB)

FlexIOHandler *pFlex;
IMXRT_FLEXIO_t *p;
const FlexIOHandler::FLEXIO_Hardware_t *hw;

uint8_t databuf[32] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, 0x1F, 0x20};

/* variables used by ISR */
volatile uint32_t bytes_remaining;
volatile uint32_t *readPtr;
uint32_t finalBurstBuffer[SHIFTNUM];

void setup() {
    Serial.begin(115200);
    Serial.print(CrashReport);
    Serial.println("Start setup");

    pinMode(14, OUTPUT); // debug
    digitalWriteFast(14, LOW);

    FlexIO2_Init();
}


void loop() {
    transmitAsync(databuf, 32);

    delay(1000);
}

void transmitAsync(void *src, uint32_t bytes) {

    int remainder = bytes % BYTES_PER_BURST;
    if (remainder != 0) {
        memset(finalBurstBuffer, 0, sizeof(finalBurstBuffer));
        memcpy(finalBurstBuffer, (uint8_t*)src + bytes - remainder, remainder);
    }

    bytes_remaining = bytes;
    readPtr = (uint32_t*)src;

    p->TIMSTAT = (1 << TIMER_IRQ); // clear timer interrupt signal

    // enable interrupts to trigger bursts
    p->TIMIEN |= (1 << TIMER_IRQ);
    p->SHIFTSIEN |= (1 << SHIFTER_IRQ);
}

FASTRUN void isr() {
    static volatile bool finalBurst = false;

    if (p->TIMSTAT & (1 << TIMER_IRQ)) { // interrupt from end of burst
        p->TIMSTAT = (1 << TIMER_IRQ); // clear timer interrupt signal
        if (finalBurst) {
            finalBurst = false;
            p->TIMIEN &= ~(1 << TIMER_IRQ); // disable timer interrupt
            transferCompleteCallback();
            return;
        }
    }

    if (p->SHIFTSTAT & (1 << SHIFTER_IRQ)) { // interrupt from empty shifter buffer
        // note, the interrupt signal is cleared automatically when writing data to the shifter buffers
        if (bytes_remaining == 0) { // just started final burst, no data to load
            p->SHIFTSIEN &= ~(1 << SHIFTER_IRQ); // disable shifter interrupt signal
            finalBurst = true;
        } else if (bytes_remaining < BYTES_PER_BURST) { // just started second-to-last burst, load data for final burst
            uint8_t beats = bytes_remaining / BYTES_PER_BEAT;
            p->TIMCMP[0] = ((beats * 2U - 1) << 8) | (SHIFT_CLOCK_DIVIDER / 2U - 1U); // takes effect on final burst
            readPtr = finalBurstBuffer;
            bytes_remaining = 0;
            for (int i = 0; i < SHIFTNUM; i++) {
                p->SHIFTBUF[i] = *readPtr++;
            }
        } else {
            bytes_remaining -= BYTES_PER_BURST;
            for (int i = 0; i < SHIFTNUM; i++) {
                p->SHIFTBUF[i] = *readPtr++;
            }
        }
    }

    asm("dsb");
}

FASTRUN void transferCompleteCallback() {
    digitalWriteFast(14, HIGH);
    delayNanoseconds(80);
    digitalWriteFast(14, LOW);
    //  Serial.println("done");
}

void FlexIO2_Init() {
    /* Get a FlexIO channel */
    pFlex = FlexIOHandler::flexIOHandler_list[1]; // use FlexIO2

    /* Pointer to the port structure in the FlexIO channel */
    p = &pFlex->port();

    /* Pointer to the hardware structure in the FlexIO channel */
    hw = &pFlex->hardware();

    /* Basic pin setup */
    pinMode(10, OUTPUT); // FlexIO2:0 // use as clock
    pinMode(40, OUTPUT); // FlexIO2:4
    pinMode(41, OUTPUT); // FlexIO2:5
    pinMode(42, OUTPUT); // FlexIO2:6
    pinMode(43, OUTPUT); // FlexIO2:7
    pinMode(44, OUTPUT); // FlexIO2:8
    pinMode(45, OUTPUT); // FlexIO2:9
    pinMode(6, OUTPUT); // FlexIO2:10
    pinMode(9, OUTPUT); // FlexIO2:11

    /* High speed and drive strength configuration */
    *(portControlRegister(10)) = 0xFF;
    *(portControlRegister(40)) = 0xFF;
    *(portControlRegister(41)) = 0xFF;
    *(portControlRegister(42)) = 0xFF;
    *(portControlRegister(43)) = 0xFF;
    *(portControlRegister(44)) = 0xFF;
    *(portControlRegister(45)) = 0xFF;
    *(portControlRegister(6)) = 0xFF;
    *(portControlRegister(9)) = 0xFF;

    /* Set clock */
    switch (FLEXIO_BASE_CLOCK) {
        case 480000000UL:
            pFlex->setClockSettings(3, 0, 0); // (480 MHz PLL3_SW_CLK clock, CLK_PRED=1, CLK_PODF=1)
            break;
        case 240000000UL:
            pFlex->setClockSettings(3, 1, 0); // (480 MHz PLL3_SW_CLK clock, CLK_PRED=2, CLK_PODF=1)
            break;
        case 120000000UL:
        default:
            pFlex->setClockSettings(3, 1, 1); // (480 MHz PLL3_SW_CLK clock, CLK_PRED=2, CLK_PODF=2)
            break;
    }
    // Remark: 120 MHz is the maximum FlexIO frequency shown in the reference manual, but testing shows that 240 MHz works.
    // 480 MHz has also been shown to work but some bugs may occur if CPU speed is less than 600 MHz...

    /* Set up pin mux */
    pFlex->setIOPinToFlexMode(10);
    pFlex->setIOPinToFlexMode(40);
    pFlex->setIOPinToFlexMode(41);
    pFlex->setIOPinToFlexMode(42);
    pFlex->setIOPinToFlexMode(43);
    pFlex->setIOPinToFlexMode(44);
    pFlex->setIOPinToFlexMode(45);
    pFlex->setIOPinToFlexMode(6);
    pFlex->setIOPinToFlexMode(9);

    /* Enable the clock */
    hw->clock_gate_register |= hw->clock_gate_mask  ;

    uint8_t beats = SHIFTNUM * BEATS_PER_SHIFTER;                                //Number of beats = number of shifters * beats per shifter
    Serial.printf("Multi Beat Quantity: %d \n", beats);

    /* Disable and reset FlexIO */
    p->CTRL &= ~FLEXIO_CTRL_FLEXEN;

    /* Configure the shifters */
    for (int i = 0; i <= SHIFTNUM - 1; i++)
    {
        p->SHIFTCFG[i] =
            FLEXIO_SHIFTCFG_INSRC * (1U)                                             /* Shifter input from next shifter's output */
            | FLEXIO_SHIFTCFG_SSTOP(0U)                                               /* Shifter stop bit disabled */
            | FLEXIO_SHIFTCFG_SSTART(0U)                                              /* Shifter start bit disabled and loading data on enabled */
            | FLEXIO_SHIFTCFG_PWIDTH(8U - 1U);            /* 8 bit shift width */
    }

    p->SHIFTCTL[0] =
        FLEXIO_SHIFTCTL_TIMSEL(0)                         /* Shifter's assigned timer index */
        | FLEXIO_SHIFTCTL_TIMPOL * (0U)                                            /* Shift on posedge of shift clock */
        | FLEXIO_SHIFTCTL_PINCFG(3U)                                              /* Shifter's pin configured as output */
        | FLEXIO_SHIFTCTL_PINSEL(4)                    /* Shifter's pin start index */
        | FLEXIO_SHIFTCTL_PINPOL * (0U)                                            /* Shifter's pin active high */
        | FLEXIO_SHIFTCTL_SMOD(2U);               /* shifter mode transmit */

    for (int i = 1; i <= SHIFTNUM - 1; i++)
    {
        p->SHIFTCTL[i] =
            FLEXIO_SHIFTCTL_TIMSEL(0)                         /* Shifter's assigned timer index */
            | FLEXIO_SHIFTCTL_TIMPOL * (0U)                                            /* Shift on posedge of shift clock */
            | FLEXIO_SHIFTCTL_PINCFG(0U)                                              /* Shifter's pin configured as output disabled */
            | FLEXIO_SHIFTCTL_PINSEL(4)                    /* Shifter's pin start index */
            | FLEXIO_SHIFTCTL_PINPOL * (0U)                                            /* Shifter's pin active high */
            | FLEXIO_SHIFTCTL_SMOD(2U);               /* shifter mode transmit */
    }

    /* Configure the timer for shift clock */
    p->TIMCMP[0] =
        ((beats * 2U - 1) << 8)                                     /* TIMCMP[15:8] = number of beats x 2 – 1 */
        | (SHIFT_CLOCK_DIVIDER / 2U - 1U);                          /* TIMCMP[7:0] = shift clock divide ratio / 2 - 1 */

    p->TIMCFG[0] =   FLEXIO_TIMCFG_TIMOUT(0U)                                                /* Timer output logic one when enabled and not affected by reset */
                     | FLEXIO_TIMCFG_TIMDEC(0U)                                                /* Timer decrement on FlexIO clock, shift clock equals timer output */
                     | FLEXIO_TIMCFG_TIMRST(0U)                                                /* Timer never reset */
                     | FLEXIO_TIMCFG_TIMDIS(2U)                                                /* Timer disabled on timer compare */
                     | FLEXIO_TIMCFG_TIMENA(2U)                                                /* Timer enabled on trigger high */
                     | FLEXIO_TIMCFG_TSTOP(0U)                                                 /* Timer stop bit disabled */
                     | FLEXIO_TIMCFG_TSTART * (0U);                                            /* Timer start bit disabled */

    p->TIMCTL[0] =
        FLEXIO_TIMCTL_TRGSEL(((SHIFTNUM - 1) << 2) | 1U)                           /* Timer trigger selected as highest shifter's status flag */
        | FLEXIO_TIMCTL_TRGPOL * (1U)                                              /* Timer trigger polarity as active low */
        | FLEXIO_TIMCTL_TRGSRC * (1U)                                              /* Timer trigger source as internal */
        | FLEXIO_TIMCTL_PINCFG(3U)                                                /* Timer' pin configured as output */
        | FLEXIO_TIMCTL_PINSEL(0)                         /* Timer' pin index: WR pin */
        | FLEXIO_TIMCTL_PINPOL * (1U)                                              /* Timer' pin active low */
        | FLEXIO_TIMCTL_TIMOD(1U);                                                 /* Timer mode 8-bit baud counter */

    /* Enable the FlexIO with fast access */
    p->CTRL = FLEXIO_CTRL_FLEXEN | FLEXIO_CTRL_FASTACC;

    // configure interrupts
    attachInterruptVector(IRQ_FLEXIO2, isr);
    NVIC_ENABLE_IRQ(IRQ_FLEXIO2);
    NVIC_SET_PRIORITY(IRQ_FLEXIO2, FLEXIO_ISR_PRIORITY);

    // disable interrupts until later
    p->SHIFTSIEN &= ~(1 << SHIFTER_IRQ);
    p->TIMIEN &= ~(1 << TIMER_IRQ);
}
