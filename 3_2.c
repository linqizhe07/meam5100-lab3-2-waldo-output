/*************************************************************************
 * MEAM 5100 Lab 3.2.3 - Waldo: the slave (output) arm mirrors the master arm
 * Linqi Zhe
 *
 * Planar 2-DOF arm on both sides (3.1.1 design document):
 *   shoulder : pot 1 on PF7/ADC7 (A0)  ->  SG90 #1 on PB5/OC1A (D9)
 *   elbow    : pot 2 on PF6/ADC6 (A1)  ->  SG90 #2 on PB6/OC1B (D10)
 *
 * Every 20 ms servo frame: read both pots (10 bit), map the counts between
 * the two calibrated end positions linearly onto the servo pulse width, apply
 * the software limits -- a pulse clamp that keeps the servo short of its
 * internal stops and a slew-rate limit that caps the acceleration -- and
 * load Timer1's OCR1A/OCR1B.  Timer1 generates the two 50 Hz servo signals
 * in hardware, so the USB debug print cannot disturb them.
 *************************************************************************/
#include "MEAM_general.h"
#include "m_usb.h"

/* ---- joints and pins ---- */
#define SHOULDER_CH   7                 /* PF7 / ADC7 / A0 */
#define ELBOW_CH      6                 /* PF6 / ADC6 / A1 */

/* ---- calibration: ADC counts at -90 deg and +90 deg of each joint.
 *      Defaults assume a pot at mid-travel (512) with the link straight and
 *      270 deg of travel: 512 -/+ (90/270)*1023 = 171 / 853.  Replace them
 *      with the counts printed by 3_1_3.c at the two end positions.       */
#define SHOULDER_ADC_NEG90   171
#define SHOULDER_ADC_POS90   853
#define ELBOW_ADC_NEG90      171
#define ELBOW_ADC_POS90      853

/* ---- Timer1 servo PWM: 16 MHz / 8 = 2 MHz, 1 tick = 0.5 us ---- */
#define SERVO_PERIOD_TICKS  40000       /* 20 ms frame = 50 Hz               */
#define SERVO_PULSE_MIN      1000       /* 0.5 ms: one mechanical end        */
#define SERVO_PULSE_MAX      4000       /* 2.0 ms: other mechanical end      */
#define SERVO_SOFT_MIN       1100       /* software limits: 50 us (~3 deg)   */
#define SERVO_SOFT_MAX       3900       /*   inside the ends, never stalled  */
#define SERVO_CENTRE         3000       /* 1.5 ms                            */
#define SLEW_TICKS_PER_FRAME  150       /* 75 us per frame ~ 4.5 deg/frame   */
                                        /*   -> at most ~225 deg/s           */
#define FRAME_MS               20
#define PRINT_EVERY_N_FRAMES    5       /* debug line at 10 Hz               */

/* ------------- ADC subroutines (identical to 3_1_2.c) ------------------- */
static uint8_t g_adc_channel = 0;

static void adc_disable_digital_input(uint8_t ch)
{
    if (ch <= 7)       { set(DIDR0, ch); }
    else if (ch <= 13) { set(DIDR2, (ch - 8)); }
}

void adc_setup(uint8_t ch)
{
    if (ch > 13) ch = 0;
    g_adc_channel = ch;
    clear(ADMUX, REFS1);  set(ADMUX, REFS0);        /* AVcc reference        */
    clear(ADMUX, ADLAR);                            /* 10-bit, right-adjusted */
    clear(ADMUX, MUX0); clear(ADMUX, MUX1); clear(ADMUX, MUX2); clear(ADMUX, MUX3);
    clear(ADCSRB, MUX5);
    if (ch <= 7) {
        if (ch & 0x01) set(ADMUX, MUX0);
        if (ch & 0x02) set(ADMUX, MUX1);
        if (ch & 0x04) set(ADMUX, MUX2);
    } else {
        uint8_t low = (uint8_t)(ch - 8);
        set(ADCSRB, MUX5);
        if (low & 0x01) set(ADMUX, MUX0);
        if (low & 0x02) set(ADMUX, MUX1);
        if (low & 0x04) set(ADMUX, MUX2);
    }
    adc_disable_digital_input(ch);
    set(ADCSRA, ADPS2); set(ADCSRA, ADPS1); set(ADCSRA, ADPS0);   /* clk/128 */
    clear(ADCSRA, ADATE);
    clear(ADCSRA, ADIE);
    set(ADCSRA, ADEN);
}

uint16_t adc_read(void)
{
    set(ADCSRA, ADSC);
    while (bit_is_set(ADCSRA, ADSC)) { }
    return ADC;
}

uint16_t adc_read_channel(uint8_t ch)
{
    if (ch != g_adc_channel) {
        adc_setup(ch);
        (void)adc_read();                           /* settle after the switch */
    }
    return adc_read();
}

/* ------------- Timer1: fast PWM mode 14, TOP = ICR1, OC1A/OC1B ---------- */
void ServoPWM_setup(void)
{
    set(DDRB, 5);                                   /* PB5 = OC1A, servo 1    */
    set(DDRB, 6);                                   /* PB6 = OC1B, servo 2    */
    set(TCCR1A, WGM11);                             /* WGM13:0 = 1110         */
    set(TCCR1B, WGM12);
    set(TCCR1B, WGM13);
    set(TCCR1A, COM1A1);                            /* non-inverting outputs  */
    set(TCCR1A, COM1B1);
    set(TCCR1B, CS11);                              /* clk/8 -> 2 MHz         */
    ICR1  = SERVO_PERIOD_TICKS;                     /* 20 ms period           */
    OCR1A = SERVO_CENTRE;                           /* both joints centred    */
    OCR1B = SERVO_CENTRE;                           /*   at power-up          */
}

/* ------------- mapping and software limits ------------------------------ */
/* linear map of the calibrated ADC range onto the software pulse range */
static unsigned int joint_to_pulse(uint16_t adc, uint16_t adc_neg90, uint16_t adc_pos90)
{
    unsigned long span = (unsigned long)(adc_pos90 - adc_neg90);
    if (adc < adc_neg90) adc = adc_neg90;           /* clamp to the end stops */
    if (adc > adc_pos90) adc = adc_pos90;
    return SERVO_SOFT_MIN +
           (unsigned int)(((unsigned long)(adc - adc_neg90) *
                           (SERVO_SOFT_MAX - SERVO_SOFT_MIN)) / span);
}

/* move at most SLEW_TICKS_PER_FRAME towards the target per frame */
static unsigned int slew_limit(unsigned int target, unsigned int now)
{
    if (target > now + SLEW_TICKS_PER_FRAME) return now + SLEW_TICKS_PER_FRAME;
    if (target + SLEW_TICKS_PER_FRAME < now) return now - SLEW_TICKS_PER_FRAME;
    return target;
}

int main(void)
{
    uint16_t     adc1, adc2;
    unsigned int pulse1 = SERVO_CENTRE, pulse2 = SERVO_CENTRE;
    uint8_t      frames = 0;

    _clockdivide(0);                                /* 16 MHz                 */
    m_usb_init();                                   /* debug only: the loop   */
                                                    /* runs without a host    */
    adc_setup(SHOULDER_CH);
    ServoPWM_setup();
    _delay_ms(500);                                 /* let the servos centre  */

    for (;;) {
        adc1 = adc_read_channel(SHOULDER_CH);
        adc2 = adc_read_channel(ELBOW_CH);

        pulse1 = slew_limit(joint_to_pulse(adc1, SHOULDER_ADC_NEG90, SHOULDER_ADC_POS90), pulse1);
        pulse2 = slew_limit(joint_to_pulse(adc2, ELBOW_ADC_NEG90,    ELBOW_ADC_POS90),    pulse2);
        OCR1A = pulse1;                             /* taken at the next frame */
        OCR1B = pulse2;

        if (++frames >= PRINT_EVERY_N_FRAMES) {     /* 10 Hz debug line        */
            frames = 0;
            m_usb_tx_string("adc1="); m_usb_tx_uint(adc1);
            m_usb_tx_string(" p1=");  m_usb_tx_uint(pulse1 / 2); m_usb_tx_string("us   ");
            m_usb_tx_string("adc2="); m_usb_tx_uint(adc2);
            m_usb_tx_string(" p2=");  m_usb_tx_uint(pulse2 / 2); m_usb_tx_string("us\r\n");
            m_usb_tx_push();
        }
        _delay_ms(FRAME_MS);
    }
    return 0;
}
