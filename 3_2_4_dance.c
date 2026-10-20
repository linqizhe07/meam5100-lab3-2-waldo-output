/*************************************************************************
 * MEAM 5100 Lab 3.2.4 - "Do a dance!": a 10 s choreography for the slave arm
 * Linqi Zhe
 *
 * Same Timer1 servo set-up and software limits as 3_2.c (shoulder on
 * PB5/OC1A, elbow on PB6/OC1B); the potentiometers are not read.  Keyframes
 * are (time, shoulder angle, elbow angle) in degrees relative to the straight
 * arm, and the program interpolates linearly between them every 20 ms frame,
 * so the arm moves smoothly from pose to pose.  Loops with a 2 s pause.
 *************************************************************************/
#include "MEAM_general.h"

#define SERVO_PERIOD_TICKS  40000       /* 20 ms frame = 50 Hz            */
#define SERVO_SOFT_MIN       1100       /* software limits as in 3_2.c    */
#define SERVO_SOFT_MAX       3900
#define SERVO_CENTRE         3000
#define FRAME_MS               20

typedef struct {
    unsigned int t_ms;                  /* time of the keyframe            */
    int          shoulder_deg;          /* -90..+90                        */
    int          elbow_deg;             /* -90..+90                        */
} keyframe_t;

/* 10 s at 120 BPM (one beat = 500 ms): wave, reach, fold, big sweep, bow */
static const keyframe_t dance[] = {
    {     0,   0,   0 },
    {   500,   0,  60 }, {  1000,   0, -60 }, {  1500,   0,  60 }, {  2000,   0, -60 },
    {  2500,  60,   0 }, {  3000, -60,   0 }, {  3500,  60,   0 }, {  4000, -60,   0 },
    {  4500,  45, -80 }, {  5000, -45,  80 }, {  5500,  45, -80 }, {  6000, -45,  80 },
    {  6500,   0,   0 }, {  7000,  85,  85 }, {  7500, -85, -85 }, {  8000,  85,  85 },
    {  8500,   0,   0 }, {  9000,  30,  80 }, {  9500, -30,  80 }, { 10000,   0,   0 },
};
#define N_KEYS ((unsigned char)(sizeof(dance) / sizeof(dance[0])))

void ServoPWM_setup(void)
{
    set(DDRB, 5);  set(DDRB, 6);
    set(TCCR1A, WGM11);  set(TCCR1B, WGM12);  set(TCCR1B, WGM13);   /* mode 14 */
    set(TCCR1A, COM1A1); set(TCCR1A, COM1B1);                       /* non-inv */
    set(TCCR1B, CS11);                                              /* clk/8   */
    ICR1  = SERVO_PERIOD_TICKS;
    OCR1A = SERVO_CENTRE;
    OCR1B = SERVO_CENTRE;
}

/* -90..+90 deg -> pulse ticks inside the software limits */
static unsigned int deg_to_pulse(int deg)
{
    if (deg < -90) deg = -90;
    if (deg >  90) deg =  90;
    return SERVO_SOFT_MIN +
           (unsigned int)(((long)(deg + 90) * (SERVO_SOFT_MAX - SERVO_SOFT_MIN)) / 180L);
}

/* linear interpolation between two keyframe angles at time t */
static int lerp_deg(int a, int b, unsigned int t0, unsigned int t1, unsigned int t)
{
    if (t1 <= t0 || t >= t1) return b;
    if (t <= t0)             return a;
    return (int)((long)a + ((long)b - (long)a) * (long)(t - t0) / (long)(t1 - t0));
}

int main(void)
{
    unsigned int  t;
    unsigned char k, kn;

    _clockdivide(0);
    ServoPWM_setup();
    _delay_ms(1000);                    /* settle straight before the music */

    for (;;) {
        for (t = 0, k = 0; t <= dance[N_KEYS - 1].t_ms; t += FRAME_MS) {
            while (k + 1 < N_KEYS && t >= dance[k + 1].t_ms) k++;
            kn = (k + 1 < N_KEYS) ? (unsigned char)(k + 1) : k;
            OCR1A = deg_to_pulse(lerp_deg(dance[k].shoulder_deg, dance[kn].shoulder_deg,
                                          dance[k].t_ms, dance[kn].t_ms, t));
            OCR1B = deg_to_pulse(lerp_deg(dance[k].elbow_deg, dance[kn].elbow_deg,
                                          dance[k].t_ms, dance[kn].t_ms, t));
            _delay_ms(FRAME_MS);
        }
        _delay_ms(2000);
    }
    return 0;
}
