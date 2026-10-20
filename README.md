# MEAM 5100 Lab 3.2 -- Waldo output side (Adafruit ItsyBitsy 32u4)

Code for the **output (slave) side** of the planar two-DOF Waldo of MEAM 5100 *Mechatronics*,
University of Pennsylvania, Fall 2026. Author: Linqi Zhe. Two TowerPro SG90 servos mirror the
master arm's shoulder and elbow potentiometers. The input-side programs (ADC subroutines and the
potentiometer display) are in the separate Lab 3.1 repository `meam5100-lab3-1-waldo-input`.

| File | Program | Report |
|---|---|---|
| `3_2.c` | complete Waldo: potentiometers in, Timer1 servo PWM out, calibrated linear mapping with software limits (pulse clamp + slew-rate limit) | Section 3.2.3 |
| `3_2_4_dance.c` | keyframe choreography player for the dance | Section 3.2.4 |
| `lib/m_usb.c`, `inc/m_usb.h` | course USB-serial library (J. Fiene & J. Romano, MAEVARM) | -- |
| `inc/MEAM_general.h` | course `set()` / `clear()` register macros (adapted by P. Stegall / M. Yim) | -- |

## Pins

| Joint | Master side (input) | Slave side (output) |
|---|---|---|
| 1 -- shoulder | pot 1 wiper on **A0 = PF7 / ADC7** | SG90 #1 signal on **D9 = PB5 / OC1A** |
| 2 -- elbow    | pot 2 wiper on **A1 = PF6 / ADC6** | SG90 #2 signal on **D10 = PB6 / OC1B** |

The servos need their own 5 V, >= 2 A supply with a common ground (two stalled SG90s draw about 1.5 A).

## Build and flash

Requires `avr-gcc`, `avr-libc` and `avrdude` (`brew install avr-gcc avrdude` on macOS). The
`Makefile` is the course Makefile with a `PROG` variable selecting the program:

```sh
make PROG=3_2.c              # default when PROG is not given  -> main.hex
make PROG=3_2_4_dance.c
make PROG=3_2.c flash PORT=/dev/tty.usbmodemXXXX   # double-tap reset, then flash (avr109)
make clean
```

## Calibration (3_2.c)

`SHOULDER_ADC_NEG90/POS90` and `ELBOW_ADC_NEG90/POS90` are the ADC counts printed by `3_1_3.c`
(Lab 3.1 repository) with each master joint at -90 deg and +90 deg (defaults 171 / 853 assume a pot
at mid-travel with 270 deg of travel). `SERVO_SOFT_MIN/MAX` keep the pulse 50 us inside the servo's
mechanical ends; `SLEW_TICKS_PER_FRAME` caps the joint speed at about 225 deg/s.
