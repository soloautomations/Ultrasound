# Ultrasonic Ranger — PIC16F887

A 40 KHz ultrasonic distance meter for the PIC16F887, written in CCS PIC C.
The transducer is driven straight off a port pin (no MOSFET driver), the echo
comes back through an op-amp / comparator front end, and the reading goes to a
20x4 character LCD and out the serial port.

## Hardware

| Pin | Function |
| --- | --- |
| RA0 | 40 KHz square wave out → TX transducer (direct drive, idles LOW) |
| RA1 | Echo in, from the comparator (active high, idles low) |
| RB0 | Mode select: 0 = ranging, 1 = continuous 40 KHz carrier |
| RC6 / RC7 | RS232 @ 38400 baud, TTL levels (USB-serial adapter goes straight on) |
| RB1, RB2, RC0, RB4-RB7 | 20x4 LCD: RS, RW, E, D4-D7 |

Clock is 20 MHz. The full hardware map, including the receive chain
(LM741 gain stages → band-pass → comparator against the VR1 threshold), is
documented at the top of [Ultrasound.c](Ultrasound.c).

## How it works

- **Carrier** — Timer2 free-runs with `PR2 = 61`, so its flag sets every
  12.4 µs at Fosc/4. Toggling RA0 on each flag gives 40.32 KHz. The flag is
  polled, not serviced by an interrupt: there are only 62 instruction cycles
  between events and CCS interrupt entry/exit alone costs more than that.
- **Ping** — 8 cycles out, then `BLANK_TIME_US` of blanking to ride out
  ring-down and receiver recovery before listening.
- **Timing** — Timer1 at Fosc/4/8 = 1.6 µs per tick. `distance_cm =
  ticks * 4 / 145`, done in `int32` to keep the float library out of an 8K part.
- **Failure modes are reported, not faked** — `---` when nothing comes back
  within 30 ms (~5 m), `RX busy` when the echo line never went idle after
  blanking, which means a saturated front end or VR1 wound down too far.

## Calibration

Two knobs matter on real hardware, both explained inline in the source:

- `BLANK_TIME_US` — minimum range in cm is `BLANK_TIME_US / 58`. If you see
  `RX busy`, walk it up until real readings appear, then back it off.
- `ECHO_OFFSET_TICKS` — fixed lag from the LM741 receive chain. Put a flat
  target at a known distance `D` cm, read what the display says (`R` cm), and
  set this to `(R - D) * 36`.

If the direct-drive output is too weak for the range you need, raise
`BURST_CYCLES` to 12-16 and raise `BLANK_TIME_US` to match.

## Building

CCS PIC C compiler (PICC) 5.019 or later.

    ccsc +FM Ultrasound.c

Or open [Ultrasound.ccspjt](Ultrasound.ccspjt) in the CCS IDE and build.
Target is `Ultrasound.hex` for a PIC16F887. Build output (`.hex`, `.cof`,
`.lst`, and friends) is not tracked.

## Files

- [Ultrasound.c](Ultrasound.c) — the firmware
- [flex_lcd.c](flex_lcd.c) — 4-bit character LCD driver, pins configurable at
  the top of the file
- [Ultrasound.ccspjt](Ultrasound.ccspjt) — CCS IDE project file
