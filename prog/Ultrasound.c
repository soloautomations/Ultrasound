#include <16F887.h>

#use delay(clock=20000000)
#fuses NOWDT, HS, PUT, MCLR, PROTECT, NODEBUG, NOBROWNOUT, NOLVP, CPD, NOWRT, NOFCMEN, NOIESO
#use rs232(baud=38400, xmit=PIN_C6, rcv=PIN_C7)

#include "flex_lcd.c"

/*===========================================================================
  Hardware map
  ---------------------------------------------------------------------------
  RA0   40KHz square wave out -> ultrasound TX directly. No MOSFET driver.
        Q1 (IRF9540) and its gate pull-up R1 are gone from the schematic;
        the transducer now hangs on P2, pin 2 to RA0 and pin 1 to GND,
        with TP2 on RA0 for scoping.
        ACTIVE HIGH: the pin is the driver, so it must idle LOW. Parking it
        high leaves the whole 5V standing across the transducer between
        pings and turns every burst boundary into a step that the receive
        chain then has to recover from.
        Direct drive is only ~5V p-p where the switched rail was several
        times that, so both the acoustic output and the usable range are
        well down - see the note on BURST_CYCLES.
  RA1   Echo input. Chain on VI.SchDoc is
          P5 (RX transducer) -> R14/C22 -> U6 inverting amp
          -> R15/C19/C23 band-pass around U7
          -> SIGNAL -> U5 pin 3 (+), compared against the VR1 threshold
             on pin 2 (-), R7 pulling that node up from +5V
          -> U5 pin 6 -> R9 1K -> RA1, clamped by Z1 (4.7V to GND).
        ACTIVE HIGH, idle LOW: U6 and U7 are DC-coupled inverting stages
        with their + inputs at GND, so SIGNAL sits at 0V between pings.
        With VR1 set to a positive threshold U5 rests at its negative
        rail, Z1 conducts forward and RA1 sits near -0.7V = a logic low.
        An echo above threshold slams U5 positive, Z1 clamps it to 4.7V
        and RA1 reads high.
        If VR1 is wound down to the GND end the threshold goes to zero,
        U5 idles high or chatters on its own noise, and every reading
        comes back as the blanking floor - see BLANK_TIME_US.
        There is no envelope detector in this chain, so during an echo
        RA1 is a 40KHz square burst, not one long pulse. Only the first
        rising edge is timed, which is what ranging wants anyway.
  RB0   Mode select:  0 = HC-SR04 style ranging
                      1 = continuous 40KHz carrier, no ranging
        S1 is an SPDT between GND and +5V through R4, so the level is
        always driven - no pull resistor is needed on this board.
  C6/C7 RS232 @ 38400, TTL levels straight to the P1 header (no MAX232),
        so a USB-serial adapter goes on P1 as-is. Distance is echoed
        here as well as to the LCD.
  P3    20x4 character LCD: RB1=RS, RB2=RW, RC0=E, RB4-RB7=D4-D7.
        Twenty columns, not sixteen - every field written to it has to
        be padded to 20 or the tail of the previous screen survives.
===========================================================================*/

#define TX_PIN     PIN_A0
#define ECHO_PIN   PIN_A1
#define MODE_PIN   PIN_B0

// Direct drive: the pin swings the transducer, so high = drive, low = idle.
#define TX_ON()    output_high(TX_PIN)
#define TX_OFF()   output_low(TX_PIN)

/*===========================================================================
  40KHz carrier
  ---------------------------------------------------------------------------
  Timer2 free-runs with PR2 = 61, so at Fosc/4 (5MHz) its flag sets every
  62 cycles = 12.4us. Toggling RA0 on each flag gives a 40.32KHz square
  wave - the closest a 20MHz clock gets to 40KHz, and well inside any
  transducer bandwidth. (PR2 = 62 would give 39.68KHz if the transducer
  happens to peak below 40KHz.)

  The flag is POLLED rather than serviced by an interrupt on purpose: at
  80K toggles/sec there are only 62 instruction cycles between events, and
  CCS interrupt entry/exit alone costs more than that. Polling leaves the
  timing exact and the loop body trivial.
===========================================================================*/
#define PR2_40KHZ       61
#define BURST_CYCLES    8      // Cycles per ping - same as an HC-SR04. That
                               // number assumes an HC-SR04 drive level; a
                               // pin driving the transducer on its own puts
                               // out far less, so a longer burst (12-16) is
                               // the cheapest way to put the energy back.
                               // It costs ring-down, so raise BLANK_TIME_US
                               // to match if you lengthen it.
#define BLANK_TIME_US   58     // Transducer ring-down plus receiver recovery;
                               // listening before this has expired just
                               // measures our own ping.
                               // Minimum range in cm is simply
                               // BLANK_TIME_US / 58, so 600us = 10cm.
                               // A reading pinned at exactly 10cm is
                               // therefore not a measurement at all - it
                               // means the echo line was still asserted the
                               // moment blanking ended, which is why that
                               // case is now rejected instead of reported.
                               // 600us is an HC-SR04 number and is probably
                               // optimistic here: U6 and U7 are LM741s and
                               // are driven hard into the rails by the TX
                               // burst, and a 741 comes out of saturation
                               // slowly. If the RX busy message is what you
                               // see, walk this up (1500us = 26cm minimum,
                               // 3000us = 52cm) until real readings appear,
                               // then back it off.

/*===========================================================================
  Echo timing
  ---------------------------------------------------------------------------
  Timer1 runs at Fosc/4/8 = 625KHz -> 1.6us per tick.
  Round trip: distance(cm) = time_us / 58   (speed of sound ~343 m/s)
              time_us      = ticks * 1.6
  so distance = ticks * 1.6 / 58 = ticks * 4 / 145, done in int32 to keep
  the float library out of a 8K part.
===========================================================================*/
#define ECHO_TIMEOUT_TICKS 18750u   // 30ms -> a little over 5 metres
#define NO_ECHO            0        // Nothing came back
#define RX_BUSY            0xFFFF   // Echo line never went idle - saturated
                                    // front end, or VR1 wound down so far
                                    // that U5 chatters. Not a distance.
#define ECHO_OFFSET_TICKS  0        // Fixed lag from the receive chain, in
                                    // 1.6us ticks, subtracted from every
                                    // reading. It is not zero on this board:
                                    // U5/U6/U7 are LM741s, whose 0.5V/us
                                    // slew rate cannot square up a 40KHz
                                    // signal, so the comparator edge arrives
                                    // late by a fixed amount. Calibrate it
                                    // once: put a flat target at a measured
                                    // distance D cm, read what the display
                                    // says (R cm), and set this to
                                    // (R - D) * 58 / 1.6, ie (R - D) * 36.

/*--------------------------------------------------------------------------
  Emit n half-cycles (n/2 full 40KHz cycles) on TX_PIN, then park it off.
  Starting from the off state means the first toggle is the leading edge of
  the drive, so every burst has the same phase. n is even, so the pin lands
  back on the idle level by itself and the closing TX_OFF() adds no extra
  edge for the receiver to ring on.
--------------------------------------------------------------------------*/
void tx_half_cycles(int16 n)
{
   TX_OFF();
   clear_interrupt(INT_TIMER2);   // Start on a full-width half cycle

   while(n--)
   {
      while(!interrupt_active(INT_TIMER2)) ;
      clear_interrupt(INT_TIMER2);
      output_toggle(TX_PIN);
   }
   TX_OFF();
}

/*--------------------------------------------------------------------------
  Run the carrier for as long as RB0 stays high. Reading the pin costs a
  handful of cycles, which still fits comfortably in the 62-cycle window.
--------------------------------------------------------------------------*/
void tx_while_mode_high(void)
{
   TX_OFF();
   clear_interrupt(INT_TIMER2);

   while(input(MODE_PIN))
   {
      while(!interrupt_active(INT_TIMER2)) ;
      clear_interrupt(INT_TIMER2);
      output_toggle(TX_PIN);
   }
   TX_OFF();   // Leave the transducer unbiased, not sitting at 5V
}

/*--------------------------------------------------------------------------
  One ping. Returns distance in cm, NO_ECHO if nothing came back, or
  RX_BUSY if the echo line was still asserted when the blanking window
  closed - in that state the next edge we would time is our own ping, and
  timing it yields the blanking floor and nothing else.
--------------------------------------------------------------------------*/
int16 measure_distance_cm(void)
{
   int16 ticks;

   tx_half_cycles(BURST_CYCLES * 2);   // 2 toggles per cycle
   delay_us(BLANK_TIME_US);             // Ignore our own ring-down
   set_timer1(0);                       // Clock starts at end of burst 
   // The line has to be seen idle before a rising edge on it means
   // anything. Blanking is a guess at the recovery time; this is the check.
   while(input(ECHO_PIN))
   {
      if(get_timer1() > ECHO_TIMEOUT_TICKS)
         return RX_BUSY;
   }

   while(!input(ECHO_PIN))
   {
      if(get_timer1() > ECHO_TIMEOUT_TICKS)
         return NO_ECHO;
   }

   ticks = get_timer1();

#if ECHO_OFFSET_TICKS > 0
   ticks = (ticks > ECHO_OFFSET_TICKS) ? (ticks - ECHO_OFFSET_TICKS) : 0;
#endif

   return (int16)(((int32)ticks * 4) / 145);
}

/*--------------------------------------------------------------------------
  LCD
--------------------------------------------------------------------------*/
void lcd_header(void)
{
   lcd_gotoxy(1,1);
   printf(lcd_putc, "Dist =              ");
   lcd_gotoxy(1,2);
   printf(lcd_putc, "SASTHA'S            ");
}

// Column 8 onwards, 8 characters wide. Every branch pads to that same
// width so a short reading cannot leave the tail of a longer one behind.
void show_distance(int16 dist_cm)
{
   lcd_gotoxy(8,1);
   if(dist_cm == NO_ECHO)
      printf(lcd_putc, "---     ");
   else if(dist_cm == RX_BUSY)
      printf(lcd_putc, "RX busy ");
   else
      printf(lcd_putc, "%3Lu cm  ", dist_cm+1);
}

/*--------------------------------------------------------*/
void main()
{
   int16 distance;

   setup_adc_ports(NO_ANALOGS);        // RA0/RA1/RB0 as plain digital I/O
   setup_adc(ADC_OFF);

   // Idle the transmitter as early as possible - but only after the line
   // above, since RA0 powers up as an analog pin and cannot drive a valid
   // logic level until then. RA0 is high-Z out of reset, so the transducer
   // sees no drive during that window either way.
   TX_OFF();

   setup_timer_1(T1_INTERNAL | T1_DIV_BY_8);      // 1.6us tick, echo timing
   setup_timer_2(T2_DIV_BY_1, PR2_40KHZ, 1);      // 12.4us tick, 40KHz carrier

   delay_ms(500);   /* LCD Power-up Delay */

   lcd_init();                //Initialize LCD Module
   lcd_setcursor_vb(0, 0);    //Set cursor OFF

   lcd_header();

   while(TRUE)
   {
      if(input(MODE_PIN))
      {
         /*---------- RB0 = 1 : continuous 40KHz, no measurement ----------*/
         lcd_gotoxy(1,1);
         printf(lcd_putc, "40KHz TX ON         ");
         lcd_gotoxy(1,2);
         printf(lcd_putc, "No Measurement      ");
         printf("40KHz continuous TX mode\r\n");

         tx_while_mode_high();          // Returns when RB0 goes back low

         lcd_header();                  // Restore the ranging screen
      }
      else
      {
         /*---------- RB0 = 0 : HC-SR04 style ranging ----------*/
         distance = measure_distance_cm();

         show_distance(distance);

         if(distance == NO_ECHO)
            printf("Out of range\r\n");
         else if(distance == RX_BUSY)
            printf("Echo line still active after blanking - check RA1\r\n");
         else
            printf("Distance = %3Lu cm\r\n", distance);

         delay_ms(500);   //Update interval
      }
   }
}
/*------------------------------------------------------*/
