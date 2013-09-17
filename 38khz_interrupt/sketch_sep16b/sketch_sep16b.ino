const byte LED = 9;  // Timer 1 "A" output: OC1A

ISR (TIMER2_COMPA_vect)
{
   TCCR1A ^= _BV (COM1A0) ;  // Toggle OC1A on Compare Match
   
   if ((TCCR1A & _BV (COM1A0)) == 0)
     digitalWrite (LED, LOW);  // ensure off
     
}  // end of TIMER2_COMPA_vect

void setup() {
  pinMode (LED, OUTPUT);
  
  // set up Timer 1 - gives us 38.095 MHz (correction: 38.095 KHz)
  TCCR1A = 0; 
  TCCR1B = _BV(WGM12) | _BV (CS10);   // CTC, No prescaler
  OCR1A =  209;          // compare A register value (210 * clock speed)
                         //  = 13.125 nS , so frequency is 1 / (2 * 13.125) = 38095
  
  // Timer 2 - gives us our 1 mS counting interval
  // 16 MHz clock (62.5 nS per tick) - prescaled by 128
  //  counter increments every 8 uS. 
  // So we count 125 of them, giving exactly 1000 uS (1 mS)
  TCCR2A = _BV (WGM21) ;   // CTC mode
  OCR2A  = 124;            // count up to 125  (zero relative!!!!)
  TIMSK2 = _BV (OCIE2A);   // enable Timer2 Interrupt
  TCCR2B =  _BV (CS20) | _BV (CS22) ;  // prescaler of 128

}  // end of setup

void loop()
  {
  // all done by interrupts 
  }
