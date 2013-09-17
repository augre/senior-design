

// Arduino demo program 
// by Mark Bauer Last update 12/20/12


// PORT        Arduino label       processor pin     
//  PORTB.0  0x01      8                 14    Used for debug, connect to scope ch1  
//  PORTB.1  0x02      9                 15    Used for debug, connect to scope ch2
//  PORTB.2  0x04     10                 16    communications input
//  PORTB.3  0x08     11                 17    OC2A  Ir output.
//  PORTB.4  0x10     12                 18    LED2
//  PORTB.5  0x20     13                 19    LED (this is the one on the arduino board)

//  PORTC.0  0x01     A0                 23    A/D converter input
//  PORTC.1  0x02     A1                 24
//  PORTC.2  0x04     A2                 25    button #1
//  PORTC.3  0x08     A3                 26    button #2
//  PORTC.4  0x10     A4                 27    
//  PORTC.5  0x20     A5                 28

//  PORTD.0  0x01      0                  2    Rx
//  PORTD.1  0x02      1                  3    Tx
//  PORTD.2  0x04      2                  4    Servo 0
//  PORTD.3  0x08      3                  5    Servo 1  OC2B    output compare timer 2 B
//  PORTD.4  0x10      4                  6    Servo 2
//  PORTD.5  0x20      5                 11    Servo 3
//  PORTD.6  0x40      6                 12
//  PORTD.7  0x80      7                 13    Beeper

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h> 

// ****************************************************************************
// ****************************************************************************
// ****************************************************************************
// ****************************************************************************
// ****************************************************************************
// ****************************************************************************
// ****************************************************************************

// example clock calculation for periodic timer interrupt using timer 1

// 16Mhz with /8 prescale is 2Mhz or 500nS
// timer 1 will overflow at 65536 so for 1mS we need it to overflow in 2000 cycles
//  65536 - 2000 = 63536
// looks like the overhead is about 5uS so we bump the number by 10 more counts or 63539

#define CLOCK_20mS  25546      //    50 Hz
#define CLOCK_1mS   63539      //  1000 Hz
#define CLOCK_125uS 65289      //  8000 Hz

// for debugging, a few variables will be needed.
volatile uint16_t xx0,xx1,xx2;    // these are just used for debugging


// these variables are used to keep track of time

volatile uint8_t tClick;          // incremented every 1mS in the interrupt routine
volatile uint8_t tClickX;         // incremented every 1mS in the main loop (non interrupt)

volatile uint8_t time_1mS;        // count the number of mS, only used to generate time_100mS
volatile uint8_t time_100mS;      // this only goes from 0->9;
volatile uint32_t time_1S;        // how many seconds we have been running.

// serial I/O variables

volatile uint8_t txBuf[64];       // ring buffer for transmit   if(txBufInPtr == txBufOutPtr)  we are empty
volatile uint8_t txBufInPtr;      // points to where the next characer will go into the ring buffer  
volatile uint8_t txBufOutPtr;     // points to where the next characer will be removed from the buffer

volatile uint8_t rxBuf[64];       // ring buffer to receive
volatile uint8_t rxBufInPtr;      // points to where the next characer will go into the ring buffer
volatile uint8_t rxBufOutPtr;     // points to where the next characer will be removed from the buffer

// these are used for infrared communications 

volatile uint8_t irXState;        // transmit state machine 
volatile uint8_t irXBits;         // bits to send in this byte
volatile uint8_t irXBytes;        // bytes to send in this packet
volatile uint8_t irXPtr;          // next byte to send
volatile uint8_t irXSend;         // byte being sent
volatile uint8_t irXBuf[16];      // holds string to send

volatile uint8_t irRState;        // receive state machine
volatile uint8_t irRTime;         // 
volatile uint8_t irRBit;          //
volatile uint8_t irRReady;        //
volatile uint8_t irRBitCount;     //
volatile uint8_t irRByte;         //
volatile uint8_t irRBuf[16];      // packet we are getting in
volatile uint8_t irRPtr;          //

// lights and beeper

volatile uint8_t led1;            // how long for led1 to be on (mS)
volatile uint8_t led2;            // how long for led2 to be on (mS)

volatile uint8_t buttonPressed;   // set in interrupt routine to let us know what button has been pressed.
volatile uint8_t beep;            // how long to beep at 500Hz (mS)
volatile uint8_t beepLow;         // how long to beep at 250Hz (mS)

// servo control

volatile uint8_t servo[4];        // values to set servos 
volatile uint8_t servoState;      // servo state machine

uint16_t adX;                     // value from the A/D converter, written in interrupt routine

uint32_t loopCounter;             // Main while loop counter, it is to see how busy we are

// Random number generator variables and constants

uint32_t randX;                  // only needed for random number generator.

#define IB1 1
#define IB2 2
#define IB5 16
#define IB18 131072
#define RANDOM_MASK (IB1+IB2+IB5+IB18)

// Servo pin definitions, all are on port D

#define SERVO_0 0x04
#define SERVO_1 0x08
#define SERVO_2 0x10
#define SERVO_3 0x20

// ***********************************************************************
// Your variables can go here
// ***********************************************************************

uint8_t stateMachineA;
uint8_t stateMachineB;



// ***********************************************************************
// IR communications
//       packet can not send 0xFF, we use that to synchronize everything
// ***********************************************************************

void inline irGotBit(){
  // this is to process 1 bit of data from the infrared receiver
  irRByte = irRByte << 1;         // make room for the bit
  irRByte += irRBit;              // stuff in the bit
  irRBitCount++;                  // count how many bits we have stuffed
  if(irRBitCount == 8){           // have we stuffed the full 8 bits yet?
    // we have a byte;
    irRBitCount = 0xFF;           // we have a bit between bytes that will roll this to 0x00 
    irRPtr &= 0x0F;               // keep pointer in buffer
    if(irRReady == 0){            // Don't overwrite the buffer if they haven't looked at it yet
      irRBuf[irRPtr++] = irRByte; // stuff the byte into the ring buffer
    }
  }
  if(irRByte == 0xFF){            // 0xFF is used to synchronize the bytes and packets coming in
    irRBitCount = 0xFF;           // setup for next byte and packet
    if(irRPtr > 3){               // the data in the packet is ready for them
      irRReady = 1;               // let the main loop know it
    }
    irRPtr = 0;                   // reset pointer back to start of buffer
  }  
}

// All interrupt vectors

// Interrupt Vector 0 is the reset vector.
// SIGNAL(INT0_vect){}   // External Interrupt Request 0 
// SIGNAL(INT1_vect){}   // External Interrupt Request 1
#define PCINT0_vect       _VECTOR(3)   /* Pin Change Interrupt Request 0 */
#define PCINT1_vect       _VECTOR(4)   /* Pin Change Interrupt Request 0 */
#define PCINT2_vect       _VECTOR(5)   /* Pin Change Interrupt Request 1 */
#define WDT_vect          _VECTOR(6)   /* Watchdog Time-out Interrupt */
#define TIMER2_COMPA_vect _VECTOR(7)   /* Timer/Counter2 Compare Match A */
#define TIMER2_COMPB_vect _VECTOR(8)   /* Timer/Counter2 Compare Match A */
#define TIMER2_OVF_vect   _VECTOR(9)   /* Timer/Counter2 Overflow */
#define TIMER1_CAPT_vect  _VECTOR(10)  /* Timer/Counter1 Capture Event */
#define TIMER1_COMPA_vect _VECTOR(11)  /* Timer/Counter1 Compare Match A */
#define TIMER1_COMPB_vect _VECTOR(12)  /* Timer/Counter1 Compare Match B */ 
#define TIMER1_OVF_vect   _VECTOR(13)  /* Timer/Counter1 Overflow */
#define TIMER0_COMPA_vect _VECTOR(14)  /* TimerCounter0 Compare Match A */
#define TIMER0_COMPB_vect _VECTOR(15)  /* TimerCounter0 Compare Match B */
#define TIMER0_OVF_vect   _VECTOR(16)  /* Timer/Couner0 Overflow */
#define SPI_STC_vect      _VECTOR(17)  /* SPI Serial Transfer Complete */
#define USART_RX_vect     _VECTOR(18)  /* USART Rx Complete */
#define USART_UDRE_vect   _VECTOR(19)  /* USART, Data Register Empty */
#define USART_TX_vect     _VECTOR(20)  /* USART Tx Complete */
#define ADC_vect          _VECTOR(21)  /* ADC Conversion Complete */
#define EE_READY_vect     _VECTOR(22)  /* EEPROM Ready */
#define ANALOG_COMP_vect  _VECTOR(23)  /* Analog Comparator */
#define TWI_vect          _VECTOR(24)  /* Two-wire Serial Interface */
#define SPM_READY_vect    _VECTOR(25)  /* Store Program Memory Read */




// This is called when a pin on port B changes and it has not been masked

SIGNAL(PCINT0_vect){               // interrupts on every pin change
  irRBit = PINB;                   // take a look at port B to find out    
  irRBit = 0x01 & (irRBit >> 2);   // only interested in bit 0x04

  if(irXState == 0){               // if we are transmitting, don't bother looking at anything
    irRTime = TCNT2;               // find out how long from last bit change (timer 2)
    TCNT2 = 0;                     // reset it for the next bit

    if(irRState == 0){
                                   // last change was middle of bit frame
      
      if(irRTime < 93){            // draw the line at 1.5mS
        irRState = 1;              //   it was less, move to state 1
      } else {
        irGotBit();                //   is was over 1.5mS, we have a bit, process it
      }
    } else {
      irRState = 0;                // we weren't in state 0 so go back to state 0
      irGotBit();                  // must have been a bit, process it
    }
  }
}

// ***********************************************************************
// This is called when a pin on port C changes and it has not been masked
// Used to indicate a button has been pressed.
// ***********************************************************************

SIGNAL(PCINT1_vect){
  xx2++;
  buttonPressed = (~PINC) & 0x0C;     // we are looking at only the pins 0x08 and 0x04 on port C
}

// ***********************************************************************
// This is called when a pin on port D changes      not used at this time
// ***********************************************************************

SIGNAL(PCINT2_vect){
}

// ***********************************************************************
// This is called when the A/D convert finishes a conversion
// ***********************************************************************

SIGNAL(ADC_vect){  // this signals the A/D converter is done
  // The A/D converter is started in the 1mS interrupt routine, so this will get called 1,000 times/second
  adX = ADCH;    //
  servo[0] = adX;
}

// ***********************************************************************
// Serial I/O routines
// ***********************************************************************
  
SIGNAL(USART_UDRE_vect){  // transmit data register empty interrupt
  // we should actually use this, but we cheat and just look at 1mS intervals. 
  // that will limit us to 1,000 per second but we are at 9600 baud so that is ok
}

SIGNAL(USART_RX_vect){  // receive register has something in it
  rxBuf[rxBufInPtr++] = UDR0;  // put character in buffer
  rxBufInPtr &= 0x3F;          // keep it in the buffer
}

// ********************************************************************************
// Timer 0 interrupt, used to terminate the variable portion of the servo controls
// ********************************************************************************

SIGNAL(TIMER0_OVF_vect){    
  switch(servoState){
    case 1: PORTD &= ~SERVO_0; break;
    case 3: PORTD &= ~SERVO_1; break;
    case 5: PORTD &= ~SERVO_2; break;
    case 7: PORTD &= ~SERVO_3; break;
  }
  TIMSK0 = 0;        // no more interrupts needed from timer0
}  

// ********************************************************************************
// Timer 1 is the 1mS interrupt routine, many things happen here are we break it up
// into many subroutines.  Notice the inline statement on the subroutines.  The 
// compiler doesn't actually make them subroutines, it will place the code inline
// with where we need it.  This just makes it easier for us to read.
// ********************************************************************************

// ********************************************************************************
// IR transmit routine, the 38Khz modulation is done by timer 2
// we just turn it on and off here
// ********************************************************************************

void inline processIRTx(){  // we are calling this every 1.0 mS.

  if(irXState){
    // we are sending something by IR

    if(irXState == 1){

      // ready to send the next bit

      if(irXBits == 0){  // do we have any bits left in this byte?

        // no more bits in this byte

        if(irXBytes == 0){  // do we have any bytes left to send?

          // no more bytes to send

          TCCR2A = 0x00;
          TCCR2B = 0x06;
          OCR2A = 0xFF;   // let counter go way up.
          irXState = 0;   // we must be done

        } else {

          // we have another byte to send

          irXSend = irXBuf[irXPtr++];  // the byte we need to send next
          irXBytes--;
          irXBits = 9;   // 8 data bits with a stop bit     

        }
      }
    }

    if(irXState){

      // we have a bit ready to send

      if(irXState == 1){

        // first half of bit, is the bit just as it is.

        if(irXSend & 0x80){
          // send high
          TCNT2 = 104;   // fix phase of 38Khz signal
          TCCR2A = 0x41; // turn on 38Khz signal
          PORTB &= ~0x10;
        } else {
          // send low
          TCCR2A = 0x01;  // kill 38Khz output
          PORTB |= 0x10;
        }            

        irXState = 2;

      } else {

        // second half of bit

        if(irXSend & 0x80){

          // send low

          TCCR2A = 0x01;  // kill 38Khz output

        } else {

          // send high

          TCNT2 = 104;   // fix phase of 38Khz signal
          TCCR2A = 0x41; // turn on 38Khz signal
        }            
        irXBits--;
        irXState = 1;     
        irXSend = irXSend << 1;
      }
    }
  }
}

// ********************************************************************************
// Servo control routines 
//     called every 1mS
// ********************************************************************************

void inline processServos(){

  servoState++;             
  if(servoState > 19){            // we have 20 states, 0->19, 1mS in each
    servoState = 0;               // the whole thing repeats every 20mS
  }

  switch(servoState){
    case 0:                       // state 0 is servo 0 on for 1mS
      PORTD |= SERVO_0;  
      break;
    case 1:                       // state 1 is servo 0 on for additional 0->1mS
      TCNT0 = ~servo[0];          //         turned off in other interrupt routine
      TIFR0 |= 0x01; 
      TIMSK0 = 1; 
      break;
    case 2:  
      PORTD |= SERVO_1;           // same as state 0 but for servo 1
      break;
    case 3:  
      TCNT0 = ~servo[1]; 
      TIFR0 |= 0x01; 
      TIMSK0 = 1; 
      break;
    case 4:  
      PORTD |= SERVO_2;           // same as state 0 but for servo 2
      break;
    case 5:  
      TCNT0 = ~servo[2]; 
      TIFR0 |= 0x01; 
      TIMSK0 = 1; 
      break;
    case 6:  
      PORTD |= SERVO_3;           // same as state 0 but for servo 3
      break;
    case 7:  
      TCNT0 = ~servo[3]; 
      TIFR0 |= 0x01; 
      TIMSK0 = 1; 
      break;
  }
}

// ********************************************************************************
// led routines 
//     called every 1mS
// ********************************************************************************

void inline processLEDs(){  // led dimming

  if(led1){             // if led1 has a non zero value, turn on the led
    PORTB |= 0x20;
    led1--;             // keep bumping it down if not zero
  } else {
    PORTB &= ~0x20;     // it was zero, turn it off
  }

  if(led2){             // other led, same thing as last one.
    PORTB |= 0x10;
    led2--;
  } else {
    PORTB &= ~0x10;
  }
}

// ********************************************************************************
// beeper routines 
//     called every 1mS
// ********************************************************************************

void inline processBeeper(){
  if(beepLow){  
    if(tClick & 1){     // only hits every other mS so changes every 2mS
      PIND = 0x80;      // toggle PORTD.7
    }
    beepLow--;          // used as a timer to stop it
  }
  if(beep){  
    PIND = 0x80;  // toggle PORTD.7
    beep--;
  }
}

// ********************************************************************************
// serial transmit routine, Yes it is cheating to some degree
//     called every 1mS
// ********************************************************************************

void inline processTxBuf(){  // serial transmit routines.
  if(txBufInPtr != txBufOutPtr){
    // we have something to send.
    if(UCSR0A & (1 << UDRE0)){
      // the uart is ready for another character
      UDR0 = txBuf[txBufOutPtr++];
      txBufOutPtr &= 0x3F;
    }
  }  
}

// ***************************************************************************************
// A/D converter start routine, we won't get a value till it interrupts us somewhere else
//     called every 1mS
// ***************************************************************************************

void inline processADConverter(){
  ADCSRA |= 0x40;      // start a conversion
}


// ***************************************************************************************
// Timer 1 overflow interrupt
// ***************************************************************************************

SIGNAL(TIMER1_OVF_vect){
  TCNT1 = CLOCK_1mS;      // must be first instruction!!!!

  tClick++;               // keep track of time

  processIRTx();          // IR transmit routines

  processADConverter();   // A/D converter

  processServos();        // Servos
  
  processLEDs();          // Lights

  processBeeper();        // Beepers

  processTxBuf();         // Serial transmit
 
}


// ***************************************************************************************
// ***************************************************************************************
// ***************************************************************************************
// ***************************************************************************************

//   Everything before this point is interrupt routines
//   Everything after this point is called from the main loop

// ***************************************************************************************
// ***************************************************************************************
// ***************************************************************************************
// ***************************************************************************************



// ***************************************************************************************
// Random number generation, useful for various things.
// ***************************************************************************************

uint8_t randomBit(){  //  I didn't write it, got it many years ago, don't remember where.
  uint8_t r;
  if(randX & IB18){
    r = 1;
    randX = ((randX ^ RANDOM_MASK) << 1) | 1;
  } else {
    r = 0;
    randX <<= 1;
  }
  return r;
}

uint8_t randomByte(){
  uint8_t i,r;
  r=0;
  for(i=0;i<8;i++){
    r <<= 1;
    r += randomBit();
  }
  return r;
}

// ***************************************************************************************
// ***************************************************************************************
// Serial output routines
// ***************************************************************************************
// ***************************************************************************************


// ***************************************************************************************
// Queue up a character to go out the serial port
// The actual sending out the port is done in an interrupt routine
// ***************************************************************************************

void outChar(uint8_t c){     // just put it in the buffer to go out
  txBuf[txBufInPtr++] = c;   // it is output in the interrupt routine
  txBufInPtr &= 0x3F;        // we don't actually worry about overflow, if it does, we get junk.
}

// ***************************************************************************************
// output a single nibble (0->F) as a hex character
// ***************************************************************************************

void printHex4(uint8_t x){   // just convert a binary nibble to 0123456789ABCDEF
  x = x & 0x0F;    // sometimes we get more bits that we need to ignore
  x = x + '0';     // convert it to ASCII
  if(x > '9'){     // in ASCII there are 7 characters between '9' and 'A'
    x+=7;
  }
  outChar(x);
}

// ***************************************************************************************
// output a single byte in hex
// ***************************************************************************************

void printHex8(uint8_t x){  // print in hex a single byte
  printHex4(x >> 4);
  printHex4(x);
}

// ***************************************************************************************
// output a two byte variable in hex
// ***************************************************************************************

void printHex16(uint16_t x){ // print in hex 2 bytes
  printHex8((x >> 8) & 0xFF);
  printHex8(x & 0xFF);
}

// ***************************************************************************************
// output a four byte variable in hex
// ***************************************************************************************

void printHex32(uint32_t x){  // print in hex 4 bytes
  printHex16((x >> 16) & 0xFFFF);
  printHex16(x & 0xFFFF);
}

// ***************************************************************************************
// output a character string until we hit a null (0x00) character
// ***************************************************************************************

void printString(char *s){  // print a character string
  uint8_t i;
  i=0;
  while(s[i] != 0){   // look for NULL termination
    outChar(s[i]); 
    i++;
  }
}

// ***************************************************************************************
// output a two byte value as a unsigned decimal integer
// ***************************************************************************************

void printDecimal(uint16_t n){  // print an unsigned 16 bit integer
  uint8_t i;
  char x[6];
  for(i=5;i!=0;i--){
    x[i]= (n % 10) + '0';
    n = n / 10;
  }
  i=1;
  while(i < 4){
    if(x[i] == '0'){
      x[i] = ' ';
    } else {
      i = 5;
    }
    i++;
  }
  for(i=0;i<5;i++){
    outChar(x[i]);
  }
}  

// ***************************************************************************************
// Convert a single ascii character to a 4 bit nibble (0->F)
// ***************************************************************************************

uint8_t atoi4(uint8_t c){
  uint8_t x;
  x = c - '0';
  if(x > 9){
    x = x - 7;
  }
  if(x > 0x0F){
    x = 0;
  }
  return x;
}

// ***************************************************************************************
// Convert two ascii characters to an 8 bit byte (0x00->0xFF)
// ***************************************************************************************

uint8_t atoi8(uint8_t *s){
  uint8_t x;
  x = (atoi4(s[0]) << 4) + atoi4(s[1]);
  return(x);
}

// ***************************************************************************************
// Get a character from the serial input buffer.  It was put there by an interrupt routine
// ***************************************************************************************

uint8_t getChar(){
  uint8_t c;
  c = rxBuf[rxBufOutPtr++];
  rxBufOutPtr &= 0x3F;
  return c ;
}

// ***************************************************************************************
// ***************************************************************************************
// ***************************************************************************************
// ***************************************************************************************

// Everything up to this point is code to support what you want it to do

// ***************************************************************************************
// ***************************************************************************************
// ***************************************************************************************
// ***************************************************************************************

// ***************************************************************************************
// Command processing
//    keep getting characters from the serial port
//    if we see a carriage return (0x0D) process it
// ***************************************************************************************

void processSerialCharacter(){
  uint8_t x;
  static uint8_t cmdBuf[16];
  static uint8_t cmdBufPtr;
  x = getChar();
  cmdBuf[cmdBufPtr++] = x;
  cmdBufPtr &= 0x0F;
  if(x == 0x0D){
    if(cmdBufPtr > 1){
      // process command  ... assume single letter followed by 2 hex dig
      switch(cmdBuf[0]){
        case 'A':
          stateMachineA = atoi8(&cmdBuf[1]);
          break;
        case 'X':
          stateMachineA = 0;
          break;
        case 'S':
          servo[0] = atoi8(&cmdBuf[1]);
          break;
      }        
    } 
    cmdBufPtr = 0;  // get ready for next command
  }
}

// ***************************************************************************************
// printStatus
//    Use this to print debug information or anything you might find useful
//    In this example, it is called every 100mS
// ***************************************************************************************
    
void printStatus(){
  printHex32(loopCounter);
  outChar(' ');
  printHex8(adX);
  outChar(' ');  
  printHex8(buttonPressed);
  printString(" stA=");
  printHex8(stateMachineA);
  printString(" stB=");
  printHex8(stateMachineB);
  outChar(0x0D);
  outChar(0x0A);
}

// ***************************************************************************************
// Send a packet out the IR port       
// ***************************************************************************************

void sendIRPacket(){
  TCCR2A = 0x01;  // setup timer2 to produce the 38Khz (but leave it off)
  TCCR2B = 0x09;
  OCR2A = 105;
  OCR2B = 30;

  irXBuf[0] = 0x55;              // this is needed to sync the system
  irXBuf[1] = 0xFF;              // this is needed to sync the system
  irXBuf[2] = 0x34;              // your first byte of data
  irXBuf[3] = time_1S & 0xFF;    // your second byte of data
  irXBuf[4] = 0x78;              // your third byte of data
  irXBuf[5] = 0xFF;              // this is needed after your data
  irXBits = 0;
  irXBytes = 6;                  // sending 6 bytes 3 + numberYouAreSending.
  irXPtr = 0;
  irXState = 1;  // tell it to go.
}


void stateMachine_A(){
//   switch(state_A){
  
//  }  
}





void processPeriodic100mS(){
  uint8_t i;
  // this would be a good place to put your state machine.


  stateMachine_A();

  printStatus();
  loopCounter = 0;
}

void processPeriodic1S(){

//  if((tS & 0x03) == 0){
 //   sendIRPacket();
//  }

//  beep = 1;

  led2 = 50;

//  servo[0] = randomByte();
  servo[1] = randomByte();
  servo[2] = randomByte();
  servo[3] = randomByte();

//  xx1 = randomByte();

//  beep = 2;

 

}


int main(){


  DDRB = 0xFB;  // 0x04 is comm in.
  DDRC = 0xF2;  // 0x0C are button in
  DDRD = 0xFF;  // servo drive

  PORTB = 0x20;

  // setup baud rate

  UBRR0H = 0;       // Serial baud rate generator  with 16Mhz xtal 
  UBRR0L = 104;     // set baud rate to 9600    ((16000000/16)/Baud)-1         I think
//  UCSR0B = 0x18;    //RXCIE=0x80 TXCIE=0x40 UDRIE=0x20 RXEN=0x10 TXEN=0x08
  UCSR0B = 0x98;    //RXCIE=0x80 TXCIE=0x40 UDRIE=0x20 RXEN=0x10 TXEN=0x08
//  UCSR0C = (1 << USBS0) | (3<<UCSZ00);
  UCSR0C = 0x06;

  TCCR0B = 0x03;  // clk/64 or 250Khz   used to run servos

  TIMSK1 = 1;        // 1mS timer
  TCCR1A = 0x00;
  TCCR1B = 0x02;
  
  // timer2 is used for I/R communications
  
  TCCR2A = 0x00;
  TCCR2B = 0x06;

  PCMSK0 = 0x04;  // enable pin change interrupt for PORTB.2  communications in.
  PCMSK1 = 0x0C;  // enable pin change interrupts for PORTC.2 and PORTC.3 for the two button inputs
  PCICR = 0x03;   // enable pin change interrupts for both PCMSK0 and PCMSK1

  PORTC = 0x0C;   // pull up resistors for button pins.

  ADMUX  = 0x40;      // 0100 0000   vref = Vcc AD0 selected  use 0x60 for left justified
  ADCSRA = 0xCF;      // 1100 0101
  ADCSRB = 0x00;
  sei();   // enable ints

  tClickX = tClick;  // not actually needed
  time_1mS = 0;

  servo[0] = 128;
  servo[1] = 196;
  servo[2] = 30;
  servo[3] = 240;

  randX = 0x1234;  // seed for random number generator

  while(1){

    loopCounter++;    // fastest loop is about 523440

//    PINB = 0x01;  //  main loop indicator

    while(tClick != tClickX){
      tClickX++;
      time_1mS += 1;   // counting mS

      if(buttonPressed){
        if(buttonPressed & 0x08){
          beepLow = 250;
        } else {
          beep = 25;
        }
        buttonPressed = 0;
        led1 = 50;
      }

      if(rxBufInPtr != rxBufOutPtr){  // character from uart
        processSerialCharacter();
      }
 
      if(irRReady){
        // process packet from ir
        printStatus();
        irRReady = 0;
      }

      if(time_1mS >= 100){
//        PORTB ^= 0x20;  // toggle LED
        time_1mS = 0;
        time_100mS++;
        processPeriodic100mS();

 
        if(time_100mS >= 10){
          time_100mS = 0;
          time_1S++;
          processPeriodic1S();
        }
      }      
    }
  }
}
