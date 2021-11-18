// ONLY for 16MHz 328P based systems (Arduino uno, nano, pro mini,..)
#ifndef __AVR_ATmega328P__
#error Only ATMega328P based boards are supported.
#endif

#include <stdint.h>

#define ADCPIN 0 // 0=A0..7=A7

volatile uint16_t sample,offset;
volatile uint8_t lowbyte,flag;

// Initialization of ADC for auto trigger, 1.1V ref and pin A0, 19.2 kHz
void setupADC() {

  uint16_t adcval;

  pinMode(ADCPIN+14,INPUT);
  analogReference(INTERNAL);
  delay(100);
  adcval=analogRead(ADCPIN+14);
  delay(100);
  adcval=analogRead(ADCPIN+14);
  delay(100);
  adcval=analogRead(ADCPIN+14);
  offset=adcval-128;
  
  // clear ADLAR in ADMUX (0x7C) to right-adjust the result
  // ADCL will contain lower 8 bits, ADCH upper 2 (in last two bits)
  ADMUX &= B11011111;
 
  // Set REFS1..0 in ADMUX (0x7C) to change reference voltage to the
  // proper source (11)
  ADMUX |= B11000000;
 
  // Clear MUX3..0 in ADMUX (0x7C) in preparation for setting the analog
  // input
  ADMUX &= B11110000;
 
  // Set MUX3..0 in ADMUX (0x7C) to read from AD8 (Internal temp)
  // Do not set above 15! You will overrun other parts of ADMUX. A full
  // list of possible inputs is available in Table 24-4 of the ATMega328
  // datasheet
  //ADMUX |= 8;
  ADMUX |= ADCPIN; // Binary equivalent 0 --> A0, 3 --> A3
 
  // Set ADEN in ADCSRA (0x7A) to enable the ADC.
  // Note, this instruction takes 12 ADC clocks to execute
  ADCSRA |= B10000000;
 
  // Set ADATE in ADCSRA (0x7A) to enable auto-triggering.
  ADCSRA |= B00100000;
 
  // Clear ADTS2..0 in ADCSRB (0x7B) to set trigger mode to free running.
  // This means that as soon as an ADC has finished, the next will be
  // immediately started.
  ADCSRB &= B11111000;
 
  // Set the Prescaler to 64 (16000KHz/64 = 250KHz)
  // Above 200KHz 10-bit results are not reliable.
  ADCSRA |= B00000110;
  ADCSRA &= B11111110;
  
 
  // Set ADIE in ADCSRA (0x7A) to enable the ADC interrupt.
  // Without this, the internal interrupt will not trigger.
  ADCSRA |= B00001000;
 
  // Enable global interrupts
  // AVR macro included in <avr/interrupts.h>, which the Arduino IDE
  // supplies by default.
  //sei();
 
  // Kick off the first ADC
  //readFlag = 0;
  // Set ADSC in ADCSRA (0x7A) to start the ADC conversion
  ADCSRA |=B01000000;
}

ISR(ADC_vect){

    //get sample
    sample=ADCL | (ADCH << 8);
    sample-=offset;
    lowbyte=sample;
    flag=1;

    return;
}

// Initialization
void setup() {
    
    // set-up serial
    Serial.begin(1000000);
    flag=0;
    // init ADC 19.2 kHz auto-trigger
    setupADC();

    return;
}

// Processor loop
void loop() {

    if (flag) {
        Serial.write(lowbyte);
        flag=0;
    }
}
