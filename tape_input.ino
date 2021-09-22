  // ONLY for 16MHz 328p based systems (Arduino uno, nano, pro mini,..)
// For SDFT
#include <stdint.h>
#include <math.h>
// For SDcard
#include <SPI.h>
#include <SD.h>

//#define FS 19200  //sample frequency for ADC
#define DFTBUFSIZE 16  // linked to FS and the frequencies to detect
#define DIVISOR 32768L  //for use with int16_t --> [-1,1)
//#define THRESHOLD 2000U // threshold for signal/ no signal in magnitude value of dft bin, experimental value TODO: how to determine?
#define LOWSIGNALLIMIT 500U // lower limit for magnitude values. anything below is considered noise
#define DECODEBUFSIZE 128 // should be big enough to buffer for 250ms of data at Fs (SD card write latency) and even () 128
#ifndef PI
#define PI 3.14159265
#endif

//// full equivalent to (int32_t)a*b/32768UL int16_t a,b
//#define fmuls16(result, multiplicand, multiplier) \
//  __asm__ __volatile__ ( \
//                         "	clr	r2 \n\t" \
//                         "	fmuls	%B2, %B1 \n\t" /* (signed)ah * (signed)bh*/ \
//                         "	movw	%A0, r0 \n\t" \
//                         "	fmul	%A2, %A1 \n\t" /* al * bl*/ \
//                         "	adc	%A0, r2 \n\t" \
//                         "	mov r3, r1 \n\t" \
//                         "	fmulsu	%B2, %A1 \n\t" /* (signed)ah * bl*/ \
//                         "	sbc	%B0, r2 \n\t" \
//                         "	add	r3, r0 \n\t" \
//                         "	adc	%A0, r1 \n\t" \
//                         "	adc	%B0, r2 \n\t" \
//                         "	fmulsu	%B1, %A2 \n\t" /* (signed)bh * al*/ \
//                         "	sbc	%B0, r2 \n\t" \
//                         "	add	r3, r0 \n\t" \
//                         "	adc	%A0, r1 \n\t" \
//                         "	adc	%B0, r2 \n\t" \
//                         "	clr r1 \n\t" \
//                         : "=&a" (result) \
//                         : "a" (multiplicand),  "a" (multiplier) \
//                         : "r2","r3" \
//                       );

// gcc avr code for 16*16->high 16 bit signed fractional multiply, mofified from AVR201 assembly less rounding error. Calculates:
//tmp_re=((int32_t)fact_re*bin_re-(int32_t)fact_im*bin_im)/DIVISOR;
//tmp_im=((int32_t)fact_re*bin_im+(int32_t)fact_im*bin_re)/DIVISOR;
//bin_re=tmp_re
//bin_im=tmp_im
#define fmuls16_c(bin_re,bin_im,fact_re,fact_im) \
  __asm__ __volatile__ ( \
                         "	clr	r2 \n\t" /* r2=0 for calculations */ \
                         "	fmuls %B2, %B0 \n\t" /* (signed)ah B2 * (signed)bh B0  for fact_re*bin_re */ \
                         "	movw r4, r0 \n\t" \
                         "	fmul %A2, %A0 \n\t" /* al * bl*/ \
                         "	adc r4, r2 \n\t" \
                         "	mov r3, r1 \n\t" \
                         "	fmulsu %B2, %A0 \n\t" /* (signed)ah B2 * bl A0 */ \
                         "	sbc	r5, r2 \n\t" \
                         "	add	r3, r0 \n\t" \
                         "	adc	r4, r1 \n\t" \
                         "	adc	r5, r2 \n\t" \
                         "	fmulsu	%B0, %A2 \n\t" /* (signed)bh B0 * al A2 */ \
                         "	sbc	r5, r2 \n\t" \
                         "	add	r3, r0 \n\t" \
                         "	adc	r4, r1 \n\t" \
                         "	adc	r5, r2 \n\t" \
                         "	fmuls %B3, %B1 \n\t" /* (signed)ah B3 * (signed)bh B1  minus fact_im*bin_im */ \
                         "	movw r6, r0 \n\t" \
                         "	fmul %A3, %A1 \n\t" /* al * bl*/ \
                         "	adc r6, r2 \n\t" \
                         "	mov r3, r1 \n\t" \
                         "	fmulsu	%B3, %A1 \n\t" /* (signed)ah * bl*/ \
                         "	sbc	r7, r2 \n\t" \
                         "	add	r3, r0 \n\t" \
                         "	adc	r6, r1 \n\t" \
                         "	adc	r7, r2 \n\t" \
                         "	fmulsu	%B1, %A3 \n\t" /* (signed)bh * al*/ \
                         "	sbc	r7, r2 \n\t" \
                         "	add	r3, r0 \n\t" \
                         "	adc	r6, r1 \n\t" \
                         "	adc	r7, r2 \n\t" \
                         "	sub	r4, r6 \n\t" /* substract */ \
                         "	sbc	r5, r7 \n\t" \
                         "	fmuls %B2, %B1 \n\t" /* (signed)ah * (signed)bh  for fact_re*bin_im */ \
                         "	movw r6, r0 \n\t" \
                         "	fmul %A2, %A1 \n\t" /* al * bl*/ \
                         "	adc r6, r2 \n\t" \
                         "	mov r3, r1 \n\t" \
                         "	fmulsu	%B2, %A1 \n\t" /* (signed)ah * bl*/ \
                         "	sbc	r7, r2 \n\t" \
                         "	add	r3, r0 \n\t" \
                         "	adc	r6, r1 \n\t" \
                         "	adc	r7, r2 \n\t" \
                         "	fmulsu	%B1, %A2 \n\t" /* (signed)bh * al*/ \
                         "	sbc	r7, r2 \n\t" \
                         "	add	r3, r0 \n\t" \
                         "	adc	r6, r1 \n\t" \
                         "	adc	r7, r2 \n\t" \
                         "	fmuls %B3, %B0 \n\t" /* (signed)ah * (signed)bh  plus fact_im*bin_re */ \
                         "	add	r6, r0 \n\t" \
                         "	adc	r7, r1 \n\t" \
                         "	fmul %A3, %A0 \n\t" /* al * bl*/ \
                         "	adc r6, r2 \n\t" \
                         "	adc	r7, r2 \n\t" \
                         "	mov r3, r1 \n\t" \
                         "	fmulsu %B3, %A0 \n\t" /* (signed)ah * bl*/ \
                         "	sbc	r7, r2 \n\t" \
                         "	add	r3, r0 \n\t" \
                         "	adc	r6, r1 \n\t" \
                         "	adc	r7, r2 \n\t" \
                         "	fmulsu	%B0, %A3 \n\t" /* (signed)bh * al*/ \
                         "	sbc	r7, r2 \n\t" \
                         "	add	r3, r0 \n\t" \
                         "	adc	r6, r1 \n\t" \
                         "	adc	r7, r2 \n\t" \
                         "	movw %A0, r4 \n\t" /* bin_re store */ \
                         "	movw %A1, r6 \n\t" /* bin_im store */ \
                         "	clr r1 \n\t" \
                         : "+a" (bin_re), "+a" (bin_im) \
                         : "a" (fact_re),  "a" (fact_im) \
                         : "r2", "r3", "r4", "r5", "r6", "r7"  /* r2 used as '0' for carry corrections, r3 as rounding register, r4 and r5 as temporary registers for bin_re and r6 and r7 as temporary registers for bin_im */ \
                       );

volatile int16_t samplebuf[DFTBUFSIZE], bin0_re, bin0_im, bin1_re, bin1_im, fact0_re, fact0_im, fact1_re, fact1_im;
volatile uint16_t mag0,mag1; //,sig0[256],sig1[256];
volatile uint32_t timeout = 400000; // ~20s for timeout on header, default for first header
volatile uint8_t sampleindex, mode, bitnr, byte_val, decodebufpos, cnt0, cnt1, writeflag, bufpart, bit_timeout, headerdetected, sigcnt, record;  //, header0, header1;
volatile uint8_t decodebuf[DECODEBUFSIZE];
uint8_t N,cycle025,cycle075,cycle100,cycle150,cycle200,cycle050;
uint16_t signalthreshold;
int16_t signalmultiplier;
File recordFile;
const uint8_t headerid[] = {0x1F, 0xA6, 0xDE, 0xBA, 0xCC, 0x13, 0x7D, 0x74}; // MSX .cas code for a header, must by 8-byte aligned in .cas file


// Initialization of ADC for auto trigger, 1.1V ref and pin A0, 19.2 kHz
void setupADC() {

  //int16_t adc_val;

  /* get initial value from ADC to fill the buffer to avoid large signal step
  pinMode(A0, INPUT);
  analogReference(INTERNAL);
  // Get sample to prepare buffers
  delay(1);
  adc_val = analogRead(A0);
  delay(1);
  adc_val = analogRead(A0);
  adc_val *= 16;
  */

  // set-up ADC for 19.2kHz sample rate and auto-trigger with interrupt routine, ADC frequency 250kHz
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
  ADMUX |= B00000000; // Binary equivalent 0 --> A0
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
  // Kick off the first ADC
  // Set ADSC in ADCSRA (0x7A) to start the ADC conversion
  ADCSRA |= B01000000;

  //return adc_val;
}

// converts float to 16 bit fixed point int (*DIVISOR), 1.15
int16_t float2int(float x) {

  int32_t tmp;

  tmp = x * DIVISOR;
  // check limits
  if (tmp < -32768) tmp = -32768; // minimum is -1
  if (tmp > 32767) tmp = 32767; // maximum is 0.9999x (not 1!)
  return (int16_t)tmp;
}

// converts 16 bit fixed point int to float (/DIVISOR)
float int2float(int16_t x) {

  return (float)x / DIVISOR;
}

void dftcoefficient(uint8_t m, uint8_t N, volatile int16_t *coef_re, volatile int16_t *coef_im) {

  float re, im;

  re = cos(2 * PI * m / (float)N);
  im = sin(2 * PI * m / (float)N);
  *coef_re = float2int(re);
  *coef_im = float2int(im);

  return;
}

// num must be 2^i and smaller or equal to DFTBUFSIZE
void setupsdft(uint8_t num) {

  uint8_t i;

  mode = 0; // just monitor
  N = num; // N-bin dft
  if (N<=8) signalmultiplier=32; else signalmultiplier=16;
  sampleindex = 0;
  decodebufpos = 0;
  bufpart = 0;
  headerdetected = 0;
  // reset sdft values
  bin0_re=0;
  bin0_im=0;
  bin1_re=0;
  bin1_im=0;
  // set msx .cas decoder variables for timing (bit,stopbit length etc.)
  cycle100=N; // cycles for 1 bit
  cycle050=N/2; // cycles for 1 bit
  cycle200=N*2; // cycles for 2 stopbits
  cycle025=N/4; // wait period for sync on startbit
  cycle075=cycle025*3; // time for valid startbit
  cycle150=cycle075*2; // time for valid stopbit
  
  // setup constants for sdft coef=exp(i*2*PI*m/N)
  // for dft bin 1 ('0' value, 1200Hz)
  dftcoefficient(1, num, &fact0_re, &fact0_im);
  // for dft bin 2 ('1' value, 2400Hz)
  dftcoefficient(2, num, &fact1_re, &fact1_im);
  delay(1); // wait for adc sample buffer to fill to avoid weird step functions

  return;
}

// sliding window dft, 2 bins for either 1200 & 2400 Hz (N=16) or 2400 & 4800 Hz (N=8) at Fs 19200 Hz.
// magnitude approx by adding abs value instead of sqrt(re^2+im^2) for speed.
// sdft algorithm Xm(n)=exp(i*2*pi*m/N)*[Xm(n-1)+x(n)-x(n-N)]
ISR(ADC_vect) {

  int16_t xin, xout;

  // get ADC signal value
  xin = ADCL | (ADCH << 8);
  //multiply for resolution in 16 bit fixed point. default: 16
  xin *= signalmultiplier;

  //swap samples in and out of ring buffer for sdft
  xout = samplebuf[sampleindex];
  samplebuf[sampleindex] = xin;
  sampleindex++;
  sampleindex &= (N - 1);

  // in signal monitor monitor mode (no recording) skip all processing
  if (mode==0) return;

  //real additions
  bin0_re += xin;
  bin1_re += xin;
  bin0_re -= xout;
  bin1_re -= xout;

  //complex multiply
  // for '0' 1200Hz/2400Hz
  fmuls16_c(bin0_re, bin0_im, fact0_re, fact0_im);

  // for '1' 2400Hz/4800Hz
  fmuls16_c(bin1_re, bin1_im, fact1_re, fact1_im);

  // magnitude estimate
  mag0 = abs(bin0_re) + abs(bin0_im);
  mag1 = abs(bin1_re) + abs(bin1_im);

  /* signal recording (optional for debugging)
  if (record) {
    sig0[sigcnt]=mag0;
    sig1[sigcnt]=mag1;
    sigcnt++;
    if (sigcnt==0) record=0;
  }
  */
  // determine if count the sample as a 'one', a 'zero' or nothing
  if ((mag0 > mag1) && (mag0 > signalthreshold)) cnt0++;
  else if ((mag1 > mag0) && (mag1 > signalthreshold)) cnt1++;

  // decoding MSX tape: header,data,header,data...
  switch (mode) {
    case 1: // signal threshold determination, only timeout
      timeout--;
      if (timeout==0) mode=15; // no valid signal detected within time frame
      break;
    case 2: // wait for proper header
      timeout--;
      if (timeout == 0) {
        mode = 16; //timeout error for header
        break;
      }
      if (cnt0 > 0) {
        cnt0 = 0;
        cnt1 = 0;
        break;
      }
      if (cnt1 == 255) {
        //mode = 3;
        cnt1 = 0;
        cnt0 = 0;
        headerdetected = 1;
        timeout = 200000; // wait for ~10s for the first startbit
        break;
      }
      break;
    case 3: // wait for first startbit after header
      if (cnt1 > 0) {
        cnt1 = 0;
        cnt0 = 0;
      }
      else {
        if (cnt0 == cycle075) { // was >11
          //startbit=1;
          mode = 5;
          bit_timeout = cycle025; // 16 samples - 12 (startbit)=4
          break;
        }
      }
      timeout--;
      if (timeout == 0) mode = 17; //timeout error for first startbit
      break;
    case 4: // wait for normal startbit
      if (cnt1 > 0) {
        cnt1 = 0;
        //cnt0 = 0;
      }
      else {
        //if (cnt0 == cycle075) { // was >11
        if (cnt0 > cycle050) { // was >11
          //startbit=1;
          mode = 5;
          bit_timeout = cycle050-1; // 16 samples - 12 (startbit)=4 --> now half
          break;
        }
      }
      bit_timeout--;
      if (bit_timeout == 0) {
        mode = 18; //bit_timeout error for startbit
      }
      break;
    case 5: // delay before byte decoding
      bit_timeout--;
      if (bit_timeout == 0) {
        mode = 6;
        cnt0 = 0;
        cnt1 = 0;
        bitnr = 0;
        byte_val = 0;
        bit_timeout = cycle100; // 1 bit
      }
      break;
    case 6: // decoding the byte value
      bit_timeout--;
      if (bit_timeout == 0) {
        //if (cnt1 > cnt0) byte_val += (1 << bitnr);
        //if (cnt1 > cycle050) byte_val += (1 << bitnr); // the bit it is a one
        if (cnt1 > cycle050) byte_val += (1 << bitnr); // the bit it is a one
        else {
          if (cnt0 < cycle050) { // the bit is not a valid zero
             mode=20; // no valid bit, it is not a proper zero or one
             break;
          }
        }
        bitnr++;
        cnt0 = 0;
        cnt1 = 0;
        if (bitnr > 7) {
          mode = 7;
          bit_timeout = cycle200; // 2 stopbits (32)
          break;
        }
        bit_timeout = cycle100; // next (1 bit)
      }
      break;
    case 7: // 2 stopbits
      if (cnt0 > 0) {
        cnt0 = 0;
        cnt1 = 0;
      }
      else {
        //if (cnt1 == cycle150) { // was >23
        if (cnt1 > cycle100) { 
          decodebuf[decodebufpos] = byte_val;
          decodebufpos++;
          if (decodebufpos == DECODEBUFSIZE / 2) {
            bufpart = 0;
            writeflag = 1;
          }
          if (decodebufpos == DECODEBUFSIZE) {
            bufpart = 1;
            writeflag = 1;
            decodebufpos = 0;
          }
          cnt0 = 0;
          cnt1 = 0;
          bit_timeout = cycle200; // was 24, later 32
          mode = 4; // next byte, startbit
          break;
        }
      }
      bit_timeout--;
      if (bit_timeout == 0) mode = 19; // stopbit error
  }
  return;
}

// Initialization
void setup() {

  int16_t adc_val;

  // set-up serial
  Serial.begin(115200);
  // init sdcard
  if (!SD.begin(4)) {
    Serial.println(F("initialization of sdcard failed!"));
    while (1);  // hang
  }
  // remove test file
  SD.remove("record.cas");
  // open file for recording
  recordFile = SD.open("record.cas", FILE_WRITE);
  //*/
  // init ADC 19.2 kHz auto-trigger
  setupADC();
  // init sdft for 2400bps for signal and baudrate detection (1200/2400bps)
  setupsdft(8);
  Serial.println("Ready");
  mode=1;
  //record=1;

  return;
}

// save full part of the buffer (option=0) or all remaining data (option!=0)
void savedata(uint8_t option) {

  uint16_t i, start;

  if (writeflag) {
    //recordFile.write((char*)decodebuf+bufpart*(DECODEBUFSIZE/2),(DECODEBUFSIZE/2));
    for (i = 0; i < DECODEBUFSIZE / 2; i++) {
      Serial.println(decodebuf[i + bufpart * (DECODEBUFSIZE / 2)]);
      recordFile.write(decodebuf[i + bufpart * (DECODEBUFSIZE / 2)]);
    }
    writeflag = 0;
  }
  if (option) {
    if ((decodebufpos == 0) || (decodebufpos == (DECODEBUFSIZE / 2))) return; // no additional data present
    // write last data portion from buffer
    if (decodebufpos > (DECODEBUFSIZE / 2)) start = (DECODEBUFSIZE / 2); else start = 0;
    for (i = start; i < decodebufpos; i++) {
      Serial.println(decodebuf[i]);
      recordFile.write(decodebuf[i]);
    }
    // reset buffer position to initial state, avoid writing data multiple times
    decodebufpos = 0;
    bufpart = 0;
  }
  return;
}


void writeheader() {

  uint8_t i;

  savedata(1); // ensure all available data is saved before the header is written

  // TODO align at 8-byte boundaries, pad with zeros
  for (i = 0; i < sizeof(headerid); i++) {
    //Serial.println(headerid[i]);
    recordFile.write(headerid[i]);
  }
  
  Serial.println("header");
  headerdetected = 0;
  mode=3; // continue with detection of startbit

  return;
}

// take average of 100 samples of magnitude values over 0.1s
void signalstrength(uint16_t *avg0, uint16_t *avg1) {

  uint32_t vol0,vol1;
  uint8_t i;

  vol0=0;
  vol1=0;
  // sample 100 magnitude values
  for (i=0;i<100;i++) {
    cli();
    vol0+=mag0;
    sei();
    cli();
    vol1+=mag1;
    sei();
    delay(1);
  }
  // calculate average values
  vol0/=100;
  *avg0=(uint16_t)vol0;
  vol1/=100;
  *avg1=(uint16_t)vol1;
}

// Processor loop
void loop() {

  uint16_t vol0,vol1,noiselevel;
  int i;
   
  /* for debug
  if (record==0) {
    // display data
    for (i=0;i<=255;i++) {
      Serial.print(sig0[i]);
      Serial.print(",");
      Serial.println(sig1[i]);
    }
    delay(1000);
    record=1;
  }
  */

  // write data from decode buffer to serial
  if (headerdetected) writeheader();
  if (writeflag) savedata(0);
  // monitor mode (signal volume detection)
  if (mode==1) {
    signalstrength(&vol0,&vol1);
    if ((vol0>LOWSIGNALLIMIT)&&(vol0>5*vol1)) {
      signalstrength(&vol0,&vol1);
      if ((vol0>LOWSIGNALLIMIT)&&(vol0>5*vol1)) {
        signalthreshold=vol0/2; // divided by 2 to compensate for the signal multiplier of 16 iso 32
        Serial.print(F("Signal detected, 1200bps, threshold [au]:"));
        Serial.println(signalthreshold);
        setupsdft(16); // set for 1200bps
        mode=2; // continue with searching for a header
      }
    }
    else {
      if ((vol1>LOWSIGNALLIMIT)&&(vol1>5*vol0)) {
        signalstrength(&vol0,&vol1);
        if ((vol1>LOWSIGNALLIMIT)&&(vol1>5*vol0)) {
          signalthreshold=vol1/2;
          Serial.print(F("Signal detected, 2400bps, threshold [au]:"));
          Serial.println(signalthreshold);
          //signalthreshold=500;
          mode=2; // continue with searching for a header, system was already set for 2400bps
        }
      }
      else noiselevel=(vol0+vol1)>>1;
    }
  }
  // mode>=15 indicate an error (time-out/bit error).
  if (mode >= 15) {
    Serial.print(F("bit_timeout, mode:"));
    Serial.println(mode);
    // in case of header timeout, assume we are finished. TODO: check motor signal or stop button
    if (mode == 16) {
      savedata(1); // save all remaining data
      recordFile.close(); // close the file
      Serial.println(F("File closed"));
      while (1) {} //hang
    }
    else {
      delay(100);
      // reset and start waiting for a new header
      timeout = 60000; // ~3s timeout, otherwise stop recording
      cnt0 = 0;
      cnt1 = 0;
      mode = 2; //wait for header
    }
  }
}

/*
void headerdetection() {
  if (bit_timeout==255) {
    if (cnt0==255) {
      header0++; // +1 for 1200bps header (2400Hz)
      if (header0>74) {
        headerdetected=1; // detected header for 1200bps
        header0=0;
      }
    }
    else {
      if (cnt1==255) {
        header1++; // +1 for 2400bps header (4800Hz)
        if (header1>74) {
          headerdetected=2; // detected header for 2400bps
          header1=0;
        }
      }
      else { // no proper header, reset all
        header1=0;
        header0=0;
        cnt0=0;
        cnt1=0;
      }
    }
  }
  bit_timeout++;
  timeout--;
}
*/
