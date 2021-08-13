// ONLY for 16MHz 328p based systems
#include <stdint.h>
#include <math.h>

//#define FS 19200  //sample frequency for ADC
#define DFTBUFSIZE 16  // linked to FS and the frequencies to detect TODO: calculate from other parameters
#define PI 3.141593
#define DIVISOR 32768U  //for use with int16_t --> [-1,1)
#define THRESHOLD 1500U // threshold for signal/ no signal in magnitude value of dft bin, experimental value TODO: how to determine?
#define DECODEBUFSIZE 32 // should be big enough to buffer for 250ms of data (SD card write latency) and even () 128

// gcc avr code for 16*16->high 16 bit signed fractional multiply, mofified from AVR201 assembly for fixed point math: 1.15 * 1.15 -> 1.15
#define fmuls16x16_16h(result, multiplicand, multiplier) \
__asm__ __volatile__ ( \
"  clr r2 \n\t" \
" fmuls %B2, %B1 \n\t" /* (signed)ah * (signed)bh*/ \
" movw  %A0, r0 \n\t" \
" fmulsu  %B2, %A1 \n\t" /* (signed)ah * bl*/ \
" sbc %B0, r2 \n\t" \
" add %A0, r1 \n\t" \
" adc %B0, r2 \n\t" \
" fmulsu  %B1, %A2 \n\t" /* (signed)bh * al*/ \
" sbc %B0, r2 \n\t" \
" add %A0, r1 \n\t" \
" adc %B0, r2 \n\t" \
" clr r1 \n\t" \
: "=&a" (result) \
: "a" (multiplicand),  "a" (multiplier) \
: "r2" \
);

//volatile uint16_t mag0,mag1;
volatile int16_t samplebuf[DFTBUFSIZE],bin0_re,bin0_im,bin1_re,bin1_im,fact0_re,fact0_im,fact1_re,fact1_im,tmp1_re,tmp2_re,tmp1_im,tmp2_im;
volatile uint32_t timeout=400000; // ~20s for timeout on header, default
volatile uint8_t sampleindex,mode,bitnr,byte_val,decodebuf[DECODEBUFSIZE],decodebufindex,decodebufpos,cnt0,cnt1,writeflag,bufpart,bit_timeout,record;
uint8_t N;

// for graphing
volatile int16_t mag0_a[256],mag1_a[256];
volatile uint8_t aindex,stop_int,startbit,headerdetected;

// Initialization of ADC for auto trigger, 1.1V ref and pin A0, 19.2 kHz
void setupADC() {

  pinMode(A0,INPUT);
  analogReference(INTERNAL); 
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
 
  // Enable global interrupts
  // AVR macro included in <avr/interrupts.h>, which the Arduino IDE
  // supplies by default.
  //sei();
 
  // Kick off the first ADC
  //readFlag = 0;
  // Set ADSC in ADCSRA (0x7A) to start the ADC conversion
  ADCSRA |=B01000000;
}

// converts float to 16 bit fixed point int (*DIVISOR), 1.15
int16_t float2int(float x) {

    int32_t tmp;

    tmp=x*DIVISOR;
    // check limits
    if (tmp<-32768) tmp=-32768;  // minimum is -1
    if (tmp>32767) tmp=32767;  // maximum is 0.9999 (not 1!)
    return (int16_t)tmp;
}

// converts 16 bit fixed point int to float (/DIVISOR) 
float int2float(int16_t x) {

    return (float)x/DIVISOR;
}

void dftcoefficient(uint8_t m, uint8_t N, volatile int16_t *coef_re, volatile int16_t *coef_im) {

    float re,im;

    re=cos(2*PI*m/(float)N);
    im=sin(2*PI*m/(float)N);
    // for stability, multiply largest component with dampening factor
    if (abs(re)>abs(im)) re*=0.9998; // 0.9975 (0.9998) 0.998
    if (abs(im)>abs(re)) im*=0.9998;
    *coef_re=float2int(re);
    *coef_im=float2int(im);

    return;
}

// num must be 2^i and smaller or equal to DFTBUFSIZE
void setupsdft(uint8_t num) {
    
    uint8_t i;

    N=num;  //N-bin dft
    sampleindex=0;
    mode=0;  // wait for header
    decodebufindex=0;
    // empty buffer
    for (i=0;i<num;i++) samplebuf[i]=0;
    // setup constants for sdft coef=exp(i*2*PI*m/N)
    // for dft bin 1 ('0' value)
    dftcoefficient(1,num,&fact0_re,&fact0_im);
    // for dft bin 2 ('1' value)
    dftcoefficient(2,num,&fact1_re,&fact1_im);
    return;
}

// sliding window dft, 2 bins for either 1200 & 2400 Hz (N=16) or 2400 & 4800 Hz (N=8) at Fs 19200 Hz.
// magnitude approx by adding abs value instead of sqrt(re^2+im^2) for speed.
// sdft algorithm Xm(n)=exp(i*2*pi*m/N)*[Xm(n-1)+x(n)-x(n-N)]
ISR(ADC_vect){

    int16_t xin,xout,mag0,mag1;

    //if (stop_int) return;

    // get ADC signal
    xin=ADCL | (ADCH << 8);
    //multiply for signal resolution
    xin*=16;

    //swap samples in and out of buffer for sdft
    xout=samplebuf[sampleindex];
    samplebuf[sampleindex]=xin;
    sampleindex++;
    sampleindex&=(N-1);
    //sampleindex&=15;

    //real additions
    bin0_re+=xin;
    bin1_re+=xin;
    bin0_re-=xout;
    bin1_re-=xout;

    //complex multiply fact*f_bin, temp vars to avoid interaction
    // for '0'
    // real part
    fmuls16x16_16h(tmp1_re,fact0_re,bin0_re);
    fmuls16x16_16h(tmp2_re,fact0_im,bin0_im);
    //tmp_re=((int32_t)fact0_re*bin0_re-(int32_t)fact0_im*bin0_im)/DIVISOR;
    // imag part
    fmuls16x16_16h(tmp1_im,fact0_re,bin0_im);
    fmuls16x16_16h(tmp2_im,fact0_im,bin0_re);
    //tmp_im=((int32_t)fact0_re*bin0_im+(int32_t)fact0_im*bin0_re)/DIVISOR;
    bin0_re=tmp1_re-tmp2_re;
    bin0_im=tmp1_im+tmp2_im;
    // bin0_re=tmp_re;
    // bin0_im=tmp_im;

    // for '1'
    // real part
    fmuls16x16_16h(tmp1_re,fact1_re,bin1_re);
    fmuls16x16_16h(tmp2_re,fact1_im,bin1_im);
    // tmp_re=((int32_t)fact1_re*bin1_re-(int32_t)fact1_im*bin1_im)/DIVISOR;
    // imag part
    fmuls16x16_16h(tmp1_im,fact1_re,bin1_im);
    fmuls16x16_16h(tmp2_im,fact1_im,bin1_re);
    // tmp_im=((int32_t)fact1_re*bin1_im+(int32_t)fact1_im*bin1_re)/DIVISOR;
    bin1_re=tmp1_re-tmp2_re;
    bin1_im=tmp1_im+tmp2_im;
    // bin1_re=tmp_re;
    // bin1_im=tmp_im;

    // magnitude estimate
    mag0=abs(bin0_re)+abs(bin0_im);
    mag1=abs(bin1_re)+abs(bin1_im);
    if ((mag0>mag1) && (mag0>THRESHOLD)) cnt0++;
    if ((mag1>mag0) && (mag1>THRESHOLD)) cnt1++;
//    if (mag0>(mag1+1500)) cnt0++;
//    if (mag1>(mag0+1500)) cnt1++;

    // build graph for stability analysis
//    if (startbit) {
    if (record==1) {
      mag0_a[aindex]=mag0;
      mag1_a[aindex]=mag1;
      aindex++;
//      if (aindex==0) stop_int=1;
      if (aindex==0) record=2;
    }

    // decoding MSX tape: header,data,header,data...
    switch (mode) {
        case 0: // wait for header
//            timeout--;
//            if (timeout==0) {
//                mode=7; //timeout error for header
//                break;
//            }
            if (cnt0>0) {
                cnt0=0;
                cnt1=0;
                break;
            }
            if (cnt1==255) {
                mode=1;
                cnt1=0;
                cnt0=0;
                //headerdetected=1;
                timeout=200000; // wait for ~10s for the first startbit
            }
            break;
        case 1: // wait for first startbit after header
            timeout--;
            if (timeout==0) {
                mode=8; //timeout error for first startbit
                break;
            }
            if (cnt1>0) {
                cnt1=0;
                cnt0=0;
                break;
            }
            if (cnt0>11) { // was 11
                //startbit=1;
                mode=3;
                bit_timeout=4; // 16 samples - 12 (startbit)=4
            }
            break;
        case 2: // wait for normal startbit
            bit_timeout--;
            if (bit_timeout==0) {
                mode=9; //bit_timeout error for startbit
                break;
            }
            if (cnt1>0) {
                cnt1=0;
                cnt0=0;
                break;
            }
            if (cnt0>10) { // was 11
                //startbit=1;
                mode=3;
                bit_timeout=5; // 16 samples - 12 (startbit)=4
            }
            break;
        case 3: // delay before byte decoding
            bit_timeout--;
            if (bit_timeout==0) {
                mode=4;
                cnt0=0;
                cnt1=0;
                bitnr=0;
                byte_val=0;
                bit_timeout=16; // 1 bit
            }
            break;
        case 4: // decoding the byte value
            bit_timeout--;
            if (bit_timeout==0) {
                if (cnt1>cnt0) byte_val+=(1<<bitnr);
                bitnr++;
                cnt0=0;
                cnt1=0;
                if (bitnr>7) {
                    mode=5;
                    bit_timeout=32; // 2 stopbits (32)
                    break;
                }
                bit_timeout=16; // next (1 bit)
            }
            break; 
        case 5: // 2 stopbits
            bit_timeout--;
            if (bit_timeout==0) {
                mode=10; // stopbit error
                break;
            }
            if (cnt0>0) {
                cnt0=0;
                cnt1=0;
                break;
            }
            if (cnt1>23) { // was 23
                decodebuf[decodebufpos]=byte_val;
                decodebufpos++;
                if (decodebufpos==DECODEBUFSIZE/2) {
                  bufpart=0;
                  writeflag=1;
                }
                if (decodebufpos==DECODEBUFSIZE) {
                  bufpart=1;
                  writeflag=1;
                  decodebufpos=0;
                }
                cnt0=0;
                cnt1=0;
                bit_timeout=24; // was 24
                mode=2; // next byte, startbit
            }
            break;
    }
    return;
}

// Initialization
void setup() {
    
    // set-up serial
    Serial.begin(115200);
    // init sdft for 1200bps
    setupsdft(16);
    // init ADC 19.2 kHz auto-trigger
    setupADC();
    // enable recording
    record=1;
    return;
}

// Processor loop
void loop() {

    uint16_t i;

//    Serial.print(cnt0);
//    Serial.print(",");
//    Serial.println(cnt1);
//    delay(50);
    
//    if (mode>5) {
//      Serial.print("bit_timeout, mode:");
//      Serial.println(mode);
//      delay(150);
//      //setupsdft(16);
//      timeout=400000; // 20s
//      cnt0=0;
//      cnt1=0;
//      mode=0;
//    }
//
//    if (writeflag) {
//      for (i=0;i<DECODEBUFSIZE/2;i++) {
//        Serial.println(decodebuf[i+bufpart*(DECODEBUFSIZE/2)]);
//      }
//      writeflag=0;
//    }

//  if (stop_int) {
  if (record==2) {
    for (i=0;i<=255;i++) {
      Serial.print(mag0_a[i]);
      Serial.print(",");
      Serial.println(mag1_a[i]);
    }
    record=1;
    //while(1) {}
  }
}
