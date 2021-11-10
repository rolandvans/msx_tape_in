// read tape data from MSX via opamp interface circuit. save data to sdcard
// First experimental version, for testing purposes only!
// Should work on many boards (no specific machine code/registers are used)
#include <stdint.h>
// For SDcard
#include <SPI.h>
#include <SD.h>
// for file numbering
#include <EEPROM.h>

#define INTPIN 2 // pin for external interrupt (2 or 3)
#define MININTERVAL 100 // minimum time between two rising flanks on INTPIN (otherwise data is invalid) in µs
#define MAXINTERVAL 1800 // maximum time between two rising flanks on INTPIN in µs
#define HEADERCOUNT 512 // number of rising flanks in the HEADER part (min 32)
#define HEADERVAR 64 // maximum difference allowed between header pulses in µs
#define DECODEBUFSIZE 128 // should be big enough to buffer for 250ms of data at Fs (SD card write latency) and even. default: 128, for SDHC cards (500ms), use 220
#define FILENAMESIZE 13 // 8+3 and the . and the 0 -> 13)
#define CS_PIN 10 // SD card CS pin number

volatile uint16_t headercount,headermin,headermax,bit0min,bit0max,bit1min,bit1max,interval;
volatile uint8_t byte_val,bitcnt,bytecnt,mode,headerdetected,halfbit,validbit,decodebufpos,writeflag,bufpart;
volatile uint8_t decodebuf[DECODEBUFSIZE];
volatile uint32_t timestamp, headertimesum;
const uint8_t headerid[] = {0x1F, 0xA6, 0xDE, 0xBA, 0xCC, 0x13, 0x7D, 0x74}; // MSX .cas code for a header, must by 8-byte aligned in .cas file
File recordFile;
uint32_t starttime,timeout;

void resetall() {

  decodebufpos=0;
  headerdetected=0;
  halfbit=0;
  headercount=0;
  timestamp=0;
  headertimesum=0;
  headermin=65535;
  headermax=0;
  starttime=micros();
}

void ISRPulse() {

  uint32_t dt;

  if (mode==0) return;
  
  // is it the first pulse? -> set timer to zero and exit
  if (timestamp==0) {
    timestamp=micros();
    return;
  }
  
  // calculate interval, for a 1200Hz cycle, this is ~833µs
  dt=micros()-timestamp;
  timestamp=micros();

  if ((dt<MININTERVAL) || (dt>MAXINTERVAL)) {
    mode=15; // interval out of range
    return;
  }
  interval=(uint16_t)dt;
  
  // decoding MSX tape: header,data,header,data...
  switch (mode) {
    case 1: // wait for proper header
      headertimesum+=interval;
      headercount++;
      if (interval<headermin) headermin=interval; // record min and max pulsewidth
      if (interval>headermax) headermax=interval;
      
      if (headercount>=HEADERCOUNT) { // have we received the entire header?
        if ((headermax-headermin)>HEADERVAR) { // are the pulses within the boundaries?
          mode=16; // header error
          return;
        }
        bit1min=headertimesum/(HEADERCOUNT+(HEADERCOUNT/10)); // add +/- 10% boundaries on pulse timing for '1' and '0'
        bit1max=headertimesum/(HEADERCOUNT-(HEADERCOUNT/10));
        bit0min=bit1min*2;
        bit0max=bit1max*2;
        headerdetected=1;
        mode=2; // wait for startbit
      }
      break;
    case 2: // wait for first startbit after header
      if ((interval>bit0min) && (interval<bit0max)) { // yes, we have a startbit, continue with decoding the byte
        mode=3;
        halfbit=0;
        bitcnt=0;
      }
      break;
    case 3: // decode byte
      validbit=0;
      if ((interval>bit1min) && (interval<bit1max)) { // do we have a valid '1'? a '1' consists of 2 pulses.
        if (halfbit==0) {
          halfbit=1;
          return;
        }
        bitSet(byte_val,bitcnt);
        halfbit=0;
        validbit=1;
      }
      if ((interval>bit0min) && (interval<bit0max) && (!halfbit)) { // do we have a valid '0'?
        bitClear(byte_val,bitcnt);
        validbit=1;
      }
      if (validbit) { // in case of a valid bit, move to next bit or byte
        bitcnt++;
        if (bitcnt>7) { // continue with stopbits (2)
          mode=4;
          bitcnt=2;
          //halfbit=0;
        }
        break;
      }
      mode=17; // invalid bit in byte detected
      break;
    case 4: // 2 stopbits
      if ((interval>bit1min) && (interval<bit1max)) { // do we have a valid '1'? a '1' consists of 2 pulses.
        if (halfbit==0) {
          halfbit=1;
          return;
        }
        bitcnt--;
        halfbit=0;
        if (bitcnt==0) {// we have received a byte, store and continue for next byte
          decodebuf[decodebufpos] = byte_val;
          decodebufpos++;
          if (decodebufpos == DECODEBUFSIZE / 2) {
            if (writeflag) {
              mode = 21; // buffer overrun
              break;
            }
            bufpart = 0;
            writeflag = 1;
          }
          if (decodebufpos == DECODEBUFSIZE) {
            if (writeflag) {
              mode = 21; // buffer overrun
              break;
            }
            bufpart = 1;
            writeflag = 1;
            decodebufpos = 0;
          }
          mode=5; // startbit
          break;
        }
        break;
      }
      mode=18; // no proper stopbit
      break;
    case 5: // startbit
      if ((interval>bit0min) && (interval<bit0max)) { // yes, we have a startbit, continue with decoding the byte
        mode=3;
        halfbit=0;
        bitcnt=0;
        break;
      }
      mode=19; // no startbit
      break;
    }
  return;
}

// save full part of the buffer (option=0) or all remaining data (option!=0)
void savedata(uint8_t option) {

  uint16_t i, start;
  uint32_t dt;

  dt = micros();
  if (writeflag) {
    recordFile.write((uint8_t*)decodebuf+bufpart*(DECODEBUFSIZE/2),(DECODEBUFSIZE/2));
    writeflag = 0;
  }
  if (option) {
    if ((decodebufpos == 0) || (decodebufpos == (DECODEBUFSIZE / 2))) return; // no additional data present
    // write last data portion from buffer
    if (decodebufpos > (DECODEBUFSIZE / 2)) start = (DECODEBUFSIZE / 2); else start = 0;
    recordFile.write((uint8_t*)decodebuf+start,decodebufpos-start);
    // reset buffer position to initial state, avoid writing data multiple times
    decodebufpos = 0;
    bufpart = 0;
  }
  dt = micros() - dt;

  Serial.print(F("Write time [µs]:"));
  Serial.println(dt);

  return;
}

void writeheader() {

  uint8_t i;
  uint8_t zeros[]={0,0,0,0,0,0,0}; // 7 zeros for padding/aligning the .cas header
  uint8_t zeropadding;
  uint32_t dt;
  char szFilename[FILENAMESIZE];

  // if no file is open: create one.
  if (!recordFile) {
    // read and update eeprom to get file number
    i = EEPROM.read(0);
    // update value (0-99), reset on overflow
    i++;
    if (i >= 100) i = 0;
    snprintf(szFilename, FILENAMESIZE, "RECORD%02i.CAS", i);
    dt = micros();
    recordFile = SD.open(szFilename, FILE_WRITE);
    dt = micros() - dt;
    if (!recordFile) {
      Serial.println(F("Error opening file. Halt."));
      while (1); //hang
    }
    Serial.print(F("Open file time [µs]:"));
    Serial.println(dt);

    // succesful file creation, update eeprom
    EEPROM.write(0, i);
    Serial.print(F("Filename:"));
    Serial.println(szFilename);
  }

  savedata(1); // ensure all available data is saved before the header is written (in case this is not the first header)

  // Align header at 8-byte boundaries, pad with zeros (1-7)
  zeropadding = (uint8_t)(recordFile.position() & 7);
  zeropadding ^= 7;
  zeropadding++;
  zeropadding &= 7;
  dt = micros();
  // write padding zero's if they exist
  if (zeropadding) recordFile.write(zeros,zeropadding);
  // write msx .cas header
  recordFile.write(headerid,sizeof(headerid));
  dt = micros() - dt;

  Serial.println(F("header"));
  headerdetected = 0;
  //mode = 2; // continue with detection of startbit

  Serial.print(F("Write time header [µs]:"));
  Serial.println(dt);

  return;
}


void setup() {
  // put your setup code here, to run once:
  pinMode(INTPIN, INPUT);
  // setup serial com
  Serial.begin(115200);
  Serial.println(F("MSX"));
  // init sdcard
  if (!SD.begin(CS_PIN)) {
    Serial.println(F("Initialization of sdcard failed!"));
    while (1);  // hang
  }
  resetall();
  // setup INT handler for pulses
  attachInterrupt(digitalPinToInterrupt(INTPIN), ISRPulse, RISING);
  mode=1; // all ready to go;
}

void loop() {

  if ((timestamp==0) && (mode==1)) { // anything detected at all?
    timeout=micros()-starttime;
    if (timeout>20000000UL) {//~20s
      mode=20;
      Serial.println(F("No signal"));
    }
  }
  // check for timeout of pulses
  if ((timestamp) && (mode<15)) {
    cli();
    timeout=micros()-timestamp;
    sei();
    if (timeout>3000000UL) {//~3s
      Serial.print(F("Error:"));
      Serial.println(mode);
      Serial.println(F("No more data"));
      savedata(1); // save all remaining data
      recordFile.close();
      while (1) ; //hang
    }
  }

  if (headerdetected) writeheader();
  if (writeflag) savedata(0);
  if (mode >= 15) { // no signal detected, just try again
    Serial.print(F("Error:"));
    Serial.println(mode);
    savedata(1); // save all remaining data
    //recordFile.close();
    //while (1) ; //hang
    delay(100);
    resetall();
    // reset and start waiting for a new header
    mode = 1; //wait for header
  }
}
