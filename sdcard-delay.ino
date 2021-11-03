// Test SD card write delays for typical 32k tape files
#include <stdint.h>
// For SDcard
#include <SPI.h>
#include <SD.h>

#ifndef __AVR_ATmega328P__
#error Only ATMega328P based boards are supported.
#endif

#define FILENAMESIZE 13 // 8+3 and the . and the 0 -> 13)
#define CS_PIN 10 // SD card CS pin number
#define BUFSIZE 64

File recordFile;
uint8_t buf[BUFSIZE];
uint32_t write_sum,write_min=65000,write_max=0,cnt,avg;

// fill buffer with random data
void fillbuf() {

  uint8_t i;

  for (i=0;i<BUFSIZE;i++) buf[i]=random(0,256);
}

// Initialization
void setup() {

  uint8_t i;
  uint32_t write_cycles,cycles,dt;
  char szFilename[FILENAMESIZE];

  // set-up serial
  Serial.begin(115200);
  // init sdcard
  if (!SD.begin(CS_PIN)) {
    Serial.println(F("Initialization of sdcard failed!"));
    while (1);  // hang
  }
  Serial.println(F("Ready"));

  // save 10 times a 32k data file and record write times
  for (i=0;i<100;i++) {
    snprintf(szFilename, FILENAMESIZE, "RECORD%02i.CAS", i);
    recordFile = SD.open(szFilename, FILE_WRITE);
    if (!recordFile) {
      Serial.println(F("Error opening file. Halt."));
      while (1); //hang
    }
    Serial.println(i);
    // write 32k of data
    write_cycles=32768/BUFSIZE;
    for (cycles=0;cycles<write_cycles;cycles++) {
      fillbuf();

      dt=micros();
      recordFile.write(buf,BUFSIZE);
      dt=micros()-dt;

      if (dt<write_min) write_min=dt;
      if (dt>write_max) write_max=dt;
    }
    recordFile.close();
  }
  Serial.println(F("Test results"));
  Serial.print(F("Minimum write time [us]:"));
  Serial.println(write_min);
  Serial.print(F("Maximum write time [us]:"));
  Serial.println(write_max);
}

void loop() {
}
