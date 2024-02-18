# msx_tape_in
Arduino code to record tape data from MSX
Use the Arduino as a tape deck for the MSX. 
Records and saves as .cas on a SD-card.
Only works on 16MHz 328p processors (Arduino Uno, nano and pro mini)

It samples pin A0 at approx 19.2 kHz and decodes the tape signal from the MSX to a .cas file.
Both 1200 bps and 2400 bps are supported.
This is still in an experimental state. At 1200 bps, recording is more reliable than at 2400 bps.

Alternative tape input via small circuit with a dual opamp.
This circuit amplifies the signal from the MSX, and abuses the second opamp as a comparator.

