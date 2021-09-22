# msx_tape_in
Arduino code to record tape data from MSX
Use the Arduino as a tape deck for the MSX. 
Records and saves as .cas on a SD-card.

It samples pin A0 at approx 19.2 kHz and decodes the tape signal from the MSX to a .cas file.
Both 1200 bps and 2400 bps are supported.
This is still in an experimental state. At 1200 bps, recording is more reliable than at 2400 bps.

