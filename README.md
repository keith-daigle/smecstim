# smecstim
## AVR based engine simulator

This is the code and board files to build a smecstim which I initially built 9 years ago to bench-test mid 80's chrysler HEP based ecus.  It's got enough features in it to run the ECU pretty well handling baro reads and constraining the outputs of various sensors.  It was based on an attiny84 but any avr with 2 timers should probably be enough to run it (though you may have to adjust the pins).  

The schematic and board files are in the eagle directory.  I'm really fuzzy on the board layout at this point but things seem to be clearly labelled in the eagle files so it should be much of a task to figure out what the BOM is.
