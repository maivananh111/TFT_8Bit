# TFT_8Bit
Library for TFT LCD 8 bit wire bus without FSMC

Create an lcd object with the screen size and timer parameters to be used for delay. Timer must be installed to be able to delay 1us.
Before the while(1) function, read the ID(display controller) and store it in a variable, then call the lcd constructor with the input
parameter being the ID of the screen controller just read. The font used in the library uses MikroElektronika's GLCD Font Creator software.
change the pin configuration of the microcontroller connected to the display and the write_8 and read_8 functions in the Setting.h file to match the hardware connection.

It is done.

