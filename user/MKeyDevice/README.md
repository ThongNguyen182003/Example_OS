# MKeyDevice
MKeyDevice is a little userspace program for Linux that allows using mouse keys on keypad for navigating mouse on the screen. I built it while experimenting with /dev/uinput interface so I decided that I may post it here in my first repository.

# Installation and using
At the moment, entire program is composed of just one file - MKeyDevice.c so you should download it, change paths in lines 68 and 69 to correspond to ones on your computer(see the note in file MKeyDevice.c) and compile it(gcc MKeyDevice.c -o MKeyDevice). After compilation, run the program as root and you should be able to use mouse keys on keypad for moving mouse cursor while num lock is off. You can temporarily disable the mouse keys by turning on the num lock.

# Default controls
Default controls for mouse are:
  8 - moving up;
  4 - moving left;
  5 - moving down;
  6 - moving right;
  7 - left click;
  9 - right click
