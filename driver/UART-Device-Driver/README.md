To check if everything is working:

* Find the uart port: `ls /dev/ttyUSB*`
* Connect to uart port on development machine: `tio /dev/ttyUSB1 -b 115200`
* Reload LKM: `watch "rmmod serial.ko && insmod serial.ko"`
