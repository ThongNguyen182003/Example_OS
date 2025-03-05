/* Copyright (C) Marko Petrović 2022
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 2 of the License, or 
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
* or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with this program. 
* If not, see <https://www.gnu.org/licenses/>.
*/

/*
Notes about the program:
Replace event19 in line 68 with corresponding keyboard event. (check /proc/bus/input/devices)
Replace number 61 in "input61::numlock" in line 69 with corresponding number (check what folder exists in /sys/class/leds).
*/

#include <unistd.h>
#include <fcntl.h>
#include <linux/uinput.h>
#include <string.h>

void send(int fd, int type, int code, int val)
{
   struct input_event ie;

   ie.type = type;
   ie.code = code;
   ie.value = val;
   
   ie.time.tv_sec = 0;
   ie.time.tv_usec = 0;

   write(fd, &ie, sizeof(ie));
}
int main()
{
	struct uinput_setup usetup;
	int a, b, c;
	struct input_event ie;
	char led;
	
	b = open("/dev/uinput", O_WRONLY | O_NONBLOCK);
	if(b < 0)
		goto End;
	
	ioctl(b, UI_SET_EVBIT, EV_KEY);
	ioctl(b, UI_SET_KEYBIT, BTN_LEFT);
	ioctl(b, UI_SET_KEYBIT, BTN_RIGHT);
	
	ioctl(b, UI_SET_EVBIT, EV_REL);
	ioctl(b, UI_SET_RELBIT, REL_X);
	ioctl(b, UI_SET_RELBIT, REL_Y);
	
	memset(&usetup, 0, sizeof(usetup));
	usetup.id.vendor = 0x1234;
	usetup.id.product = 0x5678;
	strcpy(usetup.name, "Virtual mouse");
	
	ioctl(b, UI_DEV_SETUP, &usetup);
	ioctl(b, UI_DEV_CREATE);
	sleep(1);
	
	a = open("/dev/input/event19", O_RDONLY);
	c = open("/sys/class/leds/input61::numlock/brightness", O_RDONLY);
	while(a > 0)
	{
		read(a, &ie, sizeof(ie));
		if(ie.type == EV_KEY && ie.value != 0)
		{
			if(ie.code == KEY_ESC)
				break;
			lseek(c, 0, SEEK_SET);
			read(c, &led, 1);
			if(led == '0')
			{
				if(ie.code == KEY_KP8)
				{
					send(b, EV_REL, REL_Y, -5);
					send(b, EV_SYN, SYN_REPORT, 0);
				}
				if(ie.code == KEY_KP5)
				{
					send(b, EV_REL, REL_Y, 5);
					send(b, EV_SYN, SYN_REPORT, 0);
				}
				if(ie.code == KEY_KP4)
				{
					send(b, EV_REL, REL_X, -5);
					send(b, EV_SYN, SYN_REPORT, 0);
				}
				if(ie.code == KEY_KP6)
				{
					send(b, EV_REL, REL_X, 5);
					send(b, EV_SYN, SYN_REPORT, 0);
				}
				if(ie.code == KEY_KP7)
				{
					send(b, EV_KEY, BTN_LEFT, 1);
					send(b, EV_SYN, SYN_REPORT, 0);
					send(b, EV_KEY, BTN_LEFT, 0);
					send(b, EV_SYN, SYN_REPORT, 0);
					
				}
				if(ie.code == KEY_KP9)
				{
					send(b, EV_KEY, BTN_RIGHT, 1);
					send(b, EV_SYN, SYN_REPORT, 0);
					send(b, EV_KEY, BTN_RIGHT, 0);
					send(b, EV_SYN, SYN_REPORT, 0);
				}
			}
		}
	}
	
	sleep(1);
	ioctl(b, UI_DEV_DESTROY);
	
	close(a);
	close(b);
	End:
	
	return 0;
}
