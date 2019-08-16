#!/bin/bash

#reboot RC_in and PWM_out when power is cycled during runtime: 
if test -f "/home/pi/adept-fc/running"; then
	/home/pi/adept-fc/bin/rc_in  
fi
