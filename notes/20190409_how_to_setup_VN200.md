# Setup

Make sure UART is enabled with this line in `/boot/config.txt`:
```
enable_uart=1
```

Disable the Bluetooth device and restore `ttyAMA0` to GPIOs 14 and 15 by adding this line to `/boot/config.txt`:
```
dtoverlay=pi3-disable-bt
```

Run this command in a terminal to disable the system service that initializes the Bluetooth modem:
```
sudo systemctl disable hciuart
```

See [here](https://www.raspberrypi.org/documentation/configuration/uart.md) for more information about Bluetooth configuration on Rasperry Pi. It is possible to re-enable Bluetooth using the "miniuart", for example.

Delete all references to `serial0` in `/boot/cmdline.txt`. For example, this...
```
dwc_otg.lpm_enable=0 console=serial0,115200 console=tty1 root=PARTUUID=61628a42-02 rootfstype=ext4 elevator=deadline fsck.repair=yes rootwait quiet splash plymouth.ignore-serial-consoles
```
...becomes this:
```
dwc_otg.lpm_enable=0 console=tty1 root=PARTUUID=61628a42-02 rootfstype=ext4 elev
ator=deadline fsck.repair=yes rootwait quiet splash plymouth.ignore-serial-conso
les
```

Reboot.

Confirm that `/dev/serial0` is an alias for `/dev/ttyAMA0`:
```
pi@raspberrypi:~ $ ls -al /dev/serial*
lrwxrwxrwx 1 root root 7 Apr  4 17:02 /dev/serial0 -> ttyAMA0
lrwxrwxrwx 1 root root 5 Apr  4 17:02 /dev/serial1 -> ttyS0
```

Read from the port:
```
pi@raspberrypi:~ $ cat -v < /dev/ttyAMA0
0M-%!*M-5M-dM-/)1CM-^MM-%M-$M-%M-^UM-%1M-yM-^DM-5M--M-^UM-$M-^E)
1
kM-^P!!1!J!1!1M-yM-/)1^K91
!M-%jM-5M-^DM-5)M-^UM-%!M-%5
M-^EM-mM-/)1M-4M-^MM-%M-$M-%M-^UM-%1M-yM-^PM-%M-=*H%!1!M-4M-tM-^DKz9*
!J1
... etc. ...
```

Sources include [here](https://github.com/RPi-Distro/repo/issues/31#issuecomment-220776609) and [here](https://raspberrypihobbyist.blogspot.com/2012/08/raspberry-pi-serial-port.html).


# Programmatic IO

You can test IO with the `serial` module in python. For example, here is a simple read test:
```python
import serial

portname = '/dev/serial0'	# alias to UART after configuration
baudrate = 115200			# factory default for VN-200
timeout = 1 				# read timeout in seconds
write_timeout = 1			# write timeout in seconds

# open serial port
port = serial.Serial(portname, baudrate, timeout=timeout, write_timeout=write_timeout)

# read once
print(port.readline())

# close serial port
port.close()
```

In particular, if the VN-200 is at its factory defaults (configured to send a filtered INS solution at 40Hz), you should see a result like this:
```
b'$VNINS,224159.474991,2048,8085,-149.042,+013.639,+010.915,+40.09623362,-088.20139124,+00204.179,+000.023,+000.007,-000.248,20.5,02.5,0.20*6F\r\n'
```

Note that *you must connect an active antenna to the VN-200 in order to get GPS data*. If you don't connect an active antenna, you should see an error code (e.g., something like `8048` instead of `2048` as the second number after the `$VNINS` header), and everything other than roll/pitch/yaw will be zero.

Here is a simple write test (again, assuming the VN-200 configuration is at factory default):
```python
import serial


portname = '/dev/serial0'
baudrate = 115200

def main():
	# open serial port
	port = serial.Serial(portname, baudrate, timeout=1, write_timeout=1)
	
	# read 10 times
	for i in range(10):
		print(port.readline())
	
	# pause async outputs
	print('$VNASY,0*XX\r\n'.encode())
	port.write('$VNASY,0*XX\r\n'.encode())
	
	# read 5 times (should timeout each time)
	for i in range(5):
		print(port.readline())
	
	# resume async outputs
	port.write('$VNASY,1*XX\r\n'.encode())
	
	# read 10 times
	for i in range(10):
		print(port.readline())
	port.close()

if __name__ == "__main__":
	main()
```

Note the importance of starting the header with `$` and of ending each line with `\r\n` (this is "carriage return" + "line feed", which is the common convention for a new line character in Windows).
