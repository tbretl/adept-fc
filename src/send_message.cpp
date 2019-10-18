#include <iostream>
#include <iomanip>
//#include <string.h>
#include <stdio.h>
#include <cstdio>

#include <fcntl.h> // Contains file controls like O_RDWR
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

using namespace std;

#define BAUDRATE115200 B115200
#define PORT "/dev/ttyUSB0"

int main()
{
    // Open port0
    int port0 = open(PORT, O_RDWR | O_NOCTTY | O_SYNC);

    struct termios tty;
    memset(&tty, 0, sizeof tty);

    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);

    tcsetattr(port0, TCSANOW, &tty);

    while(true){
        // Define msg
        char msg[] = {'H', 'e', 'l', 'l', 'o', ' ', 'w', 'o', 'r', 'l', 'd', '!','\0'};

        // Write string to port0, allocating size
        write(port0, msg, sizeof(msg));

        usleep(1000000);
    }

    return 0;
}