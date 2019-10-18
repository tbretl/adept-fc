#include <iostream>
#include <iomanip>
#include <string.h>
#include <stdio.h>
#include <cstdio>

#include <fcntl.h> // Contains file controls like O_RDWR
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

using namespace std;

#define BAUDRATE115200 B115200
#define PORT "/dev/ttyUSB2"

int main()
{

    //Open port1
    int port0 = open(PORT, O_RDWR | O_NOCTTY | O_SYNC);
    
    struct termios tty;
    memset(&tty, 0, sizeof tty);

    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);

    tcsetattr(port0, TCSANOW, &tty);

    while(true){

        //Allocate memory for read buffer
        char buf[255];

        //Read msg from port1
        read(port0, buf, sizeof(buf));

        //Allocate memory for sprintf buffer
//        char print_buf[255];

        //Print msg from buf onto print_buf
//        sprintf(print_buf, "%s", buf);

        //Show msg
        cout << buf << endl;

        usleep(1000000);
    }

    return 0;
}