#include "rs232.h"
#include <stdio.h>
#include <time.h>

#define COMPORT			22 		// '/dev/ttyAMA0'
#define BAUDRATE		115200
#define RECEIVE_CHARS	1

unsigned char incomingData[RECEIVE_CHARS];
unsigned char buf[255];

int flush() {
    unsigned char c;
    size_t len;
    while (1) {
        len = RS232_PollComport(COMPORT, &c, 1);
        if ((len < 0) || (len > 1)) {
            printf("error - poll returned %d in readline\n", len);
            return 1;
        } else if (len == 0) {
            // do nothing
        } else if (len == 1) {
            if (c == '\n') {
                return 0;
            }
        }
    }
}

int readline(unsigned char* buf, int maxlen) {
    unsigned char c;
    int nc = 0;
    size_t len;
    while (1) {
        len = RS232_PollComport(COMPORT, &c, 1);
        if (len < 0) {
            printf("error - poll returned code %d in readline\n", len);
            return 1;
        } else if (len > 1) {
            printf("error - read %d > 1 bytes in readline\n", len);
            return 1;
        } else if (len == 1) {
            //printf(" %d : %c\n", nc, c);
            buf[nc] = c;
            nc++;
            if (c == '\n') {
                buf[nc] = '\0';
                return nc;
            } else if (nc >= maxlen) {
                printf("error - read %d bytes with no end of line\n", nc);
                return -1;
            }
        }
    }
}

int main()
{    
    // open serial port
    printf("open port\n");
    if (RS232_OpenComport(COMPORT, BAUDRATE, "8N1")) {
        printf("error while opening port\n");
        return 1;
    }
    
    // flush serial port
    printf("flush port (Rx/Tx)\n");
    RS232_flushRXTX(COMPORT);
    printf("read until newline\n");
    if (flush() != 0) {
        return 1;
    }
    
    // read lines from port
    int iters = 400;
    int result = 0;
    unsigned char line[iters][255];
    clock_t tstart = clock();
    for (int i=0; i<iters; ++i) {
        result = readline(line[i], 255);
        if (result < 0) {
            printf("error while reading from port: %d\n", result);
            return 1;
        }
    }
    clock_t tstop = clock();
    
    // print what was read to the terminal
    for (int i=0; i<iters; ++i) {
        printf(" %4d (%d bytes) : %s", i, strlen((char*) line[i]), line[i]);
    }
    
    // find rate
    float tdiff = ((float) (tstop - tstart)) / (float) CLOCKS_PER_SEC;
    float rate = ((float) iters) / tdiff;
    printf("====\n  tdiff = %6.2f : rate = %6.2f\n", tdiff, rate);
    
    // close serial port
    printf("close port\n");
    RS232_CloseComport(COMPORT);
    
    // exit
    printf("exit with status 0\n");
    return 0;
}
