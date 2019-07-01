//
// Use of this file is governed by the MIT License - see adept_fc/LICENSE_MIT
//
// Copyright (c) 2019 Timothy Bretl, Aaron Perry, and Phillip Ansell
//

#include "rs232.h"
#include <stdio.h>
#include <stdlib.h>
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

bool is_valid_line(unsigned char* line) {
    // it has non-zero length
    unsigned int len = strlen((char*) line);
    if (len == 0) {
        printf("error: line has zero length\n%s\n", line);
        return false;
    }

    // it starts with '$'
    if (line[0] != '$') {
        printf("error: line starts with %c and not with $\n%s\n", line[0], line);
        return false;
    }

    // it has an '*'
    bool has_asterix = false;
    unsigned int i;
    unsigned char cs8 = 0;
    for (i = 1; i < len; ++i) {
        if (line[i] == '*') {
            has_asterix = true;
            break;
        }
        cs8 ^= line[i];
    }
    if (! has_asterix) {
        printf("error: line contains no *\n%s\n", line);
        return false;
    }

    // it has the correct checksum
    unsigned long a = strtoul((char*) line + i + 1, NULL, 16);
    unsigned long b = (unsigned long) cs8;
    if (a != b) {
        printf("error: line checksum does not match (%lu, %lu)\n%s\n", a, b, line);
        return false;
    }

    return true;
}

void parseline(unsigned char* line) {
    // Check that line has correct length for VNINS
    const int len_expected = 142;
    int len = strlen((char*) line);
    if (len != len_expected) {
        printf("error: line has length %d, should be %d\n", len, len_expected);
        return;
    }

    // Terminate line at *
    line[len_expected - 5] = '\0';

    // Get each field
    struct ins {
        double time;
        unsigned long week;
        bool tracking;
        bool gpsfix;
        bool error;
        float yaw;          // degrees
        float pitch;        // degrees
        float roll;         // degrees
        double latitude;    // degrees
        double longitude;   // degrees
        double altitude;
        float vx;
        float vy;
        float vz;
        float attuncertainty;
        float posuncertainty;
        float veluncertainty;
    };
    struct ins data;
    char* field;
    field = strtok((char*) line + 1, ",");
    printf("%s\n", field);
    field = strtok(NULL, ",");
    data.time = strtod(field, NULL);
    printf("%s (%f)\n", field, data.time);
    field = strtok(NULL, ",");
    data.week = strtoul(field, NULL, 0);
    printf("%s (%lu)\n", field, data.week);
    field = strtok(NULL, ",");
    unsigned long tmp = strtoul(field, NULL, 16);
    bool bits[16];
    for (int i=0; i<16; ++i) {
        bits[i] = (tmp & 1);
        tmp >>= 1;
    }
    data.tracking = (bits[0] && (! bits[1]));
    data.gpsfix = bits[2];
    data.error = (bits[3] || bits[4] || bits[5] || bits[6]);
    printf("%s (%d, %d, %d)\n", field, data.tracking, data.gpsfix, data.error);
    field = strtok(NULL, ",");
    data.yaw = strtod(field, NULL);
    field = strtok(NULL, ",");
    data.pitch = strtod(field, NULL);
    field = strtok(NULL, ",");
    data.roll = strtod(field, NULL);
    printf("%f, %f, %f\n", data.yaw, data.pitch, data.roll);
    field = strtok(NULL, ",");
    data.latitude = strtod(field, NULL);
    field = strtok(NULL, ",");
    data.longitude = strtod(field, NULL);
    field = strtok(NULL, ",");
    data.altitude = strtod(field, NULL);
    field = strtok(NULL, ",");
    data.vx = strtod(field, NULL);
    field = strtok(NULL, ",");
    data.vy = strtod(field, NULL);
    field = strtok(NULL, ",");
    data.vz = strtod(field, NULL);
    field = strtok(NULL, ",");
    data.attuncertainty = strtod(field, NULL);
    field = strtok(NULL, ",");
    data.posuncertainty = strtod(field, NULL);
    field = strtok(NULL, ",");
    data.veluncertainty = strtod(field, NULL);
    printf("%f\n", data.veluncertainty);

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
    int iters = 4;
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
        printf(" %4d (%d bytes : %s) : %s", i, strlen((char*) line[i]), is_valid_line(line[i]) ? "valid" : "invalid", line[i]);
    }

    // parse each line
    for (int i=0; i<iters; ++i) {
        if (is_valid_line(line[i])) {
            parseline(line[i]);
        }
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
