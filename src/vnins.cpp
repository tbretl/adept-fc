#include <zcm/zcm-cpp.hpp>
#include "vnins_data_t.hpp"
#include "status_t.hpp"
#include "rs232.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <string>
#include <iostream>

#define COMPORT			22 		// '/dev/ttyAMA0'
#define BAUDRATE		115200
#define BUFFER_LENGTH 	255
#define MESSAGE_LENGTH  142

using std::string;

class Handler
{
    public:
        ~Handler() = default;

        status_t stat;

        Handler()
        {
            memset(&stat, 0, sizeof(stat));
            stat.should_exit = 0;
        }

        void read_stat(const zcm::ReceiveBuffer* rbuf,const string& chan,const status_t *msg)
        {
            stat = *msg;
        }
};

int flush()
{
    unsigned char c;
    size_t len;
    while (true)
    {
        len = RS232_PollComport(COMPORT, &c, 1);
        if ((len < 0) || (len > 1))
        {
            std::cout << "error - poll returned " << len <<" in readline\n";
            return 1;
        } else if (len == 0)
        {
            // do nothing
        } else if (len == 1)
        {
            if (c == '\n')
            {
                return 0;
            }
        }
    }
}

int readline(unsigned char* buf, int maxlen)
{
    unsigned char c;
    int nc = 0;
    size_t len;
    while (true)
    {
        len = RS232_PollComport(COMPORT, &c, 1);
        if (len < 0)
        {
            std::cout << "error - poll returned code " << len << " in readline\n";
            return -1;
        } else if (len > 1)
        {
            std::cout << "error - read " << len << " > 1 bytes in readline\n";
            return -1;
        } else if (len == 1)
        {
            buf[nc] = c;
            nc++;
            if (c == '\n') {
                buf[nc] = '\0';
                return nc;
            } else if (nc >= maxlen)
            {
                std::cout << "error - read " << nc << " bytes with no end of line\n";
                return -1;
            }
        }
    }
}

bool is_valid_line(unsigned char* line)
{
    // it has non-zero length
    unsigned int len = strlen((char*) line);
    if (len == 0)
    {
        std::cout << "error: line has zero length\n" << line << std::endl;
        return false;
    }

    // it starts with '$'
    if (line[0] != '$')
    {
        std::cout << "error: line starts with " << line[0] << " and not with $\n" << line << std::endl;
        return false;
    }

    // it has an '*'
    bool has_asterix = false;
    unsigned int i;
    unsigned char cs8 = 0;
    for (i = 1; i < len; ++i)
    {
        if (line[i] == '*')
        {
            has_asterix = true;
            break;
        }
        cs8 ^= line[i];
    }
    if (! has_asterix)
    {
        std::cout << "error: line contains no *\n" << line << std::endl;
        return false;
    }

    // it has the correct checksum
    unsigned long a = strtoul((char*) line + i + 1, NULL, 16);
    unsigned long b = (unsigned long) cs8;
    if (a != b)
    {
        std::cout << "error: line checksum does not match (" << a << "," << b << ")\n" << line << std::endl;
        return false;
    }

    return true;
}

int parseline(unsigned char* line, vnins_data_t *msg)
{
    // Check that line has correct length for VNINS
    const int len_expected = MESSAGE_LENGTH;
    int len = strlen((char*) line);
    if (len != len_expected)
    {
        std::cout << "error: line has length "<< len << ", should be " << len_expected << std::endl;
        return 1;
    }

    // Terminate line at *
    line[len_expected - 5] = '\0';

    // Get each field
    char* field;
    field = strtok((char*) line + 1, ",");
    field = strtok(NULL, ",");
    msg->time = strtod(field, NULL);
    field = strtok(NULL, ",");
    msg->week = (long long) strtoul(field, NULL, 0);
    field = strtok(NULL, ",");
    unsigned long tmp = strtoul(field, NULL, 16);
    bool bits[16];
    for (int i=0; i<16; ++i)
    {
        bits[i] = (tmp & 1);
        tmp >>= 1;
    }
    msg->tracking = (bits[0] && (! bits[1]));
    msg->gpsfix = bits[2];
    msg->error = (bits[3] || bits[4] || bits[5] || bits[6]);
    field = strtok(NULL, ",");
    msg->yaw = strtod(field, NULL);
    field = strtok(NULL, ",");
    msg->pitch = strtod(field, NULL);
    field = strtok(NULL, ",");
    msg->roll = strtod(field, NULL);
    field = strtok(NULL, ",");
    msg->latitude = strtod(field, NULL);
    field = strtok(NULL, ",");
    msg->longitude = strtod(field, NULL);
    field = strtok(NULL, ",");
    msg->altitude = strtod(field, NULL);
    field = strtok(NULL, ",");
    msg->vx = strtod(field, NULL);
    field = strtok(NULL, ",");
    msg->vy = strtod(field, NULL);
    field = strtok(NULL, ",");
    msg->vz = strtod(field, NULL);
    field = strtok(NULL, ",");
    msg->attuncertainty = strtod(field, NULL);
    field = strtok(NULL, ",");
    msg->posuncertainty = strtod(field, NULL);
    field = strtok(NULL, ",");
    msg->veluncertainty = strtod(field, NULL);

    return 0;
}

int main()
{
    // open serial port
    if (RS232_OpenComport(COMPORT, BAUDRATE, "8N1"))
    {
        std::cout << "error while opening port" << std::endl;
        return 1;
    }

    // flush serial port
    RS232_flushRXTX(COMPORT);
    if (flush() != 0)
    {
        return 1;
    }

    // initialize zcm
    zcm::ZCM zcm {"ipc"};

    //subscribe
    Handler handlerObject;
    zcm.subscribe("STATUS",&Handler::read_stat,&handlerObject);

    // create objects to publish
    vnins_data_t msg;
    memset(&msg,0,sizeof(msg));

    status_t module_stat;
    memset(&module_stat,0,sizeof(module_stat));
    module_stat.module_status = 1;//module running

    // start zcm as a separate thread
    zcm.start();

    unsigned char line[BUFFER_LENGTH];
    int result;

    while(!handlerObject.stat.should_exit)
    {
        zcm.publish("STATUS1",&module_stat);

        result = readline(line, BUFFER_LENGTH);
        if (result < 0)
        {
        //log an error message:
            std::cout << "WARNING: error while reading from port: " << result << std::endl;
            continue;
        }

        if (! is_valid_line(line))
        {
            continue;
        }

        if (parseline(line, &msg) == 0)
        {
            zcm.publish("VNINS_DATA", &msg);
        }
    }

    // close serial port
    RS232_CloseComport(COMPORT);

    module_stat.module_status = 0;
    zcm.publish("STATUS1",&module_stat);

    std::cout << "vn200 module exiting..." << std::endl;

    // stop zcm
    zcm.stop();

    // exit
    return 0;
}
