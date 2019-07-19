//
// Use of this file is governed by the MIT License - see adept_fc/LICENSE_MIT
//
// Copyright (c) 2019 Timothy Bretl, Aaron Perry, and Phillip Ansell
//

#include <zcm/zcm-cpp.hpp>
#include "vnins_data_t.hpp"
#include "status_t.hpp"
#include "rs232.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <string>
#include <iostream>
#include <iomanip>
#include <chrono>

#define COMPORT			22 		// '/dev/ttyAMA0'
#define BAUDRATE		115200
#define BUFFER_LENGTH 	255
#define MESSAGE_LENGTH  154

// Constants for communication protocol control (p80 of VN-200 user manual)
enum class SerialCount { NONE, SYNCIN_COUNT, SYNCIN_TIME, SYNCOUT_COUNT, GPS_PPS };
enum class SerialState { OFF, VPE, INS };
enum class SPICount { NONE, SYNCIN_COUNT, SYNCIN_TIME, SYNCOUT_COUNT, GPS_PPS };
enum class SPIStatus { OFF, VPE, INS };
enum class SerialChecksum { CS_8 = 1, CS_16 = 3 };
enum class SPIChecksum { OFF, CS_8 = 1, CS_16 = 3 };
enum class ErrorMode { IGNORE, SEND, SEND_AND_ADOR_OFF };

class Handler
{
    public:
        ~Handler() = default;

        status_t stat;

        Handler()
        {
            memset(&stat, 0, sizeof(stat));
        }

        void read_stat(const zcm::ReceiveBuffer* rbuf,const std::string& chan,const status_t *msg)
        {
            stat = *msg;
        }
};

int flush(unsigned int timeout_ms = 500)
{
    auto start_time = std::chrono::steady_clock::now();

    unsigned char c;
    size_t len;
    while (true)
    {
        auto current_time = std::chrono::steady_clock::now();
        unsigned int time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time).count();
        if (time_ms >= timeout_ms) {
            // time out (no data to flush)
            return 0;
        }

        len = RS232_PollComport(COMPORT, &c, 1);
        if ((len < 0) || (len > 1))
        {
            std::cout << "error - poll returned " << len << " in flush" << std::endl;
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
            std::cout << "error - poll returned code " << len << " in readline" << std::endl;
            return -1;
        } else if (len > 1)
        {
            std::cout << "error - read " << len << " > 1 bytes in readline" << std::endl;
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
                std::cout << "error - read " << nc << " bytes with no end of line" << std::endl;
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
    msg->week = (long long) strtoul(field, NULL, 10);
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
    field = strtok(NULL, ",T");
    // it is important to force base 10 in this conversion, because
    // the time is most likely padded with leading zeros, which causes
    // a default interpretation as octal
    msg->time_gpspps = (long long) strtoul(field, NULL, 10);

    return 0;
}

bool writecommand(std::string &s) {
    // compute 8-bit checksum with a range-based for loop
    unsigned char cs8 = 0;
    for (unsigned char const &c: s) {
        cs8 ^= c;
    }

    // convert 8-bit checksum to hex string
    std::stringstream stream;
    stream << std::setfill('0')         // output has form "0X" and not "X"
           << std::setw(2)              // output has two characters
           << std::hex                  // convert anything after this to hex
           << (unsigned int) cs8;       // interpret checksum as int before converting

    // prefix
    s.insert(0, "$");

    // suffix (with checksum)
    s += "*";
    s += stream.str();

    // termination characters
    s += "\r\n";

    // send command
    int result = RS232_SendBuf(COMPORT, (unsigned char*) s.c_str(), s.size());
    if (result < 0) {
        std::cout << "WARNING: error ("
                  << result
                  << ") on sending command "
                  << s << std::endl;
        return false;
    }

    // get response
    unsigned char line[BUFFER_LENGTH];
    result = readline(line, BUFFER_LENGTH);
    if (result < 0) {
        std::cout << "WARNING: error ("
                  << result
                  << ") on responding to command "
                  << s << std::endl;
        return false;
    }

    // check if response is valid
    if (! is_valid_line(line)) {
        return false;
    }

    // replace s with response
    s.clear();
    s.append((const char*) line);

    // check for error in response
    char* field;
    field = strtok((char*) line + 1, ",");
    if (strcmp(field, "VNERR") == 0) {
        field = strtok(NULL, ",");
        field = strtok(field, "*");
        std::cout << "WARNING: error (VNERR,"
                  << field
                  << ") in response to command "
                  << s << std::endl;
        return false;
    }

    return true;
}

bool async(bool on) {
    std::string s = "VNASY,";
    s += (on ? "1" : "0");
    return writecommand(s);
}

bool getprotocol() {
    // The default communicaton protocol (at factory settings) is:
    //
    //  $VNRRG,30,0,0,0,0,1,0,1*6C
    //

    std::string s = "VNRRG,30";
    bool result = writecommand(s);
    std::cout << "VN-200 communication protocol: " << s << std::endl;
    return result;
}

bool setprotocol(
    SerialCount serialcount=SerialCount::GPS_PPS,
    SerialState serialstate=SerialState::OFF,
    SPICount spicount=SPICount::NONE,
    SPIStatus spistatus=SPIStatus::OFF,
    SerialChecksum serialchecksum=SerialChecksum::CS_8,
    SPIChecksum spichecksum=SPIChecksum::OFF,
    ErrorMode errormode=ErrorMode::SEND
) {
    std::string s = "VNWRG,30";
    s += ",";
    s += std::to_string(static_cast<unsigned char>(serialcount));
    s += ",";
    s += std::to_string(static_cast<unsigned char>(serialstate));
    s += ",";
    s += std::to_string(static_cast<unsigned char>(spicount));
    s += ",";
    s += std::to_string(static_cast<unsigned char>(spistatus));
    s += ",";
    s += std::to_string(static_cast<unsigned char>(serialchecksum));
    s += ",";
    s += std::to_string(static_cast<unsigned char>(spichecksum));
    s += ",";
    s += std::to_string(static_cast<unsigned char>(errormode));

    return writecommand(s);
}

bool setoutputfrequency(int rate=40) {
    const int nrates = 11;
    const int rates[nrates] = {1, 2, 4, 5, 10, 20, 25, 40, 50, 100, 200};

    bool acceptable = false;
    for (int i=0; i<nrates; ++i) {
        if (rate == rates[i]) {
            acceptable = true;
        }
    }
    if (! acceptable) {
        std::cout << "WARNING: unacceptable rate " << rate << " in setoutputfrequency" << std::endl;
        return false;
    }

    std::string s = "VNWRG,07";
    s += ",";
    s += std::to_string(rate);

    return writecommand(s);
}

void config() {
    // turn off asynchronous outputs
    async(false);

    // set communication protocol
    setprotocol();

    // set output frequency (will fail if baud rate does not support the desired frequency)
    setoutputfrequency();

    // turn on asynchronous outputs
    async(true);
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

    // configure
    config();

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

    zcm.start();

    unsigned char line[BUFFER_LENGTH];
    int result;

    std::cout << "VN-200 started" << std::endl;

    while(!handlerObject.stat.should_exit)
    {
        zcm.publish("STATUS1",&module_stat);

        result = readline(line, BUFFER_LENGTH);
        if (result < 0)
        {
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

        //loop timing
        usleep(10000); //100 hz
    }

    // close serial port
    RS232_CloseComport(COMPORT);

    module_stat.module_status = 0;
    zcm.publish("STATUS1",&module_stat);

    std::cout << "vn200 module exiting..." << std::endl;

    zcm.stop();

    return 0;
}
