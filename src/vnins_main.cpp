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
#include <assert.h>

#define COMPORT			    22 		// '/dev/ttyAMA0'
#define CURRENT_BAUDRATE	230400  // the baudrate you have
#define BAUDRATE            230400  // the baudrate you want
#define BUFFER_LENGTH 	    255
#define MESSAGE_LENGTH      96 //154
#define SIZEOF_FLOAT        4       // number of bytes in float
#define SIZEOF_DOUBLE       8       // number of bytes in double

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
            std::cout << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count() <<
            " error - poll returned " << len << " in flush" << std::endl;
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

int readbyte(unsigned char* c) {
    // blocks until one byte is read
    size_t len;
    while (true) {
        len = RS232_PollComport(COMPORT, c, 1);
        if (len < 0) {
            std::cout << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count() <<
            " error - poll returned code " << len << " in readbyte" << std::endl;
            return 1;
        } else if (len > 1) {
            std::cout << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count() <<
            " error - read " << len << " > 1 bytes in readbyte" << std::endl;
            return 1;
        } else if (len == 1) {
            return 0;
        }
    }
}

int readline_binary(unsigned char* buf, int len, vnins_data_t *msg) {
    // message has 96 bytes:
    //
    //  header has 4 bytes
    //  payload has 90 bytes
    //  crc (16-bit) has 2 bytes
    //
    
    unsigned char c;
    
    // read first byte
    if (readbyte(&c)) {
        std::cout << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count()
                  << " error - could not read byte 0 in readline" << std::endl;
        return 1;
    }
    buf[0] = c;
    
    // verify first byte is sync
    uint8_t sync = buf[0];
    if (sync != 0xFA) {
        std::cout << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count()
                  << " error - non-sync byte 0 ("
                  << std::setfill('0')         // output has form "0X" and not "X"
                  << std::setw(2)              // output has two characters
                  << std::hex                  // convert anything after this to hex
                  << (unsigned int) sync       // cast before converting
                  << ") in readline" << std::endl;
        return 1;
    }
    
    // read all other bytes
    int nc = 1;
    unsigned short cs16 = 0;
    while (nc < len) {
        if (readbyte(&c)) {
            std::cout << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count()
                  << " error - could not read byte " << nc << " in readline" << std::endl;
            return 1;
        }
        
        // compute 16-bit checksum as we go
        cs16 = (unsigned char)(cs16 >> 8) | (cs16 << 8);
        cs16 ^= c;
        cs16 ^= (unsigned char)(cs16 & 0xff) >> 4;
        cs16 ^= cs16 << 12;
        cs16 ^= (cs16 & 0x00ff) << 5;
        
        buf[nc] = c;
        nc++;
    }
    
    // verify zero checksum
    if (cs16 != 0) {
        std::cout << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count()
                  << " error - bad checksum (" << cs16 << ") in readline for "
                  << nc << " bytes" << std::endl;
        return 1;
    }
    
    // verify group
    uint8_t group = buf[1];
    if (group != 0x01) {
        std::cout << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count()
                  << " error - bad group ("
                  << std::setfill('0')         // output has form "0X" and not "X"
                  << std::setw(2)              // output has two characters
                  << std::hex                  // convert anything after this to hex
                  << (unsigned int) group      // cast before converting
                  << ") in readline" << std::endl;
        return 1;
    }
    
    // verify field
    uint16_t field;
    memcpy(&field,              &buf[2],    2);
    if (field != 0x51EA) {
        std::cout << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count()
                  << " error - bad field ("
                  << std::setfill('0')         // output has form "0X" and not "X"
                  << std::setw(4)              // output has four characters
                  << std::hex                  // convert anything after this to hex
                  << (unsigned int) field      // cast before converting
                  << ") in readline"
                  << std::endl;
        return 1;
    }
    
    // parse line
    uint64_t timegps;
    memcpy(&timegps,            &buf[4],    8);
    msg->time = timegps / 1000000000;
    memcpy(&(msg->yaw),         &buf[12],   SIZEOF_FLOAT);
    memcpy(&(msg->pitch),       &buf[16],   SIZEOF_FLOAT);
    memcpy(&(msg->roll),        &buf[20],   SIZEOF_FLOAT);
    memcpy(&(msg->wx),          &buf[24],   SIZEOF_FLOAT);
    memcpy(&(msg->wy),          &buf[28],   SIZEOF_FLOAT);
    memcpy(&(msg->wz),          &buf[32],   SIZEOF_FLOAT);
    memcpy(&(msg->latitude),    &buf[36],   SIZEOF_DOUBLE);
    memcpy(&(msg->longitude),   &buf[44],   SIZEOF_DOUBLE);
    memcpy(&(msg->altitude),    &buf[52],   SIZEOF_DOUBLE);
    memcpy(&(msg->vn),          &buf[60],   SIZEOF_FLOAT);
    memcpy(&(msg->ve),          &buf[64],   SIZEOF_FLOAT);
    memcpy(&(msg->vd),          &buf[68],   SIZEOF_FLOAT);
    memcpy(&(msg->ax),          &buf[72],   SIZEOF_FLOAT);
    memcpy(&(msg->ay),          &buf[76],   SIZEOF_FLOAT);
    memcpy(&(msg->az),          &buf[80],   SIZEOF_FLOAT);
    uint16_t insstatus;
    memcpy(&insstatus,          &buf[84],   2);
    //
    // FIXME: check the result of this code with a working GPS antenna
    //
    bool bits[16];
    for (int i=0; i<16; ++i)
    {
        bits[15 - i] = (insstatus & 1);
        insstatus >>= 1;
    }
    msg->tracking = (bits[0] && (! bits[1]));
    msg->gpsfix = bits[2];
    msg->error = (bits[3] || bits[4] || bits[5] || bits[6]);
    //
    memcpy(&(msg->time_gpspps), &buf[86],   8);
    
    // return with success
    return 0;
}

int readline_ascii(unsigned char* buf, int maxlen)
{
    unsigned char c;
    int nc = 0;
    size_t len;
    while (true)
    {
        len = RS232_PollComport(COMPORT, &c, 1);
        if (len < 0)
        {
            std::cout << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count() <<
            " error - poll returned code " << len << " in readline" << std::endl;
            return -1;
        } else if (len > 1)
        {
            std::cout << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count() <<
            " error - read " << len << " > 1 bytes in readline" << std::endl;
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
                std::cout << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count() <<
                " error - read " << nc << " bytes with no end of line" << std::endl;
                return -1;
            }
        }
    }
}

bool is_valid_line_ascii(unsigned char* line)
{
    // it has non-zero length
    unsigned int len = strlen((char*) line);
    if (len == 0)
    {
        std::cout << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count() <<
        " error: line has zero length\n" << line << std::endl;
        return false;
    }

    // it starts with '$'
    if (line[0] != '$')
    {
        std::cout << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count() <<
        " error: line starts with " << line[0] << " and not with $\n" << line << std::endl;
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
        std::cout << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count() <<
        " error: line contains no *\n" << line << std::endl;
        return false;
    }

    // it has the correct checksum
    unsigned long a = strtoul((char*) line + i + 1, NULL, 16);
    unsigned long b = (unsigned long) cs8;
    if (a != b)
    {
        std::cout << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count() <<
        " error: line checksum does not match (" << a << "," << b << ")\n" << line << std::endl;
        return false;
    }

    return true;
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
        std::cout << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count()
                  << " WARNING: error ("
                  << result
                  << ") on sending command "
                  << s << std::endl;
        return false;
    }

    // get response
    unsigned char line[BUFFER_LENGTH];
    result = readline_ascii(line, BUFFER_LENGTH);
    if (result < 0) {
        std::cout << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count()
                  << " WARNING: error ("
                  << result
                  << ") on responding to command "
                  << s << std::endl;
        return false;
    }

    // check if response is valid
    if (! is_valid_line_ascii(line)) {
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
        std::cout << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count()
                  << "WARNING: error (VNERR,"
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

bool async_ascii(bool on) {
    std::string s = "VNWRG,06,";
    s += (on ? "1" : "0");
    return writecommand(s);
}

bool setbaudrate(int rate) {
    const int nrates = 9;
    const int rates[nrates] = {9600, 19200, 38400, 57600, 115200, 128000, 230400, 460800, 921600};

    bool acceptable = false;
    for (int i=0; i<nrates; ++i) {
        if (rate == rates[i]) {
            acceptable = true;
        }
    }
    if (! acceptable) {
        std::cout << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count() <<
        " WARNING: unacceptable rate " << rate << " in setbaudrate" << std::endl;
        return false;
    }
    
    std::string s = "VNWRG,05";
    s += ",";
    s += std::to_string(rate);
    
    return writecommand(s);
}

bool setmessage() {
    // We want these data fields from the common group (Section 4.4):
    //
    // name (bit offset)
    //
    // TimeGps (1)
    // Ypr (3)
    // AngularRate (5)
    // Position (6) - LLA
    // Velocity (7) - NED
    // Accel (8) - Body
    // InsStatus (12)
    //
    // Compare to the data fields in the "INS Solution - LLA" that we
    // had been using (Section 9.2.1).
    //
    // To get this, we use the following message (see Section 4.2.4):
    //
    //  header          VN          ASCII message header
    //  command         WRG         write register command
    //  register ID     75          register 75 (first output message)
    //  asyncmode       2           output on second serial port
    //  rate divisor    8           if imurate is 800 Hz, output rate is 800 / 8 = 100 Hz
    //  outputgroup     01          groups = 0x01 (binary group 1 enabled)
    //  groupfield 1    51EA        binary: 0b0101000111101010
    //                                  bit 1  - timegps                - 8 bytes
    //                                  bit 3  - ypr                    - 12 bytes
    //                                  bit 5  - angularrate            - 12 bytes
    //                                  bit 6  - position (LLA)         - 24 bytes
    //                                  bit 7  - velocity (NED)         - 12 bytes
    //                                  bit 8  - acceleration (body)    - 12 bytes
    //                                  bit 12 - insstatus              - 2 bytes
    //                                  bit 14 - gpspps                 - 8 bytes
    //  + checksum + endline
    
    std::string s = "VNWRG,75,2,8,01,51EA";
    return writecommand(s);
}

int config() {
    // turn off asynchronous outputs
    async(false);
    
    // flush serial port
    RS232_flushRXTX(COMPORT);
    if (flush() != 0)
    {
        return 1;
    }
    
    // turn off serial ASCII outputs
    async_ascii(false);
    
    // set user-configurable binary output
    setmessage();
    
    // turn on asynchronous outputs
    async(true);
    
    // return success
    return 0;
}

int main()
{
    // confirm sizes of types
    assert(sizeof(float) == SIZEOF_FLOAT);
    assert(sizeof(double) == SIZEOF_DOUBLE);
    
    // handle change in baudrate if necessary
    if (CURRENT_BAUDRATE != BAUDRATE)
    {
        // open serial port
        if (RS232_OpenComport(COMPORT, CURRENT_BAUDRATE, "8N1"))
        {
            std::cout << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count() <<
            " error while opening port" << std::endl;
            return 1;
        }
        
        // flush serial port
        RS232_flushRXTX(COMPORT);
        if (flush() != 0)
        {
            return 1;
        }
        
        // turn off asynchronous outputs
        async(false);
        
        // change baudrate
        if (setbaudrate(BAUDRATE)) {
            std::cout << "\n===============================================================\n"
                      << "WARNING: baudrate changed from "
                      << CURRENT_BAUDRATE << " to " << BAUDRATE << ", "
                      << "so you MUST remember to set\n\n"
                      << "#define CURRENT_BAUDRATE	" << BAUDRATE << "  // the baudrate you have\n\n"
                      << "in vnins_main.cpp and rebuild (also push to master!)\n"
                      << "===============================================================\n"
                      << std::endl;
        }
        
        // turn on asynchronous outputs
        async(true);
        
        // close serial port
        RS232_CloseComport(COMPORT);
    }
    
    // open serial port
    if (RS232_OpenComport(COMPORT, BAUDRATE, "8N1"))
    {
        std::cout << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count() <<
        " error while opening port" << std::endl;
        return 1;
    }
    
    // config
    if (config() != 0) {
        std::cout << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count()
                  << " error in config()" << std::endl;
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

    zcm.start();

    unsigned char line[BUFFER_LENGTH];
    
    std::cout << "VN-200 started" << std::endl;
    
    while(!handlerObject.stat.should_exit)
    {
        zcm.publish("STATUS1",&module_stat);
        
        if (readline_binary(line, MESSAGE_LENGTH, &msg) == 0) {
            zcm.publish("VNINS_DATA", &msg);
        } else {
            std::cout << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count()
                      << " WARNING: error while reading from port, attempting reset" << std::endl;
            // turn off asynchronous outputs
            async(false);
            // turn on asynchronous outputs
            async(true);
        }
        
        usleep(1000);   // allows a max rate of 1000 Hz
    }
    
    // turn off asynchronous outputs
    async(false);

    // close serial port
    RS232_CloseComport(COMPORT);

    module_stat.module_status = 0;
    zcm.publish("STATUS1",&module_stat);

    std::cout << "vn200 module exiting..." << std::endl;

    zcm.stop();

    return 0;
}
