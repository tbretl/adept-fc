//
// Use of this file is governed by the MIT License - see adept_fc/LICENSE_MIT
//
// Copyright (c) 2019 Timothy Bretl, Aaron Perry, and Phillip Ansell
//

#include <zcm/zcm-cpp.hpp>
#include "adc_data_t.hpp"
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
#include <cmath>

#define COMPORT			24 		// '/dev/ttyACM0'
#define BAUDRATE		230400
#define BUFFER_LENGTH 	255

class Handler
{
    public:
        ~Handler() = default;

        status_t stat;
        vnins_data_t vn200;

        Handler()
        {
            memset(&stat, 0, sizeof(stat));
            memset(&vn200, 0, sizeof(vn200));
        }

        void read_stat(const zcm::ReceiveBuffer* rbuf,const std::string& chan,const status_t *msg)
        {
            stat = *msg;
        }

        void read_vn200(const zcm::ReceiveBuffer* rbuf,const std::string& chan,const vnins_data_t *msg)
        {
            vn200 = *msg;
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
            " ADC: error - poll returned " << len << " in flush" << std::endl;
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
            std::cout << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count() <<
            " ADC: error - poll returned code " << len << " in readline" << std::endl;
            return -1;
        } else if (len > 1)
        {
            std::cout << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count() <<
            " ADC: error - read " << len << " > 1 bytes in readline" << std::endl;
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
                " ADC: error - read " << nc << " bytes with no end of line" << std::endl;
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
        std::cout << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count() <<
        " ADC: error: line has zero length\n" << line << std::endl;
        return false;
    }

    return true;
}

int parseline(unsigned char* line, adc_data_t *msg)
{
    /**
    // Check that line has correct length for VNINS
    const int len_expected = MESSAGE_LENGTH;
    int len = strlen((char*) line);
    if (len != len_expected)
    {
        std::cout << "error: line has length "<< len << ", should be " << len_expected << std::endl;
        return 1;
    }
    **/

    // it is important to force base 10 in this conversion, because
    // the time is most likely padded with leading zeros, which causes
    // a default interpretation as octal
    char* field;
    field = strtok((char*) line, ",");
    if (field == NULL) {
        std::cout << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count() <<
        " ADC: error: line contains no commas" << std::endl;
        return 1;
    }
    msg->time_gpspps = (long long) strtoul(field, NULL, 10);
    for (int i=0; i<16; ++i) {
        field = strtok(NULL, ",");
        if (field == NULL) {
            std::cout << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count() <<
            " ADC: error: in parseline at data field " << i << std::endl;
            return 1;
        }
        
	msg->data[i] = strtod(field, NULL);
	
	if (msg->data[i] > 70000) { 
            std::cout << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count() <<
            " ADC: error: bit value is too high on channel " << i << std::endl;
	    return 1; 
	} 
    }

    return 0;
}

int main()
{
    // open serial port
    if (RS232_OpenComport(COMPORT, BAUDRATE, "8N1"))
    {
        std::cout << std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count() <<
        " ADC: error while opening port" << std::endl;
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
    vnins_data_t local_vnins; 
    zcm.subscribe("STATUS",&Handler::read_stat,&handlerObject);
    zcm.subscribe("VNINS_DATA",&Handler::read_vn200,&handlerObject);

    // create objects to publish
    adc_data_t msg;
    memset(&msg,0,sizeof(msg));

    status_t module_stat;
    memset(&module_stat,0,sizeof(module_stat));
    module_stat.module_status = 1;//module running

    // start zcm as a separate thread
    zcm.start();

    unsigned char line[BUFFER_LENGTH];
    int result;

    std::cout << "ADC started" << std::endl;
    double prev_gps = 0;    
    while(!handlerObject.stat.should_exit)
    {

        zcm.publish("STATUS2",&module_stat);

        result = readline(line, BUFFER_LENGTH);

        //get most recent time conditions: 
        int64_t rpi_time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
	local_vnins = handlerObject.vn200; 	

        if (result < 0)
        {
            //log an error message:
            std::cout << rpi_time << "ADC: WARNING: error while reading from port: " << result << std::endl;
            continue;
        }

        if (! is_valid_line(line))
        {
            continue;
        }

        if (parseline(line, &msg) == 0)
        {
            //compute ADC time_gps
            double lastppstime = std::floor(local_vnins.time + (rpi_time - local_vnins.time_rpi)/1000000.0);
            if (msg.time_gpspps > 900000 && lastppstime > prev_gps) {
	        lastppstime = prev_gps; 
	    }
	    msg.time_gps = lastppstime + msg.time_gpspps/1000000.0;
	    prev_gps = lastppstime; 
	    //add the RPI time
            msg.time_rpi = rpi_time;
            //publish ADC data
            if (msg.time_gpspps < 1000010){ //prevent bad PPS or ADC read from being logged
	    	zcm.publish("ADC_DATA", &msg);
	    }
            //loop timing
            usleep(2500);
        }
    }

    // close serial port
    RS232_CloseComport(COMPORT);

    module_stat.module_status = 0;
    zcm.publish("STATUS2",&module_stat);

    std::cout << "adc module exiting..." << std::endl;

    // stop zcm
    zcm.stop();

    // exit
    return 0;
}
