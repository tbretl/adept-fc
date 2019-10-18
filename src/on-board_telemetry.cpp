#include <stdio.h>
#include <iostream>
#include <unistd.h>
#include <string.h>
#include <zcm/zcm-cpp.hpp>
#include <chrono>
//message types:
#include "actuators_t.hpp"
#include "status_t.hpp"
#include "adc_data_t.hpp"
#include "vnins_data_t.hpp"
#include "rc_t.hpp"
#include "pwm_t.hpp"

using std::string;

// define serial port buadrate, name, and location
#define BAUDRATE B115200
#define PORT "/dev/ttyUSB0"

class Handler
{
    public:
        ~Handler() = default;

        adc_data_t adc;
        status_t stat;

        // new local variables
        actuators_t acts;
        rc_t rc;
        vnins_data_t vnins_data;

        Handler()
        {
            memset(&adc,0,sizeof(adc));
            memset(&stat, 0, sizeof(stat));

            // new memory allocations
            memset(&acts, 0, sizeof(acts));
            memset(&rc, 0, sizeof(rc));
            memset(&vnins_data, 0, sizeof(vnins_data));
        }

        void read_adc(const zcm::ReceiveBuffer* rbuf,const string& chan,const adc_data_t *msg)
        {
            adc = *msg;
        }

        void read_stat(const zcm::ReceiveBuffer* rbuf,const string& chan,const status_t *msg)
        {
            stat = *msg;
        }
        
        // new read functions
        void read_rc(const zcm::ReceiveBuffer* rbuf,const string& chan,const rc_t *msg)
        {
            rc = *msg;
        }

        void read_acts(const zcm::ReceiveBuffer* rbuf,const string& chan,const actuators_t *msg)
        {
            acts = *msg;
        }

        void read_vn200(const zcm::ReceiveBuffer* rbuf,const string& chan,const vnins_data_t *msg)
        {
            vnins_data = *msg;
        }
};

double get_gps_time(Handler* adchandle)
{
    double adc_gps = adchandle->adc.time_gps;
    int64_t adc_time = adchandle->adc.time_rpi;
    int64_t rpi_time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
    return  adc_gps + (rpi_time - adc_time)/1000000.0;
}


int main(int argc, char *argv[])
{
    //initialize zcm
    zcm::ZCM zcm {"ipc"};

    //initialize message objects
    //actuators_t acts;
    //memset(&acts, 0, sizeof(acts));

    //subscribe to incoming channels:
    Handler handlerObject;
    zcm.subscribe("STATUS",&Handler::read_stat,&handlerObject);
    zcm.subscribe("ADC_DATA",&Handler::read_adc,&handlerObject);

    // new changes
    // subscribe to additional zcm channels
    zcm.subscribe("VNINS_DATA",&Handler::read_vn200,&handlerObject);
    zcm.subscribe("ACTUATORS", &Handler::read_acts, &handlerObject);
    zcm.subscribe("PWN_OUT", &Handler::read_pwm, &handlerObject);
    zcm.subscribe("RC_IN",&Handler::read_rc,&handlerObject);

    //for bublishing stat of this module
    //status_t module_stat;
    //memset(&module_stat,0,sizeof(module_stat));
    //module_stat.module_status = 1;//module running

    //run zcm as a separate thread:
    zcm.start();

    // Open port0 to write live data to it
    int port0 = open(PORT, O_RDWR | O_NOCTTY | O_SYNC);

    // use termios struct to define serial port properties
    struct termios tty;
    // allocate memory for tty
    memset(&tty, 0, sizeof tty);

    // set serial port in and out speed as the baud rate
    cfsetispeed(&tty, BAUDRATE);
    cfsetospeed(&tty, BAUDRATE);

    // set tty as serial port to use
    tcsetattr(port0, TCSANOW, &tty);

    //std::cout << "autopilot started" << std::endl;
    //control loop:
    while (!handlerObject.stat.should_exit)
    {
        // output to ground station -- will have to be formatted
        char msg[] = {adc, status, rc, pwm_out, actuators, vn200};

        // Write string to port0, allocating size
        write(port0, msg, sizeof(msg));

        usleep(10000);

    }

    //module_stat.module_status = 0;
    //zcm.publish("STATUS4",&module_stat);

    //std::cout << "autopilot module exiting..." << std::endl;

    zcm.stop();

    return 0;
}