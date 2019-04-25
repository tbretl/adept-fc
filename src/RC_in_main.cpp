//Acquires RC input channel values and publishes them
//Aaron Perry, 4/10/2019

#include <stdio.h>
#include <unistd.h>
#include <iostream>
#include <fstream>
#include <string.h>
#include <zcm/zcm-cpp.hpp>
#include <Navio2/RCInput_Navio2.h>
#include <Common/Util.h>
#include <memory>
//message types:
#include "rc_t.hpp"
#include "status_t.hpp"

#define READ_FAILED -1

using std::string;

class Handler
{
    public:
        ~Handler() = default;

        status_t stat;

        Handler()
        {
            memset(&stat,0,sizeof(stat));
            stat.should_exit = 0;
        }

        void read_stat(const zcm::ReceiveBuffer* rbuf,const string& chan,const status_t *msg)
        {
            stat = *msg;
        }
};

std::unique_ptr <RCInput> get_rcin()
{
    auto ptr = std::unique_ptr <RCInput>{ new RCInput_Navio2() };
    return ptr;
}

int main()
{
    //load configuration
    int rc_fail_servo = 1500;
    int rc_fail_esc = 1000;
    string dump;
    std::ifstream config_stream;

    config_stream.open("config_files/rc_in.config");
    config_stream >> dump >> rc_fail_servo;
    config_stream >> dump >> rc_fail_esc;
    config_stream.close();

    //initialize ZCM
    zcm::ZCM zcm {"ipc"};

    //subscribe to incoming channels:
    Handler handlerObject;
    zcm.subscribe("STATUS",&Handler::read_stat,&handlerObject);

    //initialize message objects
    rc_t rc_in;
    memset(&rc_in,0,sizeof(rc_in));

    //initialize RC input
    if (check_apm())
    {
        return 1;
    }
    auto rcin = get_rcin();
    rcin->initialize();

    zcm.start();
    //main loop
    while (!handlerObject.stat.should_exit)
    {
        //read in each of the 8 channels:
        for (int i = 0; i<8; i++)
        {

            rc_in.rc_chan[i]=rcin->read(i);

            if ((rc_in.rc_chan[i] == READ_FAILED) || (rc_in.rc_chan[i]<500))
            {
                //failsafe logic
                if (i<3)
                {
                    rc_in.rc_chan[i] = rc_fail_servo;
                }
                else if (i==3)
                {
                    rc_in.rc_chan[i] = rc_fail_esc;
                }
                else if (i==4)//consider removing hard code on this one
                {
                    rc_in.rc_chan[i] = 1000; //manual flight mode
                }
                //log an error message
                //std::cout << "error: RC read fail" << std::endl;
            }

        }

        //publish the RC values
        zcm.publish("RC_IN", &rc_in);
    }

    std::cout << "rc_in module exiting..." << std::endl;
    //pass a message back to monitor as well (feature to add)

    zcm.stop();

    return 0;
}
