//just to check that zcm module status is working:
#include <stdio.h>
#include <fstream>
#include <iostream>
#include <unistd.h>
#include <string.h>
#include "zcm/zcm-cpp.hpp"
//message types:
#include "status_t.hpp"

using std::string;

class Handler
{
    public:
        ~Handler() = default;

        status_t stat;

        Handler()
        {
            memset(&stat, 0, sizeof(stat));
        }

        void read_stat(const zcm::ReceiveBuffer* rbuf,const string& chan,const status_t *msg)
        {
            stat = *msg;
        }

};



int main(int argc, char* argv[])
{
    //initialize zcm:
    zcm::ZCM zcm {"ipc"};

    //subscribe to incoming channels:
    Handler h0,h1,h2,h3,h4,h5,h6;
    //module status channels:
    zcm.subscribe("STATUS0",&Handler::read_stat,&h0);
    zcm.subscribe("STATUS1",&Handler::read_stat,&h1);
    zcm.subscribe("STATUS2",&Handler::read_stat,&h2);
    zcm.subscribe("STATUS3",&Handler::read_stat,&h3);
    zcm.subscribe("STATUS4",&Handler::read_stat,&h4);
    zcm.subscribe("STATUS5",&Handler::read_stat,&h5);
    zcm.subscribe("STATUS6",&Handler::read_stat,&h6);

    zcm.start();

    while (true)
    {
        usleep(1000000);
        std::cout << "\n\nModule status: " << h0.stat.module_status << std::endl;
        std::cout << "Module status: " << h1.stat.module_status << std::endl;
        std::cout << "Module status: " << h2.stat.module_status << std::endl;
        std::cout << "Module status: " << h3.stat.module_status << std::endl;
        std::cout << "Module status: " << h4.stat.module_status << std::endl;
        std::cout << "Module status: " << h5.stat.module_status << std::endl;
        std::cout << "Module status: " << h6.stat.module_status << std::endl;

    }

    zcm.stop();

    return 0;
}
