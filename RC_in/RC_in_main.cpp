//Acquires RC input channel values and publishes them
//Aaron Perry, 4/10/2019

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <zcm/zcm-cpp.hpp>
#include <Navio2/RCInput_Navio2.h>
#include <Common/Util.h>
#include <memory>
//message types:
#include <../message_types/rc_t.hpp>

#define READ_FAILED -1

using std::string;

std::unique_ptr <RCInput> get_rcin()
{
    auto ptr = std::unique_ptr <RCInput>{ new RCInput_Navio2() };
    return ptr;
}

int main()
{
    //initialize ZCM
    zcm::ZCM zcm {"ipc"};

    //initialize message objects
    rc_t rc_in;

    //initialize RC input
    if (check_apm()) {
        return 1;
    }
    auto rcin = get_rcin();
    rcin->initialize();

    //main loop
    while (1)
    {
        //read in each of the 8 channels:
        for (int i = 0; i<8; i++){

            rc_in.rc_chan[i]=rcin->read(i);

            if (rc_in.rc_chan[i] == READ_FAILED){
                printf("Read failed on channel %i\n", i);
            }
            /*else {}
                printf("Channel %i period (ms): %i\n",rc_in.rc_chan[i]);
            }*/
        //publish the RC values
        zcm.publish("RC_IN", &rc_in);
    }

    return 0;
}
