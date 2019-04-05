//test program to publish messages:

#include <unistd.h>
#include <zcm/zcm.h>
#include "msg_t.h"

int main()
{
    zcm_t *zcm = zcm_create("ipc");

    msg_t msg;
    msg.str = (char*)"Message 1";

    while(1){
        msg_t_publish(zcm,"MESSAGE",&msg);
        usleep(1000000);
    }

    zcm_destroy(zcm);
    return 0;
}
