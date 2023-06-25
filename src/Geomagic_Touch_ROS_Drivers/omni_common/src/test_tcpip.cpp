//服务端
#include <iostream>
#include "tcpip/data_type.h"
#include <tcpip/tcpip_port.h>

using namespace std;

int main(int argc, char** argv)
{
    tcpip_port port;
    port.initial();
    port.accept_client();
    if (port.recvData())
    {
        port.analysisBuf();
        vector<LEECHAO::FootStep> steps;
        LEECHAO::FootStep step;
        step.is_left = true;
        step.x = 0.1;
        step.y = 0.01;
        step.z = 0.4;
        step.roll = 0.5;
        step.pitch = 0.7;
        step.yaw = 0.9;
        steps.emplace_back(step);
        port.sendSteps(steps);
    }
    // sleep(1);
    port.close_client();
    port.closetcpip();
    return 0;
}