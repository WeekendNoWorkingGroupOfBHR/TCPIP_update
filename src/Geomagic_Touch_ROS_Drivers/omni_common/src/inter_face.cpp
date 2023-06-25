//客户端
#include "TCPIP_Port.hpp"
#include <iostream>
using namespace std;

int main(int argc, char** argv)
{
    perception::tcpip_port port("192.168.123.113");
    port.initial();
    perception::Com_Data send_data;
    // send_data.flag = 'A';
    // send_data.direct = 0.2f;
    // send_data.left_arm[0] = 0.;
    // send_data.left_arm[1] = 0.2f;
    // send_data.left_arm[2] = 0.1f;

    // send_data.left_arm[6] = {0., 0.2, 0.1};
    // send_data.right_arm[6] = {0., 0.7, 0.2};
    // send_data.left_leg[6] = {0., 0.8, 0.3};
    // send_data.right_leg[6] = {0., 0.9, 0.4};
    port.sendFlag(send_data);
    // Sleep(1000);
    vector<perception::footstep> steps;
    if (port.receive_steps() == 1)
    {
        steps = port.getRecvSteps();
    }
    
    // port.receive_steps();
    // vector<perception::footstep> steps = port.getRecvSteps();
    for (auto & step : steps)
    {
        // cout<<"is left: "<<step.is_left<<endl;
        // cout<<"x: "      <<step.x      <<endl;
        // cout<<"y: "      <<step.y      <<endl;
        // cout<<"z: "      <<step.z      <<endl;
        // cout<<"roll: "   <<step.roll   <<endl;
        // cout<<"pitch: "  <<step.pitch  <<endl;
        // cout<<"yaw: "    <<step.yaw    <<endl;
    }
    
    return 0;
}