//服务端
#include "tcpip/tcpip_port.h"
#include <iostream>
#include <mutex>
#define SERVPORT 3333
#define BACKLOG 10
using namespace std;
#include <vector>

// extern int sock_fd,client_fd;
// extern std::mutex m_tcpip;
tcpip_port::tcpip_port(/* args */)
{
}

tcpip_port::~tcpip_port()
{
    close_client();
    closetcpip();
}

int tcpip_port::initial()
{
    {
        // unique_lock<mutex> g(m_tcpip, std::defer_lock);
        // g.lock();
        if((sock_fd = socket(AF_INET, SOCK_STREAM/* |SOCK_NONBLOCK */, 0)) == -1) {
                perror("socket construct error");
                exit(1);
                return -1;
        }
        // g.unlock();
    }
    
    int flag = 1;
	if (setsockopt(sock_fd, SOL_SOCKET, SO_REUSEADDR, &flag, sizeof(flag)) < 0)
	{
		printf("socket setsockopt error=%d(%s)!!!\n", errno, strerror(errno));
		exit(1);
	}


    my_addr.sin_family=AF_INET;
    my_addr.sin_port=htons(SERVPORT);
    my_addr.sin_addr.s_addr = INADDR_ANY;
    bzero(&(my_addr.sin_zero),8);
    if(bind(sock_fd, (struct sockaddr *)&my_addr, sizeof(struct sockaddr)) == -1) {
        perror("bind error");
        exit(1);
        return -1;
    }
    if(listen(sock_fd, BACKLOG) == -1) {
        perror("listen error");
        exit(1);
        return -1;
    }
    // int timeout = 50;
    
    // setsockopt(sock_fd, SOL_SOCKET, SO_RCVTIMEO, (char*)&timeout, sizeof(timeout));
    sin_size = sizeof(struct sockaddr_in);
    printf("initial port finish...\n");
    return 0;
}

int tcpip_port::accept_client()
{
    // unique_lock<mutex> g(m_tcpip, std::defer_lock);
    // g.lock();
    if((client_fd = accept(sock_fd, (struct sockaddr *)&remote_addr, (socklen_t *)&sin_size)) == -1) {
        perror("accept error");
        exit(1);
        return -1;
    }
    // g.unlock();
    printf("received a connection from %s\n", inet_ntoa(remote_addr.sin_addr));
    return 0;
}

int tcpip_port::recvChar()
{
    return recv(client_fd, &recvFlag, sizeof(recvFlag), 0);
}

int tcpip_port::recvData()
{
    cout<<"enter recve data "<<endl;
    int tmp = recv(client_fd, &recevBuf, sizeof(recevBuf), 0);
    return tmp; 
}

void tcpip_port::analysisBuf()
{
    com_data = *((LEECHAO::Com_Data*)recevBuf);
    cout<<"flag: "<<com_data.flag<<endl;
    cout<<"walk direct "<<com_data.direct<<endl;
    cout<<"left arm: ";
    for (size_t i = 0; i < 6; i++)
    {
        cout<<com_data.left_arm_joints[i]<<" ";
    }
    cout<<endl;

    cout<<"right arm: ";
    for (size_t i = 0; i < 6; i++)
    {
        cout<<com_data.right_arm_joints[i]<<" ";
    }
    cout<<endl;

    cout<<"left leg: ";
    for (size_t i = 0; i < 6; i++)
    {
        cout<<com_data.left_leg_joints[i]<<" ";
    }
    cout<<endl;

    cout<<"right leg: ";
    for (size_t i = 0; i < 6; i++)
    {
        cout<<com_data.right_leg_joints[i]<<" ";
    }
    cout<<endl;

}

int tcpip_port::sendSteps(vector<LEECHAO::FootStep> footsteps)
{
    cout<<"send "<<footsteps.size()<<" footsteps to control end."<<endl;
    struct SendData
    {
        int n;
        LEECHAO::FootStep steps[16];
    };
    SendData send_data;
    cout<<"size of FootStep "<<sizeof(LEECHAO::FootStep)<<endl;//28
    cout<<"size of data "<<sizeof(send_data)<<endl;//452
    cout<<"size of data "<<sizeof(SendData)<<endl;//452
    send_data.n = footsteps.size();
    for (size_t i = 0; i < footsteps.size(); i++)
    {
        send_data.steps[i].is_left = footsteps.at(i).is_left;
        send_data.steps[i].x       = footsteps.at(i).x;
        send_data.steps[i].y       = footsteps.at(i).y;
        send_data.steps[i].z       = footsteps.at(i).z;
        send_data.steps[i].roll    = footsteps.at(i).roll;
        send_data.steps[i].pitch   = footsteps.at(i).pitch;
        send_data.steps[i].yaw     = footsteps.at(i).yaw;
    }
    return send(client_fd, (char*)(&send_data), sizeof(send_data), 0);
}

void tcpip_port::close_client()
{
    cout<<"close client"<<endl;
    close(client_fd);

}

void tcpip_port::closetcpip()
{
    cout<<"CLOSE TCPIP PORT..."<<endl;
    close(sock_fd);
}

char tcpip_port::getRecvFlag()
{
    // return recvFlag;
    return com_data.flag;
}

void tcpip_port::resetRecvFlag()
{
    recvFlag = '\0';
}
