#pragma once
#include <stdio.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <string.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <iostream>
#include <string>
#include <chrono>
#include <thread>
#include <vector>
// #pragma comment(lib, "wsock32")
// #pragma comment(lib, "ws2_32.lib")
namespace perception{
    constexpr int SERVPORT = 6001;
    constexpr int MAXDATASIZE = 1460; /*每次最大数据传输量 */
    namespace internal
    {
        enum CONNECT_STATE
        {
            NONE,
            GOOD,
            CREATE_ERROR,
            CONNECT_ERROR,
            SEND_ERROR,
            RECV_ERROR,
            CONNECT_STATE_NUM
        };
    }

    struct footstep
    {
        // bool is_left;
        // float x, y, z, roll, pitch, yaw;
        double fx;
        double fy;
        double fz;
        double f1;
        double f2;
        double f3;
    };

    struct Com_Data
    {
        // char flag;
        // float direct;
        // float left_arm[6] = {0.};
        // float right_arm[6] = {0.};
        // float left_leg[6] = {0.};
        // float right_leg[6] = {0.};
        int LeftoRight;
        double q1;
        double q2;
        double q3;
        double q4;
        double q5;
        double q6;
        
    };
    

    class tcpip_port
    {
    private:
        // send message
        char flag_off_ground;
        // receive message
        std::vector<footstep> receiveSteps;
        // port ip
        std::string mport_ip;
        // net data
        // WSAData wsa;
        int sock_fd;
        int recvbytes;
        struct sockaddr_in serv_addr;
        int ConnectState;
        std::vector<std::string> ConnectStateStr;

    public:
        inline auto getRecvSteps(){return this->receiveSteps;};
        inline auto getConnectState(){return this->ConnectState;};
        inline auto &getConnectStateStr(){return this->ConnectStateStr[this->ConnectState];};
        tcpip_port(std::string port_ip)
        {
            this->ConnectStateStr.resize(internal::CONNECT_STATE_NUM);
            this->ConnectState = internal::NONE;
            mport_ip = port_ip;
            this->ConnectStateStr[internal::NONE] = "None";
            this->ConnectStateStr[internal::GOOD] = "GooD";
            this->ConnectStateStr[internal::CREATE_ERROR] = "Create Error";
            this->ConnectStateStr[internal::CONNECT_ERROR] = "Connect Error";
            this->ConnectStateStr[internal::SEND_ERROR] = "Send Error";
            this->ConnectStateStr[internal::RECV_ERROR] = "Recv Error";
            std::cout << "connect ip is " << mport_ip << std::endl;
        }

        void initial()
        {
            
            //创建套接字
            if ((sock_fd = socket(AF_INET, SOCK_STREAM, 0)) == -1)
            {
                printf("socket create error!\n");
                this->ConnectState = internal::CREATE_ERROR;
                exit(0);
            }else{
                printf("socket create success!\n");
            }

            
            //连接服务器
            serv_addr.sin_family = AF_INET;//指定地址家族
            serv_addr.sin_port = htons(SERVPORT);//指定端口号
            serv_addr.sin_addr.s_addr = inet_addr(mport_ip.c_str());//指定IP地址
            // serv_addr.sin_addr.s_addr = inet_addr("127.0.0.1");

            memset(serv_addr.sin_zero, 0, 8);
            if (connect(sock_fd, (struct sockaddr *)&serv_addr, sizeof(struct sockaddr)) == -1)
            {
                printf("connect error!\n");
                this->ConnectState = internal::CONNECT_ERROR;
                exit(0);
            }else{
                printf("connect success!\n");
            }
            this->ConnectState = internal::GOOD;
        };


        int sendFlag(Com_Data send_data)
        {
            if (send(sock_fd, (char*)(&send_data), sizeof(send_data), 0) == -1)
            {
                printf("send error!\n");
                this->ConnectState = internal::SEND_ERROR;
                exit(0);
            } else{
                printf("send success!\n");
            }
            
        };

        int receive_steps()
        { // 24 * 8
            receiveSteps.clear();
            printf("enter receive function\n");
            char temp[128]={};
            memset(temp, 0, sizeof(temp));
            if ((recvbytes = recv(sock_fd, temp, sizeof(temp), 0)) == -1)
            {
                printf("recv error!\n");
                this->ConnectState = internal::RECV_ERROR;
                exit(0);
            }else{
                printf("recv success!\n");
            }
            footstep tmp_step;
            memcpy(&tmp_step, temp, sizeof(footstep));
            receiveSteps.emplace_back(tmp_step);
            
            // int num = *((int*)(temp));//将指针“temp”强制转换为整数指针，然后取消引用它以获取它指向的整数的值。
            // std::cout<<"num "<<num<<std::endl;
            // for (size_t i = 0; i < num; i++)
            // {
            //     std::cout<<"i = "<<i<<std::endl;
            //     footstep tmp_step;
            //     memcpy(&tmp_step, temp + 8 + i*(sizeof(footstep)), sizeof(footstep));
            //     receiveSteps.emplace_back(tmp_step);
            // }
            // struct GetData
            // {
            //     int num;
            //     std::vector<footstep> footsteps;
            // };
            // GetData getdata;
            // getdata.num = num;
            // getdata.footsteps.resize(num);
            // memcpy(&getdata, temp, sizeof(getdata));
            // receiveSteps = getdata.footsteps;
            printf("Received force.\n");
            return 1;
        };

        void close()
        {
            close();
        };
        ~tcpip_port()
        {
            close();
        }
    };
}