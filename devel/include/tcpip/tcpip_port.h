#ifndef _TCPIP_PORT_H_
#define _TCPIP_PORT_H_
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <arpa/inet.h>  
#include <unistd.h> 
#include "data_type.h"
#include <vector>
class tcpip_port
{
private:
    int sock_fd,client_fd;
    int sin_size;
    struct sockaddr_in my_addr;
    struct sockaddr_in remote_addr;
    char recevBuf[200];
    char recvFlag = '\0';
public:
    LEECHAO::Com_Data com_data;
    tcpip_port(/* args */);
    ~tcpip_port();
    int initial();
    
    int accept_client();
    int recvChar();
    int recvData();
    void analysisBuf();
    int sendSteps(std::vector<LEECHAO::FootStep> footsteps);
    void close_client();
    void closetcpip();
    char getRecvFlag();
    void resetRecvFlag();
};

#endif