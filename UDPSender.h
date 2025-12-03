// UDPSender.h
#ifndef UDPSENDER_H
#define UDPSENDER_H

#include <netinet/in.h>   // sockaddr_in, htons, etc.
#include <sys/socket.h>   // socket(), sendto(), etc.
#include <arpa/inet.h>    // inet_addr()
#include <unistd.h>       // close()

#include "UDPMessage.h"

#pragma comment(lib, "ws2_32.lib")

class UDPSender {
private:
    int sock;
    sockaddr_in receiverAddr;

public:
    UDPSender(const std::string& ip, int port);
    ~UDPSender();
    void send_data(const UDPMessage& msg);
};

#endif // UDPSENDER_H
