#include <iostream>
#include <cstring>          // voor memset()
#include <unistd.h>         // voor close()
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "UDPSender.h"

UDPSender::UDPSender(const std::string& ip, int port) {
    sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        std::cerr << "Socket creation failed!" << std::endl;
        exit(1);
    }

    std::memset(&receiverAddr, 0, sizeof(receiverAddr));
    receiverAddr.sin_family = AF_INET;
    receiverAddr.sin_port = htons(port);
    receiverAddr.sin_addr.s_addr = inet_addr(ip.c_str());
}

UDPSender::~UDPSender() {
    close(sock);
}

void UDPSender::send_data(const UDPMessage& msg) {
    int bytesSent = sendto(sock, reinterpret_cast<const char*>(&msg), sizeof(UDPMessage), 0,
                           (sockaddr*)&receiverAddr, sizeof(receiverAddr));

    if (bytesSent < 0) {
        std::cerr << "Sending failed!" << std::endl;
    } else {
        // std::cout << "Sent: " << msg.flag << std::endl;
    }
}
