#include <winsock2.h>
#include <stdio.h>
#include <stdlib.h>

#define IP "192.168.3.41"
#define PORT 6000
#define BUFFER_SIZE 1024

int main() {
    WSADATA wsaData;
    SOCKET sock;
    struct sockaddr_in serverAddr;
    char *message = "101";
    char buffer[BUFFER_SIZE];
    int msgLength;

    // 初始化Winsock
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
        fprintf(stderr, "WSAStartup failed: %d\n", WSAGetLastError());
        return 1;
    }

    // 创建套接字
    sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock == INVALID_SOCKET) {
        fprintf(stderr, "socket creation failed: %d\n", WSAGetLastError());
        WSACleanup();
        return 1;
    }

    // 设置服务器地址结构
    memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(PORT);
    serverAddr.sin_addr.s_addr = inet_addr(IP);

    // 发送数据
    msgLength = sendto(sock, message, strlen(message), 0,
                       (struct sockaddr *)&serverAddr, sizeof(serverAddr));
    if (msgLength == SOCKET_ERROR) {
        fprintf(stderr, "sendto failed: %d\n", WSAGetLastError());
        closesocket(sock);
        WSACleanup();
        return 1;
    }

    printf("Message sent successfully.\n");

    // 关闭套接字和清理Winsock
    closesocket(sock);
    WSACleanup();

    return 0;
}

