#include "aruco_server.h"
#include "globals.h"

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#include <iostream>
#include <cstdlib>

void aruco_socket_server(int port)
{
    int server_fd = socket(AF_INET, SOCK_STREAM, 0);
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = inet_addr("127.0.0.1");
    address.sin_port = htons(port);
    bind(server_fd, (struct sockaddr *)&address, sizeof(address));
    listen(server_fd, 1);
    std::cout << "Aruco socket server listening on 127.0.0.1:" << port << std::endl;
    while (true)
    {
        int client_fd = accept(server_fd, nullptr, nullptr);
        char buffer[32];
        while (true)
        {
            ssize_t len = recv(client_fd, buffer, sizeof(buffer) - 1, 0);
            if (len > 0)
            {
                buffer[len] = '\0';
                int id = atoi(buffer);
                g_last_aruco_id = id;
                std::cout << "Received aruco id: " << id << std::endl;
            }
            else if (len == 0)
            {
                close(client_fd);
                break;
            }
        }
    }
    close(server_fd);
}