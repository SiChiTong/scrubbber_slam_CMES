//
// Created by gaoxiang on 19-7-15.
//

#include <arpa/inet.h>
#include <netinet/in.h>
#include <iostream>

#include <glog/logging.h>

#include "app/mapping_server/mapping_client.h"
#include "io/yaml_io.h"

namespace mapping::app {

MappingClient::MappingClient() {
    // load server ip and port in config/server.yaml
    io::YAML_IO yaml("./config/server.yaml");
    server_ip_ = yaml.GetValue<std::string>("server.ip");
    port_ = yaml.GetValue<int>("server.port");

    bool use_internal_ip = yaml.GetValue<bool>("server.use_internal_ip");
    if (use_internal_ip) {
        server_ip_ = yaml.GetValue<std::string>("server.internal_ip");
    }
}

bool MappingClient::DoCommand(const std::string &cmd) {
    int client_sockfd;
    struct sockaddr_in remote_addr {};
    memset(&remote_addr, 0, sizeof(remote_addr));
    remote_addr.sin_family = AF_INET;
    remote_addr.sin_addr.s_addr = inet_addr(server_ip_.c_str());
    remote_addr.sin_port = htons(port_);

    if ((client_sockfd = socket(PF_INET, SOCK_STREAM, 0)) < 0) {
        LOG(ERROR) << "socket error";
        return false;
    }

    if (connect(client_sockfd, (struct sockaddr *)&remote_addr, sizeof(struct sockaddr)) < 0) {
        LOG(ERROR) << "connect error";
        return false;
    }

    send(client_sockfd, cmd.c_str(), cmd.size(), 0);

    char buf[100000] = "";
    size_t len = recv(client_sockfd, buf, 100000, 0);
    if (len > 0 && len < 100000) {
        buf[len] = '\0';
    }
    std::string ret(buf);
    std::cout << ret << std::endl;

    close(client_sockfd);
    return true;
}

}  // namespace mapping::app