//
// Created by gaoxiang on 19-7-15.
//

#ifndef MAPPING_MAPPING_CLIENT_H
#define MAPPING_MAPPING_CLIENT_H

#include <string>

namespace mapping::app {

class MappingClient {
   public:
    MappingClient();

    /// 执行指令，返回结果
    bool DoCommand(const std::string &cmd);

   private:
    int port_ = 6666;
    std::string server_ip_ = "127.0.0.1";
};

}  // namespace mapping::app

#endif  // MAPPING_MAPPING_CLIENT_H
