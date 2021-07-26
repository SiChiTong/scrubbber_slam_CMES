//
// Created by gaoxiang on 2019/12/17.
//

#ifndef HAMO_LINUX_SYSTEM_H
#define HAMO_LINUX_SYSTEM_H

#include <string>

namespace HAMO {

// TODO 这些函数依赖操作系统，寻找Linux下的实现
class System {
   public:
    static int GetNumberOfProcessors();

    /**
     * 获取当前程序的路径
     * @param szBuffer  预分配的buffer
     * @param nBufSize
     * @return 0 if success
     */
    static int GetAppPath(std::string &path);

    static bool MakeDirectory(const char *szDir);

    static bool DeleteDirectory(const char *szDir);

    static bool IsDirectoryExist(const char *szDir);

    static bool IsFileExist(const char *szFile);

    static bool RemoveFile(const char *szFile);
};

}  // namespace HAMO

#endif  // HAMO_LINUX_SYSTEM_H
