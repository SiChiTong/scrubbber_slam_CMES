//
// Created by gaoxiang on 2020/8/11.
//

#ifndef MAPPING_BAG_CONVERT_H
#define MAPPING_BAG_CONVERT_H

#include "common/message_def.h"
#include "common/std_headers.h"

#include <glog/logging.h>
#include <rosbag/bag.h>
#include <functional>

namespace mapping::pipeline {

/**
 * 将不同车辆采集到的数据转换为统一格式
 */
class BagConvert {
   public:
    BagConvert();

    ~BagConvert(){}

    /**
     * 输入包转输出包
     * @param in_file  待转换的包路径
     * @param out_file 转换完成的包路径
     * @return 是否成功
     */
    bool Run(const std::string& in_file, const std::string& out_file);

    /**
     * 数据处理函数定义：处理一个message instance，如果可以处理，写入至out，返回true;
     */
    using MessageProcessFunction = std::function<bool(const rosbag::MessageInstance& m, rosbag::Bag& out)>;

   private:
    /**
     * 不需要处理，直接转存的消息
     * @tparam T 消息类型
     * @param topic_name 消息名称
     * @return
     */
    template <typename T>
    MessageProcessFunction MakeDirectPassFunc(const std::string& topic_name) {
        return [&topic_name](const rosbag::MessageInstance& m, rosbag::Bag& out) -> bool {
            auto msg = m.instantiate<T>();
            if (msg != nullptr && m.getTopic() == topic_name && msg->header.stamp.isZero() == false) {
                out.write(topic_name, msg->header.stamp, *msg);
                return true;
            }
            return false;
        };
    }

    /**
     * 需要转换的消息
     * @tparam T1   旧消息类型
     * @tparam T2   新消息类型
     * @param topic_name topic名
     * @return
     */
    template <typename T1, typename T2>
    MessageProcessFunction MakeConvertFunc(const std::string& topic_name,
                                           const std::function<void(const T1& old_msg, T2& new_msg)>& convert_func) {
        return [&topic_name, &convert_func](const rosbag::MessageInstance& m, rosbag::Bag& out) -> bool {
            auto msg = m.instantiate<T1>();
            if (msg != nullptr && m.getTopic() == topic_name && msg->header.stamp.isZero() == false) {
                auto new_msg = boost::make_shared<T2>();
                convert_func(*msg, *new_msg);
                out.write(topic_name, new_msg->header.stamp, *new_msg);
                return true;
            }
            return false;
        };
    }

    template <typename T1>
    MessageProcessFunction MakeConvertFunc(
        const std::string& topic_name, const std::function<void(const T1& old_msg, AppWxbMsg& new_msg)>& convert_func) {
        return [&topic_name, &convert_func](const rosbag::MessageInstance& m, rosbag::Bag& out) -> bool {
            auto msg = m.instantiate<T1>();
            if (msg != nullptr && m.getTopic() == topic_name && !m.getTime().isZero()) {
                auto new_msg = boost::make_shared<AppWxbMsg>();
                convert_func(*msg, *new_msg);
                out.write(topic_name, m.getTime(), *new_msg);
                return true;
            }
            return false;
        };
    }

    template <typename T1>
    MessageProcessFunction MakeConvertFunc(
        const std::string& topic_name, const std::function<void(const T1& old_msg, AppWbdMsg& new_msg)>& convert_func) {
        return [&topic_name, &convert_func](const rosbag::MessageInstance& m, rosbag::Bag& out) -> bool {
            auto msg = m.instantiate<T1>();
            if (msg != nullptr && m.getTopic() == topic_name && !m.getTime().isZero()) {
                auto new_msg = boost::make_shared<AppWbdMsg>();
                convert_func(*msg, *new_msg);
                out.write(topic_name, m.getTime(), *new_msg);
                return true;
            }
            return false;
        };
    }

    void ReIndexBag();

    void DoBagConvert();

    std::string in_path_;
    std::string out_path_;

    std::vector<MessageProcessFunction> process_funcs_;
};

/// 没有header的消息特化处理
template <>
inline BagConvert::MessageProcessFunction BagConvert::MakeDirectPassFunc<AppWxbMsg>(const std::string& topic_name) {
    return [&topic_name](const rosbag::MessageInstance& m, rosbag::Bag& out) -> bool {
        auto msg = m.instantiate<AppWxbMsg>();
        if (msg != nullptr && m.getTopic() == topic_name) {
            out.write(topic_name, m.getTime(), *msg);
            return true;
        }
        return false;
    };
}

template <>
inline BagConvert::MessageProcessFunction BagConvert::MakeDirectPassFunc<AppWbdMsg>(const std::string& topic_name) {
    return [&topic_name](const rosbag::MessageInstance& m, rosbag::Bag& out) -> bool {
        auto msg = m.instantiate<AppWbdMsg>();
        if (msg != nullptr && m.getTopic() == topic_name) {
            out.write(topic_name, m.getTime(), *msg);
            return true;
        }
        return false;
    };
}

template <>
inline BagConvert::MessageProcessFunction BagConvert::MakeDirectPassFunc<SuTengPacketsMsg>(
    const std::string& topic_name) {
    return [&topic_name](const rosbag::MessageInstance& m, rosbag::Bag& out) -> bool {
        auto msg = m.instantiate<SuTengPacketsMsg>();
        if (msg != nullptr && m.getTopic() == topic_name && m.getTime().isZero() == false) {
            msg->stamp = m.getTime();  // 使用stamp接口，保持后续操作统一
            out.write(topic_name, m.getTime(), *msg);
            return true;
        }
        return false;
    };
}
// 函数预留着。以防万一
   template <>
   inline BagConvert::MessageProcessFunction BagConvert::MakeDirectPassFunc<SuTengScanMsg>(
       const std::string& topic_name) {
       return [&topic_name](const rosbag::MessageInstance& m, rosbag::Bag& out) -> bool {
            auto msg = m.instantiate<SuTengScanMsg>();
            if (msg != nullptr && m.getTopic() == topic_name && msg->header.stamp.isZero() == false) {
                // msg->header.stamp.nsec *= 1000;
                out.write(topic_name, msg->header.stamp, *msg);
                return true;
            }
            return false;
        };
    }

}  // namespace mapping::pipeline

#endif  // MAPPING_BAG_CONVERT_H
