/***************************************************************************************************************************
* message_utils.h
*
* Author: Jario
*
* Update Time: 2020.4.29
***************************************************************************************************************************/
#ifndef DRONE_MESSAGE_UTILS_H
#define DRONE_MESSAGE_UTILS_H


#include <string>
#include <drone_msg/Message.h>
using namespace std;

// 消息发布函数
inline void pub_message(ros::Publisher& puber, int msg_type, std::string source_node, std::string msg_content)
{
    drone_msg::Message exect_msg;
    exect_msg.header.stamp = ros::Time::now();
    exect_msg.message_type = msg_type;
    exect_msg.source_node = source_node;
    exect_msg.content = msg_content;
    puber.publish(exect_msg);
}

// 消息打印函数
inline void printf_message(const drone_msg::Message& message)
{
    //cout <<">>>>>>>>>>>>>>>>>>>>>>>> Message <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
    if(message.message_type == drone_msg::Message::NORMAL)
    {
        cout << "[NORMAL]" << "["<< message.source_node << "]:" << message.content <<endl;
    }else if(message.message_type == drone_msg::Message::WARN)
    {
        cout << "[WARN]" << "["<< message.source_node << "]:" <<message.content <<endl;
    }else if(message.message_type == drone_msg::Message::ERROR)
    {
        cout << "[ERROR]" << "["<< message.source_node << "]:" << message.content <<endl;
    }
    
}

#endif
