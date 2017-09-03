//
// Created by wuwei on 17-9-3.
//

#ifndef UAVPRODUCT_EVENTSENDER_H
#define UAVPRODUCT_EVENTSENDER_H

#include "event2/event.h"
#include <string>

class eventSender {
public:
    //监听端口,获取从客户端发送数据的消息
    void eventReceiver(int port);
};


#endif //UAVPRODUCT_EVENTSENDER_H
