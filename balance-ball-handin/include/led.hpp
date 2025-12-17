#pragma once
#include "iconsumer.hpp"
#include "message.hpp"

class Led : public IConsumer
{
    int fd;
//    void handleMessage(const Message &msg);

public:
    explicit Led(std::string path_name);
    ~Led();
    void onMessage(const Message &msg) override;
    void on();
    void off();
};
