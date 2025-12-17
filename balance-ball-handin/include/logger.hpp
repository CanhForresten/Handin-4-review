#pragma once
#include "iconsumer.hpp"
#include "message.hpp"

class Logger : public IConsumer
{

public:
//    explicit Led(std::string path_name);
    void onMessage(const Message &msg) override;
};
