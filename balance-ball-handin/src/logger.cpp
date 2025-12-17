#include <iostream>
#include "logger.hpp"


void Logger::onMessage(const Message &msg)
{
    std::cout << "[" << msg.topic << "] " << msg.data << std::endl;
}