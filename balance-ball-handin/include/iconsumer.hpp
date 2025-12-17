#pragma once
#include "message.hpp"

// ---------------------------
// IConsumer
// ---------------------------
class IConsumer {
public:
//    IConsumer(std::string n) : name(std::move(n)) {}
    virtual ~IConsumer() {}
    virtual void onMessage(const Message &msg) = 0;
    // void onMessage(const Message& msg) {
    //     std::cout << "[IConsumer " << name << "] received data on topic " << msg.topic << "\n";
    // }
};
