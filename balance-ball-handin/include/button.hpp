#pragma once
#include "message.hpp"

class Button
{
    bool isActive;
    int fd;
    int gpio;

public:
    explicit Button(std::string path_name, int gpio);
    ~Button();
    void ButtonThread();
};
