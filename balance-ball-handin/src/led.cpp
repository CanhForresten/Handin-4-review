#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <thread>
#include <iostream>
#include "broker.hpp"
#include "led.hpp"

Led::Led(std::string path_name){
    fd = open(path_name.c_str(), O_WRONLY);
     if (fd == -1){
        perror("open");  
        throw std::runtime_error("Failed to open GPIO device: " + path_name);
     }
}

Led::~Led(){
    if (fd >= 0){
        ::close(fd);
        fd = -1;
    }
}

void Led::on()
{
    const char buf[] = "1\n";            // samme som echo 1
    if (::write(fd, buf, sizeof(buf) - 1) < 0) {
        perror("Write Led on");
    }
}

void Led::off()
{
    const char buf[] = "0\n";            // samme som echo 0
    if (::write(fd, buf, sizeof(buf) - 1) < 0) {
        perror("Write Led off");
    }
}

void Led::onMessage(const Message &msg){
    if(msg.topic == "boundary"){
        if (msg.data == "1"){
            on();
        }
        else{
            off();
        }
    }
}
