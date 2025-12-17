#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <thread>
#include <iostream>
#include "broker.hpp"
#include "button.hpp"

Button::Button(std::string path_name, int gpio) : gpio(gpio)
{
    /*
     * TODO: 
     * Request GPIO Inputs
     * - Open gpio device
     */

     fd = open(path_name.c_str(), O_RDONLY);
     if (fd == -1){
        perror("open");  
        throw std::runtime_error("Failed to open GPIO device: " + path_name);
     }

    isActive = true;
}

Button::~Button()
{
    isActive = false;
    /*
     * Close File descriptors
     */
    if (fd >= 0){
        close(fd);
    }
}

void Button::ButtonThread()
{
    while (isActive)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        /*
         * TODO:
         * Read value from button device node (Posix)
         * Encode value to message data string
         * Create a message
         * Publish the message
         */
        char buffer[4];
        ssize_t bytesRead = read(fd, buffer, sizeof(buffer));
        if(bytesRead == -1){
            perror("read");
        }

        int value = (buffer[0] == '1') ? 1 : 0;

        std::string data = Message::encodeButtonData(gpio,value);

        auto msg = std::make_unique<Message>("btn", data);

        Broker::getInstance().publish(std::move(msg));
    }
}
