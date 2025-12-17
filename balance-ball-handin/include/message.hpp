#pragma once
#include <iostream>
#include <memory>
#include <string>
#include <bits/stdc++.h>

// // ---------------------------
// // Message Data Base
// // ---------------------------
// struct MessageData {
//     virtual ~MessageData() = default;
// };

// // ---------------------------
// // Derived Message Data Types
// // ---------------------------
// struct AccelerometerData : public MessageData {
//     int x, y, z;
// };

// struct ButtonData : public MessageData {
//     int gpio, state;
// };

// ---------------------------
// Message
// ---------------------------
struct Message {
    std::string topic;
    std::string data;
    
    Message(std::string topic, std::string data) : topic(std::move(topic)), data(std::move(data)) {}

    static std::string encodeAccelerometerData(const double &x, const double &y, const double &z) {
        return std::to_string(x) + "," + std::to_string(y) + "," + std::to_string(z);
    }

    static void decodeAccelerometerData(std::string data, double *x, double *y, double *z) {
        std::stringstream ss(data);
        char comma;

        ss >> *x >> comma >> *y >> comma >> *z;
    }

    static std::string encodeButtonData(const int &gpio, const int &value) {
        return std::to_string(gpio) + "," + std::to_string(value);
    }

    static void decodeButtonData(std::string data, int *gpio, int *value) {
        std::stringstream ss(data);
        char comma;

        ss >> *gpio >> comma >> *value;
    }

};
