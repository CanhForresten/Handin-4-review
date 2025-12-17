#pragma once
#include <iostream>
#include <string>
#include <map>
#include <vector>
#include <functional>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <queue>
#include <memory>
#include <chrono>
#include "message.hpp"
#include "iconsumer.hpp"

// ------------------------------
// Broker (Thread-Safe Singleton)
// ------------------------------
class Broker {
private:
    std::map<std::string, std::vector<std::weak_ptr<IConsumer>>> subscribers;
    std::mutex mtx;
    Broker() = default;

public:
    Broker(const Broker&) = delete;
    Broker& operator=(const Broker&) = delete;

    static Broker& getInstance() {
        static Broker instance;
        return instance;
    }

    void subscribe(const std::string &topic, std::shared_ptr<IConsumer> IConsumer);

    void unsubscribe(const std::string &topic, std::shared_ptr<IConsumer> IConsumer);

    void publish(std::unique_ptr<Message> msg);

    // void subscribe(const std::string& topic, std::shared_ptr<IConsumer> IConsumer) {
    //     std::lock_guard<std::mutex> lock(mtx);
    //     subscribers[topic].push_back(IConsumer);
    // }

    // // Weak Pointer solution (solution)
    // void publish(const std::string& topic, const Message& msg) 
    // {
    //     {
    //         std::lock_guard<std::mutex> lock(mtx);

    //         // Look up Topic in subscribers
    //         auto subscribers_it = subscribers.find(topic);
    //         if (subscribers_it == subscribers.end()) {
    //             std::cout << "[Broker] No subscribers for topic: " << topic << "\n";
    //             return;
    //         }

    //         // Publish to all IConsumers of topic
    //         for (auto IConsumer : subscribers_it->second)
    //         {
    //             if (auto c = IConsumer.lock())
    //             c->onMessage(msg);
    //         }

    //     }
    // }


    // void unsubscribe(const std::string& topic, std::shared_ptr<IConsumer> IConsumer) 
    // {
    //     {
    //         std::lock_guard<std::mutex> lock(mtx);

    //         // Look up Topic in subscribers
    //         auto subscribers_it = subscribers.find(topic);
    //         if (subscribers_it == subscribers.end()) {
    //             // Topic not found
    //             return;
    //         }

    //         // Erase IConsumer from topic
    //         for (auto IConsumer_it = subscribers_it->second.begin(); IConsumer_it != subscribers_it->second.end();) 
    //         {
    //             if(IConsumer_it->lock() == IConsumer)
    //                 IConsumer_it = subscribers_it->second.erase(IConsumer_it);

    //         }

    //         // Erase topic if empty
    //         if (subscribers_it->second.size() == 0) // if IConsumer.size == 0
    //             subscribers_it = subscribers.erase(subscribers_it);
    //     }
    // }
};


// ---------------------------
// Example Usage
// ---------------------------
// int main() {

//     // Define IConsumers as shared pointers
//     auto c1 = std::make_shared<IConsumer>("Heater");
//     auto c2 = std::make_shared<IConsumer>("Fan");
//     auto c3 = std::make_shared<IConsumer>("Window");

//     // subscribe to topics
//     Broker::getInstance().subscribe("temperature", c1);
//     Broker::getInstance().subscribe("temperature", c2);
//     Broker::getInstance().subscribe("humidity", c2);

//     c1.reset(); // Release "Heater"

//     // unsubscribe to topic (when unsubscribe is implemented)
//     Broker::getInstance().unsubscribe("humidity", c2);


//     std::thread t1([&]() {
//         for (int i = 0; i < 3; ++i) {
//             std::string str{"Temperature is: " + std::to_string(rand() % 40)};
//             Message msg{"temperature", str};
//             Broker::getInstance().publish(msg.topic, msg);
//             std::this_thread::sleep_for(std::chrono::milliseconds(50));
//         }
//     });

//     std::thread t2([&]() {
//         for (int i = 0; i < 3; ++i) {
//             std::string str{"Humidity is: " + std::to_string(rand() % 100)};
//             Message msg{"humidity", str};
//             Broker::getInstance().publish(msg.topic, msg);
//             std::this_thread::sleep_for(std::chrono::milliseconds(70));
//         }
//     });

//     t1.join();
//     t2.join();
//     return 0;
// }
