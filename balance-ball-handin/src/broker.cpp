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
#include "broker.hpp"

// ------------------------------
// Broker (Thread-Safe Singleton)
// ------------------------------
void Broker::subscribe(const std::string& topic, std::shared_ptr<IConsumer> consumer) {
    std::lock_guard<std::mutex> lock(mtx);
    subscribers[topic].push_back(consumer);
}

// Weak Pointer solution (solution) (Not thread safe!!)
//void Broker::publish(const Message& msg)
// void Broker::publish(std::unique_ptr<Message> msg)  
// {
//     {
//         std::lock_guard<std::mutex> lock(mtx);

//         // Look up Topic in subscribers
//         auto subscribers_it = subscribers.find(msg->topic);
//         if (subscribers_it == subscribers.end()) {
//             std::cout << "[Broker] No subscribers for topic: " << msg->topic << "\n";
//             return;
//         }

//         // Publish to all IConsumers of topic
//         // But deadlocks if consumer publishes message! 
//         for (auto consumer : subscribers_it->second)
//         {
//             if (auto c = consumer.lock())
//             c->onMessage(*msg);
//         }

//     }
// }

void Broker::publish(std::unique_ptr<Message> msg)
{
    std::vector<std::weak_ptr<IConsumer>> copiedSubscribers;

    {
        std::lock_guard<std::mutex> lock(mtx);

        auto it = subscribers.find(msg->topic);
        if (it == subscribers.end()) {
            std::cout << "[Broker] No subscribers for topic: " << msg->topic << "\n";
            return;
        }

        // Copy list of subscribers
        copiedSubscribers = it->second;
    } // <-- mutex unlocks here

    // Now it's safe to call into user code
    for (auto& subscriberWeak : copiedSubscribers)
    {
        if (auto consumer = subscriberWeak.lock())
            consumer->onMessage(*msg);  // no lock held
    }
}



void Broker::unsubscribe(const std::string& topic, std::shared_ptr<IConsumer> consumer) 
{
    {
        std::lock_guard<std::mutex> lock(mtx);

        // Look up Topic in subscribers
        auto subscribers_it = subscribers.find(topic);
        if (subscribers_it == subscribers.end()) {
            // Topic not found
            return;
        }

        // Erase IConsumer from topic
        for (auto consumer_it = subscribers_it->second.begin(); consumer_it != subscribers_it->second.end();) 
        {
            if(consumer_it->lock() == consumer)
                consumer_it = subscribers_it->second.erase(consumer_it);

        }

        // Erase topic if empty
        if (subscribers_it->second.size() == 0) // if IConsumer.size == 0
            subscribers_it = subscribers.erase(subscribers_it);
    }
}

