#include <thread>
#include <iostream>
#include "display.hpp"
#include "bitmaps.hpp"
#include "message.hpp"
#include "broker.hpp"
#include "game_control.hpp"
#include "game_state.hpp"


static const int ballCenterPosX = 64;
static const int ballCenterPosY = 16;


GameControl::GameControl(Display& display) : display(&display) {
    gameState.ball_x = ballCenterPosX;
    gameState.ball_y = ballCenterPosY;
    gameState.score = 0;
    display.drawDisplay(gameState);
}

void GameControl::handleAccelerometer(const std::string& data) {
    double x, y, z;
    int speed = 4;

    

    /*
     * TODO:
     * Decode the message data
     * Update gameState ball_x, ball_y according to accl values
     * Update gameState.score according to accl values
     * Draw display
     * Activity 5: publish 'boundary' messsage if ball out of screen
     */
    Message::decodeAccelerometerData(data, &x, &y, &z);
    const double hys = 0.25;

    int newX = gameState.ball_x;
    int newY = gameState.ball_y;

    if (y > hys){
        newX -= speed;
    }
    else if(y < -hys){
        newX += speed;
    }

    if (x > hys){
        newY -= speed;
    }
    else if (x < -hys){
        newY += speed;
    }

    const int minX = 0;
    const int minY = 0;
    const int maxX = 127;
    const int maxY = 31;

    bool outofscreen = false;

    if (newX < minX){
        newX = minX;
        outofscreen = true;
    }
    else if (newX > maxX){
        newX = maxX;
        outofscreen = true;
    }

    if (newY < minY){
        newY = minY;
        outofscreen = true;
    }
    else if (newY > maxY){
        newY = maxY;
        outofscreen = true;
    }

    gameState.ball_x = newX;
    gameState.ball_y = newY;

    if (!outofscreen){
        if((std::abs(x) + std::abs(y)) > hys){
        gameState.score += ((std::abs(x) + std::abs(y)) * 100);
    }
        auto msg = std::make_unique<Message>("boundary", "0");
        Broker::getInstance().publish(std::move(msg));
    }
    else {
        auto msg = std::make_unique<Message>("boundary", "1");
        Broker::getInstance().publish(std::move(msg));
    }

    display->drawDisplay(gameState);
    }

void GameControl::handleButton(const std::string& data) {
    int gpio, value;

    Message::decodeButtonData(data, &gpio, &value);
    if (value == 0) 
    {
        std::cout << "RESET" << std::endl;
        /*
         * TODO:
         * Center ball
         * Reset score
         */
        gameState.ball_x = ballCenterPosX;
        gameState.ball_y = ballCenterPosY;
        gameState.score = 0;
        display->drawDisplay(gameState);
    }
}

/*
* TODO:
* Create handler for 'boundary' messages
* Subtract score by 1000 if the data value is "1"
*/
    void GameControl::handleBoundary(const std::string& data){
        if (data == "1"){
            gameState.score -= 1000;
        }
        display->drawDisplay(gameState);

    }


void GameControl::onMessage(const Message& msg)
{
    if (msg.topic == "btn") {
        handleButton(msg.data);
    }
    else if (msg.topic == "accl") {
        handleAccelerometer(msg.data);
    }
    else if (msg.topic == "boundary") {
        handleBoundary(msg.data);
    }
}
