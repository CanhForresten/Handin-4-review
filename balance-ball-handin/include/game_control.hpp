#pragma once
#include "SSD1306_OLED.hpp"
#include "iconsumer.hpp"
#include "display.hpp"
#include "game_state.hpp"

class GameControl : public IConsumer
{
    Display *display;
    GameState gameState;
    void handleAccelerometer(const std::string& data);
    void handleButton(const std::string& data);
    void handleBoundary(const std::string& data);

public:
    explicit GameControl(Display& display);
    void onMessage(const Message &msg) override;
};
