#pragma once
#include <mutex>
#include "SSD1306_OLED.hpp"
#include "iconsumer.hpp"
#include "game_state.hpp"

class Display
{
public:
    explicit Display(SSD1306 &oledRef);
    void drawDisplay(GameState gameState);
private:
    SSD1306 &oled;
    std::mutex display_mtx;
};
