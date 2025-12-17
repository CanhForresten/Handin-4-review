#include <thread>
#include <iostream>
#include "display.hpp"
#include "bitmaps.hpp"
#include "message.hpp"
#include "broker.hpp"

static const int ballCenterPosX = 64;
static const int ballCenterPosY = 16;

Display::Display(SSD1306 &oledRef) : oled(oledRef) {
    oled.OLEDclearBuffer();
}

void Display::drawDisplay(GameState gameState) {
    std::lock_guard<std::mutex> lock(display_mtx);

    // Clear display buffer first
    oled.OLEDclearBuffer();

    // Draw ball
    oled.OLEDBitmap(gameState.ball_x, gameState.ball_y, 8, 8, ballBitmap, false);

    //  Render score as text
    oled.setTextColor(WHITE);
    oled.setTextSize(1);
    oled.setCursor(0, 0);
    oled.print("Score ");
    oled.print(gameState.score);

    // Send buffer to OLED
    oled.OLEDupdate();
}
