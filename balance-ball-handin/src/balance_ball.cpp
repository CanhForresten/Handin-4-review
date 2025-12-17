#include <memory>
#include "I2Cdriver.hpp"
#include "SSD1306_OLED.hpp"
#include "display.hpp"
#include "button.hpp"
#include "accelerometer.hpp"
#include "broker.hpp"
#include "led.hpp"
#include "game_control.hpp"
#include "logger.hpp"
#include "led.hpp"

#define myOLEDwidth 128
#define myOLEDheight 32
#define FULLSCREEN (myOLEDwidth * (myOLEDheight / 8))
int main(int argc, char *argv[]) {
  I2CDriver i2cDriver;
  SSD1306 oled(myOLEDwidth, myOLEDheight, i2cDriver);
  uint8_t screenBuffer[FULLSCREEN];
  if (!oled.OLEDSetBufferPtr(myOLEDwidth, myOLEDheight, screenBuffer, sizeof(screenBuffer)))
  return -1;
  oled.OLEDbegin(0x3c); // initialize the OLED

  Display display(oled);

  // Create and add consumers to Broker
  auto gameCtrl = std::make_shared<GameControl>(display);
  auto led = std::make_shared<Led>("/dev/mygpio-led");
  Broker::getInstance().subscribe("accl", gameCtrl);
  Broker::getInstance().subscribe("btn", gameCtrl);
  Broker::getInstance().subscribe("boundary", gameCtrl);
  Broker::getInstance().subscribe("boundary", led);

  auto logger = std::make_shared<Logger>();
  Broker::getInstance().subscribe("btn", logger);
  Broker::getInstance().subscribe("boundary", logger);
  Broker::getInstance().subscribe("accl", logger);

  // Create Publishers
  Accelerometer accl("/dev/spidev0.0");
  Button but27("/dev/mygpio-but0", 27);

  // Start Publisher threads
  std::thread t1([&accl]() { accl.accelerometerThread(); });
  std::thread t2([&but27]() { but27.ButtonThread();});

  t1.join();
  t2.join();
 
  //oled.OLEDPowerDown();

  return 0;
}
