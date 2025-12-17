cmake_minimum_required(VERSION 3.10)

project(BalancingBall LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Add include path
include_directories(${CMAKE_SOURCE_DIR}/include)

# Create executable from source files
add_executable(balance_ball
    src/accelerometer.cpp
    src/balance_ball.cpp
    src/broker.cpp
    src/button.cpp
    src/display.cpp
    src/I2Cdriver.cpp
    src/SPIdriver.cpp
    src/game_control.cpp
    src/logger.cpp
    src/led.cpp
)

# If I2Cdriver needs external libraries (e.g., -lrt), link them here:
target_link_libraries(balance_ball PRIVATE SSD1306_OLED_RPI)
target_link_libraries(balance_ball PRIVATE BMI160Wrapper)