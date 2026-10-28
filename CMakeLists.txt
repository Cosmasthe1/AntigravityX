cmake_minimum_required(VERSION 3.16)

# AntigravityX — ESP-IDF project root
# Target: ESP32 (Xtensa LX6, dual-core, 240 MHz)
#
# Build:
#   idf.py set-target esp32
#   idf.py build
#   idf.py -p /dev/ttyUSB0 flash monitor

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(AntigravityX)
