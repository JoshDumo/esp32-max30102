# MAX30102 Library
Author: Joshua D. JOHN
Date: 2024/07/26

This project develops a library in C++ for the MAX30102 on ESP32. This documents the preparation and development of the project.

## Outline
### Setting up Dev Environment on Windows
### Loading up a Getting Started
### Loading up a C++ Getting Started
### Analyze how to I2C communication

## Setting up Dev Environment on Windows
Please follow the instructions at 
https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/windows-setup.html

The required software packages are downloaded and stored in Projects/Storage.

## Loading up a Getting Started
Clone the esp32 repository to some folder
https://github.com/espressif/esp-idf.git

The follow the instructions here 
https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/windows-setup.html#get-started-windows-first-steps

When the device is connected it should be visible under Device Manager->Ports as SiliconLabs CP210x

Build the project
``` sh
cd \hello_world
idf.py set-target esp32
idf.py menuconfig

idf.py build

idf.py -p COM5 flash

idf.py -p COM5 monitor
```
Remember to press the button on the right to reset and trigger the flashing.

## Loading up a C++ Getting Started








