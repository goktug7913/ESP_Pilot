# ESP Pilot - Open source flight controller for ESP32

# What is this?
This is an open source flight controller for ESP32. It's still in development, <b>IT IS NOT FLIGHT-WORTHY RIGHT NOW.</b>

I'm planning to add more features in the future, but I'm not sure when I'll have time to work on it. I'm also planning to add support for more hardware, but I don't have all the hardware to test it. If you want to help, you can open an issue or a pull request.

# Why?
I wanted to build a quadcopter, but I didn't want to purchase a pre-made flight controller. I also wanted to expand my knowledge on flight controllers, so I decided to create one myself. However, I couldn't find any suitable open-source flight controllers for ESP32, so I took it upon myself to build one from scratch.

# Why ESP32?
When it comes to building a flight controller, the most commonly used microcontrollers are the STM32 and Arduino. However, I wanted to experiment with something different and chose to use the ESP32 microcontroller.

The ESP32 is a powerful microcontroller with a dual-core processor, 520kB of RAM, and 4MB of flash memory. This combination of features makes it an excellent choice for a flight controller. Additionally, the ESP32 has built-in WiFi and Bluetooth capabilities, which can be used for telemetry and configuration purposes. This is an advantage over traditional flight controllers that require additional hardware for such features.

Another reason why the ESP32 is a great choice for a flight controller is its affordability. A development board can be purchased for less than $5, making it an accessible and cost-effective option for hobbyists and DIY enthusiasts.

Overall, the ESP32's processing power, built-in connectivity options, and affordability make it a compelling choice for a flight controller.

# Development Hardware
- NodeMCU-32S
- MPU6050
- F450 Frame
- 4x 30A XXD HW30A PWM ESC's
- 4x Racerstar 2212 BR2212 980KV 2S-4S Brushless Motors
- 4x 1045 Propellers
- 300W 12V Power Supply (Tuned to 11.1V with trimpot on the power supply)
- Flysky FS-IA6 Receiver
- Flysky FS-I6 Transmitter
- 1000uF capacitor on ESP's power rail
- Good 'ol breadboard and jumper wires (Seriously, don't do this.)

There is also a compass, barometer and ultrasonic sensor, but they're not used yet, so I didn't include them in the list.
Right now, I'm using a power supply instead of batteries because batteries are expensive and I don't have any. I'm planning to use LiPo batteries in the future. The tether might be causing stability issues even though it's very light. I will hopefully change that when I can fund some batteries and a charger.

# Building and flashing
You need PlatformIO to build and upload the binary to ESP32.
- Clone the repository
- Open with PlatformIO (Optimally in VSCode)
- Download the project dependencies
- Edit `include/Config.h` and `include/definitions.h` according to your setup.
- Build and upload.

You can use ESP-Configurator with serial connection to monitor telemetry and configure the flight controller, <b>but that will be deprecated soon</b> as I'm working on a web based configurator. The new configurator will be able to configure the flight controller over WiFi, via the ESP32's AP mode. I might also add a Bluetooth based configurator, but that's not planned yet.

# Features
- Modular PID system, you can add/remove PID controllers for any axis. For example, you can have a PID controller for roll, pitch and yaw, but you can also have a PID controller for altitude, or a PID controller for heading. Some of the PID controllers are already implemented, but you can add your own using the `PIDController` class.
- Configurable over serial connection (via ESP-Configurator) and <b>in the future</b> over WiFi (via web configurator)

# Supported hardware
Currently supports:
- ESP32
- PWM receivers
- PWM controlled ESC's
- MPU6050 based IMU's

# Future plans
- Migration to ESP-IDF from Arduino framework. I'm planning to migrate to ESP-IDF because it's more powerful and it has more features. I'm also planning to use FreeRTOS for multithreading.
- Custom MPU6050 driver. Currently MPU6050_Light is used, but I'm not comfortable with it. I'm planning to write my own driver which will be more configurable and use quaternions instead of Euler angles.
- AP mode and web configurator
- Bluetooth configurator (I don't see a benefit of this, but I might add it)
- Autoland and minimum altitude protection support (I'm planning to use ultrasonic sensor for this)
- Altitude measurement without GPS using barometer.
- RC receiver support (SBUS, PPM, etc.)
- Support for more IMU's
- Support for more ESC's
- GPS support (I don't have a GPS module, so I can't test it)
- Support for more sensors (barometer, compass, etc.)
- Support for more flight modes (currently only supports manual mode)
- Wider range of ESP32 boards
- Logging to SD card
- Long range telemetry (I'm thinking of using LoRa or nRF24L01+)

# Contributing
If you want to contribute, you can open an issue or a pull request. I'll try to respond as soon as possible.

# License
The project will stay open source, but I'm not sure which license to use. Feel free to send me a message if you have any suggestions.