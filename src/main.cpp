// main.cpp
#include "main.hpp"

ESP_Pilot::ESP_Pilot() : core() {}

void ESP_Pilot::setup() {
    // Initialize system components
    // This would include setting up RTOS tasks, initializing drivers, etc.
    core.init();
}

void ESP_Pilot::loop() {
    // Main application loop
    while (true) {
        // Execute flight control loop
        core.update();
    }
}

int main() {
    ESP_Pilot app;
    app.setup();
    app.loop();
    return 0; // This should never be reached
}