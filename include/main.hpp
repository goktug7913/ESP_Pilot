// main.hpp
#ifndef MAIN_HPP
#define MAIN_HPP

#include "core.hpp"

class ESP_Pilot {
public:
    ESP_Pilot();
    void setup();
    void loop();
private:
    EsppCore core;
};

#endif // MAIN_HPP