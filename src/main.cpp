// main.cpp
#include "main.hpp"
#include "core.hpp"

EsppCore core;

// Entry point
// FreeRTOS will call this function as the first task.
extern "C" void app_main()
{
    core.start();
    return;
}