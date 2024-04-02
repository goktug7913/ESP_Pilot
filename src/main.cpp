// main.cpp
#include "main.hpp"
#include "core.hpp"

EsppCore core;

extern "C" void app_main()
{
    core.start();
}