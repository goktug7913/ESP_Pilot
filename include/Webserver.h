// Configuration and telemetry webserver
#include <WiFi.h>
#include "SPIFFS.h"
#include "ESPAsyncWebServer.h"

class Webserver {

    const char* ssid     = "Reapers";
    const char* password = "Reapers21";
    
    String header;

    bool active = false;

    TaskHandle_t task;

    public:
        void init();
        static void handleClient();
};
