// Configuration and telemetry webserver
#include <WiFi.h>
#include "SPIFFS.h"
#include "ESPAsyncWebServer.h"

class Webserver {

    const char* ssid     = "";
    const char* password = "";
    
    String header;

    bool active = false;

    TaskHandle_t task;
    File homepage;

    public:
        void init();
        void handleClient();
};
