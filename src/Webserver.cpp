#include "Webserver.h"

AsyncWebServer server(80);

void Webserver::init() {
    WiFi.mode(WIFI_MODE_APSTA);
    WiFi.begin(ssid, password);

    // Print the IP address when connected
    while (WiFi.status() != WL_CONNECTED) {
        delay(3000);
        Serial.print(".");
    }

    active = true;

    Serial.println("");
    Serial.print("Connected to ");
    Serial.println(ssid);
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());

    if (!SPIFFS.begin(true)) {
        Serial.println("An Error has occurred while mounting SPIFFS");
        return;
    }

    homepage = SPIFFS.open("/index.html", "r");
    if (!homepage) {
        Serial.println("Failed to open homepage");
    }

    server.begin();

    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
            request->send(SPIFFS, "/index.html", "text/html");
        });

    server.on("/index.css", HTTP_GET, [](AsyncWebServerRequest *request){
            request->send(SPIFFS, "/index.css", "text/css");
        });

    server.on("/normalize.css", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(SPIFFS, "/normalize.css", "text/css");
    });


}

void Webserver::handleClient() {
    return;
}