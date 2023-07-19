#include "Webserver.h"
#include "Config.h"

AsyncWebServer server(80);
extern FC_cfg cfg;

void Webserver::init() {
    WiFiClass::mode(WIFI_MODE_APSTA);
    WiFi.begin(ssid, password);

    // Print the IP address when connected
    while (WiFiClass::status() != WL_CONNECTED) {
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

    // We serve SPIFFS files from the root path ("/"), basic web app
    server.serveStatic("/", SPIFFS, "/").setDefaultFile("index.html");

    // API endpoints
    server.on("/api", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(200, "text/plain", "Hello, world");
    });

    server.on("/pid", HTTP_GET, [](AsyncWebServerRequest *request) {
        // Convert cfg to JSON
        String buffer = "{";
        buffer += "\"pitchKp\": " + String(cfg.Kp_pitch) + ",";
        buffer += "\"pitchKi\": " + String(cfg.Ki_pitch) + ",";
        buffer += "\"pitchKd\": " + String(cfg.Kd_pitch) + ",";
        buffer += "\"rollKp\": " + String(cfg.Kp_roll) + ",";
        buffer += "\"rollKi\": " + String(cfg.Ki_roll) + ",";
        buffer += "\"rollKd\": " + String(cfg.Kd_roll) + ",";
        buffer += "\"yawKp\": " + String(cfg.Kp_yaw) + ",";
        buffer += "\"yawKi\": " + String(cfg.Ki_yaw) + ",";
        buffer += "\"yawKd\": " + String(cfg.Kd_yaw) + ",";
        buffer += "\"maxAngle\": " + String(cfg.max_angle);
        buffer += "}";
        request->send(200, "application/json", buffer);
    });


}

void Webserver::handleClient() {
}