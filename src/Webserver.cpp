#include "Webserver.h"
#include "Config.h"
#include "ArduinoJson.h"
#include "Controller.h"

extern FC FliCon;
extern ConfigSuite CfgMan;
AsyncWebServer server(80);

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

    DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");
    server.begin();

    // We serve SPIFFS files from the root path ("/"), basic web app
    server.serveStatic("/", SPIFFS, "/").setDefaultFile("index.html");

    // API endpoints
    server.on("/api", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(200, "text/plain", "Hello, world");
    });

    server.on("/pid", HTTP_GET, [](AsyncWebServerRequest *request) {
        auto const cfg = CfgMan.getActiveCfg();
        // Convert cfg to JSON
        StaticJsonDocument<200> doc;
        doc["pitchKp"] = cfg->Kp_pitch;
        doc["pitchKi"] = cfg->Ki_pitch;
        doc["pitchKd"] = cfg->Kd_pitch;
        doc["rollKp"] = cfg->Kp_roll;
        doc["rollKi"] = cfg->Ki_roll;
        doc["rollKd"] = cfg->Kd_roll;
        doc["yawKp"] = cfg->Kp_yaw;
        doc["yawKi"] = cfg->Ki_yaw;
        doc["yawKd"] = cfg->Kd_yaw;
        doc["maxAngle"] = cfg->max_angle;

        String json;
        serializeJson(doc, json);

        request->send(200, "application/json", json);
    });

    server.on("/pid", HTTP_POST, [](AsyncWebServerRequest *request){
        // Convert JSON to cfg
        StaticJsonDocument<200> doc;
        deserializeJson(doc, request->arg("plain"));
        
        auto cfg = CfgMan.getActiveCfg();
        cfg->Kp_pitch = doc["pitchKp"];
        cfg->Ki_pitch = doc["pitchKi"];
        cfg->Kd_pitch = doc["pitchKd"];
        cfg->Kp_roll = doc["rollKp"];
        cfg->Ki_roll = doc["rollKi"];
        cfg->Kd_roll = doc["rollKd"];
        cfg->Kp_yaw = doc["yawKp"];
        cfg->Ki_yaw = doc["yawKi"];
        cfg->Kd_yaw = doc["yawKd"];
        cfg->max_angle = doc["maxAngle"];

        CfgMan.setCfg(cfg);

        auto const newcfg = CfgMan.getActiveCfg();
        StaticJsonDocument<32> response;

        response["pitchKp"] = newcfg->Kp_pitch;
        response["pitchKi"] = newcfg->Ki_pitch;
        response["pitchKd"] = newcfg->Kd_pitch;
        response["rollKp"] = newcfg->Kp_roll;
        response["rollKi"] = newcfg->Ki_roll;
        response["rollKd"] = newcfg->Kd_roll;
        response["yawKp"] = newcfg->Kp_yaw;
        response["yawKi"] = newcfg->Ki_yaw;
        response["yawKd"] = newcfg->Kd_yaw;
        response["maxAngle"] = newcfg->max_angle;
    
        String json;
        serializeJson(response, json);

        request->send(200, "application/json", json);
    });

    server.on("/temperature", HTTP_GET, [](AsyncWebServerRequest *request){
        // Convert cfg to JSON
        StaticJsonDocument<32> doc;
        doc["temperature"] = FliCon.temperature;

        String json;
        serializeJson(doc, json);

        request->send(200, "application/json", json);
    });

    server.on("/restart", HTTP_POST, [](AsyncWebServerRequest *request){
        request->send(200, "text/plain", "OK");
        ESP.restart();
    });

    server.on("/armState", HTTP_GET, [](AsyncWebServerRequest *request){
        // Convert cfg to JSON
        StaticJsonDocument<32> doc;
        doc["armed"] = FliCon.armed;

        String json;
        serializeJson(doc, json);

        request->send(200, "application/json", json);
    });
}

void Webserver::handleClient() {
    
}