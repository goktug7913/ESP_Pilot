#include "Webserver.h"
#include "Controller.h"

extern FC controller;

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
        return;
    }

    server.begin();
    /*
        Architecture planning:
        We can either serve a frontend from the ESP, or we can serve a frontend from a separate server.
        Ideally we will use a React App as the frontend, and serve it from a separate server.
        But that adds some caveats:
        - We need to make sure the client is connected to the same network as the ESP
        - Frontend needs to know the IP address of the ESP so it can send requests to it
        - Where will the frontend be hosted? options:
            - Hosted on Vercel, frontend will explore the client's network for the ESP
            - Hosted on the ESP, frontend will be served from the ESP
            - Distribute the frontend as a standalone app, frontend will explore the client's network for the ESP
    */

    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
            request->send(SPIFFS, "/index.html", "text/html");
        });

    server.on("/index.css", HTTP_GET, [](AsyncWebServerRequest *request){
            request->send(SPIFFS, "/index.css", "text/css");
        });

    server.on("/normalize.css", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(SPIFFS, "/normalize.css", "text/css");
    });

    // API endpoints
    // - - - - - - - - - - - - - - - -
    // Reboot ESP
    server.on("/api/espreboot", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(200, "text/plain", "Rebooting ESP");
        ESP.restart();
    });

    // Set PID values
    server.on("/api/pid", HTTP_POST, [](AsyncWebServerRequest *request){
        if (request->hasParam("p") && request->hasParam("i") && request->hasParam("d")) {
            controller.setGains(
                request->getParam("p")->value().toFloat(),
                request->getParam("i")->value().toFloat(),
                request->getParam("d")->value().toFloat()
            );
            printf("New PID values: %f, %f, %f\n", controller.pitch_pid.Kp, controller.pitch_pid.Ki, controller.pitch_pid.Kd);
            request->send(200, "text/plain", "OK");
        } else {
            request->send(400, "text/plain", "Bad Request");
        }
    });

    // Get current PID values
    server.on("/api/pid", HTTP_GET, [](AsyncWebServerRequest *request){
        String response = "{\"p\":";
        response += controller.pitch_pid.Kp;
        response += ",\"i\":";
        response += controller.pitch_pid.Ki;
        response += ",\"d\":";
        response += controller.pitch_pid.Kd;
        response += "}";
        request->send(200, "application/json", response);
    });
}

void Webserver::handleClient() {
    return;
}