#include "WebIF.h"
#include "display.h"
#include "variables.h"
#include "HardwareLevel.h"
#include "./Resources/resources.h"

#include <Preferences.h>
#include <WiFi.h>
#include <WebServer.h>

HM_Display dis_webif;
Preferences preferences_webif;

//const char* apSSID = "HaniMandl_Setup";
//const char* apPassword = "12345678";

HM_WEBIF::HM_WEBIF() : apserver(80) {} // WebServer Port 80 initialisieren

int numNetworks = 0;  // Anzahl der gefundenen Netzwerke
bool exit_webib = false;

void HM_WEBIF::handleRoot() {
    String html = "<html><head>";
html += "<style>";
html += "body { font-family: Arial, sans-serif; display: flex; flex-direction: column; align-items: center; background: #eaeaea; }";
html += ".box { padding: 20px; border-radius: 10px; width: 250px; text-align: left; background: #f4f4f4; }";
html += "label, input, select, button { display: block; margin-bottom: 10px; width: 100%; }";
html += "input, select, button { padding: 8px; border: 1px solid #ccc; border-radius: 5px; }";
html += "form { width: 100%; }";
html += ".button-container { display: flex; flex-direction: column; align-items: center; gap: 10px; margin-top: 20px; width: 100%; }";
// Verringere das Padding, um die Buttons niedriger zu machen
html += ".btn { background: #007bff; color: white; border: none; cursor: pointer; padding: 5px; border-radius: 5px; width: 100%; text-align: center; font-size: 16px; }";
html += "</style></head><body>";

html += "<h2>" + String(HANIMANDL_WEB_IF[lingo]) + "</h2>";
html += String(SW_RELEASE[lingo]) + ": " + String(VERSION_STRING) + "<br><br>";
html += "<div class='box'>";
html += "<h3>" + String(WLAN_SETTINGS[lingo]) + "</h3>";

// WLAN-Formular
if (ssid == String(WiFi.SSID()) and WiFi.getMode() == WIFI_STA) {
    html += "<form action='/resetwifi' method='POST'>";
    html += "<label for='ssid'>" + String(SSID[lingo]) + ":</label>";
    html += "<input type='text' id='ssid' name='ssid' value='" + String(WiFi.SSID()) + "' readonly>";
} else {
    html += "<form action='/setwifi' method='POST'>";
    html += "<label for='ssid'>" + String(SSID[lingo]) + ":</label>";
    html += "<select id='ssid' name='ssid'>";
    for (int i = 0; i < numNetworks; i++) {
        html += "<option value='" + WiFi.SSID(i) + "'>" + WiFi.SSID(i) + " (" + String(WiFi.RSSI(i)) + " dBm)</option>";
    }
    html += "</select>";
    html += "<label for='password'>" + String(PASSWORD[lingo]) + ":</label>";
    html += "<input type='password' id='password' name='password' placeholder='WLAN-Passwort'>";
}

// Buttons untereinander mit gleicher Breite
html += "<div class='button-container'>";
if (ssid == String(WiFi.SSID()) and WiFi.getMode() == WIFI_STA) {
    html += "<input class='btn' type='submit' value='" + String(RESET[lingo]) + "'>";
} else {
    html += "<input class='btn' type='submit' value='" + String(CONNECT[lingo]) + "'>";
}
html += "</form>";

html += "<form action='/exit' method='POST'>";
html += "<button class='btn' type='submit'>" + String(EXIT[lingo]) + "</button>";
html += "</form>";
html += "</div>"; // Ende button-container

html += "</div>"; // Ende der box
html += "</body></html>";

apserver.send(200, "text/html; charset=UTF-8", html);
}

void HM_WEBIF::setupAPMode() {
    // STA-Modus aktivieren, um Netzwerke zu scannen
    WiFi.mode(WIFI_AP);
    #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
        dis_webif.setup_webif_oled(2);
    #endif
    #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
        dis_webif.setup_webif_tft(2);
    #endif
    #if WIFI_DEBUG >= 1
        Serial.println("Wechsel in den STA-Modus und scanne nach WLAN-Netzwerken...");
    #endif
    WiFi.mode(WIFI_STA);
    delay(500); // Warten auf den Moduswechsel
    numNetworks = WiFi.scanNetworks(); // Netzwerke scannen
    #if WIFI_DEBUG >= 1
        Serial.println("Scan abgeschlossen");
        if (numNetworks == 0) {
            Serial.println("Keine Netzwerke gefunden.");
        } else {
            Serial.println("Gefundene Netzwerke:");
            for (int i = 0; i < numNetworks; i++) {
                Serial.println("SSID: " + WiFi.SSID(i) + " mit RSSI: " + String(WiFi.RSSI(i)) + " dBm");
            }
        }
        // Wechsel in den AP-Modus und Access Point starten
        Serial.println("Wechsel in den AP-Modus und starte Access Point...");
    #endif
    WiFi.mode(WIFI_AP);                                                             // Wechsel in den AP-Modus
    WiFi.softAP(apSSID, apPassword);
    #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
        dis_webif.setup_webif_oled(6);
    #endif
    #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
        dis_webif.setup_webif_tft(6);
    #endif                                                // Access Point starten
    #if WIFI_DEBUG >= 1
        Serial.println("AP gestartet! IP-Adresse: " + WiFi.softAPIP().toString());
    #endif
}

void HM_WEBIF::handleSetWiFi() {
    if (apserver.hasArg("ssid") && apserver.hasArg("password")) {
        String newSSID = apserver.arg("ssid");
        String newPassword = apserver.arg("password");
        apserver.send(200, "text/html; charset=UTF-8", "<h3>Neues WLAN gespeichert! Neustart...</h3>");
        delay(2000);
        WiFi.softAPdisconnect(true);
        WiFi.mode(WIFI_STA);
        WiFi.begin(newSSID.c_str(), newPassword.c_str());
        int timeout = 10;
        #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
            dis_webif.setup_webif_oled(2);
        #endif
        #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
            dis_webif.setup_webif_tft(2);
        #endif
        while (WiFi.status() != WL_CONNECTED && timeout-- > 0) {
            #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
                dis_webif.setup_webif_oled(3);
            #endif
            #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
                dis_webif.setup_webif_tft(3);
            #endif
            delay(1000);
            Serial.print(".");
        }
        if (WiFi.status() == WL_CONNECTED) {
            ssid = newSSID;
            password = newPassword;
            preferences_webif.begin("EEPROM", false);
            preferences_webif.putString("ssid", ssid);
            preferences_webif.putString("password", password);
            preferences_webif.end();
            #if WIFI_DEBUG >= 1
                Serial.println("\nVerbunden mit: " + String(WiFi.SSID()));
                Serial.println("Neue IP-Adresse: " + WiFi.localIP().toString());
            #endif
            #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
                dis_webif.setup_webif_oled(4);
            #endif
            #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
                dis_webif.setup_webif_tft(4);
            #endif
        } 
        else {
            #if WIFI_DEBUG >= 1
                Serial.println("\nVerbindung fehlgeschlagen! Neustart im AP-Modus...");
            #endif
            WiFi.disconnect(true);
            setupAPMode();
        }
    } 
    else {
        apserver.send(400, "text/html; charset=UTF-8", "<h3>Fehler: SSID oder Passwort fehlt!</h3>");
    }
}

void HM_WEBIF::handleReSetWiFi() {
    preferences_webif.begin("EEPROM", false);
    preferences_webif.putString("ssid", "");
    preferences_webif.putString("password", "");
    preferences_webif.end();
    apserver.send(200, "text/html; charset=UTF-8", "<h3>WiFi zurückgesetzt. Neustart...</h3>");
    delay(1000);
    ESP.restart();
}

void HM_WEBIF::handleExit() {
    apserver.send(400, "text/html; charset=UTF-8", "<h3>Stop HaniMandl Webinterface</h3>");
    exit_webib = true;
}

void HM_WEBIF::setupWebIF() {
    exit_webib = false;
    WiFi.mode(WIFI_STA);
    // Prüfe, ob eine SSID definiert ist
    #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
        dis_webif.setup_webif_oled(1);
    #endif
    #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
        dis_webif.setup_webif_tft(1);
    #endif
    if (ssid != "") {
        #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
            dis_webif.setup_webif_oled(2);
        #endif
        #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
            dis_webif.setup_webif_tft(2);
        #endif
        #if WIFI_DEBUG >= 1
            Serial.println("Versuche, mit gespeicherter SSID zu verbinden...");
        #endif
        WiFi.begin(ssid.c_str(), password.c_str());
        int timeout = 10;
        while (WiFi.status() != WL_CONNECTED && timeout-- > 0) {
            #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
                dis_webif.setup_webif_oled(3);
            #endif
            #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
                dis_webif.setup_webif_tft(3);
            #endif
            delay(1000);
            #if WIFI_DEBUG >= 1
                Serial.print(".");
            #endif
        }
        if (WiFi.status() == WL_CONNECTED) {
            #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
                dis_webif.setup_webif_oled(4);
            #endif
            #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
                dis_webif.setup_webif_tft(4);
            #endif
            #if WIFI_DEBUG >= 1
                Serial.println("\nVerbunden mit: " + String(WiFi.SSID()));
                Serial.println("Neue IP-Adresse: " + WiFi.localIP().toString());
            #endif
        } 
        else {
            WiFi.disconnect(true);
            #if WIFI_DEBUG >= 1
                Serial.println("\nVerbindung fehlgeschlagen! Wechsel in den AP-Modus...");
            #endif
            setupAPMode();
        }
    } 
    else {
        #if WIFI_DEBUG >= 1
            Serial.println("Keine SSID gespeichert, Wechsel in den AP-Modus...");
        #endif
        setupAPMode();
    }

    // Webserver starten
    apserver.on("/", HTTP_GET, [this]() { this->handleRoot(); });
    apserver.on("/setwifi", HTTP_POST, [this]() { this->handleSetWiFi(); });
    apserver.on("/resetwifi", HTTP_POST, [this]() { this->handleReSetWiFi(); });
    apserver.on("/exit", HTTP_POST, [this]() { this->handleExit(); });
    apserver.begin();
    #if WIFI_DEBUG >= 1
        Serial.println("Webserver gestartet!");
    #endif
    while (exit_webib == false and digitalRead(switch_setup_pin) == HIGH) {
        if (digitalRead(button_stop_pin) == HIGH or digitalRead(switch_setup_pin) == LOW) {
            while (digitalRead(button_stop_pin) == HIGH) {
              delay(1);
            }
            exit_webib = true;
        }
        if (exit_webib == false) {
            apserver.handleClient();
        }
    }
    #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
        dis_webif.setup_webif_oled(5);
    #endif
    #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
        dis_webif.setup_webif_tft(5);
    #endif
    #if WIFI_DEBUG >= 1
        Serial.println("Stopping Access Point...");
    #endif
    WiFi.softAPdisconnect(true);
    delay(500);
    #if WIFI_DEBUG >= 1
        Serial.println("Disconnecting clients...");
    #endif
    WiFi.disconnect(true);
    delay(500);
    #if WIFI_DEBUG >= 1
        Serial.println("Setting WiFi mode to NULL...");
    #endif
    WiFi.mode(WIFI_MODE_NULL);  // Erst auf NULL setzen
    delay(500);
    #if WIFI_DEBUG >= 1
        Serial.println("Turning WiFi OFF...");
    #endif
    WiFi.mode(WIFI_OFF);  // Dann WiFi ganz ausschalten
    delay(500);
    // Warten, bis alle Verbindungen wirklich getrennt sind
    int timeout = 10000;  // 10 Sekunden Timeout
    unsigned long start = millis();
    while ((WiFi.status() == WL_CONNECTED || WiFi.softAPgetStationNum() > 0) && millis() - start < timeout) {
        #if WIFI_DEBUG >= 1
            Serial.println("Waiting for WiFi to disconnect...");
        #endif
        delay(500);
    }
    #if WIFI_DEBUG >= 1
        Serial.println("WiFi completely shut down.");
    #endif
}