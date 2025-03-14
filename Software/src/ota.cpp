//ota.cpp

#include "ota.h"
//#include "_usersettings.h"
#include "display.h"
#include "variables.h"

//#include "./Resources/wifi.h"
#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <ElegantOTA.h>
WebServer server(80);
unsigned long ota_progress_millis = 0;

HM_Display dis_ota;

HM_OTA::HM_OTA() {}

void onOTAStart() {
    #if DEBUG_HM >= 1
      Serial.println("OTA update started!");
    #endif
    #if DISPLAY_TYPE || 991 or DISPLAY_TYPE || 992 or DISPLAY_TYPE || 999
      dis_ota.on_ota_start_oled();
    #endif
    #if DISPLAY_TYPE || 993 or DISPLAY_TYPE || 999
      dis_ota.ota_on_ota_start_tft();
    #endif
  }

void onOTAProgress(size_t current, size_t final) {
  if (millis() - ota_progress_millis > 1000) {
    ota_progress_millis = millis();
    #if DISPLAY_TYPE || 991 or DISPLAY_TYPE || 992 or DISPLAY_TYPE || 999
      dis_ota.on_ota_progress_oled(current, final);
    #endif
    #if DISPLAY_TYPE || 993 or DISPLAY_TYPE || 999
      dis_ota.ota_on_ota_progress_tft(current, final);
    #endif
    #if DEBUG_HM >= 1
      Serial.printf("OTA Progress Current: %u bytes, Final: %u bytes\n", current, final);
    #endif
  }
}

void onOTAEnd(bool success) {
  if (success) {
    #if DISPLAY_TYPE || 991 or DISPLAY_TYPE || 992 or DISPLAY_TYPE || 999
      dis_ota.on_on_end1_oled(success);
    #endif
    #if DISPLAY_TYPE || 993 or DISPLAY_TYPE || 999
      dis_ota.ota_on_on_end1_tft(success);
    #endif
    ota_done = 1;
    #if DEBUG_HM >= 1
      Serial.println("OTA update finished successfully!");
    #endif
  } 
  else {
    #if DISPLAY_TYPE || 991 or DISPLAY_TYPE || 992 or DISPLAY_TYPE || 999
      dis_ota.on_on_end2_oled();
    #endif
    #if DISPLAY_TYPE || 993 or DISPLAY_TYPE || 999
      dis_ota.ota_on_on_end2_tft();
    #endif
    #if DEBUG_HM >= 1
      Serial.println("There was an error during OTA update!");
    #endif
  }
}

void OTAdisconnect(const int switch_setup_pin) {
  #if DEBUG_HM >= 1
    Serial.println("OTAdisconnect");
  #endif
  int x_pos;
  #if DISPLAY_TYPE || 991 or DISPLAY_TYPE || 992 or DISPLAY_TYPE || 999
    dis_ota.ota_disconnect_oled();
  #endif
  #if DISPLAY_TYPE || 993 or DISPLAY_TYPE || 999
    dis_ota.ota_disconnect_tft();
  #endif
  WiFi.disconnect();
  delay(500);
  while (WiFi.status() == WL_CONNECTED and digitalRead(switch_setup_pin) == HIGH) {delay(500);}
}

void OTALoop(const int switch_setup_pin, const int button_stop_pin) {
    #if DEBUG_HM >= 1
      Serial.println("OTALoop");
    #endif
    while (WiFi.status() == WL_CONNECTED and digitalRead(switch_setup_pin) == HIGH) {
      if (digitalRead(button_stop_pin) == HIGH) {
        while (digitalRead(button_stop_pin) == HIGH) {delay(100);}
      OTAdisconnect(switch_setup_pin);
    }
    else {
      server.handleClient();
      if (ota_done == 1) {
        delay(5000);
        ESP.restart();
      }
    }
  }
  if (digitalRead(switch_setup_pin) == LOW) {
    OTAdisconnect(switch_setup_pin);
  }
}

void HM_OTA::ota_setup(const int switch_setup_pin, const int button_stop_pin) {
  #if OTA == 1
      #if DEBUG_HM >= 1
        Serial.println("OTASetup");
      #endif
      #if DISPLAY_TYPE || 993 or DISPLAY_TYPE || 999
        dis_ota.ota_setup1_tft();
      #endif
      WiFi.mode(WIFI_STA);
      WiFi.setSleep(false);
      WiFi.begin(ssid, password);
      // Wait for connection
      char x[] = "";
      int i = 0;
      unsigned long wifi_start_millis = millis();
      char ip[30];
      while (WiFi.status() != WL_CONNECTED and millis() - wifi_start_millis < 15000 and digitalRead(switch_setup_pin) == HIGH) {
        #if DISPLAY_TYPE || 991 or DISPLAY_TYPE || 992 or DISPLAY_TYPE || 999
          dis_ota.ota_setup1_oled();
        #endif
        if (i < 10) {
          sprintf(x, "%s*", x);
          i++;
        }
        else {
          i = 1;
          sprintf(x, "%s", "*");
        }
        #if DISPLAY_TYPE || 991 or DISPLAY_TYPE || 992 or DISPLAY_TYPE || 999
          dis_ota.ota_setup2_oled(x);
        #endif
        #if DISPLAY_TYPE || 993 or DISPLAY_TYPE || 999
          dis_ota.ota_setup2_tft(x);
        #endif
        delay(500);
      }
      if (WiFi.status() == WL_CONNECTED and digitalRead(switch_setup_pin) == HIGH) {
        #if DISPLAY_TYPE || 991 or DISPLAY_TYPE || 992 or DISPLAY_TYPE || 999
          sprintf(ip, "%i.%i.%i.%i", WiFi.localIP()[0], WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3]);
          dis_ota.ota_setup3_oled(ip);
        #endif
        #if DISPLAY_TYPE || 993 or DISPLAY_TYPE || 999
          sprintf(ip, "%i.%i.%i.%i", WiFi.localIP()[0], WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3]);
          dis_ota.ota_setup3_tft(ip);
        #endif
        server.on("/", []() {
          char output[30];
          sprintf(output, "HaniMandl %s", VERSION_STRING);
          server.send(200, "text/plain", output);
        });
        #if DEBUG_HM >= 1
          Serial.print("IP adress: "); Serial.println(WiFi.localIP());
        #endif
        ElegantOTA.begin(&server);
        ElegantOTA.onStart(onOTAStart);
        ElegantOTA.onProgress(onOTAProgress);
        ElegantOTA.onEnd(onOTAEnd);
        server.begin();
      }
      if (WiFi.status() == WL_CONNECTED and digitalRead(switch_setup_pin) == HIGH) {
        OTALoop(switch_setup_pin, button_stop_pin);
      }
      else {
        #if DISPLAY_TYPE || 991 or DISPLAY_TYPE || 992 or DISPLAY_TYPE || 999
          dis_ota.ota_setup4_oled();
        #endif
        #if DISPLAY_TYPE || 993 or DISPLAY_TYPE || 999
          dis_ota.ota_setup4_tft();
        #endif
        #if DEBUG_HM >= 1
          Serial.print("IP adress: "); Serial.println(WiFi.localIP());
        #endif
      }
      delay(3000);
  #endif
}

void HM_OTA::ota_search_chanel() {
  #if (OTA == 1 and DREHTELLER == 1)
    WiFi.mode(WIFI_STA);
    #if DEBUG_HM >= 1
      Serial.println("Scanne nach WLAN Netzwerken...");
    #endif
    int n = WiFi.scanNetworks();
    #if DEBUG_HM >= 1
      Serial.println("Scan abgeschlossen.");
    #endif
    for (int i = 0; i < n; i++) {
      if (WiFi.SSID(i) == ssid) {
        channel = WiFi.channel(i);
        #if DEBUG_HM >= 1
          Serial.print("SSID: ");
          Serial.print(WiFi.SSID(i));
          Serial.print(" Kanal: ");
          Serial.println(channel);
        #endif
        break; // Beende die Schleife, wenn das Netzwerk gefunden wurde
      }
    }
  #endif
}