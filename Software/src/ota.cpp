//ota.cpp

#include "ota.h"
#include "display.h"
#include "variables.h"
#include "variables_display.h"
#include "HardwareLevel.h"

#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <ElegantOTA.h>
#include <Update.h>
#include <HTTPClient.h>
WebServer server(80);
unsigned long ota_progress_millis = 0;

const char* firmware_url = "https://github.com/RolandRust/HM_Online_Update/releases/latest/download";
const int maxFiles = 30;
String binFiles[maxFiles];
int binFileCount = 0;
String FileName = "";
bool run = true;
int progress_old = -1;

HM_Display dis_ota;

//Offline OTA update
HM_OTA::HM_OTA() {}

void onOTAStart() {
    #if OTA_DEBUG >= 1
      Serial.println("OTA update started!");
    #endif
    dis_ota.ota_setup(5);       //OTA start
  }

void onOTAProgress(size_t current, size_t final) {
  ota_current = current;
  ota_final = final;
  if (millis() - ota_progress_millis > 1000) {
    ota_progress_millis = millis();
    dis_ota.ota_setup(6);       //OTA progress
    #if OTA_DEBUG >= 1
      Serial.printf("OTA Progress Current: %u bytes, Final: %u bytes\n", current, final);
    #endif
  }
}

void onOTAEnd(bool success) {
  if (success) {
    dis_ota.ota_setup(7);       //OTA successfully
    ota_done = 1;
    #if OTA_DEBUG >= 1
      Serial.println("OTA update finished successfully!");
    #endif
  } 
  else {
    dis_ota.ota_setup(8);       //OTA error
    #if OTA_DEBUG >= 1
      Serial.println("There was an error during OTA update!");
    #endif
  }
}

void OTAdisconnect(const int switch_setup_pin) {
  #if OTA_DEBUG >= 1
    Serial.println("OTAdisconnect");
  #endif
  dis_ota.ota_setup(9);       //OTA disconnect
  WiFi.disconnect();
  delay(500);
  while (WiFi.status() == WL_CONNECTED and digitalRead(switch_setup_pin) == HIGH) {delay(500);}
}

void OTALoop(const int switch_setup_pin, const int button_stop_pin) {
    #if OTA_DEBUG >= 1
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
      #if OTA_DEBUG >= 1
        Serial.println("OTASetup");
      #endif
      dis_ota.ota_setup(1);       //Clear Display
      WiFi.mode(WIFI_STA);
      WiFi.setSleep(false);
      WiFi.begin(ssid, password);
      // Wait for connection
      int i = 0;
      unsigned long wifi_start_millis = millis();
      strcpy(stars, "");
      strcpy(ip, "");
      while (WiFi.status() != WL_CONNECTED and millis() - wifi_start_millis < 15000 and digitalRead(switch_setup_pin) == HIGH) {
        if (i < 10) {
          sprintf(stars, "%s*", stars);
          i++;
        }
        else {
          i = 1;
          sprintf(stars, "%s", "*");
        }
        dis_ota.ota_setup(2);       //Start WiFi
        delay(500);
      }
      if (WiFi.status() == WL_CONNECTED and digitalRead(switch_setup_pin) == HIGH) {
        sprintf(ip, "%i.%i.%i.%i", WiFi.localIP()[0], WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3]);
        dis_ota.ota_setup(3);       //Show IP adress
        server.on("/", []() {
          char output[30];
          sprintf(output, "HaniMandl %s", VERSION_STRING);
          server.send(200, "text/plain", output);
        });
        #if OTA_DEBUG >= 1
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
        dis_ota.ota_setup(4);       //Failed
        #if OTA_DEBUG >= 1
          Serial.print("IP adress: "); Serial.println(WiFi.localIP());
        #endif
      }
      delay(5000);
  #endif
}

void HM_OTA::ota_search_chanel() {
  #if (OTA == 1 and DREHTELLER == 1)
    WiFi.mode(WIFI_STA);
    #if OTA_DEBUG >= 1
      Serial.println("Scanne nach WLAN Netzwerken...");
    #endif
    int n = WiFi.scanNetworks();
    #if OTA_DEBUG >= 1
      Serial.println("Scan abgeschlossen.");
    #endif
    for (int i = 0; i < n; i++) {
      if (WiFi.SSID(i) == ssid) {
        channel = WiFi.channel(i);
        #if OTA_DEBUG >= 1
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

void startFirmwareUpdate() {
  #if OTA_DEBUG >= 1
    Serial.println("Starte OTA-Update...");
  #endif
  HTTPClient http;
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  String fullURL = String(firmware_url) + "/" + FileName;
  #if OTA_DEBUG >= 1
    Serial.println(fullURL);
  #endif
  http.begin(fullURL);
  int httpCode = http.GET();
  if (httpCode == HTTP_CODE_OK) {
    int contentLength = http.getSize();
    if (contentLength <= 0) {
      #if OTA_DEBUG >= 1
          Serial.println("Fehler: Ungültige Datei");
      #endif
      dis_ota.onlineOTA(10);
      run = false;
      delay(5000);
      return;
    }
    if (!Update.begin(contentLength)) {
      #if OTA_DEBUG >= 1
        Serial.println("Fehler beim Starten des Updates");
      #endif
      dis_ota.onlineOTA(11);
      run = false;
      delay(5000);
      return;
    }
    WiFiClient *stream = http.getStreamPtr();
    size_t written = 0;
    uint8_t buffer[1024];  // Puffer für den Download
    #if OTA_DEBUG >= 1
      Serial.println("Lade Firmware herunter...");
    #endif
    while (http.connected() && written < contentLength) {
      size_t available = stream->available();
      if (available) {
        size_t readBytes = stream->readBytes(buffer, min(available, sizeof(buffer)));
        size_t bytesWritten = Update.write(buffer, readBytes);
        written += bytesWritten;
        // Fortschritt in Prozent berechnen und anzeigen
        progress = (written * 100) / contentLength;
        if (progress != progress_old) {
          progress_old = progress;
          dis_ota.onlineOTA(6);
          #if OTA_DEBUG >= 1
            Serial.printf("\rUpdate-Fortschritt: %d%%", progress);
          #endif
        }
      }
      delay(1);  // Verhindert Blockieren der CPU
    }
    #if OTA_DEBUG >= 1
      Serial.println();  // Neue Zeile nach Fortschritt
    #endif
    if (written == contentLength) {
      #if OTA_DEBUG >= 1
        Serial.println("Update erfolgreich!");
      #endif
    } 
    else {
      #if OTA_DEBUG >= 1
        Serial.println("Fehler beim Schreiben der Firmware");
      #endif
      dis_ota.onlineOTA(12);
      run = false;
      delay(5000);
    }
    if (Update.end()) {
      dis_ota.onlineOTA(7);
      #if OTA_DEBUG >= 1
        Serial.println("Neustart...");
      #endif
      delay(5000);
      ESP.restart();
    } 
    else {
      dis_ota.onlineOTA(8);
      #if OTA_DEBUG >= 1
        Serial.println("Update fehlgeschlagen");
      #endif
      delay(5000);
      run = false;
    }
  }
  else {
    dis_ota.onlineOTA(httpCode);
    #if OTA_DEBUG >= 1
      Serial.printf("Fehler beim Abrufen der Firmware: %d\n", httpCode);
    #endif
    run = false;
    delay(5000);
  }
  http.end();
}

void checkFirmwareVersion() {
  String FW_HM = "HM_" + String(VERSION_STRING) + "_" + String(HARDWARE_LEVEL) + "_" + String(DISPLAY_TYPE) + "_" + String(DREHTELLER) + ".bin";
  if (FileName != FW_HM) {
    sscanf(FileName.c_str(), "HM_%[^_]_", new_version);
    #if OTA_DEBUG >= 1
      Serial.println("Neue Firmware gefunden. Starte Update...");
    #endif
    dis_ota.onlineOTA(13);
    bool looping = true;
    bool start_uptate = false;
    while (digitalRead(switch_setup_pin) == HIGH && looping == true) {
      if (digitalRead(button_start_pin) == HIGH) {
        while (digitalRead(button_start_pin) == HIGH) {delay(1);}
        dis_ota.onlineOTA(4);
        looping = false;
        start_uptate = true;
      }
      if (digitalRead(button_stop_pin) == HIGH) {
        while (digitalRead(button_stop_pin) == HIGH) {delay(1);}
        looping = false;
        run = false;
      }
    }
    if (start_uptate == true) {
      dis_ota.onlineOTA(6);
      startFirmwareUpdate();
    }
  } 
  else {
    dis_ota.onlineOTA(5);
    #if OTA_DEBUG >= 1
      Serial.println("Firmware ist aktuell. Kein Update erforderlich.");
    #endif
    run = false;
    delay(5000);
  }
}

String extractRelevantFileName(String fileName) {
  // Entferne die .bin Erweiterung, falls sie vorhanden ist
  int binPos = fileName.indexOf(".bin");
  if (binPos != -1) {
    fileName = fileName.substring(0, binPos);  // Entferne ".bin"
  }
  // Suche nach dem ersten und zweiten "_"
  int firstUnderscore = fileName.indexOf('_');
  int secondUnderscore = fileName.indexOf('_', firstUnderscore + 1);
  if (firstUnderscore != -1 && secondUnderscore != -1) {
    // Ersetze den Teil zwischen den ersten beiden "_" durch "_"
    fileName = fileName.substring(0, firstUnderscore + 1) + fileName.substring(secondUnderscore + 1);
  }
  return fileName;
}

void downloadFileList() {
  dis_ota.onlineOTA(4);
  HTTPClient http;
  String fullURL = String(firmware_url) + "/FileList.txt";  // Die vollständige URL erstellen
  http.begin(fullURL);
  http.addHeader("User-Agent", "ESP32");
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);   // Setze FollowRedirects auf true, um Redirects zu folgen
  int httpCode = http.GET();
  if (httpCode == HTTP_CODE_OK) {
    String payload = http.getString();
    #if OTA_DEBUG >= 1
      Serial.println("FileList.txt Inhalt:");
      Serial.println(payload);
    #endif
    // Extrahiere die .bin-Dateinamen und speichere sie im Array
    int fileStartPos = 0;
    while ((fileStartPos = payload.indexOf(".bin", fileStartPos)) != -1) {
      // Extrahiere den Dateinamen
      int nameStart = payload.lastIndexOf("\n", fileStartPos);  // Gehe zum vorherigen Zeilenumbruch
      String fileName = payload.substring(nameStart + 1, fileStartPos + 4);  // Dateiname (mit .bin)
      // Füge den Dateinamen zum Array hinzu
      if (fileName.endsWith(".bin") && binFileCount < maxFiles) {
        binFiles[binFileCount++] = fileName;
      }
      fileStartPos += 4;  // Nach der .bin-Erweiterung weitermachen
      if (binFileCount >= maxFiles) {
        break;  // Beende, wenn die maximale Anzahl erreicht ist
      }
    }
    // Generiere den Dateinamen basierend auf den build-Flags, aber ohne den VERSION_STRING
    String FileNameWithoutVersion = "HM_" + String(HARDWARE_LEVEL) + "_" + String(DISPLAY_TYPE) + "_" + String(DREHTELLER);
    // Vergleiche den generierten Dateinamen ohne VERSION_STRING mit den Dateien im Array
    bool found = false;
    for (int i = 0; i < binFileCount; i++) {
      #if OTA_DEBUG >= 1  
        Serial.print("Vergleiche mit: ");
        Serial.println(binFiles[i]);
      #endif
      // Wir extrahieren den relevanten Teil des Dateinamens
      String fileNameToCompare = extractRelevantFileName(binFiles[i]);
      // Wenn der Dateiname im Array enthalten ist (ignoriere den VERSION_STRING-Teil)
      if (fileNameToCompare == FileNameWithoutVersion) {
        #if OTA_DEBUG >= 1  
          Serial.println("Gefunden! Der Dateiname stimmt überein.");
        #endif
        FileName = binFiles[i];  // Speichere den gefundenen Dateinamen
        found = true;
        break;  // Beende den Vergleich, wenn die Datei gefunden wurde
      }
    }
    if (!found) {
      dis_ota.onlineOTA(9);
      #if OTA_DEBUG >= 1
        Serial.println("Kein passender Dateiname gefunden.");
      #endif
      run = false;
      delay(5000);
    } 
    else {
      #if OTA_DEBUG >= 1
        Serial.print("Gefundene Datei: ");
        Serial.println(FileName);
      #endif
      checkFirmwareVersion();
    }

  } else {
    dis_ota.onlineOTA(httpCode);
    #if OTA_DEBUG >= 1
      Serial.printf("Fehler beim Abrufen der Datei: %d\n", httpCode);
      #endif
    run = false;
    delay(5000);
  }
  http.end();
}

void HM_OTA::online_ota_setup() {
  run = true;
  //IPAddress dns(8, 8, 8, 8);  // Google DNS
  //WiFi.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, dns); // Setze den DNS-Server
  dis_ota.onlineOTA(1);
  WiFi.begin(ssid, password);
  int timeout = 10000;  // 10 Sekunden Timeout
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < timeout && digitalRead(switch_setup_pin) == HIGH) {
    dis_ota.onlineOTA(2);
    #if OTA_DEBUG >= 1
      Serial.print(".");
    #endif
    delay(500);
  }
  if (WiFi.status() == WL_CONNECTED) {
    #if OTA_DEBUG >= 1
      Serial.println("\nVerbunden mit WiFi");
    #endif
    downloadFileList();
  }
  else {
    run = false;
    dis_ota.onlineOTA(3);
    #if OTA_DEBUG >= 1
      Serial.println("\nVerbingung mit WiFi fehlgeschlagen");
    #endif
    delay(5000);
  }
  while (digitalRead(switch_setup_pin) == HIGH && run == true) {
    if (digitalRead(button_stop_pin) == HIGH) {
      while (digitalRead(button_stop_pin) == HIGH) {
        delay(1);
      }
      run = false;
    }
  }
  dis_ota.setup_webif(5);
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
  timeout = 10000;  // 10 Sekunden Timeout
  start = millis();
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

