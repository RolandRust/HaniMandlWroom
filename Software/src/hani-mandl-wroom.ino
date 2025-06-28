/*
  HaniMandl Version W.0.4
  ------------------------
  Copyright (C) 2018-2023 by Marc Vasterling, Marc Wetzel, Clemens Gruber, Marc Junker, Andreas Holzhammer, Johannes Kuder, Jeremias Bruker
            
  2018-05 Marc Vasterling    | initiale Version, 
                               veröffentlicht in der Facebook-Gruppe "Imkerei und Technik. Eigenbau",
                               Marc Vasterling: "meinen Code kann jeder frei verwenden, ändern und hochladen wo er will, solange er nicht seinen eigenen Namen drüber setzt."
  2018-06 Marc Vasterling    | verbesserte Version, 
                               ebenfalls veröffentlicht in der Facebook-Gruppe
  2019-01 Marc Wetzel        | Refakturierung und Dokumentation, 
                               ebenfalls veröffentlicht in der Facebook-Gruppe
  2019-02 Clemens Gruber     | code beautifying mit kleineren Umbenennungen bei Funktionen und Variablen
                               Anpassung für Heltec WiFi Kit 32 (ESP32 onboard OLED) 
                               - pins bei OLED-Initialisierung geändert
                               - pins geändert, um Konflikte mit hard wired pins des OLEDs zu vermeiden 
  2019-02 Clemens Gruber     | Aktivierung der internen pull downs für alle digitalen Eingaenge
  2019-02 Clemens Gruber     | "normale" pins zu Vcc / GND geaendert um die Verkabelung etwas einfacher und angenehmer zu machen
  2020-05 Andreas Holzhammer | Anpassungen an das veränderte ;-( pin-Layout der Version 2 des Heltec 
                               wird verkauft als "New Wifi Kit 32" oder "Wifi Kit 32 V2"
  2020-05 Marc Junker        | - Erweiterung von Poti auf Rotary Encoder 
                               - alle Serial.prints in #ifdef eingeschlossen
                               - "Glas" nun als Array mit individuellem Tara
                               - Korrekturwert und Auswahl der Füllmenge über Drücken & Drehen des Rotary einstellbar
  2020-05 Andreas Holzhammer | - Tara pro abzufüllendem Glas automatisch anpassen (Variable tara_glas)
                               - Code läuft auch ohne Waage
  2020-06 Andreas Holzhammer | - Code wahlweise mit Heltec V1 oder V2 nutzbar
                               - Code wahlweise mit Poti oder Rotary nutzbar
                               - Tara pro Glas einstellbar
                               - Öffnungswinkel für Maximale Öffnung und Feindosierung im Setup konfigurierbar
                               - Korrektur und Glasgröße im Automatikmodus per Rotary Encoder Button wählbar
                               - Preferences löschbar über Setup
                               - Gewicht blinkt bei Vollautomatik, wenn nicht vollständig gefüllt
                               - Nicht kalibrierte Waage anzeigen, fehlende Waage anzeigen
                               - Tara wird nur bei >20g gesetzt, verhindert den Autostart bei leerer Waage
                               - Tarieren der Waage bei jedem Start bis +-20g. Sonst Warnung
  2020-07 Andreas Holzhammer | Version 0.2.4
                               - SCALE_READS auf 2 setzen? ca. 100ms schneller als 3, schwankt aber um +-1g
                               - Reihenfolge der Boot-Meldungen optimiert, damit nur relevante Warnungen ausgegeben werden
                               - Autokorrektur implementiert
                               - LOGO! und Umlaute (Anregung von Johannes Kuder)
                               - Stop-Taste verlässt Setup-Untermenüs (Anregung von Johannes Kuder)
                               - Preferences nur bei Änderung speichern
  2020-07 Andreas Holzhammer | Version 0.2.5
                               - Anzeige der vorherigen Werte im Setup
                               - Kulanzwert für Autokorrektur einstellbar
                               - Setup aufgeräumt, minimaler Servowinkel einstellbar
  2020-07 Andreas Holzhammer | Version 0.2.6
                               - Kalibrierung der Waage verbessert; Messewerte runden; Waage "aufheizen" vor Bootscreen
                               - Aktiver Piezo-Buzzer (Idee von Johannes Kuder)
  2020-07 Johannes Kuder     | 0.2.7
                               - Zählwerk für abgefüllte Gläser und Gewicht (nur im Automatikbetrieb)
  2020-07 Jeremias Bruker    | 0.2.8
                               - "GlasTyp" in allen Menüs und Automatikmodus integriert
                               - 5 Gläser können vom User im Menüpunkt "Fuellmenge" in Gewicht und GlasTyp konfiguriert werden 
                                 und werden nichtflüchtig gespeichert. So kann sich jeder User seine eigenen üblichen 5 Gläser anlegen
                               - Stabilisierung des Waagenwerts nach Wunsch (define FEHLERKORREKTUR_WAAGE)
                               - das Kalibriergewicht kann beim Kalibrierungsvorgang vom User verändert 
                                 werden (nicht jeder hat 500g als Eichgewicht) und wird nichtflüchtig gespeichert
                               - rotierendes Hauptmenü
                               - Umkehrbarer Servo für linksseitige Quetschhähne :-)
  2020-10 Andreas Holzhammer | 0.2.8.1
                               - Bugfix: Servo konnte im Manuellen Modus unter Minimum bewegt werden
                               - Glastoleranz über Variable steuerbar auf +-20g angepasst 
  2020-12 Andreas Holzhammer | 0.2.9
                               - Fortschrittsanzeige eingebaut
                               - angepasst an ESP32Servo aus dem Bibliotheksverwalter
  2021-02 Andreas Holzhammer | 0.2.10
                               - Korrektur zwischen -90 und +20 anpassbar
                               - Autokorrektur auch ohne Autostart 
                               - Preferences Flash-schonender implementiert
  2021-07 Andreas Holzhammer | 0.2.11
                               - Credits-Seite
                               - Fix für Rotary mit Schrittweite > 1
  2021-11 Andreas Holzhammer | 0.2.12
                               - Glastoleranz einstellbar
                               - Komfortverstellung für Füllmengen (1g/5g/25g Schritte)
  2023-01 Clemens Gruber     | 0.2.13
                               - pin-Anpassungen für Hardware-Version V3 des Heltec "WiFi Kit 32 V3" mit wieder mal geändertem pin-Layout
                               - Anpassungen für den ESP32 Arduino core Version ≥ 2.x
                               - Display, U8g2: HW statt SW im constructor (ggf. Probleme mit älteren Heltec-Versionen)
                               - Rotary: de-bouncing code im isr2 auskommentiert, da sie zu Abstürzen führte
  2023-04 Roland Rust        | W.0.1
                               - Code angepasst für das ESP32 Wroom DevKit und 2.4Zoll Display (128x64)
                               - Pinbelegung geändert
                               - Alles was mit dem Heltec Module zu tun hatte rausgeschmissen (Hardware Level entfernt)
                               - Code für das Poti wurde entfernt. Nur noch Rotray Encoder
                               - Code für Simulation an der Waage entfernt
                               - Listen Setup entfernt. Gibt nur noch das Scroll Setup
                               - All das geblinke eliminiert.
                               - Erkennung der Waage geändert. (Habe den HX711 nicht immer angeschlossen da Waage und MC-Einheit getrennt sind. Hatte nur geschaut om ein HX711 angeschlossen ist)
                               - Auswahl OLED über I2C oder SPI hinzugefügt
                               - INA219 Menü hinzugefügt (wird Angezeigt sobald ein INA 219 instaliert ist)
                               - Messung vom Servostrom hinzugefügt (Funktion ist noch experimental)
                               - Setup Korrektur ins Setup Automatik verschoben. Macht für mich mehr Sinn und mit dem grossen Display hat es Platz
                               - TFT display implementiert (320x240)
  2023-12 Achim Pfaff        | W.0.2
                               - Automatisches abfüllen mit Volumenstrom um das zu schnelle abfüllen zu verhindern.
                               - Automatik in Menü eingebunden
  2024-01 Roland Rust          - Rotary Drehrichtung in gewissen Menues geändert
                               - Rotary HW-040 Bug gefixt (Das Menü sollte nun richtig funktionieren)
                               - Volumenstrom im OLED Display implementeiert
                               - Englische Sprache hinzugefügt
                               - OTA Update hinzugefügt
  2024-01 Achim Pfaff          - Verbesserte leere Glas Erkennung, da durch leichtes aufstellen von vollen Gläsern die Abfüllung gestartet wird.
                               - Automatischer Volumenstrom alle 3 Durchgänge
  2024-04 Roland Rust        | W.0.3      
                               - Drehteller integration
                               - Funktion implementiert bei welcher man nach dem Tara das Glas wegnehmen kann und wider Draufstellen mit eider Honigwabe oder sonstigem drin.
                               - HW Level 2 und 3 hinzugefügt. Heltec Wifi Kit V3 (geht nur mit OLED Display)
  2024-12 Roland Rust        | W.0.4
                               - Angefangen mit den Variablen auf Englisch umzustellen (Nicht Hardcore, man wechselt was gerade über den Weg läuft :-))
                               - Es können bis zu 5 voreingestellte Servo Winkel definiert werden
                               - Gewicht wo Feindosierung startet kann nun eingestellt werden.
                               - Angefangen den Code auszulagern
                               - Umstellung für den TFT von Arduino_GFX_Library auf die TFT_eSPI Library
                               - Es können versiedene Schriften für den TFT verwendet werden. Ist nicht mehr auf eine Mono Schrift angewiesen
                               - Progressbar im TFT Display gefixt
                               - Usersettings sind nun im File setup.ini definiert
                               - OTA ist nun in einem eigenen Menü
                               - WebIF hinzugefügt. Wenn keine SSID oder Passwort vorhanden sind, startet der HM im Access Mode                           

  This code is in the public domain.
  
  Hinweise zur Hardware
  ---------------------
  - bei allen digitalen Eingängen sind interne pull downs aktiviert, keine externen-Widerstände nötig!
*/

#include <Arduino.h>
#include <Wire.h>
#include <vector>
#include <U8g2lib.h>                /* aus dem Bibliotheksverwalter */
#include <HX711.h>                  /* aus dem Bibliotheksverwalter */
#include <ESP32Servo.h>             /* aus dem Bibliotheksverwalter */
#include <Adafruit_INA219.h>        /* aus dem Bibliotheksverwalter */
#include <Preferences.h>            /* aus dem BSP von expressif, wird verfügbar wenn das richtige Board ausgewählt ist */
#include <nvs_flash.h>              /* aus dem BSP von expressif, wird verfügbar wenn das richtige Board ausgewählt ist */
#include <TFT_eSPI.h>               /* aus dem Bibliotheksverwalter */

#include "HardwareLevel.h"
#include "variables.h"
#include "variables_display.h"
#include "display.h"
#include "WebIF.h"
#include "read_write_preferences.h"


#if HARDWARE_LEVEL == 2
  TwoWire I2C_2 = TwoWire(1);
#endif

/*#if SCALE_TYP == 1
  #define MAXIMALGEWICHT 1500     // Maximales Gewicht für 2kg Wägezelle
#elif SCALE_TYP == 2
  #define MAXIMALGEWICHT 4500     // Maximales Gewicht für 5kg Wägezelle
#else
  #error Keine Wägezelle definiert
#endif*/

// Ansteuerung der Waage
#define SCALE_READS 2           // Parameter für hx711 Library. Messwert wird aus der Anzahl gemittelt
#define SCALE_GETUNITS(n)       round(scale.get_units(n))

// Rotary Encoder Taster zieht Pegel auf Low
#define SELECT_SW outputSW
#define SELECT_PEGEL LOW

// Betriebsmodi 
#define MODE_SETUP       0
#define MODE_AUTOMATIK   1
#define MODE_HANDBETRIEB 2

// Buzzer Sounds
#define BUZZER_SHORT    1
#define BUZZER_LONG     2
#define BUZZER_SUCCESS  3
#define BUZZER_ERROR    4
#define BUZZER_START_UP 5

// INA 219
Adafruit_INA219 ina219;

// Display
HM_Display dis;

// HM_WEBIF
HM_WEBIF webif;

//Preferences
HM_READ_WRITE_PREFERENCES pref;

//Sprache
#include "./Resources/resources.h"

//OTA
#if OTA == 1
  #include "ota.h"
  HM_OTA ota;
#endif

Servo servo;
HX711 scale;

// Denstrukturen für Rotary Encoder
struct rotary {                        
  int Value;
  int Minimum;
  int Maximum;
  int Step;
};
//#define SW_WINKEL    0
//#define SW_KORREKTUR 1
//#define SW_FLUSS     2
//#define SW_MENU      3
struct rotary rotaries[4];          // Werden im setup() initialisiert
//rotary_select = SW_WINKEL;
bool delay_rotary = false;

// Allgemeine Variablen
int i;                                              // general count variable

//long preferences_chksum;                            // checksum to not write uncoherent prefs

bool counted = true;                                // Jar count flag

int lastpos = 0;                                    // Last position in the setup menu

bool stop_button_used = false;                      // Becomes true when the Stop button is pressed in automatic mode
bool stop_button_close_dp = false;
bool close_drip_protection = false;                 // Flac for the automatic mode that the drip prodection should be closed
int waittime_close_dp = 0;                          // Close drip protection saiting time in automatic mode

//Drehteller
#if DREHTELLER == 1
  #if CHANGE_MAC_ADDRESS_TT == 0
    #include "./Resources/mac.h"
  #endif
  #if CHANGE_MAC_ADDRESS_TT == 1
    const uint8_t MacAdressTurntable[] = {0x74, 0x00, 0x00, 0x00, 0x00, 0x02};
  #endif
  #include <WiFi.h>
  #include <esp_wifi.h>
  #include <esp_now.h>
  esp_now_peer_info_t peerInfo;

  typedef struct messageToBeSent {
    char text[64];
    int value;
  } messageToBeSent;

  typedef struct receivedMessage {
    char text[64];
    int value;
  } receivedMessage;

  messageToBeSent myMessageToBeSent; 
  receivedMessage myReceivedMessage;
  
  void messageSent(const uint8_t *macAddr, esp_now_send_status_t status) {
    #if ESP_NOW_DEBUG >= 1
      Serial.print("Send status: ");
      Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success":"Fail");
    #endif
    //drip_prodection = true;   //warum habe ich das gemacht?
  }
  void messageReceived(const uint8_t* macAddr, const uint8_t* incomingData, int len){
    memset(&myReceivedMessage, 0, sizeof(myReceivedMessage));
    memcpy(&myReceivedMessage, incomingData, sizeof(myReceivedMessage));
    esp_now_msg_recived = true;
    #if ESP_NOW_DEBUG >= 1
      Serial.print("myReceivedMessage.text: Text: "); Serial.print(myReceivedMessage.text); Serial.print(" - Value: "); Serial.println(myReceivedMessage.value);
    #endif
  }
#endif

/*
    Durchschnittsberechnung mit einem Ringbuffer
    https://forum.arduino.cc/t/mittelwert-der-letzten-x-werte/1237432
    2024-04-19 by noiasca
*/
template < typename T, size_t size>
class Avg {
    T store[size] {0};
    size_t index = 0;
    bool wraparound = false;
    
  public:
    void add(T in) {
      store[index] = in;
      index++;
      if (index >= size) {
        index = 0;
        wraparound = true;
      }
    }
    T getAvg() {
      T result = 0;
      if (wraparound) {
        for (auto &i : store) result += i;                    // Normalbetrieb
        return result / size;
      }
      if (index == 0) return 0;                               // Anfangsproblem
      for (size_t i = 0; i < index; i++) result += store[i];  // bis zum einmalig durchlauf aller Werte
      return result / index;
    }
};
Avg <int, 3 > average; // Mittelwerte über 3 Werte AP

// Rotary Taster. Der Interrupt kommt nur im Automatikmodus zum Tragen und nur wenn der Servo inaktiv ist.
// Der Taster schaltet in einen von drei Modi, in denen unterschiedliche Werte gezählt werden.
void IRAM_ATTR isr1() {
  static unsigned long last_interrupt_time = 0; 
  unsigned long interrupt_time = millis();
  if (interrupt_time - last_interrupt_time > 300) {      // If interrupts come faster than 300ms, assume it's a bounce and ignore
    if (mode == MODE_AUTOMATIK && servo_enabled == 0) { // nur im Automatik-Modus interessiert uns der Click
      rotary_select = (rotary_select + 1) % 4;
      #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
        if      (rotary_select == SW_KORREKTUR) {correction_old = -99999;}
        else if (rotary_select == SW_FLUSS)     {intWeight_old = -1; correction_old = -99999;}
        else if (rotary_select == SW_MENU)      {jar_old = -1; intWeight_old = -1;}
        else {correction_old = -99999; intWeight_old = -1; jar_old = -1;}
      #endif
      #if DEBUG_HM >= 1
        Serial.print("Rotary Button changed to ");
        Serial.println(rotary_select);
      #endif 
    }
    last_interrupt_time = interrupt_time;
  }
}

// Rotary Encoder. Zählt in eine von drei Datenstrukturen: 
// SW_WINKEL    = Einstellung des Servo-Winkels
// SW_KORREKTUR = Korrekturfaktor für Füllgewicht
// SW_FLUSS     = Faktor für Flussgeschwindigkeit
// SW_MENU      = Zähler für Menuauswahlen  
void IRAM_ATTR isr2() {
  static unsigned long lastInterruptTime = 0;
  unsigned long now = millis();
  int interrupt_delay = 0;                           // Entprellen 
  if (digitalRead(switch_setup_pin) == HIGH and delay_rotary == true and change_value == false) {
    interrupt_delay = 80;                             // Für alle die im Setup Menue den Rotary drehen wie ein Hornochse. Würde es nur für den TFT brauchen
  }
  if (now - lastInterruptTime > interrupt_delay) {    // Entprellung: x ms Mindestabstand
    int aState = digitalRead(outputA);                // Aktuellen Zustand von outputA lesen
    static int aLastState = 2;                        // reale Werte sind 0 und 1
    if (aState != aLastState) {     
      if (digitalRead(outputB) != aState) {
        rotaries[rotary_select].Value += rotaries[rotary_select].Step;
      } else {  
        rotaries[rotary_select].Value -= rotaries[rotary_select].Step;
      }
      // Begrenzung auf min/max Werte
      rotaries[rotary_select].Value = constrain(
        rotaries[rotary_select].Value,
        rotaries[rotary_select].Minimum,
        rotaries[rotary_select].Maximum
      );
      #if DEBUG_HM >= 4
        Serial.print(" Rotary Value changed to ");
        Serial.println(getRotariesValue(rotary_select));
      #endif
      aLastState = aState;  // Speichern des neuen Zustands
      lastInterruptTime = now;  // Zeit für Entprellung speichern
    }
  }
}

//
// Skalierung des Rotaries für verschiedene Rotary Encoder
int getRotariesValue(int rotary_mode) {
  return ((rotaries[rotary_mode].Value - (rotaries[rotary_mode].Value % (rotaries[rotary_mode].Step * rotary_scale))) / rotary_scale );
}

void setRotariesValue( int rotary_mode, int rotary_value ) {
  rotaries[rotary_mode].Value = rotary_value * rotary_scale;
}

void initRotaries( int rotary_mode, int rotary_value, int rotary_min, int rotary_max, int rotary_step ) {
  rotaries[rotary_mode].Value     = rotary_value * rotary_scale;
  rotaries[rotary_mode].Minimum   = rotary_min   * rotary_scale;
  rotaries[rotary_mode].Maximum   = rotary_max   * rotary_scale;
  rotaries[rotary_mode].Step      = rotary_step;
  #if DEBUG_HM >= 1
    Serial.print("initRotaries..."); 
    Serial.print(" Rotary Mode: ");  Serial.print(rotary_mode);
    Serial.print(" rotary_value: "); Serial.print(rotary_value);
    Serial.print(" Value: ");        Serial.print(rotaries[rotary_mode].Value);
    Serial.print(" Min: ");          Serial.print(rotaries[rotary_mode].Minimum);
    Serial.print(" Max: ");          Serial.print(rotaries[rotary_mode].Maximum);
    Serial.print(" Step: ");         Serial.print(rotaries[rotary_mode].Step);
    Serial.print(" Scale: ");        Serial.println(rotary_scale);
  #endif
}
// Ende Funktionen für den Rotary Encoder
//

void setupTripCounter(void) {
  int j;
  i = 1;
  float filling_weight = 0;
  #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
    draw_frame = true;
  #endif
  while (i > 0) { //Erster Screen: Anzahl pro Glasgröße
    if (digitalRead(button_stop_pin) == HIGH or digitalRead(switch_setup_pin) == LOW) {
      while (digitalRead(button_stop_pin) == HIGH) {
        delay(1);
      }
      mode = -1;
      return;
    }
    dis.setup_trip_counter(1);
    if (digitalRead(SELECT_SW) == SELECT_PEGEL) {
      //verlasse Screen
      while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
        delay(1);
      }
      i = 0;
      change = true;
    }
  }
  i = 1;
  while (i > 0) { //Zweiter Screen: Gewicht pro Glasgröße
    if (digitalRead(button_stop_pin) == HIGH or digitalRead(switch_setup_pin) == LOW) {
      while (digitalRead(button_stop_pin) == HIGH) {
        delay(1);
      }
      mode = -1;
      return;
    }
    dis.setup_trip_counter(2);
    if (digitalRead(SELECT_SW) == SELECT_PEGEL) {
      //verlasse Screen
      while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
        delay(1);
      }
      i = 0;
      change = true;
    }
  }
  i = 1;
  while (i > 0) { //Dritter Screen: Gesamtgewicht
    filling_weight = 0;
    j = 0;
    if (digitalRead(button_stop_pin) == HIGH or digitalRead(switch_setup_pin) == LOW) {
      while (digitalRead(button_stop_pin) == HIGH) {
        delay(1);
      }
      mode = -1;
      return;
    }
    while (j < 5) {
      filling_weight += glaeser[j].Gewicht * glaeser[j].TripCount / 1000.0;
      j++;
    }
    dis.setup_trip_counter(3);
    if (digitalRead(SELECT_SW) == SELECT_PEGEL) {
      //verlasse Screen
      while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
        delay(1);
      }
      i = 0;
      change = true;
    }
  }
  i = 1;
  while (i > 0) { //Vierter Screen: Zurücksetzen
    initRotaries(SW_MENU, 1, 0, 1, 1);
    rotaries[SW_MENU].Value = 1;
    i = 1;
    while (i > 0) {
      if (digitalRead(button_stop_pin) == HIGH or digitalRead(switch_setup_pin) == LOW) {
        while (digitalRead(button_stop_pin) == HIGH) {
          delay(1);
        }
        mode = -1;
        return;
      }
      pos = getRotariesValue(SW_MENU);
      dis.setup_trip_counter(4);
      if (digitalRead(SELECT_SW) == SELECT_PEGEL) {

        dis.setup_exit();
        if ( pos == 0) {
          j = 0;
          while ( j < 5  ) {
            glaeser[j].TripCount = 0;
            j++;
          }
          pref.setPreferences();
        }
        delay(1000);
        mode = -1;
        i = 0;
      }
    }
  }
}

void setupCounter(void) {
  int j;
  i = 1;
  float filling_weight = 0;
  draw_frame = true;
  while (i > 0) { //Erster Screen: Anzahl pro Glasgröße
    if (digitalRead(button_stop_pin) == HIGH or digitalRead(switch_setup_pin) == LOW) {
      while (digitalRead(button_stop_pin) == HIGH) {
        delay(1);
      }
      mode = -1;
      return;
    }
    dis.setup_counter(1);
    if (digitalRead(SELECT_SW) == SELECT_PEGEL) {
      //verlasse Screen
      while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
        delay(1);
      }
      i = 0;
      change = true;
    }
  }
  i = 1;
  while (i > 0) { //Zweiter Screen: Gewicht pro Glasgröße
    if (digitalRead(button_stop_pin) == HIGH or digitalRead(switch_setup_pin) == LOW) {
      while (digitalRead(button_stop_pin) == HIGH) {
        delay(1);
      }
      mode = -1;
      return;
    }
    dis.setup_counter(2);

    if (digitalRead(SELECT_SW) == SELECT_PEGEL) {
      //verlasse Screen
      while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
        delay(1);
      }
      i = 0;
      change = true;
    }
  }
  i = 1;
  while (i > 0) { //Dritter Screen: Gesamtgewicht
    filling_weight = 0;
    j = 0;
    if (digitalRead(button_stop_pin) == HIGH  or digitalRead(switch_setup_pin) == LOW) {
      while (digitalRead(button_stop_pin) == HIGH) {
        delay(1);
      }
      mode = -1;
      return;
    }
    while ( j < 5  ) {
      filling_weight += glaeser[j].Gewicht * glaeser[j].Count / 1000.0;
      j++;
    }
    dis.setup_counter(3);
    if ((digitalRead(SELECT_SW)) == SELECT_PEGEL) {
      //verlasse Screen
      while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
        delay(1);
      }
      i = 0;
      change = true;
    }
  }
  i = 1;
  while (i > 0) { //Vierter Screen: Zurücksetzen
    initRotaries(SW_MENU, 1, 0, 1, 1);
    rotaries[SW_MENU].Value = 1;
    i = 1;
    while (i > 0) {
      if (digitalRead(button_stop_pin) == HIGH or digitalRead(switch_setup_pin) == LOW) {
        while (digitalRead(button_stop_pin) == HIGH) {
          delay(1);
        }
        mode = -1;
        return;
      }
      pos = getRotariesValue(SW_MENU);
      dis.setup_counter(4);
      if (digitalRead(SELECT_SW) == SELECT_PEGEL) {
        while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
          delay(1);
        }
        dis.setup_exit();
        if ( pos == 0) {
          j = 0;
          while ( j < 5  ) {
            glaeser[j].Count = 0;
            glaeser[j].TripCount = 0;
            j++;
          }
          pref.setPreferences();
        }
        delay(1000);
        mode = -1;
        i = 0;
      }
    }
  }
}

void setupTare(void) {
  tare = 0;
  initRotaries(SW_MENU, 0, 0, 4, 1);
  i = 0;
  #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
    change = true;
  #endif
  while (i == 0)  {
    if (digitalRead(button_stop_pin) == HIGH or digitalRead(switch_setup_pin) == LOW) {
      while (digitalRead(button_stop_pin) == HIGH) {
        delay(1);
      }
      mode = -1;
      return;
    }
    else if (digitalRead(SELECT_SW) == SELECT_PEGEL) {
      tare = int(SCALE_GETUNITS(10));
      if (tare > 20) {                  // Gläser müssen mindestens 20g haben
         glaeser[getRotariesValue(SW_MENU)].Tare = tare;
      }
      #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
        change = true;
      #endif
      i++;
    }
    pos = getRotariesValue(SW_MENU);
    dis.setup_tare();
  }
  mode = -1;
  delay(2000);
}

void setupCalibration() {
  float gewicht_raw;
  dis.setup_calibration(1);
  i = 1;
  while (i > 0) {
    if (digitalRead(button_stop_pin) == HIGH or digitalRead(switch_setup_pin) == LOW) {
      while (digitalRead(button_stop_pin) == HIGH) {
        delay(1);
      }
      change_value = false;
      draw_frame = true;
      mode = -1;
      return;
    }
    else if (digitalRead(SELECT_SW) == SELECT_PEGEL) {
      scale.set_scale();
      scale.tare(10);
      delay(500);
      i = 0;
    }
  }
  dis.setup_calibration(2);
  initRotaries(SW_MENU, cali_weight, 100, 999999, 1); 
  i = 1;
  while (i > 0) {
    if (digitalRead(button_stop_pin) == HIGH or digitalRead(switch_setup_pin) == LOW) {
      while (digitalRead(button_stop_pin) == HIGH) {
        delay(1);
      }
      change_value = false;
      draw_frame = true;
      initRotaries(SW_MENU, 0, 0, numItems-1, 1);
      mode = -1;
      return;
    }
    cali_weight = getRotariesValue(SW_MENU);
    dis.setup_calibration(3);
    if (digitalRead(SELECT_SW) == SELECT_PEGEL) {
      gewicht_raw  = scale.get_units(10);
      factor       = gewicht_raw / cali_weight;
      scale.set_scale(factor);
      weight_empty = scale.get_offset();    // Leergewicht der Waage speichern
      #if DEBUG_HM >= 1
        Serial.print("kalibrier_gewicht = ");
        Serial.println(cali_weight);
        Serial.print("weight_empty = ");
        Serial.println(weight_empty);
        Serial.print("gewicht_raw = ");
        Serial.println(gewicht_raw);
        Serial.print(" factor = ");
        Serial.println(factor);
      #endif        
      delay(1000);
      change_value = false;
      draw_frame = true;
      mode = -1;
      i = 0;        
    }
  }
}

void setupMaxWeight() {
  menuitems = {MAX_WEIGHT[lingo], CALIBRATION_WEIGHT[lingo], SAVE[lingo]};
  numItems = menuitems.size();
  change_value = false;
  int lastmax_lc_weight     = max_lc_weight;
  int lastmax_lc_cal        = max_lc_cal;
  initRotaries(SW_MENU, 0, 0, numItems-1, 1);
  draw_frame = true;
  i = 1;
  while (i > 0) {
    if (digitalRead(button_stop_pin) == HIGH or digitalRead(switch_setup_pin) == LOW) {
      while (digitalRead(button_stop_pin) == HIGH) {
        delay(1);
      }
      max_lc_weight = lastmax_lc_weight;
      max_lc_cal = lastmax_lc_cal;
      restartMenu = true;
      draw_frame = true;
      pos = 1;
      return;
    }
    if (change_value == false) {
      pos = getRotariesValue(SW_MENU);
    }
    else {
      switch (pos) {
        case 0: max_lc_weight   = step2loadcell(getRotariesValue(SW_MENU));
                break;
        case 1: max_lc_cal      = step2calweight(getRotariesValue(SW_MENU));
                break;
      }
    }
    dis.setup_max_weight();
    if (digitalRead(SELECT_SW) == SELECT_PEGEL && pos < numItems-1  && change_value == false) {
      while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
        delay(1);
      }
      switch (pos) { 
        case 0: initRotaries(SW_MENU, loadcell2step(max_lc_weight) , 0, 6, 1);
                break;
        case 1: initRotaries(SW_MENU, calweight2step(max_lc_cal) , 0, 12, 1);
                break;
      }
      change_value = true;
    }
    // Änderung im Menupunkt übernehmen
    if (digitalRead(SELECT_SW) == SELECT_PEGEL && pos < numItems-1) {
      while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
        delay(1);
      }
      initRotaries(SW_MENU, pos, 0, numItems-1, 1);
      change_value = false;
    }
    // Menu verlassen
    if (digitalRead(SELECT_SW) == SELECT_PEGEL && pos == numItems-1) {
      while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
        delay(1);
      }
      cali_weight = max_lc_cal;
      dis.setup_exit();
      delay(1000);
      restartMenu = true;
      pos = 0;
      draw_frame = true;
      pref.setPreferences();
      return;
    }
  }
}

void setupScale(void) {
  draw_frame = true;
  restartMenu = true;
  i = 1;
  pos = 0;
  while (i > 0) {
    if (restartMenu) {
      menuitems = {CALIBRATION[lingo], MAX_WEIGHT[lingo], BACK[lingo]};
      numItems = menuitems.size();
      change_value = false;
      initRotaries(SW_MENU, pos, 0, numItems-1, 1);
      restartMenu = false;
    }
    if (digitalRead(button_stop_pin) == HIGH or digitalRead(switch_setup_pin) == LOW) {
      while (digitalRead(button_stop_pin) == HIGH) {
        delay(1);
      }
      mode = -1;
      return;
    }
    if (change_value == false) {
      pos = getRotariesValue(SW_MENU);
      if (pos == numItems - 1) {
        pos = numItems - 1;
      }
    }
    else {
      switch (pos) {
        case 0: setupCalibration();
                break;
        case 1: setupMaxWeight();
                break;
      }
    }
    if (!restartMenu) {
      dis.setup_scale();
    }
    if (digitalRead(SELECT_SW) == SELECT_PEGEL && pos < numItems - 1  && change_value == false) {
      while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
        delay(1);
      }
      change_value = true;
    }
    // Menu verlassen
    if (digitalRead(SELECT_SW) == SELECT_PEGEL && pos == numItems-1) {
      while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
        delay(1);
      }
      dis.setup_exit();
      delay(1000);
      mode = -1;
      i = 0;
    }
  }
}

void setupServoWinkel(void) {
  int lastmin               = angle_min;
  int lastfine              = angle_fine[fullmode-1];
  int lastmax               = angle_max[fullmode-1];
  int lastfinedosageweight  = finedos_weight[fullmode-1];
  int lastfullmode          = fullmode;
  int lastsqueetapleft      = squee_tap_left;
  int lastservoexpanded     = servo_expanded;
  delay_rotary = true;
  draw_frame = true;
  value_old = -1;
  menuitems = {
    LIVESETUP[lingo], MINIMUM[lingo], FULL_MODE[lingo], FINEDOSAGE_WEIGHT[lingo], FINEDOSAGE[lingo], MAXIMUM[lingo],
    SQUEE_TAP_LEFT[lingo], SERVO_EXPANDED[lingo], SAVE[lingo]
  };
  numItems = menuitems.size();
  change_value = false;
  initRotaries(SW_MENU, 0, 0, numItems-1, 1);
  i = 1;
  while (i > 0) {
    if (digitalRead(button_stop_pin) == HIGH or digitalRead(switch_setup_pin) == LOW) {
      while (digitalRead(button_stop_pin) == HIGH) {
        delay(1);
      }
      angle_min                      = lastmin;
      angle_fine[fullmode-1]         = lastfine;
      angle_max[fullmode-1]          = lastmax;
      finedos_weight[fullmode-1]     = lastfinedosageweight;
      fullmode                       = lastfullmode;
      squee_tap_left                 = lastsqueetapleft;
      servo_expanded                 = lastservoexpanded;
      delay_rotary = false;
      if (servo_live == true) {
        servo.write(squee_tap_left ? 180 - angle_min : angle_min);
      }
      mode = -1;
      return;
    }
    if (change_value == false) {
      pos = getRotariesValue(SW_MENU);
    }
    else {
      switch (pos) {
        case 0: servo_live  = getRotariesValue(SW_MENU);
                break;
        case 1: angle_min   = getRotariesValue(SW_MENU);
                if (servo_live == true) servo.write(squee_tap_left ? 180 - angle_min : angle_min);
                break;
        case 2: fullmode    = getRotariesValue(SW_MENU);
                break;
        case 3: finedos_weight[fullmode-1] = getRotariesValue(SW_MENU);
                break;
        case 4: angle_fine[fullmode-1] = getRotariesValue(SW_MENU);
                if (servo_live == true) servo.write(squee_tap_left ? 180 - angle_fine[fullmode-1] : angle_fine[fullmode-1]);
                break;
        case 5: angle_max[fullmode-1]   = getRotariesValue(SW_MENU);
                if (servo_live == true) servo.write(squee_tap_left ? 180 - angle_max[fullmode-1] : angle_max[fullmode-1]);
                break;
        case 6: squee_tap_left          = getRotariesValue(SW_MENU);
                break;
        case 7: servo_expanded          = getRotariesValue(SW_MENU);
                break;
      }
    }
    dis.setup_servoWinkel();
    if (digitalRead(SELECT_SW) == SELECT_PEGEL && pos < numItems-1  && change_value == false) {
      while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
        delay(1);
      }
      switch (pos) { 
        case 0: initRotaries(SW_MENU, servo_live, 0, 1, 1);
              break;
        case 1: initRotaries(SW_MENU, angle_min, angle_hard_min, angle_fine[fullmode-1], 1);
              value_old = lastmin;
              break;
        case 2: initRotaries(SW_MENU, fullmode, 1, 5, 1);
              value_old = lastfullmode;
              break;
        case 3: initRotaries(SW_MENU, finedos_weight[fullmode-1], 10, 500, 1);
              value_old = lastfinedosageweight;
              break;
        case 4: initRotaries(SW_MENU, angle_fine[fullmode-1], angle_min, angle_max[fullmode-1], 1);
              value_old = lastfine;
              break;
        case 5: initRotaries(SW_MENU, angle_max[fullmode-1], angle_fine[fullmode-1], angle_hard_max, 1);
              value_old = lastmax;
              break;
        case 6: initRotaries(SW_MENU, squee_tap_left, 0, 1, 1);
              value_old = lastsqueetapleft;
              break;
        case 7: initRotaries(SW_MENU, servo_expanded, 0, 1, 1);
              value_old = lastservoexpanded;
              break;
      }
      value_old = -1;
      change_value = true;
    }
    if (digitalRead(SELECT_SW) == SELECT_PEGEL && pos < numItems-1  && change_value == true) {
      while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
        delay(1);
      }
      if (servo_live == true) {
        servo.write(squee_tap_left ? 180 - angle_min : angle_min);
      }
      initRotaries(SW_MENU, pos, 0, numItems-1, 1);
      change_value = false;
    }
    if (digitalRead(SELECT_SW) == SELECT_PEGEL && pos == 8) {
      while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
        delay(1);
      }
      dis.setup_exit();
      servo.detach();
      delay(900);
      if (servo_expanded == 1) {
        servo.attach(servo_pin,  750, 2500);
        servo.setPeriodHertz(100);
      }
      else{
        servo.attach(servo_pin, 1000, 2000);
      }
      delay_rotary = false;
      mode = -1;
      delay(100);
      i = 0;
    }
  }
}

void setupAutomatik(void) {
  int lastautostart       = autostart;
  int lastglastoleranz    = jartolerance;
  int lastautokorrektur   = autocorrection;
  int lastkulanz          = overfill_gr;
  int korrektur_alt       = correction;
  int intGewicht_alt      = init_weight_f;
  change_value       = false;
  delay_rotary = true;
  draw_frame = true;
  value_old = -1;
  menuitems = {
    AUTOSTART[lingo], JAR_TOLERANCE[lingo], CORRECTION[lingo], AUTOCORRECTION[lingo], KINDNESS[lingo], FLOW_G_OVER_TIME[lingo],
    WAIT_BEFOR_FILL[lingo], SAVE[lingo]
  };
  numItems = menuitems.size();
  change_value = false;
  initRotaries(SW_MENU, 0, 0, numItems-1, 1);
  i = 1;
  while (i > 0) {
    if (digitalRead(button_stop_pin) == HIGH or digitalRead(switch_setup_pin) == LOW) {
      while (digitalRead(button_stop_pin) == HIGH) {
        delay(1);
      }
      autostart      = lastautostart;
      autocorrection = lastautokorrektur;
      overfill_gr    = lastkulanz;
      jartolerance   = lastglastoleranz;
      correction     = korrektur_alt;
      init_weight_f  = intGewicht_alt;
      setRotariesValue(SW_KORREKTUR, korrektur_alt);
      rotary_select  = SW_MENU;
      delay_rotary = false;
      mode = -1;
      delay_rotary = false;
      return;
    }
    if (change_value == false) {
      pos = getRotariesValue(SW_MENU);
    }
    else {
      switch (pos) {
        case 0: autostart       = getRotariesValue(SW_MENU);
                break;
        case 1: jartolerance    = getRotariesValue(SW_MENU);
                break;
        case 2: correction      = getRotariesValue(SW_KORREKTUR);
                break;
        case 3: autocorrection  = getRotariesValue(SW_MENU);
                break;
        case 4: overfill_gr     = getRotariesValue(SW_MENU);
                break;
        case 5: init_weight_f   = getRotariesValue(SW_MENU);
                break;
        case 6: wait_befor_fill = getRotariesValue(SW_MENU);
                break;
      }
    }
    dis.setup_automatik();
    // Menupunkt zum Ändern ausgewählt
    if (digitalRead(SELECT_SW) == SELECT_PEGEL && pos < numItems-1  && change_value == false) { 
      while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
        delay(1);
      }
      switch (pos) { 
        case 0: rotary_select = SW_MENU;
                initRotaries(SW_MENU, autostart, 0, 1, 1);
                break;
        case 1: rotary_select = SW_MENU;
                initRotaries(SW_MENU, jartolerance, 0, 99, 1);
                break;
        case 2: rotary_select = SW_KORREKTUR;
                break;
        case 3: rotary_select = SW_MENU;
                initRotaries(SW_MENU, autocorrection, 0, 1, 1);
                break;
        case 4: rotary_select = SW_MENU;
                initRotaries(SW_MENU, overfill_gr, 0, 99, 1);
                break;
        case 5: rotary_select = SW_MENU;
                initRotaries(SW_MENU, init_weight_f, 0, 99, 1);
                break;
        case 6: rotary_select = SW_MENU;
                initRotaries(SW_MENU, wait_befor_fill, 0, 1, 1);
                break;
      }
      #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
        value_old = -1;
      #endif
      change_value = true;
    }
    // Änderung im Menupunkt übernehmen
    if (digitalRead(SELECT_SW) == SELECT_PEGEL && pos < numItems-1  && change_value == true) {
      while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
        delay(1);
      }
      rotary_select = SW_MENU;
      initRotaries(SW_MENU, pos, 0, numItems-1, 1);
      change_value = false;
    }
    // Menu verlassen 
    if (digitalRead(SELECT_SW) == SELECT_PEGEL && pos == 7) {
      while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
        delay(1);
      }
      dis.setup_exit();
      delay(1000);
      delay_rotary = false;
      mode = -1;
      i = 0;
    }
  }
  rotary_select = SW_MENU;
}

void setupFuellmenge(void) {
  dis.setup_fuellmenge(0);
  int j;
  initRotaries(SW_MENU, fquantity_index, 0, 4, 1);
  i = 1;
  while (i > 0) {
    if (digitalRead(button_stop_pin) == HIGH or digitalRead(switch_setup_pin) == LOW) {
      while (digitalRead(button_stop_pin) == HIGH) {
        delay(1);
      }
      mode = -1;
      return;
    }
    pos = getRotariesValue(SW_MENU);
    dis.setup_fuellmenge(1);
    if (digitalRead(SELECT_SW) == SELECT_PEGEL) { // Füllmenge gewählt
      while((digitalRead(SELECT_SW) == SELECT_PEGEL)) {
        delay(1);
      }
      i = 0;
    }
  }
  i = 1;
  initRotaries(SW_MENU, weight2step(glaeser[pos].Gewicht) , 25, weight2step(max_lc_weight), 1);
  while (i > 0){
    if ((digitalRead(button_stop_pin)) == HIGH  or digitalRead(switch_setup_pin) == LOW) {
      mode = -1;
      return;
    }
    glaeser[pos].Gewicht = step2weight(getRotariesValue(SW_MENU));
    dis.setup_fuellmenge(2);
    if (digitalRead(SELECT_SW) == SELECT_PEGEL) { // Gewicht bestätigt
      while((digitalRead(SELECT_SW) == SELECT_PEGEL)) {
        delay(1);
      }
      i = 0;
    }
  }
  i = 1;
  initRotaries(SW_MENU, glaeser[pos].GlasTyp, 0, sizeof(GlasTypArray)/sizeof(GlasTypArray[0]) - 1, 1);
  while (i > 0){ 
    if (digitalRead(button_stop_pin) == HIGH or digitalRead(switch_setup_pin) == LOW) {
      while (digitalRead(button_stop_pin) == HIGH) {
        delay(1);
      }
      mode = -1;
      return;
    }
    glaeser[pos].GlasTyp = getRotariesValue(SW_MENU);
    dis.setup_fuellmenge(3);
    if (digitalRead(SELECT_SW) == SELECT_PEGEL) { //GlasTyp bestätigt
      while((digitalRead(SELECT_SW) == SELECT_PEGEL)) {
        delay(1);
      }
      i = 0;
    }
  }
  fquantity       = glaeser[pos].Gewicht;
  tare            = glaeser[pos].Tare;
  fquantity_index = pos; 
  mode            = -1;
  i = 0;
}

void setupParameter(void) {
  int lastbuzzer    = buzzermode;
  int lastled       = ledmode;
  int lastlogo      = showlogo;
  int lastcredits   = showcredits;
  int lastcolor_scheme = color_scheme;
  int lastcolor_marker = color_marker;
  int lastchange_menu_rotation = menu_rotation;
  delay_rotary = true;
  draw_frame = true;
  value_old = -1;
  menuitems = {
    BUZZER[lingo], LED_1[lingo], SHOW_LOGO[lingo], SHOW_CREDITS[lingo], CHANGE_ROTATION[lingo], COLORSCHEME[lingo], MARKER_COLOR[lingo],
    FONT[lingo], SAVE[lingo]
  };
  numItems = menuitems.size();
  int numItemsOld = numItems;
  change_value = false;
  initRotaries(SW_MENU, 0, 0, numItems-1, 1);
  i = 1;
  while (i > 0) {
    if (numItemsOld != numItems) {
      initRotaries(SW_MENU, 0, 0, numItems-1, 1);
      numItemsOld = numItems;
    }
    if (digitalRead(button_stop_pin) == HIGH or digitalRead(switch_setup_pin) == LOW) {
      while (digitalRead(button_stop_pin) == HIGH) {
        delay(1);
      }
      buzzermode = lastbuzzer;
      ledmode = lastled;
      showlogo = lastlogo;
      showcredits = lastcredits;
      color_scheme = lastcolor_scheme;
      color_marker = lastcolor_marker;
      menu_rotation = lastchange_menu_rotation;
      delay_rotary = false;
      mode = -1;
      return;
    }
    if (change_value == false) {
      pos = getRotariesValue(SW_MENU);
    }
    else {
      switch (pos) {
        case 0: buzzermode            = getRotariesValue(SW_MENU);
                break;
        case 1: ledmode               = getRotariesValue(SW_MENU);
                break;
        case 2: showlogo              = getRotariesValue(SW_MENU);
                break;
        case 3: showcredits           = getRotariesValue(SW_MENU);
                break;
        case 4: menu_rotation         = getRotariesValue(SW_MENU);
                break;
        case 5: color_scheme          = getRotariesValue(SW_MENU);
                break;
        case 6: color_marker          = getRotariesValue(SW_MENU);
                break;
        case 7: font_typ              = getRotariesValue(SW_MENU);
                break;
      }
    }
    // Menu
    dis.setup_parameter();
    // Menupunkt zum Ändern ausgewählt
    if (digitalRead(SELECT_SW) == SELECT_PEGEL && pos < numItems-1  && change_value == false) {
      while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
        delay(1);
      } 
      switch (pos) { 
        case 0: initRotaries(SW_MENU, buzzermode,           0, 1, 1);
                break;
        case 1: initRotaries(SW_MENU, ledmode,              0, 1, 1);
                break;
        case 2: initRotaries(SW_MENU, showlogo,             0, 1, 1);
                break;
        case 3: initRotaries(SW_MENU, showcredits,          0, 1, 1);
                break;
        case 4: initRotaries(SW_MENU, menu_rotation,        0, 1, 1);
                break;
        case 5: initRotaries(SW_MENU, color_scheme,         0, 1, 1);
                break;
        case 6: initRotaries(SW_MENU, color_marker,         0, 11, 1);
                break;
        case 7: initRotaries(SW_MENU, font_typ,             0, 1, 1);
                break;
      }
      change_value = true;
    }
    // Änderung im Menupunkt übernehmen
    if (digitalRead(SELECT_SW) == SELECT_PEGEL && pos < numItems-1) {
      while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
        delay(1);
      }
      initRotaries(SW_MENU, pos, 0, numItems-1, 1);
      change_value = false;
    }
    // Menu verlassen
    if (digitalRead(SELECT_SW) == SELECT_PEGEL && pos == numItems-1) {
      while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
        delay(1);
      }
      //setPreferences();
      dis.setup_exit();
      delay(1000);
      delay_rotary = false;
      mode = -1;
      i = 0;
    }
  }
}

void setupClearPrefs(void) {
  menuitems = {
    CLEAR_PREFERENCES[lingo], CLEAR_NVS_MEMORY[lingo], BACK[lingo]
  };
  #if DREHTELLER == 1
    if (menuitems.size() >= 2) {
      size_t last = menuitems.size() - 1;
      menuitems.insert(menuitems.begin() + last, RESET_TURNTABLE[lingo]);
    }
  #endif
  numItems = menuitems.size();
  draw_frame = true;
  initRotaries(SW_MENU, numItems-1 , 0, numItems-1, 1);
  i = 1;
  while (i > 0) {
    if (digitalRead(button_stop_pin) == HIGH  or digitalRead(switch_setup_pin) == LOW) {
      while (digitalRead(button_stop_pin) == HIGH) {
        delay(1);
      }
      mode = -1;
      return;
    }
    pos = getRotariesValue(SW_MENU);
    dis.setup_clear_prefs();
    if ((digitalRead(SELECT_SW)) == SELECT_PEGEL) {  
      while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
        delay(1);
      }
      if (pos == 0) {
        pref.clear_Preferences();
        //Da machen wir gerade einen restart
        ESP.restart();
      }
      else if (pos == 1) {
        nvs_flash_erase();    // erase the NVS partition and...
        nvs_flash_init();     // initialize the NVS partition.
        //Da machen wir gerade einen restart
        ESP.restart();
      }
      else if (pos == 2) {
        if (use_turntable == 1) {
          use_turntable = 0;
          pref.setPreferences();
          //Da machen wir gerade einen restart
          ESP.restart();
        }
      }
      dis.setup_exit();
      delay(1000);
      mode = -1;
      i = 0;
    }
  }
}

void setupINA219(void) {
  int lastcurrent             = current_servo;
  int lastwinkel_min          = angle_min;
  int lastshow_current        = show_current;
  int current_old             = show_current;
  calibration_status          = START[lingo];
  quetschhan                  = CLOSE[lingo];
  bool cal_done = false;
  cal_winkel = 0;
  int j = 0;
  int k = 0;
  change_value = false;
  change = false;
  change_menu = true;
  draw_frame = true;
  value_old = -1;
  i = 1;
  while (i > 0) {
    if (change_menu) {
      change_menu = false;
      change = true;
      if (current_servo > 0) {
        menuitems = {
          SERVO_CURRENT[lingo], CAL_HONEY_GATE[lingo], SAVE[lingo]
        };
      }
      else {
        menuitems = {
          SERVO_CURRENT[lingo], SHOW_CURRENT[lingo], SAVE[lingo]
        };
      }
      numItems = menuitems.size();
      if (change_value == false) {
        initRotaries(SW_MENU, 0, 0, numItems-1, 1);
      }
    }
    if (digitalRead(button_stop_pin) == HIGH  or digitalRead(switch_setup_pin) == LOW) {
      while (digitalRead(button_stop_pin) == HIGH) {
        delay(1);
      }
      current_servo = lastcurrent;
      angle_min = lastwinkel_min;
      show_current = lastshow_current;
      mode = -1;
      return;
    }
    if (change_value == false) {
      pos = getRotariesValue(SW_MENU);
    }
    else {
      switch (pos) {
        case 0: current_servo = getRotariesValue(SW_MENU);
                if (current_servo <= 50 and current_old != current_servo) {
                  change_menu = true;
                  current_old = current_servo;            
                }
                break;
        case 1: if (current_servo == 0) {
                  show_current          = getRotariesValue(SW_MENU);
                }
                else {
                  j                     = 1;
                  calibration_status    = START[lingo];
                  change_value          = false;
                  numItems              = 1;
                  setRotariesValue(SW_MENU, 0);
                  initRotaries(SW_MENU, 0, 0, numItems, 1);
                }
                break;
      }
    }
    // Menu
    dis.setup_INA219(1); //main menu
    if (digitalRead(SELECT_SW) == SELECT_PEGEL && pos < numItems-1  && change_value == false) {
      while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
        delay(1);
      } 
      switch (pos) { 
        case 0: initRotaries(SW_MENU, current_servo, 0, 1500, 50);
                break;
        case 1: if (current_servo == 0) {initRotaries(SW_MENU, show_current, 0, 1, 1);}
                break;
      }
      change_value = true;
    }
    // Änderung im Menupunkt übernehmen
    if (digitalRead(SELECT_SW) == SELECT_PEGEL && pos < numItems-1  && change_value == true) {
      while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
        delay(1);
      }
      initRotaries(SW_MENU, pos, 0, numItems-1, 1);
      change_value = false;
    }
    // Menu verlassen 
    if (digitalRead(SELECT_SW) == SELECT_PEGEL && pos == numItems-1) {
      while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
        delay(1);
      }
      dis.setup_exit();
      delay(1000);
      mode = -1;
      i = 0;
    }
    if (j == 1) {draw_frame = true;}
    while (j > 0) {
      if (digitalRead(switch_setup_pin) == LOW) {
        current_servo = lastcurrent;
        show_current = lastshow_current;
        mode = -1;
        return;
      }
      if (change_value == false) {
        menuitem = getRotariesValue(SW_MENU);
        if (menuitem == numItems) {
          menuitem = 6;
        }
      }
      numItems = 1;
      dis.setup_INA219(2);
      if (change_value == true && menuitem < numItems) {
        lastcurrent = current_servo;
        calibration_status = CALIBRATION_RUNNING[lingo];
        quetschhan = CLOSE[lingo];
        cal_done = false;
        cal_winkel = 0;
        k = 1;
      }
      if (digitalRead(SELECT_SW) == SELECT_PEGEL && menuitem < numItems  && change_value == false) {
        while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
          delay(1);
        }
        change_value = true;
      }
      // Änderung im Menupunkt übernehmen
      if (digitalRead(SELECT_SW) == SELECT_PEGEL && menuitem < numItems  && change_value == true) {
        while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
          delay(1);
        }
        initRotaries(SW_MENU, 0, 0, numItems, 1);
        change_value = false;
      }
      //verlassen
      if ((digitalRead(SELECT_SW) == SELECT_PEGEL && menuitem == 6) or digitalRead(button_stop_pin) == HIGH) {
        while(digitalRead(SELECT_SW) == SELECT_PEGEL or digitalRead(button_stop_pin) == HIGH) {
          delay(1);
        }
        j = 0;
        change_value = false;
        numItems = 2;
        draw_frame = true;
        initRotaries(SW_MENU, 0, 0, numItems, 1);
      }
      if (k == 1) {draw_frame = true;}
      while (k > 0) {
        servo.write(squee_tap_left ? 180 - 90 : 90);
        quetschhan = OPEN[lingo];
        scaletime = millis();
        bool measurement_run = false;
        while (!cal_done) {
          dis.setup_INA219(3);
          if (millis() - scaletime >= 800 and !measurement_run) {
            servo.write(squee_tap_left ? 180 - cal_winkel : cal_winkel);
            quetschhan = CLOSE[lingo];
            measurement_run = true;
            scaletime = millis();
            change = true;
          }
          else if (millis() - scaletime >= 800 and measurement_run) {
            current_mA = GetCurrent(50);
            if (current_mA > current_servo - 30) {   //30mA unter dem max. Wert Kalibrieren
              servo.write(squee_tap_left ? 180 - 90 : 90);
              quetschhan = OPEN[lingo];
              cal_winkel++;
              change = true;
            }
            else {
              cal_done = true;
              calibration_status = CALIBRATION_DONE[lingo];
              angle_min = cal_winkel;
              k++;
            }
            measurement_run = false;
            scaletime = millis();
          }
          //verlassen
          if ((digitalRead(SELECT_SW) == SELECT_PEGEL) or digitalRead(button_stop_pin) == HIGH) {
            while(digitalRead(SELECT_SW) == SELECT_PEGEL or digitalRead(button_stop_pin) == HIGH) {
              delay(1);
            }
            j = 0;
            k = 0;
            draw_frame = true;
            angle_min = lastwinkel_min;
            cal_done = true;
            change_value = false;
            numItems = 2;
            initRotaries(SW_MENU, 0, 0, numItems, 1);
          }
        }
        if (cal_done and k > 1) {draw_frame = true;}
        while (cal_done and k > 1) {
          current_mA = GetCurrent(50);
          if (change_value != current_mA) {
            change = true;
            change_value = current_mA;
          }
          dis.setup_INA219(4);
          //verlassen
          if (digitalRead(button_stop_pin) == HIGH) {
            while(digitalRead(button_stop_pin) == HIGH) {
              delay(1);
            }
            j = 0;
            k = 0;
            draw_frame = true;
            change_value = false;
            numItems = 2;
            angle_min = lastwinkel_min;
            initRotaries(SW_MENU, 0, 0, numItems, 1);
          }
          if (digitalRead(SELECT_SW) == SELECT_PEGEL) {
            while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
              delay(1);
            }
            j = 0;
            k = 0;
            mode = -1;
            change_value = false;
            numItems = 2;
            initRotaries(SW_MENU, 0, 0, numItems, 1);
            pref.setPreferences();
            return;
          }
        }
      }
    }
  }
}

void setupAbout(void) {
  i = 1;
  #if DREHTELLER == 1
    esp_err_t ret = esp_wifi_get_mac(WIFI_IF_STA, baseMac);
  #endif
  dis.setup_about();
  while (i == 1) {
    if (digitalRead(SELECT_SW) == SELECT_PEGEL or digitalRead(button_stop_pin) == HIGH or digitalRead(switch_setup_pin) == LOW) {
      while(digitalRead(SELECT_SW) == SELECT_PEGEL or digitalRead(button_stop_pin) == HIGH) {
        delay(1);
      }
      i = 0;
      mode = -1;
    }
  }
}

void setupLanguage(void) {
  int last_lingo = lingo;
  value_old = -1;
  delay_rotary = true;
  draw_frame = true;
  menuitems.assign(std::begin(LANGUAGE2), std::end(LANGUAGE2));
  numItems = menuitems.size();
  initRotaries(SW_MENU, lingo, 0, numItems, 1);
  i = 1;
  while (i > 0) {
    if (digitalRead(button_stop_pin) == HIGH  or digitalRead(switch_setup_pin) == LOW) {
      while (digitalRead(button_stop_pin) == HIGH) {
        delay(1);
      }
      lingo = last_lingo;
      mode = -1;
      delay_rotary = false;
      return;
    }
    pos = getRotariesValue(SW_MENU);
    if (value_old != pos) {
      value_old = pos;
      dis.setup_language();
      setRotariesValue(SW_MENU, pos); //we einä dreit wiä äs Rindvieh
    }
    // Änderung im Menupunkt übernehmen
    if (digitalRead(SELECT_SW) == SELECT_PEGEL && pos < numItems) {
      while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
        delay(1);
      }
      lingo = pos;
      value_old = -1;
      draw_frame = true;
    }
    // Menu verlassen 
    if (digitalRead(SELECT_SW) == SELECT_PEGEL && pos == numItems) {
      while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
        delay(1);
      }
      dis.setup_exit();
      delay(1000);
      delay_rotary = false;
      mode = -1;
      i = 0;
    }
  }
}

void setupWiFi(void) {
  menuitems = {
    WIFI_SETUP[lingo], BACK[lingo]
  };
  #if OTA == 1
    if (menuitems.size() >= 2) {
      size_t last = menuitems.size() - 1;
      menuitems.insert(menuitems.begin() + last, OTA_ONLINE[lingo]);
      menuitems.insert(menuitems.begin() + last, OTA_OFFLINE[lingo]);
    }
  #endif
  numItems = menuitems.size();
  draw_frame = true;
  initRotaries(SW_MENU, 0 , 0, numItems-1, 1);
  i = 1;
  while (i > 0) {
    if (digitalRead(button_stop_pin) == HIGH  or digitalRead(switch_setup_pin) == LOW) {
      while (digitalRead(button_stop_pin) == HIGH) {
        delay(1);
      }
      mode = -1;
      return;
    }
    pos = getRotariesValue(SW_MENU);
    dis.setup_wifi();
    if ((digitalRead(SELECT_SW)) == SELECT_PEGEL) {
      while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
        delay(1);
      }
      if (pos == 0) {
        webif.setupWebIF();
        draw_frame = true;
      }
      #if OTA == 1
        else if (pos == 1) {
          ota.ota_setup(switch_setup_pin, button_stop_pin);
          draw_frame = true;
        }
        else if (pos == 2 and OTA == 1) {
          ota.online_ota_setup();
          draw_frame = true;
        }
      #endif
      if (menuitems[pos] == BACK[lingo]) {
      dis.setup_exit();
      delay(1000);
      mode = -1;
      i = 0;
      }
    }
  }
}

void setupDrehteller(void) {
  #if DREHTELLER == 1
    unsigned long time;
    unsigned long pre_millis;
    turntable_init = false;
    dis.setup_turntable(0);
    //check connection and the init_turntable
    esp_now_msg_recived = false;
    memset(&myReceivedMessage, 0, sizeof(myReceivedMessage));
    memset(&myMessageToBeSent, 0, sizeof(myMessageToBeSent));
    strcpy(myMessageToBeSent.text, "check");
    espnow_send_data();
    time = millis();
    while (!esp_now_msg_recived and millis() - time <= 1000 and strcmp(myReceivedMessage.text, "") == 0) {
      delay(10);
    }
    if(strcmp(myReceivedMessage.text, "ok_init") == 0 or strcmp(myReceivedMessage.text, "nok_init") == 0 or strcmp(myReceivedMessage.text, "init_error") == 0) {
      if(strcmp(myReceivedMessage.text, "ok_init") == 0) {
        turntable_init = true;
      }
      i = 1;
    }
    else {  //no connection to the turntable
      i = 0;
      esp_now_msg_recived = false;
      dis.setup_turntable(1);
      delay(4000);
      mode = -1;
      return;
    }
    //check if ota update from the turntable is enebled
    int ota_update = 0;
    esp_now_msg_recived = false;
    memset(&myReceivedMessage, 0, sizeof(myReceivedMessage));
    memset(&myMessageToBeSent, 0, sizeof(myMessageToBeSent));
    strcpy(myMessageToBeSent.text, "ota_update_status");
    espnow_send_data();
    time = millis();
    while (!esp_now_msg_recived and millis() - time <= 1000 and strcmp(myReceivedMessage.text, "") == 0) {
      delay(10);
    }
    if(strcmp(myReceivedMessage.text, "ok_ota_update_status") == 0) {
      if (myReceivedMessage.value == 1) {
        ota_update = 1;
      }
    }
    //Variables
    int last_use_turntable = use_turntable;
    int use_turntable_old = use_turntable;
    unsigned long turntable_millis;
    int move_steps = 50;
    int jar_center_pos_old = 0;
    int move_vale = 0;
    bool ts_open_running = false;
    bool ts_close_running = false;
    bool ts_angle_running = false;
    ts_angle_min = 0;
    ts_angle_max = 180;
    int ts_angle_min_old = 0;
    int ts_angle_max_old = 180;
    int ts_angle_min_start = 0;
    int ts_angle_max_start = 0;
    ts_waittime = 0;
    int ts_waittime_old = 0;
    ts_speed = 0;
    int ts_speed_old = 0;
    pos_old = 0;
    change_menu = true;
    turntable_running = false;
    center_jar_running = false;
    change_value = false;
    clear_tft = 0;
    i = 1;
    while(i >= 0) {
      //Main menu
      while (i == 1) {
        if(strcmp(myReceivedMessage.text, "init_error") == 0) {
          memset(&myReceivedMessage, 0, sizeof(myReceivedMessage));
          change_menu = true;
          turntable_init = 0;
        }
        if (change_menu) {
          change_menu = false;
          if (turntable_init and use_turntable) {
            if (clear_tft == 0) {clear_tft = 1;}
            menuitems = {TURNTABLE[lingo], SETUP_TURNTABLE[lingo], SETUP_DRIPPRODECTION[lingo], SAVE[lingo]};
            if (ota_update = 1) {
              auto it = std::find(menuitems.begin(), menuitems.end(), SAVE[lingo]);
              if (it != menuitems.end()) {
                menuitems.insert(it, ENABLE_OTA_UPDATE[lingo]);
              }
            }
          }
          else if (use_turntable) {
            if (clear_tft == 0) {clear_tft = 1;}
            menuitems = {TURNTABLE[lingo], INIT_TURNTABLE[lingo], SAVE[lingo]};
          }
          else {
            if (clear_tft == 0) {clear_tft = 1;}
            menuitems = {TURNTABLE[lingo], SAVE[lingo]};
          }
          numItems = menuitems.size();
          if (!change_value) {
            if (turntable_init) {
              if (pos_old > 0) {
                initRotaries(SW_MENU, pos_old, 0, numItems-1, 1);
                pos_old = 0;
              }
              else {
                initRotaries(SW_MENU, 0, 0, numItems-1, 1);
              }
            }
            else if (use_turntable) {
              initRotaries(SW_MENU, 1, 0, numItems-1, 1);
            }
            else {
              initRotaries(SW_MENU, 0, 0, numItems-1, 1);
            }
          }
        }
        if ((digitalRead(button_stop_pin) == HIGH and turntable_running == false) or digitalRead(switch_setup_pin) == LOW) {
          while (digitalRead(button_stop_pin) == HIGH) {
            delay(1);
          }
          use_turntable = last_use_turntable;
          mode = -1;
          return;
        }
        if (turntable_running == true) {
          initRotaries(SW_MENU, 1, 0, numItems - 1, 1);                                      //Wenn einer am rotary dreht wider zurücksetzen
          if (millis() - time > 60000 or digitalRead(button_stop_pin) == HIGH) {             //Timeaut, wenn der Befehl nicht in 60s geschaft wird oder Abbrechen wenn Stop Taste gedrückt wird
            while (digitalRead(button_stop_pin) == HIGH) {
              delay(1);
            }
            strcpy(myMessageToBeSent.text, "stop");
            espnow_send_data();
            turntable_init = false;
            turntable_running = false;
            esp_now_wait = "";
            change_value = false;
            pre_millis = 0;
            //initRotaries(SW_MENU, 1, 0, numItems - 1, 1);
            
          }
          else if (millis() - pre_millis > 1000 and !turntable_init) {
            pre_millis = millis();
            if (esp_now_wait.length() < 3) {esp_now_wait = esp_now_wait + ".";}
            else {esp_now_wait = "";}
            if(strcmp(myReceivedMessage.text, "ok_init_done") == 0) {
              turntable_init = true;
              change_menu = true;
              turntable_running = false;
              change_value = false;
            }
          }
        }
        if (change_value == false) {
          pos = getRotariesValue(SW_MENU);
        }
        else {
          switch (pos) {
            case 0: use_turntable = getRotariesValue(SW_MENU);
                    if (use_turntable != use_turntable_old) {
                      change_menu = true;
                      use_turntable_old = use_turntable;
                    }
                    break;
          }
        }
        dis.setup_turntable(2);
        if (digitalRead(SELECT_SW) == SELECT_PEGEL && pos < numItems-1  && change_value == false) {
          while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
            delay(1);
          } 
          switch (pos) { 
            case 0: initRotaries(SW_MENU, use_turntable, 0, 1, 1);
                    break;
            case 1: if (turntable_init == false) {
                      make_changes = true;
                      turntable_running = true;
                      time = pre_millis= millis();
                      esp_now_msg_recived = false;
                      strcpy(myMessageToBeSent.text, "init");
                      espnow_send_data();
                    }
                    else if (turntable_init == true) {
                      i = 2;
                    }
                    break;
            case 2: i = 3;
                    break;
            case 3: i = 4;
                    break;
          }
          change_value = true;
        }
        // Änderung im Menupunkt übernehmen
        if (digitalRead(SELECT_SW) == SELECT_PEGEL && pos < numItems-1  && change_value == true) {
          while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
            delay(1);
          }
          initRotaries(SW_MENU, pos, 0, numItems-1, 1);
          change_value = false;
        }
        // Menü verlassen
        if (digitalRead(SELECT_SW) == SELECT_PEGEL && pos == numItems-1) {
          while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
            delay(1);
          }
          dis.setup_exit();
          delay(1000);
          mode = -1;
          return;
        }
      }
      //Menü Drehteller
      if (i == 2) {
        menuitems = {MOVE_JAR[lingo], CENTER_JAR[lingo], SPEED_INIT[lingo], SPEED_RUN[lingo], SAVE[lingo]};
        numItems = menuitems.size();
        initRotaries(SW_MENU, 0, 0, numItems-1, 1);
        change_value = false;
        esp_now_msg_recived = false;
        esp_now_wait = "";
        memset(&myReceivedMessage, 0, sizeof(myReceivedMessage));
        strcpy(myMessageToBeSent.text, "speed_init");
        espnow_send_data();
        turntable_millis = millis();
        jar_center_pos = 0;
        while (millis() - turntable_millis < 3000 and esp_now_msg_recived == false and strcmp(myReceivedMessage.text, "ok_speed_init") != 0) {  //brechce ab wenn in 3 sek keine rückmeldung kommt
          delay(10);
        }
        speed_init = speed_init_old = myReceivedMessage.value;
        esp_now_msg_recived = false;
        memset(&myReceivedMessage, 0, sizeof(myReceivedMessage));
        strcpy(myMessageToBeSent.text, "speed_run");
        espnow_send_data();
        turntable_millis = millis();
        while (millis() - turntable_millis < 3000 and esp_now_msg_recived == false and strcmp(myReceivedMessage.text, "ok_speed_run") != 0) {  //brechce ab wenn in 3 sek keine rückmeldung kommt
          delay(10);
        }
        speed_run = speed_run_old = myReceivedMessage.value;
        clear_tft = 1;

      }
      while (i == 2) {
        //Menü verlassen wenn Drehteller init false wird.
        if (esp_now_msg_recived == true and strcmp(myReceivedMessage.text, "init_error") == 0) {
          esp_now_msg_recived = false;
          turntable_init = false;
          turntable_running = false;
          center_jar_running = false;
          change_value = false;
          turntable_millis = 0;
          esp_now_wait = "";
          i = 1;
        }
        //Abbrechen wenn Stop Taster gedrückt wird
        if (digitalRead(button_stop_pin) == HIGH and turntable_running == false) {
          while (digitalRead(button_stop_pin) == HIGH) {
            delay(1);
          }
          pos_old = 1;
          jar_center_pos = 0;
          turntable_running = false;
          center_jar_running = false;
          change_value = false;
          esp_now_msg_recived = false;
          change_menu = true;
          clear_tft = 2;
          i = 1;
        }
        //Abbrechen wenn Kippschalter benutzt wurde
        if (digitalRead(switch_setup_pin) == LOW) {
          esp_now_msg_recived = false;
          rotary_select = SW_MENU;
          mode = -1;
          memset(&myReceivedMessage, 0, sizeof(myReceivedMessage));
          memset(&myMessageToBeSent, 0, sizeof(myMessageToBeSent));
          return;
        }
        if (change_value == false) {
          pos = getRotariesValue(SW_MENU);
        }
        else {
          switch (pos) {
            case 0: 
                    break;
            case 1: jar_center_pos = getRotariesValue(SW_MENU);
                    break;
            case 2: speed_init = getRotariesValue(SW_MENU);
                    break;
            case 3: speed_run = getRotariesValue(SW_MENU);
                    break;
          }
        }
        if (turntable_running == true) {
          initRotaries(SW_MENU, 0, 0, numItems - 1, 1);                                         //Wenn einer am rotary dreht wider zurücksetzen
          if (millis() - turntable_millis > 60000 or digitalRead(button_stop_pin) == HIGH) {    //Timeaut, wenn der Befehl nicht in 60s geschaft wird oder Abbrechen wenn Stop Taste gedrückt wird
            while (digitalRead(button_stop_pin) == HIGH) {
              delay(1);
            }
            strcpy(myMessageToBeSent.text, "stop");
            espnow_send_data();
            turntable_init = false;
            turntable_running = false;
            esp_now_wait = "";
            change_value = false;
            pre_millis = 0;
            change_menu = true;
            i = 1;
          }
          else if (millis() - pre_millis > 1000 and turntable_init) {
            pre_millis = millis();
            if (esp_now_wait.length() < 3) {esp_now_wait = esp_now_wait + ".";}
            else {esp_now_wait = "";}
            if(strcmp(myReceivedMessage.text, "ok_move_jar") == 0) {
              memset(&myReceivedMessage, 0, sizeof(myReceivedMessage));
              turntable_running = false;
              change_value = false;
              esp_now_wait = "";
            }
          }
        }
        if (center_jar_running == true) {
          initRotaries(SW_MENU, jar_center_pos, 0, 1500, move_steps);   //zurücksetzen wenn zu schnell am rotary gedreht wurde
          if (millis() - turntable_millis > 3000) {                     //verlassen wenn der Befehl nicht in 3sek geschafft wurde
            //muss noch gemacht werden :-)
          }

          if (esp_now_msg_recived == true) {
            //Werte zurücksetzen
            esp_now_msg_recived = false;
            turntable_millis = 0;
            turntable_init = false;
            memset(&myReceivedMessage, 0, sizeof(myReceivedMessage));
          }
        }
        dis.setup_turntable(3);
        if (digitalRead(SELECT_SW) == SELECT_PEGEL && pos < numItems-1  && change_value == false) {
          while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
            delay(1);
          } 
          switch (pos) { 
            case 0: turntable_running = true;
                    turntable_millis = millis();
                    esp_now_msg_recived = false;
                    strcpy(myMessageToBeSent.text, "move_jar");
                    espnow_send_data();
                    break;
            case 1: rotary_select = SW_MENU;
                    initRotaries(SW_MENU, jar_center_pos, 0, 1500, move_steps);
                    center_jar_running = true;
                    break;
            case 2: rotary_select = SW_MENU;
                    initRotaries(SW_MENU, speed_init, 100, 600, move_steps);
                    break;
            case 3: rotary_select = SW_MENU;
                    initRotaries(SW_MENU, speed_run, 100, 600, move_steps);
                    break;
          }
          change_value = true;
        }
        if (center_jar_running and change_value == true) {
          if (jar_center_pos != jar_center_pos_old) {
            move_vale = 0;
            if (jar_center_pos > jar_center_pos_old) {move_vale = 2 * move_steps;}
            else if (jar_center_pos < jar_center_pos_old) {move_vale = (2 * move_steps) * -1;}
            jar_center_pos_old = jar_center_pos;
            if (move_vale != 0) {
              center_jar_running = true;
              esp_now_msg_recived = false;
              turntable_millis = millis();
              strcpy(myMessageToBeSent.text, "move_pos");
              myMessageToBeSent.value = move_vale;
              espnow_send_data();
            }
          }
        }
        // Änderung im Menupunkt übernehmen
        if (digitalRead(SELECT_SW) == SELECT_PEGEL && pos < numItems-1  && change_value == true) {
          while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
            delay(1);
          }
          initRotaries(SW_MENU, pos, 0, numItems-1, 1);
          change_value = false;
          if (center_jar_running == 1) {
            clear_tft = 2;
            change_menu = true;
            center_jar_running = false;
            esp_now_wait = "";
            esp_now_msg_recived = false;
            turntable_millis = 0;
            turntable_init = false;
            memset(&myReceivedMessage, 0, sizeof(myReceivedMessage));
            i = 1;
          }
        }
        // Menü verlassen
        if (digitalRead(SELECT_SW) == SELECT_PEGEL && pos == numItems-1) {
          while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
            delay(1);
          }
          if (speed_init != speed_init_old) {
            strcpy(myMessageToBeSent.text, "speed_init_save");
            myMessageToBeSent.value = speed_init;
            espnow_send_data();
            turntable_millis = millis();
            esp_now_msg_recived = false;
            while (millis() - turntable_millis < 3000 and esp_now_msg_recived == false) {  //brechce ab wenn in 3 sek keine rückmeldung kommt
              delay(10);
            }
          }
          if (speed_run != speed_run_old) {
            strcpy(myMessageToBeSent.text, "speed_run_save");
            myMessageToBeSent.value = speed_run;
            espnow_send_data();
            turntable_millis = millis();
            esp_now_msg_recived = false;
            while (millis() - turntable_millis < 3000 and esp_now_msg_recived == false) {  //brechce ab wenn in 3 sek keine rückmeldung kommt
              delay(10);
            }
          }
          if (jar_center_pos != 0) {
            turntable_init = false;
            strcpy(myMessageToBeSent.text, "pos_jar_steps_save");
            myMessageToBeSent.value = 2 * jar_center_pos;
            espnow_send_data();
            turntable_millis = millis();
            esp_now_msg_recived = false;
            jar_center_pos = 0;
            while (millis() - turntable_millis < 3000 and esp_now_msg_recived == false) {  //brechce ab wenn in 3 sek keine rückmeldung kommt
              delay(10);
            }
          }
          dis.setup_exit();
          delay(1000);
          pos_old = 1;
          esp_now_msg_recived = false;
          change_menu = true;
          clear_tft = 2;
          i = 1;
        }
      }
      //Menü Tropfschutz
      if (i == 3) {
        menuitems = {OPEN_DRIPPROTECTION[lingo], CLOSE_DRIPPROTECTION[lingo], SPEED_DRIPPROTECTION[lingo], WAIT_TO_CLOSE_DP[lingo], DP_MIN_ANGLE[lingo], DP_MAX_ANGLE[lingo], SAVE[lingo]};
        numItems = menuitems.size();
        change_value = false;
        initRotaries(SW_MENU, 0, 0, numItems-1, 1);
        ts_open_running = false;
        ts_close_running = false;
        ts_angle_running = false;
        esp_now_msg_recived = false;
        memset(&myReceivedMessage, 0, sizeof(myReceivedMessage));
        strcpy(myMessageToBeSent.text, "ts_angle_min");
        espnow_send_data();
        turntable_millis = millis();
        while (millis() - turntable_millis < 3000 and esp_now_msg_recived == false and strcmp(myReceivedMessage.text, "ok_ts_angle_min") != 0) {  //brechce ab wenn in 3 sek keine rückmeldung kommt
          delay(10);
        }
        ts_angle_min = ts_angle_min_old = ts_angle_min_start = myReceivedMessage.value;
        esp_now_msg_recived = false;
        memset(&myReceivedMessage, 0, sizeof(myReceivedMessage));
        strcpy(myMessageToBeSent.text, "ts_angle_max");
        espnow_send_data();
        turntable_millis = millis();
        while (millis() - turntable_millis < 3000 and esp_now_msg_recived == false and strcmp(myReceivedMessage.text, "ok_ts_angle_max") != 0) {  //brechce ab wenn in 3 sek keine rückmeldung kommt
          delay(10);
        }
        ts_angle_max = ts_angle_max_old = ts_angle_max_start = myReceivedMessage.value;
        esp_now_msg_recived = false;
        memset(&myReceivedMessage, 0, sizeof(myReceivedMessage));
        strcpy(myMessageToBeSent.text, "ts_waittime");
        espnow_send_data();
        turntable_millis = millis();
        while (millis() - turntable_millis < 3000 and esp_now_msg_recived == false and strcmp(myReceivedMessage.text, "ok_ts_waittime") != 0) {  //brechce ab wenn in 3 sek keine rückmeldung kommt
          delay(10);
        }
        ts_waittime = ts_waittime_old = myReceivedMessage.value;
        esp_now_msg_recived = false;
        memset(&myReceivedMessage, 0, sizeof(myReceivedMessage));
        strcpy(myMessageToBeSent.text, "ts_speed");
        espnow_send_data();
        turntable_millis = millis();
        while (millis() - turntable_millis < 3000 and esp_now_msg_recived == false and strcmp(myReceivedMessage.text, "ok_ts_speed") != 0) {  //brechce ab wenn in 3 sek keine rückmeldung kommt
          delay(10);
        }
        clear_tft = 1;
        ts_speed = ts_speed_old = myReceivedMessage.value;
        esp_now_msg_recived = false;
        memset(&myReceivedMessage, 0, sizeof(myReceivedMessage));
      }
      while (i == 3) {
        //Abbrechen wenn Stop Taster gedrückt wird
        if (digitalRead(button_stop_pin) == HIGH) {
          while (digitalRead(button_stop_pin) == HIGH) {
            delay(1);
          }
          pos_old = 2;
          ts_angle_min = ts_angle_min_old;
          ts_angle_max = ts_angle_max_old;
          ts_waittime = ts_waittime_old;
          ts_speed = ts_speed_old;
          esp_now_msg_recived = false;
          change_value = false;
          change_menu = true;
          clear_tft = 2;
          i = 1;
          break;
        }
        //Abbrechen wenn Kippschalter benutzt wurde
        if (digitalRead(switch_setup_pin) == LOW) {
          esp_now_msg_recived = false;
          rotary_select = SW_MENU;
          mode = -1;
          memset(&myReceivedMessage, 0, sizeof(myReceivedMessage));
          memset(&myMessageToBeSent, 0, sizeof(myMessageToBeSent));
          return;
        }
        if (ts_open_running == true or ts_close_running == true or ts_angle_running == true) { //zurücksetzen wenn zu schnell am rotary gedreht wurde
          if (ts_open_running == true or ts_close_running) {initRotaries(SW_MENU, pos, 0, numItems - 1, 1);}
          else if (ts_angle_running == true and pos == 4) {initRotaries(SW_MENU, ts_angle_min, 0, ts_angle_max, 5);}
          else if (ts_angle_running == true and pos == 5) {initRotaries(SW_MENU, ts_angle_max, ts_angle_min, 180, 5);}
          if (millis() - turntable_millis > 3000) {   //verlassen wenn der Befehl nicht in 3sek geschafft wurde
            //muss noch gemacht werden :-)
          }
          if (esp_now_msg_recived == true) {
            //Werte zurücksetzen
            if (ts_open_running == true or ts_close_running) {change_value = false;}
            esp_now_msg_recived = false;
            ts_open_running = false;
            ts_close_running = false;
            ts_angle_running = false;
            turntable_millis = 0;
            memset(&myReceivedMessage, 0, sizeof(myReceivedMessage));
          }
        }
        if (change_value == false) {
          pos = getRotariesValue(SW_MENU);
        }
        else {
          switch (pos) {
            case 0: 
                    break;
            case 1: jar_center_pos = getRotariesValue(SW_MENU);
                    break;
            case 2:ts_speed = getRotariesValue(SW_MENU);
                    break;
            case 3: ts_waittime = getRotariesValue(SW_MENU);
                    break;
            case 4: ts_angle_min = getRotariesValue(SW_MENU);
                    break;
            case 5: ts_angle_max = getRotariesValue(SW_MENU);
                    break;
          }
        }
        dis.setup_turntable(4);
        if (digitalRead(SELECT_SW) == SELECT_PEGEL && pos < numItems-1  && change_value == false) {
          while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
            delay(1);
          } 
          switch (pos) { 
            case 0: ts_open_running = true;
                    turntable_millis = millis();
                    esp_now_msg_recived = false;
                    strcpy(myMessageToBeSent.text, "open_drop_prodection");
                    espnow_send_data();
                    break;
            case 1: ts_close_running = true;
                    turntable_millis = millis();
                    esp_now_msg_recived = false;
                    strcpy(myMessageToBeSent.text, "close_drop_prodection");      //braucht keine Wartezeit
                    espnow_send_data();
                    break;
            case 2: rotary_select = SW_MENU;
                    initRotaries(SW_MENU, ts_speed, 0, 500, 10);
                    break;
            case 3: rotary_select = SW_MENU;
                    initRotaries(SW_MENU, ts_waittime, 2, 120, 1);
                    break;
            case 4: rotary_select = SW_MENU;
                    initRotaries(SW_MENU, ts_angle_min, 0, ts_angle_max, 5);
                    break;
            case 5: rotary_select = SW_MENU;
                    initRotaries(SW_MENU, ts_angle_max, ts_angle_min, 180, 5);
                    break;
          }
          change_value = true;
        }
        // Änderung im Menupunkt übernehmen
        if (digitalRead(SELECT_SW) == SELECT_PEGEL && pos < numItems-1  && change_value == true) {
          while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
            delay(1);
          }
          initRotaries(SW_MENU, pos, 0, numItems-1, 1);
          change_value = false;
          esp_now_wait = "";
          esp_now_msg_recived = false;
          memset(&myReceivedMessage, 0, sizeof(myReceivedMessage));
        }
        // Menü verlassen
        if (digitalRead(SELECT_SW) == SELECT_PEGEL && pos == numItems-1) {
          while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
            delay(1);
          }
          if (ts_angle_min != ts_angle_min_start) {
            strcpy(myMessageToBeSent.text, "ts_angle_min_save");
            myMessageToBeSent.value = ts_angle_min;
            espnow_send_data();
            turntable_millis = millis();
            esp_now_msg_recived = false;
            while (millis() - turntable_millis < 3000 and esp_now_msg_recived == false) {  //brechce ab wenn in 3 sek keine rückmeldung kommt
              delay(10);
            }
          }
          if (ts_angle_max != ts_angle_max_start) {
            strcpy(myMessageToBeSent.text, "ts_angle_max_save");
            myMessageToBeSent.value = ts_angle_max;
            espnow_send_data();
            turntable_millis = millis();
            esp_now_msg_recived = false;
            while (millis() - turntable_millis < 3000 and esp_now_msg_recived == false) {  //brechce ab wenn in 3 sek keine rückmeldung kommt
              delay(10);
            }
          }
          if (ts_speed != ts_speed_old) {
            strcpy(myMessageToBeSent.text, "ts_speed_save");
            myMessageToBeSent.value = ts_speed;
            espnow_send_data();
            turntable_millis = millis();
            esp_now_msg_recived = false;
            while (millis() - turntable_millis < 3000 and esp_now_msg_recived == false) {  //brechce ab wenn in 3 sek keine rückmeldung kommt
              delay(10);
            }
          }
          if (ts_waittime != ts_waittime_old) {
            strcpy(myMessageToBeSent.text, "ts_waittime_save");
            myMessageToBeSent.value = ts_waittime;
            espnow_send_data();
            turntable_millis = millis();
            esp_now_msg_recived = false;
            while (millis() - turntable_millis < 3000 and esp_now_msg_recived == false) {  //brechce ab wenn in 3 sek keine rückmeldung kommt
              delay(10);
            }
          }
          dis.setup_exit();
          delay(1000);
          pos_old = 2;
          esp_now_msg_recived = false;
          change_menu = true;
          ts_angle_running = false;
          ts_close_running = false;
          ts_open_running = false;
          clear_tft = 2;
          i = 1;
        }
      }
      //Menü OTA Update aktivieren
      if (i == 4) {
        //ota_status: 0 --> enable ota update
        //            1 --> Turntable has an IP address
        //            2 --> Turntable was not enable to get an IP address
        //            3 --> OTA update started
        //            4 --> OTA update fail
        //            5 --> OTA update success
        ota_status = 0;
        memset(&myReceivedMessage, 0, sizeof(myReceivedMessage));
        memset(&myMessageToBeSent, 0, sizeof(myMessageToBeSent));
        esp_now_msg_recived = false;
        strcpy(myMessageToBeSent.text, "enable_ota_update");
        espnow_send_data();
        time = millis();
        dis.setup_turntable(5);
        // Turntable try to get an IP address
        while (ota_status == 0 and millis() - time <= 5000 ) { //war mal 16000
          if (strcmp(myReceivedMessage.text, "") != 0) {
            strcpy(myReceivedMessage_text, myReceivedMessage.text);
            ota_status = 1;
            delay(10);
          }
          if (ota_status == 0 and millis() - time >= 5000) { //war mal 16000
            ota_status = 2;
          }
        }
        dis.setup_turntable(5);
        //Disable WiFi
        if (ota_status == 2) {
          strcpy(myMessageToBeSent.text, "stop_ota_update");
          espnow_send_data();
          time = millis();
          while (!esp_now_msg_recived and millis() - time <= 1000 and strcmp(myReceivedMessage.text, "") == 0) {
            delay(10);
          }
          delay(3000);
          pos_old = 3;
          esp_now_msg_recived = false;
          change_value = false;
          change_menu = true;
          i = 1;
        }
      }
      while (i == 4) {
        //Abbrechen wenn Stop gedrückt wird
        if (digitalRead(button_stop_pin) == HIGH) {
          while (digitalRead(button_stop_pin) == HIGH) {
            delay(1);
          }
          strcpy(myMessageToBeSent.text, "stop_ota_update");
          espnow_send_data();
          time = millis();
          while (!esp_now_msg_recived and millis() - time <= 1000 and strcmp(myReceivedMessage.text, "") == 0) {
            delay(10);
          }
          pos_old = 3;
          esp_now_msg_recived = false;
          change_value = false;
          change_menu = true;
          i = 1;
          break;
        }
        //Abbrechen wenn Kippschalter benutzt wurde
        if (digitalRead(switch_setup_pin) == LOW) {
          memset(&myReceivedMessage, 0, sizeof(myReceivedMessage));
          strcpy(myMessageToBeSent.text, "stop_ota_update");
          espnow_send_data();
          time = millis();
          while (!esp_now_msg_recived and millis() - time <= 1000 and strcmp(myReceivedMessage.text, "") == 0) {
            delay(10);
          }
          esp_now_msg_recived = false;
          rotary_select = SW_MENU;
          mode = -1;
          return;
        }
        //OTA Update
        if (ota_status != 3 and strcmp(myReceivedMessage.text, "") != 0 and strcmp(myReceivedMessage.text, "OTAStart") != 0) {
          dis.setup_turntable(5);
          memset(&myReceivedMessage, 0, sizeof(myReceivedMessage));
        }
        if (ota_status == 1 and strcmp(myReceivedMessage.text, "") != 0 and strcmp(myReceivedMessage.text, "Success") != 0 and strcmp(myReceivedMessage.text, "Fail") != 0) {
          ota_status = 3;
          clear_tft = 1;
        }
        while (ota_status == 3) {
          if (strcmp(myReceivedMessage.text, "") != 0  and strcmp(myReceivedMessage.text, "Success") != 0 and strcmp(myReceivedMessage.text, "Fail") != 0 and strcmp(myReceivedMessage.text, "OTAStart") != 0) {
            strcpy(myReceivedMessage_text, myReceivedMessage.text);
            myReceivedMessage_value = myReceivedMessage.value;
            dis.setup_turntable(5);
          }
          if (strcmp(myReceivedMessage.text, "Fail") == 0) {
            ota_status = 4;
          }
          if (strcmp(myReceivedMessage.text, "Success") == 0) {
            ota_status = 5;
          }
        }
        if (ota_status == 4 or ota_status == 5) {
          dis.setup_turntable(5);
          delay(3000);
          memset(&myReceivedMessage, 0, sizeof(myReceivedMessage));
          esp_now_msg_recived = false;
          rotary_select = SW_MENU;
          mode = -1;
          return;
        }
      }
    }
  #endif
}

void processSetup(void) {
  int x_pos;
  menuitem_old = -1;
  size_t last;
  menuitems = {
    LANGUAGE1[lingo], TAREVALUES[lingo], SCALE[lingo], FILL_QUANTITY[lingo], AUTOMATIC[lingo], SERVOSETTINGS[lingo], PARAMETER[lingo],
    COUNTER[lingo], COUNTER_TRIP[lingo], ABOUT[lingo], CLEAR_PREFS[lingo]
  };
  if (ina219_installed) {
    last = menuitems.size() - 2;
    menuitems.insert(menuitems.begin() + last, INA219_SETUP[lingo]);
  }
  if (DREHTELLER) {
    last = menuitems.size() - 2;
    menuitems.insert(menuitems.begin() + last, TURNTABLE[lingo]);
    menuitems.insert(menuitems.begin() + last, WIFI[lingo]);
  }
  else if (OTA) {
    last = menuitems.size() - 2;
    menuitems.insert(menuitems.begin() + last, WIFI[lingo]);
  }
  numItems = menuitems.size();
  posmenu.resize(numItems);
  mode = MODE_SETUP;
  angle = angle_min;              // Hahn schliessen
  servo_enabled = 0;              // Servo-Betrieb aus
  servo.write(squee_tap_left ? 180 - angle : angle);
  rotary_select = SW_MENU;
  initRotaries(SW_MENU, lastpos, -1, numItems, -1);
  while (mode == MODE_SETUP and (digitalRead(switch_setup_pin)) == HIGH) {
    if (rotaries[SW_MENU].Value < 0) {
      rotaries[SW_MENU].Value = (numItems * rotary_scale) - 1;
    }
    else if (rotaries[SW_MENU].Value > (numItems * rotary_scale) - 1) {
      rotaries[SW_MENU].Value = 0;
    }
    menuitem = getRotariesValue(SW_MENU) % numItems;
    for (i = 0; i < numItems; i++) {
      posmenu[i] = (menuitem + i) % numItems;
    }
    dis.process_setup();
    menuitem_old = menuitem;
    lastpos = menuitem;
    if (digitalRead(SELECT_SW) == SELECT_PEGEL) {
      while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
        delay(1);
      }
    #if DEBUG_HM >= 1
      Serial.print("Setup Position: ");
      Serial.println(menuitem);
    #endif
      lastpos = menuitem;
      if (menuitems[menuitem] == TAREVALUES[lingo])        setupTare();              // Tara 
      if (menuitems[menuitem] == SCALE[lingo])             setupScale();             // Waage 
      if (menuitems[menuitem] == FILL_QUANTITY[lingo])     setupFuellmenge();        // Füllmenge 
      if (menuitems[menuitem] == AUTOMATIC[lingo])         setupAutomatik();         // Autostart/Autokorrektur konfigurieren 
      if (menuitems[menuitem] == SERVOSETTINGS[lingo])     setupServoWinkel();       // Servostellungen Minimum, Maximum und Feindosierung
      if (menuitems[menuitem] == PARAMETER[lingo])         setupParameter();         // Sonstige Einstellungen
      if (menuitems[menuitem] == COUNTER[lingo])           setupCounter();           // Zählwerk
      if (menuitems[menuitem] == COUNTER_TRIP[lingo])      setupTripCounter();       // Zählwerk Trip
      if (menuitems[menuitem] == INA219_SETUP[lingo])      setupINA219();            // INA219 Setup
      if (menuitems[menuitem] == TURNTABLE[lingo])         setupDrehteller();        // Turntable Setup
      if (menuitems[menuitem] == LANGUAGE1[lingo])         setupLanguage();          // Language setup
      if (menuitems[menuitem] == ABOUT[lingo])             setupAbout();             // About setup
      if (menuitems[menuitem] == WIFI[lingo])              setupWiFi();              // Setup WiFi
      pref.setPreferences();
      if (menuitems[menuitem] == CLEAR_PREFS[lingo])       setupClearPrefs();        // EEPROM löschen
      initRotaries(SW_MENU,lastpos, 0,255, 1);                                       // Menu-Parameter könnten verstellt worden sein
    }
    //das wird noch in ein menü verschoben
    #if OTA == 1
      if (digitalRead(button_start_pin) == HIGH) {
        while (digitalRead(button_start_pin) == HIGH) {delay(100);}
        menuitem_old = -1;
        ota.ota_setup(switch_setup_pin, button_stop_pin);
      }
    #endif
  }
}

void processAutomatic(void) {
  int force_servo_active  = 0;
  full = false;
  static float weight_old2;        // Gewicht des vorhergehenden Durchlaufs A.P.
  static int weight_before;        // Gewicht des vorher gefüllten Glases
  static int collector_num = 5;    // Anzahl identischer Messungen für Nachtropfen
  int loop1 = 3;                   // anzahl alle wieviel Durchgänge die auto Servo einstellung gemacht werden soll A.P.
  y_offset_ina = 0;
  #if DREHTELLER == 1
    unsigned long time;
  #endif
  if (mode != MODE_AUTOMATIK) {
    angle = angle_min;              // Hahn schliessen
    servo_enabled = 0;              // Servo-Betrieb aus
    servo.write(squee_tap_left ? 180 - angle : angle);
    auto_enabled = 0;               // automatische Füllung starten
    tare_jar = 0;
    tare_old_automatic = -999;
    rotary_select = SW_WINKEL;      // Einstellung für Winkel über Rotary
    offset_angle = 0;               // Offset vom Winkel wird auf 0 gestellt
    initRotaries(SW_MENU, fquantity_index, 0, 4, 1);
    setRotariesValue(SW_FLUSS, init_weight_f);  //warum auch immer :-)
    drip_prodection = true;
    weight_before = glaeser[fquantity_index].Gewicht + correction;
    stop_wait_befor_fill = 0;
    pos_old = -1;
    #if DREHTELLER == 1
      turntable_moving = false;
      turntable_jar_full_flag = false;
      stop_button_used = false;
      stop_button_close_dp = false;
      esp_now_msg_recived = false;
      close_drip_protection = false;
      memset(&myReceivedMessage, 0, sizeof(myReceivedMessage));
      if (use_turntable == 1) {
        turntable_ok = false;
        strcpy(myMessageToBeSent.text, "close_drop_prodection");    //braucht keine Wartezeit
        espnow_send_data();
      }
      else {
        strcpy(myMessageToBeSent.text, "open_drop_prodection");
        espnow_send_data();
      }
      time = millis();
      while (!esp_now_msg_recived and millis() - time <= 1000 and strcmp(myReceivedMessage.text, "") == 0) {
        delay(10);
      }
      if (use_turntable == 1) {
        #if ESP_NOW_DEBUG  >= 1
          Serial.println("Use turntable");
        #endif
        dis.process_automatic(1); //clear display
        turntable_init = false;
        esp_now_msg_recived = false;
        memset(&myReceivedMessage, 0, sizeof(myReceivedMessage));
        memset(&myMessageToBeSent, 0, sizeof(myMessageToBeSent));
        strcpy(myMessageToBeSent.text, "check");
        espnow_send_data();
        time = millis();
        while (!esp_now_msg_recived and millis() - time <= 1000 and strcmp(myReceivedMessage.text, "") == 0) {
          delay(10);
        }
        if(strcmp(myReceivedMessage.text, "ok_init") == 0 or strcmp(myReceivedMessage.text, "nok_init") == 0 or strcmp(myReceivedMessage.text, "init_error") == 0) {
          if(strcmp(myReceivedMessage.text, "ok_init") == 0) {
            turntable_init = true;
          }
        }
        else {
          esp_now_msg_recived = false;
          dis.process_automatic(2); //connection failed
          while (digitalRead(switch_betrieb_pin) == HIGH) {
            if (digitalRead(switch_betrieb_pin) == HIGH) {delay(10);}
          }
          mode = -1;
          return;
        }
        if (turntable_init == false) {
          dis.process_automatic(3); //init turntable nok
          esp_now_msg_recived = false;
          while (digitalRead(switch_betrieb_pin) == HIGH and esp_now_msg_recived == false) {
            if (digitalRead(outputSW) == LOW) {
              while (digitalRead(outputSW) == LOW) {
                delay(10);
              }
              strcpy(myMessageToBeSent.text, "init");
              int turntable_millis = millis();
              espnow_send_data();
              while ((millis() - turntable_millis > 60000 or esp_now_msg_recived == false) and digitalRead(switch_betrieb_pin) == HIGH) {
                delay(10);
              }
              if (digitalRead(switch_betrieb_pin) == LOW and esp_now_msg_recived == false) {
                esp_now_msg_recived = false;
                strcpy(myMessageToBeSent.text, "stop");
                turntable_millis = millis();
                espnow_send_data();
                while ((millis() - turntable_millis > 1000 or esp_now_msg_recived == false) and digitalRead(switch_betrieb_pin) == LOW) {
                  delay(10);
                }
              }
            }
          }
          mode = -1;
          return;
        }
        esp_now_msg_recived = false;
        waittime_close_dp = 0;
        memset(&myReceivedMessage, 0, sizeof(myReceivedMessage));
        strcpy(myMessageToBeSent.text, "ts_waittime");
        espnow_send_data();
        time = millis();
        while (!esp_now_msg_recived and millis() - time <= 1000 and strcmp(myReceivedMessage.text, "") == 0) {
          delay(10);
        }
        if (strcmp(myReceivedMessage.text, "ok_ts_waittime") == 0) {
          waittime_close_dp = myReceivedMessage.value;
        }
        if (waittime_close_dp <= 0) {
          dis.process_automatic(4); //close dropprotection
          while (digitalRead(switch_betrieb_pin) == HIGH) {
            if (digitalRead(switch_betrieb_pin) == HIGH) {delay(10);}
          }
        }
        esp_now_msg_recived = false;
        memset(&myReceivedMessage, 0, sizeof(myReceivedMessage));
        memset(&myMessageToBeSent, 0, sizeof(myMessageToBeSent));
      }
    #endif
    //#if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
      if (current_servo > 0) {
        no_ina = true;
      }
      else {
        no_ina = false;
      }
      jar_on_scale = false;
      jar_old = -1;
      current_mA_old = -1;
      correction_old = -99999;
      autocorrection_gr_old = -99999;
      angle_min_old = -1;
      angle_ist_old = -1;
      servo_enabled_old = -1;
      auto_enabled_old = -1;
      weight_old = -9999999;
      intWeight_old = -1;
      y_pos_weight = false;
      dis.process_automatic(8);
    //#endif
    mode = MODE_AUTOMATIK;
  }
  pos = getRotariesValue(SW_WINKEL);
  // nur bis winkel_fein regeln, oder über initRotaries lösen?
  if (pos < angle_fine[fullmode-1] * 100 / angle_max[fullmode-1]) {                      
    pos = angle_fine[fullmode-1] * 100 / angle_max[fullmode-1];
    setRotariesValue(SW_WINKEL, pos);
  }
  correction      = getRotariesValue(SW_KORREKTUR);
  init_weight_f   = getRotariesValue(SW_FLUSS);
  fquantity_index = getRotariesValue(SW_MENU);
  fquantity       = glaeser[fquantity_index].Gewicht;
  tare            = glaeser[fquantity_index].Tare;
  if (tare <= 0) { 
    auto_enabled = 0;
  }
  // wir starten nur, wenn das Tara für die Füllmenge gesetzt ist!
  // Ein erneuter Druck auf Start erzwingt die Aktivierung des Servo
  if (digitalRead(button_start_pin) == HIGH && tare > 0) {
    while(digitalRead(button_start_pin) == HIGH) {
       delay(1);
    }
    if (auto_enabled == 1) {
      force_servo_active = 1;
      #if DEBUG_HM >= 1
        Serial.println("erzwinge Servo aktiv");      
      #endif
    }
    auto_enabled    = 1;                                // automatisches Füllen aktivieren
    rotary_select = SW_WINKEL;                          // falls während der Parameter-Änderung auf Start gedrückt wurde    
    pref.setPreferences();                                   // falls Parameter über den Rotary verändert wurden
    //#if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
      jar_old = -1;                                     // Glas Typ Farbe zurücksetzen fals markiert ist
      correction_old = -99999;                          // Korrektur Farbe zurücksetzen fals markiert ist
    //#endif
  }
  if (digitalRead(button_stop_pin) == HIGH or stop_wait_befor_fill == 1) {
    while(digitalRead(button_stop_pin) == HIGH) {
       delay(1);
    }
    angle       = angle_min + offset_angle;
    servo_enabled = 0;
    auto_enabled  = 0;
    tare_jar    = 0;
    stop_wait_befor_fill = 0;
    #if DREHTELLER == 1
      if (use_turntable == 1) {
        stop_button_used = true;
        if (drip_prodection == false) {
          stop_button_close_dp = true;
        }
        //#if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
          //TFT Display update erzwingen
          jar_on_scale = false;
          weight_old = -999999;
        //#endif
      }
    #endif
  }
  // Fehlerkorrektur der Waage, falls Gewicht zu sehr schwankt 
  #if FEHLERKORREKTUR_WAAGE == 1
    int Vergleichsgewicht = (int(SCALE_GETUNITS(SCALE_READS))) - tare;
    for (byte j = 0 ; j < 3; j++) { // Anzahl der Wiederholungen, wenn Abweichung zu hoch
      weight = int(SCALE_GETUNITS(SCALE_READS)) - tare;
      if (abs(weight - Vergleichsgewicht) < 50)  // Abweichung für Fehlererkennung
        break; 
      delay(100);
    }
  #else
    weight = int(SCALE_GETUNITS(SCALE_READS)) -   tare;
  #endif 
  // Glas entfernt -> Servo schliessen
  if (weight < -20) {
    angle       = angle_min + offset_angle;
    servo_enabled = 0;
    tare_jar    = 0;
    stop_wait_befor_fill = 0;
    if (autostart != 1) {  // Autostart nicht aktiv
      auto_enabled  = 0;
    }
  }
  // Automatik ein, leeres Glas aufgesetzt, Servo aus -> Glas füllen
  if (auto_enabled == 1 && abs(weight) <= jartolerance && servo_enabled == 0 and turntable_ok == true and stop_wait_befor_fill == 0 and stop_button_close_dp == 0){
    rotary_select = SW_WINKEL;     // falls während der Parameter-Änderung ein Glas aufgesetzt wird
    dis.process_automatic(5); //Start
    // A.P. kurz warten und über 5 Messungen prüfen ob das Gewicht nicht nur eine zufällige Schwankung war 
    int gewicht_Mittel = 0;
    for (int i = 0; i < 5; i++){                    //R.R  hier sollte man vileich noch prüfen ob ein gemessener Wert nicht ein Ausreisser war. Sonst wird das tara verfälscht.
      weight = int(SCALE_GETUNITS(SCALE_READS)) - tare;
      gewicht_Mittel += weight;
      delay(300);
    }
    weight = gewicht_Mittel / 5;                   //A.P.  Mittleres Gewicht über die 1,5s
    if (init_weight_f > 0) {
      angle = angle_marker ;                       //A.P. für Folgeglas optimalen Winkel vom Vorglas einstellen.
    }
    if (abs(weight) <= jartolerance) {
      if (init_weight_f <= 0) {
        weight = int(SCALE_GETUNITS(SCALE_READS)) - tare;  // A.P.
      }
      tare_jar = weight;
      //#if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
        dis.process_automatic(9);
      //#endif
      #if DEBUG_HM  >= 1
        Serial.print(" weight: ");                Serial.print(weight);
        Serial.print(" weight_before: ");         Serial.print(weight_before);
        Serial.print(" tareweight: ");            Serial.print(fquantity + correction + tare_jar + autocorrection_gr);
        Serial.print(" overfill_gr: ");           Serial.print(overfill_gr);
        Serial.print(" autocorrection_gr: ");     Serial.println(autocorrection_gr);
      #endif
      if (wait_befor_fill == 1 and stop_wait_befor_fill == 0) {   
        dis.process_automatic(6); //wait befor fill
        buzzer(BUZZER_SHORT);
        while (digitalRead(button_start_pin) == LOW and digitalRead(button_stop_pin) == LOW and servo_enabled == 0 and stop_wait_befor_fill == 0 and digitalRead(switch_betrieb_pin) == HIGH) {
          delay(1);
          if (digitalRead(button_start_pin) == HIGH) {
            while(digitalRead(button_start_pin) == HIGH) {
              delay(1);
            }
            servo_enabled = 1;
          }
          if (digitalRead(button_stop_pin) == HIGH) {
            while(digitalRead(button_stop_pin) == HIGH) {
              delay(1);
            }
            stop_wait_befor_fill = 1;
          }
        }
      }
      else {
        servo_enabled = 1;
      }
      collector_num = 0;
      full = false;
      counted = false;
      weight_old = -9999999;               //Damit die Gewichtsanzeige aktiviert wird
      buzzer(BUZZER_SHORT);
    }
  }
  target_weight = fquantity + correction + tare_jar + autocorrection_gr;
  // Anpassung des Autokorrektur-Werts
  if (autocorrection == 1) {                                       
    if (auto_enabled == 1 && servo_enabled == 0 && angle == angle_min + offset_angle && weight >= target_weight && collector_num <= 5) {
      full = true;                       
      if (weight == weight_before && collector_num < 5) { // wir wollen 5x das identische Gewicht sehen  
        collector_num++;
      } 
      else if (weight != weight_before) {               // sonst gewichtsänderung nachführen
        weight_before = weight;
        collector_num = 0;
      } 
      else if (collector_num == 5) {                        // gewicht ist 5x identisch, autokorrektur bestimmen
        autocorrection_gr = (fquantity + overfill_gr + tare_jar) - (weight - autocorrection_gr);
        if (correction + autocorrection_gr > overfill_gr) {   // Autokorrektur darf nicht überkorrigieren, max Füllmenge plus Kulanz
          autocorrection_gr = overfill_gr - correction;
          #if DEBUG_HM >= 1
            Serial.print("Autokorrektur begrenzt auf ");
            Serial.println(autocorrection_gr);
          #endif
        }
        buzzer(BUZZER_SUCCESS);
        collector_num++;                                      // Korrekturwert für diesen Durchlauf erreicht
        #if DREHTELLER == 1
          if (use_turntable == 1 and mode == MODE_AUTOMATIK) {
            turntable_jar_full_flag = true;
          }
        #endif
      }
      if (full == true && counted == false) {
        glaeser[fquantity_index].TripCount++;
        glaeser[fquantity_index].Count++;
        counted = true;
      }
      #if DEBUG_HM >= 1
        Serial.print("Nachtropfen:");
        Serial.print(" weight: ");                 Serial.print(weight);
        Serial.print(" weight_before: ");          Serial.print(weight_before);
        Serial.print(" collector_num: ");          Serial.print(collector_num);
        Serial.print(" autocorrection_gr: ");      Serial.println(autocorrection_gr);
        Serial.print(" counter trip: ");           Serial.print(glaeser[fquantity_index].TripCount); //Kud
        Serial.print(" counter: ");                Serial.println(glaeser[fquantity_index].Count); //Kud
      #endif
    }
  }
  // Glas ist teilweise gefüllt. Start wird über Start-Taster erzwungen
  if (auto_enabled == 1 && weight > 5 && force_servo_active == 1 && turntable_ok == true) {
    servo_enabled = 1;
    full = false;
    counted = false;
    #if DREHTELLER == 1
      if (use_turntable == 1 and turntable_init_check == false and mode == MODE_AUTOMATIK) {
        collector_num = 0;
      }
    #endif
    buzzer(BUZZER_SHORT);
  }
  if (servo_enabled == 1 && init_weight_f <= 0) { 
    angle = (angle_max[fullmode-1] * pos / 100);  // A.P.wird deaktiviert wenn intGewicht ist gleich 0 (automatische Durchflussgeschwindigkeit ist 0)
  }
  if (init_weight_f > 0) {
    if (servo_enabled == 1 && (target_weight - weight <= finedos_weight[fullmode-1]) && ((angle_max[fullmode-1]*pos / 100) * (target_weight-weight) / finedos_weight[fullmode-1] <= angle)) {
      angle = ((angle_max[fullmode-1]*pos / 100) * ((target_weight-weight) / finedos_weight[fullmode-1]));    // AP nur wenn der hier berechnete Winkel kleiner dem aktuellen Winkel ist wird das Ventil zugefahren (ansonsten geht es bei kleineren Öffnungsgraden auf
    }
  }
  else {
    if (servo_enabled == 1 && (target_weight - weight <= finedos_weight[fullmode-1])) {
      angle = ((angle_max[fullmode-1]*pos / 100) * ((target_weight-weight) / finedos_weight[fullmode-1]));
    }
  }
  if (servo_enabled == 1 && angle <= angle_fine[fullmode-1]) { 
    angle = angle_fine[fullmode-1];
  }
  if (init_weight_f > 0) {
    // A.P.  Es wird ein gleitender Mittelwert über 3 Messungen durchgeführt.
    float RunningAverageGewicht;
    int RawGewicht = weight-weight_old2;
    average.add(RawGewicht);
    RunningAverageGewicht = (average.getAvg());
    //Optimierung der >Durchflussgeschwindigkeit  A.P.
    if (loop1_c < loop1){ loop1_c = loop1_c + 1;}
    else if (servo_enabled == 1 && ((RunningAverageGewicht + init_weight_f_D) > init_weight_f) && (init_weight_f > 0) && ((target_weight - weight) > finedos_weight[fullmode-1])) {
      angle = angle -2;
      angle_marker  = angle; // A.P. Merkt sich den letzten optimalen Winkel des Servos.
      loop1_c =0;
    }    //A.P.  Automatische Geschwindigkeit des Abfüllvorgangs
    else if (servo_enabled == 1 && ((RunningAverageGewicht - init_weight_f_D) < init_weight_f) && (init_weight_f > 0) && ((target_weight - weight) > finedos_weight[fullmode-1])) {
      angle = angle +2;
      if (angle >= (angle_max[fullmode-1]*pos/100)) {
        angle = (angle_max[fullmode-1]*pos/100);
        angle_marker  = angle; // A.P. Merkt sich den letzten optimalen Winkel des Servos.
      }
      loop1_c =0;
    }
    else {loop1_c =0;}
    weight_old2 = weight;
  }
  // Glas ist voll
  if (servo_enabled == 1 && weight >= target_weight) {
    angle = angle_min + offset_angle;
    servo_enabled = 0;
    if (counted == false) {
      glaeser[fquantity_index].TripCount++;
      glaeser[fquantity_index].Count++;
      counted = true;
    }
    if (autostart != 1)       // autostart ist nicht aktiv, kein weiterer Start
      auto_enabled = 0;
    if (autocorrection == 1)  // autokorrektur, gewicht merken
      weight_before = weight;
    buzzer(BUZZER_SHORT);
  }   
  // Glas entfernt -> Servo schliessen zusätzliche Abfrage
  if (angle > (angle_min + offset_angle)) {
    weight = int(SCALE_GETUNITS(SCALE_READS)) - tare; 
    if (weight < -20) { 
      delay(200);  // A.P.  Um einen Reset des ESP zu verhindern wenn er kurz nach öffnen geschlossen wird.
      angle      = angle_min + offset_angle;
    }
  }
  servo.write(squee_tap_left ? 180 - angle : angle);
  #if DEBUG_HM >= 4
    Serial.print("Automatic:");  
    Serial.print(" weight: ");             Serial.print(weight);
    Serial.print(" angle: ");              Serial.print(angle);
    Serial.print(" Time ");                Serial.print(millis() - scaletime);
    Serial.print(" fill quantity: ");      Serial.print(fquantity);
    Serial.print(" correction: ");         Serial.print(correction);
    Serial.print(" tare_jar:");            Serial.print(tare_jar);
    Serial.print(" autocorrection_gr: ");  Serial.print(autocorrection_gr);
    Serial.print(" target_weight ");       Serial.print(target_weight);
    Serial.print(" force_servo_active: "); Serial.print(force_servo_active);
    Serial.print(" servo_enabled ");       Serial.print(servo_enabled);
    Serial.print(" auto_enabled ");        Serial.println(auto_enabled);
  #endif
  if (ina219_installed and (current_servo > 0 or show_current == 1)) {
    y_offset_ina = 4;
  }
  //#if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
  //  dis.process_automatic_main_oled(rotary_select, SW_KORREKTUR, SW_FLUSS, SW_MENU, full, y_offset_ina);
  //#endif
  //#if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
  //  dis.process_automatic_main_tft(rotary_select, SW_KORREKTUR, SW_FLUSS, SW_MENU);
  //#endif
  dis.process_automatic(7); //Main
  //Turntable
  #if DREHTELLER == 1
    if (use_turntable == 1 and stop_button_used == true and mode == MODE_AUTOMATIK) {
      turntable_moving = false;
      collector_num = 0;
      if (use_turntable == 1) {
        esp_now_msg_recived = false;
        memset(&myReceivedMessage, 0, sizeof(myReceivedMessage));
        esp_now_msg_recived = false;
        strcpy(myMessageToBeSent.text, "stop");
        espnow_send_data();
        time = millis();
        while (!esp_now_msg_recived and millis() - time <= 1000  and strcmp(myReceivedMessage.text, "") == 0) {
          delay(10);
        }
      }
      stop_button_used = false;
    }
    if (use_turntable == 1 and turntable_init_check == true and mode == MODE_AUTOMATIK) {
      esp_now_msg_recived = false;
      memset(&myReceivedMessage, 0, sizeof(myReceivedMessage));
      strcpy(myMessageToBeSent.text, "turn_off_stepper_init_check");
      espnow_send_data();
      time = millis();
      while (!esp_now_msg_recived and millis() - time <= 10000 and strcmp(myReceivedMessage.text, "") == 0) {
        delay(10);
      }
      if (strcmp(myReceivedMessage.text, "ok_off_init_check") == 0) {
        turntable_init_check = false;
      }
      #ifdef TURNTABLE_DEBUG
        Serial.print("turn_off_stepper_init_check - turntable_init_check = "); Serial.println(turntable_init_check);
      #endif
    }
    if ((use_turntable == 1 and tare > 0 and mode == MODE_AUTOMATIK and stop_button_close_dp == 1) or (use_turntable == 1 and stop_button_close_dp == 1 and drip_prodection == 0 and mode == MODE_AUTOMATIK)) {
      drip_prodection = false;
    }
    if ((use_turntable == 1 and auto_enabled == 1 and tare > 0 and mode == MODE_AUTOMATIK) or (use_turntable == 1 and drip_prodection == 0 and tare > 0 and mode == MODE_AUTOMATIK and stop_button_close_dp == 1)) {
      if (weight >= -20 and drip_prodection == 0 and turntable_moving == 0 and stop_button_close_dp == 0) {
        turntable_ok = true;
      } 
      else {
        turntable_ok = false;
      }
      //Glass ist auf der Waage
      if (weight >= -20 and drip_prodection == 1 and turntable_moving == 0 and weight <= target_weight and stop_button_close_dp == 0) {
        esp_now_msg_recived = false;
        memset(&myReceivedMessage, 0, sizeof(myReceivedMessage));
        strcpy(myMessageToBeSent.text, "open_drop_prodection");
        espnow_send_data();
        time = millis();
        while (!esp_now_msg_recived and millis() - time <= 10000 and strcmp(myReceivedMessage.text, "") == 0) {
          delay(10);
        }
        if (strcmp(myReceivedMessage.text, "ok_open_dp") == 0) {
          drip_prodection = false;
        }
      }
      //Glass wurde von der Waage entfernt
      else if (weight < -20 and drip_prodection == 0 and turntable_moving == 0) {
        close_drip_protection = true;
      }
      //Glass ist voll
      else if (weight >= target_weight and drip_prodection == 0 and turntable_moving == 0 and servo_enabled == 0 and collector_num > 5 and turntable_jar_full_flag == true) {
        close_drip_protection = true;
      }
      else if(stop_button_close_dp == true and drip_prodection == 0) {
        close_drip_protection = true;
      }
      if (close_drip_protection == true and drip_prodection == 0 and servo_enabled == 0) {
        time = millis();
        int update_display_time = 0;
        #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
          gfx->fillRect(70, 23, 250, 80, BACKGROUND);
          gfx->setFont(Punk_Mono_Bold_160_100);
          gfx->setCursor(75, 70);
          sprintf(output,"%s: ", CLOSE_DRIPPROTECTION[lingo]);
          gfx->print(output);
        #endif
        while (millis() - time <= waittime_close_dp * 1000 and digitalRead(switch_betrieb_pin) == HIGH) {
          if (millis() - time >= update_display_time) {
            #if DISPLAY_TYPE == 1 or DISPLAY_TYPE == 2 or DISPLAY_TYPE == 99
              u8g2.clearBuffer();
              u8g2.setFont(u8g2_font_courB08_tf);
              x_pos = CenterPosX(CLOSE_DRIPPROTECTION[lingo], 6, 128);
              u8g2.setCursor(x_pos,20);
              u8g2.print(CLOSE_DRIPPROTECTION[lingo]);
              u8g2.setFont(u8g2_font_courB12_tf);
              sprintf(output,"%i", waittime_close_dp - (update_display_time / 1000));
              x_pos = CenterPosX(output, 10, 128);
              u8g2.setCursor(x_pos,38);
              u8g2.print(output);
              u8g2.sendBuffer();
            #endif
            #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
              gfx->fillRect(277, 54, 43, 20, BACKGROUND);
              gfx->setCursor(278, 70);
              sprintf(output,"%3is", waittime_close_dp - (update_display_time / 1000));
              gfx->print(output);
            #endif
            update_display_time = update_display_time + 1000;
          }
          else {
            delay(10);
          }
        }
        if (digitalRead(switch_betrieb_pin) == HIGH and drip_prodection == false and servo_enabled == 0) {   //nur schliessen wenn im Automatik Modus
          memset(&myReceivedMessage, 0, sizeof(myReceivedMessage));
          esp_now_msg_recived = false;
          strcpy(myMessageToBeSent.text, "close_drop_prodection");
          espnow_send_data();
          time = millis();
          while (!esp_now_msg_recived and millis() - time <= 1000  and strcmp(myReceivedMessage.text, "") == 0) {
            delay(10);
          }
          if (strcmp(myReceivedMessage.text, "ok_close_dp") == 0) {
            drip_prodection = true;
          }
        }
        close_drip_protection = false;
        stop_button_close_dp = false;
        weight_old = -9999999;
      }
      //Move Turntable
      if ((weight < -20 and drip_prodection == 1 and turntable_moving == 0 and digitalRead(switch_betrieb_pin) == HIGH) or (turntable_jar_full_flag == true and drip_prodection == 1 and turntable_moving == 0 and digitalRead(switch_betrieb_pin) == HIGH)) {
        memset(&myReceivedMessage, 0, sizeof(myReceivedMessage));
        esp_now_msg_recived = false;
        turntable_moving = true;
        strcpy(myMessageToBeSent.text, "move_jar");
        espnow_send_data();
        //Wird nicht implementiert für den OLED Display
        #if DISPLAY_TYPE == 1 or DISPLAY_TYPE == 2 or DISPLAY_TYPE == 99
          u8g2.clearBuffer();
          u8g2.setFont(u8g2_font_courB08_tf);
          x_pos = CenterPosX(MOVE_JAR[lingo], 6, 128);
          u8g2.setCursor(x_pos,25);
          u8g2.print(MOVE_JAR[lingo]);
          u8g2.sendBuffer();
        #endif
        #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
          gfx->fillRect(70, 23, 250, 80, BACKGROUND);
          gfx->setFont(Punk_Mono_Bold_240_150);
          gfx->setCursor(80, 70);
          gfx->print(MOVE_JAR[lingo]);
        #endif
      }
      else if (drip_prodection == 1 and turntable_moving == 1 and esp_now_msg_recived == true and digitalRead(switch_betrieb_pin) == HIGH) {
        esp_now_msg_recived = false;
        if (strcmp(myReceivedMessage.text, "ok_move_jar") == 0) {
          turntable_moving = false;
          turntable_jar_full_flag = false;
        }
        memset(&myReceivedMessage, 0, sizeof(myReceivedMessage));
      }
    }
  #endif
  //end Turntable
  if (alarm_overcurrent) {i = 1;}
  while (i > 0) {
    inawatchdog = 0;                    //schalte die kontiunirliche INA Messung aus
    //Servo ist zu
    if (servo.read() <= angle_min  + offset_angle and offset_angle < 3) {
      while(offset_angle < 3 and current_servo < current_mA) {
        offset_angle = offset_angle + 1;
        servo.write(squee_tap_left ? 180 - angle_min + offset_angle : angle_min + offset_angle);
        current_mA = GetCurrent(10);
        delay(1000);  //really a delay in the while. next code to fix :-)
      }
      alarm_overcurrent = 0;
    }
    i = 0;
    inawatchdog = 1;
  }
  pref.setPreferences();
}

void processManualmode(void) {
  i = 0;
  int y_offset = 0;
  if (mode != MODE_HANDBETRIEB) {
    mode = MODE_HANDBETRIEB;
    angle = angle_min;          // Hahn schliessen
    servo_enabled = 0;              // Servo-Betrieb aus
    servo.write(squee_tap_left ? 180 - angle : angle);
    rotary_select = SW_WINKEL;
    tare = 0;
    offset_angle = 0;            // Offset vom Winkel wird auf 0 gestellt
    draw_frame = true;
  }
  pos = getRotariesValue(SW_WINKEL);
  weight = SCALE_GETUNITS(SCALE_READS) - tare;
  if ((digitalRead(button_start_pin)) == HIGH) {
    servo_enabled = 1;
  }
  if ((digitalRead(button_stop_pin)) == HIGH) {
    servo_enabled = 0;
  }
  if ((digitalRead(outputSW)) == LOW) {
      tare = SCALE_GETUNITS(SCALE_READS);
  }
  if (servo_enabled == 1) {
    angle = ((angle_max[fullmode-1] * pos) / 100);
  }
  else { 
    angle = angle_min + offset_angle;
  }
  angle = constrain(angle, angle_min + offset_angle, angle_max[fullmode-1]);
  servo.write(squee_tap_left ? 180 - angle : angle);
  if (ina219_installed and (current_servo > 0 or show_current == 1)) {
    y_offset = 4;
  }
  #if DEBUG_HM >= 4
    Serial.print("Handbetrieb:");  
    Serial.print(" weight ");      Serial.print(weight);
    Serial.print(" angle ");       Serial.print(angle);
    Serial.print(" servo_enabled "); Serial.println(servo_enabled);
  #endif
  dis.process_manualmode();
  if (alarm_overcurrent) {i = 1;}
  while (i > 0) {
    inawatchdog = 0;                    //schalte die kontiunirliche INA Messung aus
    //Servo ist zu
    if (servo.read() <= angle_min  + offset_angle and offset_angle < 3) {
      while(offset_angle < 3 and current_servo < current_mA) {
        offset_angle = offset_angle + 1;
        servo.write(squee_tap_left ? 180 - angle_min + offset_angle : angle_min + offset_angle);
        current_mA = GetCurrent(10);
        delay(1000);
      }
    }
    i = 0;
    inawatchdog = 1;
    alarm_overcurrent = 0;
  }
  pref.setPreferences();
}

void setup() {
  // enable internal pull downs for digital inputs 
  pinMode(button_start_pin, INPUT_PULLDOWN);
  pinMode(button_stop_pin, INPUT_PULLDOWN);
  pinMode(switch_betrieb_pin, INPUT_PULLDOWN);
  pinMode(switch_setup_pin, INPUT_PULLDOWN);
  pinMode(outputSW, INPUT_PULLUP);
  // Buzzer
  pinMode(buzzer_pin, OUTPUT);
  pinMode(led_pin, OUTPUT);
  // additional pin setup for hardwarelevel 2
  #if HARDWARE_LEVEL == 2 or HARDWARE_LEVEL == 3
    pinMode (switch_vcc_pin, OUTPUT);
    pinMode (button_start_vcc_pin, OUTPUT);
    pinMode (button_stop_vcc_pin, OUTPUT);
    digitalWrite (switch_vcc_pin, HIGH); 
    digitalWrite (button_start_vcc_pin, HIGH); 
    digitalWrite (button_stop_vcc_pin, HIGH);
  #endif
  //Calibrations weight
  #if USER == 2
    cali_weight = 2000;                             // choosen weight for calibration
  #elif USER == 3
    cali_weight = 500;                              // choosen weight for calibration
  #else
    cali_weight = 500;                              // choosen weight for calibration
  #endif
  Serial.begin(115200);
  while (!Serial) {}
  #if DEBUG_HM >= 1
    delay(1000);
    Serial.println("Hanimandl Start");
  #endif
  unsigned long button_millis;
  // Panik prodzedure wenn das NSV zerschossen wurde
  if (digitalRead(outputSW) == LOW ) {
    button_millis = millis();
    while (digitalRead(outputSW) == LOW) {
      if (millis() - button_millis >= 1000 and millis() - button_millis <= 1100) {buzzer(BUZZER_START_UP);}
      delay(1);
    }
    nvs_flash_erase();    // erase the NVS partition and...
    nvs_flash_init();     // initialize the NVS partition.
    //Da machen wir gerade einen restart
    ESP.restart();
  }
  // rotary_scale zurücksetzen
  if (digitalRead(button_stop_pin) == HIGH) {
    button_millis = millis();
    while (digitalRead(button_stop_pin) == HIGH) {
      if (millis() - button_millis >= 1000 and millis() - button_millis <= 1100) {buzzer(BUZZER_START_UP);}
      delay(1);
    }
    pref.set_rotary_scale_Preferences(0);
  }
  // Get Preferences
  pref.getPreferences();
  // Start Display
  dis.begin();
  // Set rotary scale when the varuable rotary_scale = 0
  while (rotary_scale == 0) {
    dis.set_rotary_scale();
    if (digitalRead(button_start_pin) == HIGH and digitalRead(button_stop_pin) == LOW) {
      button_millis = millis();
      while (digitalRead(button_start_pin) == HIGH) {
        if (millis() - button_millis >= 1000 and millis() - button_millis <= 1100) {buzzer(BUZZER_START_UP);}
        delay(1);
      }
      pref.set_rotary_scale_Preferences(1);
    }
    else if (digitalRead(button_start_pin) == LOW and digitalRead(button_stop_pin) == HIGH) {
      button_millis = millis();
      while (digitalRead(button_stop_pin) == HIGH) {
        if (millis() - button_millis >= 1000 and millis() - button_millis <= 1100) {buzzer(BUZZER_START_UP);}
        delay(1);
      }
      pref.set_rotary_scale_Preferences(2);
    }
  }
  dis.clear();
  // Try to initialize the INA219
  #if HARDWARE_LEVEL == 2
    I2C_2.begin(20, 19);    //SDA (20), SDL (19)
  #endif
  #if HARDWARE_LEVEL == 2
    if (ina219.begin(&I2C_2)) { 
      ina219_installed = 1;
      #if DEBUG_HM >= 1
        Serial.println("INA219 chip gefunden");
      #endif
    }
  #else
    if (ina219.begin()) { 
      ina219_installed = 1;
      #if DEBUG_HM >= 1
        Serial.println("INA219 chip gefunden");
      #endif
    }
  #endif
  else {
    current_servo = 0;                              // ignore INA wenn keiner gefunden wird
    #if DEBUG_HM >= 1
      Serial.println("INA219 chip nicht gefunden");
    #endif
  }
  // Rotary
  attachInterrupt(outputSW, isr1, FALLING);
  pinMode(outputA,INPUT);
  pinMode(outputB,INPUT);
  attachInterrupt(outputA, isr2, CHANGE);
  // short delay to let chip power up
  delay (100);
  // Servo initialisieren und schliessen
  if (servo_expanded == 1) {
    servo.attach(servo_pin,  750, 2500); // erweiterte Initialisierung, steuert nicht jeden Servo an
    servo.setPeriodHertz(100);
  }
  else{
    servo.attach(servo_pin, 1000, 2000); // default Werte. Achtung, steuert den Nullpunkt weniger weit aus!
  } 
  servo.write(squee_tap_left ? 180 - angle_min : angle_min);
  // Waage erkennen - machen wir vor dem Boot-Screen, dann hat sie 3 Sekunden Zeit zum aufwärmen
  scale.begin(hx711_dt_pin, hx711_sck_pin);
  if (scale.wait_ready_timeout(1000)) {               // Waage angeschlossen? :Roli - Ist immer null wenn kein HX711 angeschlossen ist
    scale.power_up();
    if (scale.read() != 0) {                          // Roli - Wenn 0, nehme ich an, das kein HX711 angeschlossen ist
      scale_present = 1;
      #if DEBUG_HM >= 1
        Serial.println("Waage erkannt");
      #endif
    }
  }
  // Boot Screen
  if (showlogo) {
    dis.print_logo();
    if (showcredits) {delay(3000);}                 // Wenn credits nicht angezeigt werden, können wir gerade weiter machen mit dem booten
  }
  if (showcredits) {
    buzzer(BUZZER_SHORT);
    dis.print_credits();   
    delay(2500);
  }
  // Kanal Nummer suchen wenn OTA und Drehteller gleichzeitig benützt werden können
  #if (OTA == 1 and DREHTELLER == 1)
    ota.ota_search_chanel();
  #endif
  // Setup der Waage, Skalierungsfaktor setzen
  if (scale_present ==1) {                           // Waage angeschlossen?
    if (factor == 0) {                               // Vorhanden aber nicht kalibriert
      dis.not_calibrated();
      buzzer(BUZZER_ERROR);
      #if DEBUG_HM >= 1
        Serial.println("Waage nicht kalibriert!");
      #endif
      delay(2000);
    }
    else {                                          // Tara und Skalierung setzen
      scale.set_scale(factor);
      scale.set_offset(long(weight_empty));
      #if DEBUG_HM >= 1
        Serial.println("Waage initialisiert");
      #endif
    }
  }
  else {                                            // Keine Waage angeschlossen
    dis.no_scale();
    buzzer(BUZZER_ERROR);
    #if DEBUG_HM >= 1
      Serial.println("Keine Waage!");
    #endif
    delay(2000);
  }
  // initiale Kalibrierung des Leergewichts wegen Temperaturschwankungen
  // Falls mehr als 20g Abweichung steht vermutlich etwas auf der Waage.
  if (scale_present == 1) {
    weight = SCALE_GETUNITS(SCALE_READS);
    if ((weight > -20) && (weight < 20)) {
      scale.tare(10);
      buzzer(BUZZER_SUCCESS);
      #if DEBUG_HM >= 1
        Serial.print("Tara angepasst um: ");
        Serial.println(weight);
      #endif
    }
    else if (factor != 0) {
      dis.empty_the_scale();
      #if DEBUG_HM >= 1
        Serial.print("Gewicht auf der Waage: ");
        Serial.println(weight);
      #endif
      delay(5000);
      // Neuer Versuch, falls Gewicht entfernt wurde
      weight = SCALE_GETUNITS(SCALE_READS);
      if ((weight > -20) && (weight < 20)) {
        scale.tare(10);
        buzzer(BUZZER_SUCCESS);
        #if DEBUG_HM >= 1
          Serial.print("Tara angepasst um: ");
          Serial.println(weight);
        #endif
      }
      else {    // Warnton ausgeben
        buzzer(BUZZER_LONG);
      }
    }
  }
  // die vier Datenstrukturen des Rotaries initialisieren
  initRotaries(SW_WINKEL,    0,   0, 100, 5);     // Winkel
  initRotaries(SW_KORREKTUR, 0, -90,  20, 1);     // Korrektur
  initRotaries(SW_FLUSS,     0,   0,  99, 1);     // Fluss
  initRotaries(SW_MENU,      0,   0,   7, 1);     // Menuauswahlen
  // Parameter aus den Preferences für den Rotary Encoder setzen
  setRotariesValue(SW_WINKEL,    pos);   
  setRotariesValue(SW_KORREKTUR, correction);
  setRotariesValue(SW_FLUSS,     init_weight_f);
  setRotariesValue(SW_MENU,      fquantity_index);
  //Drehteller
  #if DREHTELLER == 1
    WiFi.mode(WIFI_STA);
    esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
    #if CHANGE_MAC_ADDRESS_HM == 1
      uint8_t newMACAddress[] = {0x74, 0x00, 0x00, 0x00, 0x00, 0x01};
      esp_err_t err = esp_wifi_set_mac(WIFI_IF_STA, &newMACAddress[0]);
      #ifdef DEBUG
        if (err == ESP_OK) {Serial.println("Success changing Mac Address");}
        else {Serial.println("Fail changing Mac Address");}
      #endif
    #endif
    if (esp_now_init() == ESP_OK) {
      esp_now_ini =true;
      #if DEBUG_HM >= 1
        Serial.println("ESPNow Init success");
      #endif
    }
    else {
      esp_now_ini =false;
      #if DEBUG_HM >= 1
        Serial.println("ESPNow Init fail");
      #endif
    }
    esp_now_register_send_cb(messageSent);  
    esp_now_register_recv_cb(messageReceived); 
    memcpy(peerInfo.peer_addr, MacAdressTurntable, 6);
    peerInfo.channel = channel;
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      esp_now_ini =false;
      #if DEBUG_HM >= 1
        Serial.println("Failed to add peer");
      #endif
    }
    if (use_turntable == 1) {
      unsigned long turntable_millis = millis();
      esp_now_msg_recived = false;
      strcpy(myMessageToBeSent.text, "close_drop_prodection");
      espnow_send_data();
      while (millis() - turntable_millis < 1000 and esp_now_msg_recived == false) {
        delay(10);
      }
    }
  #endif
}

void loop() {
  #if DREHTELLER == 1
    if (use_turntable == 1 and turntable_init_check == false and ((digitalRead(switch_setup_pin) == HIGH) or (digitalRead(switch_betrieb_pin) == LOW and digitalRead(switch_setup_pin) == LOW))) {
      esp_now_msg_recived = false;
      memset(&myReceivedMessage, 0, sizeof(myReceivedMessage));
      strcpy(myMessageToBeSent.text, "turn_on_stepper_init_check");
      espnow_send_data();
      unsigned long time = millis();
      while (!esp_now_msg_recived and millis() - time <= 10000 and strcmp(myReceivedMessage.text, "") == 0) {
        delay(10);
      }
      if (strcmp(myReceivedMessage.text, "ok_on_init_check") == 0) {
        turntable_init_check = true;
      }
    }
  #endif
  //INA219 Messung
  if (ina219_installed and inawatchdog == 1 and (current_servo > 0 or show_current == 1) and (mode == MODE_HANDBETRIEB or mode == MODE_AUTOMATIK)) {
    ina219_measurement();
  }
  // Setup Menu 
  if ((digitalRead(switch_setup_pin)) == HIGH) {
    processSetup();
  }
  // Automatik-Betrieb 
  else if ((digitalRead(switch_betrieb_pin)) == HIGH) {
    processAutomatic();
  }
  // Handbetrieb 
  else if ((digitalRead(switch_betrieb_pin) == LOW) && (digitalRead(switch_setup_pin) == LOW)) {
    processManualmode();
  }
}

// Wir nutzen einen aktiven Summer, damit entfällt die tone Library, die sich sowieso mit dem Servo beisst.
void buzzer(byte type) {
  if ((buzzermode == 1 or ledmode == 1) or type == BUZZER_START_UP) {
    switch (type) {
      case BUZZER_SHORT: //short
        if (buzzermode == 1) {digitalWrite(buzzer_pin,HIGH);}
        if (ledmode == 1) {digitalWrite(led_pin,HIGH);}
        if (buzzermode == 1 or ledmode == 1) {delay(100);}
        if (buzzermode == 1) {digitalWrite(buzzer_pin,LOW);}
        if (ledmode == 1) {digitalWrite(led_pin,LOW);}
        break;
      case BUZZER_LONG: //long
        if (buzzermode == 1) {digitalWrite(buzzer_pin,HIGH);}
        if (ledmode == 1) {digitalWrite(led_pin,HIGH);}
        if (buzzermode == 1 or ledmode == 1) {delay(500);}
        if (buzzermode == 1) {digitalWrite(buzzer_pin,LOW);}
        if (ledmode == 1) {digitalWrite(led_pin,LOW);}
        break;
      case BUZZER_SUCCESS: //success
        if (buzzermode == 1) {digitalWrite(buzzer_pin,HIGH);}
        if (ledmode == 1) {digitalWrite(led_pin,HIGH);}
        if (buzzermode == 1 or ledmode == 1) {delay(100);}
        if (buzzermode == 1) {digitalWrite(buzzer_pin,LOW);}
        if (ledmode == 1) {digitalWrite(led_pin,LOW);}
        if (buzzermode == 1 or ledmode == 1) {delay(100);}
        if (buzzermode == 1) {digitalWrite(buzzer_pin,HIGH);}
        if (ledmode == 1) {digitalWrite(led_pin,HIGH);}
        if (buzzermode == 1 or ledmode == 1) {delay(100);}
        if (buzzermode == 1) {digitalWrite(buzzer_pin,LOW);}
        if (ledmode == 1) {digitalWrite(led_pin,LOW);}
        if (buzzermode == 1 or ledmode == 1) {delay(100);}
        if (buzzermode == 1) {digitalWrite(buzzer_pin,HIGH);}
        if (ledmode == 1) {digitalWrite(led_pin,HIGH);}
        if (buzzermode == 1 or ledmode == 1) {delay(100);}
        if (buzzermode == 1) {digitalWrite(buzzer_pin,LOW);}
        if (ledmode == 1) {digitalWrite(led_pin,LOW);}
        break;
      case BUZZER_ERROR: //error
        if (buzzermode == 1) {digitalWrite(buzzer_pin,HIGH);}
        if (ledmode == 1) {digitalWrite(led_pin,HIGH);}
        if (buzzermode == 1 or ledmode == 1) {delay(1500);}
        if (buzzermode == 1) {digitalWrite(buzzer_pin,LOW);}
        if (ledmode == 1) {digitalWrite(led_pin,LOW);}
        break;
      case BUZZER_START_UP: //start up
        digitalWrite(buzzer_pin,HIGH);
        delay(200);
        digitalWrite(buzzer_pin,LOW);
        break;
    }
  }
}

// Supportfunktionen für stufenweise Gewichtsverstellung
int step2weight(int step) {
  int sum = 0;
  if (step > 210) {sum += (step-210)*1000; step -= (step-210);}
  if (step > 200) {sum += (step-200)* 500; step -= (step-200);}  
  if (step > 160) {sum += (step-160)* 100; step -= (step-160);}
  if (step > 140) {sum += (step-140)*  25; step -= (step-140);}
  if (step >  50) {sum += (step- 50)*   5; step -= (step- 50);}
  sum += step;  
  return sum;
}

int weight2step(int sum) {
  int step = 0;
  if (sum > 10000) {step += (sum-10000)/1000; sum -= (sum-10000);}
  if (sum >  5000) {step += (sum-5000)/500;   sum -= (sum-5000); }
  if (sum >  1000) {step += (sum-1000)/100;   sum -= (sum-1000); }
  if (sum >   500) {step += (sum-500)/25;     sum -= (sum-500);  }
  if (sum >    50) {step += (sum-50)/5;       sum -= (sum-50);   }
  step += sum;
  return step;
}

int step2steps(int step) {
  int res = 0;
  if (step > 40) {res += (step-40)*100; step -= (step- 40);}
  if (step > 24) {res += (step-24)*25;  step -= (step- 24);}
  if (step >  5) {res += (step- 5)*5;   step -= (step- 5); }
  res += step;
  return res;
}

int steps2step(int sum) {
  int res = 0;
  if (sum > 500) {res += (sum-500)/100; sum -= (sum-500);}
  if (sum > 100) {res += (sum-100)/25;  sum -= (sum-100);}
  if (sum >   5) {res += (sum-  5)/5;   sum -= (sum-5);  }
  res += sum;
  return res;
}

int step2loadcell(int step) { 
  int res = 0;
  if      (step == 0)  {res = 1000;}
  else if (step == 1)  {res = 2000;}
  else if (step == 2)  {res = 3000;}
  else if (step == 3)  {res = 5000;}
  else if (step == 4)  {res = 10000;}
  else if (step == 5)  {res = 20000;}
  else if (step == 6)  {res = 50000;}
  return res;
}

int loadcell2step(int weight) { 
  int res = 0;
  if      (weight == 1000)   {res = 0;}
  else if (weight == 2000)   {res = 1;}
  else if (weight == 3000)   {res = 2;}
  else if (weight == 5000)   {res = 3;}
  else if (weight == 10000)  {res = 4;}
  else if (weight == 20000)  {res = 5;}
  else if (weight == 50000)  {res = 6;}
  return res;
}

int step2calweight(int step) { 
  int res = 0;
  if      (step == 0)   {res = 500;}
  else if (step == 1)   {res = 1000;}
  else if (step == 2)   {res = 2000;}
  else if (step == 3)   {res = 3000;}
  else if (step == 4)   {res = 5000;}
  else if (step == 5)   {res = 10000;}
  else if (step == 6)   {res = 15000;}
  else if (step == 7)   {res = 20000;}
  else if (step == 8)   {res = 25000;}
  else if (step == 9)   {res = 30000;}
  else if (step == 10)  {res = 35000;}
  else if (step == 11)  {res = 40000;}
  else if (step == 12)  {res = 45000;}
  return res;
}

int calweight2step(int weight) { 
  int res = 0;
  if      (weight == 500)    {res = 0;}
  else if (weight == 1000)   {res = 1;}
  else if (weight == 2000)   {res = 2;}
  else if (weight == 3000)   {res = 3;}
  else if (weight == 5000)   {res = 4;}
  else if (weight == 10000)  {res = 5;}
  else if (weight == 15000)  {res = 6;}
  else if (weight == 20000)  {res = 7;}
  else if (weight == 25000)  {res = 8;}
  else if (weight == 30000)  {res = 9;}
  else if (weight == 35000)  {res = 10;}
  else if (weight == 40000)  {res = 11;}
  else if (weight == 45000)  {res = 12;}
  return res;
}

//Sub function for INA219
int GetCurrent(int count) {
  int res = 0;
  for (int x = 0; x < count; x++) {
    res = res + ina219.getCurrent_mA();
  }
  if (count > 0) {
    res = res / count;
  }
  return res;
}

void ina219_measurement() {
  if (current_mA > current_servo and current_servo > 0) {
    if (last_overcurrenttime == 0) {
      last_overcurrenttime = millis();
    }
    else if (millis() - last_overcurrenttime >= overcurrenttime) {
      last_overcurrenttime = 0;
      alarm_overcurrent = 1;
    }
  }
  else {
    last_overcurrenttime = 0;
  }
  if (millis() - last_ina219_measurement >= updatetime_ina219) {
    last_ina219_measurement = millis();
    current_mA = GetCurrent(10);
  }
}

//Subfunktion für ERP NOW
void espnow_send_data() {
  #if DREHTELLER == 1
    esp_err_t result = esp_now_send(MacAdressTurntable, (uint8_t *) &myMessageToBeSent, sizeof(myMessageToBeSent));
    if (result != ESP_OK) {
      bool esp_now_send_error = true; 
      #if ESP_NOW_DEBUG  >= 1
        Serial.print("Sending error: Text: "); Serial.print(myMessageToBeSent.text); Serial.print(" - Value: "); Serial.println(myMessageToBeSent.value);
      #endif
    }
    else {
      bool esp_now_send_error = false;
      #if ESP_NOW_DEBUG  >= 1
        Serial.print("Sending sucsess: "); Serial.print(myMessageToBeSent.text); Serial.print(" - Value: "); Serial.println(myMessageToBeSent.value);
      #endif
    }
    memset(&myMessageToBeSent, 0, sizeof(myMessageToBeSent));
  #endif
}