//im manuellen modus noch die gewichtsanzeige anpassen für mit ina oder ohne (warschwidlich der y_offtet wie beim automatick auf y_offset_ina setzen)

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
#include <U8g2lib.h>                /* aus dem Bibliotheksverwalter */
#include <HX711.h>                  /* aus dem Bibliotheksverwalter */
#include <ESP32Servo.h>             /* aus dem Bibliotheksverwalter */
#include <Adafruit_INA219.h>        /* aus dem Bibliotheksverwalter */
#include <Preferences.h>            /* aus dem BSP von expressif, wird verfügbar wenn das richtige Board ausgewählt ist */
#include <nvs_flash.h>              /* aus dem BSP von expressif, wird verfügbar wenn das richtige Board ausgewählt ist */
#include <TFT_eSPI.h>

#include "HardwareLevel.h"
#include "variables.h"
#include "display.h"
#include "WebIF.h"


#if HARDWARE_LEVEL == 2
  TwoWire I2C_2 = TwoWire(1);
#endif

#if SCALE_TYP == 1
  #define MAXIMALGEWICHT 1500     // Maximales Gewicht für 2kg Wägezelle
#elif SCALE_TYP == 2
  #define MAXIMALGEWICHT 4500     // Maximales Gewicht für 5kg Wägezelle
#else
  #error Keine Wägezelle definiert
#endif

// Ansteuerung der Waage
#define SCALE_READS 2           // Parameter für hx711 Library. Messwert wird aus der Anzahl gemittelt
#define SCALE_GETUNITS(n)       round(scale.get_units(n))

// Ansteuerung Servo
#if QUETSCHHAHN_LINKS == 1
  #define SERVO_WRITE(n)     servo.write(180-n)
#else
  #define SERVO_WRITE(n)     servo.write(n)
#endif

// Rotary Encoder Taster zieht Pegel auf Low
#define SELECT_SW outputSW
#define SELECT_PEGEL LOW

// Betriebsmodi 
#define MODE_SETUP       0
#define MODE_AUTOMATIK   1
#define MODE_HANDBETRIEB 2

// Buzzer Sounds
#define BUZZER_SHORT   1
#define BUZZER_LONG    2
#define BUZZER_SUCCESS 3
#define BUZZER_ERROR   4

// INA 219
Adafruit_INA219 ina219;

// Display
HM_Display dis;

// HM_WEBIF
HM_WEBIF webif;


#if DISPLAY_TYPE == 1
  #if HARDWARE_LEVEL == 1
    U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* clock=*/ 22, /* data=*/ 21, /* reset=*/ U8X8_PIN_NONE);
  #elif HARDWARE_LEVEL == 2
    U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ 21, /* clock=*/ 18, /* data=*/ 17);
  #elif HARDWARE_LEVEL == 3
    U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ 21, /* clock=*/ 19, /* data=*/ 20);
  #endif
#elif DISPLAY_TYPE == 2
  U8G2_SSD1309_128X64_NONAME0_F_4W_SW_SPI u8g2(U8G2_R0, /* clock=*/ 18, /* data=*/ 23, /* cs=*/ 5, /* dc=*/ 13, /* reset=*/ 14);
#elif DISPLAY_TYPE == 3
  Arduino_DataBus *bus = new Arduino_HWSPI(13 /* DC */, 5 /* CS */);
  Arduino_GFX *gfx = new Arduino_ST7789(bus, 14 /* RST */, 3 /* rotation */);
#elif DISPLAY_TYPE == 99
  U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* clock=*/ 22, /* data=*/ 21, /* reset=*/ U8X8_PIN_NONE);
  Arduino_DataBus *bus = new Arduino_HWSPI(13 /* DC */, 5 /* CS */);
  Arduino_GFX *gfx = new Arduino_ST7789(bus, 14 /* RST */, 3 /* rotation */);
  //Arduino_GFX *gfx = new Arduino_ILI9341(bus, 14 /* RST */, 1 /* rotation */);
//#else
//  #error Kein Display definiert
#endif

//Sprache
#include "./Resources/resources.h"

//OTA
#if OTA == 1
  #include "ota.h"
  HM_OTA ota;
#endif

// Fonts
#if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
  #include "./Fonts/Punk_Mono_Bold_120_075.h"           //10 x 7
  #include "./Fonts/Punk_Mono_Bold_160_100.h"           //13 x 9
  #include "./Fonts/Punk_Mono_Bold_200_125.h"           //16 x 12
  #include "./Fonts/Punk_Mono_Bold_240_150.h"           //19 x 14
  #include "./Fonts/Punk_Mono_Bold_320_200.h"           //25 x 18
  #include "./Fonts/Punk_Mono_Bold_600_375.h"           //48 x 36
  #include "./Fonts/Punk_Mono_Thin_120_075.h"           //10 x 7
  #include "./Fonts/Punk_Mono_Thin_160_100.h"           //13 x 9
  #include "./Fonts/Punk_Mono_Thin_240_150.h"           //19 x 14
  #include "./Fonts/Icons_Start_Stop.h"                 //A=Start, B=Stop, M=Rahmen
  #include "./Fonts/Checkbox.h"                         //A=OK, B=nOK
#endif

//Logos
#if DISPLAY_TYPE == 1 or DISPLAY_TYPE == 2 or DISPLAY_TYPE == 99
  #if USER == 2
    #include "./Logos/LogoGeroldOLED.h"
  #elif USER == 3
    #include "./Logos/LogoRoliOLED.h"
  #else
    #include "./Logos/LogoBieneOLED.h"
  #endif
#endif


#if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
  #if USER == 2
    #include "./Logos/LogoGeroldTFT.h"
  #elif USER == 3
    #include "./Logos/LogoRoliTFT.h"
  #else
    #include "./Logos/LogoBieneTFT.h"
  #endif
#endif

Servo servo;
HX711 scale;
Preferences preferences;

// Denstrukturen für Rotary Encoder
struct rotary {                        
  int Value;
  int Minimum;
  int Maximum;
  int Step;
};
#define SW_WINKEL    0
#define SW_KORREKTUR 1
#define SW_FLUSS     2
#define SW_MENU      3
struct rotary rotaries[4];          // Werden im setup() initialisiert
int rotary_select = SW_WINKEL;

// Allgemeine Variablen
int i;                                              // general count variable

long preferences_chksum;                            // checksum to not write uncoherent prefs

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
    if ( mode == MODE_AUTOMATIK && servo_enabled == 0 ) { // nur im Automatik-Modus interessiert uns der Click
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
  int aState = digitalRead(outputA);          // Reads the "current" state of the outputA
  static int aLastState = 2;                  // reale Werte sind 0 und 1
  if (aState != aLastState) {     
    // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
    if (digitalRead(outputB) != aState) {
      rotaries[rotary_select].Value += rotaries[rotary_select].Step;
    }
    else {    // counter-clockwise
      rotaries[rotary_select].Value -= rotaries[rotary_select].Step;
    }
    rotaries[rotary_select].Value = constrain(
      rotaries[rotary_select].Value,
      rotaries[rotary_select].Minimum,
      rotaries[rotary_select].Maximum
    );
    #if DEBUG_HM >= 4
      Serial.print(" Rotary Value changed to ");
      Serial.println(getRotariesValue(rotary_select));
    #endif
  }
  aLastState = aState; // Updates the previous state of the outputA with the current state
}

//
// Skalierung des Rotaries für verschiedene Rotary Encoder
int getRotariesValue( int rotary_mode ) {
  return ((rotaries[rotary_mode].Value - (rotaries[rotary_mode].Value % (rotaries[rotary_mode].Step * ROTARY_SCALE))) / ROTARY_SCALE );
}

void setRotariesValue( int rotary_mode, int rotary_value ) {
  rotaries[rotary_mode].Value = rotary_value * ROTARY_SCALE;
}

void initRotaries( int rotary_mode, int rotary_value, int rotary_min, int rotary_max, int rotary_step ) {
  rotaries[rotary_mode].Value     = rotary_value * ROTARY_SCALE;
  rotaries[rotary_mode].Minimum   = rotary_min   * ROTARY_SCALE;
  rotaries[rotary_mode].Maximum   = rotary_max   * ROTARY_SCALE;
  rotaries[rotary_mode].Step      = rotary_step;
  #if DEBUG_HM >= 1
    Serial.print("initRotaries..."); 
    Serial.print(" Rotary Mode: ");  Serial.print(rotary_mode);
    Serial.print(" rotary_value: "); Serial.print(rotary_value);
    Serial.print(" Value: ");        Serial.print(rotaries[rotary_mode].Value);
    Serial.print(" Min: ");          Serial.print(rotaries[rotary_mode].Minimum);
    Serial.print(" Max: ");          Serial.print(rotaries[rotary_mode].Maximum);
    Serial.print(" Step: ");         Serial.print(rotaries[rotary_mode].Step);
    Serial.print(" Scale: ");        Serial.println(ROTARY_SCALE);
  #endif
}
// Ende Funktionen für den Rotary Encoder
//

void getPreferences(void) {
  char output[30];
  preferences.begin("EEPROM", false);                     // Parameter aus dem EEPROM lesen
  factor                = preferences.getFloat("factor", 0.0);  // falls das nicht gesetzt ist -> Waage ist nicht kalibriert
  pos                   = preferences.getUInt("pos", 0);
  weight_empty          = preferences.getUInt("weight_empty", 0); 
  correction            = preferences.getUInt("correction", 0);
  autostart             = preferences.getUInt("autostart", 0);
  autocorrection        = preferences.getUInt("autocorrection", 0);
  init_weight_f         = preferences.getUInt("init_weight_f", init_weight_f);  // bei 0 aus.A.P.
  overfill_gr           = preferences.getUInt("overfill_gr", 5);
  fquantity_index       = preferences.getUInt("fquantity_index", 3);
  angle_min             = preferences.getUInt("angle_min", angle_min);
  buzzermode            = preferences.getUInt("buzzermode", buzzermode);
  ledmode               = preferences.getUInt("ledmode", ledmode);
  showlogo              = preferences.getUInt("showlogo", showlogo);
  showcredits           = preferences.getUInt("showcredits", showcredits);
  cali_weight           = preferences.getUInt("cali_weight", cali_weight);
  jartolerance          = preferences.getUInt("jartolerance", jartolerance);
  current_servo         = preferences.getUInt("current_servo", current_servo);
  show_current          = preferences.getUInt("show_current", show_current);
  color_scheme          = preferences.getUInt("color_scheme", color_scheme);
  color_marker          = preferences.getUInt("color_marker", color_marker);
  use_turntable         = preferences.getUInt("use_turntable", use_turntable);
  lingo                 = preferences.getUInt("lingo", lingo);
  wait_befor_fill       = preferences.getUInt("wait_befor_fill", wait_befor_fill);
  fullmode              = preferences.getUInt("fullmode", fullmode);
  font_typ              = preferences.getUInt("font_typ", font_typ);
  menu_rotation         = preferences.getUInt("menu_rotation", menu_rotation);
  ssid                  = preferences.getString("ssid", ssid);
  password              = preferences.getString("password", password);
  preferences_chksum = factor + pos + weight_empty + correction + autostart + autocorrection + init_weight_f + 
                       overfill_gr + fquantity_index + angle_min + buzzermode + ledmode + 
                       showlogo + showcredits +  cali_weight + current_servo + jartolerance + show_current + 
                       color_scheme + color_marker + use_turntable + lingo + wait_befor_fill + fullmode + font_typ +
                       menu_rotation;
  #if USER == 2
    int ResetGewichte[] = {50,125,250,500,1000,};
    int ResetGlasTyp[] = {0,0,0,0,0,};
  #elif USER == 3
    int ResetGewichte[] = {250,500,0,0,0,};
    int ResetGlasTyp[] = {0,0,0,0,0,};
  #else
    int ResetGewichte[] = {125,250,250,500,500,};
    int ResetGlasTyp[] = {0,1,2,1,0,};
  #endif
  i = 0;
  while( i < 5) {
    sprintf(output, "Gewicht%d", i);
    glaeser[i].Gewicht = preferences.getInt(output, ResetGewichte[i]);
    preferences_chksum += glaeser[i].Gewicht;
    sprintf(output, "GlasTyp%d", i);
    glaeser[i].GlasTyp = preferences.getInt(output, ResetGlasTyp[i]);
    preferences_chksum += glaeser[i].GlasTyp;
    sprintf(output, "Tara%d", i);
    glaeser[i].Tare= preferences.getInt(output, -9999);
    preferences_chksum += glaeser[i].Tare;
    sprintf(output, "TripCount%d", i);
    glaeser[i].TripCount = preferences.getInt(output, 0);
    preferences_chksum += glaeser[i].TripCount;
    sprintf(output, "Count%d", i);
    glaeser[i].Count = preferences.getInt(output, 0);
    preferences_chksum += glaeser[i].Count;
    i++;
  }
  i = 0;
  while( i < 5) {
    sprintf(output, "angle_max%d", i);
    angle_max[i] = preferences.getInt(output, angle_max[i]);
    preferences_chksum += angle_max[i];
    sprintf(output, "angle_fine%d", i);
    angle_fine[i] = preferences.getInt(output, angle_fine[i]);
    preferences_chksum += angle_fine[i];
    sprintf(output, "finedos_weight%d", i);
    finedos_weight[i] = preferences.getFloat(output, finedos_weight[i]);
    preferences_chksum += finedos_weight[i];
    i++;
  }
  preferences.end();
  #if DEBUG_HM >= 1
    Serial.println("get Preferences:");
    Serial.print("pos = ");                   Serial.println(pos);
    Serial.print("factor = ");                Serial.println(factor);
    Serial.print("weight_empty = ");          Serial.println(weight_empty);
    Serial.print("correction = ");            Serial.println(correction);
    Serial.print("autostart = ");             Serial.println(autostart);
    Serial.print("autocorrection = ");        Serial.println(autocorrection);
    Serial.print("init_weight_f = ");         Serial.println(init_weight_f);  //A.P.
    Serial.print("overfill_gr = ");           Serial.println(overfill_gr);
    Serial.print("fquantity_index = ");       Serial.println(fquantity_index);
    Serial.print("angle_min = ");             Serial.println(angle_min);
    Serial.print("fullmode = ");              Serial.println(fullmode);
    Serial.print("buzzermode = ");            Serial.println(buzzermode);
    Serial.print("ledmode = ");               Serial.println(ledmode);
    Serial.print("showlogo = ");              Serial.println(showlogo);
    Serial.print("showcredits = ");           Serial.println(showcredits);
    Serial.print("current_servo = ");         Serial.println(current_servo);
    Serial.print("color_scheme = ");          Serial.println(color_scheme);
    Serial.print("color_marker = ");          Serial.println(color_marker);
    Serial.print("cali_weight = ");           Serial.println(cali_weight);
    Serial.print("jartolerance = ");          Serial.println(jartolerance);
    Serial.print("show_current = ");          Serial.println(show_current);
    Serial.print("use_turntable = ");         Serial.println(use_turntable);
    Serial.print("lingo = ");                 Serial.println(lingo);
    Serial.print("wait_befor_fill = ");       Serial.println(wait_befor_fill);
    Serial.print("font_typ = ");              Serial.println(font_typ);
    Serial.print("menu_rotation = ");         Serial.println(menu_rotation);
    Serial.print("ssid = ");                  Serial.println(ssid);
    Serial.print("password = ");              Serial.println(password);
    i = 0;
    while( i < 5 ) {
      sprintf(output, "Gewicht_%d = ", i);
      Serial.print(output);
      Serial.println(glaeser[i].Gewicht);
      sprintf(output, "GlasTyp_%d = ", i);
      Serial.print(output);         
      Serial.println(GlasTypArray[glaeser[i].GlasTyp]);
      sprintf(output, "Tara%d = ", i);
      Serial.print(output);         
      Serial.println(glaeser[i].Tare);
      i++;
    }
    i = 0;
    while( i < 5 ) {
      sprintf(output, "angle_max%d = ", i);
      Serial.print(output);         Serial.println(angle_max[i]);
      sprintf(output, "angle_fine%d = ", i);
      Serial.print(output);         Serial.println(angle_fine[i]);
      sprintf(output, "finedos_weight%d = ", i);
      Serial.print(output);         Serial.println(finedos_weight[i]);
      i++;
    }
    Serial.print("Checksumme:");    Serial.println(preferences_chksum);    
  #endif
}

void setPreferences(void) {
  char output[30];
  long preferences_newchksum;
  int winkel = getRotariesValue(SW_WINKEL);
  int i;
  preferences.begin("EEPROM", false);
  // Winkel-Einstellung separat behandeln, ändert sich häufig
  if ( winkel != preferences.getUInt("pos", 0) ) {
    preferences.putUInt("pos", winkel);
    #if DEBUG_HM >= 1
      Serial.print("winkel gespeichert: ");
      Serial.println(winkel);
    #endif
  }
  // Counter separat behandeln, ändert sich häufig
  for ( i=0 ; i < 5; i++ ) {
    sprintf(output, "TripCount%d", i);
    if (glaeser[i].TripCount != preferences.getInt(output, 0)) {
      preferences.putInt(output, glaeser[i].TripCount);
    }
    sprintf(output, "Count%d", i);
    if (glaeser[i].Count != preferences.getInt(output, 0)) {
      preferences.putInt(output, glaeser[i].Count);
    }
    #if DEBUG_HM >= 1
      Serial.print("Counter gespeichert: Index ");
      Serial.print(i);
      Serial.print(" Trip ");
      Serial.print(glaeser[fquantity_index].TripCount);
      Serial.print(" Gesamt ");
      Serial.println(glaeser[fquantity_index].Count);      
    #endif
  }
  // Den Rest machen wir gesammelt, das ist eher statisch
  preferences_newchksum = factor + weight_empty + correction + autostart + autocorrection + init_weight_f +
                          fquantity_index + angle_min + overfill_gr +
                          buzzermode + ledmode + showlogo + showcredits + current_servo + cali_weight + 
                          jartolerance + show_current + color_scheme + color_marker + use_turntable + lingo + 
                          wait_befor_fill + fullmode + font_typ + menu_rotation;
  i = 0;
  while( i < 5 ) {
    preferences_newchksum += glaeser[i].Gewicht;
    preferences_newchksum += glaeser[i].GlasTyp;
    preferences_newchksum += glaeser[i].Tare;
    i++;
  }
  i = 0;
  while( i < 5 ) {
    preferences_newchksum += angle_max[i];
    preferences_newchksum += angle_fine[i];
    preferences_newchksum += finedos_weight[i];
    i++;
  }
  if( preferences_newchksum == preferences_chksum ) {
    #if DEBUG_HM >= 1
      Serial.println("Preferences unverändert");
    #endif
    return;
  }
  preferences_chksum = preferences_newchksum;
  preferences.putFloat("factor", factor);
  preferences.putUInt("weight_empty", weight_empty);
  preferences.putUInt("correction", correction);
  preferences.putUInt("autostart", autostart);
  preferences.putUInt("autocorrection", autocorrection);
  preferences.putUInt("init_weight_f", init_weight_f); //A.P.
  preferences.putUInt("overfill_gr", overfill_gr);
  preferences.putUInt("angle_min", angle_min);
  //preferences.putUInt("angle_max", angle_max);
  //preferences.putUInt("angle_fine", angle_fine);
  preferences.putUInt("fullmode", fullmode);
  preferences.putUInt("fquantity_index", fquantity_index);
  preferences.putUInt("buzzermode", buzzermode);
  preferences.putUInt("ledmode", ledmode);
  preferences.putUInt("showlogo", showlogo);
  preferences.putUInt("showcredits", showcredits);
  preferences.putUInt("cali_weight", cali_weight);
  preferences.putUInt("jartolerance", jartolerance);
  preferences.putUInt("current_servo", current_servo);
  preferences.putUInt("show_current", show_current);
  preferences.putUInt("color_scheme", color_scheme);
  preferences.putUInt("color_marker", color_marker);
  preferences.putUInt("use_turntable", use_turntable);
  preferences.putUInt("lingo", lingo);
  preferences.putUInt("wait_befor_fill", wait_befor_fill);
  preferences.putUInt("font_typ", font_typ);
  preferences.putUInt("menu_rotation", menu_rotation);
  //preferences.putString("ssid", ssid);
  //preferences.putString("password", password);
  i = 0;
  while( i < 5 ) {
    sprintf(output, "Gewicht%d", i);
    preferences.putInt(output, glaeser[i].Gewicht);
    sprintf(output, "GlasTyp%d", i);
    preferences.putInt(output, glaeser[i].GlasTyp);  
    sprintf(output, "Tara%d", i);
    preferences.putInt(output, glaeser[i].Tare);
    i++;
  }
  i = 0;
  while( i < 5 ) {
    sprintf(output, "angle_max%d", i);
    preferences.putInt(output, angle_max[i]);
    sprintf(output, "angle_fine%d", i);
    preferences.putInt(output, angle_fine[i]);
    sprintf(output, "finedos_weight%d", i);
    preferences.putFloat(output, finedos_weight[i]);
    i++;
  }
  preferences.end();
  #if DEBUG_HM >= 1
    Serial.println("Set Preferences:");
    Serial.print("pos = ");                   Serial.println(winkel);
    Serial.print("factor = ");                Serial.println(factor);
    Serial.print("weight_empty = ");          Serial.println(weight_empty);
    Serial.print("correction = ");            Serial.println(correction);
    Serial.print("autostart = ");             Serial.println(autostart);
    Serial.print("init_weight_f = ");         Serial.println(init_weight_f); //A.P.
    Serial.print("autocorrection = ");        Serial.println(autocorrection);
    Serial.print("overfill_gr = ");           Serial.println(overfill_gr);
    Serial.print("fquantity_index = ");       Serial.println(fquantity_index);
    Serial.print("angle_min = ");             Serial.println(angle_min);
    //Serial.print("angle_max = ");           Serial.println(angle_max);
    //Serial.print("angle_fine = ");          Serial.println(angle_fine);
    Serial.print("fullmode = ");              Serial.println(fullmode);
    Serial.print("buzzermode = ");            Serial.println(buzzermode);
    Serial.print("ledmode = ");               Serial.println(ledmode);
    Serial.print("showlogo = ");              Serial.println(showlogo);
    Serial.print("showcredits = ");           Serial.println(showcredits);
    Serial.print("current_servo = ");         Serial.println(current_servo);
    Serial.print("cali_weight = ");           Serial.println(cali_weight);
    Serial.print("jartolerance = ");          Serial.println(jartolerance);
    Serial.print("show_current = ");          Serial.println(show_current);
    Serial.print("color_scheme = ");          Serial.println(color_scheme);
    Serial.print("color_marker = ");          Serial.println(color_marker);
    Serial.print("use_turntable = ");         Serial.println(use_turntable);
    Serial.print("lingo = ");                 Serial.println(lingo);
    Serial.print("wait_befor_fill = ");       Serial.println(wait_befor_fill);
    Serial.print("font_typ = ");              Serial.println(font_typ);
    Serial.print("menu_rotation = ");         Serial.println(menu_rotation);
    //Serial.print("ssid = ");                  Serial.println(ssid);
    //Serial.print("password = ");              Serial.println(password);
    i = 0;
    while( i < 5 ) {
      sprintf(output, "Gewicht%d = ", i);
      Serial.print(output);         Serial.println(glaeser[i].Gewicht);
      sprintf(output, "GlasTyp%d = ", i);
      Serial.print(output);         Serial.println(GlasTypArray[glaeser[i].GlasTyp]);
      sprintf(output, "Tara%d = ", i);
      Serial.print(output);         Serial.println(glaeser[i].Tare);
      i++;
    }
    i = 0;
    while( i < 5 ) {
      sprintf(output, "angle_max%d = ", i);
      Serial.print(output);         Serial.println(angle_max[i]);
      sprintf(output, "angle_fine%d = ", i);
      Serial.print(output);         Serial.println(angle_fine[i]);
      sprintf(output, "finedos_weight%d = ", i);
      Serial.print(output);         Serial.println(finedos_weight[i]);
      i++;
    }
  #endif
}

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
    #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
      dis.setup_trip_counter_oled(1);
    #endif
    #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
      dis.setup_trip_counter_tft(1);
    #endif
    if (digitalRead(SELECT_SW) == SELECT_PEGEL) {
      //verlasse Screen
      while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
        delay(1);
      }
      i = 0;
      #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
        change = true;
      #endif
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
    #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
      dis.setup_trip_counter_oled(2);
    #endif
    #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
      dis.setup_trip_counter_tft(2);
    #endif
    if (digitalRead(SELECT_SW) == SELECT_PEGEL) {
      //verlasse Screen
      while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
        delay(1);
      }
      i = 0;
      #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
        change = true;
      #endif
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
    #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
      dis.setup_trip_counter_oled(3);
    #endif
    #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
      dis.setup_trip_counter_tft(3);
    #endif
    if (digitalRead(SELECT_SW) == SELECT_PEGEL) {
      //verlasse Screen
      while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
        delay(1);
      }
      i = 0;
      #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
        change = true;
      #endif
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
      #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
        dis.setup_trip_counter_oled(4);
      #endif
      #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
        dis.setup_trip_counter_tft(4);
      #endif
      if (digitalRead(SELECT_SW) == SELECT_PEGEL) {
        #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
          dis.setup_trip_counter_oled_exit();
        #endif
        #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
          dis.setup_trip_counter_tft_exit();
        #endif
        if ( pos == 0) {
          j = 0;
          while ( j < 5  ) {
            glaeser[j].TripCount = 0;
            j++;
          }
          setPreferences();
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
    #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
      dis.setup_counter_oled(1);
    #endif
    #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
      dis.setup_counter_tft(1);
    #endif
    if (digitalRead(SELECT_SW) == SELECT_PEGEL) {
      //verlasse Screen
      while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
        delay(1);
      }
      i = 0;
      #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
        change = true;
      #endif
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
    #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
      dis.setup_counter_oled(2);
    #endif
    #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
      dis.setup_counter_tft(2);
    #endif
    if (digitalRead(SELECT_SW) == SELECT_PEGEL) {
      //verlasse Screen
      while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
        delay(1);
      }
      i = 0;
      #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
        change = true;
      #endif
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
    #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
      dis.setup_counter_oled(3);
    #endif
    #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
      dis.setup_counter_tft(3);
    #endif
    if ((digitalRead(SELECT_SW)) == SELECT_PEGEL) {
      //verlasse Screen
      while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
        delay(1);
      }
      i = 0;
      #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
        change = true;
      #endif
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
      #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
        dis.setup_counter_oled(4);
      #endif
      #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
        dis.setup_counter_tft(4);
      #endif
      if (digitalRead(SELECT_SW) == SELECT_PEGEL) {
        while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
          delay(1);
        }
        #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
          dis.setup_counter_oled_exit();
        #endif
        #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
          dis.setup_counter_tft_exit();
        #endif
        if ( pos == 0) {
          j = 0;
          while ( j < 5  ) {
            glaeser[j].Count = 0;
            glaeser[j].TripCount = 0;
            j++;
          }
          setPreferences();
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
    #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
      dis.setup_tare_oled(getRotariesValue(SW_MENU));
    #endif
    #if DISPLAY_TYPE ==993 or DISPLAY_TYPE == 999
      dis.setup_tare_tft(getRotariesValue(SW_MENU));
    #endif
  }
  mode = -1;
  delay(2000);
}

void setupCalibration(void) {
  float gewicht_raw;
  #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
    dis.setup_calibration_oled(1);
  #endif
  #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
    dis.setup_calibration_tft(1);
  #endif
  i = 1;
  while (i > 0) {
    if (digitalRead(button_stop_pin) == HIGH or digitalRead(switch_setup_pin) == LOW) {
      while (digitalRead(button_stop_pin) == HIGH) {
        delay(1);
      }
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
  #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
    dis.setup_calibration_tft(2);
  #endif
  initRotaries(SW_MENU, cali_weight, 100, 9999, 1); 
  i = 1;
  while (i > 0) {
    if (digitalRead(button_stop_pin) == HIGH or digitalRead(switch_setup_pin) == LOW) {
      while (digitalRead(button_stop_pin) == HIGH) {
        delay(1);
      }
      mode = -1;
      return;
    }
    cali_weight = getRotariesValue(SW_MENU);
    #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
      dis.setup_calibration_oled(2);
    #endif
    #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
      dis.setup_calibration_tft(3);
    #endif
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
      mode = -1;
      i = 0;        
    }
  }
}

void setupServoWinkel(void) {
  int menuitem;
  int menuitem_used         = 6;
  int lastmin               = angle_min;
  int lastfine              = angle_fine[fullmode-1];
  int lastmax               = angle_max[fullmode-1];
  int lastfinedosageweight  = finedos_weight[fullmode-1];
  int lastfullmode          = fullmode;
  int value_old;
  bool change_value = false;
  initRotaries(SW_MENU, 0, 0, menuitem_used, 1);
  #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
    draw_frame = true;
    change = false;
    value_old = -1;
    int menu_items_number = 7;
    const char *menuitems[menu_items_number] = {LIVESETUP[lingo], MINIMUM[lingo], FULL_MODE[lingo], FINEDOSAGE_WEIGHT[lingo], FINEDOSAGE[lingo], MAXIMUM[lingo], SAVE[lingo]};
  #endif
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
      if (servo_live == true) {
        SERVO_WRITE(angle_min);
      }
      mode = -1;
      return;
    }
    if (change_value == false) {
      menuitem = getRotariesValue(SW_MENU);
      #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
        pos = menuitem;
      #endif
      if (menuitem == menuitem_used) {
        menuitem = 6;
      }
    }
    else {
      switch (menuitem) {
        case 0: servo_live  = getRotariesValue(SW_MENU);
                break;
        case 1: angle_min   = getRotariesValue(SW_MENU);
                if (servo_live == true) SERVO_WRITE(angle_min);
                break;
        case 2: fullmode    = getRotariesValue(SW_MENU);
                break;
        case 3: finedos_weight[fullmode-1] = getRotariesValue(SW_MENU);
                break;
        case 4: angle_fine[fullmode-1] = getRotariesValue(SW_MENU);
                if (servo_live == true) SERVO_WRITE(angle_fine[fullmode-1]);
                break;
        case 5: angle_max[fullmode-1]   = getRotariesValue(SW_MENU);
                if (servo_live == true) SERVO_WRITE(angle_max[fullmode-1]);
                break;
      }
    }
    #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
      dis.setup_servoWinkel_oled(change_value, menuitem, menuitem_used);
    #endif
    #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
      dis.setup_servoWinkel_tft(change_value, menu_items_number, menuitems);
    #endif
    if (digitalRead(SELECT_SW) == SELECT_PEGEL && menuitem < menuitem_used  && change_value == false) {
      while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
        delay(1);
      }
      switch (menuitem) { 
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
      }
      #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
        value_old = -1;
      #endif
      change_value = true;
    }
    if (digitalRead(SELECT_SW) == SELECT_PEGEL && menuitem < menuitem_used  && change_value == true) {
      while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
        delay(1);
      }
      if (servo_live == true) {
        SERVO_WRITE(angle_min);
      }
      initRotaries(SW_MENU, menuitem, 0, menuitem_used, 1);
      change_value = false;
    }
    if (digitalRead(SELECT_SW) == SELECT_PEGEL && menuitem == 6) {
      while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
        delay(1);
      }
      #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
        dis.setup_servoWinkel_oled_exit(menuitem);
      #endif
      #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
        dis.setup_servoWinkel_tft_exit();
      #endif
      mode = -1;
      delay(1000);
      i = 0;
    }
  }
}

void setupAutomatik(void) {
  int menuitem;
  int menuitem_used       = 7;
  int menuoffset          = 0;
  int menuoffset_tft      = 0;
  int lastautostart       = autostart;
  int lastglastoleranz    = jartolerance;
  int lastautokorrektur   = autocorrection;
  int lastkulanz          = overfill_gr;
  int korrektur_alt       = correction;
  int intGewicht_alt      = init_weight_f;
  bool change_value       = false;
  #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
    draw_frame = true;
    change = false;
    value_old = -1;
    int menuitems_number = 8;
    const char *menuitems[menuitems_number] = {AUTOSTART[lingo], JAR_TOLERANCE[lingo], CORRECTION[lingo], AUTOCORRECTION[lingo], KINDNESS[lingo], FLOW_G_OVER_TIME[lingo], WAIT_BEFOR_FILL[lingo], SAVE[lingo]};
  #endif
  initRotaries(SW_MENU, 0, 0, menuitem_used, 1);
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
      mode = -1;
      return;
    }
    if (change_value == false) {
      menuitem = getRotariesValue(SW_MENU);
      #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
        pos = menuitem;
      #endif
      if (menuitem == menuitem_used) {
        menuitem = 7;
      }
    }
    else {
      switch (menuitem) {
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
    if (menuitem >= 6 and menuoffset == 0) {
      menuoffset = 1;
    }
    else if (menuitem == 0 and menuoffset == 1) {
      menuoffset = 0;
    }
    #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
      dis.setup_automatik_oled(menuoffset, change_value, menuitem, menuitem_used);
    #endif
    #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
      dis.setup_automatik_tft(change_value, menuitems, menuitems_number);
    #endif
    // Menupunkt zum Ändern ausgewählt
    if (digitalRead(SELECT_SW) == SELECT_PEGEL && menuitem < menuitem_used  && change_value == false) { 
      while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
        delay(1);
      }
      switch (menuitem) { 
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
    if (digitalRead(SELECT_SW) == SELECT_PEGEL && menuitem < menuitem_used  && change_value == true) {
      while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
        delay(1);
      }
      rotary_select = SW_MENU;
      initRotaries(SW_MENU, menuitem, 0, menuitem_used, 1);
      change_value = false;
    }
    // Menu verlassen 
    if (digitalRead(SELECT_SW) == SELECT_PEGEL && menuitem == 7) {
      while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
        delay(1);
      }
      #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
        dis.setup_automatik_oled_exit(menuitem);
      #endif
      #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
        dis.setup_automatik_tft_exit();
      #endif
      delay(1000);
      mode = -1;
      i = 0;
    }
  }
  rotary_select = SW_MENU;
}

void setupFuellmenge(void) {
  #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
    dis.setup_fuellmenge_tft(0);
  #endif
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
    #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
      dis.setup_fuellmenge_oled(1);
    #endif
    #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
      dis.setup_fuellmenge_tft(1);
    #endif
    if (digitalRead(SELECT_SW) == SELECT_PEGEL) { // Füllmenge gewählt
      while((digitalRead(SELECT_SW) == SELECT_PEGEL)) {
        delay(1);
      }
      i = 0;
    }
  }
  i = 1;
  initRotaries(SW_MENU, weight2step(glaeser[pos].Gewicht) , 25, weight2step(MAXIMALGEWICHT), 1);
  while (i > 0){
    if ((digitalRead(button_stop_pin)) == HIGH  or digitalRead(switch_setup_pin) == LOW) {
      mode = -1;
      return;
    }
    glaeser[pos].Gewicht = step2weight(getRotariesValue(SW_MENU));
    #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
      dis.setup_fuellmenge_oled(2);
    #endif
    #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
      dis.setup_fuellmenge_tft(2);
    #endif
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
    #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
      dis.setup_fuellmenge_oled(3);
    #endif
    #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
      dis.setup_fuellmenge_tft(3);
    #endif
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
  int menuitem;
  int menuitem_used = 5;
  int lastbuzzer    = buzzermode;
  int lastled       = ledmode;
  int lastlogo      = showlogo;
  int lastcredits   = showcredits;
  int lastcolor_scheme = color_scheme;
  int lastcolor_marker = color_marker;
  int lastchange_menu_rotation = menu_rotation;
  bool change_value = false;
  #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
    change_scheme = true;
    change_marker = true;
    value_old = -1;
    change = false;
    menuitem_used = 8;
    int menu_items_number = 9;
    const char *menuitems[menu_items_number] = {BUZZER[lingo], LED_1[lingo], SHOW_LOGO[lingo], SHOW_CREDITS[lingo], CHANGE_ROTATION[lingo], COLORSCHEME[lingo], MARKER_COLOR[lingo], FONT[lingo], SAVE[lingo]};
  #endif
  initRotaries(SW_MENU, 0, 0, menuitem_used, 1);
  i = 1;
  while (i > 0) {
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
      mode = -1;
      return;
    }
    if (change_value == false) {
      menuitem = getRotariesValue(SW_MENU);
      //#if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
        pos = menuitem;
      //#endif
      if (menuitem == menuitem_used) {
        menuitem = menuitem_used;
      }
    }
    else {
      switch (menuitem) {
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
    #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
      dis.setup_parameter_oled(change_value, menuitem, menuitem_used);
    #endif
    #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
      dis.setup_parameter_tft(menu_items_number, change_value, menuitems);
    #endif
    // Menupunkt zum Ändern ausgewählt
    if (digitalRead(SELECT_SW) == SELECT_PEGEL && menuitem < menuitem_used  && change_value == false) {
      while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
        delay(1);
      } 
      switch (menuitem) { 
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
      #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
        wert_old = -1;
      #endif
      change_value = true;
    }
    // Änderung im Menupunkt übernehmen
    Serial.println("-------");
    Serial.println(pos);
    Serial.println(menuitem_used);

    if (digitalRead(SELECT_SW) == SELECT_PEGEL && pos < menuitem_used ) {
      while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
        delay(1);
      }
      initRotaries(SW_MENU, menuitem, 0, menuitem_used, 1);
      change_value = false;
    }
    // Menu verlassen
    if (digitalRead(SELECT_SW) == SELECT_PEGEL && pos == menuitem_used) {
      while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
        delay(1);
      }
      #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
        dis.setup_parameter_oled_exit(menuitem);
      #endif
      #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
        dis.setup_parameter_tft_exit();
      #endif
      delay(1000);
      mode = -1;
      i = 0;
    }
  }
}

void setupClearPrefs(void) {
  int menu_items_number = 3;
  #if DREHTELLER == 1
    menu_items_number = menu_items_number + 1;
  #endif
  #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
    const char *menuitems[menu_items_number] = {CLEAR_PREFERENCES[lingo], CLEAR_NVS_MEMORY[lingo], BACK[lingo]};
    #if DREHTELLER == 1
      menuitems[menu_items_number - 1] = menuitems[menu_items_number -2];
      menuitems[menu_items_number - 2] = RESET_TURNTABLE[lingo];
    #endif
    draw_frame = true;
  #endif
  initRotaries(SW_MENU, menu_items_number -1 , 0, menu_items_number -1, 1);
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
    #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
      dis.setup_clear_prefs_oled(menu_items_number);
    #endif
    #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
      dis.setup_clear_prefs_tft(menu_items_number, menuitems);
    #endif
    if ((digitalRead(SELECT_SW)) == SELECT_PEGEL) {  
      while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
        delay(1);
      }
      if (pos == 0) {
        preferences.begin("EEPROM", false);
        preferences.clear();
        preferences.end();
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
          setPreferences();
          //Da machen wir gerade einen restart
          ESP.restart();
        }
      }
      #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
        dis.setup_clear_prefs_oled_exit();
      #endif
      #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
        dis.setup_clear_prefs_tft_exit();
      #endif
      delay(1000);
      mode = -1;
      i = 0;
    }
  }
}

void setupINA219(void) {                            //Funktioniert nur wenn beide Menüs die gleiche Anzahl haben. Feel free to change it :-)
  int menuitem;
  int lastcurrent             = current_servo;
  int lastwinkel_min          = angle_min;
  int lastshow_current        = show_current;
  bool change_value = false;
  int menuitem_used           = 2;
  String calibration_status   = START[lingo];
  String quetschhan           = CLOSE[lingo];
  bool cal_done = false;
  int cal_winkel = 0;
  int j = 0;
  int k = 0;
  #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
    change = false;
    draw_frame = true;
    value_old = -1;
    int menu_items_number = 3;
    const char *menuitems_1[menu_items_number] = {SERVO_CURRENT[lingo], CAL_HONEY_GATE[lingo], SAVE[lingo]};
    const char *menuitems_2[menu_items_number] = {SERVO_CURRENT[lingo], SHOW_CURRENT[lingo], SAVE[lingo]};
  #endif
  initRotaries(SW_MENU, 0, 0, menuitem_used, 1);
  i = 1;
  while (i > 0) {
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
      menuitem = getRotariesValue(SW_MENU);
      #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
        pos = menuitem;
      #endif
      if (menuitem == menuitem_used) {
        menuitem = 6;
      }
    }
    else {
      switch (menuitem) {
        case 0: current_servo         = getRotariesValue(SW_MENU);
                break;
        case 1: if (current_servo == 0) {
                  show_current = getRotariesValue(SW_MENU);
                }
                else {
                  j                     = 1;
                  calibration_status    = START[lingo];
                  change_value          = false;
                  menuitem_used         = 1;
                  setRotariesValue(SW_MENU, 0);
                  initRotaries(SW_MENU, 0, 0, menuitem_used, 1);
                }
                break;
      }
    }
    // Menu
    #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
      dis.setup_INA219_menu_oled(menuitem, menuitem_used, change_value);
    #endif
    #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
      dis.setup_INA219_menu_tft(change_value, menu_items_number, menuitems_1, menuitems_2);
    #endif
    if (digitalRead(SELECT_SW) == SELECT_PEGEL && menuitem < menuitem_used  && change_value == false) {
      while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
        delay(1);
      } 
      switch (menuitem) { 
        case 0: initRotaries(SW_MENU, current_servo, 0, 1500, 50);
                break;
        case 1: if (current_servo == 0) {initRotaries(SW_MENU, show_current, 0, 1, 1);}
                break;
      }
      change_value = true;
    }
    // Änderung im Menupunkt übernehmen
    if (digitalRead(SELECT_SW) == SELECT_PEGEL && menuitem < menuitem_used  && change_value == true) {
      while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
        delay(1);
      }
      initRotaries(SW_MENU, menuitem, 0, menuitem_used, 1);
      change_value = false;
    }
    // Menu verlassen 
    if (digitalRead(SELECT_SW) == SELECT_PEGEL && menuitem == 6) {
      while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
        delay(1);
      }
      #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
      dis.setup_INA219_oled_save(menuitem);
      #endif
      #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
        dis.setup_INA219_tft_save();
      #endif
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
        if (menuitem == menuitem_used) {
          menuitem = 6;
        }
      }
      menuitem_used = 1;
      #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
        dis.setup_INA219_cal1_oled(calibration_status, change_value, menuitem,  menuitem_used);
      #endif
      #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
        dis.setup_INA219_cal1_tft(calibration_status, change_value, menuitem,  menuitem_used); //das j könnte probleme machen
      #endif
      if (change_value == true && menuitem < menuitem_used) {
        lastcurrent = current_servo;
        calibration_status = CALIBRATION_RUNNING[lingo];
        quetschhan = CLOSE[lingo];
        cal_done = false;
        cal_winkel = 0;
        k = 1;
      }
      if (digitalRead(SELECT_SW) == SELECT_PEGEL && menuitem < menuitem_used  && change_value == false) {
        while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
          delay(1);
        } 
        change_value = true;
      }
      // Änderung im Menupunkt übernehmen
      if (digitalRead(SELECT_SW) == SELECT_PEGEL && menuitem < menuitem_used  && change_value == true) {
        while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
          delay(1);
        }
        initRotaries(SW_MENU, 0, 0, menuitem_used, 1);
        change_value = false;
      }
      //verlassen
      if ((digitalRead(SELECT_SW) == SELECT_PEGEL && menuitem == 6) or digitalRead(button_stop_pin) == HIGH) {
        while(digitalRead(SELECT_SW) == SELECT_PEGEL or digitalRead(button_stop_pin) == HIGH) {
          delay(1);
        }
        j = 0;
        change_value = false;
        menuitem_used = 2;
        draw_frame = true;
        initRotaries(SW_MENU, 0, 0, menuitem_used, 1);
      }
      if (k == 1) {draw_frame = true;}
      while (k > 0) {
        SERVO_WRITE(90);
        quetschhan = OPEN[lingo];
        scaletime = millis();
        bool measurement_run = false;
        while (!cal_done) {
          #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
            dis.setup_INA219_cal2_oled(calibration_status, cal_winkel, quetschhan);
          #endif
          #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
            dis.setup_INA219_cal2_tft(calibration_status, cal_winkel, quetschhan);
          #endif
          if (millis() - scaletime >= 800 and !measurement_run) {
            SERVO_WRITE(cal_winkel);
            quetschhan = CLOSE[lingo];
            measurement_run = true;
            scaletime = millis();
            #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
              change = true;
            #endif
          }
          else if (millis() - scaletime >= 800 and measurement_run) {
            current_mA = GetCurrent(50);
            if (current_mA > current_servo - 30) {   //30mA unter dem max. Wert Kalibrieren
              SERVO_WRITE(90);
              quetschhan = OPEN[lingo];
              cal_winkel++;
              #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
                change = true;
              #endif
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
            menuitem_used = 2;
            initRotaries(SW_MENU, 0, 0, menuitem_used, 1);
          }
        }
        if (cal_done and k > 1) {draw_frame = true;}
        while (cal_done and k > 1) {
          current_mA = GetCurrent(50);
          #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
            if (change_value != current_mA) {
              change = true;
              change_value = current_mA;
            }
          #endif
          #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
            dis.setup_INA219_cal3_oled(calibration_status);
          #endif
          #if DISPLAY_TYPE == 99 or DISPLAY_TYPE == 999
            dis.setup_INA219_cal3_tft(calibration_status);
          #endif
          //verlassen
          if (digitalRead(button_stop_pin) == HIGH) {
            while(digitalRead(button_stop_pin) == HIGH) {
              delay(1);
            }
            j = 0;
            k = 0;
            draw_frame = true;
            change_value = false;
            menuitem_used = 2;
            angle_min = lastwinkel_min;
            initRotaries(SW_MENU, 0, 0, menuitem_used, 1);
          }
          if (digitalRead(SELECT_SW) == SELECT_PEGEL) {
            while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
              delay(1);
            }
            j = 0;
            k = 0;
            mode = -1;
            change_value = false;
            menuitem_used = 2;
            initRotaries(SW_MENU, 0, 0, menuitem_used, 1);
            setPreferences();
            return;
          }
        }
      }
    }
  }
}

void setupAbout(void) {
  i = 1;
  uint8_t baseMac[6];
  #if DREHTELLER == 1
    esp_err_t ret = esp_wifi_get_mac(WIFI_IF_STA, baseMac);
  #endif
  #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
    dis.setup_about_oled(baseMac);
  #endif
  #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
    dis.setup_about_tft(baseMac);
  #endif
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
  int MenuepunkteAnzahl = sizeof(LANGUAGE2)/sizeof(LANGUAGE2[0]);
  const char *menuepunkte[MenuepunkteAnzahl];
  value_old = -1;
  for (int i = 0; i < MenuepunkteAnzahl; i++) {
    menuepunkte[i] = LANGUAGE2[i];
  }
  #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
    pos_old = -1;
    change_scheme = true;
  #endif
  initRotaries(SW_MENU, lingo, 0, MenuepunkteAnzahl, 1);
  i = 1;
  while (i > 0) {
    if (digitalRead(button_stop_pin) == HIGH  or digitalRead(switch_setup_pin) == LOW) {
      while (digitalRead(button_stop_pin) == HIGH) {
        delay(1);
      }
      lingo = last_lingo;
      mode = -1;
      return;
    }
    pos = getRotariesValue(SW_MENU);
    if (value_old != pos) {
      value_old = pos;
      #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
        dis.setup_language_oled(pos, MenuepunkteAnzahl, menuepunkte);
      #endif
      #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
        dis.setup_language_tft(pos, MenuepunkteAnzahl, menuepunkte);
      #endif
      setRotariesValue(SW_MENU, pos); //we einä dreit wiä äs Rindvieh
    }
    // Änderung im Menupunkt übernehmen
    if (digitalRead(SELECT_SW) == SELECT_PEGEL && pos < MenuepunkteAnzahl) {
      while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
        delay(1);
      }
      lingo = pos;
      value_old = -1;
      #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
        change_scheme = true;
      #endif
    }
    // Menu verlassen 
    if (digitalRead(SELECT_SW) == SELECT_PEGEL && pos == MenuepunkteAnzahl) {
      while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
        delay(1);
      }
      #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
        dis.setup_language_oled_exit();
      #endif
      #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
        dis.setup_language_tft_exit();
      #endif
      delay(1000);
      mode = -1;
      i = 0;
    }
  }
}

void setupWiFi(void) {
  int menu_items_number = 2;
  #if OTA == 1
    menu_items_number++;
  #endif
  const char *menuitems[menu_items_number] = {WIFI_SETUP[lingo], BACK[lingo]};
  #if OTA == 1
    menuitems[menu_items_number - 1] = menuitems[menu_items_number -2];
    menuitems[menu_items_number - 2] = START_OTA[lingo];
  #endif
  #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
    draw_frame = true;
  #endif
  initRotaries(SW_MENU, 0 , 0, menu_items_number -1, 1);
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
    #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
      dis.setup_setup_oled(menuitems, menu_items_number);
    #endif
    #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
      dis.setup_setup_tft(menuitems, menu_items_number);
    #endif
    if ((digitalRead(SELECT_SW)) == SELECT_PEGEL) {
      while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
        delay(1);
      }
      if (pos == 0) {
        #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
          draw_frame = true;
        #endif
        webif.setupWebIF();
      }
      else if (pos == 1) {
        ota.ota_setup(switch_setup_pin, button_stop_pin);
        #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
          draw_frame = true;
        #endif
      }
      if (menuitems[pos] == BACK[lingo]) {
      #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
        dis.setup_setup_oled_exit();
      #endif
      #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
        dis.setup_setup_tft_exit();
      #endif
      delay(1000);
      mode = -1;
      i = 0;
      }
    }
  }
}

void setupDrehteller(void) {
  //Start Menue
  #if DREHTELLER == 1
    unsigned long time;
    int ota_update = 0;
    bool ota_update_enable = false;
    int menu_items_number_2 = 4;
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
        menu_items_number_2 = 5;
      }
    }
    int x_pos;
    int k = 0;
    int menuitem_1;
    int menu_items_number_1 = 3;
    int last_menu_pos_1 = 0;
    const char *menuitems_1[menu_items_number_1] = {TURNTABLE[lingo], INIT_TURNTABLE[lingo], SAVE[lingo]};
    int menuitem_2;
    int last_menu_pos_2 = 0;
    const char *menuitems_2[menu_items_number_2] = {TURNTABLE[lingo], SETUP_TURNTABLE[lingo], SETUP_DRIPPRODECTION[lingo], SAVE[lingo]};
    int menuitem_3;
    int menu_items_number_3 = 5;
    int last_menu_pos_3 = 0;
    const char *menuitems_3[menu_items_number_3] = {MOVE_JAR[lingo], CENTER_JAR[lingo], SPEED_INIT[lingo], SPEED_RUN[lingo], SAVE[lingo]};
    int menuitem_4;
    int menu_items_number_4 = 7;
    int last_menu_pos_4 = 0;
    const char *menuitems_4[menu_items_number_4] = {OPEN_DRIPPROTECTION[lingo], CLOSE_DRIPPROTECTION[lingo], SPEED_DRIPPROTECTION[lingo], WAIT_TO_CLOSE_DP[lingo], DP_MIN_ANGLE[lingo], DP_MAX_ANGLE[lingo], SAVE[lingo]};
    int last_use_turntable = use_turntable;
    bool change_value = false;
    bool turntable_running = false;
    bool center_jar_running = false;
    bool ts_open_running = false;
    bool ts_close_running = false;
    bool ts_angle_running = false;
    unsigned long turntable_millis;
    unsigned long prev_millis = 0;
    int speed_init = 0;
    int speed_run = 0;
    int jar_center_pos = 0;
    int speed_init_old = 0;
    int speed_run_old = 0;
    int jar_center_pos_old = 0;
    int move_steps = 50;
    int move_vale = 0;
    int ts_angle_min = 0;
    int ts_angle_max = 180;
    int ts_waittime = 0;
    int ts_speed = 0;
    int ts_angle_min_old = 0;
    int ts_angle_max_old = 180;
    int ts_waittime_old = 0;
    int ts_speed_old = 0;
    int ts_angle_min_start = 0;
    int ts_angle_max_start = 0;
    esp_now_msg_recived = false;
    esp_now_change = true;
    esp_now_wait = "";
    #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
      change = false;
      value_old = -1;
      esp_now_wait_old = "...";
      dis.setup_turntable_frame();
    #endif
    initRotaries(SW_MENU, 0, 0, menu_items_number_1 -1, 1);
    //Init Turntable
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
      i = 1;
    }
    else {  //no connection to the turntable
      i = 0;
      esp_now_msg_recived = false;
      #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
        dis.setup_turntable_no_connection_oled();
      #endif
      #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
        dis.setup_turntable_no_connection_tft();
      #endif
      delay(4000);
    }
    if (ota_update == 1) {
      menuitems_2[menu_items_number_2 - 2] = ENABLE_OTA_UPDATE[lingo];
      menuitems_2[menu_items_number_2 - 1] = SAVE[lingo];
    }
    while (i > 0) {
      //Menue 1 (Turntable init nOK)
      while (i == 1) {
        if (turntable_init == true) {
          i = 2;
          initRotaries(SW_MENU, 0, 0, menu_items_number_2 -1, 1);
          esp_now_msg_recived = false;
          memset(&myReceivedMessage, 0, sizeof(myReceivedMessage));
          memset(&myMessageToBeSent, 0, sizeof(myMessageToBeSent));
          #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
            dis.setup_turntable_menu1_1_oled();
          #endif
          #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
            dis.setup_turntable_menu1_1_tft();
          #endif
          break;
        }
        if ((digitalRead(button_stop_pin) == HIGH and turntable_running == false) or digitalRead(switch_setup_pin) == LOW) {
          while (digitalRead(button_stop_pin) == HIGH) {
            delay(1);
          }
          esp_now_msg_recived = false;
          use_turntable = last_use_turntable;
          rotary_select = SW_MENU;
          mode = -1;
          memset(&myReceivedMessage, 0, sizeof(myReceivedMessage));
          memset(&myMessageToBeSent, 0, sizeof(myMessageToBeSent));
          return;
        }
        if (esp_now_msg_recived == true and strcmp(myReceivedMessage.text, "init_error") == 0) {
          esp_now_msg_recived = false;
          turntable_init = false;
          turntable_running = false;
          turntable_millis = 0;
          prev_millis = 0;
          esp_now_wait = "";
          initRotaries(SW_MENU, 1, 0, 1, 1);
          memset(&myReceivedMessage, 0, sizeof(myReceivedMessage));
          #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
            esp_now_change = true;
          #endif
        }
        if (turntable_running == true) {
          initRotaries(SW_MENU, last_menu_pos_1, 0, menu_items_number_1 - 1, 1);                         //Wenn einer am rotary dreht wider zurücksetzen
          if (millis() - turntable_millis > 60000 or digitalRead(button_stop_pin) == HIGH) {             //Timeaut, wenn der Befehl nicht in 60s geschaft wird oder Abbrechen wenn Stop Taste gedrückt wird
            while (digitalRead(button_stop_pin) == HIGH) {
              delay(1);
            }
            strcpy(myMessageToBeSent.text, "stop");
            espnow_send_data();
            turntable_init = false;
            turntable_running = false;
            turntable_millis = 0;
            prev_millis = 0;
            esp_now_wait = "";
            initRotaries(SW_MENU, 1, 0, 1, 1);
            memset(&myReceivedMessage, 0, sizeof(myReceivedMessage));
            #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
              esp_now_change = true;
            #endif
          }
          else if (millis() - prev_millis > 1000 and (turntable_init == 1 or menuitem_1 == 1)) {
            prev_millis = millis();
            if (esp_now_wait.length() < 3) {esp_now_wait = esp_now_wait + ".";}
            else {esp_now_wait = "";}
            #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
              esp_now_change = true;
            #endif
          }
          if (esp_now_msg_recived == true) {
            if (last_menu_pos_1 == 1) {
              if(strcmp(myReceivedMessage.text, "ok_init_done") == 0) {
                turntable_init = true;
              }
              #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
                esp_now_change = true;
              #endif
            }
            //Werte zurücksetzen
            esp_now_msg_recived = false;
            turntable_running = false;
            turntable_millis = 0;
            esp_now_wait = "";
            memset(&myReceivedMessage, 0, sizeof(myReceivedMessage));
          }
        }
        if (change_value == false) {
          if (use_turntable == 0 and getRotariesValue(SW_MENU) != menuitem_1) {
            menuitem_1 = getRotariesValue(SW_MENU);
            initRotaries(SW_MENU, menuitem_1, 0, 1, 1);
          }
          else if (getRotariesValue(SW_MENU) != menuitem_1) {
            menuitem_1 = getRotariesValue(SW_MENU);
            initRotaries(SW_MENU, menuitem_1, 0, menu_items_number_1 - 1, 1);
          }
          #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
            pos = menuitem_1;
            if ((use_turntable == 0 and menuitem_1 == 1) or (menuitem_1 == 2 and use_turntable == 1 and turntable_init == false)) {
              pos = menu_items_number_1 - 1;
            }
          #endif
          if (menuitem_1 == menu_items_number_1 - 1 or (use_turntable == 0 and menuitem_1 == 1) or (turntable_init == false and menuitem_1 == 2)) {
            menuitem_1 = 7;
          }
        }
        else {
          switch (menuitem_1) {
            case 0: use_turntable = getRotariesValue(SW_MENU);
                    break;
          }
        }
        #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
          dis.setup_turntable_menu1_2_oled(change_value, menuitems_1, menuitem_1, menu_items_number_1, last_menu_pos_1, turntable_running, esp_now_wait);
        #endif
        #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
          dis.setup_turntable_menu1_2_tft(change_value, menuitems_1, menu_items_number_1, turntable_running);
        #endif
        // Menupunkt zum ändern ausgewählt
        if (digitalRead(SELECT_SW) == SELECT_PEGEL && menuitem_1 < menu_items_number_1 - 1  && change_value == false) {
          while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
            delay(1);
          }
          last_menu_pos_1 = menuitem_1;
          switch (menuitem_1) {
            case 0: rotary_select = SW_MENU;
                    initRotaries(SW_MENU, use_turntable, 0, 1, 1);
                    break;
            case 1: if (turntable_init == false) {
                      turntable_running = true;
                      turntable_millis = millis();
                      esp_now_msg_recived = false;
                      strcpy(myMessageToBeSent.text, "init");
                      espnow_send_data();
                    }
                    break;
          }
          #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
            value_old = -1;
          #endif
          change_value = true;
        }
        // Änderung im Menupunkt übernehmen
        if ((digitalRead(SELECT_SW) == SELECT_PEGEL && menuitem_1 < menu_items_number_1 - 1  && change_value == true) or (turntable_running == true and change_value == true)) {
          while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
            delay(1);
          }
          rotary_select = SW_MENU;
          initRotaries(SW_MENU, menuitem_1, 0, menu_items_number_1 - 1, 1);
          change_value = false;
        }
        // Menu verlassen
        if (digitalRead(SELECT_SW) == SELECT_PEGEL && menuitem_1 == 7 && turntable_running == false) {
          while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
            delay(1);
          }
          #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
            dis.setup_turntable_menu1_oled_exit(menuitem_1);
          #endif
          #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
            dis.setup_turntable_menu1_tft_exit();
          #endif
          esp_now_msg_recived = false;
          delay(1000);
          i = 0;
        }
      }
      //Menu 2 (Turntable init OK)
      while (i == 2) {
        if (esp_now_msg_recived == true and strcmp(myReceivedMessage.text, "init_error") == 0) {
          esp_now_msg_recived = false;
          turntable_init = false;
          turntable_running = false;
          turntable_millis = 0;
          prev_millis = 0;
          esp_now_wait = "";
          #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
            esp_now_wait_old = "...";
          #endif
        }
        if (turntable_init == false) {
          i = 1;
          initRotaries(SW_MENU, 0, 0, menu_items_number_1 -1, 1);
          esp_now_msg_recived = false;
          memset(&myReceivedMessage, 0, sizeof(myReceivedMessage));
          memset(&myMessageToBeSent, 0, sizeof(myMessageToBeSent));
          #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
            dis.setup_turntable_menu2_1_oled();
          #endif
          #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
            dis.setup_turntable_menu2_1_tft();
          #endif
          break;
        }
        if ((digitalRead(button_stop_pin) == HIGH and turntable_running == false) or digitalRead(switch_setup_pin) == LOW) {
          while (digitalRead(button_stop_pin) == HIGH) {
            delay(1);
          }
          esp_now_msg_recived = false;
          use_turntable = last_use_turntable;
          rotary_select = SW_MENU;
          mode = -1;
          memset(&myReceivedMessage, 0, sizeof(myReceivedMessage));
          memset(&myMessageToBeSent, 0, sizeof(myMessageToBeSent));
          return;
        }
        if (change_value == false) {
          if (use_turntable == 0 and getRotariesValue(SW_MENU) != menuitem_2) {
            menuitem_2 = getRotariesValue(SW_MENU);
            initRotaries(SW_MENU, menuitem_2, 0, 1, 1);
          }
          else if (getRotariesValue(SW_MENU) != menuitem_2) {
            menuitem_2 = getRotariesValue(SW_MENU);
            initRotaries(SW_MENU, menuitem_2, 0, menu_items_number_2 - 1, 1);
          }
          #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
            pos = menuitem_2;
            if ((use_turntable == 0 and menuitem_2 == 1)) {
              pos = menu_items_number_2 -1;
            }
          #endif
          if (menuitem_2 == menu_items_number_2 - 1 or (use_turntable == 0 and menuitem_2 == 1)) {
            menuitem_2 = 7;
          }
        }
        else {
          switch (menuitem_2) {
            case 0: use_turntable = getRotariesValue(SW_MENU);
                    break;
          }
        }
        #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
          dis.setup_turntable_menu2_2_oled(change_value, menuitems_2, menuitem_2, menu_items_number_2, ota_update);
        #endif
        #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
          dis.setup_turntable_menu2_2_tft(change_value, menuitems_2, menu_items_number_2, ota_update);
        #endif
        // Menupunkt zum ändern ausgewählt
        if (digitalRead(SELECT_SW) == SELECT_PEGEL && menuitem_2 < menu_items_number_2 - 1  && change_value == false) {
          while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
            delay(1);
          }
          last_menu_pos_2 = menuitem_2;
          switch (menuitem_2) {
            case 0: rotary_select = SW_MENU;
                    initRotaries(SW_MENU, use_turntable, 0, 1, 1);
                    break;
            case 1: i = 3;
                    break;
            case 2: i = 4;
                    break;
            case 3: ota_update_enable = true;
                    break;
          }
          #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
            value_old = -1;
          #endif
          change_value = true;
          if (i == 3 or i == 4) {
            menuitem_3 = pos = 0;
            change_value = false;
            #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
              dis.setup_turntable_menu2_1_oled();
            #endif
            #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
              dis.setup_turntable_menu2_1_tft();
            #endif
            break;
          }
        }
        // Enable OTA update for the Turntable
        int ota_status = 0;
        if (ota_update_enable == true) {
          #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
            dis.setup_turntable_menu2_3_tft();
          #endif
        }
        while (ota_update_enable == true) {
          #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
            dis.setup_turntable_menu2_4_oled();
          #endif
          #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
            dis.setup_turntable_menu2_4_tft();
          #endif
          esp_now_msg_recived = false;
          memset(&myReceivedMessage, 0, sizeof(myReceivedMessage));
          strcpy(myMessageToBeSent.text, "enable_ota_update");
          espnow_send_data();
          time = millis();
          while (!esp_now_msg_recived and millis() - time <= 16000 and strcmp(myReceivedMessage.text, "") == 0 and digitalRead(switch_setup_pin) == HIGH and digitalRead(button_stop_pin) == LOW) {
            delay(10);
          }
          if (digitalRead(switch_setup_pin) == LOW or digitalRead(button_stop_pin) == HIGH) {
            while (digitalRead(button_stop_pin) == HIGH) {
              delay(10);
            }
            stop_button_used = true;
            ota_update_enable = false;
          }
          if (strcmp(myReceivedMessage.text, "") != 0 and stop_button_used == false) {
            ota_status = 1;     //Turntable has an IP adress
            #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
              dis.setup_turntable_menu2_5_oled(myReceivedMessage.text);
            #endif
            #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
              dis.setup_turntable_menu2_5_tft(myReceivedMessage.text);
            #endif
            memset(&myReceivedMessage, 0, sizeof(myReceivedMessage));
            while ((ota_status == 1 or ota_status == 2) and stop_button_used == false) {
              delay(1000);
              if (digitalRead(switch_setup_pin) == LOW or digitalRead(button_stop_pin) == HIGH and ota_status == 1) {
                while (digitalRead(button_stop_pin) == HIGH) {
                  delay(10);
                }
                stop_button_used = true;
                ota_update_enable = false;
              }
              if (strcmp(myReceivedMessage.text, "") != 0 and strcmp(myReceivedMessage.text, "Success") != 0 and strcmp(myReceivedMessage.text, "Fail") != 0) {
                ota_status = 2;     //OTA Update ist am laufen
                #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
                  dis.setup_turntable_menu2_6_oled(myReceivedMessage.text, myReceivedMessage.value);
                #endif
                #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
                  dis.setup_turntable_menu2_6_tft(myReceivedMessage.text, myReceivedMessage.value);
                #endif
                if (strcmp(myReceivedMessage.text, "Success") != 0 or strcmp(myReceivedMessage.text, "Fail") != 0) {
                  memset(&myReceivedMessage, 0, sizeof(myReceivedMessage));
                }
              }
              else if (strcmp(myReceivedMessage.text, "Success") == 0 or strcmp(myReceivedMessage.text, "Fail") == 0) {
                ota_status = 3;     //OTA Update ist fertig oder hat einen Error
              }
            }
            while (ota_status == 3) {
              ota_status = 4;     //verlasse OTA
              ota_update_enable = false;
              i = 0;
              int x_pos;
              if (strcmp(myReceivedMessage.text, "Success") == 0) {
                #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
                  dis.setup_turntable_menu2_7_oled();
                #endif
                #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
                  dis.setup_turntable_menu2_7_tft();
                #endif
                ota_done = 1;
                #if DEBUG_HM >= 1
                  Serial.println("OTA update finished successfully!");
                #endif
              } 
              else {
                #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
                  dis.setup_turntable_menu2_8_oled();
                #endif
                #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
                  dis.setup_turntable_menu2_8_tft();
                #endif
                #if DEBUG_HM >= 1
                  Serial.println("There was an error during OTA update!");
                #endif
              }
              memset(&myReceivedMessage, 0, sizeof(myReceivedMessage));
              delay(5000);
            }
          }
          else if (ota_status == 0) {
            ota_update_enable = false;
            #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
              dis.setup_turntable_menu2_9_oled();
            #endif
            #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
              dis.setup_turntable_menu2_9_tft(1);
            #endif
            delay(5000);
            #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
              dis.setup_turntable_menu2_9_tft(2);
            #endif
          }
          //if (digitalRead(switch_setup_pin) == LOW or digitalRead(button_stop_pin) == HIGH) {
          //  while (digitalRead(button_stop_pin) == HIGH) {
          //    delay(10);
          //  }
          //}
          if (ota_update_enable == false) {
            //OTA verlassen
            memset(&myReceivedMessage, 0, sizeof(myReceivedMessage));
            esp_now_msg_recived = false;
            strcpy(myMessageToBeSent.text, "stop_ota_update");
            espnow_send_data();
            time = millis();
            while (!esp_now_msg_recived and millis() - time <= 1000 and strcmp(myReceivedMessage.text, "") == 0) {
              delay(10);
            }
            change_value = false;
            stop_button_used = false;
            #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
              dis.setup_turntable_menu2_9_tft(3);
            #endif
          }
        }
        // Änderung im Menupunkt übernehmen
        if ((digitalRead(SELECT_SW) == SELECT_PEGEL && menuitem_2 < menu_items_number_2 - 1  && change_value == true)) {
          while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
            delay(1);
          }
          rotary_select = SW_MENU;
          initRotaries(SW_MENU, menuitem_2, 0, menu_items_number_2 - 1, 1);
          change_value = false;
        }
        // Menu verlassen
        if (digitalRead(SELECT_SW) == SELECT_PEGEL && menuitem_2 == 7) {
          while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
            delay(1);
          }
          #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
            dis.setup_turntable_menu2_oled_exit(menuitem_2);
          #endif
          #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
            dis.setup_turntable_menu2_tft_exit();
          #endif
          esp_now_msg_recived = false;
          delay(1000);
          i = 0;
        }
      }
      if (i == 3) {       //Menü Drehteller
        esp_now_msg_recived = false;
        memset(&myReceivedMessage, 0, sizeof(myReceivedMessage));
        strcpy(myMessageToBeSent.text, "speed_init");
        espnow_send_data();
        turntable_millis = millis();
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
        initRotaries(SW_MENU, menuitem_3, 0, menu_items_number_3 - 1, 1);
      }
      while (i == 3) {
        if (esp_now_msg_recived == true and strcmp(myReceivedMessage.text, "init_error") == 0) {
          esp_now_msg_recived = false;
          turntable_init = false;
          turntable_running = false;
          center_jar_running = false;
          turntable_millis = 0;
          prev_millis = 0;
          esp_now_wait = "";
          #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
            esp_now_wait_old = "...";
          #endif
        }
        if (turntable_init == false) {
          i = 1;
          initRotaries(SW_MENU, 0, 0, menu_items_number_1 -1, 1);
          esp_now_msg_recived = false;
          memset(&myReceivedMessage, 0, sizeof(myReceivedMessage));
          memset(&myMessageToBeSent, 0, sizeof(myMessageToBeSent));
          #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
            dis.setup_turntable_menu3_1_oled();
          #endif
          #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
            dis.setup_turntable_menu3_1_tft();
          #endif
          break;
        }
        if (digitalRead(button_stop_pin) == HIGH and turntable_running == false) {
          while (digitalRead(button_stop_pin) == HIGH) {
            delay(1);
          }
          #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
            value_old = -1;
          #endif
          change_value = false;
          rotary_select = SW_MENU;
          initRotaries(SW_MENU, menuitem_2, 0, menu_items_number_2 - 1, 1);
          esp_now_msg_recived = false;
          memset(&myReceivedMessage, 0, sizeof(myReceivedMessage));
          memset(&myMessageToBeSent, 0, sizeof(myMessageToBeSent));
          i = 2;
          #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
            dis.setup_turntable_menu3_1_oled();
          #endif
          #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
            dis.setup_turntable_menu3_1_tft();
          #endif
          break;
        }
        if (digitalRead(switch_setup_pin) == LOW) {
          esp_now_msg_recived = false;
          rotary_select = SW_MENU;
          mode = -1;
          memset(&myReceivedMessage, 0, sizeof(myReceivedMessage));
          memset(&myMessageToBeSent, 0, sizeof(myMessageToBeSent));
          return;
        }
        if (turntable_running == true) {
          initRotaries(SW_MENU, last_menu_pos_3, 0, menu_items_number_3 - 1, 1);                         //Wenn einer am rotary dreht wider zurücksetzen
          if (millis() - turntable_millis > 60000 or digitalRead(button_stop_pin) == HIGH) {             //Timeaut, wenn der Befehl nicht in 60s geschaft wird oder Abbrechen wenn Stop Taste gedrückt wird
            while (digitalRead(button_stop_pin) == HIGH) {
              delay(1);
            }
            strcpy(myMessageToBeSent.text, "stop");
            espnow_send_data();
            esp_now_msg_recived = false;
            change_value = false;
            turntable_init = false;
            turntable_running = false;
            turntable_millis = 0;
            prev_millis = 0;
            esp_now_wait = "";
            #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
              esp_now_wait_old = "...";
            #endif
            initRotaries(SW_MENU, 1, 0, 1, 1);
            memset(&myReceivedMessage, 0, sizeof(myReceivedMessage));
            #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
              esp_now_change = true;
            #endif
          }
          else if (millis() - prev_millis > 1000 and menuitem_3 == 0) {
            prev_millis = millis();
            if (esp_now_wait.length() < 3) {esp_now_wait = esp_now_wait + ".";}
            else {esp_now_wait = "";}
            #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
              esp_now_change = true;
            #endif
          }
          if (esp_now_msg_recived == true) {
            //Werte zurücksetzen
            change_value = false;
            esp_now_msg_recived = false;
            turntable_running = false;
            turntable_millis = 0;
            esp_now_wait = "";
            #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
              esp_now_wait_old = "...";
              change = true;
            #endif
            memset(&myReceivedMessage, 0, sizeof(myReceivedMessage));
          }
        }
        if (center_jar_running == true) {
          initRotaries(SW_MENU, jar_center_pos, 0, 1500, move_steps); //zurücksetzen wenn zu schnell am rotary gedreht wurde
          if (millis() - turntable_millis > 3000) {   //verlassen wenn der Befehl nicht in 3sek geschafft wurde
            //muss noch gemacht werden :-)
          }
          if (esp_now_msg_recived == true) {
            //Werte zurücksetzen
            esp_now_msg_recived = false;
            center_jar_running = false;
            turntable_millis = 0;
            memset(&myReceivedMessage, 0, sizeof(myReceivedMessage));
          }
        }
        if (change_value == false and getRotariesValue(SW_MENU) != menuitem_3) {
          menuitem_3 = getRotariesValue(SW_MENU);
          initRotaries(SW_MENU, menuitem_3, 0, menu_items_number_3 - 1, 1);
          #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
            pos = menuitem_3;
          #endif
          if (menuitem_3 == menu_items_number_3 - 1) {
            menuitem_3 = 7;
          }
        }
        else if (getRotariesValue(SW_MENU) != menuitem_3) {
          switch (menuitem_3) {
            case 1: jar_center_pos = getRotariesValue(SW_MENU);
                    break;
            case 2: speed_init = getRotariesValue(SW_MENU);
                    break;
            case 3: speed_run = getRotariesValue(SW_MENU);
                    break;
          }
        }
        #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
          dis.setup_turntable_menu3_2_oled(change_value, menuitems_3, menuitem_3, menu_items_number_3, esp_now_wait, jar_center_pos, speed_init, speed_run);
        #endif
        #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
          dis.setup_turntable_menu3_2_tft(change_value, menuitems_3, menu_items_number_3, turntable_running, jar_center_pos, speed_init, speed_run);
        #endif
        if (digitalRead(SELECT_SW) == SELECT_PEGEL && menuitem_3 < menu_items_number_3 - 1  && change_value == false) {
          while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
            delay(1);
          }
          last_menu_pos_3 = menuitem_3;
          switch (menuitem_3) {
            case 0: turntable_running = true;
                    turntable_millis = millis();
                    esp_now_msg_recived = false;
                    strcpy(myMessageToBeSent.text, "move_jar");
                    espnow_send_data();
                    break;
            case 1: rotary_select = SW_MENU;
                    initRotaries(SW_MENU, jar_center_pos, 0, 1500, move_steps);
                    break;
            case 2: rotary_select = SW_MENU;
                    initRotaries(SW_MENU, speed_init, 100, 600, 50);
                    break;
            case 3: rotary_select = SW_MENU;
                    initRotaries(SW_MENU, speed_run, 100, 600, 50);
                    break;
          }
          #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
            value_old = -1;
          #endif
          change_value = true;
        }
        if (menuitem_3 == 1 and change_value == true) {
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
        if ((digitalRead(SELECT_SW) == SELECT_PEGEL && menuitem_3 < menu_items_number_3 - 1  && change_value == true)) {
          while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
            delay(1);
          }
          rotary_select = SW_MENU;
          initRotaries(SW_MENU, menuitem_3, 0, menu_items_number_3 - 1, 1);
          change_value = false;
        }
        // Menu verlassen
        if (digitalRead(SELECT_SW) == SELECT_PEGEL && menuitem_3 == 7) {
          while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
            delay(1);
          }
          if (speed_init != speed_init_old) {
            strcpy(myMessageToBeSent.text, "speed_init_save");
            myMessageToBeSent.value = speed_init;
            espnow_send_data();
            //noch überlegen was gemacht werden soll, wenn der Wert nicht Gespeichert wurde
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
            //noch überlegen was gemacht werden soll, wenn der Wert nicht Gespeichert wurde
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
            //noch überlegen was gemacht werden soll, wenn der Wert nicht Gespeichert wurde
            turntable_millis = millis();
            esp_now_msg_recived = false;
            while (millis() - turntable_millis < 3000 and esp_now_msg_recived == false) {  //brechce ab wenn in 3 sek keine rückmeldung kommt
              delay(10);
            }
          }
          #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
            dis.setup_turntable_menu3_oled_exit(menuitem_3);
          #endif
          #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
            dis.setup_turntable_menu3_tft_exit();
          #endif
          esp_now_msg_recived = false;
          jar_center_pos = 0;
          jar_center_pos_old = 0;
          delay(1000);
          initRotaries(SW_MENU, 1, 0, menu_items_number_2 -1, 1);
          i = 2;
          #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
            dis.setup_turntable_menu3_1_oled();
          #endif
          #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
            dis.setup_turntable_menu3_1_tft();
          #endif
        }
      }
      if (i == 4) {       //Menü Setup Tropfschutz
        setRotariesValue(SW_MENU, 0);       //Workaround --> noch suchen warum der Rotary Wert auf 2 ist
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
        ts_speed = ts_speed_old = myReceivedMessage.value;
        esp_now_msg_recived = false;
        memset(&myReceivedMessage, 0, sizeof(myReceivedMessage));
        //initRotaries(SW_MENU, menuitem_4, 0, MenuepunkteAnzahl_4 - 1, 1);
      }
      while (i == 4) {
        if (digitalRead(button_stop_pin)) {
          while (digitalRead(button_stop_pin) == HIGH) {
            delay(1);
          }
          change_value = false;
          rotary_select = SW_MENU;
          initRotaries(SW_MENU, menuitem_2, 0, menu_items_number_2 - 1, 1);
          esp_now_msg_recived = false;
          memset(&myReceivedMessage, 0, sizeof(myReceivedMessage));
          memset(&myMessageToBeSent, 0, sizeof(myMessageToBeSent));
          i = 2;
          #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
            dis.setup_turntable_menu4_1_oled();
          #endif
          #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
            value_old = -1;
            dis.setup_turntable_menu4_1_tft();
          #endif
          break;
        }
        if (digitalRead(switch_setup_pin) == LOW) {
          esp_now_msg_recived = false;
          rotary_select = SW_MENU;
          mode = -1;
          memset(&myReceivedMessage, 0, sizeof(myReceivedMessage));
          memset(&myMessageToBeSent, 0, sizeof(myMessageToBeSent));
          return;
        }
        if (ts_open_running == true or ts_close_running == true or ts_angle_running == true) { //zurücksetzen wenn zu schnell am rotary gedreht wurde
          if (ts_open_running == true or ts_close_running) {initRotaries(SW_MENU, menuitem_4, 0, menu_items_number_4 - 1, 1);}
          else if (ts_angle_running == true and menuitem_4 == 4) {initRotaries(SW_MENU, ts_angle_min, 0, ts_angle_max, 5);}
          else if (ts_angle_running == true and menuitem_4 == 5) {initRotaries(SW_MENU, ts_angle_max, ts_angle_min, 180, 5);}
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
        if (change_value == false and getRotariesValue(SW_MENU) != menuitem_4) {
          menuitem_4 = getRotariesValue(SW_MENU);
          initRotaries(SW_MENU, menuitem_4, 0, menu_items_number_4 - 1, 1);
          #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
            pos = menuitem_4;
          #endif
          if (menuitem_4 == menu_items_number_4 - 1) {
            menuitem_4 = 7;
          }
        }
        else if (getRotariesValue(SW_MENU) != menuitem_4) {
          switch (menuitem_4) {
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
        #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
          dis.setup_turntable_menu4_2_oled(change_value, menuitems_4, menuitem_4, menu_items_number_4, ts_speed, ts_waittime, ts_angle_min, ts_angle_max);
        #endif
        #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
          dis.setup_turntable_menu4_2_tft(change_value, menuitems_4, menu_items_number_4, ts_speed, ts_waittime, ts_angle_min, ts_angle_max);
        #endif
        if (digitalRead(SELECT_SW) == SELECT_PEGEL && menuitem_4 < menu_items_number_4 - 1  && change_value == false) {
          while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
            delay(1);
          }
          last_menu_pos_4 = menuitem_4;
          switch (menuitem_4) {
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
          #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
            value_old = -1;
          #endif
          change_value = true;
        }
        if ((menuitem_4 == 4 or menuitem_4 == 5) and change_value == true) {
          move_vale = -1;
          if (menuitem_4 == 4 and ts_angle_min != ts_angle_min_old) {move_vale = ts_angle_min; ts_angle_min_old = ts_angle_min;}
          if (menuitem_4 == 5 and ts_angle_max != ts_angle_max_old) {move_vale = ts_angle_max; ts_angle_max_old = ts_angle_max;}
          if (move_vale >= 0) {
            ts_angle_running = true;
            esp_now_msg_recived = false;
            turntable_millis = millis();
            strcpy(myMessageToBeSent.text, "move_dp");
            myMessageToBeSent.value = move_vale;
            espnow_send_data();
          }
        }
        // Änderung im Menupunkt übernehmen
        if ((digitalRead(SELECT_SW) == SELECT_PEGEL && menuitem_4 < menu_items_number_4 - 1  && change_value == true)) {
          while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
            delay(1);
          }
          rotary_select = SW_MENU;
          initRotaries(SW_MENU, menuitem_4, 0, menu_items_number_4 - 1, 1);
          change_value = false;
        }
        // Menu verlassen
        if (digitalRead(SELECT_SW) == SELECT_PEGEL && menuitem_4 == 7) {
          while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
            delay(1);
          }
          if (ts_angle_min != ts_angle_min_start) {
            strcpy(myMessageToBeSent.text, "ts_angle_min_save");
            myMessageToBeSent.value = ts_angle_min;
            espnow_send_data();
            //noch überlegen was gemacht werden soll, wenn der Wert nicht Gespeichert wurde
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
            //noch überlegen was gemacht werden soll, wenn der Wert nicht Gespeichert wurde
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
            //noch überlegen was gemacht werden soll, wenn der Wert nicht Gespeichert wurde
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
            //noch überlegen was gemacht werden soll, wenn der Wert nicht Gespeichert wurde
            turntable_millis = millis();
            esp_now_msg_recived = false;
            while (millis() - turntable_millis < 3000 and esp_now_msg_recived == false) {  //brechce ab wenn in 3 sek keine rückmeldung kommt
              delay(10);
            }
          }
          #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
            dis.setup_turntable_menu4_oled_exit(menuitem_4);
          #endif
          #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
            dis.setup_turntable_menu4_tft_exit();
          #endif
          esp_now_msg_recived = false;
          ts_angle_running = false;
          ts_close_running = false;
          ts_open_running = false;
          delay(1000);
          initRotaries(SW_MENU, 2, 0, menu_items_number_2 -1, 1);
          i = 2;
          #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
            dis.setup_turntable_menu4_1_oled();
          #endif
          #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
            dis.setup_turntable_menu4_1_tft();
          #endif
        }
      }
    }
    mode = -1;
    rotary_select = SW_MENU;
    memset(&myReceivedMessage, 0, sizeof(myReceivedMessage));
    memset(&myMessageToBeSent, 0, sizeof(myMessageToBeSent));
  #endif
}

void processSetup(void) {
  int x_pos;
  int MenuepunkteAnzahl = 11;
  int menuitem_old = -1;
  if (ina219_installed) {MenuepunkteAnzahl++;}
  if (DREHTELLER) {MenuepunkteAnzahl += 2;}
  else if (OTA) {MenuepunkteAnzahl++;}
  int posmenu[MenuepunkteAnzahl];
  const char *menuepunkte[MenuepunkteAnzahl] = {LANGUAGE1[lingo], TAREVALUES[lingo], CALIBRATION[lingo], FILL_QUANTITY[lingo], AUTOMATIC[lingo], SERVOSETTINGS[lingo], PARAMETER[lingo], COUNTER[lingo], COUNTER_TRIP[lingo], ABOUT[lingo], CLEAR_PREFS[lingo]};
  if (ina219_installed) {
    int offset = 0;
    if (DREHTELLER) {offset = 2;}
    else if (OTA) {offset = 1;}
    menuepunkte[MenuepunkteAnzahl - 1 - offset] = menuepunkte[MenuepunkteAnzahl - 2 - offset];    //Clear Pref eins nach hinten schieben
    menuepunkte[MenuepunkteAnzahl - 2 - offset] = menuepunkte[MenuepunkteAnzahl - 3 - offset];    //About eins nach hinten schieben
    menuepunkte[MenuepunkteAnzahl - 3 - offset] = INA219_SETUP[lingo];
  }
  if (DREHTELLER) {
    menuepunkte[MenuepunkteAnzahl - 1] = menuepunkte[MenuepunkteAnzahl - 3];    //Clear Pref zwei nach hinten schieben
    menuepunkte[MenuepunkteAnzahl - 2] = menuepunkte[MenuepunkteAnzahl - 4];    //About eins nach hinten schieben
    menuepunkte[MenuepunkteAnzahl - 4] = TURNTABLE[lingo];
    menuepunkte[MenuepunkteAnzahl - 3] = WIFI[lingo];
  }
  else if (OTA) {
    menuepunkte[MenuepunkteAnzahl - 1] = menuepunkte[MenuepunkteAnzahl - 2];    //Clear Pref eins nach hinten schieben
    menuepunkte[MenuepunkteAnzahl - 2] = menuepunkte[MenuepunkteAnzahl - 3];    //About eins nach hinten schieben
    menuepunkte[MenuepunkteAnzahl - 3] = WIFI[lingo];
  }
  mode = MODE_SETUP;
  angle = angle_min;              // Hahn schliessen
  servo_enabled = 0;              // Servo-Betrieb aus
  SERVO_WRITE(angle);
  rotary_select = SW_MENU;
  initRotaries(SW_MENU, lastpos, -1, MenuepunkteAnzahl, -1);
  while (mode == MODE_SETUP and (digitalRead(switch_setup_pin)) == HIGH) {
    if (rotaries[SW_MENU].Value < 0) {
      rotaries[SW_MENU].Value = (MenuepunkteAnzahl * ROTARY_SCALE) - 1;
    }
    else if (rotaries[SW_MENU].Value > (MenuepunkteAnzahl * ROTARY_SCALE) - 1) {
      rotaries[SW_MENU].Value = 0;
    }
    int menuitem = getRotariesValue(SW_MENU) % MenuepunkteAnzahl;
    for (i = 0; i < MenuepunkteAnzahl; i++) {
      posmenu[i] = (menuitem + i) % MenuepunkteAnzahl;
    }
    dis.process_setup(menuepunkte, posmenu, MenuepunkteAnzahl, menuitem, menuitem_old);
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
      if (menuepunkte[menuitem] == TAREVALUES[lingo])        setupTare();              // Tara 
      if (menuepunkte[menuitem] == CALIBRATION[lingo])       setupCalibration();       // Kalibrieren 
      if (menuepunkte[menuitem] == FILL_QUANTITY[lingo])     setupFuellmenge();        // Füllmenge 
      if (menuepunkte[menuitem] == AUTOMATIC[lingo])         setupAutomatik();         // Autostart/Autokorrektur konfigurieren 
      if (menuepunkte[menuitem] == SERVOSETTINGS[lingo])     setupServoWinkel();       // Servostellungen Minimum, Maximum und Feindosierung
      if (menuepunkte[menuitem] == PARAMETER[lingo])         setupParameter();         // Sonstige Einstellungen
      if (menuepunkte[menuitem] == COUNTER[lingo])           setupCounter();           // Zählwerk
      if (menuepunkte[menuitem] == COUNTER_TRIP[lingo])      setupTripCounter();       // Zählwerk Trip
      if (menuepunkte[menuitem] == INA219_SETUP[lingo])      setupINA219();            // INA219 Setup
      if (menuepunkte[menuitem] == TURNTABLE[lingo])         setupDrehteller();        // Turntable Setup
      if (menuepunkte[menuitem] == LANGUAGE1[lingo])         setupLanguage();          // Language setup
      if (menuepunkte[menuitem] == ABOUT[lingo])             setupAbout();             // About setup
      if (menuepunkte[menuitem] == WIFI[lingo])              setupWiFi();              // Setup WiFi
      setPreferences();
      if (menuepunkte[menuitem] == CLEAR_PREFS[lingo])       setupClearPrefs();        // EEPROM löschen
      initRotaries(SW_MENU,lastpos, 0,255, 1);                                         // Menu-Parameter könnten verstellt worden sein
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
  boolean full = false;
  static float weight_old2;        // Gewicht des vorhergehenden Durchlaufs A.P.
  static int weight_before;        // Gewicht des vorher gefüllten Glases
  static int collector_num = 5;    // Anzahl identischer Messungen für Nachtropfen
  int loop1 = 3;                   // anzahl alle wieviel Durchgänge die auto Servo einstellung gemacht werden soll A.P.
  int y_offset_ina = 0;
  #if DREHTELLER == 1
    unsigned long time;
  #endif
  if (mode != MODE_AUTOMATIK) {
    angle = angle_min;              // Hahn schliessen
    servo_enabled = 0;              // Servo-Betrieb aus
    SERVO_WRITE(angle);
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
        //Clear Display
        #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
          dis.process_automatic_clear_oled();
        #endif
        #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
          dis.process_automatic_clear_tft();
        #endif
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
          #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
            dis.process_automatic_connection_failed_oled();
          #endif
          #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
            dis.process_automatic_connection_failed_tft();
          #endif
          while (digitalRead(switch_betrieb_pin) == HIGH) {
            if (digitalRead(switch_betrieb_pin) == HIGH) {delay(10);}
          }
          mode = -1;
          return;
        }
        if (turntable_init == false) {
          #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
            dis.process_automatic_init_turntable_nok_oled();
          #endif
          #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
            dis.process_automatic_init_turntable_nok_tft();
          #endif
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
          #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
            dis.process_automatic_read_time_close_tripprodection_failed_oled();
          #endif
          #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
            dis.process_automatic_read_time_close_tripprodection_failed_tft();
          #endif
          while (digitalRead(switch_betrieb_pin) == HIGH) {
            if (digitalRead(switch_betrieb_pin) == HIGH) {delay(10);}
          }
        }
        esp_now_msg_recived = false;
        memset(&myReceivedMessage, 0, sizeof(myReceivedMessage));
        memset(&myMessageToBeSent, 0, sizeof(myMessageToBeSent));
      }
    #endif
    #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
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
      pos_old = -1;
      angle_ist_old = -1;
      servo_enabled_old = -1;
      auto_enabled_old = -1;
      weight_old = -9999999;
      intWeight_old = -1;
      y_pos_weight = false;
      dis.process_automatic_init_screen_tft(1);
    #endif
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
    setPreferences();                                   // falls Parameter über den Rotary verändert wurden
    #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
      jar_old = -1;                                     // Glas Typ Farbe zurücksetzen fals markiert ist
      correction_old = -99999;                          // Korrektur Farbe zurücksetzen fals markiert ist
    #endif
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
        #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
          //TFT Display update erzwingen
          jar_on_scale = false;
          weight_old = -999999;
        #endif
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
    #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999 
      dis.process_automatic_start_oled();
    #endif
    #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
      dis.process_automatic_start_tft();
    #endif
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
      #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
        dis.process_automatic_set_tare_jar_tft();
      #endif
      #if DEBUG_HM  >= 1
        Serial.print(" weight: ");                Serial.print(weight);
        Serial.print(" weight_before: ");         Serial.print(weight_before);
        Serial.print(" tareweight: ");            Serial.print(fquantity + correction + tare_jar + autocorrection_gr);
        Serial.print(" overfill_gr: ");           Serial.print(overfill_gr);
        Serial.print(" autocorrection_gr: ");     Serial.println(autocorrection_gr);
      #endif
      if (wait_befor_fill == 1 and stop_wait_befor_fill == 0) {   
        #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
          dis.process_automatic_wait_befor_fill_oled();
        #endif
        #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
          dis.process_automatic_wait_befor_fill_tft();
        #endif
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
  SERVO_WRITE(angle);
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
  #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
    dis.process_automatic_main_oled(rotary_select, SW_KORREKTUR, SW_FLUSS, SW_MENU, full, y_offset_ina);
  #endif
  #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
    dis.process_automatic_main_tft(rotary_select, SW_KORREKTUR, SW_FLUSS, SW_MENU);
  #endif
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
        SERVO_WRITE(angle_min + offset_angle);
        current_mA = GetCurrent(10);
        delay(1000);  //really a delay in the while. next code to fix :-)
      }
      alarm_overcurrent = 0;
    }
    i = 0;
    inawatchdog = 1;
  }
  setPreferences();
}

void processManualmode(void) {
  i = 0;
  int y_offset = 0;
  if (mode != MODE_HANDBETRIEB) {
    mode = MODE_HANDBETRIEB;
    angle = angle_min;          // Hahn schliessen
    servo_enabled = 0;              // Servo-Betrieb aus
    SERVO_WRITE(angle);
    rotary_select = SW_WINKEL;
    tare = 0;
    offset_angle = 0;            // Offset vom Winkel wird auf 0 gestellt
    #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
      dis.process_manualmode_tft(1);
    #endif
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
  SERVO_WRITE(angle);
  if (ina219_installed and (current_servo > 0 or show_current == 1)) {
    y_offset = 4;
  }
  #if DEBUG_HM >= 4
    Serial.print("Handbetrieb:");  
    Serial.print(" weight ");      Serial.print(weight);
    Serial.print(" angle ");       Serial.print(angle);
    Serial.print(" servo_enabled "); Serial.println(servo_enabled);
  #endif
  #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
    dis.process_manualmode_oled();
  #endif
  #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
    dis.process_manualmode_tft(2);
  #endif
  if (alarm_overcurrent) {i = 1;}
  while (i > 0) {
    inawatchdog = 0;                    //schalte die kontiunirliche INA Messung aus
    //Servo ist zu
    if (servo.read() <= angle_min  + offset_angle and offset_angle < 3) {
      while(offset_angle < 3 and current_servo < current_mA) {
        offset_angle = offset_angle + 1;
        SERVO_WRITE(angle_min + offset_angle);
        current_mA = GetCurrent(10);
        delay(1000);
      }
    }
    i = 0;
    inawatchdog = 1;
    alarm_overcurrent = 0;
  }
  setPreferences();         //irgendwo anderst setzen. es muss nicht jede änderung geschrieben werden
}

void setup() {
  // enable internal pull downs for digital inputs 
  pinMode(button_start_pin, INPUT_PULLDOWN);
  pinMode(button_stop_pin, INPUT_PULLDOWN);
  pinMode(switch_betrieb_pin, INPUT_PULLDOWN);
  pinMode(switch_setup_pin, INPUT_PULLDOWN);
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
    Serial.println("Hanimandl Start");
  #endif
  // Panik prodzedure wenn das NSV zerschossen wurde
  if (digitalRead(button_stop_pin) == HIGH) {
    while (digitalRead(button_stop_pin) == HIGH) {
      delay(1);
    }
    nvs_flash_erase();    // erase the NVS partition and...
    nvs_flash_init();     // initialize the NVS partition.
    //Da machen wir gerade einen restart
    ESP.restart();
  }
  // Preferences aus dem EEPROM lesen
  getPreferences();
  //Init display
  dis.begin();
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
  pinMode(outputSW, INPUT_PULLUP);
  attachInterrupt(outputSW, isr1, FALLING);
  pinMode(outputA,INPUT);
  pinMode(outputB,INPUT);
  attachInterrupt(outputA, isr2, CHANGE);
  // Buzzer
  pinMode(buzzer_pin, OUTPUT);
  pinMode(led_pin, OUTPUT);
  // short delay to let chip power up
  delay (100);
  // Servo initialisieren und schliessen
  #if SERVO_ERWEITERT == 1
    servo.attach(servo_pin,  750, 2500); // erweiterte Initialisierung, steuert nicht jeden Servo an
    servo.setPeriodHertz(100);
  #else
    servo.attach(servo_pin, 1000, 2000); // default Werte. Achtung, steuert den Nullpunkt weniger weit aus!  
  #endif
  SERVO_WRITE(angle_min);
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
    delay(3000);
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
      strcpy(myMessageToBeSent.text, "close_drop_prodection");  //braucht keine Wartezeit
      espnow_send_data();
      while (millis() - turntable_millis < 1000 and esp_now_msg_recived == false) {  //brechce ab wenn in 10 sek keine Rückmeldung kommt
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
  if (buzzermode == 1 or ledmode == 1) {
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