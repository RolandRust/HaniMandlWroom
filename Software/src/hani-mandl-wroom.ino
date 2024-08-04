// Waage erkennen

/*
  HaniMandl Version W.0.2
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
                               - Drehteller
                             
  
                               

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
#include <Arduino_GFX_Library.h>    /* aus dem Bibliotheksverwalter */

// Version
String version = "W.0.3";

//
// Usereinstellung
//
// Könnt ihr auf eins lassen. User 2 und User 3 haben andere Glaseinstellungen
//
#define USER 3                    // 1 = Hanimandl Standart (ist die default einstellung so wie Ihr es gewohnt seit)
                                  // 2 = Gerold (Wird bei euch nicht funktionieren, da die Logos fehlen im ResPos)
                                  // 3 = Roli

//
// Hier den Code auf die verwendete Hardware einstellen
//
#define HARDWARE_LEVEL 1          // 1 = ESP32-WROOM 38Pin Connector
#define SCALE_TYP 2               // 1 = 2kg Wägezelle
                                  // 2 = 5kg Wägezelle
#define SERVO_ERWEITERT           // definieren, falls die Hardware mit dem alten Programmcode mit Poti aufgebaut wurde oder der Servo zu wenig fährt
                                  // Sonst bleibt der Servo in Stop-Position einige Grad offen! Nach dem Update erst prüfen!
#define ROTARY_SCALE 1            // in welchen Schritten springt unser Rotary Encoder. 
                                  // Beispiele: KY-040 = 2, HW-040 = 1, für Poti-Betrieb auf 1 setzen
#define DISPLAY_TYPE 99            // 1 = 128x64 pixel OLED Display angeschlossen über I2C
                                  // 2 = 128x64 pixel OLED Display angeschlossen über SPI
                                  // 3 = 320x240 pixel TFT Display ST7789 angeschlossen über SPI
                                  // 99 = Oled über I2C und TFT über SPI für development
#define OTA 1                     // 0 = OTA Uptade ausgeschalten
                                  // 1 = OTA Update eingeschalten
#define DREHTELLER 1              // 0 = kein Drehteller
                                  // 1 = Drehteller vorhanden
//#define FEHLERKORREKTUR_WAAGE   // falls Gewichtssprünge auftreten, können diese hier abgefangen werden
                                  // Achtung, kann den Wägeprozess verlangsamen. Vorher Hardware prüfen.
//#define QUETSCHHAHN_LINKS       // Servo invertieren, falls der Quetschhahn von links geöffnet wird. Mindestens ein Exemplar bekannt

//
// Ende Benutzereinstellungen!
// 
//
// Ab hier nur verstellen wenn Du genau weisst, was Du tust!
//
//#define isDebug 1             // serielle debug-Ausgabe aktivieren. Mit > 3 wird jeder Messdurchlauf ausgegeben
                                // mit 4 zusätzlich u.a. Durchlaufzeiten
                                // mit 5 zusätzlich rotary debug-Infos
                                // ACHTUNG: zu viel Serieller Output kann einen ISR-Watchdog Reset auslösen!
#define ESP_NOW_DEBUG

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
#ifdef QUETSCHHAHN_LINKS
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
#if DISPLAY_TYPE == 1
  U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* clock=*/ 22, /* data=*/ 21, /* reset=*/ U8X8_PIN_NONE);
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
#else
  #error Kein Display definiert
#endif

//Sprache
#include "./Resources/resources.h"

//OTA
#if OTA == 1
  #include "./Resources/wifi.h"
  #include <WiFi.h>
  #include <WiFiClient.h>
  #include <WebServer.h>
  #include <ElegantOTA.h>                 /* aus dem Bibliotheksverwalter */
  WebServer server(80);
  unsigned long ota_progress_millis = 0;
  void OTASetup(void);
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

// ** Definition der pins 
// ----------------------

// ESP32-WROOM 38Pin Connector
//
#if HARDWARE_LEVEL == 1
  // Rotary Encoder
  const int outputA  = 33;
  const int outputB  = 26;
  const int outputSW = 32;
  // Servo
  const int servo_pin = 2;
  // 3x Schalter Ein 1 - Aus - Ein 2, VCC pins wurden direkt auf VCC gelegt
  const int switch_betrieb_pin = 15;
  const int switch_setup_pin   = 4;
  // Taster, VCC pins wurden direkt auf VCC gelegt
  const int button_start_pin     = 12;
  const int button_stop_pin      = 27;
  // Wägezelle-IC 
  const int hx711_sck_pin = 17;
  const int hx711_dt_pin  = 35;
  // Buzzer and LED
  static int buzzer_pin = 25;
  int led_pin = 0;
#else
  #error Hardware Level nicht definiert! Korrektes #define setzen!
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
struct rotary rotaries[4];         // Werden in setup() initialisiert
int rotary_select = SW_WINKEL;
static boolean rotating = false;   // debounce management für Rotary Encoder

// Füllmengen für 5 verschiedene Gläser
struct glas { 
  int Gewicht;
  int GlasTyp;
  int Tare;
  int TripCount;
  int Count;
};
#if USER == 2
const char *GlasTypArray[2] = {"TOF", "SPZ"};
struct glas glaeser[5] = { 
                            {50, 0, -9999, 0, 0},
                            {125, 0, -9999, 0, 0},
                            {250, 0, -9999, 0, 0},
                            {500, 0, -9999, 0, 0},
                            {1000, 0, -9999, 0, 0} 
                          };
#elif USER == 3
const char *GlasTypArray[1] = {"TOF"};
struct glas glaeser[5] = { 
                            {250, 0, -9999, 0, 0},
                            {500, 0, -9999, 0, 0},
                            {0, 0, -9999, 0, 0},
                            {0, 0, -9999, 0, 0},
                            {0, 0, -9999, 0, 0} 
                          };
#else
const char *GlasTypArray[3] = {"DIB", "TOF", "DEE"};
struct glas glaeser[5] = { 
                            {125, 0, -9999, 0, 0},
                            {250, 1, -9999, 0, 0},
                            {250, 2, -9999, 0, 0},
                            {500, 1, -9999, 0, 0},
                            {500, 0, -9999, 0, 0} 
                          };
#endif

// Allgemeine Variablen
int i;                              // allgemeine Zählvariable
int pos;                            // aktuelle Position des Poti bzw. Rotary 
int gewicht;                        // aktuelles Gewicht
int tare;                           // Tara für das ausgewählte Glas, für Automatikmodus
int tare_glas;                      // Tara für das aktuelle Glas, falls Glasgewicht abweicht
long gewicht_leer;                  // Gewicht der leeren Waage
float faktor;                       // Skalierungsfaktor für Werte der Waage
int fmenge;                         // ausgewählte Füllmenge
int fmenge_index;                   // Index in gläser[]
int korrektur;                      // Korrekturwert für Abfüllmenge
int autostart;                      // Vollautomatik ein/aus
int autokorrektur;                  // Autokorrektur ein/aus
int intGewicht;                     // A.P. Durchfluss pro Durchlauf welches fließen soll (g / Zeiteinheit) Abfüllgeschwindigkeit, dies soll eine konstante Geschwindigkeit beim abfüllen geben egal bei welchem Füllstand.
int intGewichtD = 1;                // A.P. erlaubte Abweichung zu intGewicht  
int Winkelmerker;                   // A.P. Merker für optimalen Winkel für nächste Befüllung
int loop1_c;                        // A.P. Zähler für die Durchgänge  
int kulanz_gr;                      // gewollte Überfüllung im Autokorrekturmodus in Gramm
int winkel;                         // aktueller Servo-Winkel
int winkel_hard_min = 0;            // Hard-Limit für Servo
int winkel_hard_max = 180;          // Hard-Limit für Servo
int winkel_min = 0;                 // konfigurierbar im Setup
int winkel_max = 85;                // konfigurierbar im Setup
int winkel_fein = 35;               // konfigurierbar im Setup
float fein_dosier_gewicht = 60;     // float wegen Berechnung des Schliesswinkels
int servo_aktiv = 0;                // Servo aktivieren ja/nein
#if USER == 2
int kali_gewicht = 2000;            // frei wählbares Gewicht zum kalibrieren
#elif USER == 3
int kali_gewicht = 500;             // frei wählbares Gewicht zum kalibrieren
#else
int kali_gewicht = 500;             // frei wählbares Gewicht zum kalibrieren
#endif
char ausgabe[30];                   // Fontsize 12 = 13 Zeichen maximal in einer Zeile
int modus = -1;                     // Bei Modus-Wechsel den Servo auf Minimum fahren
int auto_aktiv = 0;                 // Für Automatikmodus - System ein/aus?
int waage_vorhanden = 0;            // HX711 nicht ansprechen, wenn keine Waage angeschlossen ist, sonst Crash
long preferences_chksum;            // Checksumme, damit wir nicht sinnlos Prefs schreiben
int buzzermode = 0;                 // 0 = aus, 1 = ein.
int ledmode = 0;                    // 0 = aus, 1 = ein.
bool gezaehlt = true;               // Kud Zähl-Flag
int glastoleranz = 20;              // Gewicht für autostart darf um +-20g schwanken, insgesamt also 40g!
int MenuepunkteAnzahl;              // Anzahl Menüpunkte im Setupmenü
int lastpos = 0;                    // Letzte position im Setupmenü
int progressbar = 0;                // Variable für den Progressbar
int showlogo = 1;                   // 0 = aus, 1 = ein
int showcredits = 1;                // 0 = aus, 1 = ein
int ina219_installed = 0;           // 0 = kein INA219 instaliert, 1 = INA219 ist instaliert
int current_servo = 0;              // 0 = INA219 wird ignoriert, 1-1000 = erlaubter Maximalstrom vom Servo in mA
int current_mA;                     // Strom welcher der Servo zieht
int updatetime_ina219 = 500;        // Zeit in ms in welchem der INA219 gelesen wird (500 -> alle 0,5 sek wird eine Strommessung vorgenommen)
int last_ina219_measurement = 0;    // Letzte Zeit in welcher der Strom gemessen wurde
int overcurrenttime = 1500;         // Zeit in welcher der Servo mehr Strom ziehen darf als einfestellt in ms
int last_overcurrenttime = 0;       // Letzte Zeit in welcher keinen Ueberstrom gemessen wurde
int alarm_overcurrent = 0;          // Alarmflag wen der Servo zuwiel Strom zieht
int show_current = 0;               // 0 = aus, 1 = ein / Zeige den Strom an auch wenn der INA ignoriert wird
int inawatchdog = 1;                // 0 = aus, 1 = ein / wird benötigt um INA messung auszusetzen
int offset_winkel = 0;              // Offset in Grad vom Schlieswinkel wen der Servo Übersrom hatte (max +3Grad vom eingestelten Winkel min)
int color_scheme = 0;               // 0 = dunkel, 1 = hell / Wechsel vom color scheme für den TFT Display
int color_marker = 2;               // Farbe für den Marker für das TFT Display
int ota_done = 0;                   // Variable für OTA Update
int use_turntable = 0;              // 0 = Drehteller wird nicht benützt, 1 = Drehteller wird benützt
bool turntable_init = false;        // false = Drehteller Init nicht gemacht, true = Drehteller Init wurde gemacht.
bool turntable_moving = false;
bool drop_prodection = false;       // false = Tropfschutz ist offen, true = Tropfschutz ist zu
bool turntable_ok = true;           // true wenn Glass verschoben und Tropfschutz offen ist. Wenn Drehteller nicht benützt wird ist der Wert default auf true
bool turntable_init_check = true;
bool esp_now_ini = false;           // true wenn esp now aktive ist
char esp_now_textMsg[] = "";        // send String für ESP NOW 
bool esp_now_msg_recived = false;   // true wenn etwas empfangen wurde
bool esp_now_send_error = true;     // false = Daten wurden versendet, true = Daten wurden nicht versendet.
int lingo = 0;                      // Variable for the language

//Variablen für TFT update
bool no_ina;
int gewicht_alt;
int winkel_min_alt;
int pos_alt;
int winkel_ist_alt;
int tare_alt;
int current_mA_alt;
int servo_aktiv_alt;
int auto_aktiv_alt;
int glas_alt;
int korr_alt;
int autokorr_gr_alt;
int intGewicht_alt;

//Color Scheme für den TFT Display
unsigned long  BACKGROUND;
unsigned long  TEXT;
unsigned long  MENU_POS1;
unsigned long  MENU_POS2;
unsigned long  MENU_POS3;
unsigned long  MENU_POS4;
unsigned long  MARKER;
//#define MARKER 0x1AF6

#if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
  void tft_colors() {
    if (color_scheme == 0) {          //dunkel
      BACKGROUND  = 0x0000;
      TEXT        = 0xFFFF;
      MENU_POS1   = 0x73AF;
      MENU_POS2   = 0x5AEC;
      MENU_POS3   = 0x39C8;
      MENU_POS4   = 0x20E6;
    }
    else {                            //hell
      BACKGROUND  = 0xFFFF;
      TEXT        = 0x0000;
      MENU_POS1   = 0x1082;
      MENU_POS2   = 0x2104;
      MENU_POS3   = 0x3166;
      MENU_POS4   = 0x4208;
    }
  }

  void tft_marker() {                
    if (color_marker == 0)                              {MARKER = 0x05ff;}
    else if (color_marker == 1)                         {MARKER = 0x021f;}
    else if (color_marker == 2)                         {MARKER = 0x001f;}
    else if (color_marker == 3)                         {MARKER = 0x4851;}
    else if (color_marker == 4)                         {MARKER = 0xc050;}
    else if (color_marker == 5)                         {MARKER = 0xf800;}
    else if (color_marker == 6)                         {MARKER = 0xfbc0;}
    else if (color_marker == 7)                         {MARKER = 0xfde0;}
    else if (color_marker == 8)                         {MARKER = 0xd6e1;}
    else if (color_marker == 9)                         {MARKER = 0xffe0;}
    else if (color_marker == 10)                        {MARKER = 0x2724;}
    else if (color_marker == 11)                        {MARKER = 0x0da1;}
  }
#endif

//Drehteller
#if DREHTELLER == 1
  #include "./Resources/mac.h"
  #if OTA == 0
    #include <WiFi.h>
  #endif
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
  #ifdef ESP_NOW_DEBUG
    Serial.print("Send status: ");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success":"Fail");
  #endif
  drop_prodection = true;
  }
  void messageReceived(const uint8_t* macAddr, const uint8_t* incomingData, int len){
    memset(&myReceivedMessage, 0, sizeof(myReceivedMessage));
    memcpy(&myReceivedMessage, incomingData, sizeof(myReceivedMessage));
    esp_now_msg_recived = true;
    #ifdef ESP_NOW_DEBUG
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
    if ( modus == MODE_AUTOMATIK && servo_aktiv == 0 ) { // nur im Automatik-Modus interessiert uns der Click
      rotary_select = (rotary_select + 1) % 4;
      #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
        if      (rotary_select == SW_KORREKTUR) {korr_alt = -99999;}
        else if (rotary_select == SW_FLUSS)     {intGewicht_alt = -1; korr_alt = -99999;}
        else if (rotary_select == SW_MENU)      {glas_alt = -1; intGewicht_alt = -1;}
        else {korr_alt = -99999; intGewicht_alt = -1; glas_alt = -1;}
      #endif
      #ifdef isDebug
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
  static int aState;
  static int aLastState = 2;  // reale Werte sind 0 und 1
  aState = digitalRead(outputA); // Reads the "current" state of the outputA
  if (aState != aLastState) {     
    // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
    if (digitalRead(outputB) != aState) {
      rotaries[rotary_select].Value += rotaries[rotary_select].Step;
    }
    else {    // counter-clockwise
      rotaries[rotary_select].Value -= rotaries[rotary_select].Step;
    }
    rotaries[rotary_select].Value = constrain( rotaries[rotary_select].Value, rotaries[rotary_select].Minimum, rotaries[rotary_select].Maximum);
    rotating = false;
    #ifdef isDebug
      #if isDebug >= 4
        Serial.print(" Rotary Value changed to ");
        Serial.println(getRotariesValue(rotary_select));
      #endif
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
  #ifdef isDebug
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
  preferences.begin("EEPROM", false);                     // Parameter aus dem EEPROM lesen
  faktor          = preferences.getFloat("faktor", 0.0);  // falls das nicht gesetzt ist -> Waage ist nicht kalibriert
  pos             = preferences.getUInt("pos", 0);
  Serial.println(pos);
  gewicht_leer    = preferences.getUInt("gewicht_leer", 0); 
  korrektur       = preferences.getUInt("korrektur", 0);
  autostart       = preferences.getUInt("autostart", 0);
  autokorrektur   = preferences.getUInt("autokorrektur", 0);
  intGewicht      = preferences.getUInt("intGewicht", intGewicht);  // bei 0 aus.A.P.
  kulanz_gr       = preferences.getUInt("kulanz_gr", 5);
  fmenge_index    = preferences.getUInt("fmenge_index", 3);
  winkel_min      = preferences.getUInt("winkel_min", winkel_min);
  winkel_max      = preferences.getUInt("winkel_max", winkel_max);
  winkel_fein     = preferences.getUInt("winkel_fein", winkel_fein);
  buzzermode      = preferences.getUInt("buzzermode", buzzermode);
  ledmode         = preferences.getUInt("ledmode", ledmode);
  showlogo        = preferences.getUInt("showlogo", showlogo);
  showcredits     = preferences.getUInt("showcredits", showcredits);
  kali_gewicht    = preferences.getUInt("kali_gewicht", kali_gewicht);
  glastoleranz    = preferences.getUInt("glastoleranz", glastoleranz);
  current_servo   = preferences.getUInt("current_servo", current_servo);
  show_current    = preferences.getUInt("show_current", show_current);
  color_scheme    = preferences.getUInt("color_scheme", color_scheme);
  color_marker    = preferences.getUInt("color_marker", color_marker);
  use_turntable   = preferences.getUInt("use_turntable", use_turntable);
  lingo           = preferences.getUInt("lingo", lingo);
  preferences_chksum = faktor + pos + gewicht_leer + korrektur + autostart + autokorrektur + intGewicht + kulanz_gr + fmenge_index +
                       winkel_min + winkel_max + winkel_fein + buzzermode + ledmode + showlogo + showcredits +  kali_gewicht +
                       current_servo + glastoleranz + show_current + color_scheme + color_marker + use_turntable + lingo;
  i = 0;
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
  while( i < 5) {
    sprintf(ausgabe, "Gewicht%d", i);
    glaeser[i].Gewicht = preferences.getInt(ausgabe, ResetGewichte[i]);
    preferences_chksum += glaeser[i].Gewicht;
    sprintf(ausgabe, "GlasTyp%d", i);
    glaeser[i].GlasTyp = preferences.getInt(ausgabe, ResetGlasTyp[i]);
    preferences_chksum += glaeser[i].GlasTyp;
    sprintf(ausgabe, "Tara%d", i);
    glaeser[i].Tare= preferences.getInt(ausgabe, -9999);
    preferences_chksum += glaeser[i].Tare;
    sprintf(ausgabe, "TripCount%d", i);
    glaeser[i].TripCount = preferences.getInt(ausgabe, 0);
    preferences_chksum += glaeser[i].TripCount;
    sprintf(ausgabe, "Count%d", i);
    glaeser[i].Count = preferences.getInt(ausgabe, 0);
    preferences_chksum += glaeser[i].Count;
    i++;
  }
  preferences.end();
  #ifdef isDebug
    Serial.println("get Preferences:");
    Serial.print("pos = ");             Serial.println(pos);
    Serial.print("faktor = ");          Serial.println(faktor);
    Serial.print("gewicht_leer = ");    Serial.println(gewicht_leer);
    Serial.print("korrektur = ");       Serial.println(korrektur);
    Serial.print("autostart = ");       Serial.println(autostart);
    Serial.print("autokorrektur = ");   Serial.println(autokorrektur);
    Serial.print("intGewicht = ");      Serial.println(intGewicht);  //A.P.
    Serial.print("kulanz_gr = ");       Serial.println(kulanz_gr);
    Serial.print("fmenge_index = ");    Serial.println(fmenge_index);
    Serial.print("winkel_min = ");      Serial.println(winkel_min);
    Serial.print("winkel_max = ");      Serial.println(winkel_max);
    Serial.print("winkel_fein = ");     Serial.println(winkel_fein);
    Serial.print("buzzermode = ");      Serial.println(buzzermode);
    Serial.print("ledmode = ");         Serial.println(ledmode);
    Serial.print("showlogo = ");        Serial.println(showlogo);
    Serial.print("showcredits = ");     Serial.println(showcredits);
    Serial.print("current_servo = ");   Serial.println(current_servo);
    Serial.print("color_scheme = ");    Serial.println(color_scheme);
    Serial.print("color_marker = ");    Serial.println(color_marker);
    Serial.print("kali_gewicht = ");    Serial.println(kali_gewicht);
    Serial.print("glastoleranz = ");    Serial.println(glastoleranz);
    Serial.print("show_current = ");    Serial.println(show_current);
    Serial.print("use_turntable = ");   Serial.println(use_turntable);
    Serial.print("lingo = ");           Serial.println(lingo);
    i = 0;
    while( i < 5 ) {
      sprintf(ausgabe, "Gewicht%d = ", i);
      Serial.print(ausgabe);
      Serial.println(glaeser[i].Gewicht);
      sprintf(ausgabe, "GlasTyp%d = ", i);
      Serial.print(ausgabe);         
      Serial.println(GlasTypArray[glaeser[i].GlasTyp]);
      sprintf(ausgabe, "Tara%d = ", i);
      Serial.print(ausgabe);         
      Serial.println(glaeser[i].Tare);
      i++;
    }
    Serial.print("Checksumme:");    
    Serial.println(preferences_chksum);    
  #endif
}

void setPreferences(void) {
  long preferences_newchksum;
  int winkel = getRotariesValue(SW_WINKEL);
  int i;
  preferences.begin("EEPROM", false);
  // Winkel-Einstellung separat behandeln, ändert sich häufig
  if ( winkel != preferences.getUInt("pos", 0) ) {
    preferences.putUInt("pos", winkel);
    #ifdef isDebug
      Serial.print("winkel gespeichert: ");
      Serial.println(winkel);
    #endif
  }
  // Counter separat behandeln, ändert sich häufig
  for ( i=0 ; i < 5; i++ ) {
    sprintf(ausgabe, "TripCount%d", i);
    if (glaeser[i].TripCount != preferences.getInt(ausgabe, 0)) {
      preferences.putInt(ausgabe, glaeser[i].TripCount);
    }
    sprintf(ausgabe, "Count%d", i);
    if (glaeser[i].Count != preferences.getInt(ausgabe, 0)) {
      preferences.putInt(ausgabe, glaeser[i].Count);
    }
    #ifdef isDebug
      Serial.print("Counter gespeichert: Index ");
      Serial.print(i);
      Serial.print(" Trip ");
      Serial.print(glaeser[fmenge_index].TripCount);
      Serial.print(" Gesamt ");
      Serial.println(glaeser[fmenge_index].Count);      
    #endif
  }
  // Den Rest machen wir gesammelt, das ist eher statisch
  preferences_newchksum = faktor + gewicht_leer + korrektur + autostart + autokorrektur + intGewicht +
                          fmenge_index + winkel_min + winkel_max + winkel_fein + kulanz_gr +
                          buzzermode + ledmode + showlogo + showcredits + current_servo + kali_gewicht + 
                          glastoleranz + show_current + color_scheme + color_marker + use_turntable + lingo;
  i = 0;
  while( i < 5 ) {
    preferences_newchksum += glaeser[i].Gewicht;
    preferences_newchksum += glaeser[i].GlasTyp;
    preferences_newchksum += glaeser[i].Tare;
    i++;
  }
  if( preferences_newchksum == preferences_chksum ) {
    #ifdef isDebug
      Serial.println("Preferences unverändert");
    #endif
    return;
  }
  preferences_chksum = preferences_newchksum;
  preferences.putFloat("faktor", faktor);
  preferences.putUInt("gewicht_leer", gewicht_leer);
  preferences.putUInt("korrektur", korrektur);
  preferences.putUInt("autostart", autostart);
  preferences.putUInt("autokorrektur", autokorrektur);
  preferences.putUInt("intGewicht", intGewicht); //A.P.
  preferences.putUInt("kulanz_gr", kulanz_gr);
  preferences.putUInt("winkel_min", winkel_min);
  preferences.putUInt("winkel_max", winkel_max);
  preferences.putUInt("winkel_fein", winkel_fein);
  preferences.putUInt("fmenge_index", fmenge_index);
  preferences.putUInt("buzzermode", buzzermode);
  preferences.putUInt("ledmode", ledmode);
  preferences.putUInt("showlogo", showlogo);
  preferences.putUInt("showcredits", showcredits);
  preferences.putUInt("kali_gewicht", kali_gewicht);
  preferences.putUInt("glastoleranz", glastoleranz);
  preferences.putUInt("current_servo", current_servo);
  preferences.putUInt("show_current", show_current);
  preferences.putUInt("color_scheme", color_scheme);
  preferences.putUInt("color_marker", color_marker);
  preferences.putUInt("use_turntable", use_turntable);
  preferences.putUInt("lingo", lingo);
  i = 0;
  while( i < 5 ) {
    sprintf(ausgabe, "Gewicht%d", i);
    preferences.putInt(ausgabe, glaeser[i].Gewicht);
    sprintf(ausgabe, "GlasTyp%d", i);
    preferences.putInt(ausgabe, glaeser[i].GlasTyp);  
    sprintf(ausgabe, "Tara%d", i);
    preferences.putInt(ausgabe, glaeser[i].Tare);
    i++;
  }
  preferences.end();
  #ifdef isDebug
    Serial.println("Set Preferences:");
    Serial.print("pos = ");             Serial.println(winkel);
    Serial.print("faktor = ");          Serial.println(faktor);
    Serial.print("gewicht_leer = ");    Serial.println(gewicht_leer);
    Serial.print("korrektur = ");       Serial.println(korrektur);
    Serial.print("autostart = ");       Serial.println(autostart);
    Serial.print("intGewicht = ");      Serial.println(intGewicht); //A.P.
    Serial.print("autokorrektur = ");   Serial.println(autokorrektur);
    Serial.print("kulanz_gr = ");       Serial.println(kulanz_gr);
    Serial.print("fmenge_index = ");    Serial.println(fmenge_index);
    Serial.print("winkel_min = ");      Serial.println(winkel_min);
    Serial.print("winkel_max = ");      Serial.println(winkel_max);
    Serial.print("winkel_fein = ");     Serial.println(winkel_fein);
    Serial.print("buzzermode = ");      Serial.println(buzzermode);
    Serial.print("ledmode = ");         Serial.println(ledmode);
    Serial.print("showlogo = ");        Serial.println(showlogo);
    Serial.print("showcredits = ");     Serial.println(showcredits);
    Serial.print("current_servo = ");   Serial.println(current_servo);
    Serial.print("kali_gewicht = ");    Serial.println(kali_gewicht);
    Serial.print("glastoleranz = ");    Serial.println(glastoleranz);
    Serial.print("show_current = ");    Serial.println(show_current);
    Serial.print("color_scheme = ");    Serial.println(color_scheme);
    Serial.print("color_marker = ");    Serial.println(color_marker);
    Serial.print("use_turntable = ");   Serial.println(use_turntable);
    Serial.print("lingo = ");           Serial.println(lingo);
    i = 0;
    while( i < 5 ) {
      sprintf(ausgabe, "Gewicht%d = ", i);
      Serial.print(ausgabe);         Serial.println(glaeser[i].Gewicht);
      sprintf(ausgabe, "GlasTyp%d = ", i);
      Serial.print(ausgabe);         Serial.println(GlasTypArray[glaeser[i].GlasTyp]);
      sprintf(ausgabe, "Tara%d = ", i);
      Serial.print(ausgabe);         Serial.println(glaeser[i].Tare);
      i++;
    }
  #endif
}

void setupTripCounter(void) {
  int j;
  i = 1;
  float TripAbfuellgewicht = 0;
  #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
    int y_offset_tft = 28;
    gfx->fillScreen(BACKGROUND);
    gfx->setTextColor(TEXT);
    gfx->setFont(Punk_Mono_Bold_240_150);
    int x_pos = CenterPosX(COUNTER_TRIP[lingo], 14, 320);
    gfx->setCursor(x_pos, 25);
    gfx->println(COUNTER_TRIP[lingo]);
    gfx->drawLine(0, 30, 320, 30, TEXT);
  #endif
  while (i > 0) { //Erster Screen: Anzahl pro Glasgröße
    if (digitalRead(button_stop_pin) == HIGH or digitalRead(switch_setup_pin) == LOW) {
      while (digitalRead(button_stop_pin) == HIGH) {
        delay(1);
      }
      modus = -1;
      return;
    }
    #if DISPLAY_TYPE == 1 or DISPLAY_TYPE == 2 or DISPLAY_TYPE == 99
      j = 0;
      u8g2.setFont(u8g2_font_courB08_tf);
      u8g2.clearBuffer();
      while (j < 5) {
        u8g2.setCursor(1, 10 + (j * 13));
        sprintf(ausgabe, "%4dg %3s", glaeser[j].Gewicht, GlasTypArray[glaeser[j].GlasTyp]);
        u8g2.print(ausgabe);
        u8g2.setCursor(50, 10 + (j * 13));
        sprintf(ausgabe, "%9d St.", glaeser[j].TripCount);
        u8g2.print(ausgabe);
        j++;
      }
      u8g2.sendBuffer();
    #endif
    #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
      j = 0;
      while ( j < 5 ) {
        gfx->setCursor(5, 30+((j+1) * y_offset_tft));
        sprintf(ausgabe, "%4dg %3s", glaeser[j].Gewicht,GlasTypArray[glaeser[j].GlasTyp]);
        gfx->print(ausgabe);
        gfx->setCursor(110, 30+((j+1) * y_offset_tft));
        sprintf(ausgabe, "%11d St.", glaeser[j].TripCount);
        gfx->print(ausgabe);
        j++;
      }
    #endif
    if (digitalRead(SELECT_SW) == SELECT_PEGEL) {
      //verlasse Screen
      while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
        delay(1);
      }
      i = 0;
      #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
        gfx->fillRect(0, 31, 320, 209, BACKGROUND);
      #endif
    }
  }
  i = 1;
  while (i > 0) { //Zweiter Screen: Gewicht pro Glasgröße
    if (digitalRead(button_stop_pin) == HIGH or digitalRead(switch_setup_pin) == LOW) {
      while (digitalRead(button_stop_pin) == HIGH) {
        delay(1);
      }
      modus = -1;
      return;
    }
    #if DISPLAY_TYPE == 1 or DISPLAY_TYPE == 2 or DISPLAY_TYPE == 99
      j = 0;
      u8g2.setFont(u8g2_font_courB08_tf);
      u8g2.clearBuffer();
      while (j < 5) {
        u8g2.setCursor(1, 10 + (j * 13));
        sprintf(ausgabe, "%4dg %3s", glaeser[j].Gewicht,GlasTypArray[glaeser[j].GlasTyp]);
        u8g2.print(ausgabe);
        u8g2.setCursor(65, 10 + (j * 13));
        sprintf(ausgabe, "%8.3fkg", glaeser[j].Gewicht * glaeser[j].TripCount / 1000.0);
        u8g2.print(ausgabe);
        j++;
      }
      u8g2.sendBuffer();
    #endif
    #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
      j = 0;
      while ( j < 5 ) {
        gfx->setCursor(5, 30+((j+1) * y_offset_tft));
        sprintf(ausgabe, "%4dg %3s", glaeser[j].Gewicht,GlasTypArray[glaeser[j].GlasTyp]);
        gfx->print(ausgabe);
        gfx->setCursor(150, 30+((j+1) * y_offset_tft));
        sprintf(ausgabe, "%10.3fkg", glaeser[j].Gewicht * glaeser[j].TripCount / 1000.0);
        gfx->print(ausgabe);
        j++;
      }
    #endif
    if (digitalRead(SELECT_SW) == SELECT_PEGEL) {
      //verlasse Screen
      while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
        delay(1);
      }
      i = 0;
      #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
        gfx->fillRect(0, 31, 320, 209, BACKGROUND);
      #endif
    }
  }
  i = 1;
  while (i > 0) { //Dritter Screen: Gesamtgewicht
    TripAbfuellgewicht = 0;
    j = 0;
    if (digitalRead(button_stop_pin) == HIGH or digitalRead(switch_setup_pin) == LOW) {
      while (digitalRead(button_stop_pin) == HIGH) {
        delay(1);
      }
      modus = -1;
      return;
    }
    while (j < 5) {
      TripAbfuellgewicht += glaeser[j].Gewicht * glaeser[j].TripCount / 1000.0;
      j++;
    }
    #if DISPLAY_TYPE == 1 or DISPLAY_TYPE == 2 or DISPLAY_TYPE == 99
      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_courB14_tf);
      u8g2.setCursor(5, 15);
      u8g2.print(TOTAL[lingo]);
      u8g2.setFont(u8g2_font_courB18_tf);
      u8g2.setCursor(10, 50);
      sprintf(ausgabe, "%5.1fkg", TripAbfuellgewicht);
      u8g2.print(ausgabe);
      u8g2.sendBuffer();
    #endif
    #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
      gfx->setFont(Punk_Mono_Bold_600_375);
      sprintf(ausgabe, TOTAL[lingo]);
      x_pos = CenterPosX(ausgabe, 36, 320);
      gfx->setCursor(x_pos, 124);
      gfx->print(ausgabe);
      sprintf(ausgabe, "%.1fkg", TripAbfuellgewicht);
      x_pos = CenterPosX(ausgabe, 36, 320);
      gfx->setCursor(x_pos, 184);
      gfx->print(ausgabe);
      gfx->setFont(Punk_Mono_Bold_240_150);
    #endif
    if (digitalRead(SELECT_SW) == SELECT_PEGEL) {
      //verlasse Screen
      while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
        delay(1);
      }
      i = 0;
      #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
        gfx->fillRect(0, 31, 320, 209, BACKGROUND);
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
        modus = -1;
        return;
      }
      pos = getRotariesValue(SW_MENU);
      #if DISPLAY_TYPE == 1 or DISPLAY_TYPE == 2 or DISPLAY_TYPE == 99
        u8g2.setFont(u8g2_font_courB10_tf);
        u8g2.clearBuffer();
        u8g2.setCursor(10, 12);    u8g2.print(RESET[lingo]);
        u8g2.setCursor(10, 28);    u8g2.print(ABORT[lingo]);
        u8g2.setCursor(0, 12 + ((pos) * 16));
        u8g2.print("*");
        u8g2.sendBuffer();
      #endif
      #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
        gfx->setCursor(5, 30+(1 * y_offset_tft));
        if (pos == 0) {gfx->setTextColor(MARKER);}
        else {gfx->setTextColor(TEXT);}
        gfx->print(RESET[lingo]);
        gfx->setCursor(5, 30+(2 * y_offset_tft));
        if (pos == 1) {gfx->setTextColor(MARKER);}
        else {gfx->setTextColor(TEXT);}
        gfx->print(ABORT[lingo]);
      #endif
      if (digitalRead(SELECT_SW) == SELECT_PEGEL) {
        #if DISPLAY_TYPE == 1 or DISPLAY_TYPE == 2 or DISPLAY_TYPE == 99
          u8g2.setCursor(105, 12 + ((pos) * 16));
          u8g2.print("OK");
          u8g2.sendBuffer();
        #endif
        #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
          gfx->setTextColor(MARKER);
          gfx->setCursor(283, 30+((pos+1) * y_offset_tft));
          gfx->print("OK");
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
        modus = -1;
        i = 0;
      }
    }
  }
}

void setupCounter(void) {
  int j;
  i = 1;
  float Abfuellgewicht = 0;
  #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
    int y_offset_tft = 28;
    gfx->fillScreen(BACKGROUND);
    gfx->setTextColor(TEXT);
    gfx->setFont(Punk_Mono_Bold_240_150);
    int x_pos = CenterPosX(COUNTER[lingo], 14, 320);
    gfx->setCursor(x_pos, 25);
    gfx->println(COUNTER[lingo]);
    gfx->drawLine(0, 30, 320, 30, TEXT);
  #endif
  while (i > 0) { //Erster Screen: Anzahl pro Glasgröße
    if (digitalRead(button_stop_pin) == HIGH or digitalRead(switch_setup_pin) == LOW) {
      while (digitalRead(button_stop_pin) == HIGH) {
        delay(1);
      }
      modus = -1;
      return;
    }
    #if DISPLAY_TYPE == 1 or DISPLAY_TYPE == 2 or DISPLAY_TYPE == 99
      j = 0;
      u8g2.setFont(u8g2_font_courB08_tf);
      u8g2.clearBuffer();
      while ( j < 5 ) {
        u8g2.setCursor(1, 10 + (j * 13));
        sprintf(ausgabe, "%4dg %3s", glaeser[j].Gewicht,GlasTypArray[glaeser[j].GlasTyp]);
        u8g2.print(ausgabe);
        u8g2.setCursor(50, 10 + (j * 13));
        sprintf(ausgabe, "%9d St.", glaeser[j].Count);
        u8g2.print(ausgabe);
        j++;
      }
      u8g2.sendBuffer();
    #endif
    #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
      j = 0;
      while ( j < 5 ) {
        gfx->setCursor(5, 30+((j+1) * y_offset_tft));
        sprintf(ausgabe, "%4dg %3s", glaeser[j].Gewicht,GlasTypArray[glaeser[j].GlasTyp]);
        gfx->print(ausgabe);
        gfx->setCursor(110, 30+((j+1) * y_offset_tft));
        sprintf(ausgabe, "%11d St.", glaeser[j].Count);
        gfx->print(ausgabe);
        j++;
      }
    #endif
    if (digitalRead(SELECT_SW) == SELECT_PEGEL) {
      //verlasse Screen
      while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
        delay(1);
      }
      i = 0;
      #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
        gfx->fillRect(0, 31, 320, 209, BACKGROUND);
      #endif
    }
  }
  i = 1;
  while (i > 0) { //Zweiter Screen: Gewicht pro Glasgröße
    if (digitalRead(button_stop_pin) == HIGH or digitalRead(switch_setup_pin) == LOW) {
      while (digitalRead(button_stop_pin) == HIGH) {
        delay(1);
      }
      modus = -1;
      return;
    }
    #if DISPLAY_TYPE == 1 or DISPLAY_TYPE == 2 or DISPLAY_TYPE == 99
      j = 0;
      u8g2.setFont(u8g2_font_courB08_tf);
      u8g2.clearBuffer();
      while ( j < 5) {
        u8g2.setCursor(1, 10 + (j * 13));
        sprintf(ausgabe, "%4dg %3s", glaeser[j].Gewicht,GlasTypArray[glaeser[j].GlasTyp]);
        u8g2.print(ausgabe);
        u8g2.setCursor(65, 10 + (j * 13));
        sprintf(ausgabe, "%8.3fkg", glaeser[j].Gewicht * glaeser[j].Count / 1000.0);
        u8g2.print(ausgabe);
        j++;
      }
      u8g2.sendBuffer();
    #endif
    #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
      j = 0;
      while ( j < 5 ) {
        gfx->setCursor(5, 30+((j+1) * y_offset_tft));
        sprintf(ausgabe, "%4dg %3s", glaeser[j].Gewicht,GlasTypArray[glaeser[j].GlasTyp]);
        gfx->print(ausgabe);
        gfx->setCursor(150, 30+((j+1) * y_offset_tft));
        sprintf(ausgabe, "%10.3fkg", glaeser[j].Gewicht * glaeser[j].Count / 1000.0);
        gfx->print(ausgabe);
        j++;
      }
    #endif
    if (digitalRead(SELECT_SW) == SELECT_PEGEL) {
      //verlasse Screen
      while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
        delay(1);
      }
      i = 0;
      #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
        gfx->fillRect(0, 31, 320, 209, BACKGROUND);
      #endif
    }
  }
  i = 1;
  while (i > 0) { //Dritter Screen: Gesamtgewicht
    Abfuellgewicht = 0;
    j = 0;
    if (digitalRead(button_stop_pin) == HIGH  or digitalRead(switch_setup_pin) == LOW) {
      while (digitalRead(button_stop_pin) == HIGH) {
        delay(1);
      }
      modus = -1;
      return;
    }
    while ( j < 5  ) {
      Abfuellgewicht += glaeser[j].Gewicht * glaeser[j].Count / 1000.0;
      j++;
    }
    #if DISPLAY_TYPE == 1 or DISPLAY_TYPE == 2 or DISPLAY_TYPE == 99
      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_courB14_tf);
      u8g2.setCursor(1, 15);
      u8g2.print(TOTAL[lingo]);
      u8g2.setFont(u8g2_font_courB18_tf);
      u8g2.setCursor(10, 50);
      sprintf(ausgabe, "%5.1fkg", Abfuellgewicht);
      u8g2.print(ausgabe);
      u8g2.sendBuffer();
    #endif
    #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
      gfx->setFont(Punk_Mono_Bold_600_375);
      sprintf(ausgabe, TOTAL[lingo]);
      x_pos = CenterPosX(ausgabe, 36, 320);
      gfx->setCursor(x_pos, 124);
      gfx->print(ausgabe);
      sprintf(ausgabe, "%.1fkg", Abfuellgewicht);
      x_pos = CenterPosX(ausgabe, 36, 320);
      gfx->setCursor(x_pos, 184);
      gfx->print(ausgabe);
      gfx->setFont(Punk_Mono_Bold_240_150);
    #endif
    if ((digitalRead(SELECT_SW)) == SELECT_PEGEL) {
      //verlasse Screen
      while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
        delay(1);
      }
      i = 0;
      #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
        gfx->fillRect(0, 31, 320, 209, BACKGROUND);
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
        modus = -1;
        return;
      }
      pos = getRotariesValue(SW_MENU);
      #if DISPLAY_TYPE == 1 or DISPLAY_TYPE == 2 or DISPLAY_TYPE == 99
        u8g2.setFont(u8g2_font_courB10_tf);
        u8g2.clearBuffer();
        u8g2.setCursor(10, 12);    u8g2.print(RESET[lingo]);
        u8g2.setCursor(10, 28);    u8g2.print(ABORT[lingo]);
        u8g2.setCursor(0, 12 + ((pos) * 16));
        u8g2.print("*");
        u8g2.sendBuffer();
      #endif
      #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
        gfx->setCursor(5, 30+(1 * y_offset_tft));
        if (pos == 0) {gfx->setTextColor(MARKER);}
        else {gfx->setTextColor(TEXT);}
        gfx->print(RESET[lingo]);
        gfx->setCursor(5, 30+(2 * y_offset_tft));
        if (pos == 1) {gfx->setTextColor(MARKER);}
        else {gfx->setTextColor(TEXT);}
        gfx->print(ABORT[lingo]);
      #endif
      if (digitalRead(SELECT_SW) == SELECT_PEGEL) {
        while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
          delay(1);
        }
        #if DISPLAY_TYPE == 1 or DISPLAY_TYPE == 2 or DISPLAY_TYPE == 99
          u8g2.setCursor(105, 12 + ((pos) * 16));
          u8g2.print("OK");
          u8g2.sendBuffer();
        #endif
        #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
          gfx->setTextColor(MARKER);
          gfx->setCursor(283, 30+((pos+1) * y_offset_tft));
          gfx->print("OK");
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
        modus = -1;
        i = 0;
      }
    }
  }
}

void setupTare(void) {
  int j;
  int x_pos;
  tare = 0;
  initRotaries(SW_MENU, 0, 0, 4, 1);
  i = 0;
  #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
    gfx->fillScreen(BACKGROUND);
    bool change = false;
  #endif
  while (i == 0)  {
    if (digitalRead(button_stop_pin) == HIGH or digitalRead(switch_setup_pin) == LOW) {
      while (digitalRead(button_stop_pin) == HIGH) {
        delay(1);
      }
      modus = -1;
      return;
    }
    else if (digitalRead(SELECT_SW) == SELECT_PEGEL) {
      tare = int(SCALE_GETUNITS(10));
      if (tare > 20) {                  // Gläser müssen mindestens 20g haben
         glaeser[getRotariesValue(SW_MENU)].Tare= tare;
      }
      #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
        change = true;
      #endif
      i++;
    }
    #if DISPLAY_TYPE == 1 or DISPLAY_TYPE == 2 or DISPLAY_TYPE == 99
      u8g2.setFont(u8g2_font_courB08_tf);
      u8g2.clearBuffer();
      j = 0;
      while(j < 5) {
        u8g2.setCursor(8, 10+(j*13));
        if (glaeser[j].Gewicht < 1000) {
          sprintf(ausgabe, "%4dg - %3s", glaeser[j].Gewicht, GlasTypArray[glaeser[j].GlasTyp]); 
        } 
        else {
          sprintf(ausgabe, "%.1fkg - %3s", float(glaeser[j].Gewicht) / 1000, GlasTypArray[glaeser[j].GlasTyp]); 
        }
        u8g2.print(ausgabe);
        u8g2.setCursor(80, 10+(j*13));
        if (glaeser[j].Tare > 0) { 
          sprintf(ausgabe, "%7dg", glaeser[j].Tare); 
          u8g2.print(ausgabe);
        }
        else {
          sprintf(ausgabe, "%8s", MISSING); 
          u8g2.print(ausgabe);
        }
        j++;
      }
      u8g2.setCursor(0, 10+(getRotariesValue(SW_MENU)*13) );    
      u8g2.print("*");
      u8g2.sendBuffer();
    #endif
    #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
      if (change) {
        gfx->fillScreen(BACKGROUND);
      }
      gfx->setTextColor(TEXT);
      gfx->setFont(Punk_Mono_Bold_240_150);
      x_pos = CenterPosX(TAREVALUES_JAR[lingo], 14, 320);
      gfx->setCursor(x_pos, 25);
      gfx->println(TAREVALUES_JAR[lingo]);
      gfx->drawLine(0, 30, 320, 30, TEXT);
      j = 0;
      while(j < 5) {
        if (j == getRotariesValue(SW_MENU)) {
          gfx->setTextColor(MARKER);
        }
        else {
          gfx->setTextColor(TEXT);
        }
        gfx->setCursor(5, 60+(j*30));
        if (glaeser[j].Gewicht < 1000) {
          sprintf(ausgabe, "%4dg - %3s", glaeser[j].Gewicht, GlasTypArray[glaeser[j].GlasTyp]); 
        } 
        else {
          sprintf(ausgabe, "%.1fkg - %3s", float(glaeser[j].Gewicht) / 1000, GlasTypArray[glaeser[j].GlasTyp]); 
        }
        gfx->println(ausgabe);
        gfx->setCursor(165, 60+(j*30));
        if (glaeser[j].Tare > 0) { 
          sprintf(ausgabe, "%10dg", glaeser[j].Tare); 
          gfx->print(ausgabe);
        }
        else {
          sprintf(ausgabe, "%11s", MISSING); 
          gfx->print(ausgabe);
        }
        j++;
      }
    #endif
  }
  modus = -1;
  delay(2000);
}

void setupCalibration(void) {
  float gewicht_raw;
  #if DISPLAY_TYPE == 1 or DISPLAY_TYPE == 2 or DISPLAY_TYPE == 99
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_courB08_tf);
    u8g2.setCursor(0, 12);    u8g2.print(PLEASE_EMPTY_THE[lingo]);
    u8g2.setCursor(0, 24);    u8g2.print(SCALE_AND_CONFIRM[lingo]);
    u8g2.setCursor(0, 36);    u8g2.print(WITH_OK_1[lingo]);
    u8g2.sendBuffer();
  #endif
  #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
    gfx->fillScreen(BACKGROUND);
    int kali_gewicht_old = -1;
    gfx->setTextColor(TEXT);
    gfx->setFont(Punk_Mono_Bold_240_150);
    int x_pos = CenterPosX(CALIBRATION[lingo], 14, 320);
    gfx->setCursor(x_pos, 25);
    gfx->println(CALIBRATION[lingo]);
    gfx->drawLine(0, 30, 320, 30, TEXT);
    sprintf(ausgabe, PLEASE_EMPTY_THE[lingo]);
    x_pos = CenterPosX(ausgabe, 14, 320);
    gfx->setCursor(x_pos, 100); gfx->print(ausgabe);
    sprintf(ausgabe, SCALE_AND_CONFIRM[lingo]);
    x_pos = CenterPosX(ausgabe, 14, 320);
    gfx->setCursor(x_pos, 128); gfx->print(ausgabe);
    sprintf(ausgabe, WITH_OK_1[lingo]);
    x_pos = CenterPosX(ausgabe, 14, 320);
    gfx->setCursor(x_pos, 156); gfx->print(ausgabe);
  #endif
  i = 1;
  while (i > 0) {
    if (digitalRead(button_stop_pin) == HIGH or digitalRead(switch_setup_pin) == LOW) {
      while (digitalRead(button_stop_pin) == HIGH) {
        delay(1);
      }
      modus = -1;
      return;
    }
    else if (digitalRead(SELECT_SW) == SELECT_PEGEL) {
      scale.set_scale();
      scale.tare(10);
      delay(500);
      i = 0;
    }
  }
  #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
    gfx->fillScreen(BACKGROUND);
    gfx->setTextColor(TEXT);
    gfx->setFont(Punk_Mono_Bold_240_150);
    x_pos = CenterPosX(CALIBRATION[lingo], 14, 320);
    gfx->setCursor(x_pos, 25);
    gfx->println(CALIBRATION[lingo]);
    gfx->drawLine(0, 30, 320, 30, TEXT);
  #endif
  initRotaries(SW_MENU, kali_gewicht, 100, 9999, 1); 
  i = 1;
  while (i > 0) {
    if (digitalRead(button_stop_pin) == HIGH or digitalRead(switch_setup_pin) == LOW) {
      while (digitalRead(button_stop_pin) == HIGH) {
        delay(1);
      }
      modus = -1;
      return;
    }
    kali_gewicht = getRotariesValue(SW_MENU);
    #if DISPLAY_TYPE == 1 or DISPLAY_TYPE == 2 or DISPLAY_TYPE == 99
      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_courB08_tf);
      u8g2.setCursor(0, 12);
      u8g2.print(PLEASE_SET_UP[lingo]);
      sprintf(ausgabe, " %dg", kali_gewicht);                     //Kalibrations Gewicht
      u8g2.print(ausgabe);
      u8g2.setCursor(0, 24);    u8g2.print(AND_CONFIRM[lingo]);
      u8g2.setCursor(0, 36);    u8g2.print(WITH_OK_2[lingo]);
      u8g2.sendBuffer();
    #endif
    #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
      if (kali_gewicht != kali_gewicht_old) {
        gfx->setFont(Punk_Mono_Bold_240_150);
        gfx->setTextColor(TEXT);
        sprintf(ausgabe, "%s", PLEASE_SET_UP);
        x_pos = CenterPosX(ausgabe, 14, 320);
        gfx->setCursor(x_pos, 100); gfx->print(ausgabe);
        gfx->setTextColor(BACKGROUND);
        sprintf(ausgabe, "%dg", kali_gewicht_old);
        x_pos = CenterPosX(ausgabe, 14, 320);
        gfx->setCursor(x_pos, 128); gfx->print(ausgabe);
        gfx->setTextColor(MARKER);
        sprintf(ausgabe, "%dg", kali_gewicht);
        x_pos = CenterPosX(ausgabe, 14, 320);
        gfx->setCursor(x_pos, 128); gfx->print(ausgabe);
        gfx->setTextColor(TEXT);
        sprintf(ausgabe, "%s", AND_CONFIRM);
        x_pos = CenterPosX(ausgabe, 14, 320);
        gfx->setCursor(x_pos, 156); gfx->print(ausgabe);
        sprintf(ausgabe, "%s", WITH_OK_2);
        x_pos = CenterPosX(ausgabe, 14, 320);
        gfx->setCursor(x_pos, 184); gfx->print(ausgabe);
        kali_gewicht_old = kali_gewicht;
      }
    #endif
    if (digitalRead(SELECT_SW) == SELECT_PEGEL) {
      gewicht_raw  = scale.get_units(10);
      faktor       = gewicht_raw / kali_gewicht;
      scale.set_scale(faktor);
      gewicht_leer = scale.get_offset();    // Leergewicht der Waage speichern
      #ifdef isDebug
        Serial.print("kalibrier_gewicht = ");
        Serial.println(kali_gewicht);
        Serial.print("gewicht_leer = ");
        Serial.println(gewicht_leer);
        Serial.print("gewicht_raw = ");
        Serial.println(gewicht_raw);
        Serial.print(" Faktor = ");
        Serial.println(faktor);
      #endif        
      delay(1000);
      modus = -1;
      i = 0;        
    }
  }
}

void setupServoWinkel(void) {
  int menuitem;
  int menuitem_used = 4;
  int lastmin  = winkel_min;
  int lastfein = winkel_fein;
  int lastmax  = winkel_max;
  int wert_alt;
  bool wert_aendern = false;
  bool servo_live = false;
  initRotaries(SW_MENU, 0, 0, menuitem_used, 1);
  #if DISPLAY_TYPE == 1 or DISPLAY_TYPE == 2 or DISPLAY_TYPE == 99
    int y_offset = 8;
  #endif
  #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
    int y_offset_tft = 28;
    bool change = false;
    int wert_old = -1;
    int MenuepunkteAnzahl = 5;
    const char *menuepunkte[MenuepunkteAnzahl] = {LIVESETUP[lingo], MINIMUM[lingo], FINEDOSAGE[lingo], MAXIMUM[lingo], SAVE[lingo]};
    gfx->fillScreen(BACKGROUND);
    gfx->setTextColor(TEXT);
    gfx->setFont(Punk_Mono_Bold_240_150);
    int x_pos = CenterPosX(SERVOSETTINGS[lingo], 14, 320);
    gfx->setCursor(x_pos, 25);
    gfx->println(SERVOSETTINGS[lingo]);
    gfx->drawLine(0, 30, 320, 30, TEXT);
  #endif
  i = 1;
  while (i > 0) {
    if (digitalRead(button_stop_pin) == HIGH or digitalRead(switch_setup_pin) == LOW) {
      while (digitalRead(button_stop_pin) == HIGH) {
        delay(1);
      }
      winkel_min  = lastmin;
      winkel_fein = lastfein;
      winkel_max  = lastmax;
      if (servo_live == true) {
        SERVO_WRITE(winkel_min);
      }
      modus = -1;
      return;
    }
    if (wert_aendern == false) {
      menuitem = getRotariesValue(SW_MENU);
      #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
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
        case 1: winkel_min  = getRotariesValue(SW_MENU);
                if (servo_live == true) SERVO_WRITE(winkel_min);
                break;
        case 2: winkel_fein = getRotariesValue(SW_MENU);
                if (servo_live == true) SERVO_WRITE(winkel_fein);
                break;
        case 3: winkel_max  = getRotariesValue(SW_MENU);
                if (servo_live == true) SERVO_WRITE(winkel_max);
                break;
      }
    }
    #if DISPLAY_TYPE == 1 or DISPLAY_TYPE == 2 or DISPLAY_TYPE == 99
      u8g2.setFont(u8g2_font_courB08_tf);
      u8g2.clearBuffer();
      u8g2.setCursor(10, 1 * y_offset);
      u8g2.print(LIVESETUP[lingo]);
      u8g2.setCursor(95, 1 * y_offset);
      sprintf(ausgabe,"%5s", (servo_live==false?OFF:ON));
      u8g2.print(ausgabe);
      u8g2.setCursor(10, 2 * y_offset);
      u8g2.print(MINIMUM[lingo]);
      u8g2.setCursor(95, 2 * y_offset);
      sprintf(ausgabe,"%4d°", winkel_min);
      u8g2.print(ausgabe);
      u8g2.setCursor(10, 3 * y_offset);
      u8g2.print(FINEDOSAGE[lingo]);
      u8g2.setCursor(95, 3 * y_offset);
      sprintf(ausgabe,"%4d°", winkel_fein);
      u8g2.print(ausgabe);
      u8g2.setCursor(10, 4 * y_offset);
      u8g2.print(MAXIMUM[lingo]);
      u8g2.setCursor(95, 4 * y_offset);
      sprintf(ausgabe,"%4d°", winkel_max);
      u8g2.print(ausgabe);
      u8g2.setCursor(10, (7 * y_offset) + 5);
      u8g2.print(SAVE[lingo]);
      if (wert_aendern == false && menuitem < menuitem_used) {
        u8g2.setCursor(1, 10+((menuitem)*y_offset)); u8g2.print("*");
      }
      else if (wert_aendern == true && menuitem < menuitem_used) {
        u8g2.setCursor(1, 8+((menuitem)*y_offset)); u8g2.print("-");
      }
      else if (wert_aendern == false && menuitem == 6) {
        u8g2.setCursor(1, 10+((menuitem)*y_offset) + 5); u8g2.print("*");
      }
      u8g2.sendBuffer();
    #endif
    #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
      int j = 0;
      while(j < MenuepunkteAnzahl) {
        gfx->setTextColor(TEXT);
        if (j == pos and wert_aendern == false) {
          gfx->setTextColor(MARKER);
        }
        if (j < MenuepunkteAnzahl - 1) {
          gfx->setCursor(5, 30+((j+1) * y_offset_tft));
          gfx->print(menuepunkte[j]);
          if (j == pos and wert_aendern == true) {
            //sprintf(ausgabe,"(%d°)", wert_alt);
            //if (j > 0) {
            //  gfx->setCursor(170, 30+((j+1) * y_offset_tft));
            //  gfx->print(ausgabe);
            //}
            gfx->setTextColor(MARKER);
          }
          switch (j) {
            case 0: sprintf(ausgabe,"%s", servo_live==false?OFF:ON);
                    if (wert_old != servo_live and wert_aendern == true and j == pos) {
                      change = true;
                      wert_old = servo_live;
                    }
                    break;
            case 1: sprintf(ausgabe,"%d°", winkel_min);
                    if (wert_old != winkel_min and wert_aendern == true and j == pos) {
                      change = true;
                      wert_old = winkel_min;
                    }
                    break;
            case 2: sprintf(ausgabe,"%d°", winkel_fein);
                    if (wert_old != winkel_fein and wert_aendern == true and j == pos) {
                      change = true;
                      wert_old = winkel_fein;
                    }
                    break;
            case 3: sprintf(ausgabe,"%d°", winkel_max);
                    if (wert_old != winkel_max and wert_aendern == true and j == pos) {
                      change = true;
                      wert_old = winkel_max;
                    }
                    break;
          }
          if (change) {
            gfx->fillRect(251, 27+((j+1) * y_offset_tft)-19, 64, 25, BACKGROUND);
            change = false;
          }
          int y = get_length(ausgabe);
          gfx->setCursor(320 - y * 14 - 5, 30+((j+1) * y_offset_tft));
          gfx->print(ausgabe);
        }
        else {
          gfx->setCursor(5, 30+(7 * y_offset_tft));
          gfx->print(menuepunkte[j]);
        }
        j++;
      }
    #endif
    if (digitalRead(SELECT_SW) == SELECT_PEGEL && menuitem < menuitem_used  && wert_aendern == false) {
      while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
        delay(1);
      }
      switch (menuitem) { 
        case 0: initRotaries(SW_MENU, servo_live, 0, 1, 1);
              break;
        case 1: initRotaries(SW_MENU, winkel_min, winkel_hard_min, winkel_fein, 1);
              wert_alt = lastmin;
              break;
        case 2: initRotaries(SW_MENU, winkel_fein, winkel_min, winkel_max, 1);
              wert_alt = lastfein;
              break;
        case 3: initRotaries(SW_MENU, winkel_max, winkel_fein, winkel_hard_max, 1);
              wert_alt = lastmax;
              break;
      }
      #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
        wert_old = -1;
      #endif
      wert_aendern = true;
    }
    if (digitalRead(SELECT_SW) == SELECT_PEGEL && menuitem < menuitem_used  && wert_aendern == true) {
      while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
        delay(1);
      }
      if (servo_live == true) {
        SERVO_WRITE(winkel_min);
      }
      initRotaries(SW_MENU, menuitem, 0, menuitem_used, 1);
      wert_aendern = false;
      #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
        gfx->fillRect(170, 30+y_offset_tft+3, 96, 3*y_offset_tft + 2, BACKGROUND);
      #endif
    }
    if (digitalRead(SELECT_SW) == SELECT_PEGEL && menuitem == 6) {
      while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
        delay(1);
      }
      #if DISPLAY_TYPE == 1 or DISPLAY_TYPE == 2 or DISPLAY_TYPE == 99
        u8g2.setCursor(111, ((menuitem + 1) * y_offset) + 5);
        u8g2.print("OK");
        u8g2.sendBuffer();
      #endif
      #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
        gfx->setCursor(283, 30+(7 * y_offset_tft));
        gfx->print("OK");
      #endif
      modus = -1;
      delay(1000);
      i = 0;
    }
  }
}

void setupAutomatik(void) {
  int menuitem;
  int menuitem_used     = 6;
  int lastautostart     = autostart;
  int lastglastoleranz  = glastoleranz;
  int lastautokorrektur = autokorrektur;
  int lastkulanz        = kulanz_gr;
  int korrektur_alt     = korrektur;
  int intGewicht_alt    = intGewicht;
  bool wert_aendern     = false;
  #if DISPLAY_TYPE == 1 or DISPLAY_TYPE == 2 or DISPLAY_TYPE == 99
    int y_offset = 8;
  #endif
  #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
    int y_offset_tft = 28;
    bool change = false;
    int wert_old = -1;
    int MenuepunkteAnzahl = 7;
    const char *menuepunkte[MenuepunkteAnzahl] = {AUTOSTART[lingo], JAR_TOLERANCE[lingo], CORRECTION[lingo], AUTOCORRECTION[lingo], KINDNESS[lingo], FLOW_G_OVER_TIME[lingo], SAVE[lingo]};
    gfx->fillScreen(BACKGROUND);
    gfx->setTextColor(TEXT);
    gfx->setFont(Punk_Mono_Bold_240_150);
    int x_pos = CenterPosX(AUTOMATIC[lingo], 14, 320);
    gfx->setCursor(x_pos, 25);
    gfx->println(AUTOMATIC[lingo]);
    gfx->drawLine(0, 30, 320, 30, TEXT);
  #endif
  initRotaries(SW_MENU, 0, 0, menuitem_used, 1);
  i = 1;
  while (i > 0) {
    if (digitalRead(button_stop_pin) == HIGH or digitalRead(switch_setup_pin) == LOW) {
      while (digitalRead(button_stop_pin) == HIGH) {
        delay(1);
      }
      autostart     = lastautostart;
      autokorrektur = lastautokorrektur;
      kulanz_gr     = lastkulanz;
      glastoleranz  = lastglastoleranz;
      korrektur     = korrektur_alt;
      intGewicht    = intGewicht_alt;
      setRotariesValue(SW_KORREKTUR, korrektur_alt);
      rotary_select = SW_MENU;
      modus = -1;
      return;
    }
    if (wert_aendern == false) {
      menuitem = getRotariesValue(SW_MENU);
      #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
        pos = menuitem;
      #endif
      if (menuitem == menuitem_used) {
        menuitem = 7;
      }
    }
    else {
      switch (menuitem) {
        case 0: autostart     = getRotariesValue(SW_MENU);
                break;
        case 1: glastoleranz  = getRotariesValue(SW_MENU);
                break;
        case 2: korrektur     = getRotariesValue(SW_KORREKTUR);
                break;
        case 3: autokorrektur = getRotariesValue(SW_MENU);
                break;
        case 4: kulanz_gr     = getRotariesValue(SW_MENU);
                break;
        case 5: intGewicht    = getRotariesValue(SW_MENU);
                break;
      }
    }
    #if DISPLAY_TYPE == 1 or DISPLAY_TYPE == 2 or DISPLAY_TYPE == 99
      u8g2.setFont(u8g2_font_courB08_tf);
      u8g2.clearBuffer();
      u8g2.setCursor(10, 1 * y_offset);
      u8g2.print(AUTOSTART[lingo]);
      u8g2.setCursor(95, 1 * y_offset);
      sprintf(ausgabe,"%5s", (autostart==0?OFF:ON));
      u8g2.print(ausgabe);
      u8g2.setCursor(10, 2 * y_offset);
      u8g2.print(JAR_TOLERANCE[lingo]);
      u8g2.setCursor(95, 2 * y_offset);
      if (glastoleranz > 0) {
        if (glastoleranz > 9) {
          sprintf(ausgabe," %c%2dg", 177, glastoleranz);
        }
        else {
          sprintf(ausgabe,"  %c%1dg", 177, glastoleranz);
        }
      }
      else {
        sprintf(ausgabe,"%4dg", glastoleranz);
      }
      u8g2.print(ausgabe);
      u8g2.setCursor(10, 3 * y_offset);
      u8g2.print(CORRECTION[lingo]);
      u8g2.setCursor(95, 3 * y_offset);
      sprintf(ausgabe,"%4dg", korrektur);
      u8g2.print(ausgabe);
      u8g2.setCursor(10, 4 * y_offset);
      u8g2.print(AUTOCORRECTION[lingo]);
      u8g2.setCursor(95, 4 * y_offset);
      sprintf(ausgabe,"%5s", (autokorrektur==0?OFF:ON));
      u8g2.print(ausgabe);
      u8g2.setCursor(10, 5 * y_offset);
      u8g2.print(KINDNESS[lingo]);
      u8g2.setCursor(95, 5 * y_offset);
      sprintf(ausgabe,"%4dg", kulanz_gr);
      u8g2.print(ausgabe);
      u8g2.setCursor(10, 6 * y_offset);
      u8g2.print(FLOW_G_OVER_TIME[lingo]);
      u8g2.setCursor(95, 6 * y_offset);
      if (intGewicht > 0) {
        sprintf(ausgabe,"%4dg", intGewicht);
      }
      else {
        sprintf(ausgabe,"%5s", OFF);
      }
      u8g2.print(ausgabe);
      u8g2.setCursor(10, (7 * y_offset) + 5);
      u8g2.print(SAVE[lingo]);
      // Positionsanzeige im Menu. "*" wenn nicht ausgewählt, Pfeil wenn ausgewählt
      if (wert_aendern == false && menuitem < menuitem_used) {
        u8g2.setCursor(1, 10+((menuitem)*y_offset)); u8g2.print("*");
      }
      else if (wert_aendern == true && menuitem < menuitem_used) {
        u8g2.setCursor(1, 8+((menuitem)*y_offset)); u8g2.print("-");
      }
      else if (wert_aendern == false && menuitem == 7) {
        u8g2.setCursor(1, 10+((menuitem - 1)*y_offset) + 5); u8g2.print("*");
      }
      u8g2.sendBuffer();
    #endif
    #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
      int j = 0;
      while(j < MenuepunkteAnzahl) {
        gfx->setTextColor(TEXT);
        if (j == pos and wert_aendern == false) {
          gfx->setTextColor(MARKER);
        }
        if (j < MenuepunkteAnzahl - 1) {
          gfx->setCursor(5, 30+((j+1) * y_offset_tft));
          gfx->print(menuepunkte[j]);
          if (j == pos and wert_aendern == true) {
            gfx->setTextColor(MARKER);
          }
          switch (j) {
            case 0: sprintf(ausgabe,"%s", autostart==false?OFF:ON);
                    if (wert_old != autostart and wert_aendern == true and j == pos) {
                      change = true;
                      wert_old = autostart;
                    }
                    break;
            case 1: if (glastoleranz > 0) {
                      sprintf(ausgabe,"±%dg", glastoleranz);
                    }
                    else {
                      sprintf(ausgabe,"%dg", glastoleranz);
                    }
                    if (wert_old != glastoleranz and wert_aendern == true and j == pos) {
                      change = true;
                      wert_old = glastoleranz;
                    }
                    break;
            case 2: sprintf(ausgabe,"%dg", korrektur);
                    if (wert_old != korrektur and wert_aendern == true and j == pos) {
                      change = true;
                      wert_old = korrektur;
                    }
                    break;
            case 3: sprintf(ausgabe,"%s", autokorrektur==false?OFF:ON);
                    if (wert_old != autokorrektur and wert_aendern == true and j == pos) {
                      change = true;
                      wert_old = autokorrektur;
                    }
                    break;
            case 4: sprintf(ausgabe,"%dg", kulanz_gr);
                    if (wert_old != kulanz_gr and wert_aendern == true and j == pos) {
                      change = true;
                      wert_old = kulanz_gr;
                    }
                    break;
            case 5: if (intGewicht > 0) {
                      sprintf(ausgabe,"%dg", intGewicht);
                    }
                    else {
                      sprintf(ausgabe,"%s", OFF);
                    }
                    if (wert_old != intGewicht and wert_aendern == true and j == pos) {
                      change = true;
                      wert_old = intGewicht;
                    }
                    break;
          }
          if (change) {
            gfx->fillRect(251, 27+((j+1) * y_offset_tft)-19, 64, 27, BACKGROUND);
            change = false;
          }
          int y = get_length(ausgabe);
          gfx->setCursor(320 - y * 14 - 5, 30+((j+1) * y_offset_tft));
          gfx->print(ausgabe);
        }
        else {
          gfx->setCursor(5, 30+(7 * y_offset_tft));
          gfx->print(menuepunkte[j]);
        }
        j++;
      }
    #endif
    // Menupunkt zum Ändern ausgewählt
    if (digitalRead(SELECT_SW) == SELECT_PEGEL && menuitem < menuitem_used  && wert_aendern == false) { 
      while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
        delay(1);
      }
      switch (menuitem) { 
        case 0: rotary_select = SW_MENU;
                initRotaries(SW_MENU, autostart, 0, 1, 1);
                break;
        case 1: rotary_select = SW_MENU;
                initRotaries(SW_MENU, glastoleranz, 0, 99, 1);
                break;
        case 2: rotary_select = SW_KORREKTUR;
                break;
        case 3: rotary_select = SW_MENU;
                initRotaries(SW_MENU, autokorrektur, 0, 1, 1);
                break;
        case 4: rotary_select = SW_MENU;
                initRotaries(SW_MENU, kulanz_gr, 0, 99, 1);
                break;
        case 5: rotary_select = SW_MENU;
                initRotaries(SW_MENU, intGewicht, 0, 99, 1);
                break;
      }
      #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
        wert_old = -1;
      #endif
      wert_aendern = true;
    }
    // Änderung im Menupunkt übernehmen
    if (digitalRead(SELECT_SW) == SELECT_PEGEL && menuitem < menuitem_used  && wert_aendern == true) {
      while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
        delay(1);
      }
      rotary_select = SW_MENU;
      initRotaries(SW_MENU, menuitem, 0, menuitem_used, 1);
      wert_aendern = false;
    }
    // Menu verlassen 
    if (digitalRead(SELECT_SW) == SELECT_PEGEL && menuitem == 7) {
      while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
        delay(1);
      }
      #if DISPLAY_TYPE == 1 or DISPLAY_TYPE == 2 or DISPLAY_TYPE == 99
        u8g2.setCursor(111, (menuitem * y_offset) + 5);
        u8g2.print("OK");
        u8g2.sendBuffer();
      #endif
      #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
        gfx->setCursor(283, 30+(7 * y_offset_tft));
        gfx->print("OK");
      #endif
      delay(1000);
      modus = -1;
      i = 0;
    }
  }
  rotary_select = SW_MENU;
}

void setupFuellmenge(void) {
   #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
    int y_offset_tft = 28;
    String wert_old = "";
    gfx->fillScreen(BACKGROUND);
    gfx->setTextColor(TEXT);
    gfx->setFont(Punk_Mono_Bold_240_150);
    int x_pos = CenterPosX(FILL_QUANTITY_JARS[lingo], 14, 320);
    gfx->setCursor(x_pos, 25);
    gfx->println(FILL_QUANTITY_JARS[lingo]);
    gfx->drawLine(0, 30, 320, 30, TEXT);
  #endif
  int j;
  initRotaries(SW_MENU, fmenge_index, 0, 4, 1);
  i = 1;
  while (i > 0) {
    if (digitalRead(button_stop_pin) == HIGH or digitalRead(switch_setup_pin) == LOW) {
      while (digitalRead(button_stop_pin) == HIGH) {
        delay(1);
      }
      modus = -1;
      return;
    }
    pos = getRotariesValue(SW_MENU);
    #if DISPLAY_TYPE == 1 or DISPLAY_TYPE == 2 or DISPLAY_TYPE == 99
      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_courB08_tf);
      j = 0;
      while(j < 5) {
        u8g2.setCursor(20, 10+(j*13)); 
        sprintf(ausgabe, "%4dg      %3s", glaeser[j].Gewicht, GlasTypArray[glaeser[j].GlasTyp]);
        u8g2.print(ausgabe);
        j++;
      }
      u8g2.setCursor(10, 10+(getRotariesValue(SW_MENU)*13));    
      u8g2.print("*");
      u8g2.sendBuffer();
    #endif
    #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
        j = 0;
        while(j < 5) {
          if (j == pos) {
            gfx->setTextColor(MARKER);
          }
          else {
            gfx->setTextColor(TEXT);
          }
          gfx->setCursor(10, 30+((j+1) * y_offset_tft));
          sprintf(ausgabe, "%4dg", glaeser[j].Gewicht);
          gfx->print(ausgabe);
          gfx->setCursor(110, 30+((j+1) * y_offset_tft));
          if (GlasTypArray[glaeser[j].GlasTyp] == "DIB") {
            gfx->print("DE Imker Bund");
          }
          else if (GlasTypArray[glaeser[j].GlasTyp] == "TOF") {
            gfx->print("TwistOff");
          }
          else if (GlasTypArray[glaeser[j].GlasTyp] == "DEE") {
            gfx->print("DeepTwist");
          }
          else if (GlasTypArray[glaeser[j].GlasTyp] == "SPZ") {
            gfx->print("Spezial");
          }
          j++;
       }
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
      modus = -1;
      return;
    }
    glaeser[pos].Gewicht = step2weight(getRotariesValue(SW_MENU));
    #if DISPLAY_TYPE == 1 or DISPLAY_TYPE == 2 or DISPLAY_TYPE == 99
      u8g2.clearBuffer();    
      j = 0;
      while(j < 5) {
        u8g2.setCursor(20, 10+(j*13));
        if (j == pos){ 
          u8g2.setFont(u8g2_font_open_iconic_arrow_1x_t);
          u8g2.drawGlyph(5, 11+(j*13), 0x42);
          u8g2.drawGlyph(55, 11+(j*13), 0x41);
          u8g2.setFont(u8g2_font_courB08_tf);   
          sprintf(ausgabe, "%4dg      %3s", glaeser[j].Gewicht, GlasTypArray[glaeser[j].GlasTyp]);  
        }
        else {
          sprintf(ausgabe, "%4dg      %3s", glaeser[j].Gewicht, GlasTypArray[glaeser[j].GlasTyp]);
        }
        u8g2.print(ausgabe);
        j++;
      }                     
      u8g2.sendBuffer();
    #endif
    #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
      j = 0;
      while(j < 5) {
        gfx->setCursor(10, 30+((j+1) * y_offset_tft));
        if (j == pos) {
          if (wert_old != String(glaeser[j].Gewicht)) {
            gfx->fillRect(0, 27+((j+1) * y_offset_tft)-19, 100, 27, BACKGROUND);
            wert_old = String(glaeser[j].Gewicht);
          }
          gfx->setTextColor(MARKER);
        }
        sprintf(ausgabe, "%4dg", glaeser[j].Gewicht);
        gfx->print(ausgabe);
        gfx->setTextColor(TEXT);
        gfx->setCursor(110, 30+((j+1) * y_offset_tft));
        if (GlasTypArray[glaeser[j].GlasTyp] == "DIB") {
          gfx->print("DE Imker Bund");
        }
        else if (GlasTypArray[glaeser[j].GlasTyp] == "TOF") {
          gfx->print("TwistOff");
        }
        else if (GlasTypArray[glaeser[j].GlasTyp] == "DEE") {
          gfx->print("DeepTwist");
        }
        else if (GlasTypArray[glaeser[j].GlasTyp] == "SPZ") {
          gfx->print("Spezial");
        }
        j++;
      }
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
      modus = -1;
      return;
    }
    glaeser[pos].GlasTyp = getRotariesValue(SW_MENU);
    #if DISPLAY_TYPE == 1 or DISPLAY_TYPE == 2 or DISPLAY_TYPE == 99
      u8g2.clearBuffer();
      j = 0;
      while(j < 5) {
        u8g2.setCursor(20, 10+(j*13));
        if (j == pos){
          u8g2.setFont(u8g2_font_open_iconic_arrow_1x_t);
          u8g2.drawGlyph(72, 11+(j*13), 0x42);
          u8g2.drawGlyph(108, 11+(j*13), 0x41);
          u8g2.setFont(u8g2_font_courB08_tf);
          sprintf(ausgabe, "%4dg      %3s", glaeser[j].Gewicht, GlasTypArray[glaeser[j].GlasTyp]);
        }
        else {
          sprintf(ausgabe, "%4dg      %3s", glaeser[j].Gewicht, GlasTypArray[glaeser[j].GlasTyp]);
        }
        u8g2.print(ausgabe);
        j++;
      }
      u8g2.sendBuffer();
    #endif
    #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
      j = 0;
      while(j < 5) {
        gfx->setCursor(10, 30+((j+1) * y_offset_tft));
        sprintf(ausgabe, "%4dg", glaeser[j].Gewicht);
        gfx->print(ausgabe);
        if (j == pos) {
          if (wert_old != String(GlasTypArray[glaeser[j].GlasTyp])) {
            gfx->fillRect(100, 27+((j+1) * y_offset_tft)-19, 220, 27, BACKGROUND);
            wert_old = String(GlasTypArray[glaeser[j].GlasTyp]);
          }
          gfx->setTextColor(MARKER);
        }
        gfx->setCursor(110, 30+((j+1) * y_offset_tft));
        if (GlasTypArray[glaeser[j].GlasTyp] == "DIB") {
          gfx->print("DE Imker Bund");
        }
        else if (GlasTypArray[glaeser[j].GlasTyp] == "TOF") {
          gfx->print("TwistOff");
        }
        else if (GlasTypArray[glaeser[j].GlasTyp] == "DEE") {
          gfx->print("DeepTwist");
        }
        else if (GlasTypArray[glaeser[j].GlasTyp] == "SPZ") {
          gfx->print("Spezial");
        }
        gfx->setTextColor(TEXT);
        j++;
      }
    #endif
    if (digitalRead(SELECT_SW) == SELECT_PEGEL) { //GlasTyp bestätigt
      while((digitalRead(SELECT_SW) == SELECT_PEGEL)) {
        delay(1);
      }
      i = 0;
    }
  }
  fmenge = glaeser[pos].Gewicht;
  tare   = glaeser[pos].Tare;
  fmenge_index = pos; 
  modus = -1;
  i = 0;
}

void setupParameter(void) {
  int menuitem;
  int menuitem_used = 4;
  int lastbuzzer    = buzzermode;
  int lastled       = ledmode;
  int lastlogo      = showlogo;
  int lastcredits   = showcredits;
  int lastcolor_scheme = color_scheme;
  int lastcolor_marker = color_marker;
  bool wert_aendern = false;
  #if DISPLAY_TYPE == 1 or DISPLAY_TYPE == 2 or DISPLAY_TYPE == 99
    int y_offset = 8;
  #endif
  #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
    int y_offset_tft = 28;
    int wert_old = -1;
    bool change = false;
    bool change_scheme = true;
    bool change_marker = true;
    menuitem_used = 6;
    int MenuepunkteAnzahl = 7;
    const char *menuepunkte[MenuepunkteAnzahl] = {BUZZER[lingo], LED[lingo], SHOW_LOGO[lingo], SHOW_CREDITS[lingo], COLORSCHEME[lingo], MARKER_COLOR[lingo], SAVE[lingo]};
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
      modus = -1;
      return;
    }
    if (wert_aendern == false) {
      menuitem = getRotariesValue(SW_MENU);
      #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
        pos = menuitem;
      #endif
      if (menuitem == menuitem_used) {
        menuitem = 6;
      }
    }
    else {
      switch (menuitem) {
        case 0: buzzermode    = getRotariesValue(SW_MENU);
                break;
        case 1: ledmode       = getRotariesValue(SW_MENU);
                break;
        case 2: showlogo      = getRotariesValue(SW_MENU);
                break;
        case 3: showcredits   = getRotariesValue(SW_MENU);
                break;
        case 4: color_scheme  = getRotariesValue(SW_MENU);
                break;
        case 5: color_marker  = getRotariesValue(SW_MENU);
                break;
      }
    }
    // Menu
    #if DISPLAY_TYPE == 1 or DISPLAY_TYPE == 2 or DISPLAY_TYPE == 99
      u8g2.setFont(u8g2_font_courB08_tf);
      u8g2.clearBuffer();
      u8g2.setCursor(10, 1 * y_offset);
      u8g2.print(BUZZER[lingo]);
      u8g2.setCursor(95, 1 * y_offset);
      sprintf(ausgabe,"%5s", (buzzermode==0?OFF:ON));
      u8g2.print(ausgabe);
      u8g2.setCursor(10, 2 * y_offset);
      u8g2.print(LED[lingo]);
      u8g2.setCursor(95, 2 * y_offset);
      sprintf(ausgabe,"%5s", (ledmode==0?OFF:ON));
      u8g2.print(ausgabe);
      u8g2.setCursor(10, 3 * y_offset);
      u8g2.print(SHOW_LOGO[lingo]);
      u8g2.setCursor(95, 3 * y_offset);
      sprintf(ausgabe,"%5s", (showlogo==0?OFF:ON));
      u8g2.print(ausgabe);
      u8g2.setCursor(10, 4 * y_offset);
      u8g2.print(SHOW_CREDITS[lingo]);
      u8g2.setCursor(95, 4 * y_offset);
      sprintf(ausgabe,"%5s", (showcredits==0?OFF:ON));
      u8g2.print(ausgabe);
      u8g2.setCursor(10, (7 * y_offset) + 5);
      u8g2.print(SAVE[lingo]);
      // Positionsanzeige im Menu. "*" wenn nicht ausgewählt, Pfeil wenn ausgewählt
      if (wert_aendern == false && menuitem < menuitem_used) {
        u8g2.setCursor(1, 10+((menuitem)*y_offset)); u8g2.print("*");
      }
      else if (wert_aendern == true && menuitem < menuitem_used) {
        u8g2.setCursor(1, 8+((menuitem)*y_offset)); u8g2.print("-");
      }
      else if (wert_aendern == false && menuitem == 6) {
        u8g2.setCursor(1, 10+((menuitem)*y_offset) + 5); u8g2.print("*");
      }
      u8g2.sendBuffer();
    #endif
    #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
      if(change_scheme) {
        gfx->fillScreen(BACKGROUND);
        gfx->setTextColor(TEXT);
        gfx->setFont(Punk_Mono_Bold_240_150);
        int x_pos = CenterPosX(PARAMETER[lingo], 14, 320);
        gfx->setCursor(x_pos, 25);
        gfx->println(PARAMETER[lingo]);
        gfx->drawLine(0, 30, 320, 30, TEXT);
        change_scheme = false;
        change_marker = true;
      }
      int j = 0;
      while(j < MenuepunkteAnzahl) {
        gfx->setTextColor(TEXT);
        if (j == pos and wert_aendern == false) {
          gfx->setTextColor(MARKER);
        }
        if (j < MenuepunkteAnzahl - 1) {
          gfx->setCursor(5, 30+((j+1) * y_offset_tft));
          gfx->print(menuepunkte[j]);
          if (j == pos and wert_aendern == true) {
            gfx->setTextColor(MARKER);
          }
          switch (j) {
            case 0: sprintf(ausgabe,"%s", buzzermode==false?OFF:ON);
                    if (wert_old != buzzermode and wert_aendern == true and j == pos) {
                      change = true;
                      wert_old = buzzermode;
                    }
                    break;
            case 1: sprintf(ausgabe,"%s", ledmode==false?OFF:ON);
                    if (wert_old != ledmode and wert_aendern == true and j == pos) {
                      change = true;
                      wert_old = ledmode;
                    }
                    break;
            case 2: sprintf(ausgabe,"%s", showlogo==false?OFF:ON);
                    if (wert_old != showlogo and wert_aendern == true and j == pos) {
                      change = true;
                      wert_old = showlogo;
                    }
                    break;
            case 3: sprintf(ausgabe,"%s", showcredits==false?OFF:ON);
                    if (wert_old != showcredits and wert_aendern == true and j == pos) {
                      change = true;
                      wert_old = showcredits;
                    }
                    break;
            case 4: sprintf(ausgabe,"%s", color_scheme==false?DARK:LIGHT);
                    if (wert_old != color_scheme and wert_aendern == true and j == pos) {
                      change = true;
                      change_scheme = true;
                      wert_old = color_scheme;
                      tft_colors();
                    }
                    break;
            case 5: sprintf(ausgabe,"");
                    if (wert_old != color_marker and wert_aendern == true and j == pos) {
                      change = true;
                      change_marker = true;
                      wert_old = color_marker;
                      tft_marker();
                    }
                    break;
          }
          if (change) {
            gfx->fillRect(211, 27+((j+1) * y_offset_tft)-19, 104, 25, BACKGROUND);
            change = false;
          }
          if (change_marker) {
            gfx->fillRect(265, 27+(6 * y_offset_tft)-17, 45, 21, MARKER);
            change_marker = false;
          }
          int y = get_length(ausgabe);
          gfx->setCursor(320 - y * 14 - 5, 30+((j+1) * y_offset_tft));
          gfx->print(ausgabe);
        }
        else {
          gfx->setCursor(5, 30+(7 * y_offset_tft));
          gfx->print(menuepunkte[j]);
        }
        j++;
      }
    #endif
    // Menupunkt zum Ändern ausgewählt
    if (digitalRead(SELECT_SW) == SELECT_PEGEL && menuitem < menuitem_used  && wert_aendern == false) {
      while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
        delay(1);
      } 
      switch (menuitem) { 
        case 0: initRotaries(SW_MENU, buzzermode, 0, 1, 1);
                break;
        case 1: initRotaries(SW_MENU, ledmode, 0, 1, 1);
                break;
        case 2: initRotaries(SW_MENU, showlogo, 0, 1, 1);
                break;
        case 3: initRotaries(SW_MENU, showcredits, 0, 1, 1);
                break;
        case 4: initRotaries(SW_MENU, color_scheme, 0, 1, 1);
                break;
        case 5: initRotaries(SW_MENU, color_marker, 0, 11, 1);
                break;
      }
      #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
        wert_old = -1;
      #endif
      wert_aendern = true;
    }
    // Änderung im Menupunkt übernehmen
    if (digitalRead(SELECT_SW) == SELECT_PEGEL && menuitem < menuitem_used  && wert_aendern == true) {
      while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
        delay(1);
      }
      initRotaries(SW_MENU, menuitem, 0, menuitem_used, 1);
      wert_aendern = false;
    }
    // Menu verlassen 
    if (digitalRead(SELECT_SW) == SELECT_PEGEL && menuitem == 6) {
      while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
        delay(1);
      }
      #if DISPLAY_TYPE == 1 or DISPLAY_TYPE == 2 or DISPLAY_TYPE == 99
        u8g2.setCursor(111, ((menuitem + 1) * y_offset) + 5);
        u8g2.print("OK");
        u8g2.sendBuffer();
      #endif
      #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
        gfx->setCursor(283, 30+(7 * y_offset_tft));
        gfx->print("OK");
      #endif
      delay(1000);
      modus = -1;
      i = 0;
    }
  }
}

void setupClearPrefs(void) {
  initRotaries(SW_MENU, 2, 0, 2, 1);
  int x_pos;
  #if DISPLAY_TYPE == 1 or DISPLAY_TYPE == 2 or DISPLAY_TYPE == 99
    int y_offset = 8;
  #endif
  #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
    int y_offset_tft = 28;
    gfx->fillScreen(BACKGROUND);
    int MenuepunkteAnzahl = 3;
    const char *menuepunkte[MenuepunkteAnzahl - 1] = {CLEAR_PREFERENCES[lingo], CLEAR_NVS_MEMORY[lingo], BACK[lingo]};
    gfx->setTextColor(TEXT);
    gfx->setFont(Punk_Mono_Bold_240_150);
    x_pos = CenterPosX(CLEAR_PREFERENCES[lingo], 14, 320);
    gfx->setCursor(x_pos, 25);
    gfx->println(CLEAR_PREFERENCES[lingo]);
    gfx->drawLine(0, 30, 320, 30, TEXT);
  #endif
  i = 1;
  while (i > 0) {
    if (digitalRead(button_stop_pin) == HIGH  or digitalRead(switch_setup_pin) == LOW) {
      while (digitalRead(button_stop_pin) == HIGH) {
        delay(1);
      }
      modus = -1;
      return;
    }
    pos = getRotariesValue(SW_MENU);
    #if DISPLAY_TYPE == 1 or DISPLAY_TYPE == 2 or DISPLAY_TYPE == 99
      u8g2.setFont(u8g2_font_courB08_tf);
      u8g2.clearBuffer();
      u8g2.setCursor(10, 1 * y_offset);
      u8g2.print(CLEAR_PREFERENCES[lingo]);
      u8g2.setCursor(10, 2 * y_offset);
      u8g2.print(CLEAR_NVS_MEMORY[lingo]);
      u8g2.setCursor(10, (7 * y_offset) + 5);
      u8g2.print(BACK[lingo]);
      if (pos == 2) {
        u8g2.setCursor(0, (7 * y_offset) + 7);
      }
      else {
        u8g2.setCursor(0, (pos + 1) * y_offset + 2);
      }
      u8g2.print("*");
      u8g2.sendBuffer();
    #endif
    #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
      int j = 0;
      while(j < MenuepunkteAnzahl) {
        if (j == pos) {
          gfx->setTextColor(MARKER);
        }
        else {
          gfx->setTextColor(TEXT);
        }
        if (j < MenuepunkteAnzahl - 1) {
          gfx->setCursor(10, 30+((j+1) * y_offset_tft));
          gfx->print(menuepunkte[j]);
        }
        else {
          gfx->setCursor(10, 30+(7 * y_offset_tft));
          gfx->print(menuepunkte[j]);
        }
        j++;
      }
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
      #if DISPLAY_TYPE == 1 or DISPLAY_TYPE == 2 or DISPLAY_TYPE == 99
        u8g2.setCursor(115, (7 * y_offset) + 5);   
        u8g2.print("OK");
        u8g2.sendBuffer();
      #endif
      #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
        gfx->setCursor(280, 30+(7 * y_offset_tft));
        gfx->print("OK");
      #endif
      delay(1000);
      modus = -1;
      i = 0;
    }
  }
}

void setupINA219(void) {                            //Funktioniert nur wenn beide Menüs die gleiche Anzahl haben. Feel free to change it :-)
  int menuitem;
  int lastcurrent             = current_servo;
  int lastwinkel_min          = winkel_min;
  int lastshow_current        = show_current;
  bool wert_aendern = false;
  int menuitem_used           = 2;
  String kalibration_status   = START[lingo];
  String quetschhan           = CLOSE[lingo];
  bool cal_done = false;
  int cal_winkel = 0;
  int j = 0;
  int k = 0;
  #if DISPLAY_TYPE == 1 or DISPLAY_TYPE == 2 or DISPLAY_TYPE == 99
    int y_offset = 8;
  #endif
  #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
    int y_offset_tft = 28;
    bool change = false;
    int wert_old = -1;
    int MenuepunkteAnzahl = 3;
    const char *menuepunkte_1[MenuepunkteAnzahl] = {SERVO_CURRENT[lingo], CAL_HONEY_GATE[lingo], SAVE[lingo]};
    const char *menuepunkte_2[MenuepunkteAnzahl] = {SERVO_CURRENT[lingo], SHOW_CURRENT[lingo], SAVE[lingo]};
    gfx->fillScreen(BACKGROUND);
    gfx->setTextColor(TEXT);
    gfx->setFont(Punk_Mono_Bold_240_150);
    int x_pos = CenterPosX(INA219_SETUP[lingo], 14, 320);
    gfx->setCursor(x_pos, 25);
    gfx->println(INA219_SETUP[lingo]);
    gfx->drawLine(0, 30, 320, 30, TEXT);
  #endif
  initRotaries(SW_MENU, 0, 0, menuitem_used, 1);
  i = 1;
  while (i > 0) {
    if (digitalRead(button_stop_pin) == HIGH  or digitalRead(switch_setup_pin) == LOW) {
      while (digitalRead(button_stop_pin) == HIGH) {
        delay(1);
      }
      current_servo = lastcurrent;
      winkel_min = lastwinkel_min;
      show_current = lastshow_current;
      modus = -1;
      return;
    }
    if (wert_aendern == false) {
      menuitem = getRotariesValue(SW_MENU);
      #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
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
                  kalibration_status    = START[lingo];
                  wert_aendern          = false;
                  menuitem_used         = 1;
                  setRotariesValue(SW_MENU, 0);
                  initRotaries(SW_MENU, 0, 0, menuitem_used, 1);
                }
                break;
      }
    }
    // Menu
    #if DISPLAY_TYPE == 1 or DISPLAY_TYPE == 2 or DISPLAY_TYPE == 99
      u8g2.setFont(u8g2_font_courB08_tf);
      u8g2.clearBuffer();
      u8g2.setCursor(10, 1 * y_offset);
      u8g2.print(SERVO_CURRENT[lingo]);
      u8g2.setCursor(90, 1 * y_offset);
      if (current_servo > 0) {
        sprintf(ausgabe,"%4dmA", (current_servo));
      }
      else {
        sprintf(ausgabe,"%6s", (OFF));
      }
      u8g2.print(ausgabe);
      if (current_servo > 0) {
        u8g2.setCursor(10, 2 * y_offset);
        u8g2.print(CAL_HONEY_GATE[lingo]);
        menuitem_used = 2;
      }
      else {
        u8g2.setCursor(10, 2 * y_offset);
        u8g2.print(SHOW_CURRENT[lingo]);
        u8g2.setCursor(90, 2 * y_offset);
        sprintf(ausgabe,"%6s", (show_current==0?OFF:ON));
        u8g2.print(ausgabe);
        menuitem_used = 2;
      }
      u8g2.setCursor(10, (7 * y_offset) + 5);
      u8g2.print(SAVE[lingo]);
      // Positionsanzeige im Menu. "*" wenn nicht ausgewählt, Pfeil wenn ausgewählt
      if (wert_aendern == false && menuitem < menuitem_used) {
        u8g2.setCursor(1, 10+((menuitem)*y_offset)); u8g2.print("*");
      }
      else if (wert_aendern == true && menuitem < menuitem_used) {
        u8g2.setCursor(1, 8+((menuitem)*y_offset)); u8g2.print("-");
      }
      else if (wert_aendern == false && menuitem == 6) {
        u8g2.setCursor(1, 10+((menuitem)*y_offset) + 5); u8g2.print("*");
      }
      u8g2.sendBuffer();
    #endif
    #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
      int a = 0;
      while(a < MenuepunkteAnzahl) {
        gfx->setTextColor(TEXT);
        if (a == pos and wert_aendern == false) {
          gfx->setTextColor(MARKER);
        }
        if (a < MenuepunkteAnzahl - 1) {
          if (current_servo == 0) {
            gfx->setCursor(5, 30+((a+1) * y_offset_tft));
            gfx->print(menuepunkte_2[a]);
          }
          else {
            gfx->setCursor(5, 30+((a+1) * y_offset_tft));
            gfx->print(menuepunkte_1[a]);
          }
          if (a == pos and wert_aendern == true) {
            gfx->setTextColor(MARKER);
          }
          switch (a) {
            case 0: 
                    if (current_servo > 0) {
                      sprintf(ausgabe,"%dmA", current_servo);
                    }
                    else {
                      sprintf(ausgabe,"%s", OFF[lingo]);
                    }
                    if (wert_old != current_servo and wert_aendern == true and a == pos) {
                      change = true;
                      wert_old = current_servo;
                    }
                    break;
            case 1: if (current_servo == 0) {
                      sprintf(ausgabe,"%s", show_current==false?OFF:ON);
                      if (wert_old != show_current and wert_aendern == true and a == pos) {
                        change = true;
                        wert_old = show_current;
                      }
                    }
                    else {
                      sprintf(ausgabe,"%s", "");
                      if (wert_old != show_current and wert_aendern == true and a == pos) {
                        change = true;
                        wert_old = show_current;
                      }
                    }
                    break;
          }
          if (change) {
            if (a == 0) {
              gfx->fillRect(215, 27+((a+1) * y_offset_tft)-19, 100, 25, BACKGROUND);
              if (wert_old == 0 or wert_old == 50) {                                                //nicht ganz sauber bei den Schwellwerten. 
                gfx->fillRect(0, 27+((a+2) * y_offset_tft)-19, 320, 27, BACKGROUND);
              }
            }
            else {
              gfx->fillRect(251, 27+((a+1) * y_offset_tft)-19, 64, 25, BACKGROUND);
            }
            change = false;
          }
          int y = get_length(ausgabe);
          gfx->setCursor(320 - y * 14 - 5, 30+((a+1) * y_offset_tft));
          gfx->print(ausgabe);
        }
        else {
          gfx->setCursor(5, 30+(7 * y_offset_tft));
          gfx->print(menuepunkte_2[a]);
        }
        a++;
      }
    #endif
    if (digitalRead(SELECT_SW) == SELECT_PEGEL && menuitem < menuitem_used  && wert_aendern == false) {
      while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
        delay(1);
      } 
      switch (menuitem) { 
        case 0: initRotaries(SW_MENU, current_servo, 0, 1500, 50);
                break;
        case 1: if (current_servo == 0) {initRotaries(SW_MENU, show_current, 0, 1, 1);}
                break;
      }
      wert_aendern = true;
    }
    // Änderung im Menupunkt übernehmen
    if (digitalRead(SELECT_SW) == SELECT_PEGEL && menuitem < menuitem_used  && wert_aendern == true) {
      while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
        delay(1);
      }
      initRotaries(SW_MENU, menuitem, 0, menuitem_used, 1);
      wert_aendern = false;
    }
    // Menu verlassen 
    if (digitalRead(SELECT_SW) == SELECT_PEGEL && menuitem == 6) {
      while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
        delay(1);
      }
      #if DISPLAY_TYPE == 1 or DISPLAY_TYPE == 2 or DISPLAY_TYPE == 99
        u8g2.setCursor(111, ((menuitem + 1) * y_offset) + 5);
        u8g2.print("OK");
        u8g2.sendBuffer();
      #endif
      #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
        gfx->setCursor(283, 30+(7 * y_offset_tft));
        gfx->print("OK");
      #endif
      delay(1000);
      modus = -1;
      i = 0;
    }
    while (j > 0) {
      if (digitalRead(switch_setup_pin) == LOW) {
        current_servo = lastcurrent;
        show_current = lastshow_current;
        modus = -1;
        return;
      }
      if (wert_aendern == false) {
        menuitem = getRotariesValue(SW_MENU);
        if (menuitem == menuitem_used) {
          menuitem = 6;
        }
      }
      menuitem_used = 1;
      #if DISPLAY_TYPE == 1 or DISPLAY_TYPE == 2 or DISPLAY_TYPE == 99
        u8g2.setFont(u8g2_font_courB08_tf);
        u8g2.clearBuffer();
        u8g2.setCursor(10, 1 * y_offset);
        u8g2.print(kalibration_status);
        u8g2.setCursor(10, 3 * y_offset);
        u8g2.printf("%s:", MAX_CURRENT);
        u8g2.setCursor(90, 3 * y_offset);
        sprintf(ausgabe,"%4imA", current_servo - 30);
        u8g2.print(ausgabe);
        u8g2.setCursor(10, 4 * y_offset);
        u8g2.printf("%s:", MIN_ANGLE);
        u8g2.setCursor(90, 4 * y_offset);
        sprintf(ausgabe,"%5i°", winkel_min);
        u8g2.print(ausgabe);
        u8g2.setCursor(10, (7 * y_offset) + 5);
        u8g2.print(BACK[lingo]);
        // Positionsanzeige im Menu. "*" wenn nicht ausgewählt, Pfeil wenn ausgewählt
        if (wert_aendern == false && menuitem < menuitem_used && kalibration_status == START[lingo]) {
          u8g2.setCursor(1, 10+((menuitem)*y_offset)); u8g2.print("*");
        }
        else if (wert_aendern == false && menuitem == 6) {
          u8g2.setCursor(1, 10+((menuitem)*y_offset) + 5); u8g2.print("*");
        }
        u8g2.sendBuffer();
      #endif
      #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
        if (j == 1) {
          gfx->fillScreen(BACKGROUND);
          gfx->setTextColor(TEXT);
          gfx->setFont(Punk_Mono_Bold_240_150);
          x_pos = CenterPosX(INA219_SETUP[lingo], 14, 320);
          gfx->setCursor(x_pos, 25);
          gfx->println(INA219_SETUP[lingo]);
          gfx->drawLine(0, 30, 320, 30, TEXT);
          gfx->setCursor(5, 30+(3 * y_offset_tft));
          gfx->printf("%s:", MAX_CURRENT);
          sprintf(ausgabe,"%4imA", current_servo - 30);
          int y = get_length(ausgabe);
          gfx->setCursor(320 - y * 14 - 5, 30+(3 * y_offset_tft));
          gfx->print(ausgabe);
          gfx->setCursor(5, 30+(4 * y_offset_tft));
          gfx->printf("%s:", MIN_ANGLE);
          sprintf(ausgabe,"%3i°", winkel_min);
          y = get_length(ausgabe);
          gfx->setCursor(320 - y * 14 - 5, 30+(4 * y_offset_tft));
          gfx->print(ausgabe);
          j++;
        }
        if (wert_aendern == false && menuitem < menuitem_used && kalibration_status == START[lingo]) {
          gfx->setTextColor(MARKER);
          gfx->setCursor(5, 30+(1 * y_offset_tft));
          gfx->print(kalibration_status);
          gfx->setTextColor(TEXT);
          gfx->setCursor(5, 30+(7 * y_offset_tft));
          gfx->print(BACK[lingo]);
        }
        else {
          gfx->setTextColor(TEXT);
          gfx->setCursor(5, 30+(1 * y_offset_tft));
          gfx->print(kalibration_status);
          gfx->setTextColor(MARKER);
          gfx->setCursor(5, 30+(7 * y_offset_tft));
          gfx->print(BACK[lingo]);
        }
      #endif
      if (wert_aendern == true && menuitem < menuitem_used) {
        lastcurrent = current_servo;
        kalibration_status = CALIBRATION_RUNNING[lingo];
        quetschhan = CLOSE[lingo];
        cal_done = false;
        cal_winkel = 0;
        k = 1;
      }
      if (digitalRead(SELECT_SW) == SELECT_PEGEL && menuitem < menuitem_used  && wert_aendern == false) {
        while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
          delay(1);
        } 
        wert_aendern = true;
      }
      // Änderung im Menupunkt übernehmen
      if (digitalRead(SELECT_SW) == SELECT_PEGEL && menuitem < menuitem_used  && wert_aendern == true) {
        while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
          delay(1);
        }
        initRotaries(SW_MENU, 0, 0, menuitem_used, 1);
        wert_aendern = false;
      }
      //verlassen
      if ((digitalRead(SELECT_SW) == SELECT_PEGEL && menuitem == 6) or digitalRead(button_stop_pin) == HIGH) {
        while(digitalRead(SELECT_SW) == SELECT_PEGEL or digitalRead(button_stop_pin) == HIGH) {
          delay(1);
        }
        #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
          gfx->fillScreen(BACKGROUND);
          x_pos = CenterPosX(INA219_SETUP[lingo], 14, 320);
          gfx->setCursor(x_pos, 25);
          gfx->println(INA219_SETUP[lingo]);
          gfx->drawLine(0, 30, 320, 30, TEXT);
        #endif
        j = 0;
        wert_aendern = false;
        menuitem_used = 2;
        initRotaries(SW_MENU, 0, 0, menuitem_used, 1);
      }
      while (k > 0) {
        SERVO_WRITE(90);
        quetschhan = OPEN[lingo];
        int scaletime = millis();
        bool measurement_run = false;
        while (!cal_done) {
          #if DISPLAY_TYPE == 1 or DISPLAY_TYPE == 2 or DISPLAY_TYPE == 99
            u8g2.setFont(u8g2_font_courB08_tf);
            u8g2.clearBuffer();
            u8g2.setCursor(10, 1 * y_offset);
            u8g2.print(kalibration_status);
            u8g2.setCursor(10, 3 * y_offset);
            u8g2.print(ABORT[lingo]);
            u8g2.setCursor(10, 4 * y_offset);
            u8g2.printf("%s:", CURRENT);
            u8g2.setCursor(90, 4 * y_offset);
            sprintf(ausgabe,"%4imA", current_mA);
            u8g2.print(ausgabe);
            u8g2.setCursor(10, 4 * y_offset);
            u8g2.printf("%s:", ANGLE);
            u8g2.setCursor(90, 4 * y_offset);
            sprintf(ausgabe,"%5i°", cal_winkel);
            u8g2.print(ausgabe);
            u8g2.setCursor(10, 5 * y_offset);
            u8g2.print(HONEY_GATE[lingo]);
            sprintf(ausgabe,"%6s", quetschhan);
            u8g2.print(ausgabe);
            u8g2.setCursor(1, (7 * y_offset) + 7); 
            u8g2.print("*");
            u8g2.setCursor(10, (7 * y_offset) + 5);
            u8g2.print(ABORT[lingo]);
            u8g2.sendBuffer();
          #endif
          #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
            if (k == 1) {
              gfx->fillScreen(BACKGROUND);
              gfx->setTextColor(TEXT);
              gfx->setFont(Punk_Mono_Bold_240_150);
              x_pos = CenterPosX(INA219_SETUP[lingo], 14, 320);
              gfx->setCursor(x_pos, 25);
              gfx->println(INA219_SETUP[lingo]);
              gfx->drawLine(0, 30, 320, 30, TEXT);
              gfx->setCursor(5, 30+(3 * y_offset_tft));
              change = true;
              k++;
            }
            if (change) {
              gfx->setTextColor(TEXT);
              gfx->setCursor(5, 30+(1 * y_offset_tft));
              gfx->print(kalibration_status);
              gfx->fillRect(205, 27+(3 * y_offset_tft)-19, 110, 3 * y_offset_tft, BACKGROUND);
              gfx->setCursor(5, 30+(3 * y_offset_tft));
              gfx->printf("%s:", CURRENT);
              sprintf(ausgabe,"%4imA", current_mA);
              int y = get_length(ausgabe);
              gfx->setCursor(320 - y * 14 - 5, 30+(3 * y_offset_tft));
              gfx->print(ausgabe);
              gfx->setCursor(5, 30+(4 * y_offset_tft));
              gfx->printf("%s:", ANGLE);
              sprintf(ausgabe,"%3i°", cal_winkel);
              y = get_length(ausgabe);
              gfx->setCursor(320 - y * 14 - 5, 30+(4 * y_offset_tft));
              gfx->print(ausgabe);
              gfx->setCursor(5, 30+(5 * y_offset_tft));
              gfx->print(HONEY_GATE[lingo]);
              sprintf(ausgabe,"%5s", quetschhan);
              y = get_length(ausgabe);
              gfx->setCursor(320 - y * 14 - 5, 30+(5 * y_offset_tft));
              gfx->print(ausgabe);
              gfx->setTextColor(MARKER);
              gfx->setCursor(5, 30+(7 * y_offset_tft));
              gfx->print(ABORT[lingo]);
              change = false;
            }
          #endif
          if (millis() - scaletime >= 800 and !measurement_run) {
            SERVO_WRITE(cal_winkel);
            quetschhan = CLOSE[lingo];
            measurement_run = true;
            scaletime = millis();
            #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
              change = true;
            #endif
          }
          else if (millis() - scaletime >= 800 and measurement_run) {
            current_mA = GetCurrent(50);
            if (current_mA > current_servo - 30) {   //30mA unter dem max. Wert Kalibrieren
              SERVO_WRITE(90);
              quetschhan = OPEN[lingo];
              cal_winkel++;
              #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
                change = true;
              #endif
            }
            else {
              cal_done = true;
              kalibration_status = CALIBRATION_DONE[lingo];
              winkel_min = cal_winkel;
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
            #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
              gfx->fillScreen(BACKGROUND);
              gfx->setTextColor(TEXT);
              x_pos = CenterPosX(INA219_SETUP[lingo], 14, 320);
              gfx->setCursor(x_pos, 25);
              gfx->println(INA219_SETUP[lingo]);
              gfx->drawLine(0, 30, 320, 30, TEXT);
            #endif
            j = 0;
            k = 0;
            winkel_min = lastwinkel_min;
            cal_done = true;
            wert_aendern = false;
            menuitem_used = 2;
            initRotaries(SW_MENU, 0, 0, menuitem_used, 1);
          }
        }
        while (cal_done and k > 1) {
          current_mA = GetCurrent(50);
          #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
            if (wert_old != current_mA) {
              change = true;
              wert_old = current_mA;
            }
          #endif
          #if DISPLAY_TYPE == 1 or DISPLAY_TYPE == 2 or DISPLAY_TYPE == 99
            u8g2.setFont(u8g2_font_courB08_tf);
            u8g2.clearBuffer();
            u8g2.setCursor(10, 1 * y_offset);
            u8g2.print(kalibration_status);
            u8g2.setCursor(10, 3 * y_offset);
            u8g2.printf("%s:",CURRENT);
            u8g2.setCursor(90, 3 * y_offset);
            sprintf(ausgabe,"%4imA", current_mA);
            u8g2.print(ausgabe);
            u8g2.setCursor(10, 4 * y_offset);
            u8g2.printf("%s:",ANGLE);
            u8g2.setCursor(90, 4 * y_offset);
            sprintf(ausgabe,"%5i°", winkel_min);
            u8g2.print(ausgabe);
            u8g2.setCursor(1, (7 * y_offset) + 7); 
            u8g2.print("*");
            u8g2.setCursor(10, (7 * y_offset) + 5);
            u8g2.print(SAVE[lingo]);
            u8g2.sendBuffer();
          #endif
          #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
            if (k==2) {
              gfx->setTextColor(TEXT);
              gfx->fillRect(0, 27+(1 * y_offset_tft)-19, 320, 25, BACKGROUND);
              gfx->fillRect(0, 27+(5 * y_offset_tft)-19, 320, 25, BACKGROUND);
              gfx->fillRect(0, 27+(7 * y_offset_tft)-19, 320, 25, BACKGROUND);
              gfx->setCursor(5, 30+(1 * y_offset_tft));
              gfx->print(kalibration_status);
              gfx->setTextColor(MARKER);
              gfx->setCursor(5, 30+(7 * y_offset_tft));
              gfx->print(SAVE[lingo]);
              gfx->setTextColor(TEXT);
              k++;
            }
            if (change) {
              gfx->fillRect(200, 27+(3 * y_offset_tft)-19, 120, 25, BACKGROUND);
              gfx->setCursor(5, 30+(3 * y_offset_tft));
              gfx->printf("%s:",ANGLE);
              sprintf(ausgabe,"%4imA", current_mA);
              int y = get_length(ausgabe);
              gfx->setCursor(320 - y * 14 - 5, 30+(3 * y_offset_tft));
              gfx->print(ausgabe);
              change = false;
            }
          #endif
          //verlassen
          if (digitalRead(button_stop_pin) == HIGH) {
            while(digitalRead(button_stop_pin) == HIGH) {
              delay(1);
            }
            j = 0;
            k = 0;
            wert_aendern = false;
            menuitem_used = 2;
            winkel_min = lastwinkel_min;
            initRotaries(SW_MENU, 0, 0, menuitem_used, 1);
            #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
              gfx->fillScreen(BACKGROUND);
              x_pos = CenterPosX(INA219_SETUP[lingo], 14, 320);
              gfx->setCursor(x_pos, 25);
              gfx->println(INA219_SETUP[lingo]);
              gfx->drawLine(0, 30, 320, 30, TEXT);
            #endif
          }
          if (digitalRead(SELECT_SW) == SELECT_PEGEL) {
            while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
              delay(1);
            }
            j = 0;
            k = 0;
            modus = -1;
            wert_aendern = false;
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

void setupLanguage(void) {
  //sizeof(LANGUAGE2)/sizeof(LANGUAGE2[0])
  int menuitem;
  int menuitem_used = 1;
  int last_lingo = lingo;
  bool wert_aendern = false;
  int x_pos;
  #if DISPLAY_TYPE == 1 or DISPLAY_TYPE == 2 or DISPLAY_TYPE == 99
    int y_offset = 8;
  #endif
  #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
    int y_offset_tft = 28;
    int wert_old = -1;
    bool change = false;
    bool make_changes = false;
    gfx->fillScreen(BACKGROUND);
    int MenuepunkteAnzahl = 2;
    gfx->setTextColor(TEXT);
    gfx->setFont(Punk_Mono_Bold_240_150);
    x_pos = CenterPosX(LANGUAGE1[lingo], 14, 320);
    gfx->setCursor(x_pos, 25);
    gfx->println(LANGUAGE1[lingo]);
    gfx->drawLine(0, 30, 320, 30, TEXT);
  #endif
  initRotaries(SW_MENU, 0, 0, menuitem_used, 1);
  i = 1;
  while (i > 0) {
    if (digitalRead(button_stop_pin) == HIGH  or digitalRead(switch_setup_pin) == LOW) {
      while (digitalRead(button_stop_pin) == HIGH) {
        delay(1);
      }
      lingo = last_lingo;
      modus = -1;
      return;
    }
    if (wert_aendern == false) {
      menuitem = getRotariesValue(SW_MENU);
      #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
        pos = menuitem;
      #endif
      if (menuitem == menuitem_used) {
        menuitem = 6;
      }
    }
    else {
      switch (menuitem) {
        case 0: lingo    = getRotariesValue(SW_MENU);
                break;
      }
    }
    #if DISPLAY_TYPE == 1 or DISPLAY_TYPE == 2 or DISPLAY_TYPE == 99
      u8g2.setFont(u8g2_font_courB08_tf);
      u8g2.clearBuffer();
      u8g2.setCursor(10, 1 * y_offset);
      u8g2.print(LANGUAGE2[lingo]);
      u8g2.setCursor(10, 2 * y_offset);
      u8g2.setCursor(10, (7 * y_offset) + 5);
      u8g2.print(SAVE[lingo]);
      if (wert_aendern == false && menuitem < menuitem_used) {
        u8g2.setCursor(1, 10+((menuitem)*y_offset)); u8g2.print("*");
      }
      else if (wert_aendern == true && menuitem < menuitem_used) {
        u8g2.setCursor(1, 8+((menuitem)*y_offset)); u8g2.print("-");
      }
      else if (wert_aendern == false && menuitem == 6) {
        u8g2.setCursor(1, 10+((menuitem)*y_offset) + 5); u8g2.print("*");
      }
      u8g2.sendBuffer();
    #endif
    #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
      const char *menuepunkte[MenuepunkteAnzahl - 1] = {LANGUAGE2[lingo], SAVE[lingo]};
      int j = 0;
      while(j < MenuepunkteAnzahl) {
        gfx->setTextColor(TEXT);
        if (j == pos and wert_aendern == false) {
          gfx->setTextColor(MARKER);
        }
        if (j < MenuepunkteAnzahl - 1) {
          switch (j) {
            case 0: sprintf(ausgabe,"%s", LANGUAGE2[lingo]);
                    if (wert_old != lingo and wert_aendern == true and j == pos) {
                      change = true;
                      wert_old = lingo;
                    }
                    break;
          }
          if (change) {
            gfx->fillRect(0, 0, 320, 30, BACKGROUND);
            gfx->fillRect(0, 31, 320, 209, BACKGROUND);
            gfx->setTextColor(TEXT);
            x_pos = CenterPosX(LANGUAGE1[lingo], 14, 320);
            gfx->setCursor(x_pos, 25);
            gfx->println(LANGUAGE1[lingo]);
            gfx->setTextColor(MARKER);
            int y = get_length(ausgabe);
            gfx->setCursor(320 - y * 14 - 5, 30+((j+1) * y_offset_tft));
            gfx->print(ausgabe);
            change = false;
          }
          else {
            if (!make_changes) {
              gfx->setCursor(5, 30+((j+1) * y_offset_tft));
              gfx->print(menuepunkte[j]);
              if (j == pos and wert_aendern == true) {
                gfx->setTextColor(MARKER);
              }
            }
          }
        }
        else {
            gfx->setCursor(5, 30+(7 * y_offset_tft));
            gfx->print(menuepunkte[j]);
        }
        j++;
      }
    #endif
    // Menupunkt zum Ändern ausgewählt
    if (digitalRead(SELECT_SW) == SELECT_PEGEL && menuitem < menuitem_used  && wert_aendern == false) {
      while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
        delay(1);
      }
      switch (menuitem) { 
        case 0: initRotaries(SW_MENU, lingo, 0, (sizeof(LANGUAGE2)/sizeof(LANGUAGE2[0])) - 1, 1);
                break;
      }
      #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
        wert_old = -1;
        make_changes = true;
      #endif
      wert_aendern = true;
    }
    // Änderung im Menupunkt übernehmen
    if (digitalRead(SELECT_SW) == SELECT_PEGEL && menuitem < menuitem_used  && wert_aendern == true) {
      while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
        delay(1);
      }
      #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
        gfx->fillRect(0, 31, 320, 209, BACKGROUND);
        make_changes = false;
      #endif
      initRotaries(SW_MENU, menuitem, 0, menuitem_used, 1);
      wert_aendern = false;
    }
    // Menu verlassen 
    if (digitalRead(SELECT_SW) == SELECT_PEGEL && menuitem == 6) {
      while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
        delay(1);
      }
      #if DISPLAY_TYPE == 1 or DISPLAY_TYPE == 2 or DISPLAY_TYPE == 99
        u8g2.setCursor(111, ((menuitem + 1) * y_offset) + 5);
        u8g2.print("OK");
        u8g2.sendBuffer();
      #endif
      #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
        gfx->setCursor(283, 30+(7 * y_offset_tft));
        gfx->print("OK");
      #endif
      delay(1000);
      modus = -1;
      i = 0;
    }
  }
}

void setupDrehteller(void) {
  //Start Menue
  #if DREHTELLER == 1
    int x_pos;
    int k = 0;
    unsigned long time;
    int menuitem_1;
    int MenuepunkteAnzahl_1 = 3;
    int last_menu_pos_1 = 0;
    const char *menuepunkte_1[MenuepunkteAnzahl_1 - 1] = {TURNTABLE[lingo], INIT_TURNTABLE[lingo], SAVE[lingo]};
    int menuitem_2;
    int MenuepunkteAnzahl_2 = 4;
    int last_menu_pos_2 = 0;
    const char *menuepunkte_2[MenuepunkteAnzahl_2 - 1] = {TURNTABLE[lingo], SETUP_TURNTABLE[lingo], SETUP_DROPPRODECTION[lingo], SAVE[lingo]};
    int menuitem_3;
    int MenuepunkteAnzahl_3 = 5;
    int last_menu_pos_3 = 0;
    const char *menuepunkte_3[MenuepunkteAnzahl_3 - 1] = {MOVE_JAR[lingo], CENTER_JAR[lingo], SPEED_INIT[lingo], SPEED_RUN[lingo], SAVE[lingo]};
    int menuitem_4;
    int MenuepunkteAnzahl_4 = 7;
    int last_menu_pos_4 = 0;
    const char *menuepunkte_4[MenuepunkteAnzahl_4 - 1] = {OPEN_DROPPROTECTION[lingo], CLOSE_DROPPROTECTION[lingo], SPEED_DROPPROTECTION[lingo], WAIT_TO_OPEN_DP[lingo], DP_MIN_ANGLE[lingo], DP_MAX_ANGLE[lingo], SAVE[lingo]};
    int last_use_turntable = use_turntable;
    bool wert_aendern = false;
    bool turntable_running = false;
    bool center_jar_running = false;
    bool ts_open_running = false;
    bool ts_close_running = false;
    bool ts_angle_running = false;
    unsigned long turntable_millis;
    unsigned long prev_millis = 0;
    bool esp_now_change = true;
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
    String esp_now_wait = "";
    esp_now_msg_recived = false;
    #if DISPLAY_TYPE == 1 or DISPLAY_TYPE == 2 or DISPLAY_TYPE == 99
      int y_offset = 8;
    #endif
    #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
      int y_offset_tft = 28;
      bool change = false;
      int wert_old = -1;
      String esp_now_wait_old = "...";
      gfx->fillScreen(BACKGROUND);
      gfx->setTextColor(TEXT);
      gfx->setFont(Punk_Mono_Bold_240_150);
      x_pos = CenterPosX(TURNTABLE[lingo], 14, 320);
      gfx->setCursor(x_pos, 25);
      gfx->println(TURNTABLE[lingo]);
      gfx->drawLine(0, 30, 320, 30, TEXT);
    #endif
    initRotaries(SW_MENU, 0, 0, MenuepunkteAnzahl_1 -1, 1);
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
    if(strcmp(myReceivedMessage.text, "ok_init") == 0 or strcmp(myReceivedMessage.text, "nok_init") == 0) {
      if(strcmp(myReceivedMessage.text, "ok_init") == 0) {
        turntable_init = true;
      }
      i = 1;
    }
    else {
      i = 0;
      esp_now_msg_recived = false;
      #if DISPLAY_TYPE == 1 or DISPLAY_TYPE == 2 or DISPLAY_TYPE == 99
        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_courB08_tf);
        x_pos = CenterPosX(CONNECTION[lingo], 6, 128);                 //ein Zeichen ist etwa 6 Pixel breit
        u8g2.setCursor(x_pos,22);  
        u8g2.print(CONNECTION[lingo]);
        x_pos = CenterPosX(FAILED[lingo], 6, 128);                     //ein Zeichen ist etwa 6 Pixel breit
        u8g2.setCursor(x_pos,42);  
        u8g2.print(FAILED[lingo]);
        u8g2.sendBuffer();
      #endif
      #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
        gfx->setTextColor(RED);
        gfx->setFont(Punk_Mono_Bold_320_200);
        x_pos = CenterPosX(CONNECTION[lingo], 18, 320);
        gfx->setCursor(x_pos, 120);
        gfx->println(CONNECTION[lingo]);
        x_pos = CenterPosX(FAILED[lingo], 18, 320);
        gfx->setCursor(x_pos, 150);
        gfx->println(FAILED[lingo]);
      #endif
      delay(4000);
    }
    while (i > 0) {
      //Menue 1
      while (i == 1) {
        if (turntable_init == true) {
          i = 2;
          initRotaries(SW_MENU, 0, 0, MenuepunkteAnzahl_2 -1, 1);
          esp_now_msg_recived = false;
          memset(&myReceivedMessage, 0, sizeof(myReceivedMessage));
          memset(&myMessageToBeSent, 0, sizeof(myMessageToBeSent));

          #if DISPLAY_TYPE == 1 or DISPLAY_TYPE == 2 or DISPLAY_TYPE == 99
            u8g2.clearBuffer();
            u8g2.sendBuffer();
          #endif
          #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
            gfx->fillRect(0, 32, 320, 208, BACKGROUND);
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
          modus = -1;
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
          #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
            esp_now_change = true;
          #endif
        }
        if (turntable_running == true) {
          initRotaries(SW_MENU, last_menu_pos_1, 0, MenuepunkteAnzahl_1 - 1, 1);                         //Wenn einer am rotary dreht wider zurücksetzen
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
            #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
              esp_now_change = true;
            #endif
          }
          else if (millis() - prev_millis > 1000 and (turntable_init == 1 or menuitem_1 == 1)) {
            prev_millis = millis();
            if (esp_now_wait.length() < 3) {esp_now_wait = esp_now_wait + ".";}
            else {esp_now_wait = "";}
            #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
              esp_now_change = true;
            #endif
          }
          if (esp_now_msg_recived == true) {
            if (last_menu_pos_1 == 1) {
              if(strcmp(myReceivedMessage.text, "ok_init_done") == 0) {
                turntable_init = true;
              }
              #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
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
        if (wert_aendern == false) {
          menuitem_1 = getRotariesValue(SW_MENU);
          if (use_turntable == 0) {
            initRotaries(SW_MENU, menuitem_1, 0, 1, 1);
          }
          else {
            initRotaries(SW_MENU, menuitem_1, 0, MenuepunkteAnzahl_1 - 1, 1);
          }
          #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
            pos = menuitem_1;
            if ((use_turntable == 0 and menuitem_1 == 1) or (menuitem_1 == 2 and use_turntable == 1 and turntable_init == false)) {
              pos = MenuepunkteAnzahl_1 -1;
            }
          #endif
          if (menuitem_1 == MenuepunkteAnzahl_1 - 1 or (use_turntable == 0 and menuitem_1 == 1) or (turntable_init == false and menuitem_1 == 2)) {
            menuitem_1 = 7;
          }
        }
        else {
          switch (menuitem_1) {
            case 0: use_turntable = getRotariesValue(SW_MENU);
                    break;
          }
        }
        #if DISPLAY_TYPE == 1 or DISPLAY_TYPE == 2 or DISPLAY_TYPE == 99
          u8g2.clearBuffer();
          u8g2.setFont(u8g2_font_courB08_tf);
          u8g2.setCursor(10, 1 * y_offset);
          u8g2.print(menuepunkte_1[0]);
          u8g2.setCursor(105, 1 * y_offset);
          sprintf(ausgabe,"%3s", (use_turntable==0?OFF:ON));
          u8g2.print(ausgabe);
          if (use_turntable == 1 and turntable_init == false) {
            u8g2.setCursor(10, 2 * y_offset);
            u8g2.print(menuepunkte_1[1]);
            if (last_menu_pos_1 == 1 and turntable_running == true) {
              u8g2.setCursor(105, 2 * y_offset);
              u8g2.print(esp_now_wait);
            }
            else {
              u8g2.setCursor(105, 2 * y_offset);
              sprintf(ausgabe,"%3s", (turntable_init==false?NOKAY:OKKAY));
              u8g2.print(ausgabe);
            }
          }
          u8g2.setCursor(10, (7 * y_offset) + 5);
          u8g2.print(menuepunkte_1[2]);
          if (wert_aendern == false && menuitem_1 < MenuepunkteAnzahl_1 - 1) {
            u8g2.setCursor(1, 10+((menuitem_1)*y_offset)); u8g2.print("*");
          }
          else if (wert_aendern == true && menuitem_1 < MenuepunkteAnzahl_1 - 1) {
            u8g2.setCursor(1, 8+((menuitem_1)*y_offset)); u8g2.print("-");
          }
          else if (wert_aendern == false && menuitem_1 == 7) {
            u8g2.setCursor(1, 10+((menuitem_1 - 1)*y_offset) + 5); u8g2.print("*");
          }
          u8g2.sendBuffer();
        #endif
        #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
          int j = 0;
          while(j < MenuepunkteAnzahl_1) {
            gfx->setTextColor(TEXT);
            if (j == pos and wert_aendern == false) {
              gfx->setTextColor(MARKER);
            }
            if (j < MenuepunkteAnzahl_1 - 1) {
                gfx->setCursor(5, 30+((j+1) * y_offset_tft));
                gfx->print(menuepunkte_1[j]);
              if (j == pos and wert_aendern == true) {
                gfx->setTextColor(MARKER);
              }
              switch (j) {
                case 0: sprintf(ausgabe,"%s", use_turntable==false?OFF:ON);
                        if (wert_old != use_turntable and wert_aendern == true and j == pos) {
                          change = true;
                          wert_old = use_turntable;
                        }
                        break;
                case 1: sprintf(ausgabe,"");
                        if (turntable_running == false and use_turntable == true) {
                          sprintf(ausgabe,"%s", turntable_init==false?NOKAY:OKKAY);
                        }
                        else if (esp_now_wait_old != esp_now_wait and turntable_running == true and turntable_init == false) {
                          sprintf(ausgabe,"%s", esp_now_wait);
                          esp_now_wait_old = esp_now_wait;
                          esp_now_change = true;
                        }
                        break;
              }
              if (change or esp_now_change) {
                gfx->fillRect(251, 27+((j+1) * y_offset_tft)-19, 64, 27, BACKGROUND);
                if (use_turntable == false) {
                  gfx->fillRect(0, 65, 320, 135, BACKGROUND);
                }
                change = false;
                esp_now_change = false;
              }
              int y = get_length(ausgabe);
              gfx->setCursor(320 - y * 14 - 5, 30+((j+1) * y_offset_tft));
              gfx->print(ausgabe);
            }
            else {
              gfx->setCursor(10, 30+(7 * y_offset_tft));
              gfx->print(menuepunkte_1[j]);
            }
            if (use_turntable == false) {j++;}
            j++;
          }
        #endif
        // Menupunkt zum ändern ausgewählt
        if (digitalRead(SELECT_SW) == SELECT_PEGEL && menuitem_1 < MenuepunkteAnzahl_1 - 1  && wert_aendern == false) {
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
          #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
            wert_old = -1;
          #endif
          wert_aendern = true;
        }
        // Änderung im Menupunkt übernehmen
        if ((digitalRead(SELECT_SW) == SELECT_PEGEL && menuitem_1 < MenuepunkteAnzahl_1 - 1  && wert_aendern == true) or (turntable_running == true and wert_aendern == true)) {
          while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
            delay(1);
          }
          rotary_select = SW_MENU;
          initRotaries(SW_MENU, menuitem_1, 0, MenuepunkteAnzahl_1 - 1, 1);
          wert_aendern = false;
        }
        // Menu verlassen
        if (digitalRead(SELECT_SW) == SELECT_PEGEL && menuitem_1 == 7 && turntable_running == false) {
          while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
            delay(1);
          }
          #if DISPLAY_TYPE == 1 or DISPLAY_TYPE == 2 or DISPLAY_TYPE == 99
            u8g2.setCursor(111, (menuitem_1 * y_offset) + 5);
            u8g2.print("OK");
            u8g2.sendBuffer();
          #endif
          #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
            gfx->setCursor(283, 30+(7 * y_offset_tft));
            gfx->print("OK");
          #endif
          esp_now_msg_recived = false;
          delay(1000);
          i = 0;
        }
      }
      while (i == 2) {
        if (esp_now_msg_recived == true and strcmp(myReceivedMessage.text, "init_error") == 0) {
          esp_now_msg_recived = false;
          turntable_init = false;
          turntable_running = false;
          turntable_millis = 0;
          prev_millis = 0;
          esp_now_wait = "";
          #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
            esp_now_wait_old = "...";
          #endif
        }
        if (turntable_init == false) {
          i = 1;
          initRotaries(SW_MENU, 0, 0, MenuepunkteAnzahl_1 -1, 1);
          esp_now_msg_recived = false;
          memset(&myReceivedMessage, 0, sizeof(myReceivedMessage));
          memset(&myMessageToBeSent, 0, sizeof(myMessageToBeSent));
          #if DISPLAY_TYPE == 1 or DISPLAY_TYPE == 2 or DISPLAY_TYPE == 99
            u8g2.clearBuffer();
            u8g2.sendBuffer();
          #endif
          #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
            gfx->fillRect(0, 32, 320, 208, BACKGROUND);
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
          modus = -1;
          memset(&myReceivedMessage, 0, sizeof(myReceivedMessage));
          memset(&myMessageToBeSent, 0, sizeof(myMessageToBeSent));
          return;
        }
        if (wert_aendern == false) {
          menuitem_2 = getRotariesValue(SW_MENU);
          if (use_turntable == 0) {
            initRotaries(SW_MENU, menuitem_2, 0, 1, 1);
          }
          else {
            initRotaries(SW_MENU, menuitem_2, 0, MenuepunkteAnzahl_2 - 1, 1);
          }
          #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
            pos = menuitem_2;
            if ((use_turntable == 0 and menuitem_2 == 1)) {
              pos = MenuepunkteAnzahl_2 -1;
            }
          #endif
          if (menuitem_2 == MenuepunkteAnzahl_2 - 1 or (use_turntable == 0 and menuitem_2 == 1)) {
            menuitem_2 = 7;
          }
        }
        else {
          switch (menuitem_2) {
            case 0: use_turntable = getRotariesValue(SW_MENU);
                    break;
          }
        }
        #if DISPLAY_TYPE == 1 or DISPLAY_TYPE == 2 or DISPLAY_TYPE == 99
          u8g2.clearBuffer();
          u8g2.setFont(u8g2_font_courB08_tf);
          u8g2.setCursor(10, 1 * y_offset);
          u8g2.print(menuepunkte_2[0]);
          u8g2.setCursor(105, 1 * y_offset);
          sprintf(ausgabe,"%3s", (use_turntable==0?OFF:ON));
          u8g2.print(ausgabe);
          if (use_turntable == 1) {
            u8g2.setCursor(10, 2 * y_offset);
            u8g2.print(menuepunkte_2[1]);
            u8g2.setCursor(10, 3 * y_offset);
            u8g2.print(menuepunkte_2[2]);
          }
          u8g2.setCursor(10, (7 * y_offset) + 5);
          u8g2.print(menuepunkte_2[3]);
          if (wert_aendern == false && menuitem_2 < MenuepunkteAnzahl_2 - 1) {
            u8g2.setCursor(1, 10+((menuitem_2)*y_offset)); u8g2.print("*");
          }
          else if (wert_aendern == true && menuitem_2 < MenuepunkteAnzahl_2 - 1) {
            u8g2.setCursor(1, 8+((menuitem_2)*y_offset)); u8g2.print("-");
          }
          else if (wert_aendern == false && menuitem_2 == 7) {
            u8g2.setCursor(1, 10+((menuitem_2 - 1)*y_offset) + 5); u8g2.print("*");
          }
          u8g2.sendBuffer();
        #endif
        #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
          int j = 0;
          while(j < MenuepunkteAnzahl_2) {
            gfx->setTextColor(TEXT);
            if (j == pos and wert_aendern == false) {
              gfx->setTextColor(MARKER);
            }
            if (j < MenuepunkteAnzahl_2 - 1) {
              gfx->setCursor(5, 30+((j+1) * y_offset_tft));
              gfx->print(menuepunkte_2[j]);
              if (j == pos and wert_aendern == true) {
                gfx->setTextColor(MARKER);
              }
              switch (j) {
                case 0: sprintf(ausgabe,"%s", use_turntable==false?OFF:ON);
                        if (wert_old != use_turntable and wert_aendern == true and j == pos) {
                          change = true;
                          wert_old = use_turntable;
                        }
                        break;
                case 1: sprintf(ausgabe,"");
                        break;
                case 2: sprintf(ausgabe,"");
                        break;
              }
              if (change) {
                gfx->fillRect(251, 27+((j+1) * y_offset_tft)-19, 64, 27, BACKGROUND);
                if (use_turntable == false) {
                  gfx->fillRect(0, 65, 320, 135, BACKGROUND);
                }
                change = false;
              }
              int y = get_length(ausgabe);
              gfx->setCursor(320 - y * 14 - 5, 30+((j+1) * y_offset_tft));
              gfx->print(ausgabe);
            }
            
            else {
              gfx->setCursor(10, 30+(7 * y_offset_tft));
              gfx->print(menuepunkte_2[j]);
            }
            if (use_turntable == false) {j++;j++;}
            j++;
          }
        #endif
        // Menupunkt zum ändern ausgewählt
        if (digitalRead(SELECT_SW) == SELECT_PEGEL && menuitem_2 < MenuepunkteAnzahl_2 - 1  && wert_aendern == false) {
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
          }
          #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
            wert_old = -1;
          #endif
          wert_aendern = true;
          if (i == 3 or i == 4) {
            menuitem_3 = pos = 0;
            wert_aendern = false;
            #if DISPLAY_TYPE == 1 or DISPLAY_TYPE == 2 or DISPLAY_TYPE == 99
              u8g2.clearBuffer();
              u8g2.sendBuffer();
            #endif
            #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
              gfx->fillRect(0, 32, 320, 208, BACKGROUND);
            #endif
            break;
          }
        }
        // Änderung im Menupunkt übernehmen
        if ((digitalRead(SELECT_SW) == SELECT_PEGEL && menuitem_2 < MenuepunkteAnzahl_2 - 1  && wert_aendern == true)) {
          while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
            delay(1);
          }
          rotary_select = SW_MENU;
          initRotaries(SW_MENU, menuitem_2, 0, MenuepunkteAnzahl_2 - 1, 1);
          wert_aendern = false;
        }
        // Menu verlassen
        if (digitalRead(SELECT_SW) == SELECT_PEGEL && menuitem_2 == 7) {
          while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
            delay(1);
          }
          #if DISPLAY_TYPE == 1 or DISPLAY_TYPE == 2 or DISPLAY_TYPE == 99
            u8g2.setCursor(111, (menuitem_2 * y_offset) + 5);
            u8g2.print("OK");
            u8g2.sendBuffer();
          #endif
          #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
            gfx->setCursor(283, 30+(7 * y_offset_tft));
            gfx->print("OK");
          #endif
          esp_now_msg_recived = false;
          delay(1000);
          i = 0;
        }
      }
      if (i == 3) {
        esp_now_msg_recived = false;
        memset(&myReceivedMessage, 0, sizeof(myReceivedMessage));
        strcpy(myMessageToBeSent.text, "speed_init");
        espnow_send_data();
        turntable_millis = millis();
        while (millis() - turntable_millis < 3000 and esp_now_msg_recived == false) {  //brechce ab wenn in 3 sek keine rückmeldung kommt
          delay(10);
        }
        speed_init = speed_init_old = myReceivedMessage.value;
        esp_now_msg_recived = false;
        memset(&myReceivedMessage, 0, sizeof(myReceivedMessage));
        strcpy(myMessageToBeSent.text, "speed_run");
        espnow_send_data();
        turntable_millis = millis();
        while (millis() - turntable_millis < 3000 and esp_now_msg_recived == false) {  //brechce ab wenn in 3 sek keine rückmeldung kommt
          delay(10);
        }
        speed_run = speed_run_old = myReceivedMessage.value;
        initRotaries(SW_MENU, menuitem_3, 0, MenuepunkteAnzahl_3 - 1, 1);
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
          #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
            esp_now_wait_old = "...";
          #endif
        }
        if (turntable_init == false) {
          i = 1;
          initRotaries(SW_MENU, 0, 0, MenuepunkteAnzahl_1 -1, 1);
          esp_now_msg_recived = false;
          memset(&myReceivedMessage, 0, sizeof(myReceivedMessage));
          memset(&myMessageToBeSent, 0, sizeof(myMessageToBeSent));
          #if DISPLAY_TYPE == 1 or DISPLAY_TYPE == 2 or DISPLAY_TYPE == 99
            u8g2.clearBuffer();
            u8g2.sendBuffer();
          #endif
          #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
            gfx->fillRect(0, 32, 320, 208, BACKGROUND);
          #endif
          break;
        }
        if (digitalRead(button_stop_pin) == HIGH and turntable_running == false) {
          while (digitalRead(button_stop_pin) == HIGH) {
            delay(1);
          }
          #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
            wert_old = -1;
          #endif
          wert_aendern = false;
          rotary_select = SW_MENU;
          initRotaries(SW_MENU, menuitem_2, 0, MenuepunkteAnzahl_2 - 1, 1);
          esp_now_msg_recived = false;
          memset(&myReceivedMessage, 0, sizeof(myReceivedMessage));
          memset(&myMessageToBeSent, 0, sizeof(myMessageToBeSent));
          i = 2;
          #if DISPLAY_TYPE == 1 or DISPLAY_TYPE == 2 or DISPLAY_TYPE == 99
            u8g2.clearBuffer();
            u8g2.sendBuffer();
          #endif
          #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
            gfx->fillRect(0, 32, 320, 208, BACKGROUND);
          #endif
          break;
        }
        if (digitalRead(switch_setup_pin) == LOW) {
          esp_now_msg_recived = false;
          rotary_select = SW_MENU;
          modus = -1;
          memset(&myReceivedMessage, 0, sizeof(myReceivedMessage));
          memset(&myMessageToBeSent, 0, sizeof(myMessageToBeSent));
          return;
        }
        if (turntable_running == true) {
          initRotaries(SW_MENU, last_menu_pos_3, 0, MenuepunkteAnzahl_3 - 1, 1);                         //Wenn einer am rotary dreht wider zurücksetzen
          if (millis() - turntable_millis > 60000 or digitalRead(button_stop_pin) == HIGH) {             //Timeaut, wenn der Befehl nicht in 60s geschaft wird oder Abbrechen wenn Stop Taste gedrückt wird
            while (digitalRead(button_stop_pin) == HIGH) {
              delay(1);
            }
            strcpy(myMessageToBeSent.text, "stop");
            espnow_send_data();
            esp_now_msg_recived = false;
            wert_aendern = false;
            turntable_init = false;
            turntable_running = false;
            turntable_millis = 0;
            prev_millis = 0;
            esp_now_wait = "";
            #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
              esp_now_wait_old = "...";
            #endif
            initRotaries(SW_MENU, 1, 0, 1, 1);
            memset(&myReceivedMessage, 0, sizeof(myReceivedMessage));
            #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
              esp_now_change = true;
            #endif
          }
          else if (millis() - prev_millis > 1000 and menuitem_3 == 0) {
            prev_millis = millis();
            if (esp_now_wait.length() < 3) {esp_now_wait = esp_now_wait + ".";}
            else {esp_now_wait = "";}
            #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
              esp_now_change = true;
            #endif
          }
          if (esp_now_msg_recived == true) {
            //Werte zurücksetzen
            wert_aendern = false;
            esp_now_msg_recived = false;
            turntable_running = false;
            turntable_millis = 0;
            esp_now_wait = "";
            #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
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
        if (wert_aendern == false) {
          menuitem_3 = getRotariesValue(SW_MENU);
          initRotaries(SW_MENU, menuitem_3, 0, MenuepunkteAnzahl_3 - 1, 1);
          #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
            pos = menuitem_3;
          #endif
          if (menuitem_3 == MenuepunkteAnzahl_3 - 1) {
            menuitem_3 = 7;
          }
        }
        else {
          switch (menuitem_3) {
            case 1: jar_center_pos = getRotariesValue(SW_MENU);
                    break;
            case 2: speed_init = getRotariesValue(SW_MENU);
                    break;
            case 3: speed_run = getRotariesValue(SW_MENU);
                    break;
          }
        }
        #if DISPLAY_TYPE == 1 or DISPLAY_TYPE == 2 or DISPLAY_TYPE == 99
          u8g2.clearBuffer();
          u8g2.setFont(u8g2_font_courB08_tf);
          u8g2.setCursor(10, 1 * y_offset);
          u8g2.print(menuepunkte_3[0]);
          u8g2.setCursor(105, 1 * y_offset);
          sprintf(ausgabe,"%4s", esp_now_wait);
          u8g2.print(ausgabe);
          u8g2.setCursor(10, 2 * y_offset);
          u8g2.print(menuepunkte_3[1]);
          u8g2.setCursor(105, 2 * y_offset);
          sprintf(ausgabe,"%4i", jar_center_pos);
          u8g2.print(ausgabe);
          u8g2.setCursor(10, 3 * y_offset);
          u8g2.print(menuepunkte_3[2]);
          u8g2.setCursor(105, 3 * y_offset);
          sprintf(ausgabe,"%4i", speed_init);
          u8g2.print(ausgabe);
          u8g2.setCursor(10, 4 * y_offset);
          u8g2.print(menuepunkte_3[3]);
          u8g2.setCursor(105, 4 * y_offset);
          sprintf(ausgabe,"%4i", speed_run);
          u8g2.print(ausgabe);
          u8g2.setCursor(10, (7 * y_offset) + 5);
          u8g2.print(menuepunkte_3[4]);
          if (wert_aendern == false && menuitem_3 < MenuepunkteAnzahl_3 - 1) {
            u8g2.setCursor(1, 10+((menuitem_3)*y_offset)); u8g2.print("*");
          }
          else if (wert_aendern == true && menuitem_3 < MenuepunkteAnzahl_3 - 1) {
            u8g2.setCursor(1, 8+((menuitem_3)*y_offset)); u8g2.print("-");
          }
          else if (wert_aendern == false && menuitem_3 == 7) {
            u8g2.setCursor(1, 10+((menuitem_3 - 1)*y_offset) + 5); u8g2.print("*");
          }
          u8g2.sendBuffer();
        #endif
        #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
          int j = 0;
          while(j < MenuepunkteAnzahl_3) {
            gfx->setTextColor(TEXT);
            if (j == pos and wert_aendern == false) {
              gfx->setTextColor(MARKER);
            }
            if (j < MenuepunkteAnzahl_3 - 1) {
              gfx->setCursor(5, 30+((j+1) * y_offset_tft));
              gfx->print(menuepunkte_3[j]);
              if (j == pos and wert_aendern == true) {
                gfx->setTextColor(MARKER);
              }
              switch (j) {
                case 0: sprintf(ausgabe,"");
                        if (esp_now_wait_old != esp_now_wait and turntable_running == true) {
                          sprintf(ausgabe,"%s", esp_now_wait);
                          esp_now_wait_old = esp_now_wait;
                          change = true;
                        }
                        else if (esp_now_wait != "" and turntable_running == false) {
                          change = true;
                          esp_now_wait = "";
                          esp_now_wait_old = "...";
                        }
                        break;
                case 1: sprintf(ausgabe,"%i", jar_center_pos);
                        if (wert_old != jar_center_pos and wert_aendern == true and j == pos) {
                          change = true;
                          wert_old = jar_center_pos;
                        }
                        break;
                case 2: sprintf(ausgabe,"%i", speed_init);
                        if (wert_old != speed_init and wert_aendern == true and j == pos) {
                          change = true;
                          wert_old = speed_init;
                        }
                        break;
                case 3: sprintf(ausgabe,"%i", speed_run);
                        if (wert_old != speed_run and wert_aendern == true and j == pos) {
                          change = true;
                          wert_old = speed_run;
                        }
                        break;
              }
              if (change) {
                gfx->fillRect(251, 27+((j+1) * y_offset_tft)-19, 64, 27, BACKGROUND);
                if (use_turntable == false) {
                  gfx->fillRect(0, 65, 320, 135, BACKGROUND);
                }
                change = false;
              }
              int y = get_length(ausgabe);
              gfx->setCursor(320 - y * 14 - 5, 30+((j+1) * y_offset_tft));
              gfx->print(ausgabe);
            }
            else {
              gfx->setCursor(10, 30+(7 * y_offset_tft));
              gfx->print(menuepunkte_3[j]);
            }
            j++;
          }
        #endif
        if (digitalRead(SELECT_SW) == SELECT_PEGEL && menuitem_3 < MenuepunkteAnzahl_3 - 1  && wert_aendern == false) {
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
          #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
            wert_old = -1;
          #endif
          wert_aendern = true;
        }
        if (menuitem_3 == 1 and wert_aendern == true) {
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
        if ((digitalRead(SELECT_SW) == SELECT_PEGEL && menuitem_3 < MenuepunkteAnzahl_3 - 1  && wert_aendern == true)) {
          while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
            delay(1);
          }
          rotary_select = SW_MENU;
          initRotaries(SW_MENU, menuitem_3, 0, MenuepunkteAnzahl_3 - 1, 1);
          wert_aendern = false;
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
          #if DISPLAY_TYPE == 1 or DISPLAY_TYPE == 2 or DISPLAY_TYPE == 99
            u8g2.setCursor(111, (menuitem_3 * y_offset) + 5);
            u8g2.print("OK");
            u8g2.sendBuffer();
          #endif
          #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
            gfx->setCursor(283, 30+(7 * y_offset_tft));
            gfx->print("OK");
          #endif
          esp_now_msg_recived = false;
          jar_center_pos = 0;
          jar_center_pos_old = 0;
          delay(1000);
          initRotaries(SW_MENU, 1, 0, MenuepunkteAnzahl_2 -1, 1);
          i = 2;
          #if DISPLAY_TYPE == 1 or DISPLAY_TYPE == 2 or DISPLAY_TYPE == 99
            u8g2.clearBuffer();
            u8g2.sendBuffer();
          #endif
          #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
            gfx->fillRect(0, 32, 320, 208, BACKGROUND);
          #endif
        }
      }
      if (i == 4) {
        setRotariesValue(SW_MENU, 0);       //Workaround --> noch suchen warum der Rotary Wert auf 2 ist
        ts_open_running = false;
        ts_close_running = false;
        ts_angle_running = false;
        esp_now_msg_recived = false;
        memset(&myReceivedMessage, 0, sizeof(myReceivedMessage));
        strcpy(myMessageToBeSent.text, "ts_angle_min");
        espnow_send_data();
        turntable_millis = millis();
        while (millis() - turntable_millis < 3000 and esp_now_msg_recived == false) {  //brechce ab wenn in 3 sek keine rückmeldung kommt
          delay(10);
        }
        ts_angle_min = ts_angle_min_old = ts_angle_min_start = myReceivedMessage.value;
        esp_now_msg_recived = false;
        memset(&myReceivedMessage, 0, sizeof(myReceivedMessage));
        strcpy(myMessageToBeSent.text, "ts_angle_max");
        espnow_send_data();
        turntable_millis = millis();
        while (millis() - turntable_millis < 3000 and esp_now_msg_recived == false) {  //brechce ab wenn in 3 sek keine rückmeldung kommt
          delay(10);
        }
        ts_angle_max = ts_angle_max_old = ts_angle_max_start = myReceivedMessage.value;
        esp_now_msg_recived = false;
        memset(&myReceivedMessage, 0, sizeof(myReceivedMessage));
        strcpy(myMessageToBeSent.text, "ts_waittime");
        espnow_send_data();
        turntable_millis = millis();
        while (millis() - turntable_millis < 3000 and esp_now_msg_recived == false) {  //brechce ab wenn in 3 sek keine rückmeldung kommt
          delay(10);
        }
        ts_waittime = ts_waittime_old = myReceivedMessage.value;
        esp_now_msg_recived = false;
        memset(&myReceivedMessage, 0, sizeof(myReceivedMessage));
        strcpy(myMessageToBeSent.text, "ts_speed");
        espnow_send_data();
        turntable_millis = millis();
        while (millis() - turntable_millis < 3000 and esp_now_msg_recived == false) {  //brechce ab wenn in 3 sek keine rückmeldung kommt
          delay(10);
        }
        ts_speed = ts_speed_old = myReceivedMessage.value;
        //initRotaries(SW_MENU, menuitem_4, 0, MenuepunkteAnzahl_4 - 1, 1);
      }
      while (i == 4) {
        if (digitalRead(button_stop_pin)) {
          while (digitalRead(button_stop_pin) == HIGH) {
            delay(1);
          }
          wert_aendern = false;
          rotary_select = SW_MENU;
          initRotaries(SW_MENU, menuitem_2, 0, MenuepunkteAnzahl_2 - 1, 1);
          esp_now_msg_recived = false;
          memset(&myReceivedMessage, 0, sizeof(myReceivedMessage));
          memset(&myMessageToBeSent, 0, sizeof(myMessageToBeSent));
          i = 2;
          #if DISPLAY_TYPE == 1 or DISPLAY_TYPE == 2 or DISPLAY_TYPE == 99
            u8g2.clearBuffer();
            u8g2.sendBuffer();
          #endif
          #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
            wert_old = -1;
            gfx->fillRect(0, 32, 320, 208, BACKGROUND);
          #endif
          break;
        }
        if (digitalRead(switch_setup_pin) == LOW) {
          esp_now_msg_recived = false;
          rotary_select = SW_MENU;
          modus = -1;
          memset(&myReceivedMessage, 0, sizeof(myReceivedMessage));
          memset(&myMessageToBeSent, 0, sizeof(myMessageToBeSent));
          return;
        }
        if (ts_open_running == true or ts_close_running == true or ts_angle_running == true) { //zurücksetzen wenn zu schnell am rotary gedreht wurde
          if (ts_open_running == true or ts_close_running) {initRotaries(SW_MENU, menuitem_4, 0, MenuepunkteAnzahl_4 - 1, 1);}
          else if (ts_angle_running == true and menuitem_4 == 4) {initRotaries(SW_MENU, ts_angle_min, 0, ts_angle_max, 5);}
          else if (ts_angle_running == true and menuitem_4 == 5) {initRotaries(SW_MENU, ts_angle_max, ts_angle_min, 180, 5);}
          if (millis() - turntable_millis > 3000) {   //verlassen wenn der Befehl nicht in 3sek geschafft wurde
            //muss noch gemacht werden :-)
          }
          if (esp_now_msg_recived == true) {
            //Werte zurücksetzen
            if (ts_open_running == true or ts_close_running) {wert_aendern = false;}
            esp_now_msg_recived = false;
            ts_open_running = false;
            ts_close_running = false;
            ts_angle_running = false;
            turntable_millis = 0;
            memset(&myReceivedMessage, 0, sizeof(myReceivedMessage));
          }
        }
        if (wert_aendern == false) {
          menuitem_4 = getRotariesValue(SW_MENU);
          initRotaries(SW_MENU, menuitem_4, 0, MenuepunkteAnzahl_4 - 1, 1);
          #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
            pos = menuitem_4;
          #endif
          if (menuitem_4 == MenuepunkteAnzahl_4 - 1) {
            menuitem_4 = 7;
          }
        }
        else {
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
        #if DISPLAY_TYPE == 1 or DISPLAY_TYPE == 2 or DISPLAY_TYPE == 99
          u8g2.clearBuffer();
          u8g2.setFont(u8g2_font_courB08_tf);
          u8g2.setCursor(10, 1 * y_offset);
          u8g2.print(menuepunkte_4[0]);
          u8g2.setCursor(10, 2 * y_offset);
          u8g2.print(menuepunkte_4[1]);
          u8g2.setCursor(10, 3 * y_offset);
          u8g2.print(menuepunkte_4[2]);
          u8g2.setCursor(105, 3 * y_offset);
          sprintf(ausgabe,"%4i", ts_speed);
          u8g2.print(ausgabe);
          u8g2.setCursor(10, 4 * y_offset);
          u8g2.print(menuepunkte_4[3]);
          u8g2.setCursor(105, 4 * y_offset);
          sprintf(ausgabe,"%4i", ts_waittime);
          u8g2.print(ausgabe);
          u8g2.setCursor(10, 5 * y_offset);
          u8g2.print(menuepunkte_4[4]);
          u8g2.setCursor(105, 5 * y_offset);
          sprintf(ausgabe,"%4i", ts_angle_min);
          u8g2.print(ausgabe);
          u8g2.setCursor(10, 6 * y_offset);
          u8g2.print(menuepunkte_4[5]);
          u8g2.setCursor(105, 6 * y_offset);
          sprintf(ausgabe,"%4i", ts_angle_max);
          u8g2.print(ausgabe);
          u8g2.setCursor(10, (7 * y_offset) + 5);
          u8g2.print(menuepunkte_4[6]);
          if (wert_aendern == false && menuitem_4 < MenuepunkteAnzahl_4 - 1) {
            u8g2.setCursor(1, 10+((menuitem_4)*y_offset)); u8g2.print("*");
          }
          else if (wert_aendern == true && menuitem_4 < MenuepunkteAnzahl_4 - 1) {
            u8g2.setCursor(1, 8+((menuitem_4)*y_offset)); u8g2.print("-");
          }
          else if (wert_aendern == false && menuitem_4 == 7) {
            u8g2.setCursor(1, 10+((menuitem_4 - 1)*y_offset) + 5); u8g2.print("*");
          }
          u8g2.sendBuffer();
        #endif
        #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
          int j = 0;
          while(j < MenuepunkteAnzahl_4) {
            gfx->setTextColor(TEXT);
            if (j == pos and wert_aendern == false) {
              gfx->setTextColor(MARKER);
            }
            if (j < MenuepunkteAnzahl_4 - 1) {
              gfx->setCursor(5, 30+((j+1) * y_offset_tft));
              gfx->print(menuepunkte_4[j]);
              if (j == pos and wert_aendern == true) {
                gfx->setTextColor(MARKER);
              }
              switch (j) {
                case 0: sprintf(ausgabe,"");
                        break;
                case 1: sprintf(ausgabe,"");
                        break;
                case 2: sprintf(ausgabe,"%i", ts_speed);
                        if (wert_old != ts_speed and wert_aendern == true and j == pos) {
                          change = true;
                          wert_old = ts_speed;
                        }
                        break;
                case 3: sprintf(ausgabe,"%i", ts_waittime);
                        if (wert_old != ts_waittime and wert_aendern == true and j == pos) {
                          change = true;
                          wert_old = ts_waittime;
                        }
                        break;
                case 4: sprintf(ausgabe,"%i", ts_angle_min);
                        if (wert_old != ts_angle_min and wert_aendern == true and j == pos) {
                          change = true;
                          wert_old = ts_angle_min;
                        }
                        break;
                case 5: sprintf(ausgabe,"%i", ts_angle_max);
                        if (wert_old != ts_angle_max and wert_aendern == true and j == pos) {
                          change = true;
                          wert_old = ts_angle_max;
                        }
                        break;
              }
              if (change) {
                gfx->fillRect(251, 27+((j+1) * y_offset_tft)-19, 64, 27, BACKGROUND);
                change = false;
              }
              int y = get_length(ausgabe);
              gfx->setCursor(320 - y * 14 - 5, 30+((j+1) * y_offset_tft));
              gfx->print(ausgabe);
            }
            else {
              gfx->setCursor(10, 30+(7 * y_offset_tft));
              gfx->print(menuepunkte_4[j]);
            }
            j++;
          }
        #endif
        if (digitalRead(SELECT_SW) == SELECT_PEGEL && menuitem_4 < MenuepunkteAnzahl_4 - 1  && wert_aendern == false) {
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
                    strcpy(myMessageToBeSent.text, "close_drop_prodection");
                    espnow_send_data();
                    break;
            case 2: rotary_select = SW_MENU;
                    initRotaries(SW_MENU, ts_speed, 0, 500, 10);
                    break;
            case 3: rotary_select = SW_MENU;
                    initRotaries(SW_MENU, ts_waittime, 0, 120, 1);
                    break;
            case 4: rotary_select = SW_MENU;
                    initRotaries(SW_MENU, ts_angle_min, 0, ts_angle_max, 5);
                    break;
            case 5: rotary_select = SW_MENU;
                    initRotaries(SW_MENU, ts_angle_max, ts_angle_min, 180, 5);
                    break;
          }
          #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
            wert_old = -1;
          #endif
          wert_aendern = true;
        }
        if ((menuitem_4 == 4 or menuitem_4 == 5) and wert_aendern == true) {
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
        if ((digitalRead(SELECT_SW) == SELECT_PEGEL && menuitem_4 < MenuepunkteAnzahl_4 - 1  && wert_aendern == true)) {
          while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
            delay(1);
          }
          rotary_select = SW_MENU;
          initRotaries(SW_MENU, menuitem_4, 0, MenuepunkteAnzahl_4 - 1, 1);
          wert_aendern = false;
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
          #if DISPLAY_TYPE == 1 or DISPLAY_TYPE == 2 or DISPLAY_TYPE == 99
            u8g2.setCursor(111, (menuitem_4 * y_offset) + 5);
            u8g2.print("OK");
            u8g2.sendBuffer();
          #endif
          #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
            gfx->setCursor(283, 30+(7 * y_offset_tft));
            gfx->print("OK");
          #endif
          esp_now_msg_recived = false;
          ts_angle_running = false;
          ts_close_running = false;
          ts_open_running = false;
          delay(1000);
          initRotaries(SW_MENU, 2, 0, MenuepunkteAnzahl_2 -1, 1);
          i = 2;
          #if DISPLAY_TYPE == 1 or DISPLAY_TYPE == 2 or DISPLAY_TYPE == 99
            u8g2.clearBuffer();
            u8g2.sendBuffer();
          #endif
          #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
            gfx->fillRect(0, 32, 320, 208, BACKGROUND);
          #endif
        }
      }
    }
    modus = -1;
    rotary_select = SW_MENU;
    memset(&myReceivedMessage, 0, sizeof(myReceivedMessage));
    memset(&myMessageToBeSent, 0, sizeof(myMessageToBeSent));
  #endif
}

void processSetup(void) {
  int x_pos;
  int MenuepunkteAnzahl = 10;
  int menuitem_old = -1;
  if (ina219_installed) {MenuepunkteAnzahl++;}
  if (DREHTELLER) {MenuepunkteAnzahl++;}
  int posmenu[MenuepunkteAnzahl];
  const char *menuepunkte[MenuepunkteAnzahl] = {LANGUAGE1[lingo], TAREVALUES[lingo], CALIBRATION[lingo], FILL_QUANTITY[lingo], AUTOMATIC[lingo], SERVOSETTINGS[lingo], PARAMETER[lingo], COUNTER[lingo], COUNTER_TRIP[lingo], CLEAR_PREFS[lingo]};
  if (ina219_installed) {
    menuepunkte[MenuepunkteAnzahl - 1] = menuepunkte[MenuepunkteAnzahl -2];    //Clear Pref eins nach hinten schieben
    menuepunkte[MenuepunkteAnzahl - 2] = INA219_SETUP[lingo];
  }
  if (DREHTELLER) {
    menuepunkte[MenuepunkteAnzahl - 1] = menuepunkte[MenuepunkteAnzahl -2];    //Clear Pref eins nach hinten schieben
    menuepunkte[MenuepunkteAnzahl - 2] = TURNTABLE[lingo];
  }
  modus = MODE_SETUP;
  winkel = winkel_min;          // Hahn schliessen
  servo_aktiv = 0;              // Servo-Betrieb aus
  SERVO_WRITE(winkel);
  rotary_select = SW_MENU;
  initRotaries(SW_MENU, lastpos, -1, MenuepunkteAnzahl, -1);
  while (modus == MODE_SETUP and (digitalRead(switch_setup_pin)) == HIGH) {
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
    #if DISPLAY_TYPE == 1 or DISPLAY_TYPE == 2 or DISPLAY_TYPE == 99
      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_courB08_tf);
      x_pos = CenterPosX(menuepunkte[posmenu[MenuepunkteAnzahl-1]], 6, 128); //ein Zeichen ist etwa 6 Pixel breit
      u8g2.setCursor(x_pos,12);  
      u8g2.print(menuepunkte[posmenu[MenuepunkteAnzahl-1]]);
      u8g2.drawLine(0, 18, 128, 18);
      u8g2.setFont(u8g2_font_courB12_tf);
      x_pos = CenterPosX(menuepunkte[posmenu[0]], 10, 128); //ein Zeichen ist 10 Pixel breit
      u8g2.setCursor(x_pos, 38);   
      u8g2.print(menuepunkte[posmenu[0]]);
      u8g2.drawLine(0, 47, 128, 47);
      u8g2.setFont(u8g2_font_courB08_tf);
      x_pos = CenterPosX(menuepunkte[posmenu[1]], 6, 128); //ein Zeichen ist etwa 6 Pixel breit
      u8g2.setCursor(x_pos,62);   
      u8g2.print(menuepunkte[posmenu[1]]);
      u8g2.sendBuffer();
    #endif
    #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
      if (menuitem != menuitem_old) {
        gfx->fillScreen(BACKGROUND);
        gfx->drawLine(0, 100, 320, 100, TEXT);
        gfx->drawLine(0, 139, 320, 139, TEXT);
        gfx->setTextColor(TEXT);
        gfx->setFont(Punk_Mono_Bold_320_200);
        x_pos = CenterPosX(menuepunkte[posmenu[0]], 18, 320);
        gfx->setCursor(x_pos, 130);
        gfx->println(menuepunkte[posmenu[0]]);
        gfx->setTextColor(MENU_POS1);
        gfx->setFont(Punk_Mono_Bold_240_150);
        x_pos = CenterPosX(menuepunkte[posmenu[MenuepunkteAnzahl-1]], 14, 320);
        gfx->setCursor(x_pos, 86);
        gfx->println(menuepunkte[posmenu[MenuepunkteAnzahl-1]]);
        x_pos = CenterPosX(menuepunkte[posmenu[1]], 14, 320);
        gfx->setCursor(x_pos, 169);
        gfx->println(menuepunkte[posmenu[1]]);
        gfx->setTextColor(MENU_POS2);
        gfx->setFont(Punk_Mono_Bold_200_125);                   //9 x 11 Pixel
        x_pos = CenterPosX(menuepunkte[posmenu[MenuepunkteAnzahl-2]], 12, 320);
        gfx->setCursor(x_pos, 60);
        gfx->println(menuepunkte[posmenu[MenuepunkteAnzahl-2]]);
        x_pos = CenterPosX(menuepunkte[posmenu[2]], 12, 320);
        gfx->setCursor(x_pos, 192);
        gfx->println(menuepunkte[posmenu[2]]);
        gfx->setTextColor(MENU_POS3);
        gfx->setFont(Punk_Mono_Bold_160_100);                   //7 x 9 Pixel
        x_pos = CenterPosX(menuepunkte[posmenu[MenuepunkteAnzahl-3]], 9, 320);
        gfx->setCursor(x_pos, 37);
        gfx->println(menuepunkte[posmenu[MenuepunkteAnzahl-3]]);    
        x_pos = CenterPosX(menuepunkte[posmenu[3]], 9, 320);
        gfx->setCursor(x_pos, 213);
        gfx->println(menuepunkte[posmenu[3]]);
        gfx->setTextColor(MENU_POS4);
        gfx->setFont(Punk_Mono_Bold_120_075);                   //6 x 7 Pixel
        x_pos = CenterPosX(menuepunkte[posmenu[MenuepunkteAnzahl-4]], 7, 320);
        gfx->setCursor(x_pos, 16);
        gfx->println(menuepunkte[posmenu[MenuepunkteAnzahl-4]]);
        x_pos = CenterPosX(menuepunkte[posmenu[4]], 7, 320);
        gfx->setCursor(x_pos, 232);
        gfx->println(menuepunkte[posmenu[4]]);
        menuitem_old = menuitem;
      }
    #endif
    lastpos = menuitem;
    if (digitalRead(SELECT_SW) == SELECT_PEGEL) {
      while(digitalRead(SELECT_SW) == SELECT_PEGEL) {
        delay(1);
      }
    #ifdef isDebug
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
      setPreferences();
      if (menuepunkte[menuitem] == CLEAR_PREFS[lingo])       setupClearPrefs();        // EEPROM löschen
      initRotaries(SW_MENU,lastpos, 0,255, 1);                                         // Menu-Parameter könnten verstellt worden sein
    }
    #if OTA == 1
      if (digitalRead(button_start_pin) == HIGH) {
        while (digitalRead(button_start_pin) == HIGH) {delay(100);}
        menuitem_old = -1;
        OTASetup();
      }
    #endif
  }
}
void processAutomatik(void) {
  int zielgewicht;                 // Glas + Korrektur
  static int autokorrektur_gr = 0; 
  int erzwinge_servo_aktiv = 0;
  boolean voll = false;
  static float gewicht_alt2;       // Gewicht des vorhergehenden Durchlaufs A.P.
  static int gewicht_vorher;       // Gewicht des vorher gefüllten Glases
  static int sammler_num = 5;      // Anzahl identischer Messungen für Nachtropfen
  int loop1 = 3;                   // anzahl alle wieviel Durchgänge die auto Servo einstellung gemacht werden soll A.P.
  int n;
  int y_offset = 0;
  #if DREHTELLER == 1
    unsigned long time;
  #endif
  #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
      int x_pos;
  #endif
  if (modus != MODE_AUTOMATIK) {
    winkel = winkel_min;          // Hahn schliessen
    servo_aktiv = 0;              // Servo-Betrieb aus
    SERVO_WRITE(winkel);
    auto_aktiv = 0;               // automatische Füllung starten
    tare_glas = 0;
    rotary_select = SW_WINKEL;    // Einstellung für Winkel über Rotary
    offset_winkel = 0;            // Offset vom Winkel wird auf 0 gestellt
    initRotaries(SW_MENU, fmenge_index, 0, 4, 1);
    setRotariesValue(SW_FLUSS, intGewicht);  //warum auch immer :-)
    gewicht_vorher = glaeser[fmenge_index].Gewicht + korrektur;
    #if DREHTELLER == 1
      turntable_moving = false;
      if (use_turntable == 1) {
        turntable_ok = false;
        strcpy(myMessageToBeSent.text, "close_drop_prodection");
        espnow_send_data();
      }
      else {
        strcpy(myMessageToBeSent.text, "open_drop_prodection");
        espnow_send_data();
      }
      if (use_turntable == 1) {
        #ifdef ESP_NOW_DEBUG
          Serial.println("Use turntable");
        #endif
        //Clear Display
        #if DISPLAY_TYPE == 1 or DISPLAY_TYPE == 2 or DISPLAY_TYPE == 99
          u8g2.clearBuffer();
          u8g2.sendBuffer();
        #endif
        #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
          gfx->fillScreen(BACKGROUND);
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
          #if DISPLAY_TYPE == 1 or DISPLAY_TYPE == 2 or DISPLAY_TYPE == 99
            u8g2.clearBuffer();
            u8g2.setFont(u8g2_font_courB08_tf);
            x_pos = CenterPosX(CONNECTION[lingo], 6, 128);                 //ein Zeichen ist etwa 6 Pixel breit
            u8g2.setCursor(x_pos,22);  
            u8g2.print(CONNECTION[lingo]);
            x_pos = CenterPosX(FAILED[lingo], 6, 128);                     //ein Zeichen ist etwa 6 Pixel breit
            u8g2.setCursor(x_pos,42);  
            u8g2.print(FAILED[lingo]);
            u8g2.sendBuffer();
          #endif
          #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
            gfx->setTextColor(RED);
            gfx->setFont(Punk_Mono_Bold_320_200);
            x_pos = CenterPosX(CONNECTION[lingo], 18, 320);
            gfx->setCursor(x_pos, 120);
            gfx->println(CONNECTION[lingo]);
            x_pos = CenterPosX(FAILED[lingo], 18, 320);
            gfx->setCursor(x_pos, 150);
            gfx->println(FAILED[lingo]);
            gfx->setTextColor(TEXT);
          #endif
          while (digitalRead(switch_betrieb_pin) == HIGH) {
            if (digitalRead(switch_betrieb_pin) == HIGH) {delay(10);}
          }
        }
        if (turntable_init == false) {
          #if DISPLAY_TYPE == 1 or DISPLAY_TYPE == 2 or DISPLAY_TYPE == 99
            u8g2.clearBuffer();
            u8g2.setFont(u8g2_font_courB08_tf);
            x_pos = CenterPosX(INIT_TURNTABLE[lingo], 6, 128);
            u8g2.setCursor(x_pos,22);  
            u8g2.print(INIT_TURNTABLE[lingo]);
            x_pos = CenterPosX(NOKAY[lingo], 6, 128);
            u8g2.setCursor(x_pos,42);  
            u8g2.print(NOKAY[lingo]);
            u8g2.sendBuffer();
          #endif
          #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
            gfx->setTextColor(RED);
            gfx->setFont(Punk_Mono_Bold_320_200);
            x_pos = CenterPosX(INIT_TURNTABLE[lingo], 18, 320);
            gfx->setCursor(x_pos, 120);
            gfx->println(INIT_TURNTABLE[lingo]);
            x_pos = CenterPosX(NOKAY[lingo], 18, 320);
            gfx->setCursor(x_pos, 150);
            gfx->println(NOKAY[lingo]);
            gfx->setTextColor(TEXT);
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
    #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
      if (current_servo > 0) {
        no_ina = true;
      }
      else {
        no_ina = false;
      }
      glas_alt = -1;
      current_mA_alt = -1;
      korr_alt = -99999;
      autokorr_gr_alt = -99999;
      winkel_min_alt = -1;
      pos_alt = -1;
      winkel_ist_alt = -1;
      servo_aktiv_alt = -1;
      auto_aktiv_alt = -1;
      gewicht_alt = -9999999;
      intGewicht_alt = -1;
      gfx->fillScreen(BACKGROUND);
      gfx->setTextColor(TEXT);
      gfx->drawLine(0, 167, 320, 167, TEXT);
      gfx->drawLine(160, 167, 160, 240, TEXT);
      gfx->setCursor(2, 185);
      gfx->setFont(Punk_Mono_Thin_160_100);
      gfx->print(INA219[lingo]);
      gfx->setFont(Checkbox);
      gfx->setCursor(140, 182);
      if (ina219_installed == 0){
        gfx->setTextColor(RED);
        gfx->print("B");
      }
      else {
        gfx->setTextColor(GREEN);
        gfx->print("A");
      }
      gfx->setTextColor(TEXT);
      gfx->setCursor(2, 202);
      gfx->setFont(Punk_Mono_Thin_160_100);
      gfx->print(AUTOSTART[lingo]);
      gfx->setFont(Checkbox);
      gfx->setCursor(140, 199);
      if (autostart == 0){
        gfx->setTextColor(RED);
        gfx->print("B");
      }
      else {
        gfx->setTextColor(GREEN);
        gfx->print("A");
      }
      gfx->setTextColor(TEXT);
      gfx->setFont(Punk_Mono_Thin_160_100);
      gfx->setCursor(2, 219);
      gfx->print(AUTOCORRECTION[lingo]);
      gfx->setFont(Checkbox);
      gfx->setCursor(140, 216);
      if (autokorrektur == 0){
        gfx->setTextColor(RED);
        gfx->print("B");
      }
      else {
        gfx->setTextColor(GREEN);
        gfx->print("A");
      }
      gfx->setTextColor(TEXT);
      gfx->setFont(Punk_Mono_Thin_160_100);
      gfx->setCursor(2, 236);
      gfx->print(FLOW_G_OVER_TIME[lingo]);
      gfx->setFont(Checkbox);
      gfx->setCursor(140, 233);
      if (intGewicht <= 0){
        gfx->setTextColor(RED);
        gfx->print("B");
      }
      else {
        gfx->setTextColor(GREEN);
        gfx->print("A");
      }
      gfx->setTextColor(TEXT);
      gfx->setFont(Punk_Mono_Thin_160_100);
      gfx->setCursor(170, 185);
      gfx->printf("%s:", CURRENT);
      gfx->setCursor(170, 202);
      gfx->printf("%s:", CORRECTION);
      gfx->setCursor(170, 219);
      gfx->printf("%s:", AUTOCORR);
      gfx->setCursor(170, 236);
      gfx->print(FLOW[lingo]);
      //Display Kopf
      gfx->setCursor(2, 17);
      gfx->printf("%s:", MIN);
      gfx->setCursor(119, 17);
      gfx->printf("%s:", MAX_2);
      gfx->setCursor(236, 17);
      gfx->printf("%s:", ACT);
      gfx->drawLine(0, 20, 320, 20, TEXT);
    #endif
    modus = MODE_AUTOMATIK;
  }
  pos = getRotariesValue(SW_WINKEL);
  // nur bis winkel_fein regeln, oder über initRotaries lösen?
  if (pos < winkel_fein * 100 / winkel_max) {                      
    pos = winkel_fein * 100 / winkel_max;
    setRotariesValue(SW_WINKEL, pos);
  }
  korrektur    = getRotariesValue(SW_KORREKTUR);
  intGewicht   = getRotariesValue(SW_FLUSS);
  fmenge_index = getRotariesValue(SW_MENU);
  fmenge       = glaeser[fmenge_index].Gewicht;
  tare         = glaeser[fmenge_index].Tare;
  if (tare <= 0) { 
    auto_aktiv = 0;
  }
  // wir starten nur, wenn das Tara für die Füllmenge gesetzt ist!
  // Ein erneuter Druck auf Start erzwingt die Aktivierung des Servo
  if (digitalRead(button_start_pin) == HIGH && tare > 0) {
    while(digitalRead(button_start_pin) == HIGH) {
       delay(1);
    }
    if (auto_aktiv == 1) {
      erzwinge_servo_aktiv = 1;
      #ifdef isDebug
        Serial.println("erzwinge Servo aktiv");      
      #endif
    }
    auto_aktiv    = 1;                            // automatisches Füllen aktivieren
    rotary_select = SW_WINKEL;                    // falls während der Parameter-Änderung auf Start gedrückt wurde    
    setPreferences();                             // falls Parameter über den Rotary verändert wurden
    #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
      glas_alt = -1;                              // Glas Typ Farbe zurücksetzen fals markiert ist
      korr_alt = -99999;                          // Korrektur Farbe zurücksetzen fals markiert ist
    #endif
  }
  if (digitalRead(button_stop_pin) == HIGH) {
    winkel      = winkel_min + offset_winkel;
    servo_aktiv = 0;
    auto_aktiv  = 0;
    tare_glas   = 0;
    #if DREHTELLER == 1
      turntable_moving = false;
      if (use_turntable == 1) {
        esp_now_msg_recived = false;
        strcpy(myMessageToBeSent.text, "stop");
        espnow_send_data();
        time = millis();
        while (!esp_now_msg_recived and millis() - time <= 1000 and strcmp(myReceivedMessage.text, "") == 0) {
          delay(10);
        }
        modus = -1;
      }
    #endif
  }
  // Fehlerkorrektur der Waage, falls Gewicht zu sehr schwankt 
  #ifdef FEHLERKORREKTUR_WAAGE
    int Vergleichsgewicht = (int(SCALE_GETUNITS(SCALE_READS))) - tare;
    for (byte j = 0 ; j < 3; j++) { // Anzahl der Wiederholungen, wenn Abweichung zu hoch
      gewicht = int(SCALE_GETUNITS(SCALE_READS)) - tare;
      if (abs(gewicht - Vergleichsgewicht) < 50)  // Abweichung für Fehlererkennung
        break; 
      delay(100);
    }
  #else
    gewicht = int(SCALE_GETUNITS(SCALE_READS)) -   tare;
  #endif 
  // Glas entfernt -> Servo schliessen
  if (gewicht < -20) {
    winkel      = winkel_min + offset_winkel;
    servo_aktiv = 0;
    tare_glas   = 0;
    if (autostart != 1) {  // Autostart nicht aktiv
      auto_aktiv  = 0;
    }
  }
  // Automatik ein, leeres Glas aufgesetzt, Servo aus -> Glas füllen
  if (auto_aktiv == 1 && abs(gewicht) <= glastoleranz && servo_aktiv == 0 and turntable_ok == true) {
    rotary_select = SW_WINKEL;     // falls während der Parameter-Änderung ein Glas aufgesetzt wird
    #if DISPLAY_TYPE == 1 or DISPLAY_TYPE == 2 or DISPLAY_TYPE == 99 
      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_courB24_tf);
      u8g2.setCursor(15, 43);
      u8g2.print(START[lingo]);
      u8g2.sendBuffer();
    #endif
    #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
      // START: nicht schön aber es funktioniert (vieleicht mit Subprogrammen arbeiten damit es besser wird)
      //PLAY ICON setzen
      gfx->fillRect(12, 44, 38, 38, BACKGROUND);
      gfx->setFont(Icons_Start_Stop);
      gfx->setCursor(5, 88);
      gfx->print("M");
      gfx->setCursor(5, 88);
      gfx->setTextColor(GREEN);
      gfx->print("A");
      //Korrektur Wert auf Textfarbe setzen
      gfx->setTextColor(TEXT);
      gfx->setFont(Punk_Mono_Thin_160_100);
      gfx->setCursor(265, 202);
      sprintf(ausgabe, "%5ig", korrektur + autokorrektur_gr);
      gfx->print(ausgabe);
      //Flow Wert auf Textfarbe setzen
      gfx->setTextColor(TEXT);
      gfx->setFont(Punk_Mono_Thin_160_100);
      gfx->setCursor(265, 236);
      sprintf(ausgabe, "%5ig", intGewicht);
      gfx->print(ausgabe);
      //Glasgewicht auf Textfarbe Setzen
      gfx->setTextColor(TEXT);
      gfx->fillRect(0, 136, 320, 27, BACKGROUND);
      gfx->setFont(Punk_Mono_Thin_240_150);
      gfx->setCursor(2, 156);
      sprintf(ausgabe, "%dg ", (glaeser[fmenge_index].Gewicht));
      gfx->print(ausgabe);
      gfx->setFont(Punk_Mono_Thin_120_075);
      gfx->setCursor(2 + 14*StringLenght(ausgabe), 148);
      sprintf(ausgabe, "+%dg ", (kulanz_gr));
      gfx->print(ausgabe);
      // END:   nicht schön aber es funktioniert
      gfx->fillRect(80, 24, 240, 80, BACKGROUND);
      gfx->setFont(Punk_Mono_Bold_600_375);
      gfx->setCursor(100, 85);
      gfx->print(START[lingo]);
    #endif
    // A.P. kurz warten und über 5 Messungen prüfen ob das Gewicht nicht nur eine zufällige Schwankung war 
    int gewicht_Mittel = 0;
    for (int i = 0; i < 5; i++){
      gewicht = int(SCALE_GETUNITS(SCALE_READS)) - tare;
      gewicht_Mittel += gewicht;
      delay(300);
    }
    gewicht = gewicht_Mittel / 5;                 //A.P.  Mittleres Gewicht über die 1,5s
    if (intGewicht > 0) {
      winkel = Winkelmerker ;                       //A.P. für Folgeglas optimalen Winkel vom Vorglas einstellen.
    }
    if (abs(gewicht) <= glastoleranz) {
      if (intGewicht <= 0) {
        gewicht = int(SCALE_GETUNITS(SCALE_READS)) - tare;  // A.P.
      }
      tare_glas = gewicht;
      #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
        gfx->fillRect(109, 136, 211, 27, BACKGROUND);
        gfx->setFont(Punk_Mono_Thin_240_150);
        sprintf(ausgabe, "%s:%dg ", TARE_JAR, tare_glas);
        int i = StringLenght(ausgabe);
        gfx->setCursor(320 - 14 * i, 156);
        gfx->print(ausgabe);
      #endif
      #ifdef isDebug 
        Serial.print(" gewicht: ");           Serial.print(gewicht);
        Serial.print(" gewicht_vorher: ");    Serial.print(gewicht_vorher);
        Serial.print(" zielgewicht: ");       Serial.print(fmenge + korrektur + tare_glas + autokorrektur_gr);
        Serial.print(" kulanz: ");            Serial.print(kulanz_gr);
        Serial.print(" Autokorrektur: ");     Serial.println(autokorrektur_gr);
      #endif      
      servo_aktiv = 1;
      sammler_num = 0;
      voll = false;
      gezaehlt = false;
      gewicht_alt = -9999999;               //Damit die Gewichtsanzeige aktiviert wird
      buzzer(BUZZER_SHORT);
    }
  }
  zielgewicht = fmenge + korrektur + tare_glas + autokorrektur_gr;
  // Anpassung des Autokorrektur-Werts
  if (autokorrektur == 1) {                                                       
    if ( auto_aktiv == 1 && servo_aktiv == 0 && winkel == winkel_min + offset_winkel && gewicht >= zielgewicht && sammler_num <= 5) {     
      voll = true;                       
      if (gewicht == gewicht_vorher && sammler_num < 5) { // wir wollen 5x das identische Gewicht sehen  
        sammler_num++;
      } 
      else if (gewicht != gewicht_vorher) {               // sonst gewichtsänderung nachführen
        gewicht_vorher = gewicht;
        sammler_num = 0;
      } 
      else if (sammler_num == 5) {                        // gewicht ist 5x identisch, autokorrektur bestimmen
        autokorrektur_gr = (fmenge + kulanz_gr + tare_glas) - (gewicht - autokorrektur_gr);
        if (korrektur + autokorrektur_gr > kulanz_gr) {   // Autokorrektur darf nicht überkorrigieren, max Füllmenge plus Kulanz
          autokorrektur_gr = kulanz_gr - korrektur;
          #ifdef isDebug
            Serial.print("Autokorrektur begrenzt auf ");
            Serial.println(autokorrektur_gr);
          #endif
        }
        buzzer(BUZZER_SUCCESS);
        sammler_num++;                                      // Korrekturwert für diesen Durchlauf erreicht
      }
      if (voll == true && gezaehlt == false) {
        glaeser[fmenge_index].TripCount++;
        glaeser[fmenge_index].Count++;
        gezaehlt = true;
      }
      #ifdef isDebug
        Serial.print("Nachtropfen:");
        Serial.print(" gewicht: ");        Serial.print(gewicht);
        Serial.print(" gewicht_vorher: "); Serial.print(gewicht_vorher);
        Serial.print(" sammler_num: ");    Serial.print(sammler_num);
        Serial.print(" Korrektur: ");      Serial.println(autokorrektur_gr);
        Serial.print(" Zähler Trip: ");    Serial.print(glaeser[fmenge_index].TripCount); //Kud
        Serial.print(" Zähler: ");         Serial.println(glaeser[fmenge_index].Count); //Kud
      #endif
    }
  }
  // Glas ist teilweise gefüllt. Start wird über Start-Taster erzwungen
  if (auto_aktiv == 1 && gewicht > 5 && erzwinge_servo_aktiv == 1 and turntable_ok == true) {
    servo_aktiv = 1;
    voll = false;
    gezaehlt = false;
    buzzer(BUZZER_SHORT);
  }
  if (servo_aktiv == 1 && intGewicht <= 0) { 
    winkel = (winkel_max * pos / 100);  // A.P.wird deaktiviert wenn intGewicht ist gleich 0 (automatische Durchflussgeschwindigkeit ist 0)
  }
  if (intGewicht > 0) {
    if (servo_aktiv == 1 && (zielgewicht - gewicht <= fein_dosier_gewicht) && ((winkel_max*pos / 100) * (zielgewicht-gewicht) / fein_dosier_gewicht <= winkel)) {
      winkel = ((winkel_max*pos / 100) * ((zielgewicht-gewicht) / fein_dosier_gewicht));    // AP nur wenn der hier berechnete Winkel kleiner dem aktuellen Winkel ist wird das Ventil zugefahren (ansonsten geht es bei kleineren Öffnungsgraden auf
    }
  }
  else {
    if (servo_aktiv == 1 && (zielgewicht - gewicht <= fein_dosier_gewicht)) {
      winkel = ((winkel_max*pos / 100) * ((zielgewicht-gewicht) / fein_dosier_gewicht));
    }
  }
  if (servo_aktiv == 1 && winkel <= winkel_fein) { 
    winkel = winkel_fein;
  }
  if (intGewicht > 0) {
    // A.P.  Es wird ein gleitender Mittelwert über 3 Messungen durchgeführt.
    float RunningAverageGewicht;
    int RawGewicht = gewicht-gewicht_alt2;
    average.add(RawGewicht);
    RunningAverageGewicht = (average.getAvg());
    //Optimierung der >Durchflussgeschwindigkeit  A.P.
    if (loop1_c < loop1){ loop1_c = loop1_c + 1;}
    else if (servo_aktiv == 1 && ((RunningAverageGewicht + intGewichtD) > intGewicht) && (intGewicht > 0) && ((zielgewicht - gewicht) > fein_dosier_gewicht)) {
      winkel = winkel -2;
      Winkelmerker  = winkel; // A.P. Merkt sich den letzten optimalen Winkel des Servos.
      loop1_c =0;
    }    //A.P.  Automatische Geschwindigkeit des Abfüllvorgangs
    else if (servo_aktiv == 1 && ((RunningAverageGewicht - intGewichtD) < intGewicht) && (intGewicht > 0) && ((zielgewicht - gewicht) > fein_dosier_gewicht)) {
      winkel = winkel +2;
      if (winkel >= (winkel_max*pos/100)) {
        winkel = (winkel_max*pos/100);
        Winkelmerker  = winkel; // A.P. Merkt sich den letzten optimalen Winkel des Servos.
      }
      loop1_c =0;
    }
    else {loop1_c =0;}
    gewicht_alt2 = gewicht;
  }
  // Glas ist voll
  if (servo_aktiv == 1 && gewicht >= zielgewicht) {
    winkel      = winkel_min + offset_winkel;
    servo_aktiv = 0;
    if (gezaehlt == false) {
      glaeser[fmenge_index].TripCount++;
      glaeser[fmenge_index].Count++;
      gezaehlt = true;
    }
    if (autostart != 1)       // autostart ist nicht aktiv, kein weiterer Start
      auto_aktiv = 0;
    if (autokorrektur == 1)   // autokorrektur, gewicht merken
      gewicht_vorher = gewicht;
    buzzer(BUZZER_SHORT);
  }   
  // Glas entfernt -> Servo schliessen zusätzliche Abfrage
  if (winkel > (winkel_min + offset_winkel)) {
    gewicht = int(SCALE_GETUNITS(SCALE_READS)) - tare; 
    if (gewicht < -20) { 
      delay(200);  // A.P.  Um einen Reset des ESP zu verhindern wenn er kurz nach öffnen geschlossen wird.
      winkel      = winkel_min + offset_winkel;
    }
  }
  SERVO_WRITE(winkel);
  #ifdef isDebug
    #if isDebug >= 4
      Serial.print("Automatik:");  
      Serial.print(" Gewicht: ");        Serial.print(gewicht);
      Serial.print(" Winkel: ");         Serial.print(winkel);
      Serial.print(" Dauer ");           Serial.print(millis() - scaletime);
      Serial.print(" Füllmenge: ");      Serial.print(fmenge);
      Serial.print(" Korrektur: ");      Serial.print(korrektur);
      Serial.print(" Tara_glas:");       Serial.print(tare_glas);
      Serial.print(" Autokorrektur: ");  Serial.print(autokorrektur_gr);
      Serial.print(" Zielgewicht ");     Serial.print(zielgewicht);
      Serial.print(" Erzwinge Servo: "); Serial.print(erzwinge_servo_aktiv);
      Serial.print(" servo_aktiv ");     Serial.print(servo_aktiv);
      Serial.print(" auto_aktiv ");      Serial.println(auto_aktiv);
    #endif 
  #endif
  if (ina219_installed and (current_servo > 0 or show_current == 1)) {
    y_offset = 4;
  }
  #if DISPLAY_TYPE == 1 or DISPLAY_TYPE == 2 or DISPLAY_TYPE == 99
    u8g2.clearBuffer();
    // wenn kein Tara für unser Glas definiert ist, wird kein Gewicht sondern eine Warnung ausgegeben
    if (tare > 0) {
      // kein Glas aufgestellt 
      if (gewicht < -20) {
        u8g2.setFont(u8g2_font_courB10_tf);
        u8g2.setCursor(37, 31 + y_offset); u8g2.print(PLEASE_PUT[lingo]);
        u8g2.setCursor(37, 43 + y_offset); u8g2.print(UP_THE_JAR[lingo]);
      } 
      else {
        if(autostart == 1 && auto_aktiv == 1 && servo_aktiv == 0 && gewicht >= -5 && gewicht - tare_glas < fmenge) {
          u8g2.setFont(u8g2_font_unifont_t_symbols);
          u8g2.drawGlyph(14, 38 + y_offset, 0x2612);
          u8g2.setFont(u8g2_font_courB24_tf);
        }
        u8g2.setCursor(10, 42 + y_offset);
        u8g2.setFont(u8g2_font_courB24_tf);
        sprintf(ausgabe,"%5dg", gewicht - tare_glas);
        u8g2.print(ausgabe);
      }
    } 
    else {
      u8g2.setFont(u8g2_font_courB14_tf);
      int y = get_length(NO_TARE[lingo]);
      u8g2.setCursor(128 - y * 10 -5, 38 + y_offset);
      sprintf(ausgabe,"%6s", NO_TARE);                    //kein Tara
      u8g2.print(ausgabe);
    }
    // Play/Pause Icon, ob die Automatik aktiv ist
    u8g2.setFont(u8g2_font_open_iconic_play_2x_t);
    u8g2.drawGlyph(0, 40 + y_offset, (auto_aktiv==1)?0x45:0x44 );
    u8g2.setFont(u8g2_font_courB08_tf);
    // Zeile oben, Öffnungswinkel absolut und Prozent, Anzeige Autostart
    u8g2.setCursor(1, 9);
    sprintf(ausgabe,"%s=%-3d", A_UPPER_CASE, winkel);
    u8g2.print(ausgabe);
    n = StringLenght(ausgabe);
    u8g2.setCursor(n * 6 + 1, 9);
    u8g2.print("°");
    sprintf(ausgabe,"%3d%%", pos);
    u8g2.setCursor(104, 9);
    u8g2.print(ausgabe);
    sprintf(ausgabe,"%2s", (autostart==1)?AS[lingo]:" ");
    u8g2.setCursor(58, 9);
    u8g2.print(ausgabe);
    //INA219 Display
    if (ina219_installed and (current_servo > 0 or show_current == 1)) {
      u8g2.setCursor(0, 18);
      sprintf(ausgabe, "%s:", SERVO_CURRENT);
      u8g2.print(ausgabe);
      sprintf(ausgabe, "%4imA", current_mA);
      u8g2.setCursor(128 - 36, 18);
      u8g2.print(ausgabe);
    }
    // Zeile unten
    // Verstellung nur wenn Automatik inaktiv, gesteuert über Interrupt-Funktion 
    if (servo_aktiv == 1) {
      progressbar = 128.0*((float)gewicht/(float)zielgewicht);
      progressbar = constrain(progressbar,0,128);
      u8g2.drawFrame(0, 55, 128, 9);
      u8g2.drawBox  (0, 55, progressbar, 9);
    } 
    else {
      u8g2.setFont(u8g2_font_courB08_tf);
      sprintf(ausgabe, "%s%s=%-3d" ,(autokorrektur==1)?A_LOWER_CASE[lingo]:"", C_LOWER_CASE[lingo] , korrektur + autokorrektur_gr);
      n = StringLenght(ausgabe);
      if (rotary_select == SW_KORREKTUR) {
        u8g2.setCursor(10, 62);
        u8g2.setFont(u8g2_font_open_iconic_arrow_1x_t);
        u8g2.drawGlyph(0, 63, 0x42);
        u8g2.drawGlyph(n * 6 + 1 + 10, 63, 0x41);
        u8g2.setFont(u8g2_font_courB08_tf);
      }
      else {
        u8g2.setCursor(0, 62);
      }
      u8g2.print(ausgabe);
      sprintf(ausgabe, "%s=%-3d" ,F_LOWER_CASE[lingo], intGewicht);
      n = StringLenght(ausgabe);
      if (rotary_select == SW_FLUSS) {
        u8g2.setCursor(10, 54);
        u8g2.setFont(u8g2_font_open_iconic_arrow_1x_t);
        u8g2.drawGlyph(0, 55, 0x42);
        u8g2.drawGlyph(n * 6 + 1 + 10, 55, 0x41);
        u8g2.setFont(u8g2_font_courB08_tf);
      }
      else {
        u8g2.setCursor(0, 54);
      }
      u8g2.print(ausgabe);

      if (glaeser[fmenge_index].Gewicht > 999) {
        sprintf(ausgabe, "%2.1fkg-%3s", float(glaeser[fmenge_index].Gewicht) / 1000, GlasTypArray[glaeser[fmenge_index].GlasTyp]);
      }
      else if (glaeser[fmenge_index].Gewicht >= 100) {
        sprintf(ausgabe, "%3dg-%3s", glaeser[fmenge_index].Gewicht, GlasTypArray[glaeser[fmenge_index].GlasTyp]);
      }
      else {
        sprintf(ausgabe, "%2dg-%3s", glaeser[fmenge_index].Gewicht, GlasTypArray[glaeser[fmenge_index].GlasTyp]);
      }
      n = StringLenght(ausgabe);
      if (rotary_select == SW_MENU) {
        u8g2.setCursor(128 - (n * 6 + 10), 62);
        u8g2.setFont(u8g2_font_open_iconic_arrow_1x_t);
        u8g2.drawGlyph(128 - 9, 63, 0x41);
        u8g2.drawGlyph(128 - (n * 6 + 20), 63, 0x42);
        u8g2.setFont(u8g2_font_courB08_tf);
      }
      else {
        u8g2.setCursor(128 - (n * 6), 62);
      }
      u8g2.print(ausgabe);
    }
    u8g2.sendBuffer();
  #endif
  #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
    //INA219 Anzeige
    if (ina219_installed == 1 and current_mA != current_mA_alt and (current_servo > 0 or show_current == 1)) {
      gfx->fillRect(260, 170, 70, 18, BACKGROUND);
      gfx->setTextColor(TEXT);
      gfx->setFont(Punk_Mono_Thin_160_100);
      gfx->setCursor(265, 185);
      if (current_mA > current_servo and current_servo > 0) {
        gfx->setTextColor(RED);
      }
      sprintf(ausgabe, "%4imA", current_mA);
      gfx->print(ausgabe);
      gfx->setTextColor(TEXT);
      current_mA_alt = current_mA;
    }
    else if (ina219_installed == 1 and no_ina == 0 and show_current == 0) {
      no_ina = true;
      gfx->setTextColor(TEXT);
      gfx->setFont(Punk_Mono_Thin_160_100);
      gfx->setCursor(265, 185);
      sprintf(ausgabe, "%6s", OFF);
      gfx->print(ausgabe);
    }
    else if (ina219_installed == 0 and no_ina == 0) {
      no_ina = true;
      gfx->setTextColor(TEXT);
      gfx->setFont(Punk_Mono_Thin_160_100);
      gfx->setCursor(265, 185);
      sprintf(ausgabe, "%6s", "-");
      gfx->print(ausgabe);
    }
    //Korrektur Anzeige
    if (korr_alt != korrektur + autokorrektur_gr) {
      gfx->setFont(Punk_Mono_Thin_160_100);
      gfx->setTextColor(BACKGROUND);
      gfx->setCursor(265, 202);
      sprintf(ausgabe, "%5ig", korr_alt);
      gfx->print(ausgabe);
      gfx->setTextColor(TEXT);
      if (rotary_select == SW_KORREKTUR and servo_aktiv == 0) {
        gfx->setTextColor(MARKER);
      }
      gfx->setCursor(265, 202);
      sprintf(ausgabe, "%5ig", korrektur + autokorrektur_gr);
      gfx->print(ausgabe);
      gfx->setTextColor(TEXT);
      korr_alt = korrektur + autokorrektur_gr;
    }
    //Autokorektur Anzeige
    if (autokorr_gr_alt != autokorrektur_gr) {
      gfx->setFont(Punk_Mono_Thin_160_100);
      gfx->setTextColor(BACKGROUND);
      gfx->setCursor(265, 219);
      sprintf(ausgabe, "%5ig", autokorr_gr_alt);
      gfx->print(ausgabe);
      gfx->setTextColor(TEXT);
      gfx->setCursor(265, 219);
      sprintf(ausgabe, "%5ig", autokorrektur_gr);
      gfx->print(ausgabe);
      autokorr_gr_alt = autokorrektur_gr;
    }
    //Fluss Anzeige
    if (intGewicht_alt != intGewicht) {
      gfx->setFont(Punk_Mono_Thin_160_100);
      gfx->setTextColor(BACKGROUND);
      gfx->setCursor(265, 236);
      sprintf(ausgabe, "%5ig", intGewicht_alt);
      gfx->print(ausgabe);
      gfx->setTextColor(TEXT);
      if (rotary_select == SW_FLUSS and servo_aktiv == 0) {
        gfx->setTextColor(MARKER);
      }
      gfx->setCursor(265, 236);
      sprintf(ausgabe, "%5ig", intGewicht);
      gfx->print(ausgabe);
      gfx->setTextColor(TEXT);
      if (intGewicht_alt == 0 or intGewicht == 0) {
        gfx->setFont(Checkbox);
        if (intGewicht == 0){
          gfx->setCursor(140, 233);
          gfx->setTextColor(BACKGROUND);
          gfx->print("A");
          gfx->setCursor(140, 233);
          gfx->setTextColor(RED);
          gfx->print("B");
        }
        else {
          gfx->setCursor(140, 233);
          gfx->setTextColor(BACKGROUND);
          gfx->print("B");
          gfx->setCursor(140, 233);
          gfx->setTextColor(GREEN);
          gfx->print("A");
        }
      }
      gfx->setTextColor(TEXT);
      intGewicht_alt = intGewicht;
    }
    //Glasauswahl
    if ((glas_alt != fmenge_index and servo_aktiv == 0 and gewicht <= glaeser[fmenge_index].Gewicht - tare)) {
      if (rotary_select == SW_MENU and servo_aktiv == 0) {
        gfx->setTextColor(MARKER);
      }
      gfx->fillRect(0, 136, 320, 27, BACKGROUND);
      gfx->setFont(Punk_Mono_Thin_240_150);
      gfx->setCursor(2, 156);
      sprintf(ausgabe, "%dg ", (glaeser[fmenge_index].Gewicht));
      gfx->print(ausgabe);
      gfx->setFont(Punk_Mono_Thin_120_075);
      gfx->setCursor(2 + 14*StringLenght(ausgabe), 148);
      sprintf(ausgabe, "+%dg ", (kulanz_gr));
      gfx->print(ausgabe);
      gfx->setFont(Punk_Mono_Thin_240_150);
      gfx->setCursor(110, 156);
      if (GlasTypArray[glaeser[fmenge_index].GlasTyp] == "DIB") {
        gfx->print("DE Imker Bund");
      }
      else if (GlasTypArray[glaeser[fmenge_index].GlasTyp] == "TOF") {
        gfx->print("TwistOff");
      }
      else if (GlasTypArray[glaeser[fmenge_index].GlasTyp] == "DEE") {
        gfx->print("DeepTwist");
      }
      else if (GlasTypArray[glaeser[fmenge_index].GlasTyp] == "SPZ") {
        gfx->print("Spezial");
      }
      gfx->setTextColor(TEXT);
      glas_alt = fmenge_index;
    }
    //min. Winkel
    if (winkel_min  + offset_winkel != winkel_min_alt) {
      gfx->setTextColor(TEXT);
      gfx->fillRect(37, 3, 40, 16, BACKGROUND);
      gfx->setFont(Punk_Mono_Thin_160_100);
      gfx->setCursor(38, 17);
      sprintf(ausgabe, "%3i°", winkel_min + offset_winkel);
      gfx->print(ausgabe);
      winkel_min_alt = winkel_min + offset_winkel;
    }
    //Ist Position
    if (pos != pos_alt) {
      gfx->fillRect(154, 3, 40, 16, BACKGROUND);
      gfx->setFont(Punk_Mono_Thin_160_100);
      gfx->setCursor(155, 17);
      sprintf(ausgabe, "%3i°", winkel_max*pos/100);
      gfx->print(ausgabe);
      pos_alt = pos;
    }
    //max. Winkel
    if (winkel != winkel_ist_alt) {
      gfx->fillRect(271, 3, 40, 16, BACKGROUND);
      gfx->setFont(Punk_Mono_Thin_160_100);
      gfx->setCursor(272, 17);
      sprintf(ausgabe, "%3i°", winkel);
      gfx->print(ausgabe);
      winkel_ist_alt = winkel;
    }
    //Start/Stop Icon
    if (auto_aktiv != auto_aktiv_alt) {
      gfx->fillRect(12, 44, 38, 38, BACKGROUND);
      gfx->setFont(Icons_Start_Stop);
      gfx->setCursor(5, 88);
      gfx->print("M");
      gfx->setCursor(5, 88);
      if (auto_aktiv == 1) {
        gfx->setTextColor(GREEN);
        gfx->print("A");
        glas_alt = -1;
        korr_alt = -99999;
        intGewicht_alt = -1;
      }
      else {
        gfx->setTextColor(RED);
        gfx->print("B");
      }
      auto_aktiv_alt = auto_aktiv;
      gfx->setTextColor(TEXT);
    }
    //Glas aufstellen
    if (tare > 0) {
      if (gewicht < -20 and gewicht != gewicht_alt) {
        gfx->fillRect(80, 24, 240, 80, BACKGROUND);
        gfx->setFont(Punk_Mono_Bold_320_200);
        gfx->setCursor(120, 58);
        gfx->print(PLEASE_PUT[lingo]);
        gfx->setCursor(120, 90);
        gfx->print(UP_THE_JAR[lingo]);
        glas_alt = -1;
      } 
      else if (gewicht != gewicht_alt) {
        gfx->fillRect(80, 24, 240, 80, BACKGROUND);
        gfx->setFont(Punk_Mono_Bold_600_375);
        gfx->setCursor(100, 85);
        sprintf(ausgabe, "%5ig", gewicht - tare_glas);
        gfx->print(ausgabe);
      }
      if (gewicht != gewicht_alt) {
        progressbar = 318.0*((float)gewicht/(float)(glaeser[fmenge_index].Gewicht));
        progressbar = constrain(progressbar,0,318);
        gfx->drawRect(0, 107, 320, 15, TEXT);
        if (glaeser[fmenge_index].Gewicht > gewicht) {
          gfx->fillRect  (1, 108, progressbar, 13, RED);
        }
        else if (gewicht >= glaeser[fmenge_index].Gewicht and gewicht <= glaeser[fmenge_index].Gewicht + kulanz_gr){
          gfx->fillRect  (1, 108, progressbar, 13, GREEN);
        }
        else {
          gfx->fillRect  (1, 108, progressbar, 13, ORANGE);
        }
        gfx->fillRect  (1 + progressbar, 108, 318 - progressbar, 13, BACKGROUND);
        gewicht_alt = gewicht;
      }
    }
    //kein Tara vorhanden
    else if (gewicht != gewicht_alt) {        //kein Tara vorhanden
      gfx->fillRect(80, 24, 240, 80, BACKGROUND);
      gfx->fillRect(0, 107, 320, 15, BACKGROUND);
      gfx->setTextColor(RED);
      gfx->setFont(Punk_Mono_Bold_320_200);
      gfx->setCursor(120, 74);
      gfx->print(NO_TARE[lingo]);                 //kein Tara
      gfx->setTextColor(TEXT);
      gewicht_alt = gewicht;
    }
  #endif
  //Turntable
  #if DREHTELLER == 1
    if (use_turntable == 1 and turntable_init_check == true) {
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
    }
    if (use_turntable == 1 and auto_aktiv == 1 and tare > 0) {
      if (gewicht >= -20 and drop_prodection == 0 and turntable_moving == 0) {
        turntable_ok = true;
      } 
      else {
        turntable_ok = false;
      }
      //Glass ist auf der Waage
      if (gewicht >= -20 and drop_prodection == 1 and turntable_moving == 0 and gewicht <= zielgewicht) {
        esp_now_msg_recived = false;
        memset(&myReceivedMessage, 0, sizeof(myReceivedMessage));
        strcpy(myMessageToBeSent.text, "open_drop_prodection");
        espnow_send_data();
        time = millis();
        while (!esp_now_msg_recived and millis() - time <= 10000 and strcmp(myReceivedMessage.text, "") == 0) {
          delay(10);
        }
        if (strcmp(myReceivedMessage.text, "ok_open_dp") == 0) {
          drop_prodection = false;
        }
      }
      //Glass wurde von der Waage entfernt
      else if (gewicht < -20 and drop_prodection == 0 and turntable_moving == 0) {
        esp_now_msg_recived = false;
        memset(&myReceivedMessage, 0, sizeof(myReceivedMessage));
        strcpy(myMessageToBeSent.text, "close_drop_prodection");
        espnow_send_data();
        time = millis();
        while (!esp_now_msg_recived and millis() - time <= 60000 and strcmp(myReceivedMessage.text, "") == 0) {
          delay(10);
        }
        if (strcmp(myReceivedMessage.text, "ok_close_dp") == 0) {
          drop_prodection = true;
        }
      }
      //Glass ist voll
      else if (gewicht >= zielgewicht and drop_prodection == 0 and turntable_moving == 0) {
        SERVO_WRITE(winkel_min + offset_winkel);
        delay(2000);      //lassen wir mal dem Servo Zeit um zu schliessen. haben es ja nicht eilig
        esp_now_msg_recived = false;
        memset(&myReceivedMessage, 0, sizeof(myReceivedMessage));
        strcpy(myMessageToBeSent.text, "close_drop_prodection");
        espnow_send_data();
        time = millis();
        while (!esp_now_msg_recived and millis() - time <= 60000 and strcmp(myReceivedMessage.text, "") == 0) {
          delay(10);
        }
        if (strcmp(myReceivedMessage.text, "ok_close_dp") == 0) {
          drop_prodection = true;
        }
      }
      //Move Turntable
      if ((gewicht < -20 and drop_prodection == 1 and turntable_moving == 0) or (gewicht >= zielgewicht and drop_prodection == 1 and turntable_moving == 0)) {
        memset(&myReceivedMessage, 0, sizeof(myReceivedMessage));
        esp_now_msg_recived = false;
        turntable_moving = true;
        strcpy(myMessageToBeSent.text, "move_jar");
        espnow_send_data();
      }
      else if (drop_prodection == 1 and turntable_moving == 1 and esp_now_msg_recived == true) {
        esp_now_msg_recived = false;
        if (strcmp(myReceivedMessage.text, "ok_move_jar") == 0) {
          turntable_moving = false;
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
    if (servo.read() <= winkel_min  + offset_winkel and offset_winkel < 3) {
      while(offset_winkel < 3 and current_servo < current_mA) {
        offset_winkel = offset_winkel + 1;
        SERVO_WRITE(winkel_min + offset_winkel);
        current_mA = GetCurrent(10);
        delay(1000);
      }
      alarm_overcurrent = 0;
    }
    i = 0;
    inawatchdog = 1;
  }
  setPreferences();
}

void processHandbetrieb(void) {
  i = 0;
  int y_offset = 0;
  if (modus != MODE_HANDBETRIEB) {
    modus = MODE_HANDBETRIEB;
    winkel = winkel_min;          // Hahn schliessen
    servo_aktiv = 0;              // Servo-Betrieb aus
    SERVO_WRITE(winkel);
    rotary_select = SW_WINKEL;
    tare = 0;
    offset_winkel = 0;            // Offset vom Winkel wird auf 0 gestellt
    #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
      int x_pos;
      gewicht_alt = -9999999;
      pos_alt = -1;
      tare_alt = -1;
      current_mA_alt = -1;
      servo_aktiv_alt = -1;
      winkel_ist_alt = -1;
      gfx->fillScreen(BACKGROUND);
      gfx->setTextColor(TEXT);
      gfx->setFont(Punk_Mono_Bold_320_200);
      sprintf(ausgabe,MANUAL[lingo]);
      x_pos = CenterPosX(ausgabe, 18, 320);
      gfx->setCursor(x_pos, 27);
      gfx->print(ausgabe);
      gfx->drawLine(0, 30, 320, 30, TEXT);
      gfx->setFont(Punk_Mono_Bold_200_125);
      gfx->setCursor(5, 192);
      gfx->printf("%s:", SERVO);
      gfx->drawLine(0, 170, 320, 170, TEXT);
      gfx->drawLine(160, 170, 160, 240, TEXT);
      gfx->setCursor(170, 192);
      gfx->printf("%s:", TARE);
      if (ina219_installed == 1) {
        gfx->setCursor(170, 215);
        sprintf(ausgabe,"%s:",INA);
        gfx->print(ausgabe);
        sprintf(ausgabe, "%s", (current_servo==0?OFF:ON));
        int y = get_length(ausgabe);
        gfx->setCursor(320 - y * 12 - 5, 215);
        gfx->print(ausgabe);
        if (ina219_installed == 1 and current_mA != current_mA_alt and (current_servo > 0 or show_current == 1)) {
          gfx->setCursor(170, 238);
          gfx->printf("%s:", CURR);
        }
      }
    #endif
  }
  pos = getRotariesValue(SW_WINKEL);
  gewicht = SCALE_GETUNITS(SCALE_READS) - tare;
  if ((digitalRead(button_start_pin)) == HIGH) {
    servo_aktiv = 1;
  }
  if ((digitalRead(button_stop_pin)) == HIGH) {
    servo_aktiv = 0;
  }
  if ((digitalRead(outputSW)) == LOW) {
      tare = SCALE_GETUNITS(SCALE_READS);
  }
  if (servo_aktiv == 1) {
    winkel = ((winkel_max * pos) / 100);
  }
  else { 
    winkel = winkel_min + offset_winkel;
  }
  winkel = constrain(winkel, winkel_min + offset_winkel, winkel_max);
  SERVO_WRITE(winkel);
  if (ina219_installed and (current_servo > 0 or show_current == 1)) {
    y_offset = 4;
  }
  #ifdef isDebug
    #if isDebug >= 4
      Serial.print("Handbetrieb:");  
      Serial.print(" Gewicht ");     Serial.print(gewicht);
      Serial.print(" Winkel ");      Serial.print(winkel);
      Serial.print(" servo_aktiv "); Serial.println(servo_aktiv);
    #endif
  #endif
  #if DISPLAY_TYPE == 1 or DISPLAY_TYPE == 2 or DISPLAY_TYPE == 99
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_courB24_tf);
    u8g2.setCursor(10, 42 + y_offset);
    sprintf(ausgabe,"%5dg", gewicht);
    u8g2.print(ausgabe);
    u8g2.setFont(u8g2_font_open_iconic_play_2x_t);
    u8g2.drawGlyph(0, 40 + y_offset, (servo_aktiv==1)?0x45:0x44 );
    u8g2.setFont(u8g2_font_courB08_tf);
    u8g2.setCursor(0, 11);
    u8g2.setCursor(1, 9);
    sprintf(ausgabe,"%s=%d°", A_UPPER_CASE[lingo], winkel);
    u8g2.print(ausgabe);
    sprintf(ausgabe,"%3d%%", pos);
    u8g2.setCursor(104, 9);
    u8g2.print(ausgabe);
    if (ina219_installed and (current_servo > 0 or show_current == 1)) {
      u8g2.setCursor(0, 18);
      sprintf(ausgabe,"%s:", SERVO_CURRENT[lingo]);
      u8g2.print(ausgabe);
      sprintf(ausgabe, "%4imA", current_mA);
      u8g2.setCursor(128 - 36, 18);
      u8g2.print(ausgabe);
    }
    u8g2.setCursor(0, 64);
    u8g2.print(MANUAL[lingo]);
    if (tare > 0) {
      int y = get_length(TARE[lingo]);
      u8g2.setCursor(128 - y * 6, 64);
      u8g2.print(TARE[lingo]);
    }
    u8g2.sendBuffer();
  #endif
  #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
    gfx->setFont(Punk_Mono_Bold_600_375);
    if (gewicht != gewicht_alt) {
      gfx->fillRect(80, 70, 200, 55, BACKGROUND);
      gfx->setFont(Punk_Mono_Bold_600_375);
      gfx->setCursor(100, 120);
      sprintf(ausgabe, "%5ig", gewicht);
      gfx->print(ausgabe);
      gewicht_alt = gewicht;
    }
    if (winkel != winkel_ist_alt) {
      gfx->fillRect(97, 198, 50, 20, BACKGROUND);
      gfx->setFont(Punk_Mono_Bold_200_125);
      gfx->setCursor(5, 215);
      gfx->printf("%s:", ACTUAL);
      sprintf(ausgabe, "%3i°", winkel);
      int y = get_length(ausgabe);
      gfx->setCursor(160 - y * 12 - 11, 215);
      gfx->print(ausgabe);
      winkel_ist_alt = winkel;
    }
    if (pos != pos_alt) {
      gfx->fillRect(97, 221, 50, 20, BACKGROUND);
      gfx->setFont(Punk_Mono_Bold_200_125);
      gfx->setCursor(5, 238);
      gfx->printf("%s:", MAX_1);
      sprintf(ausgabe, "%3i°", winkel_max*pos/100);
      int y = get_length(ausgabe);
      gfx->setCursor(160 - y * 12 - 11, 238);
      gfx->print(ausgabe);
      pos_alt = pos;
    }
    if (tare != tare_alt) {
      gfx->fillRect(230, 175, 90, 22, BACKGROUND);
      gfx->setFont(Punk_Mono_Bold_200_125);      
      sprintf(ausgabe, "%5ig", tare);
      int y = get_length(ausgabe);
      gfx->setCursor(320 - y * 12 - 5, 192);
      gfx->print(ausgabe);
      tare_alt = tare;
    }
    if (ina219_installed == 1 and current_mA != current_mA_alt and (current_servo > 0 or show_current == 1)) {
      gfx->fillRect(240, 221, 80, 20, BACKGROUND);
      gfx->setFont(Punk_Mono_Bold_200_125);
      if (current_mA > current_servo and current_servo > 0) {
        gfx->setTextColor(RED);
      }
      sprintf(ausgabe, "%4imA", current_mA);
      int y = get_length(ausgabe);
      gfx->setCursor(320 - y * 12 - 5, 238);
      gfx->print(ausgabe);
      gfx->setTextColor(TEXT);
      current_mA_alt = current_mA;
    }
    if (servo_aktiv != servo_aktiv_alt) {
      gfx->fillRect(17, 79, 38, 38, BACKGROUND);
      gfx->setFont(Icons_Start_Stop);
      gfx->setCursor(10, 123);
      gfx->print("M");
      gfx->setCursor(10, 123);
      if (servo_aktiv == 1) {
        gfx->setTextColor(GREEN);
        gfx->print("A");
      }
      else {
        gfx->setTextColor(RED);
        gfx->print("B");
      }
      servo_aktiv_alt = servo_aktiv;
      gfx->setTextColor(TEXT);
    }
    
  #endif
  if (alarm_overcurrent) {i = 1;}
  while (i > 0) {
    inawatchdog = 0;                    //schalte die kontiunirliche INA Messung aus
    //Servo ist zu
    if (servo.read() <= winkel_min  + offset_winkel and offset_winkel < 3) {
      while(offset_winkel < 3 and current_servo < current_mA) {
        offset_winkel = offset_winkel + 1;
        SERVO_WRITE(winkel_min + offset_winkel);
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
  Serial.begin(115200);
  while (!Serial) {}
  #ifdef isDebug
    Serial.println("Hanimandl Start");
  #endif
  // Try to initialize the INA219
  if (ina219.begin()) {
    ina219_installed = 1;
    #ifdef isDebug
      Serial.println("INA219 chip gefunden");
    #endif
  }
  else {
    current_servo = 0;                              // ignore INA wenn keiner gefunden wird
    #ifdef isDebug
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
  // Preferences aus dem EEPROM lesen
  getPreferences();
  // Servo initialisieren und schliessen
  #ifdef SERVO_ERWEITERT
    servo.attach(servo_pin,  750, 2500); // erweiterte Initialisierung, steuert nicht jeden Servo an
    servo.setPeriodHertz(100);
  #else
    servo.attach(servo_pin, 1000, 2000); // default Werte. Achtung, steuert den Nullpunkt weniger weit aus!  
  #endif
  SERVO_WRITE(winkel_min);
  // Waage erkennen - machen wir vor dem Boot-Screen, dann hat sie 3 Sekunden Zeit zum aufwärmen
  scale.begin(hx711_dt_pin, hx711_sck_pin);
  if (scale.wait_ready_timeout(1000)) {               // Waage angeschlossen? :Roli - Ist immer null wenn kein HX711 angeschlossen ist
    scale.power_up();
    if (scale.read() != 0) {                          // Roli - Wenn 0, nehme ich an, das kein HX711 angeschlossen ist
      waage_vorhanden = 1;
      #ifdef isDebug
        Serial.println("Waage erkannt");
      #endif
    }
  }
  // Boot Screen
  #if DISPLAY_TYPE == 1 or DISPLAY_TYPE == 2 or DISPLAY_TYPE == 99
    u8g2.setBusClock(800000);   // experimental
    u8g2.begin();
    u8g2.enableUTF8Print();
  #endif
  #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
    tft_colors();
    tft_marker();
    gfx->begin();
    gfx->fillScreen(BACKGROUND);
    gfx->setUTF8Print(true);
  #endif
  if (showlogo) {
    #if DISPLAY_TYPE == 1 or DISPLAY_TYPE == 2 or DISPLAY_TYPE == 99
      u8g2.clearBuffer();
    #endif
    print_logo();
    delay(3000);
  }
  if (showcredits) {
    buzzer(BUZZER_SHORT);
    print_credits();   
    delay(2500);
  }
  // Setup der Waage, Skalierungsfaktor setzen
  int x_pos;
  if (waage_vorhanden ==1) {                         // Waage angeschlossen?
    if (faktor == 0) {                               // Vorhanden aber nicht kalibriert
      #if DISPLAY_TYPE == 1 or DISPLAY_TYPE == 2 or DISPLAY_TYPE == 99
        u8g2.clearBuffer();
        //u8g2.setFont(u8g2_font_courB18_tf);
        u8g2.setFont(u8g2_font_courB12_tf);
        sprintf(ausgabe,NOT[lingo]);
        x_pos = CenterPosX(ausgabe, 10, 128);
        u8g2.setCursor(x_pos, 30); u8g2.print(ausgabe);
        sprintf(ausgabe,CALIBRATED[lingo]);
        x_pos = CenterPosX(ausgabe, 10, 128);
        u8g2.setCursor(x_pos, 46); u8g2.print(ausgabe);
        u8g2.sendBuffer();
      #endif
      #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
        gfx->fillScreen(BACKGROUND);
        gfx->setTextColor(RED);
        gfx->setFont(Punk_Mono_Bold_320_200);
        sprintf(ausgabe,NOT[lingo]);
        x_pos = CenterPosX(ausgabe, 18, 320);
        gfx->setCursor(x_pos, 110);
        gfx->print(ausgabe);
        sprintf(ausgabe,CALIBRATED[lingo]);
        x_pos = CenterPosX(ausgabe, 18, 320);
        gfx->setCursor(x_pos, 154);
        gfx->print(ausgabe);
      #endif
      buzzer(BUZZER_ERROR);
      #ifdef isDebug
        Serial.println("Waage nicht kalibriert!");
      #endif
      delay(2000);
    }
    else {                                          // Tara und Skalierung setzen
      scale.set_scale(faktor);
      scale.set_offset(long(gewicht_leer));
      #ifdef isDebug
        Serial.println("Waage initialisiert");
      #endif
    }
  }
  else {                                            // Keine Waage angeschlossen
    #if DISPLAY_TYPE == 1 or DISPLAY_TYPE == 2 or DISPLAY_TYPE == 99
      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_courB24_tf);
      sprintf(ausgabe,NO[lingo]);
      x_pos = CenterPosX(ausgabe, 20, 128);
      u8g2.setCursor( x_pos, 24); u8g2.print(ausgabe);
      sprintf(ausgabe,SCALE[lingo]);
      x_pos = CenterPosX(ausgabe, 20, 128);
      u8g2.setCursor( x_pos, 56);  u8g2.print(ausgabe);
      u8g2.sendBuffer();
    #endif
    #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
      gfx->fillScreen(BACKGROUND);
      gfx->setTextColor(RED);
      gfx->setFont(Punk_Mono_Bold_320_200);
      sprintf(ausgabe,NO[lingo]);
      x_pos = CenterPosX(ausgabe, 18, 320);
      gfx->setCursor(x_pos, 110);
      gfx->print(ausgabe);
      sprintf(ausgabe,SCALE[lingo]);
      x_pos = CenterPosX(ausgabe, 18, 320);
      gfx->setCursor(x_pos, 154);
      gfx->print(ausgabe);
    #endif
    buzzer(BUZZER_ERROR);
    #ifdef isDebug
      Serial.println("Keine Waage!");
    #endif
    delay(2000);
  }
  // initiale Kalibrierung des Leergewichts wegen Temperaturschwankungen
  // Falls mehr als 20g Abweichung steht vermutlich etwas auf der Waage.
  if (waage_vorhanden == 1) {
    gewicht = SCALE_GETUNITS(SCALE_READS);
    if ((gewicht > -20) && (gewicht < 20)) {
      scale.tare(10);
      buzzer(BUZZER_SUCCESS);
      #ifdef isDebug
        Serial.print("Tara angepasst um: ");
        Serial.println(gewicht);
      #endif
    }
    else if (faktor != 0) {
      int x_pos;
      #if DISPLAY_TYPE == 1 or DISPLAY_TYPE == 2 or DISPLAY_TYPE == 99
        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_courB14_tf);
        sprintf(ausgabe,EMPTY[lingo]);
        x_pos = CenterPosX(ausgabe, 11, 128);
        u8g2.setCursor( x_pos, 28); u8g2.print(ausgabe);
        sprintf(ausgabe,THE_SCALE[lingo]);
        x_pos = CenterPosX(ausgabe, 11, 128);
        u8g2.setCursor(x_pos, 52); u8g2.print(ausgabe);
        u8g2.sendBuffer();
      #endif
      #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
        gfx->fillScreen(BACKGROUND);
        gfx->setTextColor(RED);
        gfx->setFont(Punk_Mono_Bold_320_200);
        sprintf(ausgabe,EMPTY[lingo]);
        x_pos = CenterPosX(ausgabe, 18, 320);
        gfx->setCursor(x_pos, 110);
        gfx->print(ausgabe);
        sprintf(ausgabe,THE_SCALE[lingo]);
        x_pos = CenterPosX(ausgabe, 18, 320);
        gfx->setCursor(x_pos, 154);
        gfx->print(ausgabe);
      #endif
      #ifdef isDebug
        Serial.print("Gewicht auf der Waage: ");
        Serial.println(gewicht);
      #endif
      delay(5000);
      // Neuer Versuch, falls Gewicht entfernt wurde
      gewicht = SCALE_GETUNITS(SCALE_READS);
      if ((gewicht > -20) && (gewicht < 20)) {
        scale.tare(10);
        buzzer(BUZZER_SUCCESS);
        #ifdef isDebug
          Serial.print("Tara angepasst um: ");
          Serial.println(gewicht);
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
  setRotariesValue(SW_KORREKTUR, korrektur);
  setRotariesValue(SW_FLUSS,     intGewicht);
  setRotariesValue(SW_MENU,      fmenge_index);
  //Drehteller
  #if DREHTELLER == 1
    WiFi.mode(WIFI_STA);
    if (esp_now_init() == ESP_OK) {
      esp_now_ini =true;
      #ifdef isDebug
        Serial.println("ESPNow Init success");
      #endif
    }
    else {
      esp_now_ini =false;
      #ifdef isDebug
        Serial.println("ESPNow Init fail");
      #endif
    }
    
    esp_now_register_send_cb(messageSent);  
    esp_now_register_recv_cb(messageReceived); 
    memcpy(peerInfo.peer_addr, MacAdressTurntable, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      esp_now_ini =false;
      #ifdef isDebug
        Serial.println("Failed to add peer");
      #endif
    }
    unsigned long turntable_millis = millis();
    esp_now_msg_recived = false;
    strcpy(myMessageToBeSent.text, "close_drop_prodection");
    espnow_send_data();
    while (millis() - turntable_millis < 10000 and esp_now_msg_recived == false) {  //brechce ab wenn in 10 sek keine Rückmeldung kommt
      delay(10);
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
  rotating = true;     // debounce Management
  //INA219 Messung
  if (ina219_installed and inawatchdog == 1 and (current_servo > 0 or show_current == 1) and (modus == MODE_HANDBETRIEB or modus == MODE_AUTOMATIK)) {
    ina219_measurement();
  }
  // Setup Menu 
  if ((digitalRead(switch_setup_pin)) == HIGH) {
    processSetup();
  }
  // Automatik-Betrieb 
  else if ((digitalRead(switch_betrieb_pin)) == HIGH) {
    processAutomatik();
  }
  // Handbetrieb 
  else if ((digitalRead(switch_betrieb_pin) == LOW) && (digitalRead(switch_setup_pin) == LOW)) {
    processHandbetrieb();
  }
}

void print_credits() {
  #if DISPLAY_TYPE == 1 or DISPLAY_TYPE == 2 or DISPLAY_TYPE == 99
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_helvB08_tf);
    u8g2.setCursor(0, 10);    u8g2.print("Idee: M. Vasterling");
    u8g2.setCursor(0, 22);    u8g2.print("Code: M. Vasterling,");
    u8g2.setCursor(0, 32);    u8g2.print("M. Wetzel, C. Gruber,");
    u8g2.setCursor(0, 42);    u8g2.print("A. Holzhammer, M. Junker,");
    u8g2.setCursor(0, 52);    u8g2.print("J. Kuder, J. Bruker,");
    u8g2.setCursor(0, 62);    u8g2.print("R. Rust, A. Pfaff");
    u8g2.sendBuffer();
  #endif
  #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
    int offset = 22;
    gfx->fillScreen(BACKGROUND);
    gfx->setTextColor(TEXT);
    gfx->setFont(Punk_Mono_Bold_240_150);
    gfx->setCursor(10, 1*offset);
    gfx->print("Idee:");
    gfx->setCursor(90, 1*offset);
    gfx->print("M. Vasterling");
    gfx->setCursor(10, 2*offset+10);
    gfx->print("Code:");
    gfx->setCursor(90, 2*offset+10);
    gfx->print("M. Vasterling");
    gfx->setCursor(90, 3*offset+10);
    gfx->print("M. Wetzel");
    gfx->setCursor(90, 4*offset+10);
    gfx->print("C. Gruber");
    gfx->setCursor(90, 5*offset+10);
    gfx->print("A. Holzhammer");
    gfx->setCursor(90, 6*offset+10);
    gfx->print("M. Junker");
    gfx->setCursor(90, 7*offset+10);
    gfx->print("J. Kuder");
    gfx->setCursor(90, 8*offset+10);
    gfx->print("J. Bruker");
    gfx->setCursor(90, 9*offset+10);
    gfx->print("R. Rust");             //A.P.
    gfx->setCursor(90, 10*offset+10);
    gfx->print("A. Pfaff");            //A.P.
  #endif
}

#if USER == 2
  void print_logo() {
    #if DISPLAY_TYPE == 1 or DISPLAY_TYPE == 2 or DISPLAY_TYPE == 99
      u8g2.clearBuffer();
      u8g2.drawXBM(0,0,128,64,LogoGeroldOLED);
      u8g2.sendBuffer();
    #endif
    #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
    gfx->drawXBitmap(0, 0, LogoGeroldTFT, 320, 240, TEXT);
  #endif
  }
#elif USER == 3
void print_logo() {
  #if DISPLAY_TYPE == 1 or DISPLAY_TYPE == 2 or DISPLAY_TYPE == 99
    u8g2.clearBuffer();
    u8g2.drawXBM(0,0,128,64,LogoRoliOLED);
    u8g2.setFont(u8g2_font_courB14_tf);
    u8g2.setCursor(30, 27);    u8g2.print("HANI");
    u8g2.setCursor(40, 43);    u8g2.print("MANDL");
    u8g2.setFont(u8g2_font_courB08_tf);
    u8g2.setCursor(95, 64);    u8g2.print(version);
    u8g2.sendBuffer();
  #endif
  #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
    gfx->drawXBitmap(0, 0, LogoRoliTFT, 320, 240, YELLOW);
    gfx->setTextColor(YELLOW);
    gfx->setFont(Punk_Mono_Bold_200_125);
    gfx->setCursor(232, 220);
    gfx->print(version);
  #endif
}
#else
void print_logo() {
  #if DISPLAY_TYPE == 1 or DISPLAY_TYPE == 2 or DISPLAY_TYPE == 99
    u8g2.clearBuffer();
    u8g2.drawXBM(0,0,80,64,LogoBieneOLED);
    u8g2.setFont(u8g2_font_courB14_tf);
    u8g2.setCursor(85, 27);    u8g2.print("HANI");
    u8g2.setCursor(75, 43);    u8g2.print("MANDL");
    u8g2.setFont(u8g2_font_courB08_tf);
    u8g2.setCursor(77, 64);    u8g2.print(version);
    u8g2.sendBuffer();
    //u8g2.writeBufferXBM(Serial);	
  #endif
  #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
    gfx->fillScreen(BACKGROUND);
    gfx->drawXBitmap(60, 10, LogoBieneTFT, 200, 160, TEXT);
    gfx->setTextColor(TEXT);
    gfx->setFont(Punk_Mono_Bold_600_375);
    gfx->setCursor(0, 180);
    gfx->print("HANI");
    gfx->setCursor(140, 235);
    gfx->print("MANDL");
    gfx->setFont(Punk_Mono_Bold_200_125);
    gfx->setCursor(255, 180);
    gfx->print(version);
  #endif
}
#endif

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

//Sub function for Display
int StringLenght(String a) {
  a.trim();
  int res = a.length();
  return res;
}

int CenterPosX(const char a[], float font_width, int display_widht) {
  int res = 0;
  int counter = get_length(a);
  res = int(float(display_widht - font_width * counter) / 2);
  return res;
}

int get_length(const char a[]) {
  int counter = 0;
  for (int i = 0; i < strlen(a); i++) {
    if ((unsigned char) a[i] >= 0x00 and (unsigned char) a[i] <= 0x7F) {
      counter++;
    }
    else if ((unsigned char) a[i] >= 0x80 and (unsigned char) a[i] <= 0x07FF) {
      i += 1;
      counter++;
    }
    else if ((unsigned char) a[i] >= 0x0800 and (unsigned char) a[i] <= 0xFFFF) {
      i += 2;
      counter++;
    }
    else if ((unsigned char) a[i] >= 0x010000 and (unsigned char) a[i] <= 0x10FFFF) {
      i += 3;
      counter++;
    }
    else {
      i += 4;      //ist kein UTF8 Zeichen. Sollte also nie hier ankommen
    }
  }
  return counter;
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
      #ifdef ESP_NOW_DEBUG
        Serial.print("Sending error: Text: "); Serial.print(myMessageToBeSent.text); Serial.print(" - Value: "); Serial.println(myMessageToBeSent.value);
      #endif
    }
    else {
      bool esp_now_send_error = false;
      #ifdef ESP_NOW_DEBUG
        Serial.print("Sending sucsess: "); Serial.print(myMessageToBeSent.text); Serial.print(" - Value: "); Serial.println(myMessageToBeSent.value);
      #endif
    }
    memset(&myMessageToBeSent, 0, sizeof(myMessageToBeSent));
  #endif
}

//OTA
#if OTA == 1
  void onOTAStart() {
    int x_pos;
    #ifdef isDebug
      Serial.println("OTA update started!");
    #endif
    #if DISPLAY_TYPE == 1 or DISPLAY_TYPE == 2 or DISPLAY_TYPE == 99
      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_courB08_tf);
      sprintf(ausgabe, "Update started");
      x_pos = CenterPosX(ausgabe, 6, 128);
      u8g2.setCursor(x_pos, 35);
      u8g2.print(ausgabe);
      u8g2.sendBuffer();
    #endif
    #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
      gfx->fillScreen(BACKGROUND);
      gfx->setTextColor(TEXT);
      gfx->setFont(Punk_Mono_Bold_320_200);
      sprintf(ausgabe, "Update started");
      x_pos = CenterPosX(ausgabe, 18, 320);
      gfx->setCursor(x_pos, 27);
      gfx->print(ausgabe);
      gfx->drawLine(0, 37, 320, 37, TEXT);
    #endif
  }

  void onOTAProgress(size_t current, size_t final) {
    if (millis() - ota_progress_millis > 1000) {
      ota_progress_millis = millis();
      #if DISPLAY_TYPE == 1 or DISPLAY_TYPE == 2 or DISPLAY_TYPE == 99
        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_courB08_tf);
        u8g2.setCursor(1, 10);
        u8g2.print("OTA Progress:");
        u8g2.setCursor(1, 25);
        sprintf(ausgabe, "Current: %u", current);
        u8g2.print(ausgabe);
        u8g2.setCursor(1, 35);
        sprintf(ausgabe, "Final:   %u", final);
        u8g2.print(ausgabe);
        u8g2.sendBuffer();
      #endif
      #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
        gfx->fillRect(0, 38, 320, 202, BACKGROUND);
        gfx->setTextColor(TEXT);
        gfx->setFont(Punk_Mono_Bold_320_200);
        gfx->setCursor(10, 100);
        gfx->print("OTA Progress:");
        gfx->setCursor(10, 140);
        sprintf(ausgabe, "Current: %u", current);
        gfx->print(ausgabe);
        gfx->setCursor(10, 170);
        sprintf(ausgabe, "Final:   %u", final);
        gfx->print(ausgabe);
      #endif
      #ifdef isDebug
        Serial.printf("OTA Progress Current: %u bytes, Final: %u bytes\n", current, final);
      #endif
    }
  }

  void onOTAEnd(bool success) {
    int x_pos;
    if (success) {
      #if DISPLAY_TYPE == 1 or DISPLAY_TYPE == 2 or DISPLAY_TYPE == 99
        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_courB08_tf);
        sprintf(ausgabe, "Successfully");
        x_pos = CenterPosX(ausgabe, 6, 128);
        u8g2.setCursor(x_pos, 35);
        u8g2.print(ausgabe);
        u8g2.sendBuffer();
      #endif
      #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
        gfx->fillScreen(BACKGROUND);
        gfx->setTextColor(GREEN);
        gfx->setFont(Punk_Mono_Bold_320_200);
        sprintf(ausgabe, "Successfully");
        x_pos = CenterPosX(ausgabe, 18, 320);
        gfx->setCursor(x_pos, 150);
        gfx->print(ausgabe);
      #endif
      ota_done = 1;
      #ifdef isDebug
        Serial.println("OTA update finished successfully!");
      #endif
    } 
    else {
      #if DISPLAY_TYPE == 1 or DISPLAY_TYPE == 2 or DISPLAY_TYPE == 99
        u8g2.clearBuffer();
        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_courB08_tf);
        sprintf(ausgabe, "Error");
        x_pos = CenterPosX(ausgabe, 6, 128);
        u8g2.setCursor(x_pos, 35);
        u8g2.print(ausgabe);
        u8g2.sendBuffer();
      #endif
      #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
        gfx->fillScreen(BACKGROUND);
        gfx->setTextColor(RED);
        gfx->setFont(Punk_Mono_Bold_320_200);
        sprintf(ausgabe, "Error");
        x_pos = CenterPosX(ausgabe, 18, 320);
        gfx->setCursor(x_pos, 150);
        gfx->print(ausgabe);
      #endif
      #ifdef isDebug
        Serial.println("There was an error during OTA update!");
      #endif
    }
  }

  void OTAdisconnect(void) {
    #ifdef isDebug
      Serial.println("OTAdisconnect");
    #endif
    int x_pos;
    #if DISPLAY_TYPE == 1 or DISPLAY_TYPE == 2 or DISPLAY_TYPE == 99
      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_courB14_tf);
      sprintf(ausgabe, "Stop WiFi");
      x_pos = CenterPosX(ausgabe, 11, 128);
      u8g2.setCursor(x_pos, 35);
      u8g2.print(ausgabe);
      u8g2.sendBuffer();
    #endif
    #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
      gfx->fillRect(0, 38, 320, 202, BACKGROUND);
      gfx->setTextColor(TEXT);
      gfx->setFont(Punk_Mono_Bold_320_200);
      sprintf(ausgabe, "Stop WiFi");
      x_pos = CenterPosX(ausgabe, 18, 320);
      gfx->setCursor(x_pos, 150);
      gfx->print(ausgabe);
    #endif
    WiFi.disconnect();
    delay(500);
    while (WiFi.status() == WL_CONNECTED and digitalRead(switch_setup_pin) == HIGH) {delay(500);}
  }

  void OTALoop(void) {
    #ifdef isDebug
      Serial.println("OTALoop");
    #endif
    while (WiFi.status() == WL_CONNECTED and digitalRead(switch_setup_pin) == HIGH) {
      if (digitalRead(button_stop_pin) == HIGH) {
        while (digitalRead(button_stop_pin) == HIGH) {delay(100);}
        OTAdisconnect();
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
      OTAdisconnect();
    }
  }

  void OTASetup(void) {
    int x_pos;
    #ifdef isDebug
      Serial.println("OTASetup");
    #endif
    #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
      gfx->fillScreen(BACKGROUND);
      gfx->setTextColor(TEXT);
      gfx->setFont(Punk_Mono_Bold_320_200);
      sprintf(ausgabe, "ElegantOTA");
      x_pos = CenterPosX(ausgabe, 18, 320);
      gfx->setCursor(x_pos, 27);
      gfx->print(ausgabe);
      gfx->drawLine(0, 37, 320, 37, TEXT);
    #endif
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    // Wait for connection
    char x[] = "";
    int i = 0;
    unsigned long wifi_start_millis = millis();
    while (WiFi.status() != WL_CONNECTED and millis() - wifi_start_millis < 15000 and digitalRead(switch_setup_pin) == HIGH) {
      #if DISPLAY_TYPE == 1 or DISPLAY_TYPE == 2 or DISPLAY_TYPE == 99
        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_courB14_tf);
        sprintf(ausgabe, "Start WiFi");
        x_pos = CenterPosX(ausgabe, 11, 128);
        u8g2.setCursor(x_pos, 25);
        u8g2.print(ausgabe);
      #endif
      if (i < 10) {
        sprintf(x, "%s*", x);
        i++;
      }
      else {
        i = 1;
        sprintf(x, "%s", "*");
      }
      #if DISPLAY_TYPE == 1 or DISPLAY_TYPE == 2 or DISPLAY_TYPE == 99
        x_pos = CenterPosX(x, 11, 128);
        u8g2.setCursor(x_pos, 45);
        u8g2.print(x);
        u8g2.sendBuffer();
      #endif
      #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
        gfx->setTextColor(TEXT);
        gfx->setFont(Punk_Mono_Bold_320_200);
        sprintf(ausgabe, "Start WiFi");
        x_pos = CenterPosX(ausgabe, 18, 320);
        gfx->setCursor(x_pos, 125);
        gfx->print(ausgabe);
        gfx->fillRect(0, 150, 320, 22, BACKGROUND);
        x_pos = CenterPosX(x, 18, 320);
        gfx->setCursor(x_pos, 170);
        gfx->print(x);
      #endif
      delay(500);
    }
    if (WiFi.status() == WL_CONNECTED and digitalRead(switch_setup_pin) == HIGH) {
      #if DISPLAY_TYPE == 1 or DISPLAY_TYPE == 2 or DISPLAY_TYPE == 99
        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_courB08_tf);
        sprintf(ausgabe, "%i.%i.%i.%i", WiFi.localIP()[0], WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3]);
        x_pos = CenterPosX(ausgabe, 6, 128);
        u8g2.setCursor(x_pos, 35);
        u8g2.print(ausgabe);
        u8g2.sendBuffer();
      #endif
      #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
        gfx->fillRect(0, 38, 320, 202, BACKGROUND);
        gfx->setTextColor(TEXT);
        gfx->setFont(Punk_Mono_Bold_320_200);
        sprintf(ausgabe, "%i.%i.%i.%i", WiFi.localIP()[0], WiFi.localIP()[1], WiFi.localIP()[2], WiFi.localIP()[3]);
        x_pos = CenterPosX(ausgabe, 18, 320);
        gfx->setCursor(x_pos, 150);
        gfx->print(ausgabe);
      #endif
      server.on("/", []() {
        sprintf(ausgabe, "HaniMandl %s", version);
        server.send(200, "text/plain", ausgabe);
      });
      #ifdef isDebug
        Serial.print("IP adress: "); Serial.println(WiFi.localIP());
      #endif
      ElegantOTA.begin(&server);
      ElegantOTA.onStart(onOTAStart);
      ElegantOTA.onProgress(onOTAProgress);
      ElegantOTA.onEnd(onOTAEnd);
      server.begin();
    }
    if (WiFi.status() == WL_CONNECTED and digitalRead(switch_setup_pin) == HIGH) {
      OTALoop();
    }
    else {
      #if DISPLAY_TYPE == 1 or DISPLAY_TYPE == 2 or DISPLAY_TYPE == 99
        u8g2.clearBuffer();
        u8g2.setFont(u8g2_font_courB14_tf);
        sprintf(ausgabe, "Start WiFi");
        x_pos = CenterPosX(ausgabe, 11, 128);
        u8g2.setCursor(x_pos, 25);
        u8g2.print(ausgabe);
        sprintf(ausgabe, "failed");
        x_pos = CenterPosX(ausgabe, 11, 128);
        u8g2.setCursor(x_pos, 45);
        u8g2.print(ausgabe);
        u8g2.sendBuffer();
      #endif
      #if DISPLAY_TYPE == 3 or DISPLAY_TYPE == 99
        gfx->fillRect(0, 150, 320, 22, BACKGROUND);
        gfx->setTextColor(RED);
        gfx->setFont(Punk_Mono_Bold_320_200);
        sprintf(ausgabe, "failed");
        x_pos = CenterPosX(ausgabe, 18, 320);
        gfx->setCursor(x_pos, 170);
        gfx->print(ausgabe);
        gfx->setTextColor(TEXT);
      #endif
      #ifdef isDebug
        Serial.print("IP adress: "); Serial.println(WiFi.localIP());
      #endif
    }
    delay(3000);
  }
#endif
