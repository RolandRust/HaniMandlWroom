# HaniMandlWroom/Hardware
HaniMandel für das Node MCU ESP32 38Pin Evaluation Board
## Was findet man in diesem Ordner
In diesem Ordner findet man alles was mir der Software vom HaniMandel zu tun hat
## Einstellungen
```
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
#define DISPLAY_TYPE 3            // 1 = 128x64 pixel OLED Display angeschlossen über I2C
                                  // 2 = 128x64 pixel OLED Display angeschlossen über SPI
                                  // 3 = 320x240 pixel TFT Display ST7789 angeschlossen über SPI
                                  // 99 = Oled über I2C und TFT über SPI für development
#define LANGUAGE 1                // 1 = Deutsch
                                  // 2 = Englisch
#define OTA 0                     // 0 = OTA Uptade ausgeschalten
                                  // 1 = OTA Update eingeschalten
//#define FEHLERKORREKTUR_WAAGE   // falls Gewichtssprünge auftreten, können diese hier abgefangen werden
                                  // Achtung, kann den Wägeprozess verlangsamen. Vorher Hardware prüfen.
//#define QUETSCHHAHN_LINKS       // Servo invertieren, falls der Quetschhahn von links geöffnet wird. Mindestens ein Exemplar bekannt

//
// Ende Benutzereinstellungen!
// 
```
