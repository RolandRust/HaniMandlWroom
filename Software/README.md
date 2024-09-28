# HaniMandlWroom/Hardware
HaniMandel für das Node MCU ESP32 38Pin Evaluation Board
## Was findet man in diesem Ordner
In diesem Ordner findet man alles was mir der Software vom HaniMandel zu tun hat
## Einstellungen
```
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
#define DISPLAY_TYPE 2            // 1 = 128x64 pixel OLED Display angeschlossen über I2C
                                  // 2 = 128x64 pixel OLED Display angeschlossen über SPI
                                  // 3 = 320x240 pixel TFT Display ST7789 angeschlossen über SPI
                                  // 99 = Oled über I2C und TFT über SPI für development
#define OTA 1                     // 0 = OTA Uptade ausgeschalten
                                  // 1 = OTA Update eingeschalten
#define DREHTELLER 1              // 0 = kein Drehteller
                                  // 1 = Drehteller vorhanden
#define CHANGE_MAC_ADDRESS_HM 1   // 0 = behalte die Orginale Mac Adresse
                                  // 1 = Wechsle die Mac Adresse auf {0x74, 0x00, 0x00, 0x00, 0x00, 0x01}
#define CHANGE_MAC_ADDRESS_TT 1   // 0 = Mac Adresse vom Drehteller ist die orginale
                                  // 1 = Mac Adresse vom Drehteller wurde gewechselt auf {0x74, 0x00, 0x00, 0x00, 0x00, 0x02}
//#define FEHLERKORREKTUR_WAAGE   // falls Gewichtssprünge auftreten, können diese hier abgefangen werden
                                  // Achtung, kann den Wägeprozess verlangsamen. Vorher Hardware prüfen.
//#define QUETSCHHAHN_LINKS       // Servo invertieren, falls der Quetschhahn von links geöffnet wird. Mindestens ein Exemplar bekannt

//
// Ende Benutzereinstellungen!
// 
```
