# !!! Entwickler Version NICHT GETESTET !!!
# HaniMandlWroom
HaniMandel für das Node MCU ESP32 38Pin Evaluation Board<br>

![HaniMandl_1](./Hardware/Gehäuse/Bilder/HaniMandl_1.jpg)

Dies ist eine abgeänderte Version von dem Orginal HaniMandl Projekt. Die Idee und Veröffentlichung wurde auf der Facebook-Gruppe ["Imkerei und Technik. Eigenbau"](https://www.facebook.com/groups/139671009967454) veröffentlicht.
Als Grundlage für dieses Projekt bediente ich mich am [develop Branch (V0.2.13)](https://github.com/ClemensGruber/hani-mandl/tree/develop) von Clemens Gruber.

## Menüsprache
Die Menüsprache kann im Setupmenü gewechselt werden. Momentan gibt es Deutsch und Englisch. Weitere Sprachdateien können selbst hinzugefügt werden. Wer eine neue Sprachdatei macht, bitte mir senden, damit alle etwas davon haben.

## OTA Update
Neu kann man auch über WLAN eine neue Firmware aufspielen.
Diese Funktion wurde mit Hilfe von ElegantOTA implementiert: https://github.com/ayushsharma82/ElegantOTA
Die SSID und das Passwort wird in dem File ./src/Resources/wifi.h eingetraden.
Um das OTA zu aktivieren betätigt die Start Taste währent Ihr im Setupmenü Hauptbildschirm seid. Danach verbindet sich der HM mit dem WLAN und wenn es klappt wird die IP-Adresse angeteigt.
Diese könnt Ihr nun an eurem Computer eingeben (z.B. http://192.168.76.232/update) und danach ein Beliebiges Binary File über den Browser flaschen.  

![HaniMandl_1](./Hardware/Gehäuse/Bilder/OTA_1.jpg)

## Automatischer Volumenstrom beim Abfüllen
Unter Setup - Automatik kann neu ein Automatisch geregelter Volumenstrom eingestellt werden. Dies kann Hilfreich sein, wenn sehr flüssiger Honig abgefüllt wird.

## Drehteller (Honig schlepp Esel)
Integation von dem Honig schlepp Esel. Dieser kann aktiviert werden in den Usereinstellungen unter #define DREHTELLER. Ist dieser Wert auf 1, wird im Setup ein neues Menü Drehteller aufgeführt. Ist der Wert 0, so sollte sich der HaniMandel gleich verhalten wie wir den kennen.

## Bekannte Käferlein
Beim ersten Start ist der maximale Öffnungswinkel im Manuel und Automatik Betrieb auf 0. Dieser muss dann mit der Rotary eingestellt werden. Danach wird der Wert gespeichert. Ob das in der Alten Version auch schon so war, kann ich aus dem Kopf nicht sagen :-)

## Changelog
W.0.2
- OTA update implementiert
- Automatischer Volumenstrom beim Abfüllen implementiert (Nicht von mir getestet)
- Der HM kann nun auch über eine Sprachdatei die Menüsprache ändern (implementiert wurde bis jetzt Deutsch und Englisch)

W.0.3
- Integration vom Drehteller

## Copyright

Die Software unterliegt dem gleichen Coryright und Anforderungen wie das Hauptprojekt.
Die Hardware (Gehäuse und PCB) sind von mir gemacht worden. Ich verzichte auf ein Copyriht für den Eigengebrauch. Auch wenn ich den Hanimandel nicht kommerziell vertreibe, gehören mir die Daten vom Gehäuse und PCB und diese dürfen nicht ohne meine Zustimmung kommerziell benützt werden.

## Benutzte Programme und Lizensen

Gehäuse: Ondsel/FreeCAD (keine Lizenz notwendig)<br>
PCB: DipTrace (kommerzielle Lizenz)

## Drehteller addon für den HaniMandel (HonigSchleppElesl)

zu finden unter: https://github.com/RolandRust/HonigSchleppElesl/

## Appell an dich

Bringt nix :-)
