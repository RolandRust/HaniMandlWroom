# Nicht getestete Version!!!

# HaniMandlWroom
HaniMandel für das Node MCU ESP32 38Pin Evaluation Board<br>

## Kompiliert nur mit Visual Studio Code. Die Arduino IDE wurde abgeschossen :-)

## Dev Version 
Dies ist eine Developer Version. Es sind nicht alle funktionen getestet und es wurde noch nie Abgefüllt mit dieser Version.
Habe ziemlich viel geändert. Ist also warscheidnlich das es noch Bugs drin hat.

## Changelog
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

## Bekante Käfer im Code
  - Wenn in den Automatik Modus geschalten wird und auf der Waage was draufsteht, wird das Ausgewählte Glas nicht angezeigt. Ein leeren der Wage behabt das Problem.
  - Wenn in dem Automatik Mode bei eingeschaltetem Automatik Mode die Stop taste gedrückt wird, bleibt beim TFT die angezeigte Farbe bestehen.
  - Display TFT für den Drehteller (Honig schlepp Esel) ist noch nicht gemacht. 

Die werden aber noch behoben und stören die Funtion nicht.


