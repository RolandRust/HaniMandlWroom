//display.cpp

/*TL_DATUM = Top left
TC_DATUM = Top centre
TR_DATUM = Top right
ML_DATUM = Middle left
MC_DATUM = Middle centre
MR_DATUM = Middle right
BL_DATUM = Bottom left
BC_DATUM = Bottom centre
BR_DATUM = Bottom right*/

//Automatik ist was komisch mit dem no tara. wird nicht angezeigt.

// addr2line -e /home/kroki/RiouxSVN/Software/ESP32/HaniMandl/.pio/build/esp32dev/firmware.elf 0x40086ced

#include <U8g2lib.h>
#include <TFT_eSPI.h>
#include <pgmspace.h>
#include <WiFi.h>

#include "display.h"
#include "variables.h"
#include "./Resources/resources.h"

String x = "";

// Fonts
#if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
  const char* font_used[2] = {"FreeSans", "PunkMono"};
  //Icons
  #include "./Fonts/icon10pt.h"
  #include "./Fonts/icon26pt.h"
  //Free Sans
  #include "./Fonts/FreeSansBold28pt.h"
  #include "./Fonts/FreeSansBold26pt.h"
  #include "./Fonts/FreeSansBold20pt.h"
  #include "./Fonts/FreeSansBold18pt.h"
  #include "./Fonts/FreeSansBold16pt.h"
  #include "./Fonts/FreeSansBold14pt.h"
  #include "./Fonts/FreeSansBold12pt.h"
  #include "./Fonts/FreeSansBold10pt.h"
  #include "./Fonts/FreeSansBold8pt.h"
  #include "./Fonts/FreeSansBold6pt.h"
  #include "./Fonts/FreeSans14pt.h"
  #include "./Fonts/FreeSans8pt.h"
  #include "./Fonts/FreeSans6pt.h"
  //PunkMono
  #include "./Fonts/punk_mono_bold28pt.h"
  #include "./Fonts/punk_mono_bold26pt.h"
  #include "./Fonts/punk_mono_bold20pt.h"
  #include "./Fonts/punk_mono_bold18pt.h"
  #include "./Fonts/punk_mono_bold16pt.h"
  #include "./Fonts/punk_mono_bold14pt.h"
  #include "./Fonts/punk_mono_bold12pt.h"
  #include "./Fonts/punk_mono_bold10pt.h"
  #include "./Fonts/punk_mono_bold8pt.h"
  #include "./Fonts/punk_mono_bold6pt.h"
  #include "./Fonts/punk_mono_thin14pt.h"
  #include "./Fonts/punk_mono_thin8pt.h"
  #include "./Fonts/punk_mono_thin6pt.h"
  const GFXfont* const F06B[] PROGMEM = {&FreeSansBold6pt , &punk_mono_bold6pt};
  const GFXfont* const F08B[] PROGMEM = {&FreeSansBold8pt , &punk_mono_bold8pt};
  const GFXfont* const F10B[] PROGMEM = {&FreeSansBold10pt, &punk_mono_bold10pt};
  const GFXfont* const F12B[] PROGMEM = {&FreeSansBold12pt, &punk_mono_bold12pt};
  const GFXfont* const F14B[] PROGMEM = {&FreeSansBold14pt, &punk_mono_bold14pt};
  const GFXfont* const F16B[] PROGMEM = {&FreeSansBold16pt, &punk_mono_bold16pt};
  const GFXfont* const F18B[] PROGMEM = {&FreeSansBold18pt, &punk_mono_bold18pt};
  const GFXfont* const F20B[] PROGMEM = {&FreeSansBold20pt, &punk_mono_bold20pt};
  const GFXfont* const F26B[] PROGMEM = {&FreeSansBold26pt, &punk_mono_bold26pt};
  const GFXfont* const F28B[] PROGMEM = {&FreeSansBold28pt, &punk_mono_bold28pt};
  const GFXfont* const F06[] PROGMEM = {&FreeSans6pt      , &punk_mono_thin6pt};
  const GFXfont* const F08[] PROGMEM = {&FreeSans8pt      , &punk_mono_thin8pt};
  const GFXfont* const F14[] PROGMEM = {&FreeSans14pt     , &punk_mono_thin14pt};
#endif

//Offset
int x_pos;
int y_pos;
#if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
  int y_offset = 8;
#endif
#if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
  uint16_t x_y_h_w_b[5];
  int y_offset_tft = 28;
  uint16_t pos_y_weight = 240;
  int longest_string_1;
  int longest_string_2;
  char del_text_1[30] = "";
  char del_text_2[30] = "";
  char del_text_3[30] = "";
#endif

//Logos
#if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
  #if USER == 2
    #include "./Logos/LogoGeroldOLED.h"
  #elif USER == 3
    #include "./Logos/LogoRoliOLED.h"
  #else
    #include "./Logos/LogoBieneOLED.h"
  #endif
#endif
#if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
  #if USER == 2
    #include "./Logos/LogoGeroldTFT.h"
  #elif USER == 3
    #include "./Logos/LogoRoliTFT.h"
  #else
    #include "./Logos/LogoBieneTFT.h"
  #endif
#endif

#if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
  unsigned long  BACKGROUND;
  unsigned long  TEXT;
  unsigned long  MENU_POS1;
  unsigned long  MENU_POS2;
  unsigned long  MENU_POS3;
  unsigned long  MENU_POS4;
  unsigned long  MARKER;
  int cali_weight_old;
#endif
//Text
char output[30];


#if DISPLAY_TYPE == 991
    #if HARDWARE_LEVEL == 1
        U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* clock=*/ 22, /* data=*/ 21, /* reset=*/ U8X8_PIN_NONE);
    #elif HARDWARE_LEVEL == 2
        U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ 21, /* clock=*/ 18, /* data=*/ 17);
    #elif HARDWARE_LEVEL == 3
        U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ 21, /* clock=*/ 19, /* data=*/ 20);
    #endif
#elif  DISPLAY_TYPE == 992
    U8G2_SSD1309_128X64_NONAME0_F_4W_SW_SPI u8g2(U8G2_R0, /* clock=*/ 18, /* data=*/ 23, /* cs=*/ 5, /* dc=*/ 13, /* reset=*/ 14);
#elif  DISPLAY_TYPE == 993
    TFT_eSPI tft = TFT_eSPI();
#elif  DISPLAY_TYPE == 999
    U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* clock=*/ 22, /* data=*/ 21, /* reset=*/ U8X8_PIN_NONE);
    TFT_eSPI tft = TFT_eSPI();
#endif

#if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
    void tft_colors(void);
    void tft_marker(void);
#endif

//Initialisierung von einigen Funktionen
int CenterPosX(const char a[], float font_width, int display_widht);
int get_length(const char a[]);
int StringLenght(String a);
void calculateStringMetrics(const GFXfont *font, const char *datum, const char *text, int x, int y, uint16_t res[4]);
    
HM_Display::HM_Display() {}

void HM_Display::begin() {
    #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
        u8g2.setBusClock(800000);
        u8g2.begin();
        u8g2.enableUTF8Print();
    #endif
    #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
        tft_colors();
        tft_marker();
        tft_colors();
        tft_marker();
        tft.begin();
        tft.setRotation(3);
        tft.fillScreen(BACKGROUND);
    #endif
}

void HM_Display::print_logo() {
  #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
    u8g2.clearBuffer();
  #endif
  #if USER == 2
    #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
        u8g2.clearBuffer();
        u8g2.drawXBM(0,0,128,64,LogoGeroldOLED);
        u8g2.sendBuffer();
    #endif
    #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
      tft.drawXBitmap(0, 0, LogoGeroldTFT, 320, 240, TEXT, BACKGROUND);
    #endif
  #elif USER == 3
    #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
      u8g2.clearBuffer();
      u8g2.drawXBM(0,0,128,64,LogoRoliOLED);
      u8g2.setFont(u8g2_font_courB14_tf);
      u8g2.setCursor(30, 27);    u8g2.print("HANI");
      u8g2.setCursor(40, 43);    u8g2.print("MANDL");
      u8g2.setFont(u8g2_font_courB08_tf);
      u8g2.setCursor(95, 64);    u8g2.print(VERSION_STRING);
      u8g2.sendBuffer();
    #endif
      #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
        tft.drawXBitmap(0, 0, LogoRoliTFT, 320, 240, TFT_YELLOW, BACKGROUND);
        tft.setTextColor(TFT_YELLOW);
        tft.setFreeFont(F12B[font_typ]);
        tft.setTextDatum(BL_DATUM);
        tft.setCursor(228, 220);
        tft.print(VERSION_STRING);
    #endif
  #else
    #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
      u8g2.clearBuffer();
      u8g2.drawXBM(0,0,80,64,LogoBieneOLED);
      u8g2.setFont(u8g2_font_courB14_tf);
      u8g2.setCursor(85, 27);    u8g2.print("HANI");
      u8g2.setCursor(75, 43);    u8g2.print("MANDL");
      u8g2.setFont(u8g2_font_courB08_tf);
      u8g2.setCursor(77, 64);    u8g2.print(VERSION_STRING);
      u8g2.sendBuffer();
    #endif
    #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
      tft.fillScreen(BACKGROUND);
      tft.drawXBitmap(60, 10, LogoBieneTFT, 200, 160, TEXT, BACKGROUND);
      tft.setTextColor(TFT_YELLOW);
      tft.setFreeFont(F26B[font_typ]);
      tft.setTextDatum(BL_DATUM);
      tft.setTextColor(TEXT);
      tft.setCursor(5, 190);
      tft.print("HANI");
      sprintf(output,"MANDL");
      tft.setCursor(315 - tft.textWidth(output), 235);
      tft.print(output);
      tft.setFreeFont(F12B[font_typ]); 
      tft.setCursor(315 - tft.textWidth(VERSION_STRING), 190);
      tft.print(VERSION_STRING);
    #endif
  #endif
}

void HM_Display::print_credits() {
  #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
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
  #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
    int offset = 20;
    tft.fillScreen(BACKGROUND);
    tft.setTextDatum(BL_DATUM);
    tft.setTextColor(TEXT);
    tft.setFreeFont(F10B[font_typ]);
    tft.setCursor(5, 1 * offset);
    tft.print("Idee:");
    tft.setCursor(90, 1 * offset);
    tft.print("M. Vasterling");
    tft.setCursor(5, 2 * offset);
    tft.print("Code:");
    tft.setCursor(90, 2 * offset);
    tft.print("M. Vasterling");
    tft.setCursor(90, 3 * offset);
    tft.print("M. Wetzel");
    tft.setCursor(90, 4 * offset);
    tft.print("C. Gruber");
    tft.setCursor(90, 5 * offset);
    tft.print("A. Holzhammer");
    tft.setCursor(90, 6 * offset);
    tft.print("M. Junker");
    tft.setCursor(90, 7 * offset);
    tft.print("J. Kuder");
    tft.setCursor(90, 8 * offset);
    tft.print("J. Bruker");
    tft.setCursor(90, 9 * offset);
    tft.print("R. Rust");
    tft.setCursor(90, 10 * offset);
    tft.print("A. Pfaff");
  #endif
}

void HM_Display::not_calibrated() {
  #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_courB12_tf);
    sprintf(output,NOT[lingo]);
    x_pos = CenterPosX(output, 10, 128);
    u8g2.setCursor(x_pos, 30); u8g2.print(output);
    sprintf(output,CALIBRATED[lingo]);
    x_pos = CenterPosX(output, 10, 128);
    u8g2.setCursor(x_pos, 46); u8g2.print(output);
    u8g2.sendBuffer();
  #endif
  #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
    float a = 0.3;
    tft.fillScreen(BACKGROUND);
    tft.setTextColor(TFT_RED);
    tft.setFreeFont(F28B[font_typ]);
    sprintf(output, "%s", NOT[lingo]);
    calculateStringMetrics(F28B[font_typ], "MC", output, tft.width() / 2, tft.height() / 2, x_y_h_w_b);
    tft.setCursor(x_y_h_w_b[0], x_y_h_w_b[1] - x_y_h_w_b[2]/2 - x_y_h_w_b[2] * a);
    tft.print(output);
    sprintf(output, "%s", CALIBRATED[lingo]);
    calculateStringMetrics(F28B[font_typ], "MC", output, tft.width() / 2, tft.height() / 2, x_y_h_w_b);
    tft.setCursor(x_y_h_w_b[0], x_y_h_w_b[1] + x_y_h_w_b[2]/2 + x_y_h_w_b[2] * a);
    tft.print(output);
  #endif
}

void HM_Display::no_scale() {
  #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_courB24_tf);
    sprintf(output,NO[lingo]);
    x_pos = CenterPosX(output, 20, 128);
    u8g2.setCursor( x_pos, 24); u8g2.print(output);
    sprintf(output,SCALE[lingo]);
    x_pos = CenterPosX(output, 20, 128);
    u8g2.setCursor( x_pos, 56);  u8g2.print(output);
    u8g2.sendBuffer();
  #endif
  #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
    float a = 0.3;
    tft.fillScreen(BACKGROUND);
    tft.setTextColor(TFT_RED);
    tft.setFreeFont(F28B[font_typ]);
    sprintf(output, "%s", NO[lingo]);
    calculateStringMetrics(F28B[font_typ], "MC", output, tft.width() / 2, tft.height() / 2, x_y_h_w_b);
    tft.setCursor(x_y_h_w_b[0], x_y_h_w_b[1] - x_y_h_w_b[2]/2 - x_y_h_w_b[2] * a);
    tft.print(output);
    sprintf(output, "%s", SCALE[lingo]);
    calculateStringMetrics(F28B[font_typ], "MC", output, tft.width() / 2, tft.height() / 2, x_y_h_w_b);
    tft.setCursor(x_y_h_w_b[0], x_y_h_w_b[1] + x_y_h_w_b[2]/2 + x_y_h_w_b[2] * a);
    tft.print(output);
  #endif
}

void HM_Display::empty_the_scale() {
  #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_courB14_tf);
    sprintf(output,EMPTY[lingo]);
    x_pos = CenterPosX(output, 11, 128);
    u8g2.setCursor( x_pos, 28); u8g2.print(output);
    sprintf(output,THE_SCALE[lingo]);
    x_pos = CenterPosX(output, 11, 128);
    u8g2.setCursor(x_pos, 52); u8g2.print(output);
    u8g2.sendBuffer();
  #endif
  #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
    float a = 0.3;
    tft.fillScreen(BACKGROUND);
    tft.setTextColor(TFT_RED);
    tft.setFreeFont(F28B[font_typ]);
    sprintf(output, "%s", EMPTY[lingo]);
    calculateStringMetrics(F28B[font_typ], "MC", output, tft.width() / 2, tft.height() / 2, x_y_h_w_b);
    tft.setCursor(x_y_h_w_b[0], x_y_h_w_b[1] - x_y_h_w_b[2]/2 - x_y_h_w_b[2] * a);
    tft.print(output);
    sprintf(output, "%s", THE_SCALE[lingo]);
    calculateStringMetrics(F28B[font_typ], "MC", output, tft.width() / 2, tft.height() / 2, x_y_h_w_b);
    tft.setCursor(x_y_h_w_b[0], x_y_h_w_b[1] + x_y_h_w_b[2]/2 + x_y_h_w_b[2] * a);
    tft.print(output);
  #endif
}

void HM_Display::process_setup(const char *menuitems[], int posmenu[], int menuitems_number, int menuitem, int menuitem_old) {
  #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
    if (menuitem != menuitem_old) {
      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_courB08_tf);
      if (menu_rotation == 1) {
        x_pos = CenterPosX(menuitems[posmenu[1]], 6, 128); //ein Zeichen ist etwa 6 Pixel breit
        u8g2.setCursor(x_pos,12);   
        u8g2.print(menuitems[posmenu[1]]);
      }
      else {
        x_pos = CenterPosX(menuitems[posmenu[menuitems_number-1]], 6, 128); //ein Zeichen ist etwa 6 Pixel breit
        u8g2.setCursor(x_pos,12);  
        u8g2.print(menuitems[posmenu[menuitems_number-1]]);
      }
      u8g2.drawLine(0, 18, 128, 18);
      u8g2.setFont(u8g2_font_courB12_tf);
      x_pos = CenterPosX(menuitems[posmenu[0]], 10, 128); //ein Zeichen ist 10 Pixel breit
      u8g2.setCursor(x_pos, 38);   
      u8g2.print(menuitems[posmenu[0]]);
      u8g2.drawLine(0, 47, 128, 47);
      u8g2.setFont(u8g2_font_courB08_tf);
      if (menu_rotation == 1) {
        x_pos = CenterPosX(menuitems[posmenu[menuitems_number-1]], 6, 128); //ein Zeichen ist etwa 6 Pixel breit
        u8g2.setCursor(x_pos,62);  
        u8g2.print(menuitems[posmenu[menuitems_number-1]]);
      }
      else {
        x_pos = CenterPosX(menuitems[posmenu[1]], 6, 128); //ein Zeichen ist etwa 6 Pixel breit
        u8g2.setCursor(x_pos,62);   
        u8g2.print(menuitems[posmenu[1]]);
      }
      u8g2.sendBuffer();
    }
  #endif
  #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
    if (menuitem != menuitem_old) {
      int offset_line = 24;
      int offset_menue[4] = {35,65,90,110};
      long unsigned int menue_color[4] = {MENU_POS1, MENU_POS2, MENU_POS3, MENU_POS4};
      const GFXfont* menue_text_size[4] = {F14B[font_typ], F12B[font_typ], F08B[font_typ], F06B[font_typ]};
      x_pos = tft.width() / 2;
      y_pos = tft.height() / 2;
      tft.fillRect(0, 0, tft.width(), y_pos - offset_line, BACKGROUND);                                             //TOP
      tft.fillRect(0, y_pos + offset_line + 1, tft.width(), tft.height() - (y_pos + offset_line + 1), BACKGROUND);  //BOTTOM
      tft.fillRect(0, y_pos - offset_line + 1, tft.width(), 2 * offset_line - 1, BACKGROUND);                       //MIDDLE
      tft.drawLine(0, y_pos - offset_line, tft.width(), y_pos - offset_line, TEXT);
      tft.drawLine(0, y_pos + offset_line, tft.width(), y_pos + offset_line, TEXT);
      tft.setTextColor(TEXT);
      tft.setFreeFont(F18B[font_typ]);
      sprintf(output, "%s", menuitems[posmenu[0]]);
      calculateStringMetrics(F18B[font_typ], "MC", output, x_pos, y_pos, x_y_h_w_b);
      tft.setCursor(x_y_h_w_b[0], x_y_h_w_b[1]);
      tft.print(output);
      for (int a = 0; a < 4; a++) {
        calculateStringMetrics(menue_text_size[a], "MC", "A", x_pos,  y_pos, x_y_h_w_b);
        int font_h = x_y_h_w_b[2];
        tft.setTextColor(menue_color[a]);
        tft.setFreeFont(menue_text_size[a]);
        sprintf(output, "%s", menuitems[posmenu[(menuitems_number-1-a)]]);
        calculateStringMetrics(menue_text_size[a], "MC", output, x_pos,  y_pos, x_y_h_w_b);
        if (menu_rotation == 1) {
          tft.setCursor(x_y_h_w_b[0], y_pos + offset_menue[a] + font_h);
        }
        else {
          tft.setCursor(x_y_h_w_b[0], y_pos - offset_menue[a]);
        }
        tft.print(output);
        sprintf(output, "%s", menuitems[posmenu[1+a]]);
        calculateStringMetrics(menue_text_size[a], "MC", output, x_pos,  y_pos, x_y_h_w_b);
        if (menu_rotation == 1) {
          tft.setCursor(x_y_h_w_b[0], y_pos - offset_menue[a]);
        }
        else {
          tft.setCursor(x_y_h_w_b[0], y_pos + offset_menue[a] + font_h);
        }
        tft.print(output);
      }
    }
  #endif
}

void HM_Display::setup_parameter_oled(bool change_value, int menuitem, int menuitem_used) {
  #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
    u8g2.setFont(u8g2_font_courB08_tf);
    u8g2.clearBuffer();
    u8g2.setCursor(10, 1 * y_offset);
    u8g2.print(BUZZER[lingo]);
    u8g2.setCursor(95, 1 * y_offset);
    sprintf(output,"%5s", (buzzermode==0?OFF[lingo]:ON[lingo]));
    u8g2.print(output);
    u8g2.setCursor(10, 2 * y_offset);
    u8g2.print(LED_1[lingo]);
    u8g2.setCursor(95, 2 * y_offset);
    sprintf(output,"%5s", (ledmode==0?OFF[lingo]:ON[lingo]));
    u8g2.print(output);
    u8g2.setCursor(10, 3 * y_offset);
    u8g2.print(SHOW_LOGO[lingo]);
    u8g2.setCursor(95, 3 * y_offset);
    sprintf(output,"%5s", (showlogo==0?OFF[lingo]:ON[lingo]));
    u8g2.print(output);
    u8g2.setCursor(10, 4 * y_offset);
    u8g2.print(SHOW_CREDITS[lingo]);
    u8g2.setCursor(95, 4 * y_offset);
    sprintf(output,"%5s", (showcredits==0?OFF[lingo]:ON[lingo]));
    u8g2.print(output);
    u8g2.setCursor(10, 5 * y_offset);
    u8g2.print(CHANGE_ROTATION[lingo]);
    u8g2.setCursor(95, 5 * y_offset);
    sprintf(output,"%5s", (menu_rotation==0?OFF[lingo]:ON[lingo]));
    u8g2.print(output);
    u8g2.setCursor(10, (7 * y_offset) + 5);
    u8g2.print(SAVE[lingo]);
    // Positionsanzeige im Menu. "*" wenn nicht ausgewählt, Pfeil wenn ausgewählt
    if (change_value == false && menuitem < menuitem_used) {
      u8g2.setCursor(1, 10+((menuitem)*y_offset)); u8g2.print("*");
    }
    else if (change_value == true && menuitem < menuitem_used) {
      u8g2.setCursor(1, 8+((menuitem)*y_offset)); u8g2.print("-");
    }
    else if (change_value == false && menuitem == menuitem_used) {
      u8g2.setCursor(1, 10+(6*y_offset) + 5); u8g2.print("*");
    }
    u8g2.sendBuffer();
  #endif
}

void HM_Display::setup_parameter_oled_exit(int menuitem) {
  #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
    u8g2.setCursor(111, (7 * y_offset) + 5);
    u8g2.print("OK");
    u8g2.sendBuffer();
  #endif
}

void HM_Display::setup_parameter_tft(int menuitems_number, bool change_value, const char *menuitems[]) {
  #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
    if(change_scheme) {
      tft.fillScreen(BACKGROUND);
      tft.setTextColor(TEXT, BACKGROUND);
      tft.setFreeFont(F16B[font_typ]);
      sprintf(output, "%s", PARAMETER[lingo]);
      calculateStringMetrics(F16B[font_typ], "MC", output, tft.width()/2, 15, x_y_h_w_b);
      tft.setCursor(x_y_h_w_b[0], x_y_h_w_b[1]);
      tft.print(output);
      tft.drawLine(0, 30, tft.width(), 30, TEXT);
      if (menuitems_number > 6) {
        tft.drawLine(0, 207, tft.width(), 207, TEXT);
      }
      change_scheme = false;
      change_marker = false;  //Marker ist erst auf der zweiten Seite
    }
    if (((pos >= 6 and pos != menuitems_number - 1) or (pos_old == 6 and pos == 5)) and pos_old != pos) {
      tft.fillRect(0, 31, 320, 175, BACKGROUND);
      pos_old = pos;
    }
    String del_text = "";
    char del_text_1[30] = "";
    int i = 0;
    int j = 6;
    int offset = 0;
    if (pos >= 6 and pos != menuitems_number - 1) {
      i = pos - 5;
      j = j + i;
      offset = i;
      change_marker = true;
    }
    else if (pos == menuitems_number - 1) {
      i = pos - 6;              //nicht gut, aber verschieben wir das Problem mal in die nächste version wenn dieses Menü ändert :-)
      j = menuitems_number - 1;
      offset = i;
    }
    tft.setFreeFont(F12B[font_typ]);
    tft.setTextDatum(BL_DATUM);
    while(i <= j) {
      tft.setTextColor(TEXT);
      if (i == pos and change_value == false) {
        tft.setTextColor(MARKER);
      }
      if (i < j) {
        tft.setCursor(5, 27+((i+1-offset) * y_offset_tft));
        tft.print(menuitems[i]);
        tft.setTextDatum(BR_DATUM);
        tft.setTextColor(BACKGROUND);
        switch (i) {
          case 0: sprintf(output,"%s", buzzermode==false?OFF[lingo]:ON[lingo]);
                  if (value_old != buzzermode and change_value == true and i == pos) {
                    del_text = buzzermode==true?OFF[lingo]:ON[lingo];
                    tft.setCursor(tft.width() - 5 - tft.textWidth(del_text), 27+((i+1) * y_offset_tft));
                    tft.print(del_text);
                    value_old = buzzermode;
                  }
                  break;
          case 1: sprintf(output,"%s", ledmode==false?OFF[lingo]:ON[lingo]);
                  if (value_old != ledmode and change_value == true and i == pos) {
                    del_text = ledmode==true?OFF[lingo]:ON[lingo];
                    tft.setCursor(tft.width() - 5 - tft.textWidth(del_text), 27+((i+1) * y_offset_tft));
                    tft.print(del_text);
                    value_old = ledmode;
                  }
                  break;
          case 2: sprintf(output,"%s", showlogo==false?OFF[lingo]:ON[lingo]);
                  if (value_old != showlogo and change_value == true and i == pos) {
                    del_text = showlogo==true?OFF[lingo]:ON[lingo];
                    tft.setCursor(tft.width() - 5 - tft.textWidth(del_text), 27+((i+1) * y_offset_tft));
                    tft.print(del_text);
                    value_old = showlogo;
                  }
                  break;
          case 3: sprintf(output,"%s", showcredits==false?OFF[lingo]:ON[lingo]);
                  if (value_old != showcredits and change_value == true and i == pos) {
                    del_text = showcredits==true?OFF[lingo]:ON[lingo];
                    tft.setCursor(tft.width() - 5 - tft.textWidth(del_text), 27+((i+1) * y_offset_tft));
                    tft.print(del_text);
                    value_old = showcredits;
                  }
                  break;
          case 4: sprintf(output,"%s", menu_rotation==false?OFF[lingo]:ON[lingo]);
                  if (value_old != menu_rotation and change_value == true and i == pos) {
                    del_text = menu_rotation==true?OFF[lingo]:ON[lingo];
                    tft.setCursor(tft.width() - 5 - tft.textWidth(del_text), 27+((i+1) * y_offset_tft));
                    tft.print(del_text);
                    value_old = menu_rotation;
                  }
                  break;
          case 5: sprintf(output,"%s", color_scheme==false?DARK[lingo]:LIGHT[lingo]);
                  if (value_old != color_scheme and change_value == true and i == pos) {
                    del_text = color_scheme==true?DARK[lingo]:LIGHT[lingo];
                    tft.setCursor(tft.width() - 5 - tft.textWidth(del_text), 27+((i+1) * y_offset_tft));
                    tft.print(del_text);
                    change_scheme = true;
                    value_old = color_scheme;
                    tft_colors();
                  }
                  break;
          case 6: sprintf(output,"");
                  if (value_old != color_marker and change_value == true and i == pos) {
                    change_marker = true;
                    value_old = color_marker;
                    tft_marker();
                  }
                  break;
          case 7: sprintf(output,"%s", font_used[font_typ]);
                  if (value_old != font_typ and change_value == true and i == pos) {
                    if (value_old >= 0) {change_scheme = true;}
                    value_old = font_typ;
                  }
                  break;
        }
        if (change_marker) {
          tft.fillRect(265, 27+((6 - (pos - 6)) * y_offset_tft)-17, 45, 21, MARKER);
          change_marker = false;
        }
        if (i == pos) {
          tft.setTextColor(MARKER);
        }
        else {
          tft.setTextColor(TEXT);
        }
        tft.setCursor(tft.width() - 5 - tft.textWidth(output), 27+((i+1-offset) * y_offset_tft));
        tft.print(output);
      }
      else {    //Save
        tft.setCursor(5, 35+(7 * y_offset_tft));
        tft.print(menuitems[menuitems_number - 1]);
      }
      i++;
    }
  #endif
}

void HM_Display::setup_parameter_tft_exit() {
  #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
    tft.setCursor(tft.width() - 5 - tft.textWidth("OK"), 35+(7 * y_offset_tft));
    tft.print("OK");
  #endif
}

void HM_Display::setup_language_oled(int pos, int menuitems_number, const char *menuitems[]) {
  #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
    u8g2.setFont(u8g2_font_courB08_tf);
    u8g2.clearBuffer();
    int i = 0;
    int j;
    if (menuitems_number < 6) {j = menuitems_number;}
    else {j = 6;}
    if (pos >= 6 and pos != menuitems_number) {i = pos - 5; j = pos + 1;}
    else if (pos == menuitems_number and menuitems_number > 6) {i = pos - 6; j = pos;}
    for (i; i < j; i++) {
      if (i == lingo) {
        sprintf(output,"[ %s ]", menuitems[i]);
      }
      else {
        sprintf(output,"%s", menuitems[i]);
      }
      if (menuitems_number < 6) {
        u8g2.setCursor(10, (i + 1) * y_offset);
      }
      else {
        u8g2.setCursor(10, (i + 7 - j) * y_offset);
      }
      u8g2.print(output);
    }
    if (menuitems_number > 6) {
      u8g2.drawLine(0, 52, 128, 52);
    }
    u8g2.setCursor(10, (7 * y_offset) + 5);
    u8g2.print(SAVE[lingo]);
    if ((pos < 6 and menuitems_number > 6) or (menuitems_number <= 6 and pos != menuitems_number)) {
      u8g2.setCursor(1, 10+(pos*y_offset)); u8g2.print("*");
    }
    else if (pos < menuitems_number  and menuitems_number > 6) {
      u8g2.setCursor(1, 10+(5*y_offset)); u8g2.print("*");
    }
    else {
      u8g2.setCursor(1, 10+(6*y_offset) + 5); u8g2.print("*");
    }
    u8g2.sendBuffer();
  #endif
}

void HM_Display::setup_language_oled_exit(){
  #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
    u8g2.setCursor(111, (7*y_offset) + 5); u8g2.print("OK");
    u8g2.sendBuffer();
  #endif
}

void HM_Display::setup_language_tft(int pos, int menuitems_number, const char *menuitems[]) {
  #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
    if (change_scheme) {
      tft.fillScreen(BACKGROUND);
      tft.setTextColor(TEXT);
      tft.setFreeFont(F16B[font_typ]);
      sprintf(output, "%s", LANGUAGE1[lingo]);
      calculateStringMetrics(F16B[font_typ], "MC", output, tft.width()/2, 15, x_y_h_w_b);
      tft.setCursor(x_y_h_w_b[0], x_y_h_w_b[1]);
      tft.print(output);
      tft.drawLine(0, 30, tft.width(), 30, TEXT);
      if (menuitems_number > 6) {
        tft.drawLine(0, 207, 320, 207, TEXT);
      }
      change_scheme = false;
    }
    if (((pos >= 6 and pos != menuitems_number) or (pos_old == 6 and pos == 5)) and pos_old != pos) {
      tft.fillRect(0, 31, 320, 175, BACKGROUND);
      pos_old = pos;
    }
    int i = 0;
    int j;
    if (menuitems_number < 6) {j = menuitems_number;}
    else {j = 6;}
    if (pos >= 6 and pos != menuitems_number) {i = pos - 5; j = pos + 1;}
    else if (pos == menuitems_number and menuitems_number > 6) {i = pos - 6; j = pos; }
    tft.setFreeFont(F12B[font_typ]);
    tft.setTextDatum(BR_DATUM);
    for (i; i < j; i++) {
      tft.setTextColor(TEXT);
      if (i == pos) {tft.setTextColor(MARKER);}
      if (i == lingo) {
        sprintf(output,"[%s]", menuitems[i]);
      }
      else {
        sprintf(output,"%s", menuitems[i]);
      }
      if (menuitems_number < 6) {
        tft.setCursor(5, 27 + ((i + 1) * y_offset_tft));
      }
      else {
        tft.setCursor(5, 27 + ((i + 7 - j) * y_offset_tft));
      }
      tft.println(output);
    }
    tft.setTextColor(TEXT);
    if (pos == menuitems_number) {tft.setTextColor(MARKER);}
    tft.setCursor(5, 35+(7 * y_offset_tft));
    tft.print(SAVE[lingo]);
  #endif
}

void HM_Display::setup_language_tft_exit() {
  #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
    tft.setCursor(tft.width() - 5 - tft.textWidth("OK"), 35+(7 * y_offset_tft));
    tft.print("OK");
  #endif
}

void HM_Display::setup_calibration_oled(int a) {
  #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
    if (a == 1){
      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_courB08_tf);
      u8g2.setCursor(0, 12);    u8g2.print(PLEASE_EMPTY_THE[lingo]);
      u8g2.setCursor(0, 24);    u8g2.print(SCALE_AND_CONFIRM[lingo]);
      u8g2.setCursor(0, 36);    u8g2.print(WITH_OK_1[lingo]);
      u8g2.sendBuffer();
    }
    else if (a == 2) {
      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_courB08_tf);
      u8g2.setCursor(0, 12);
      u8g2.print(PLEASE_SET_UP[lingo]);
      sprintf(output, " %dg", cali_weight);
      u8g2.print(output);
      u8g2.setCursor(0, 24);    u8g2.print(AND_CONFIRM[lingo]);
      u8g2.setCursor(0, 36);    u8g2.print(WITH_OK_2[lingo]);
      u8g2.sendBuffer();
    }
  #endif
}

void HM_Display::setup_calibration_tft(int a){
  #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
    float b = 0.75;
    if (a == 1) {
      tft.fillScreen(BACKGROUND);
      cali_weight_old = -1;
      tft.setTextColor(TEXT);
      tft.setFreeFont(F16B[font_typ]);
      sprintf(output, "%s", CALIBRATION[lingo]);
      calculateStringMetrics(F16B[font_typ], "MC", output, tft.width()/2, 15, x_y_h_w_b);
      tft.setCursor(x_y_h_w_b[0], x_y_h_w_b[1]);
      tft.print(output);
      tft.drawLine(0, 30, tft.width(), 30, TEXT);
      sprintf(output, PLEASE_EMPTY_THE[lingo]);
      calculateStringMetrics(F16B[font_typ], "MC", output, tft.width()/2, tft.height()/2 + 15, x_y_h_w_b);
      tft.setCursor(x_y_h_w_b[0], x_y_h_w_b[1] - 2 * x_y_h_w_b[4] * b); 
      tft.print(output);
      sprintf(output, SCALE_AND_CONFIRM[lingo]);
      calculateStringMetrics(F16B[font_typ], "MC", output, tft.width()/2, tft.height()/2 + 15, x_y_h_w_b);
      tft.setCursor(x_y_h_w_b[0], x_y_h_w_b[1]);
      tft.print(output);
      sprintf(output, WITH_OK_1[lingo]);
      calculateStringMetrics(F16B[font_typ], "MC", output, tft.width()/2, tft.height()/2 + 15, x_y_h_w_b);
      tft.setCursor(x_y_h_w_b[0], x_y_h_w_b[1] + 2 * x_y_h_w_b[4] * b); 
      tft.print(output);
    }
    else if (a == 2) {
      tft.fillScreen(BACKGROUND);
      tft.setTextColor(TEXT);
      tft.setFreeFont(F16B[font_typ]);
      sprintf(output, "%s", CALIBRATION[lingo]);
      calculateStringMetrics(F16B[font_typ], "MC", output, tft.width()/2, 15, x_y_h_w_b);
      tft.setCursor(x_y_h_w_b[0], x_y_h_w_b[1]);
      tft.print(output);
      tft.drawLine(0, 30, tft.width(), 30, TEXT);
    }
    else if (a == 3) {
      if (cali_weight != cali_weight_old) {
        tft.setFreeFont(F16B[font_typ]);
        tft.setTextColor(TEXT);
        sprintf(output, "%s", PLEASE_SET_UP[lingo]);
        calculateStringMetrics(F16B[font_typ], "MC", output, tft.width()/2, tft.height()/2 + 15, x_y_h_w_b);
        tft.setCursor(x_y_h_w_b[0], x_y_h_w_b[1] - 3 * x_y_h_w_b[4] * b); 
        tft.print(output);
        tft.setTextColor(BACKGROUND);
        sprintf(output, "%dg", cali_weight_old);
        calculateStringMetrics(F16B[font_typ], "MC", output, tft.width()/2, tft.height()/2 + 15, x_y_h_w_b);
        tft.setCursor(x_y_h_w_b[0], x_y_h_w_b[1] - x_y_h_w_b[4] * b);
        tft.print(output);
        tft.setTextColor(MARKER);
        sprintf(output, "%dg", cali_weight);
        calculateStringMetrics(F16B[font_typ], "MC", output, tft.width()/2, tft.height()/2 + 15, x_y_h_w_b);
        tft.setCursor(x_y_h_w_b[0], x_y_h_w_b[1] - x_y_h_w_b[4] * b);
        tft.print(output);
        tft.setTextColor(TEXT);
        sprintf(output, "%s", AND_CONFIRM[lingo]);
        calculateStringMetrics(F16B[font_typ], "MC", output, tft.width()/2, tft.height()/2 + 15, x_y_h_w_b);
        tft.setCursor(x_y_h_w_b[0], x_y_h_w_b[1] + x_y_h_w_b[4] * b);
        tft.print(output);
        sprintf(output, "%s", WITH_OK_2[lingo]);
        calculateStringMetrics(F16B[font_typ], "MC", output, tft.width()/2, tft.height()/2 + 15, x_y_h_w_b);
        tft.setCursor(x_y_h_w_b[0], x_y_h_w_b[1] + 3 * x_y_h_w_b[4] * b);
        tft.print(output);
        cali_weight_old = cali_weight;
      }
    }
  #endif
}

void HM_Display::setup_tare_oled(int pos) {
  #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
    u8g2.setFont(u8g2_font_courB08_tf);
    u8g2.clearBuffer();
    int j = 0;
    while(j < 5) {
      u8g2.setCursor(8, 10+(j*13));
      if (glaeser[j].Gewicht < 1000) {
        sprintf(output, "%4dg - %3s", glaeser[j].Gewicht, GlasTypArray[glaeser[j].GlasTyp]); 
      } 
      else {
        sprintf(output, "%.1fkg - %3s", float(glaeser[j].Gewicht) / 1000, GlasTypArray[glaeser[j].GlasTyp]); 
      }
      u8g2.print(output);
      u8g2.setCursor(80, 10+(j*13));
      if (glaeser[j].Tare > 0) { 
        sprintf(output, "%7dg", glaeser[j].Tare); 
        u8g2.print(output);
      }
      else {
        sprintf(output, "%8s", MISSING[lingo]); 
        u8g2.print(output);
      }
      j++;
    }
    u8g2.setCursor(0, 10+(pos*13) );    
    u8g2.print("*");
    u8g2.sendBuffer();
  #endif
}

void HM_Display::setup_tare_tft(int pos) {
  #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
    if (change) {
      tft.fillScreen(BACKGROUND);
      tft.setTextColor(TEXT);
      tft.setFreeFont(F16B[font_typ]);
      sprintf(output, "%s", TAREVALUES_JAR[lingo]);
      calculateStringMetrics(F16B[font_typ], "MC", output, tft.width()/2, 15, x_y_h_w_b);
      tft.setCursor(x_y_h_w_b[0], x_y_h_w_b[1]);
      tft.print(output);
      tft.drawLine(0, 30, tft.width(), 30, TEXT);
      tft.setFreeFont(F12B[font_typ]);
      longest_string_1 = 0;
      for (int j = 0; j < 5; j++) {
        if (glaeser[j].Gewicht < 1000) {
          sprintf(output, "%dg", glaeser[j].Gewicht);
        }
        else { 
          sprintf(output, "%.1fkg", float(glaeser[j].Gewicht) / 1000); 
        }
        calculateStringMetrics(F12B[font_typ], "BR", output, 0, 0, x_y_h_w_b);
        if (x_y_h_w_b[3] > longest_string_1) {
          longest_string_1 = x_y_h_w_b[3];
        }
      }
      change = false;
    }
    int j = 0;
    while(j < 5) {
      if (j == pos) {
        tft.setTextColor(MARKER);
      }
      else {
        tft.setTextColor(TEXT);
      }
      if (glaeser[j].Gewicht < 1000) {
        sprintf(output, "%dg", glaeser[j].Gewicht); 
      }
      else {
        sprintf(output, "%.1fkg", float(glaeser[j].Gewicht) / 1000); 
      }
      calculateStringMetrics(F12B[font_typ], "BR", output, 5 + longest_string_1, 60+(j*30), x_y_h_w_b);
      tft.setCursor(x_y_h_w_b[0], x_y_h_w_b[1]);
      tft.print(output);
      sprintf(output, " - %s", GlasTypArray[glaeser[j].GlasTyp]);
      calculateStringMetrics(F12B[font_typ], "BL", output, 5 + longest_string_1, 60+(j*30), x_y_h_w_b);
      tft.setCursor(x_y_h_w_b[0], x_y_h_w_b[1]);
      tft.print(output);
      if (glaeser[j].Tare > 0) { 
        sprintf(output, "%dg", glaeser[j].Tare); 
      }
      else {
        sprintf(output, "%s", MISSING[lingo]); 
      }
      calculateStringMetrics(F12B[font_typ], "BR", output, tft.width() - 5, 60+(j*30), x_y_h_w_b);
      tft.setCursor(x_y_h_w_b[0], x_y_h_w_b[1]);
      tft.print(output);
      j++;
    }
  #endif
}

void HM_Display::setup_fuellmenge_oled(int a) {
  #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
    if (a == 1) {
      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_courB08_tf);
      int j = 0;
      while(j < 5) {
        u8g2.setCursor(20, 10+(j*13)); 
        sprintf(output, "%4dg      %3s", glaeser[j].Gewicht, GlasTypArray[glaeser[j].GlasTyp]);
        u8g2.print(output);
        j++;
      }
      u8g2.setCursor(10, 10+(pos*13));    
      u8g2.print("*");
      u8g2.sendBuffer();
    }
    else if (a == 2) {
      u8g2.clearBuffer();    
      int j = 0;
      while(j < 5) {
        u8g2.setCursor(20, 10+(j*13));
        if (j == pos){ 
          u8g2.setFont(u8g2_font_open_iconic_arrow_1x_t);
          u8g2.drawGlyph(5, 11+(j*13), 0x42);
          u8g2.drawGlyph(55, 11+(j*13), 0x41);
          u8g2.setFont(u8g2_font_courB08_tf);   
          sprintf(output, "%4dg      %3s", glaeser[j].Gewicht, GlasTypArray[glaeser[j].GlasTyp]);  
        }
        else {
          sprintf(output, "%4dg      %3s", glaeser[j].Gewicht, GlasTypArray[glaeser[j].GlasTyp]);
        }
        u8g2.print(output);
        j++;
      }                     
      u8g2.sendBuffer();
    }
    else if (a == 3) {
      u8g2.clearBuffer();
      int j = 0;
      while(j < 5) {
        u8g2.setCursor(20, 10+(j*13));
        if (j == pos){
          u8g2.setFont(u8g2_font_open_iconic_arrow_1x_t);
          u8g2.drawGlyph(72, 11+(j*13), 0x42);
          u8g2.drawGlyph(108, 11+(j*13), 0x41);
          u8g2.setFont(u8g2_font_courB08_tf);
          sprintf(output, "%4dg      %3s", glaeser[j].Gewicht, GlasTypArray[glaeser[j].GlasTyp]);
        }
        else {
          sprintf(output, "%4dg      %3s", glaeser[j].Gewicht, GlasTypArray[glaeser[j].GlasTyp]);
        }
        u8g2.print(output);
        j++;
      }
      u8g2.sendBuffer();
    }
    #endif
}

void HM_Display::setup_fuellmenge_tft(int a) {
  #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
    if (a == 0) {
      tft.fillScreen(BACKGROUND);
      tft.setTextColor(TEXT);
      tft.setFreeFont(F16B[font_typ]);
      sprintf(output, "%s", FILL_QUANTITY_JARS[lingo]);
      calculateStringMetrics(F16B[font_typ], "MC", output, tft.width()/2, 15, x_y_h_w_b);
      tft.setCursor(x_y_h_w_b[0], x_y_h_w_b[1]);
      tft.print(output);
      tft.drawLine(0, 30, tft.width(), 30, TEXT);
    }
    else if (a == 1) {
      int j = 0;
      tft.setFreeFont(F12B[font_typ]);
      while(j < 5) {
        if (j == pos) {
          tft.setTextColor(MARKER);
        }
        else {
          tft.setTextColor(TEXT);
        }
        sprintf(output, "%dg", glaeser[j].Gewicht);
        calculateStringMetrics(F12B[font_typ], "BL", output, 5, 30+((j+1) * y_offset_tft), x_y_h_w_b);
        tft.setCursor(x_y_h_w_b[0], x_y_h_w_b[1]);
        tft.print(output);
        tft.setCursor(110, 30+((j+1) * y_offset_tft));
        sprintf(output, "%s", GlasTypArrayTFT[glaeser[j].GlasTyp]);
        calculateStringMetrics(F12B[font_typ], "BR", output, tft.width() - 5, 30+((j+1) * y_offset_tft), x_y_h_w_b);
        tft.setCursor(x_y_h_w_b[0], x_y_h_w_b[1]);
        tft.print(output);
        j++;
      }
    }
    else if (a == 2) {
      int j = 0;
      while(j < 5) {
        tft.setCursor(5, 30+((j+1) * y_offset_tft));
        if (j == pos) {
          if (text_old != String(glaeser[j].Gewicht)) {
            tft.setTextColor(BACKGROUND);
            sprintf(output, "%sg", text_old);
            calculateStringMetrics(F12B[font_typ], "BL", output, 5, 30+((j+1) * y_offset_tft), x_y_h_w_b);
            tft.setCursor(x_y_h_w_b[0], x_y_h_w_b[1]);
            tft.print(output);
            text_old = String(glaeser[j].Gewicht);
          }
          tft.setTextColor(MARKER);
        }
        sprintf(output, "%dg", glaeser[j].Gewicht);
        calculateStringMetrics(F12B[font_typ], "BL", output, 5, 30+((j+1) * y_offset_tft), x_y_h_w_b);
        tft.setCursor(x_y_h_w_b[0], x_y_h_w_b[1]);
        tft.print(output);
        tft.setTextColor(TEXT);
        sprintf(output, "%s", GlasTypArrayTFT[glaeser[j].GlasTyp]);
        calculateStringMetrics(F12B[font_typ], "BR", output, tft.width() - 5, 30+((j+1) * y_offset_tft), x_y_h_w_b);
        tft.setCursor(x_y_h_w_b[0], x_y_h_w_b[1]);
        tft.print(output);
        j++;
      }
    }
    else if (a == 3) {
      int j = 0;
      while(j < 5) {
        sprintf(output, "%dg", glaeser[j].Gewicht);
        calculateStringMetrics(F12B[font_typ], "BL", output, 5, 30+((j+1) * y_offset_tft), x_y_h_w_b);
        tft.setCursor(x_y_h_w_b[0], x_y_h_w_b[1]);
        tft.print(output);
        if (j == pos) {
          if (text_old != String(GlasTypArrayTFT[glaeser[j].GlasTyp])) {
            tft.setTextColor(BACKGROUND);
            sprintf(output, "%s", text_old);
            calculateStringMetrics(F12B[font_typ], "BR", output, tft.width() - 5, 30+((j+1) * y_offset_tft), x_y_h_w_b);
            tft.setCursor(x_y_h_w_b[0], x_y_h_w_b[1]);
            tft.print(output);
            text_old = String(GlasTypArrayTFT[glaeser[j].GlasTyp]);
          }
          tft.setTextColor(MARKER);
        }
        sprintf(output, "%s", GlasTypArrayTFT[glaeser[j].GlasTyp]);
        calculateStringMetrics(F12B[font_typ], "BR", output, tft.width() - 5, 30+((j+1) * y_offset_tft), x_y_h_w_b);
        tft.setCursor(x_y_h_w_b[0], x_y_h_w_b[1]);
        tft.print(output);
        tft.setTextColor(TEXT);
        j++;
      }
    }
  #endif
}

void HM_Display::setup_automatik_oled (int menuoffset, bool change_value, int menuitem, int menuitem_used) {
  //Das ganze funktioniert nur wenn der Manüpunkt max 1 zuviel ist. Sollte noch ein Menüpunkt hinzukommen muss es geändert werden. Gilt auch für das OLED Display
  #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
    u8g2.setFont(u8g2_font_courB08_tf);
    u8g2.clearBuffer();
    u8g2.setCursor(10, 1 * y_offset - menuoffset * y_offset);
    u8g2.print(AUTOSTART[lingo]);
    u8g2.setCursor(95, 1 * y_offset);
    sprintf(output,"%5s", (autostart==0?OFF[lingo]:ON[lingo]));
    u8g2.print(output);
    u8g2.setCursor(10, 2 * y_offset - menuoffset * y_offset);
    u8g2.print(JAR_TOLERANCE[lingo]);
    u8g2.setCursor(95, 2 * y_offset - menuoffset * y_offset);
    u8g2.drawLine(0, 52, 128, 52);
    if (jartolerance > 0) {
      if (jartolerance > 9) {
        sprintf(output," %c%2dg", 177, jartolerance);
      }
      else {
        sprintf(output,"  %c%1dg", 177, jartolerance);
      }
    }
    else {
      sprintf(output,"%4dg", jartolerance);
    }
    u8g2.print(output);
    u8g2.setCursor(10, 3 * y_offset - menuoffset * y_offset);
    u8g2.print(CORRECTION[lingo]);
    u8g2.setCursor(95, 3 * y_offset - menuoffset * y_offset);
    sprintf(output,"%4dg", correction);
    u8g2.print(output);
    u8g2.setCursor(10, 4 * y_offset - menuoffset * y_offset);
    u8g2.print(AUTOCORRECTION[lingo]);
    u8g2.setCursor(95, 4 * y_offset - menuoffset * y_offset);
    sprintf(output,"%5s", (autocorrection==0?OFF[lingo]:ON[lingo]));
    u8g2.print(output);
    u8g2.setCursor(10, 5 * y_offset - menuoffset * y_offset);
    u8g2.print(KINDNESS[lingo]);
    u8g2.setCursor(95, 5 * y_offset - menuoffset * y_offset);
    sprintf(output,"%4dg", overfill_gr);
    u8g2.print(output);
    u8g2.setCursor(10, 6 * y_offset - menuoffset * y_offset);
    u8g2.print(FLOW_G_OVER_TIME[lingo]);
    u8g2.setCursor(95, 6 * y_offset - menuoffset * y_offset);
    if (init_weight_f > 0) {
      sprintf(output,"%4dg", init_weight_f);
    }
    else {
      sprintf(output,"%5s", OFF[lingo]);
    }
    u8g2.print(output);
    if (menuoffset > 0) {
      u8g2.setCursor(10, 7 * y_offset - menuoffset * y_offset);
      u8g2.print(WAIT_BEFOR_FILL[lingo]);
      u8g2.setCursor(95, 7 * y_offset - menuoffset * y_offset);
      sprintf(output,"%5s", (wait_befor_fill==0?OFF[lingo]:ON[lingo]));
      u8g2.print(output);
    }
    u8g2.setCursor(10, (7 * y_offset) + 5);
    u8g2.print(SAVE[lingo]);
    if (change_value == false && menuitem < menuitem_used) {
      u8g2.setCursor(1, 10+((menuitem)*y_offset - menuoffset * y_offset)); u8g2.print("*");
    }
    else if (change_value == true && menuitem < menuitem_used) {
      u8g2.setCursor(1, 8+((menuitem)*y_offset - menuoffset * y_offset)); u8g2.print("-");
    }
    else if (change_value == false && menuitem == 7) {
      u8g2.setCursor(1, 10+((menuitem - 1)*y_offset) + 5); u8g2.print("*");
    }
    u8g2.sendBuffer();
  #endif
}

void HM_Display::setup_automatik_oled_exit (int menuitem) {
  #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
    u8g2.setCursor(111, (menuitem * y_offset) + 5);
    u8g2.print("OK");
    u8g2.sendBuffer();
  #endif
}

void HM_Display::setup_automatik_tft (bool change_value, const char *menuitems[], int menuitems_number) {
  //Das ganze funktioniert nur wenn der Manüpunkt max 1 zuviel ist. Sollte noch ein Menüpunkt hinzukommen muss es geändert werden. Gilt auch für das OLED Display
  #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
    if (draw_frame == true) {
      tft.fillScreen(BACKGROUND);
      tft.setTextColor(TEXT);
      tft.setFreeFont(F16B[font_typ]);
      sprintf(output, "%s", AUTOMATIC[lingo]);
      calculateStringMetrics(F16B[font_typ], "MC", output, tft.width()/2, 15, x_y_h_w_b);
      tft.setCursor(x_y_h_w_b[0], x_y_h_w_b[1]);
      tft.print(output);
      tft.drawLine(0, 30, tft.width(), 30, TEXT);
      if (menuitems_number > 6) {
        tft.drawLine(0, 207, tft.width(), 207, TEXT);
      }
      draw_frame = false;
    }
    if (((pos >= 6 and pos != menuitems_number) or (pos_old == 6 and pos == 5)) and pos_old != pos) {
      tft.fillRect(0, 31, 320, 175, BACKGROUND);
      pos_old = pos;
    }
    String del_text = "";
    char del_text_1[30] = "";
    int i = 0;
    int j = 6;
    int offset = 0;
    if (pos >= 6 and pos != menuitems_number-1) {
      i = pos - 5;
      j = j + i;
      offset = i;
    }
    else if (pos == menuitems_number-1) {
      i = menuitems_number - pos;
      j = menuitems_number-1;
      offset = i;
    }
    tft.setFreeFont(F12B[font_typ]);
    tft.setTextDatum(BL_DATUM);
    while(i <= j) {
      tft.setTextColor(TEXT);
      if (i == pos and change_value == false) {
        tft.setTextColor(MARKER);
      }
      if (i < j) {
        tft.setCursor(5, 27+((i+1-offset) * y_offset_tft));
        tft.print(menuitems[i]);
        tft.setTextDatum(BR_DATUM);
        tft.setTextColor(BACKGROUND);
        switch (i) {
          case 0: sprintf(output,"%s", autostart==false?OFF[lingo]:ON[lingo]);
                  if (value_old != autostart and change_value == true and i == pos) {
                    del_text = autostart==true?OFF[lingo]:ON[lingo];
                    tft.setCursor(tft.width() - 5 - tft.textWidth(del_text), 27+((i+1-offset) * y_offset_tft));
                    tft.print(del_text);
                    value_old = autostart;
                  }
                  break;
          case 1: if (jartolerance > 0) {
                    sprintf(output,"±%dg", jartolerance);
                  }
                  else {
                    sprintf(output,"%dg", jartolerance);
                  }
                  if (value_old != jartolerance and change_value == true and i == pos) {
                    if (value_old > 0) {sprintf(del_text_1,"±%ig", value_old);}
                    else {sprintf(del_text_1,"%ig", value_old);}
                    tft.setCursor(tft.width() - 5 - tft.textWidth(del_text_1), 27+((i+1-offset) * y_offset_tft));
                    tft.print(del_text_1);
                    value_old = jartolerance;
                  }
                  break;
          case 2: sprintf(output,"%dg", correction);
                  if (value_old != correction and change_value == true and i == pos) {
                    sprintf(del_text_1,"%ig", value_old);
                    tft.setCursor(tft.width() - 5 - tft.textWidth(del_text_1), 27+((i+1-offset) * y_offset_tft));
                    tft.print(del_text_1);
                    value_old = correction;
                  }
                  break;
          case 3: sprintf(output,"%s", autocorrection==false?OFF[lingo]:ON[lingo]);
                  if (value_old != autocorrection and change_value == true and i == pos) {
                    del_text = autocorrection==true?OFF[lingo]:ON[lingo];
                    tft.setCursor(tft.width() - 5 - tft.textWidth(del_text), 27+((i+1-offset) * y_offset_tft));
                    tft.print(del_text);
                    value_old = autocorrection;
                  }
                  break;
          case 4: sprintf(output,"%dg", overfill_gr);
                  if (value_old != overfill_gr and change_value == true and i == pos) {
                    sprintf(del_text_1,"%ig", value_old);
                    tft.setCursor(tft.width() - 5 - tft.textWidth(del_text_1), 27+((i+1-offset) * y_offset_tft));
                    tft.print(del_text_1);
                    value_old = overfill_gr;
                  }
                  break;
          case 5: if (init_weight_f > 0) {
                    sprintf(output,"%dg", init_weight_f);
                  }
                  else {
                    sprintf(output,"%s", OFF[lingo]);
                  }
                  if (value_old != init_weight_f and change_value == true and i == pos) {
                    if (value_old > 0) {sprintf(del_text_1,"%ig", value_old);}
                    else if (value_old <= 1) {sprintf(del_text_1,"%s", OFF[lingo]);}
                    tft.setCursor(tft.width() - 5 - tft.textWidth(del_text_1), 27+((i+1-offset) * y_offset_tft));
                    tft.print(del_text_1);
                    value_old = init_weight_f;
                  }
                  break;
          case 6: sprintf(output,"%s", wait_befor_fill==false?OFF[lingo]:ON[lingo]);
                  if (value_old != wait_befor_fill and change_value == true and i == pos) {
                    del_text = wait_befor_fill==true?OFF[lingo]:ON[lingo];
                    tft.setCursor(tft.width() - 5 - tft.textWidth(del_text), 27+((i+1-offset) * y_offset_tft));
                    tft.print(del_text);
                    value_old = wait_befor_fill;
                  }
                  break;
        }
        if (i == pos) {
          tft.setTextColor(MARKER);
        }
        else {
          tft.setTextColor(TEXT);
        }
        tft.setCursor(tft.width() - 5 - tft.textWidth(output), 27+((i+1-offset) * y_offset_tft));
        tft.print(output);
      }
      else {    //Save
        tft.setCursor(5, 35+(7 * y_offset_tft));
        tft.print(menuitems[menuitems_number - 1]);
      }
      i++;
    }
  #endif
}

void HM_Display::setup_automatik_tft_exit () {
  #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
    tft.setCursor(tft.width() - 5 - tft.textWidth("OK"), 35+(7 * y_offset_tft));
    tft.print("OK");
  #endif
}

void HM_Display::setup_servoWinkel_oled(bool change_value, int menuitem, int menuitem_used) {
  #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
    u8g2.setFont(u8g2_font_courB08_tf);
    u8g2.clearBuffer();
    u8g2.setCursor(10, 1 * y_offset);
    u8g2.print(LIVESETUP[lingo]);
    u8g2.setCursor(95, 1 * y_offset);
    sprintf(output,"%5s", (servo_live==false?OFF[lingo]:ON[lingo]));
    u8g2.print(output);
    u8g2.setCursor(10, 2 * y_offset);
    u8g2.print(MINIMUM[lingo]);
    u8g2.setCursor(95, 2 * y_offset);
    sprintf(output,"%4d°", angle_min);
    u8g2.print(output);
    u8g2.setCursor(10, 3 * y_offset);
    u8g2.print(FULL_MODE[lingo]);
    u8g2.setCursor(95, 3 * y_offset);
    sprintf(output,"%5d", fullmode);
    u8g2.print(output);
    u8g2.setCursor(10, 4 * y_offset);
    u8g2.print(FINEDOSAGE_WEIGHT[lingo]);
    u8g2.setCursor(95, 4 * y_offset);
    sprintf(output,"%4.0fg", finedos_weight[fullmode-1]);
    u8g2.print(output);
    u8g2.setCursor(10, 5 * y_offset);
    u8g2.print(FINEDOSAGE[lingo]);
    u8g2.setCursor(95, 5 * y_offset);
    sprintf(output,"%4d°", angle_fine[fullmode-1]);
    u8g2.print(output);
    u8g2.setCursor(10, 6 * y_offset);
    u8g2.print(MAXIMUM[lingo]);
    u8g2.setCursor(95, 6 * y_offset);
    sprintf(output,"%4d°", angle_max[fullmode-1]);
    u8g2.print(output);
    u8g2.setCursor(10, (7 * y_offset) + 5);
    u8g2.print(SAVE[lingo]);
    if (change_value == false && menuitem < menuitem_used) {
      u8g2.setCursor(1, 10+((menuitem)*y_offset)); u8g2.print("*");
    }
    else if (change_value == true && menuitem < menuitem_used) {
      u8g2.setCursor(1, 8+((menuitem)*y_offset)); u8g2.print("-");
    }
    else if (change_value == false && menuitem == 6) {
      u8g2.setCursor(1, 10+((menuitem)*y_offset) + 5); u8g2.print("*");
    }
    u8g2.sendBuffer();
  #endif
}

void HM_Display::setup_servoWinkel_oled_exit(int menuitem) {
  #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
    u8g2.setCursor(111, ((menuitem + 1) * y_offset) + 5);
    u8g2.print("OK");
    u8g2.sendBuffer();
  #endif
}

void HM_Display::setup_servoWinkel_tft(bool change_value, int menu_items_number, const char *menuitems[]) {
  #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
    int j = 0;
    if (draw_frame == true) {
      tft.fillScreen(BACKGROUND);
      tft.setTextColor(TEXT);
      tft.setFreeFont(F16B[font_typ]);
      sprintf(output, "%s", SERVOSETTINGS[lingo]);
      calculateStringMetrics(F16B[font_typ], "MC", output, tft.width()/2, 15, x_y_h_w_b);
      tft.setCursor(x_y_h_w_b[0], x_y_h_w_b[1]);
      tft.print(output);
      tft.drawLine(0, 30, tft.width(), 30, TEXT);
      draw_frame = false;
    }
    String del_text = "";
    char del_text_1[30] = "";
    tft.setFreeFont(F12B[font_typ]);
    while(j < menu_items_number) {
      tft.setTextColor(TEXT);
      if (j == pos and change_value == false) {
        tft.setTextColor(MARKER);
      }
      if (j < menu_items_number - 1) {
        tft.setCursor(5, 27+((j+1) * y_offset_tft));
        tft.print(menuitems[j]);
        tft.setTextColor(BACKGROUND);
        switch (j) {
          case 0: sprintf(output,"%s", servo_live==false?OFF[lingo]:ON[lingo]);
                  if (value_old != servo_live and change_value == true and j == pos) {
                    del_text = servo_live==true?OFF[lingo]:ON[lingo];
                    tft.setCursor(tft.width() - 5 - tft.textWidth(del_text), 27+((j+1) * y_offset_tft));
                    tft.print(del_text);                    
                    value_old = servo_live;
                  }
                  break;
          case 1: sprintf(output,"%d°", angle_min);
                  if (value_old != angle_min and change_value == true and j == pos) {
                    sprintf(del_text_1,"%d°", value_old);
                    tft.setCursor(tft.width() - 5 - tft.textWidth(del_text_1), 27+((j+1) * y_offset_tft));
                    tft.print(del_text_1);
                    value_old = angle_min;
                  }
                  break;
          case 2: sprintf(output,"%d", fullmode);
                  if (value_old != fullmode and change_value == true and j == pos) {
                    sprintf(del_text_1,"%d", value_old);
                    tft.setCursor(tft.width() - 5 - tft.textWidth(del_text_1), 27+((j+1) * y_offset_tft));
                    tft.print(del_text_1);
                    //noch Feindosierung Gewicht, Feindosierung Winkel unf Maximum winkel löschen
                    sprintf(del_text_1,"%.0fg", finedos_weight[value_old-1]);
                    tft.setCursor(tft.width() - 5 - tft.textWidth(del_text_1), 27+((j+2) * y_offset_tft));
                    tft.print(del_text_1);
                    sprintf(del_text_1,"%d°", angle_fine[value_old-1]);
                    tft.setCursor(tft.width() - 5 - tft.textWidth(del_text_1), 27+((j+3) * y_offset_tft));
                    tft.print(del_text_1);
                    sprintf(del_text_1,"%d°", angle_max[value_old-1]);
                    tft.setCursor(tft.width() - 5 - tft.textWidth(del_text_1), 27+((j+4) * y_offset_tft));
                    tft.print(del_text_1);
                    //value_old setzen
                    value_old = fullmode;
                  }
                  break;
          case 3: sprintf(output,"%.0fg", finedos_weight[fullmode-1]);
                  if (value_old * 1.0 != finedos_weight[fullmode-1] and change_value == true and j == pos) {
                    sprintf(del_text_1,"%.0fg", value_old);
                    tft.setCursor(tft.width() - 5 - tft.textWidth(del_text_1), 27+((j+1) * y_offset_tft));
                    tft.print(del_text_1);
                    value_old = finedos_weight[fullmode-1];
                  }
                  break;
          case 4: sprintf(output,"%d°", angle_fine[fullmode-1]);
                  if (value_old != angle_fine[fullmode-1] and change_value == true and j == pos) {
                    sprintf(del_text_1,"%d°", value_old);
                    tft.setCursor(tft.width() - 5 - tft.textWidth(del_text_1), 27+((j+1) * y_offset_tft));
                    tft.print(del_text_1);
                    value_old = angle_fine[fullmode-1];
                  }
                  break;
          case 5: sprintf(output,"%d°", angle_max[fullmode-1]);
                  if (value_old != angle_max[fullmode-1] and change_value == true and j == pos) {
                    sprintf(del_text_1,"%d°", value_old);
                    tft.setCursor(tft.width() - 5 - tft.textWidth(del_text_1), 27+((j+1) * y_offset_tft));
                    tft.print(del_text_1);
                    value_old = angle_max[fullmode-1];
                  }
                  break;
        }
        if (j == pos) {
          tft.setTextColor(MARKER);
        }
        else {
          tft.setTextColor(TEXT);
        }
        tft.setCursor(tft.width() - 5 - tft.textWidth(output), 27+((j+1) * y_offset_tft));
        tft.print(output);
      }
      else {
        tft.setCursor(5, 35+(7 * y_offset_tft));
        tft.print(menuitems[j]);
      }
      j++;
    }
  #endif
}

void HM_Display::setup_servoWinkel_tft_exit() {
  #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
    tft.setCursor(tft.width() - 5 - tft.textWidth("OK"), 35+(7 * y_offset_tft));
    tft.print("OK");
  #endif
}

void HM_Display::setup_counter_oled(int a) {
  #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
    int j;
    if (a == 1) { //screen 1
      j = 0;
      u8g2.setFont(u8g2_font_courB08_tf);
      u8g2.clearBuffer();
      while ( j < 5 ) {
        u8g2.setCursor(1, 10 + (j * 13));
        sprintf(output, "%4dg %3s", glaeser[j].Gewicht,GlasTypArray[glaeser[j].GlasTyp]);
        u8g2.print(output);
        u8g2.setCursor(50, 10 + (j * 13));
        sprintf(output, "%9d St.", glaeser[j].Count);
        u8g2.print(output);
        j++;
      }
      u8g2.sendBuffer();
    }
    else if (a == 2) { //screen 2
      j = 0;
      u8g2.setFont(u8g2_font_courB08_tf);
      u8g2.clearBuffer();
      while ( j < 5) {
        u8g2.setCursor(1, 10 + (j * 13));
        sprintf(output, "%4dg %3s", glaeser[j].Gewicht,GlasTypArray[glaeser[j].GlasTyp]);
        u8g2.print(output);
        u8g2.setCursor(65, 10 + (j * 13));
        sprintf(output, "%8.3fkg", glaeser[j].Gewicht * glaeser[j].Count / 1000.0);
        u8g2.print(output);
        j++;
      }
      u8g2.sendBuffer();
    }
    else if (a == 3) { //screen 3
      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_courB14_tf);
      u8g2.setCursor(1, 15);
      u8g2.print(TOTAL[lingo]);
      u8g2.setFont(u8g2_font_courB18_tf);
      u8g2.setCursor(10, 50);
      sprintf(output, "%5.1fkg", filling_weight);
      u8g2.print(output);
      u8g2.sendBuffer();
    }
    else if (a == 4) { //screen 4
      u8g2.setFont(u8g2_font_courB10_tf);
      u8g2.clearBuffer();
      u8g2.setCursor(10, 12);    u8g2.print(RESET[lingo]);
      u8g2.setCursor(10, 28);    u8g2.print(ABORT[lingo]);
      u8g2.setCursor(0, 12 + ((pos) * 16));
      u8g2.print("*");
      u8g2.sendBuffer();
    }
  #endif
}

void HM_Display::setup_counter_oled_exit() {
  #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
    u8g2.setCursor(105, 12 + ((pos) * 16));
    u8g2.print("OK");
    u8g2.sendBuffer();
  #endif
}

void HM_Display::setup_counter_tft(int a) {
  #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
    int j;
    if (a == 1) { //screen 1
      j = 0;
      if (draw_frame == true) {
        tft.fillScreen(BACKGROUND);
        tft.setTextColor(TEXT);
        tft.setFreeFont(F16B[font_typ]);
        sprintf(output, "%s", COUNTER[lingo]);
        calculateStringMetrics(F16B[font_typ], "MC", output, tft.width()/2, 15, x_y_h_w_b);
        tft.setCursor(x_y_h_w_b[0], x_y_h_w_b[1]);
        tft.print(output);
        tft.drawLine(0, 30, tft.width(), 30, TEXT);
        draw_frame = false;
      }
      tft.setFreeFont(F12B[font_typ]);
      longest_string_1 = 0;
      for (int j = 0; j < 5; j++) {
        sprintf(output, "%dg", glaeser[j].Gewicht);
        calculateStringMetrics(F12B[font_typ], "BR", output, 0, 0, x_y_h_w_b);
        if (x_y_h_w_b[3] > longest_string_1) {
          longest_string_1 = x_y_h_w_b[3];
        }
      }
      while ( j < 5 ) {
        sprintf(output, "%dg", glaeser[j].Gewicht);
        calculateStringMetrics(F12B[font_typ], "BR", output, 5 + longest_string_1, 60+(j*27), x_y_h_w_b);
        tft.setCursor(x_y_h_w_b[0], x_y_h_w_b[1]);
        tft.print(output);
        sprintf(output, " - %s", GlasTypArray[glaeser[j].GlasTyp]);
        tft.print(output);
        sprintf(output, "%d St.", glaeser[j].Count);
        calculateStringMetrics(F12B[font_typ], "BR", output, tft.width() - 5, 60+(j*27), x_y_h_w_b);
        tft.setCursor(x_y_h_w_b[0], x_y_h_w_b[1]);
        tft.print(output);
        j++;
      }
    }
    else if (a == 2) { //screen 2
      j = 0;
      if (change == true) {
        tft.fillRect(0, 31, 320, 209, BACKGROUND);
        change = false;
      }
      while ( j < 5 ) {
        sprintf(output, "%dg", glaeser[j].Gewicht);
        calculateStringMetrics(F12B[font_typ], "BR", output, 5 + longest_string_1, 60+(j*27), x_y_h_w_b);
        tft.setCursor(x_y_h_w_b[0], x_y_h_w_b[1]);
        tft.print(output);
        sprintf(output, " - %s", GlasTypArray[glaeser[j].GlasTyp]);
        tft.print(output);
        sprintf(output, "%.3fkg", glaeser[j].Gewicht * glaeser[j].Count / 1000.0);
        calculateStringMetrics(F12B[font_typ], "BR", output, tft.width() - 5, 60+(j*27), x_y_h_w_b);
        tft.setCursor(x_y_h_w_b[0], x_y_h_w_b[1]);
        tft.print(output);
        j++;
      }
    }
    else if (a == 3) { //screen 3
      if (change == true) {
        tft.fillRect(0, 31, 320, 209, BACKGROUND);
        change = false;
      }
      float b = 1.5;
      tft.setFreeFont(F26B[font_typ]);
      sprintf(output, TOTAL[lingo]);
      calculateStringMetrics(F26B[font_typ], "MC", output, tft.width()/2, tft.height()/2+15, x_y_h_w_b);
      tft.setCursor(x_y_h_w_b[0], x_y_h_w_b[1]-x_y_h_w_b[2]/2*b);
      tft.print(output);
      sprintf(output, "%.1fkg", filling_weight);
      calculateStringMetrics(F26B[font_typ], "MC", output, tft.width()/2, tft.height()/2+15, x_y_h_w_b);
      x_pos = CenterPosX(output, 36, 320);
      tft.setCursor(x_y_h_w_b[0], x_y_h_w_b[1]+x_y_h_w_b[2]/2*b);
      tft.print(output);
    }
    else if (a == 4) { //screen 4
      if (change == true) {
        tft.fillRect(0, 31, 320, 209, BACKGROUND);
        change = false;
      }
      tft.setFreeFont(F16B[font_typ]);
      if (pos == 0) {tft.setTextColor(MARKER);}
      else {tft.setTextColor(TEXT);}
      calculateStringMetrics(F16B[font_typ], "BL", output, 5, 33+(1*35), x_y_h_w_b);
      tft.setCursor(x_y_h_w_b[0], x_y_h_w_b[1]);
      tft.print(RESET[lingo]);
      if (pos == 1) {tft.setTextColor(MARKER);}
      else {tft.setTextColor(TEXT);}
      calculateStringMetrics(F16B[font_typ], "BL", output, 5, 35+(2*35), x_y_h_w_b);
      tft.setCursor(x_y_h_w_b[0], x_y_h_w_b[1]);
      tft.print(ABORT[lingo]);
    }
  #endif
}

void HM_Display::setup_counter_tft_exit() {
  #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
    tft.setTextColor(MARKER);
    tft.setCursor(tft.width() - 5 - tft.textWidth("OK"), 35+((pos+1) * 35));
    tft.print("OK");
  #endif
}

void HM_Display::setup_trip_counter_oled(int a) {
  #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
    int j;
    if (a == 1) { //screen 1
      j = 0;
      u8g2.setFont(u8g2_font_courB08_tf);
      u8g2.clearBuffer();
      while (j < 5) {
        u8g2.setCursor(1, 10 + (j * 13));
        sprintf(output, "%4dg %3s", glaeser[j].Gewicht, GlasTypArray[glaeser[j].GlasTyp]);
        u8g2.print(output);
        u8g2.setCursor(50, 10 + (j * 13));
        sprintf(output, "%9d St.", glaeser[j].TripCount);
        u8g2.print(output);
        j++;
      }
      u8g2.sendBuffer();
    }
    else if (a == 2) { //screen 2
      j = 0;
      u8g2.setFont(u8g2_font_courB08_tf);
      u8g2.clearBuffer();
      while (j < 5) {
        u8g2.setCursor(1, 10 + (j * 13));
        sprintf(output, "%4dg %3s", glaeser[j].Gewicht,GlasTypArray[glaeser[j].GlasTyp]);
        u8g2.print(output);
        u8g2.setCursor(65, 10 + (j * 13));
        sprintf(output, "%8.3fkg", glaeser[j].Gewicht * glaeser[j].TripCount / 1000.0);
        u8g2.print(output);
        j++;
      }
      u8g2.sendBuffer();
    }
    else if (a == 3) { //screen 3
      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_courB14_tf);
      u8g2.setCursor(5, 15);
      u8g2.print(TOTAL[lingo]);
      u8g2.setFont(u8g2_font_courB18_tf);
      u8g2.setCursor(10, 50);
      sprintf(output, "%5.1fkg", filling_weight);
      u8g2.print(output);
      u8g2.sendBuffer();
    }
    else if (a == 4) { //screen 4
      u8g2.setFont(u8g2_font_courB10_tf);
      u8g2.clearBuffer();
      u8g2.setCursor(10, 12);    u8g2.print(RESET[lingo]);
      u8g2.setCursor(10, 28);    u8g2.print(ABORT[lingo]);
      u8g2.setCursor(0, 12 + ((pos) * 16));
      u8g2.print("*");
      u8g2.sendBuffer();
    }
  #endif
}

void HM_Display::setup_trip_counter_oled_exit() {
  #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
    u8g2.setCursor(105, 12 + ((pos) * 16));
    u8g2.print("OK");
    u8g2.sendBuffer();
  #endif
}

void HM_Display::setup_trip_counter_tft(int a) {
  #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
    int j;
    if (a == 1) { //screen 1
      j = 0;
      if (draw_frame == true) {
        tft.fillScreen(BACKGROUND);
        tft.setTextColor(TEXT);
        tft.setFreeFont(F16B[font_typ]);
        sprintf(output, "%s", COUNTER_TRIP[lingo]);
        calculateStringMetrics(F16B[font_typ], "MC", output, tft.width()/2, 15, x_y_h_w_b);
        tft.setCursor(x_y_h_w_b[0], x_y_h_w_b[1]);
        tft.print(output);
        tft.drawLine(0, 30, tft.width(), 30, TEXT);
        draw_frame = false;
      }
      tft.setFreeFont(F12B[font_typ]);
      longest_string_1 = 0;
      for (int j = 0; j < 5; j++) {
        sprintf(output, "%dg", glaeser[j].Gewicht);
        calculateStringMetrics(F12B[font_typ], "BR", output, 0, 0, x_y_h_w_b);
        if (x_y_h_w_b[3] > longest_string_1) {
          longest_string_1 = x_y_h_w_b[3];
        }
      }
      while ( j < 5 ) {
        sprintf(output, "%dg", glaeser[j].Gewicht);
        calculateStringMetrics(F12B[font_typ], "BR", output, 5 + longest_string_1, 60+(j*27), x_y_h_w_b);
        tft.setCursor(x_y_h_w_b[0], x_y_h_w_b[1]);
        tft.print(output);
        sprintf(output, " - %s", GlasTypArray[glaeser[j].GlasTyp]);
        tft.print(output);
        sprintf(output, "%d St.", glaeser[j].TripCount);
        calculateStringMetrics(F12B[font_typ], "BR", output, tft.width() - 5, 60+(j*27), x_y_h_w_b);
        tft.setCursor(x_y_h_w_b[0], x_y_h_w_b[1]);
        tft.print(output);
        j++;
      }
    }
    else if (a == 2) { //screen 2
      j = 0;
      if (change == true) {
        tft.fillRect(0, 31, 320, 209, BACKGROUND);
        change = false;
      }
      while ( j < 5 ) {
        sprintf(output, "%dg", glaeser[j].Gewicht);
        calculateStringMetrics(F12B[font_typ], "BR", output, 5 + longest_string_1, 60+(j*27), x_y_h_w_b);
        tft.setCursor(x_y_h_w_b[0], x_y_h_w_b[1]);
        tft.print(output);
        sprintf(output, " - %s", GlasTypArray[glaeser[j].GlasTyp]);
        tft.print(output);
        sprintf(output, "%.3fkg", glaeser[j].Gewicht * glaeser[j].TripCount / 1000.0);
        calculateStringMetrics(F12B[font_typ], "BR", output, tft.width() - 5, 60+(j*27), x_y_h_w_b);
        tft.setCursor(x_y_h_w_b[0], x_y_h_w_b[1]);
        tft.print(output);
        j++;
      }
    }
    else if (a == 3) { //screen 3
      if (change == true) {
        tft.fillRect(0, 31, 320, 209, BACKGROUND);
        change = false;
      }
      float b = 1.5;
      tft.setFreeFont(F26B[font_typ]);
      sprintf(output, TOTAL[lingo]);
      calculateStringMetrics(F26B[font_typ], "MC", output, tft.width()/2, tft.height()/2+15, x_y_h_w_b);
      tft.setCursor(x_y_h_w_b[0], x_y_h_w_b[1]-x_y_h_w_b[2]/2*b);
      tft.print(output);
      sprintf(output, "%.1fkg", filling_weight);
      calculateStringMetrics(F26B[font_typ], "MC", output, tft.width()/2, tft.height()/2+15, x_y_h_w_b);
      x_pos = CenterPosX(output, 36, 320);
      tft.setCursor(x_y_h_w_b[0], x_y_h_w_b[1]+x_y_h_w_b[2]/2*b);
      tft.print(output);
    }
    else if (a == 4) { //screen 4
      if (change == true) {
        tft.fillRect(0, 31, 320, 209, BACKGROUND);
        change = false;
      }
      tft.setFreeFont(F16B[font_typ]);
      if (pos == 0) {tft.setTextColor(MARKER);}
      else {tft.setTextColor(TEXT);}
      calculateStringMetrics(F16B[font_typ], "BL", output, 5, 33+(1*35), x_y_h_w_b);
      tft.setCursor(x_y_h_w_b[0], x_y_h_w_b[1]);
      tft.print(RESET[lingo]);
      if (pos == 1) {tft.setTextColor(MARKER);}
      else {tft.setTextColor(TEXT);}
      calculateStringMetrics(F16B[font_typ], "BL", output, 5, 35+(2*35), x_y_h_w_b);
      tft.setCursor(x_y_h_w_b[0], x_y_h_w_b[1]);
      tft.print(ABORT[lingo]);
    }
  #endif
}

void HM_Display::setup_trip_counter_tft_exit() {
  #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
    tft.setTextColor(MARKER);
    tft.setCursor(tft.width() - 5 - tft.textWidth("OK"), 35+((pos+1) * 35));
    tft.print("OK");
  #endif
}

void HM_Display::setup_INA219_menu_oled(int menuitem, int menuitem_used, bool change_value) {
  #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
    u8g2.setFont(u8g2_font_courB08_tf);
    u8g2.clearBuffer();
    u8g2.setCursor(10, 1 * y_offset);
    u8g2.print(SERVO_CURRENT[lingo]);
    u8g2.setCursor(90, 1 * y_offset);
    if (current_servo > 0) {
      sprintf(output,"%4dmA", (current_servo));
    }
    else {
      sprintf(output,"%6s", (OFF[lingo]));
    }
    u8g2.print(output);
    if (current_servo > 0) {
      u8g2.setCursor(10, 2 * y_offset);
      u8g2.print(CAL_HONEY_GATE[lingo]);
      menuitem_used = 2;
    }
    else {
      u8g2.setCursor(10, 2 * y_offset);
      u8g2.print(SHOW_CURRENT[lingo]);
      u8g2.setCursor(90, 2 * y_offset);
      sprintf(output,"%6s", (show_current==0?OFF[lingo]:ON[lingo]));
      u8g2.print(output);
      menuitem_used = 2;
    }
    u8g2.setCursor(10, (7 * y_offset) + 5);
    u8g2.print(SAVE[lingo]);
    // Positionsanzeige im Menu. "*" wenn nicht ausgewählt, Pfeil wenn ausgewählt
    if (change_value == false && menuitem < menuitem_used) {
      u8g2.setCursor(1, 10+((menuitem)*y_offset)); u8g2.print("*");
    }
    else if (change_value == true && menuitem < menuitem_used) {
      u8g2.setCursor(1, 8+((menuitem)*y_offset)); u8g2.print("-");
    }
    else if (change_value == false && menuitem == 6) {
      u8g2.setCursor(1, 10+((menuitem)*y_offset) + 5); u8g2.print("*");
    }
    u8g2.sendBuffer();
  #endif
}

void HM_Display::setup_INA219_oled_save(int menuitem) {
  #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
    u8g2.setCursor(111, ((menuitem + 1) * y_offset) + 5);
    u8g2.print("OK");
    u8g2.sendBuffer();
  #endif
}

void HM_Display::setup_INA219_cal1_oled(String calibration_status, bool change_value, int menuitem, int menuitem_used) {
  #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
    u8g2.setFont(u8g2_font_courB08_tf);
    u8g2.clearBuffer();
    u8g2.setCursor(10, 1 * y_offset);
    u8g2.print(calibration_status);
    u8g2.setCursor(10, 3 * y_offset);
    u8g2.printf("%s:", MAX_CURRENT[lingo]);
    u8g2.setCursor(90, 3 * y_offset);
    sprintf(output,"%4imA", current_servo - 30);
    u8g2.print(output);
    u8g2.setCursor(10, 4 * y_offset);
    u8g2.printf("%s:", MIN_ANGLE[lingo]);
    u8g2.setCursor(90, 4 * y_offset);
    sprintf(output,"%5i°", angle_min);
    u8g2.print(output);
    u8g2.setCursor(10, (7 * y_offset) + 5);
    u8g2.print(BACK[lingo]);
    if (change_value == false && menuitem < menuitem_used && calibration_status == START[lingo]) {
      u8g2.setCursor(1, 10+((menuitem)*y_offset)); u8g2.print("*");
    }
    else if (change_value == false && menuitem == 6) {
      u8g2.setCursor(1, 10+((menuitem)*y_offset) + 5); u8g2.print("*");
    }
    u8g2.sendBuffer();
  #endif
}

void HM_Display::setup_INA219_cal2_oled(String calibration_status, int cal_winkel, String quetschhan) {
  #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
    u8g2.setFont(u8g2_font_courB08_tf);
    u8g2.clearBuffer();
    u8g2.setCursor(10, 1 * y_offset);
    u8g2.print(calibration_status);
    u8g2.setCursor(10, 3 * y_offset);
    u8g2.printf("%s:", CURRENT[lingo]);
    u8g2.setCursor(90, 3 * y_offset);
    sprintf(output,"%4imA", current_mA);
    u8g2.print(output);
    u8g2.setCursor(10, 4 * y_offset);
    u8g2.printf("%s:", ANGLE);
    u8g2.setCursor(90, 4 * y_offset);
    sprintf(output,"%5i°", cal_winkel);
    u8g2.print(output);
    u8g2.setCursor(10, 5 * y_offset);
    u8g2.print(HONEY_GATE[lingo]);
    u8g2.setCursor(90, 5 * y_offset);
    sprintf(output,"%6s", quetschhan);
    u8g2.print(output);
    u8g2.setCursor(1, (7 * y_offset) + 7); 
    u8g2.print("*");
    u8g2.setCursor(10, (7 * y_offset) + 5);
    u8g2.print(ABORT[lingo]);
    u8g2.sendBuffer();
  #endif
}

void HM_Display::setup_INA219_cal3_oled(String calibration_status) {
  #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
    u8g2.setFont(u8g2_font_courB08_tf);
    u8g2.clearBuffer();
    u8g2.setCursor(10, 1 * y_offset);
    u8g2.print(calibration_status);
    u8g2.setCursor(10, 3 * y_offset);
    u8g2.printf("%s:",CURRENT[lingo]);
    u8g2.setCursor(90, 3 * y_offset);
    sprintf(output,"%4imA", current_mA);
    u8g2.print(output);
    u8g2.setCursor(10, 4 * y_offset);
    u8g2.printf("%s:",ANGLE[lingo]);
    u8g2.setCursor(90, 4 * y_offset);
    sprintf(output,"%5i°", angle_min);
    u8g2.print(output);
    u8g2.setCursor(1, (7 * y_offset) + 7); 
    u8g2.print("*");
    u8g2.setCursor(10, (7 * y_offset) + 5);
    u8g2.print(SAVE[lingo]);
    u8g2.sendBuffer();
  #endif
}

void HM_Display::setup_INA219_menu_tft(bool change_value , int menu_items_number, const char *menuitems_1[], const char *menuitems_2[]) {
  #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
    int a = 0;
    if (draw_frame == true) {
      tft.fillScreen(BACKGROUND);
      tft.setTextColor(TEXT);
      tft.setFreeFont(F16B[font_typ]);
      sprintf(output, "%s", INA219_SETUP[lingo]);
      calculateStringMetrics(F16B[font_typ], "MC", output, tft.width()/2, 15, x_y_h_w_b);
      tft.setCursor(x_y_h_w_b[0], x_y_h_w_b[1]);
      tft.print(output);
      tft.drawLine(0, 30, tft.width(), 30, TEXT);
      draw_frame = false;
    }
    String del_text = "";
    char del_text_1[30] = "";
    tft.setFreeFont(F12B[font_typ]);
    while(a < menu_items_number) {
      tft.setTextColor(TEXT);
      if (a == pos and change_value == false) {
        tft.setTextColor(MARKER);
      }
      if (a < menu_items_number - 1) {
        if (current_servo == 0) {
          tft.setCursor(5, 27+((a+1) * y_offset_tft));
          tft.print(menuitems_2[a]);
        }
        else {
          tft.setCursor(5, 27+((a+1) * y_offset_tft));
          tft.print(menuitems_1[a]);
        }
        tft.setTextColor(BACKGROUND);
        switch (a) {
          case 0: 
                  if (current_servo > 0) {
                    sprintf(output,"%dmA", current_servo);
                  }
                  else {
                    sprintf(output,"%s", OFF[lingo]);
                  }
                  if (value_old != current_servo and change_value == true and a == pos) {
                    if (value_old > 0) {sprintf(del_text_1,"%dmA", value_old);}
                    else if (value_old <= 1) {sprintf(del_text_1,"%s", OFF[lingo]);}
                    tft.setCursor(tft.width() - 5 - tft.textWidth(del_text_1), 27+((a+1) * y_offset_tft));
                    tft.print(del_text_1);
                    //noch die zweite Zeile setzen
                    if (value_old == 0 and current_servo == 50) {
                      sprintf(del_text_1,"%s", menuitems_2[1]);
                    }
                    else if (value_old == 50 and current_servo == 0) {
                      sprintf(del_text_1,"%s", menuitems_1[1]);
                    }
                    if ((value_old == 0 and current_servo == 50) or (value_old == 50 and current_servo == 0)) {
                      tft.setCursor(5, 27+(2 * y_offset_tft));
                      tft.print(del_text_1);
                      if (value_old == 0 and current_servo == 50) {
                        del_text = show_current==false?OFF[lingo]:ON[lingo];
                        tft.setCursor(tft.width() - 5 - tft.textWidth(del_text), 27+(2 * y_offset_tft));
                        tft.print(del_text);
                      }


                    }
                    

                    //value_old setzen
                    value_old = current_servo;
                  }
                  break;
          case 1: if (current_servo == 0) {
                    sprintf(output,"%s", show_current==false?OFF[lingo]:ON[lingo]);
                    if (value_old != show_current and change_value == true and a == pos) {
                      del_text = show_current==true?OFF[lingo]:ON[lingo];
                      tft.setCursor(tft.width() - 5 - tft.textWidth(del_text), 27+((a+1) * y_offset_tft));
                      tft.print(del_text);    
                      value_old = show_current;
                    }
                  }
                  else {
                    sprintf(output,"%s", "");
                    if (value_old != show_current and change_value == true and a == pos) {
                      
                      value_old = show_current;
                    }
                  }
                  break;
        }
        if (a == pos) {
          tft.setTextColor(MARKER);
        }
        else {
          tft.setTextColor(TEXT);
        }
        tft.setCursor(tft.width() - 5 - tft.textWidth(output), 27+((a+1) * y_offset_tft));
        tft.print(output);
      }
      else {
        tft.setCursor(5, 35+(7 * y_offset_tft));
        tft.print(menuitems_2[a]);
      }
      a++;
    }
  #endif
}

void HM_Display::setup_INA219_tft_save() {
  #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
    tft.setCursor(tft.width() - 5 - tft.textWidth("OK"), 35+(7 * y_offset_tft));
    tft.print("OK");
  #endif
}

void HM_Display::setup_INA219_cal1_tft(String calibration_status, bool change_value, int menuitem, int menuitem_used) {
  #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
    if (draw_frame == true) {
      tft.fillScreen(BACKGROUND);
      tft.setTextColor(TEXT);
      tft.setFreeFont(F16B[font_typ]);
      sprintf(output, "%s", INA219_SETUP[lingo]);
      calculateStringMetrics(F16B[font_typ], "MC", output, tft.width()/2, 15, x_y_h_w_b);
      tft.setCursor(x_y_h_w_b[0], x_y_h_w_b[1]);
      tft.print(output);
      tft.drawLine(0, 30, tft.width(), 30, TEXT);
      tft.setFreeFont(F12B[font_typ]);
      tft.setCursor(5, 30+(3 * y_offset_tft));
      tft.printf("%s:", MAX_CURRENT[lingo]);
      sprintf(output,"%imA", current_servo - 30);
      tft.setCursor(tft.width() - 5 - tft.textWidth(output), 30+(3 * y_offset_tft));
      tft.print(output);
      tft.setCursor(5, 30+(4 * y_offset_tft));
      tft.printf("%s:", MIN_ANGLE[lingo]);
      sprintf(output,"%i°", angle_min);
      tft.setCursor(tft.width() - 5 - tft.textWidth(output), 30+(4 * y_offset_tft));
      tft.print(output);
      draw_frame = false;
    }
    if (change_value == false && menuitem < menuitem_used && calibration_status == START[lingo]) {
      tft.setTextColor(MARKER);
      tft.setCursor(5, 30+(1 * y_offset_tft));
      tft.print(calibration_status);
      tft.setTextColor(TEXT);
      tft.setCursor(5, 30+(7 * y_offset_tft));
      tft.print(BACK[lingo]);
    }
    else {
      tft.setTextColor(TEXT);
      tft.setCursor(5, 30+(1 * y_offset_tft));
      tft.print(calibration_status);
      tft.setTextColor(MARKER);
      tft.setCursor(5, 30+(7 * y_offset_tft));
      tft.print(BACK[lingo]);
    }
  #endif
}

void HM_Display::setup_INA219_cal2_tft(String calibration_status, int cal_winkel, String quetschhan) {
  #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
    if (draw_frame == true) {
      tft.fillScreen(BACKGROUND);
      tft.setTextColor(TEXT);
      tft.setFreeFont(F16B[font_typ]);
      sprintf(output, "%s", INA219_SETUP[lingo]);
      calculateStringMetrics(F16B[font_typ], "MC", output, tft.width()/2, 15, x_y_h_w_b);
      tft.setCursor(x_y_h_w_b[0], x_y_h_w_b[1]);
      tft.print(output);
      tft.drawLine(0, 30, tft.width(), 30, TEXT);
      change = true;
      draw_frame = false;
      strcpy(del_text_1, "");
      strcpy(del_text_2, "");
      strcpy(del_text_3, "");
    }
    if (change) {
      tft.setFreeFont(F12B[font_typ]);
      tft.setTextColor(TEXT);
      tft.setCursor(5, 30+(1 * y_offset_tft));
      tft.print(calibration_status);
      tft.setCursor(5, 30+(3 * y_offset_tft));
      tft.printf("%s:", CURRENT[lingo]);
      tft.setCursor(tft.width() - 5 - tft.textWidth(del_text_1), 30+(3 * y_offset_tft));
      tft.setTextColor(BACKGROUND);
      tft.print(del_text_1);
      tft.setTextColor(TEXT);
      sprintf(output,"%imA", current_mA);
      tft.setCursor(tft.width() - 5 - tft.textWidth(output), 30+(3 * y_offset_tft));
      tft.print(output);
      sprintf(del_text_1,"%s", output);
      tft.setCursor(5, 30+(4 * y_offset_tft));
      tft.printf("%s:", ANGLE[lingo]);
      tft.setCursor(tft.width() - 5 - tft.textWidth(del_text_2), 30+(4 * y_offset_tft));
      tft.setTextColor(BACKGROUND);
      tft.print(del_text_2);
      tft.setTextColor(TEXT);
      sprintf(output,"%i°", cal_winkel);
      tft.setCursor(tft.width() - 5 - tft.textWidth(output), 30+(4 * y_offset_tft));
      tft.print(output);
      sprintf(del_text_2,"%s", output);
      tft.setCursor(5, 30+(5 * y_offset_tft));
      tft.print(HONEY_GATE[lingo]);
      tft.setCursor(tft.width() - 5 - tft.textWidth(del_text_3), 30+(5 * y_offset_tft));
      tft.setTextColor(BACKGROUND);
      tft.print(del_text_3);
      tft.setTextColor(TEXT);
      sprintf(output,"%s", quetschhan);
      tft.setCursor(tft.width() - 5 - tft.textWidth(output), 30+(5 * y_offset_tft));
      tft.print(output);
      sprintf(del_text_3,"%s", output);
      tft.setTextColor(MARKER);
      tft.setCursor(5, 30+(7 * y_offset_tft));
      tft.print(ABORT[lingo]);
      change = false;
    }
  #endif
}

void HM_Display::setup_INA219_cal3_tft(String calibration_status) {
  #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
    if (draw_frame == true) {
      tft.setTextColor(TEXT);
      tft.fillRect(0, 31, tft.width(), tft.height() -30, BACKGROUND);
      tft.setCursor(5, 30+(1 * y_offset_tft));
      tft.print(calibration_status);
      tft.setCursor(5, 30+(3 * y_offset_tft));
      tft.printf("%s:", CURRENT[lingo]);
      sprintf(output,"%imA", current_mA);
      tft.setCursor(tft.width() - 5 - tft.textWidth(output), 30+(3 * y_offset_tft));
      tft.print(output);
      tft.setCursor(5, 30+(4 * y_offset_tft));
      tft.printf("%s:", ANGLE[lingo]);
      sprintf(output,"%i°", angle_min);
      tft.setCursor(tft.width() - 5 - tft.textWidth(output), 30+(4 * y_offset_tft));
      tft.print(output);
      tft.setTextColor(MARKER);
      tft.setCursor(5, 30+(7 * y_offset_tft));
      tft.print(SAVE[lingo]);
      draw_frame = false;
    }
  #endif
}

void HM_Display::setup_about_oled(uint8_t baseMac[]) {
  #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
    char output1[30];
    u8g2.setFont(u8g2_font_courB08_tf);
    u8g2.clearBuffer();
    u8g2.setCursor(1, 10 + (0 * 13));
    sprintf(output, "%s:", VERSION[lingo]);
    u8g2.print(output);
    u8g2.setCursor(95, 10 + (0 * 13));
    sprintf(output,"%5s", VERSION_STRING);
    u8g2.print(output);
    u8g2.setCursor(1, 10 + (1 * 13));
    sprintf(output, "%s:", TURNTABLE[lingo]);
    u8g2.print(output);
    #if DREHTELLER == 0
      u8g2.setCursor(95, 10 + (1 * 13));
      sprintf(output1,"%5s", OFF[lingo]);
    #else
      u8g2.setCursor(65, 10 + (1 * 13));
      if (use_turntable == 0) {
        sprintf(output,"%s/%s", ON[lingo], OFF[lingo]);
      }
      else {
        sprintf(output,"%s/%s", ON[lingo], ON[lingo]);
      }
      sprintf(output1,"%10s", output);
    #endif
    u8g2.print(output1);
    u8g2.setCursor(1, 10 + (2 * 13));
    sprintf(output, "%s:", OTA_UPDATE[lingo]);
    u8g2.print(output);
    u8g2.setCursor(95, 10 + (2 * 13));
    #if OTA == 0
      sprintf(output,"%5s", OFF[lingo]);
    #else
      sprintf(output,"%5s", ON[lingo]);
    #endif
    u8g2.print(output);
    #if DREHTELLER == 1
      u8g2.setCursor(1, 10 + (3 * 13));
      sprintf(output, "%s:", MAC[lingo]);
      u8g2.print(output);
      u8g2.setCursor(24, 10 + (3 * 13));
      sprintf(output, "%02x:%02x:%02x:%02x:%02x:%02x", baseMac[0], baseMac[1], baseMac[2], baseMac[3], baseMac[4], baseMac[5]);
      u8g2.print(output);
      u8g2.setCursor(1, 10 + (4 * 13));
      sprintf(output, "%s:", WLAN_CHANNEL[lingo]);
      u8g2.print(output);
      u8g2.setCursor(95, 10 + (4 * 13));
      sprintf(output,"%5i", channel);
      u8g2.print(output);
    #endif
    u8g2.sendBuffer();
  #endif
}

void HM_Display::setup_about_tft(uint8_t baseMac[]) {
  #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
    tft.fillScreen(BACKGROUND);
    tft.setTextColor(TEXT);
    tft.setFreeFont(F16B[font_typ]);
    sprintf(output, "%s", ABOUT[lingo]);
    calculateStringMetrics(F16B[font_typ], "MC", output, tft.width()/2, 15, x_y_h_w_b);
    tft.setCursor(x_y_h_w_b[0], x_y_h_w_b[1]);
    tft.print(output);
    tft.drawLine(0, 30, tft.width(), 30, TEXT);
    tft.setFreeFont(F12B[font_typ]);
    tft.setCursor(5, 30+(1 * y_offset_tft));
    sprintf(output, "%s:", VERSION[lingo]);
    tft.println(output);
    sprintf(output,"%s", VERSION_STRING);
    tft.setCursor(tft.width() - 5 - tft.textWidth(output), 30+(1 * y_offset_tft));
    tft.print(output);
    tft.setCursor(5, 30+(2 * y_offset_tft));
    sprintf(output, "%s:", TURNTABLE[lingo]);
    tft.println(output);
    #if DREHTELLER == 0
      sprintf(output,"%s", OFF[lingo]);
    #else
      if (use_turntable == 0) {
        sprintf(output,"%s/%s", ON[lingo], OFF[lingo]);
      }
      else {
        sprintf(output,"%s/%s", ON[lingo], ON[lingo]);
      }
    #endif
    tft.setCursor(tft.width() - 5 - tft.textWidth(output), 30+(2 * y_offset_tft));
    tft.print(output);
    tft.setCursor(5, 30+(3 * y_offset_tft));
    sprintf(output, "%s:", OTA_UPDATE[lingo]);
    tft.println(output);
    sprintf(output,"%s", VERSION_STRING);
    #if OTA == 0
      sprintf(output,"%5s", OFF[lingo]);
    #else
      sprintf(output,"%5s", ON[lingo]);
    #endif
    tft.setCursor(tft.width() - 5 - tft.textWidth(output), 30+(3 * y_offset_tft));
    tft.print(output);
    #if DREHTELLER == 1
      tft.setCursor(5, 30+(4 * y_offset_tft));
      sprintf(output, "%s:", MAC[lingo]);
      tft.println(output);
      sprintf(output, "%02x:%02x:%02x:%02x:%02x:%02x", baseMac[0], baseMac[1], baseMac[2], baseMac[3], baseMac[4], baseMac[5]);
      tft.setCursor(tft.width() - 5 - tft.textWidth(output), 30+(4 * y_offset_tft));
      tft.print(output);
      tft.setCursor(5, 30+(5 * y_offset_tft));
      sprintf(output, "%s:", WLAN_CHANNEL[lingo]);
      tft.println(output);
      sprintf(output,"%i", channel);
      tft.setCursor(tft.width() - 5 - tft.textWidth(output), 30+(5 * y_offset_tft));
      tft.print(output);
    #endif
  #endif
}

void HM_Display::setup_clear_prefs_oled(int menu_items_number) {
  #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
    u8g2.setFont(u8g2_font_courB08_tf);
    u8g2.clearBuffer();
    u8g2.setCursor(10, 1 * y_offset);
    u8g2.print(CLEAR_PREFERENCES[lingo]);
    u8g2.setCursor(10, 2 * y_offset);
    u8g2.print(CLEAR_NVS_MEMORY[lingo]);
    #if DREHTELLER == 1
      u8g2.setCursor(10, 3 * y_offset);
      u8g2.print(RESET_TURNTABLE[lingo]);
    #endif
    u8g2.setCursor(10, (7 * y_offset) + 5);
    u8g2.print(BACK[lingo]);
    if (pos == menu_items_number - 1) {
      u8g2.setCursor(0, (7 * y_offset) + 7);
    }
    else {
      u8g2.setCursor(0, (pos + 1) * y_offset + 2);
    }
    u8g2.print("*");
    u8g2.sendBuffer();
  #endif
}

void HM_Display::setup_clear_prefs_tft(int menu_items_number, const char *menuitems[]) {
  #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
    if (draw_frame == true) {
      tft.fillScreen(BACKGROUND);
      tft.setTextColor(TEXT);
      tft.setFreeFont(F16B[font_typ]);
      sprintf(output, "%s", CLEAR_PREFERENCES[lingo]);
      calculateStringMetrics(F16B[font_typ], "MC", output, tft.width()/2, 15, x_y_h_w_b);
      tft.setCursor(x_y_h_w_b[0], x_y_h_w_b[1]);
      tft.print(output);
      tft.drawLine(0, 32, tft.width(), 32, TEXT);
      draw_frame = false;
    }
    int j = 0;
    tft.setFreeFont(F12B[font_typ]);
    while(j < menu_items_number) {
      if (j == pos) {
        tft.setTextColor(MARKER);
      }
      else {
        tft.setTextColor(TEXT);
      }
      if (j < menu_items_number - 1) {
        tft.setCursor(5, 30+((j+1) * y_offset_tft));
        tft.print(menuitems[j]);
      }
      else {
        tft.setCursor(5, 30+(7 * y_offset_tft));
        tft.print(menuitems[j]);
      }
      j++;
    }
  #endif
}

void HM_Display::setup_clear_prefs_oled_exit() {
  #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
    u8g2.setCursor(115, (7 * y_offset) + 5);   
    u8g2.print("OK");
    u8g2.sendBuffer();
  #endif
}

void HM_Display::setup_clear_prefs_tft_exit() {
  #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
    tft.setCursor(tft.width() - 5 - tft.textWidth("OK"), 35+(7 * y_offset_tft));
    tft.print("OK");
  #endif
}

void HM_Display::setup_setup_oled(const char *menuitems[], int menu_items_number) {
  #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
    u8g2.setFont(u8g2_font_courB08_tf);
    u8g2.clearBuffer();
    int i = 0;
    while(i < menu_items_number) {
      if (i == pos and i < menu_items_number - 1) {
        u8g2.setCursor(0, (pos + 1) * y_offset + 2);
        u8g2.print("*");
      }
      else if (i == pos and i == menu_items_number - 1) {
        u8g2.setCursor(0, (7 * y_offset) + 7);
        u8g2.print("*");
      }
      if (i < menu_items_number - 1) {
        u8g2.setCursor(10, (i + 1) * y_offset);
        u8g2.print(menuitems[i]);
      }
      else if (i == menu_items_number - 1) {
        u8g2.setCursor(10, (7 * y_offset) + 5);
        u8g2.print(menuitems[i]);
      }

      i++;
    }
    u8g2.sendBuffer();
  #endif
}

void HM_Display::setup_setup_tft(const char *menuitems[], int menu_items_number) {
  #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
    if (draw_frame == true) {
      tft.fillScreen(BACKGROUND);
      tft.setTextColor(TEXT);
      tft.setFreeFont(F16B[font_typ]);
      sprintf(output, "%s", SETUP[lingo]);
      calculateStringMetrics(F16B[font_typ], "MC", output, tft.width()/2, 15, x_y_h_w_b);
      tft.setCursor(x_y_h_w_b[0], x_y_h_w_b[1]);
      tft.print(output);
      tft.drawLine(0, 32, tft.width(), 32, TEXT);
      draw_frame = false;
    }
    int j = 0;
    tft.setFreeFont(F12B[font_typ]);
    while(j < menu_items_number) {
      if (j == pos) {
        tft.setTextColor(MARKER);
      }
      else {
        tft.setTextColor(TEXT);
      }
      if (j < menu_items_number - 1) {
        tft.setCursor(5, 30+((j+1) * y_offset_tft));
        tft.print(menuitems[j]);
      }
      else {
        tft.setCursor(5, 30+(7 * y_offset_tft));
        tft.print(menuitems[j]);
      }
      j++;
    }


  #endif
}

void HM_Display::setup_setup_oled_exit() {
  #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
    u8g2.setCursor(115, (7 * y_offset) + 5);   
    u8g2.print("OK");
    u8g2.sendBuffer();
  #endif
}

void HM_Display::setup_setup_tft_exit() {
  #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
    tft.setCursor(tft.width() - 5 - tft.textWidth("OK"), 35+(7 * y_offset_tft));
    tft.print("OK");
  #endif
}

void HM_Display::ota_setup1_oled(){
  #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_courB14_tf);
    sprintf(output, "Start WiFi");
    x_pos = CenterPosX(output, 11, 128);
    u8g2.setCursor(x_pos, 25);
    u8g2.print(output);
  #endif
}

void HM_Display::ota_setup2_oled(char x[]) {
  #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
    x_pos = CenterPosX(x, 11, 128);
    u8g2.setCursor(x_pos, 45);
    u8g2.print(x);
    u8g2.sendBuffer();
  #endif
}

void HM_Display::ota_setup3_oled(char ip[]) {
  #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_courB08_tf);
    sprintf(output, ip);
    x_pos = CenterPosX(output, 6, 128);
    u8g2.setCursor(x_pos, 35);
    u8g2.print(output);
    u8g2.sendBuffer();
  #endif
}

void HM_Display::ota_setup4_oled() {
  #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_courB14_tf);
    sprintf(output, "Start WiFi");
    x_pos = CenterPosX(output, 11, 128);
    u8g2.setCursor(x_pos, 25);
    u8g2.print(output);
    sprintf(output, "failed");
    x_pos = CenterPosX(output, 11, 128);
    u8g2.setCursor(x_pos, 45);
    u8g2.print(output);
    u8g2.sendBuffer();
  #endif
}

void HM_Display::on_ota_start_oled() {
  #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_courB08_tf);
    sprintf(output, "Update started");
    x_pos = CenterPosX(output, 6, 128);
    u8g2.setCursor(x_pos, 35);
    u8g2.print(output);
    u8g2.sendBuffer();
  #endif
}

void HM_Display::on_ota_progress_oled(size_t current, size_t final) {
  #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_courB08_tf);
    u8g2.setCursor(1, 10);
    u8g2.print("OTA Progress:");
    u8g2.setCursor(1, 25);
    sprintf(output, "Current: %u", current);
    u8g2.print(output);
    u8g2.setCursor(1, 35);
    sprintf(output, "Final:   %u", final);
    u8g2.print(output);
    u8g2.sendBuffer();
  #endif
}

void HM_Display::on_on_end1_oled(bool success) {
  #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_courB08_tf);
    sprintf(output, "Successfully");
    x_pos = CenterPosX(output, 6, 128);
    u8g2.setCursor(x_pos, 35);
    u8g2.print(output);
    u8g2.sendBuffer();
  #endif
}

void HM_Display::on_on_end2_oled() {
  #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
    u8g2.clearBuffer();
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_courB08_tf);
    sprintf(output, "Error");
    x_pos = CenterPosX(output, 6, 128);
    u8g2.setCursor(x_pos, 35);
    u8g2.print(output);
    u8g2.sendBuffer();
  #endif
}

void HM_Display::ota_disconnect_oled() {
  #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_courB14_tf);
    sprintf(output, "Stop WiFi");
    x_pos = CenterPosX(output, 11, 128);
    u8g2.setCursor(x_pos, 35);
    u8g2.print(output);
    u8g2.sendBuffer();
  #endif
}

void HM_Display::ota_setup1_tft() {
  #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
    tft.fillScreen(BACKGROUND);
    tft.setTextColor(TEXT);
    tft.setFreeFont(F16B[font_typ]);
    sprintf(output, "%s", "ElegantOTA");
    calculateStringMetrics(F16B[font_typ], "MC", output, tft.width()/2, 15, x_y_h_w_b);
    tft.setCursor(x_y_h_w_b[0], x_y_h_w_b[1]);
    tft.print(output);
    tft.drawLine(0, 32, tft.width(), 32, TEXT);
    strcpy(del_text_1, "");
  #endif
}

void HM_Display::ota_setup2_tft(char x[]) {
  #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
    float a = 0.7;
    tft.setTextColor(TEXT);
    tft.setFreeFont(F20B[font_typ]);
    sprintf(output, "Start WiFi");
    calculateStringMetrics(F20B[font_typ], "MC", output, tft.width()/2, 136, x_y_h_w_b);
    tft.setCursor(x_y_h_w_b[0], x_y_h_w_b[1] - x_y_h_w_b[2] * a);
    tft.print(output);
    tft.setTextColor(BACKGROUND);
    calculateStringMetrics(F20B[font_typ], "MC", del_text_1, tft.width()/2, 136, x_y_h_w_b);
    tft.setCursor(x_y_h_w_b[0], x_y_h_w_b[1] + x_y_h_w_b[2] * a);
    tft.print(del_text_1);
    tft.setTextColor(TEXT);
    calculateStringMetrics(F20B[font_typ], "MC", x, tft.width()/2, 136, x_y_h_w_b);
    tft.setCursor(x_y_h_w_b[0], x_y_h_w_b[1] + x_y_h_w_b[2] * a);
    tft.print(x);
    sprintf(del_text_1,"%s", x);
  #endif
}

void HM_Display::ota_setup3_tft(char ip[]) {
  #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
    tft.fillRect(0, 33, tft.width(), tft.height()-33, BACKGROUND);
    tft.setTextColor(TEXT);
    tft.setFreeFont(F20B[font_typ]);
    sprintf(output, ip);
    calculateStringMetrics(F20B[font_typ], "MC", output, tft.width()/2, 136, x_y_h_w_b);
    tft.setCursor(x_y_h_w_b[0], x_y_h_w_b[1]);
    tft.print(output);
  #endif
}

void HM_Display::ota_setup4_tft() {
  #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
    float a = 0.7;
    tft.setFreeFont(F20B[font_typ]);
    tft.setTextColor(BACKGROUND);
    calculateStringMetrics(F20B[font_typ], "MC", del_text_1, tft.width()/2, 136, x_y_h_w_b);
    tft.setCursor(x_y_h_w_b[0], x_y_h_w_b[1] + x_y_h_w_b[2] * a);
    tft.print(del_text_1);
    tft.setTextColor(TFT_RED);
    sprintf(output, "failed");
    calculateStringMetrics(F20B[font_typ], "MC", output, tft.width()/2, 136, x_y_h_w_b);
    tft.setCursor(x_y_h_w_b[0], x_y_h_w_b[1] + x_y_h_w_b[2] * a);
    tft.print(output);
    tft.setTextColor(TEXT);
  #endif
}

void HM_Display::ota_on_ota_start_tft() {
  #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
    tft.fillScreen(BACKGROUND);
    tft.setTextColor(TEXT);
    tft.setFreeFont(F16B[font_typ]);
    sprintf(output, "%s", "Update started");
    calculateStringMetrics(F16B[font_typ], "MC", output, tft.width()/2, 15, x_y_h_w_b);
    tft.setCursor(x_y_h_w_b[0], x_y_h_w_b[1]);
    tft.print(output);
    tft.drawLine(0, 32, tft.width(), 32, TEXT);
    strcpy(del_text_1, "");
    strcpy(del_text_2, "");
  #endif
}

void HM_Display::ota_on_ota_progress_tft(size_t current, size_t final) {
  #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
    float a = 0.7;
    if (strcmp(del_text_1, "") == 0) {
      tft.fillRect(0, 33, tft.width(), tft.height()-33, BACKGROUND);
      tft.setTextColor(TEXT);
      tft.setFreeFont(F14B[font_typ]);
      sprintf(output, "%s", "OTA Progress:");
      calculateStringMetrics(F14B[font_typ], "BL", output, 5, 136, x_y_h_w_b);
      tft.setCursor(x_y_h_w_b[0], x_y_h_w_b[1] - 2 * x_y_h_w_b[2]);
      tft.print(output);
      sprintf(output, "%s", "Current:");
      calculateStringMetrics(F14B[font_typ], "BL", output, 5, 136, x_y_h_w_b);
      longest_string_1 = x_y_h_w_b[3];
      tft.setCursor(x_y_h_w_b[0], x_y_h_w_b[1] - x_y_h_w_b[2] * a);
      tft.print(output);
      sprintf(output, "%s", "Final:");
      calculateStringMetrics(F14B[font_typ], "BL", output, 5, 136, x_y_h_w_b);
      if (longest_string_1 < x_y_h_w_b[3]) {
        longest_string_1 = x_y_h_w_b[3];
      }
      tft.setCursor(x_y_h_w_b[0], x_y_h_w_b[1] + x_y_h_w_b[2] * a);
      tft.print(output);
    }
    tft.setTextColor(BACKGROUND);
    calculateStringMetrics(F14B[font_typ], "BL", del_text_1, longest_string_1 + 20, 136, x_y_h_w_b);
    tft.setCursor(x_y_h_w_b[0], x_y_h_w_b[1] - x_y_h_w_b[2] * a);
    tft.print(del_text_1);
    calculateStringMetrics(F14B[font_typ], "BL", del_text_2, longest_string_1 + 20, 136, x_y_h_w_b);
    tft.setCursor(x_y_h_w_b[0], x_y_h_w_b[1] + x_y_h_w_b[2] * a);
    tft.print(del_text_2);
    tft.setTextColor(TEXT);
    sprintf(del_text_1, "%u", current);
    sprintf(del_text_2, "%u", final);
    calculateStringMetrics(F14B[font_typ], "BL", del_text_1, longest_string_1 + 20, 136, x_y_h_w_b);
    tft.setCursor(x_y_h_w_b[0], x_y_h_w_b[1] - x_y_h_w_b[2] * a);
    tft.print(del_text_1);
    calculateStringMetrics(F14B[font_typ], "BL", del_text_2, longest_string_1 + 20, 136, x_y_h_w_b);
    tft.setCursor(x_y_h_w_b[0], x_y_h_w_b[1] + x_y_h_w_b[2] * a);
    tft.print(del_text_2);
  #endif
}

void HM_Display::ota_on_on_end1_tft(bool success) {
  #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
    tft.fillRect(0, 33, tft.width(), tft.height()-33, BACKGROUND);
    tft.setTextColor(TFT_GREEN);
    tft.setFreeFont(F20B[font_typ]);
    sprintf(output, "Successfully");
    calculateStringMetrics(F20B[font_typ], "MC", output, tft.width()/2, 136, x_y_h_w_b);
    tft.setCursor(x_y_h_w_b[0], x_y_h_w_b[1]);
    tft.print(output);
  #endif
}

void HM_Display::ota_on_on_end2_tft() {
  #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
    tft.fillRect(0, 33, tft.width(), tft.height()-33, BACKGROUND);
    tft.setTextColor(TFT_RED);
    tft.setFreeFont(F20B[font_typ]);
    sprintf(output, "Error");
    calculateStringMetrics(F20B[font_typ], "MC", output, tft.width()/2, 136, x_y_h_w_b);
    tft.setCursor(x_y_h_w_b[0], x_y_h_w_b[1]);
    tft.print(output);
  #endif
}

void HM_Display::ota_disconnect_tft() {
  #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
    tft.fillRect(0, 33, tft.width(), tft.height()-33, BACKGROUND);
    tft.setTextColor(TEXT);
    tft.setFreeFont(F20B[font_typ]);
    sprintf(output, "Stop WiFi");
    calculateStringMetrics(F20B[font_typ], "MC", output, tft.width()/2, 136, x_y_h_w_b);
    tft.setCursor(x_y_h_w_b[0], x_y_h_w_b[1]);
    tft.print(output);
  #endif
}

void HM_Display::setup_turntable_no_connection_oled() {
  #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_courB08_tf);
    x_pos = CenterPosX(CONNECTION[lingo], 6, 128);
    u8g2.setCursor(x_pos,22);  
    u8g2.print(CONNECTION[lingo]);
    x_pos = CenterPosX(FAILED[lingo], 6, 128);
    u8g2.setCursor(x_pos,42);  
    u8g2.print(FAILED[lingo]);
    u8g2.sendBuffer();
  #endif
}

void HM_Display::setup_turntable_menu1_1_oled() {
  #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
      u8g2.clearBuffer();
      u8g2.sendBuffer();
  #endif
}

void HM_Display::setup_turntable_menu1_2_oled(bool change_value, const char *menuitems_1[], int menuitem_1, int menu_items_number_1, int last_menu_pos_1, bool turntable_running, String esp_now_wait){
  #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_courB08_tf);
    u8g2.setCursor(10, 1 * y_offset);
    u8g2.print(menuitems_1[0]);
    u8g2.setCursor(105, 1 * y_offset);
    sprintf(output,"%3s", (use_turntable==0?OFF[lingo]:ON[lingo]));
    u8g2.print(output);
    if (use_turntable == 1 and turntable_init == false) {
      u8g2.setCursor(10, 2 * y_offset);
      u8g2.print(menuitems_1[1]);
      if (last_menu_pos_1 == 1 and turntable_running == true) {
        u8g2.setCursor(105, 2 * y_offset);
        u8g2.print(esp_now_wait);
      }
      else {
        u8g2.setCursor(105, 2 * y_offset);
        sprintf(output,"%3s", (turntable_init==false?NOKAY:OKKAY));
        u8g2.print(output);
      }
    }
    u8g2.setCursor(10, (7 * y_offset) + 5);
    u8g2.print(menuitems_1[2]);
    if (change_value == false && menuitem_1 < menu_items_number_1 - 1) {
      u8g2.setCursor(1, 10+((menuitem_1)*y_offset)); u8g2.print("*");
    }
    else if (change_value == true && menuitem_1 < menu_items_number_1 - 1) {
      u8g2.setCursor(1, 8+((menuitem_1)*y_offset)); u8g2.print("-");
    }
    else if (change_value == false && menuitem_1 == 7) {
      u8g2.setCursor(1, 10+((menuitem_1 - 1)*y_offset) + 5); u8g2.print("*");
    }
    u8g2.sendBuffer();
  #endif
}

void HM_Display::setup_turntable_menu1_oled_exit(int menuitem_1) {
  #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
    u8g2.setCursor(111, (menuitem_1 * y_offset) + 5);
    u8g2.print("OK");
    u8g2.sendBuffer();
  #endif
}

void HM_Display::setup_turntable_menu2_1_oled() {
  #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
      u8g2.clearBuffer();
      u8g2.sendBuffer();
  #endif
}

void HM_Display::setup_turntable_menu2_2_oled(bool change_value, const char *menuitems_2[], int menuitem_2, int menu_items_number_2, int ota_update) {
  #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_courB08_tf);
    u8g2.setCursor(10, 1 * y_offset);
    u8g2.print(menuitems_2[0]);
    u8g2.setCursor(105, 1 * y_offset);
    sprintf(output,"%3s", (use_turntable==0?OFF[lingo]:ON[lingo]));
    u8g2.print(output);
    if (use_turntable == 1) {
      u8g2.setCursor(10, 2 * y_offset);
      u8g2.print(menuitems_2[1]);
      u8g2.setCursor(10, 3 * y_offset);
      u8g2.print(menuitems_2[2]);
      if (ota_update == 1) {
        u8g2.setCursor(10, 4 * y_offset);
        u8g2.print(menuitems_2[3]);
      }
    }
    u8g2.setCursor(10, (7 * y_offset) + 5);
    u8g2.print(menuitems_2[3 + ota_update]);
    if (change_value == false && menuitem_2 < menu_items_number_2 - 1) {
      u8g2.setCursor(1, 10+((menuitem_2)*y_offset)); u8g2.print("*");
    }
    else if (change_value == true && menuitem_2 < menu_items_number_2 - 1) {
      u8g2.setCursor(1, 8+((menuitem_2)*y_offset)); u8g2.print("-");
    }
    else if (change_value == false && menuitem_2 == 7) {
      u8g2.setCursor(1, 10+((menuitem_2 - 1)*y_offset) + 5); u8g2.print("*");
    }
    u8g2.sendBuffer();
  #endif
}

void HM_Display::setup_turntable_menu2_3_oled() {
  #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
    
  #endif
}

void HM_Display::setup_turntable_menu2_4_oled() {
  #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_courB14_tf);
    sprintf(output, "Start WiFi");
    x_pos = CenterPosX(output, 11, 128);
    u8g2.setCursor(x_pos, 35);
    u8g2.print(output);
    u8g2.sendBuffer();
  #endif
}

void HM_Display::setup_turntable_menu2_5_oled(char myReceivedMessage_text[]) {
  #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_courB08_tf);
    sprintf(output, myReceivedMessage_text);
    x_pos = CenterPosX(output, 6, 128);
    u8g2.setCursor(x_pos, 35);
    u8g2.print(output);
    u8g2.sendBuffer();
  #endif
}

void HM_Display::setup_turntable_menu2_6_oled(char myReceivedMessage_text[], int myReceivedMessage_value) {
  #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_courB08_tf);
    u8g2.setCursor(1, 10);
    u8g2.print("OTA Progress:");
    u8g2.setCursor(1, 25);
    sprintf(output, "Current: %s", myReceivedMessage_text);
    u8g2.print(output);
    u8g2.setCursor(1, 35);
    sprintf(output, "Final:   %i", myReceivedMessage_value);
    u8g2.print(output);
    u8g2.sendBuffer();
  #endif
}

void HM_Display::setup_turntable_menu2_7_oled() {
  #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_courB08_tf);
    sprintf(output, "Successfully");
    x_pos = CenterPosX(output, 6, 128);
    u8g2.setCursor(x_pos, 35);
    u8g2.print(output);
    u8g2.sendBuffer();
  #endif
}

void HM_Display::setup_turntable_menu2_8_oled() {
  #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
    u8g2.clearBuffer();
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_courB08_tf);
    sprintf(output, "Error");
    x_pos = CenterPosX(output, 6, 128);
    u8g2.setCursor(x_pos, 35);
    u8g2.print(output);
    u8g2.sendBuffer();
  #endif
}

void HM_Display::setup_turntable_menu2_9_oled() {
  #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_courB14_tf);
    sprintf(output, "Start WiFi");
    x_pos = CenterPosX(output, 11, 128);
    u8g2.setCursor(x_pos, 25);
    u8g2.print(output);
    sprintf(output, "failed");
    x_pos = CenterPosX(output, 11, 128);
    u8g2.setCursor(x_pos, 45);
    u8g2.print(output);
    u8g2.sendBuffer();
  #endif
}

void HM_Display::setup_turntable_menu2_oled_exit(int menuitem_2) {
  #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
    u8g2.setCursor(111, (menuitem_2 * y_offset) + 5);
    u8g2.print("OK");
    u8g2.sendBuffer();
  #endif
}

void HM_Display::setup_turntable_menu3_1_oled() {
  #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
      u8g2.clearBuffer();
      u8g2.sendBuffer();
  #endif
}

void HM_Display::setup_turntable_menu3_2_oled(bool change_value, const char *menuitems_3[], int menuitem_3, int menu_items_number_3, String esp_now_wait, int jar_center_pos, int speed_init, int speed_run) {
  #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_courB08_tf);
    u8g2.setCursor(10, 1 * y_offset);
    u8g2.print(menuitems_3[0]);
    u8g2.setCursor(105, 1 * y_offset);
    sprintf(output,"%4s", esp_now_wait);
    u8g2.print(output);
    u8g2.setCursor(10, 2 * y_offset);
    u8g2.print(menuitems_3[1]);
    u8g2.setCursor(105, 2 * y_offset);
    sprintf(output,"%4i", jar_center_pos);
    u8g2.print(output);
    u8g2.setCursor(10, 3 * y_offset);
    u8g2.print(menuitems_3[2]);
    u8g2.setCursor(105, 3 * y_offset);
    sprintf(output,"%4i", speed_init);
    u8g2.print(output);
    u8g2.setCursor(10, 4 * y_offset);
    u8g2.print(menuitems_3[3]);
    u8g2.setCursor(105, 4 * y_offset);
    sprintf(output,"%4i", speed_run);
    u8g2.print(output);
    u8g2.setCursor(10, (7 * y_offset) + 5);
    u8g2.print(menuitems_3[4]);
    if (change_value == false && menuitem_3 < menu_items_number_3 - 1) {
      u8g2.setCursor(1, 10+((menuitem_3)*y_offset)); u8g2.print("*");
    }
    else if (change_value == true && menuitem_3 < menu_items_number_3 - 1) {
      u8g2.setCursor(1, 8+((menuitem_3)*y_offset)); u8g2.print("-");
    }
    else if (change_value == false && menuitem_3 == 7) {
      u8g2.setCursor(1, 10+((menuitem_3 - 1)*y_offset) + 5); u8g2.print("*");
    }
    u8g2.sendBuffer();
  #endif
}

void HM_Display::setup_turntable_menu3_oled_exit(int menuitem_3) {
  #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
    u8g2.setCursor(111, (menuitem_3 * y_offset) + 5);
    u8g2.print("OK");
    u8g2.sendBuffer();
  #endif
}

void HM_Display::setup_turntable_menu4_1_oled() {
  #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
      u8g2.clearBuffer();
      u8g2.sendBuffer();
  #endif
}

void HM_Display::setup_turntable_menu4_2_oled(bool change_value, const char *menuitems_4[], int menuitem_4, int menu_items_number_4, int ts_speed, int ts_waittime, int ts_angle_min, int ts_angle_max) {
  #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_courB08_tf);
    u8g2.setCursor(10, 1 * y_offset);
    u8g2.print(menuitems_4[0]);
    u8g2.setCursor(10, 2 * y_offset);
    u8g2.print(menuitems_4[1]);
    u8g2.setCursor(10, 3 * y_offset);
    u8g2.print(menuitems_4[2]);
    u8g2.setCursor(105, 3 * y_offset);
    sprintf(output,"%4i", ts_speed);
    u8g2.print(output);
    u8g2.setCursor(10, 4 * y_offset);
    u8g2.print(menuitems_4[3]);
    u8g2.setCursor(105, 4 * y_offset);
    sprintf(output,"%4i", ts_waittime);
    u8g2.print(output);
    u8g2.setCursor(10, 5 * y_offset);
    u8g2.print(menuitems_4[4]);
    u8g2.setCursor(105, 5 * y_offset);
    sprintf(output,"%4i", ts_angle_min);
    u8g2.print(output);
    u8g2.setCursor(10, 6 * y_offset);
    u8g2.print(menuitems_4[5]);
    u8g2.setCursor(105, 6 * y_offset);
    sprintf(output,"%4i", ts_angle_max);
    u8g2.print(output);
    u8g2.setCursor(10, (7 * y_offset) + 5);
    u8g2.print(menuitems_4[6]);
    if (change_value == false && menuitem_4 < menu_items_number_4 - 1) {
      u8g2.setCursor(1, 10+((menuitem_4)*y_offset)); u8g2.print("*");
    }
    else if (change_value == true && menuitem_4 < menu_items_number_4 - 1) {
        u8g2.setCursor(1, 8+((menuitem_4)*y_offset)); u8g2.print("-");
    }
    else if (change_value == false && menuitem_4 == 7) {
      u8g2.setCursor(1, 10+((menuitem_4 - 1)*y_offset) + 5); u8g2.print("*");
    }
    u8g2.sendBuffer();
  #endif
}

void HM_Display::setup_turntable_menu4_oled_exit(int menuitem_4) {
  #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
    u8g2.setCursor(111, (menuitem_4 * y_offset) + 5);
    u8g2.print("OK");
    u8g2.sendBuffer();
  #endif
}

void HM_Display::setup_turntable_frame() {
  #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
    
  #endif
}

void HM_Display::setup_turntable_no_connection_tft(){
  #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
    
  #endif
}

void HM_Display::setup_turntable_menu1_1_tft() {
  #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
   
  #endif
}

void HM_Display::setup_turntable_menu1_2_tft(bool change_value, const char *menuitems_1[], int menu_items_number_1, bool turntable_running) {
  #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
    
  #endif
}

void HM_Display::setup_turntable_menu1_tft_exit() {
  #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
   
  #endif
}

void HM_Display::setup_turntable_menu2_1_tft() {
  #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
    
  #endif
}

void HM_Display::setup_turntable_menu2_2_tft(bool change_value, const char *menuitems_2[], int menu_items_number_2, int ota_update) {
  #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
    
  #endif
}

void HM_Display::setup_turntable_menu2_3_tft() {
  #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
   
  #endif
}

void HM_Display::setup_turntable_menu2_4_tft() {
  #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
    
  #endif
}

void HM_Display::setup_turntable_menu2_5_tft(char myReceivedMessage_text[]) {
  #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
   
  #endif
}

void HM_Display::setup_turntable_menu2_6_tft(char myReceivedMessage_text[], int myReceivedMessage_value) {
  #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
    
  #endif
}

void HM_Display::setup_turntable_menu2_7_tft() {
  #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
    
  #endif
}

void HM_Display::setup_turntable_menu2_8_tft() {
  #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
    
  #endif
}

void HM_Display::setup_turntable_menu2_9_tft(int a) {
  #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
    
  #endif
}

void HM_Display::setup_turntable_menu2_tft_exit() {
  #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
   
  #endif
}

void HM_Display::setup_turntable_menu3_1_tft() {
  #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
    
  #endif
}

void HM_Display::setup_turntable_menu3_2_tft(bool change_value, const char *menuitems_3[], int menu_items_number_3, bool turntable_running, int jar_center_pos, int speed_init, int speed_run) {
  #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
    
  #endif
}

void HM_Display::setup_turntable_menu3_tft_exit() {
  #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
    
  #endif
}

void HM_Display::setup_turntable_menu4_1_tft() {
  #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
    
  #endif
}

void HM_Display::setup_turntable_menu4_2_tft(bool change_value, const char *menuitems_4[], int menu_items_number_4, int ts_speed, int ts_waittime, int ts_angle_min, int ts_angle_max) {
  #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
    
  #endif
}

void HM_Display::setup_turntable_menu4_tft_exit() {
  #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
   
  #endif
}

void HM_Display::setup_webif_oled(int i) {
  #if DISPLAY_TYPE == 991 || DISPLAY_TYPE == 992 || DISPLAY_TYPE == 999
    if (i == 1) { //Clrat display
      x = "";
      u8g2.clearBuffer();
      u8g2.sendBuffer();
    }
    if (i == 2 || i == 3) { //Start WiFi / Access Point
      Serial.println();
      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_courB14_tf);
      if (WiFi.getMode() == WIFI_MODE_STA) {sprintf(output, START_WIFI[lingo]);}
      else {sprintf(output, START_AP[lingo]);}
      x_pos = CenterPosX(output, 11, 128);
      u8g2.setCursor(x_pos, 25);
      u8g2.print(output);
      if (WiFi.getMode() == WIFI_AP) {
        sprintf(output, PLEACE_WAIT[lingo]);
        x_pos = CenterPosX(output, 11, 128);
        u8g2.setCursor(x_pos, 45);
        u8g2.print(output);
      }
      u8g2.sendBuffer();
    }
    if (i == 3) { //Draw "*"
      if (x.length() > 10) {x = "*";}
      else {x += "*";}
      x_pos = CenterPosX(x.c_str(), 11, 128);
      u8g2.setCursor(x_pos, 45);
      u8g2.print(x);
      u8g2.sendBuffer();
    }
    if (i == 4) { //Show IP adress
      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_courB08_tf);
      String ipStr = WiFi.localIP().toString();  // IP-Adresse als String
      ipStr.toCharArray(output, sizeof(output)); // In char-Array umwandeln
      x_pos = CenterPosX(output, 6, 128);
      u8g2.setCursor(x_pos, 35);
      u8g2.print(output);
      u8g2.sendBuffer();
    }
    if (i == 5) { //Stop Wifi
      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_courB14_tf);
      if (WiFi.getMode() == WIFI_MODE_STA) {sprintf(output, STOP_WIFI[lingo]);}
      else {sprintf(output, STOP_AP[lingo]);}
      x_pos = CenterPosX(output, 11, 128);
      u8g2.setCursor(x_pos, 35);
      u8g2.print(output);
      u8g2.sendBuffer();
    }
    if (i == 6) { //Acces Point data
      u8g2.clearBuffer();
      u8g2.setFont(u8g2_font_courB08_tf);
      sprintf(output, "%s:", SSID[lingo]);
      x_pos = CenterPosX(output, 6, 128);
      u8g2.setCursor(x_pos, 8);
      u8g2.print(output);
      sprintf(output, apSSID);
      x_pos = CenterPosX(output, 6, 128);
      u8g2.setCursor(x_pos, 18);
      u8g2.print(output);
      sprintf(output, "%s:", PASSWORD[lingo]);
      x_pos = CenterPosX(output, 6, 128);
      u8g2.setCursor(x_pos, 30);
      u8g2.print(output);
      sprintf(output, apPassword);
      x_pos = CenterPosX(output, 6, 128);
      u8g2.setCursor(x_pos, 40);
      u8g2.print(output);
      sprintf(output, "%s:", IP_ADDRESS[lingo]);
      x_pos = CenterPosX(output, 6, 128);
      u8g2.setCursor(x_pos, 52);
      u8g2.print(output);
      String ipStr = WiFi.softAPIP().toString();  // IP-Adresse als String
      ipStr.toCharArray(output, sizeof(output));  // In char-Array umwandeln
      x_pos = CenterPosX(output, 6, 128);
      u8g2.setCursor(x_pos, 62);
      u8g2.print(output);
      u8g2.sendBuffer();
    }
  #endif
}

void HM_Display::setup_webif_tft(int i) {
  #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
    if (i == 1) { //Draw Frame
      tft.fillScreen(BACKGROUND);
      tft.setTextColor(TEXT);
      tft.setFreeFont(F16B[font_typ]);
      sprintf(output, "%s", WIFI_SETUP[lingo]);
      calculateStringMetrics(F16B[font_typ], "MC", output, tft.width()/2, 15, x_y_h_w_b);
      tft.setCursor(x_y_h_w_b[0], x_y_h_w_b[1]);
      tft.print(output);
      tft.drawLine(0, 32, tft.width(), 32, TEXT);
    }
    if (i == 2) { //Start WiFi
      float a = 0.7;
      tft.fillRect(0, 33, tft.width(), tft.height()-33, BACKGROUND);
      tft.setTextColor(TEXT);
      tft.setFreeFont(F20B[font_typ]);
      if (WiFi.getMode() == WIFI_MODE_STA) {sprintf(output, START_WIFI[lingo]);}
      else {sprintf(output, START_AP[lingo]);}
      calculateStringMetrics(F20B[font_typ], "MC", output, tft.width()/2, 136, x_y_h_w_b);
      tft.setCursor(x_y_h_w_b[0], x_y_h_w_b[1] - x_y_h_w_b[2] * a);
      tft.print(output);
      if (WiFi.getMode() == WIFI_AP) {
        sprintf(output, PLEACE_WAIT[lingo]);
        x_pos = CenterPosX(output, 11, 128);
        calculateStringMetrics(F20B[font_typ], "MC", output, tft.width()/2, 136, x_y_h_w_b);
        tft.setCursor(x_y_h_w_b[0], x_y_h_w_b[1] + x_y_h_w_b[2] * a);
        tft.print(output);
      }
    }
    if (i == 3) { //Draw "*"
      float a = 0.7;
      if (x.length() > 10) {x = "*";}
      else {x += "*";}
      tft.setTextColor(BACKGROUND);
      calculateStringMetrics(F20B[font_typ], "MC", del_text_1, tft.width()/2, 136, x_y_h_w_b);
      tft.setCursor(x_y_h_w_b[0], x_y_h_w_b[1] + x_y_h_w_b[2] * a);
      tft.print(del_text_1);
      tft.setTextColor(TEXT);
      calculateStringMetrics(F20B[font_typ], "MC", x.c_str(), tft.width()/2, 136, x_y_h_w_b);
      tft.setCursor(x_y_h_w_b[0], x_y_h_w_b[1] + x_y_h_w_b[2] * a);
      tft.print(x);
      sprintf(del_text_1,"%s", x);
    }
    if (i == 4) { //Show IP adress
      tft.fillRect(0, 33, tft.width(), tft.height()-33, BACKGROUND);
      tft.setTextColor(TEXT);
      tft.setFreeFont(F20B[font_typ]);
      String ipStr = WiFi.localIP().toString();  // IP-Adresse als String
      ipStr.toCharArray(output, sizeof(output)); // In char-Array umwandeln
      calculateStringMetrics(F20B[font_typ], "MC", output, tft.width()/2, 136, x_y_h_w_b);
      tft.setCursor(x_y_h_w_b[0], x_y_h_w_b[1]);
      tft.print(output);
    }
    if (i == 5) { //Stop WiFi
      tft.fillRect(0, 33, tft.width(), tft.height()-33, BACKGROUND);
      tft.setTextColor(TEXT);
      tft.setFreeFont(F20B[font_typ]);
      if (WiFi.getMode() == WIFI_MODE_STA) {sprintf(output, STOP_WIFI[lingo]);}
      else {sprintf(output, STOP_AP[lingo]);}
      calculateStringMetrics(F20B[font_typ], "MC", output, tft.width()/2, 136, x_y_h_w_b);
      tft.setCursor(x_y_h_w_b[0], x_y_h_w_b[1]);
      tft.print(output);
    }
    if (i == 6) { //Acces Point data
      tft.fillRect(0, 33, tft.width(), tft.height()-33, BACKGROUND);
      tft.setFreeFont(F12B[font_typ]);
      sprintf(output, "%s:", SSID[lingo]);
      calculateStringMetrics(F12B[font_typ], "MC", output, tft.width() / 2, 27 + (1 * y_offset_tft), x_y_h_w_b);
      tft.setCursor(x_y_h_w_b[0], x_y_h_w_b[1]);
      tft.print(output);
      sprintf(output, apSSID);
      calculateStringMetrics(F12B[font_typ], "MC", output, tft.width() / 2, 27 + (2 * y_offset_tft), x_y_h_w_b);
      tft.setCursor(x_y_h_w_b[0], x_y_h_w_b[1]);
      tft.print(output);
      sprintf(output, "%s:", PASSWORD[lingo]);
      calculateStringMetrics(F12B[font_typ], "MC", output, tft.width() / 2, 37 + (3 * y_offset_tft), x_y_h_w_b);
      tft.setCursor(x_y_h_w_b[0], x_y_h_w_b[1]);
      tft.print(output);
      sprintf(output, apPassword);
      calculateStringMetrics(F12B[font_typ], "MC", output, tft.width() / 2, 37 + (4 * y_offset_tft), x_y_h_w_b);
      tft.setCursor(x_y_h_w_b[0], x_y_h_w_b[1]);
      tft.print(output);
      sprintf(output, "%s:", IP_ADDRESS[lingo]);
      calculateStringMetrics(F12B[font_typ], "MC", output, tft.width() / 2, 47 + (5 * y_offset_tft), x_y_h_w_b);
      tft.setCursor(x_y_h_w_b[0], x_y_h_w_b[1]);
      tft.print(output);
      String ipStr = WiFi.softAPIP().toString();  // IP-Adresse als String
      ipStr.toCharArray(output, sizeof(output));  // In char-Array umwandeln
      calculateStringMetrics(F12B[font_typ], "MC", output, tft.width() / 2, 47 + (6 * y_offset_tft), x_y_h_w_b);
      tft.setCursor(x_y_h_w_b[0], x_y_h_w_b[1]);
      tft.print(output);
    }
  #endif
}

void HM_Display::process_manualmode_oled() {
  #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_courB24_tf);
    u8g2.setCursor(10, 42 + y_offset);
    sprintf(output,"%5dg", weight);
    u8g2.print(output);
    u8g2.setFont(u8g2_font_open_iconic_play_2x_t);
    u8g2.drawGlyph(0, 40 + y_offset, (servo_enabled==1)?0x45:0x44 );
    u8g2.setFont(u8g2_font_courB08_tf);
    u8g2.setCursor(0, 11);
    u8g2.setCursor(1, 9);
    sprintf(output,"%s=%d°", A_UPPER_CASE[lingo], angle);
    u8g2.print(output);
    sprintf(output,"%3d%%", pos);
    u8g2.setCursor(104, 9);
    u8g2.print(output);
    if (ina219_installed and (current_servo > 0 or show_current == 1)) {
      u8g2.setCursor(0, 18);
      sprintf(output,"%s:", SERVO_CURRENT[lingo]);
      u8g2.print(output);
      sprintf(output, "%4imA", current_mA);
      u8g2.setCursor(128 - 36, 18);
      u8g2.print(output);
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
}

void HM_Display::process_manualmode_tft(int a) {
  #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
    if (a == 1) {
      weight_old = -9999;
      pos_old = -1;
      tare_old = -1;
      current_mA_old = -1;
      servo_enabled_old = -1;
      angle_ist_old = -1;
      tft.fillScreen(BACKGROUND);
      tft.setTextDatum(MC_DATUM);
      tft.setTextColor(TEXT);
      tft.setFreeFont(F16B[font_typ]);
      x_pos = tft.width() / 2;
      tft.drawString(MANUAL[lingo], x_pos, 11);
      tft.drawLine(0, 30, 320, 30, TEXT);
      tft.drawLine(0, 165, 320, 165, TEXT);
      tft.drawLine(160, 165, 160, 240, TEXT);
      tft.setTextDatum(BL_DATUM);
      tft.setFreeFont(F10B[font_typ]);
      sprintf(output, "%s:", SERVO[lingo]);
      tft.drawString(output, 5, 192);
      sprintf(output, "%s:", ACTUAL[lingo]);
      tft.drawString(output, 5, 215);
      sprintf(output, "%s:", MAX_1[lingo]);
      tft.drawString(output, 5, 238);
      sprintf(output, "%s:", TARE[lingo]);
      tft.drawString(output, 170, 192);
      if (ina219_installed == 1) {
        sprintf(output, "%s:", INA[lingo]);
        tft.drawString(output, 170, 215);
        tft.setTextDatum(BR_DATUM);
        sprintf(output, "%s", (current_servo==0?OFF[lingo]:ON[lingo]));
        tft.drawString(output, 320, 215);
        tft.setTextDatum(BL_DATUM);
        if (ina219_installed == 1 and current_mA != current_mA_old and (current_servo > 0 or show_current == 1)) {
          sprintf(output, "%s:", CURR[lingo]);
          tft.drawString(output, 170, 238);
        }
      }
      tft.setTextDatum(CR_DATUM);
      tft.setFreeFont(F28B[font_typ]);
      tft.setTextColor(TEXT);
      tft.drawString("g", 310, 91);
      tft.setTextDatum(CL_DATUM);
      tft.fillRoundRect(10, 67, 58, 58, 3, TEXT);
      tft.fillRoundRect(13, 70, 52, 52, 3, BACKGROUND);
    }
    else if (a == 2) {
      if (weight != weight_old) {
        tft.setTextDatum(CR_DATUM);
        tft.setFreeFont(F28B[font_typ]);
        tft.setTextColor(BACKGROUND, BACKGROUND);
        tft.drawNumber(weight_old, 310 - tft.textWidth("g") , 91);
        tft.setTextColor(TEXT, BACKGROUND);
        tft.drawNumber(weight, 310 - tft.textWidth("g") , 91);
        weight_old = weight;
      }
      if (angle != angle_ist_old) {
        tft.setTextDatum(BR_DATUM);
        tft.setFreeFont(F10B[font_typ]);
        tft.setTextColor(BACKGROUND, BACKGROUND);
        sprintf(output, "%i°", angle_ist_old);
        tft.drawString(output, 150, 215);
        tft.setTextColor(TEXT, BACKGROUND);
        sprintf(output, "%i°", angle);
        tft.drawString(output, 150, 215);
        angle_ist_old = angle;
      }
      if (pos != pos_old) {
        tft.setTextDatum(BR_DATUM);
        tft.setFreeFont(F10B[font_typ]);
        tft.setTextColor(BACKGROUND, BACKGROUND);
        sprintf(output, "%i°", angle_min + ((angle_max[fullmode-1] - angle_min) * pos_old / 100));
        tft.drawString(output, 150, 238);
        tft.setTextColor(TEXT, BACKGROUND);
        sprintf(output, "%i°", angle_min + ((angle_max[fullmode-1] - angle_min) * pos / 100));
        tft.drawString(output, 150, 238);
        pos_old = pos;
      }
      if (tare != tare_old) {
        tft.setTextDatum(BR_DATUM);
        tft.setFreeFont(F10B[font_typ]);
        tft.setTextColor(BACKGROUND, BACKGROUND);
        sprintf(output, "%ig", tare_old);
        tft.drawString(output, 320, 192);
        tft.setTextColor(TEXT, BACKGROUND);
        sprintf(output, "%ig", tare);
        tft.drawString(output, 320, 192);
        tare_old = tare;
      }
      if (ina219_installed == 1 and current_mA != current_mA_old and (current_servo > 0 or show_current == 1)) {
        tft.setTextDatum(BR_DATUM);
        tft.setFreeFont(F10B[font_typ]);
        if (current_mA_old < 1000) {
          sprintf(output, "%imA", current_mA_old);
        }
        else {
          sprintf(output, "%.3fA", current_mA_old / 1000.0);
        }
        tft.setTextColor(BACKGROUND, BACKGROUND);
        tft.drawString(output, 320, 238);
        tft.setTextColor(TEXT, BACKGROUND);
        if (current_mA > current_servo and current_servo > 0) {
          tft.setTextColor(TFT_RED, BACKGROUND);
        }
        if (current_mA < 1000) {
          sprintf(output, "%imA", current_mA);
        }
        else {
          sprintf(output, "%.3fA", current_mA / 1000.0);
        }
        tft.drawString(output, 320, 238);
        tft.setTextColor(TEXT);
        current_mA_old = current_mA;
      }
      if (servo_enabled != servo_enabled_old) {
        tft.fillRoundRect(13, 70, 52, 52, 3, BACKGROUND);
        tft.setFreeFont(&icon26pt);
        tft.setTextDatum(MC_DATUM);
        if (servo_enabled == 1) {
          tft.setTextColor(TFT_GREEN);
          tft.drawString("A", 34, 95);
        }
        else {
          tft.setTextColor(TFT_RED);
          tft.drawString("B", 33, 95);
        }
        servo_enabled_old = servo_enabled;
        tft.setTextColor(TEXT);
      }
    }
  #endif
}

void HM_Display::process_automatic_clear_oled() {
  #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
    u8g2.clearBuffer();
    u8g2.sendBuffer();
  #endif
}

void HM_Display::process_automatic_connection_failed_oled() {
  #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
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
}

void HM_Display::process_automatic_init_turntable_nok_oled() {
  #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
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
}

void HM_Display::process_automatic_read_time_close_tripprodection_failed_oled() {
  #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_courB08_tf);
    x_pos = CenterPosX(CLOSE_DRIP_DP[lingo], 6, 128);
    u8g2.setCursor(x_pos,18);  
    u8g2.print(CLOSE_DRIP_DP[lingo]);
    x_pos = CenterPosX(PROTECTION_WAIT_DP[lingo], 6, 128);
    u8g2.setCursor(x_pos,28);  
    u8g2.print(PROTECTION_WAIT_DP[lingo]);
    x_pos = CenterPosX(TIME_READING_DP[lingo], 6, 128);
    u8g2.setCursor(x_pos,38);  
    u8g2.print(TIME_READING_DP[lingo]);
    x_pos = CenterPosX(FAILED_DP[lingo], 6, 128);
    u8g2.setCursor(x_pos,48);  
    u8g2.print(FAILED_DP[lingo]);
    u8g2.sendBuffer();
  #endif
}

void HM_Display::process_automatic_start_oled() {
  #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999 
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_courB24_tf);
    u8g2.setCursor(15, 43);
    u8g2.print(START[lingo]);
    u8g2.sendBuffer();
  #endif
}

void HM_Display::process_automatic_wait_befor_fill_oled() {
  #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_courB08_tf);
    x_pos = CenterPosX(CONTINUE_WITH[lingo], 6, 128);
    u8g2.setCursor(x_pos, 35);
    u8g2.print(CONTINUE_WITH[lingo]);
    x_pos = CenterPosX(THE_START_BUTTON[lingo], 6, 128);
    u8g2.setCursor(x_pos, 45);
    u8g2.print(THE_START_BUTTON[lingo]);
    u8g2.sendBuffer();
  #endif
}

void HM_Display::process_automatic_main_oled(int rotary_select, int SW_KORREKTUR, int SW_FLUSS, int SW_MENU, bool full, int y_offset_ina) {
  #if DISPLAY_TYPE == 991 or DISPLAY_TYPE == 992 or DISPLAY_TYPE == 999
    u8g2.clearBuffer();
    // wenn kein Tara für unser Glas definiert ist, wird kein Gewicht sondern eine Warnung ausgegeben
    if (tare > 0) {
      // kein Glas aufgestellt 
      if (weight < -20) {
        u8g2.setFont(u8g2_font_courB10_tf);
        u8g2.setCursor(37, 31 + y_offset_ina); u8g2.print(PLEASE_PUT[lingo]);
        u8g2.setCursor(37, 43 + y_offset_ina); u8g2.print(UP_THE_JAR[lingo]);
      } 
      else {
        if(autostart == 1 && auto_enabled == 1 && servo_enabled == 0 && weight >= -5 && weight - tare_jar < fquantity and turntable_ok == true and full == false) {
          u8g2.setFont(u8g2_font_unifont_t_symbols);
          u8g2.drawGlyph(14, 38 + y_offset_ina, 0x2612);      //[x] Graph
          u8g2.setFont(u8g2_font_courB24_tf);
        }
        u8g2.setCursor(10, 42 + y_offset_ina);
        u8g2.setFont(u8g2_font_courB24_tf);
        sprintf(output,"%5dg", weight - tare_jar);
        u8g2.print(output);
      }
    } 
    else {
      u8g2.setFont(u8g2_font_courB14_tf);
      int y = get_length(NO_TARE[lingo]);
      u8g2.setCursor(128 - y * 10 -5, 38 + y_offset_ina);
      sprintf(output,"%6s", NO_TARE[lingo]);            //kein Tara
      u8g2.print(output);
    }
    // Play/Pause Icon, ob die Automatik aktiv ist
    u8g2.setFont(u8g2_font_open_iconic_play_2x_t);
    u8g2.drawGlyph(0, 40 + y_offset_ina, (auto_enabled==1)?0x45:0x44 );
    u8g2.setFont(u8g2_font_courB08_tf);
    // Zeile oben, Öffnungswinkel absolut und Prozent, Anzeige Autostart
    u8g2.setCursor(1, 9);
    sprintf(output,"%s=%-3d", A_UPPER_CASE[lingo], angle);
    u8g2.print(output);
    int n = StringLenght(output);
    u8g2.setCursor(n * 6 + 1, 9);
    u8g2.print("°");
    sprintf(output,"%3d%%", pos);
    u8g2.setCursor(104, 9);
    u8g2.print(output);
    sprintf(output,"%2s", (autostart==1)?AS[lingo]:" ");
    u8g2.setCursor(58, 9);
    u8g2.print(output);
    //INA219 Display
    if (ina219_installed and (current_servo > 0 or show_current == 1)) {
      u8g2.setCursor(0, 18);
      sprintf(output, "%s:", SERVO_CURRENT[lingo]);
      u8g2.print(output);
      sprintf(output, "%4imA", current_mA);
      u8g2.setCursor(128 - 36, 18);
      u8g2.print(output);
    }
    // Zeile unten
    // Verstellung nur wenn Automatik inaktiv, gesteuert über Interrupt-Funktion 
    if (servo_enabled == 1) {
      progressbar = 128.0*((float)weight/(float)target_weight);
      progressbar = constrain(progressbar,0,128);
      u8g2.drawFrame(0, 55, 128, 9);
      u8g2.drawBox  (0, 55, progressbar, 9);
    } 
    else {
      u8g2.setFont(u8g2_font_courB08_tf);
      sprintf(output, "%s%s=%-3d" ,(autocorrection==1)?A_LOWER_CASE[lingo]:"", C_LOWER_CASE[lingo] , correction + autocorrection_gr);
      n = StringLenght(output);
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
      u8g2.print(output);
      sprintf(output, "%s=%-3d" ,F_LOWER_CASE[lingo], init_weight_f);
      n = StringLenght(output);
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
      u8g2.print(output);

      if (glaeser[fquantity_index].Gewicht > 999) {
        sprintf(output, "%2.1fkg-%3s", float(glaeser[fquantity_index].Gewicht) / 1000, GlasTypArray[glaeser[fquantity_index].GlasTyp]);
      }
      else if (glaeser[fquantity_index].Gewicht >= 100) {
        sprintf(output, "%3dg-%3s", glaeser[fquantity_index].Gewicht, GlasTypArray[glaeser[fquantity_index].GlasTyp]);
      }
      else {
        sprintf(output, "%2dg-%3s", glaeser[fquantity_index].Gewicht, GlasTypArray[glaeser[fquantity_index].GlasTyp]);
      }
      n = StringLenght(output);
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
      u8g2.print(output);
    }
    u8g2.sendBuffer();
  #endif
}

void HM_Display::process_automatic_clear_tft() {
  #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
    tft.fillScreen(BACKGROUND);
    tft.setTextColor(TEXT);
  #endif
}

void HM_Display::process_automatic_connection_failed_tft() {
  #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
   
  #endif
}

void HM_Display::process_automatic_init_turntable_nok_tft() {
  #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
   
  #endif
}

void HM_Display::process_automatic_read_time_close_tripprodection_failed_tft() {
  #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
   
  #endif
}

void HM_Display::process_automatic_init_screen_tft(int a) {
  #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
    if (a == 1) {
      tft.fillScreen(BACKGROUND);
      //Frame
      tft.drawLine(0, 24, 320, 24, TEXT);
      tft.drawLine(0, 164, 320, 164, TEXT);
      tft.drawLine(160, 164, 160, 240, TEXT);
      //Bottom
      tft.setFreeFont(F08[font_typ]);
      tft.setTextColor(TEXT);
      tft.setTextDatum(BL_DATUM);
      tft.drawString(INA219[lingo], 2, 185);
      tft.drawString(AUTOSTART[lingo], 2, 202);
      tft.drawString(AUTOCORRECTION[lingo], 2, 219);
      tft.drawString(FLOW_G_OVER_TIME[lingo], 2, 236);
      sprintf(output, "%s:", CURRENT[lingo]);
      tft.drawString(output, 170, 185);
      sprintf(output, "%s:", CORRECTION[lingo]);
      tft.drawString(output, 170, 202);
      sprintf(output, "%s:", AUTOCORR[lingo]);
      tft.drawString(output, 170, 219);
      sprintf(output, "%s:", FLOW[lingo]);
      tft.drawString(output, 170, 236);
      tft.setFreeFont(&icon10pt);
      tft.setTextDatum(BR_DATUM);
      if (ina219_installed == 0){
        tft.setTextColor(TFT_RED);
        tft.drawString("D", 155, 185);
      }
      else {
        tft.setTextColor(TFT_GREEN);
        tft.drawString("C", 155, 185);
      }
      if (autostart == 0){
        tft.setTextColor(TFT_RED);
        tft.drawString("D", 155, 202);
      }
      else {
        tft.setTextColor(TFT_GREEN);
        tft.drawString("C", 155, 202);
      }
      if (autocorrection == 0){
        tft.setTextColor(TFT_RED);
        tft.drawString("D", 155, 219);
      }
      else {
        tft.setTextColor(TFT_GREEN);
        tft.drawString("C", 155, 219);
      }
      if (init_weight_f == 0) {
        tft.setTextColor(TFT_RED, BACKGROUND);
        tft.drawString("D", 155, 236);
      }
      else {
        tft.setTextColor(TFT_GREEN, BACKGROUND);
        tft.drawString("C", 155, 236);
      }
      //Top
      tft.setFreeFont(F08[font_typ]);
      tft.setTextColor(TEXT);
      tft.setTextDatum(CL_DATUM);
      sprintf(output, "%s:%i°", MIN[lingo], angle_min);
      tft.drawString(output, 2, 10);
      //Middle (94)
      tft.fillRoundRect(5, 40, 58, 58, 3, TEXT);
      tft.fillRoundRect(8, 43, 52, 52, 3, BACKGROUND);
      tft.drawRect(2, 115, 316, 10, TEXT);
      //Wenn INA istaliert ist aber nicht zum Messen gebraucht wird
      if (ina219_installed == 1 and no_ina == 0 and show_current == 0) {
        tft.setTextDatum(BR_DATUM);
        sprintf(output, "%s", OFF[lingo]);
        tft.drawString(output, 320, 185);
      }
    }
  #endif
}

void HM_Display::process_automatic_start_tft() {
  #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
    int i;
    //PLAY ICON setzen
    jar_on_scale = false;
    tft.fillRoundRect(8, 43, 52, 52, 3, BACKGROUND);
    tft.setTextDatum(MC_DATUM);
    tft.setFreeFont(&icon26pt);
    tft.setTextColor(TFT_GREEN);
    tft.drawString("A", 29, 69);
    //Korrektur Wert auf Textfarbe setzen
    tft.setTextDatum(BR_DATUM);
    tft.setFreeFont(F08[font_typ]);
    tft.setTextColor(TEXT, BACKGROUND);
    sprintf(output, "%ig", correction);
    tft.drawString(output, 320, 202);
    //Flow Wert auf Textfarbe setzen
    sprintf(output, "%ig", init_weight_f);
    tft.drawString(output, 320, 236);
    //Glasgewicht auf Textfarbe Setzen
    tft.setTextDatum(TL_DATUM);
    tft.setFreeFont(F14[font_typ]);
    sprintf(output, "%dg ", (glaeser[fquantity_index].Gewicht));
    tft.drawString(output, 2, 128);
    i = tft.textWidth(output);
    tft.setFreeFont(F08[font_typ]);
    sprintf(output, "+%dg ", (overfill_gr));
    tft.drawString(output, i, 128);
    i = i + tft.textWidth(output);
    tft.fillRect(i, 125, 320 - i, 38, BACKGROUND);
    tft.setTextColor(TEXT);
    tft.setFreeFont(F28B[font_typ]);
    tft.fillRect(70, 26, 250, 87, BACKGROUND);
    sprintf(output, "%s", START[lingo]);
    calculateStringMetrics(F28B[font_typ], "MC", output, 195, 69, x_y_h_w_b);
    tft.setCursor(x_y_h_w_b[0], x_y_h_w_b[1]);
    tft.print(output);
  #endif
}

void HM_Display::process_automatic_set_tare_jar_tft() {
  #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
    tft.setTextDatum(TR_DATUM);
    tft.setFreeFont(F14[font_typ]);
    tft.setTextColor(TEXT, BACKGROUND);
    sprintf(output, "%s:%dg ", TARE_JAR[lingo], tare_jar);
    tft.drawString(output, 318, 128);
  #endif
}

void HM_Display::process_automatic_wait_befor_fill_tft() {
  #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
    tft.fillRect(70, 26, 250, 87, BACKGROUND);
    tft.setTextColor(TEXT, BACKGROUND);
    tft.setFreeFont(F12B[font_typ]);
    sprintf(output, "%s", CONTINUE_WITH[lingo]);
    calculateStringMetrics(F12B[font_typ], "LC", output, 100, 69, x_y_h_w_b);
    tft.setCursor(x_y_h_w_b[0], x_y_h_w_b[1] - x_y_h_w_b[2]/2 - 2);
    tft.print(output);
    sprintf(output, "%s", THE_START_BUTTON[lingo]);
    calculateStringMetrics(F12B[font_typ], "LC", output, 100, 69, x_y_h_w_b);
    tft.setCursor(x_y_h_w_b[0], x_y_h_w_b[1] + x_y_h_w_b[2]/2 + 2);
    tft.print(output);
  #endif
}

void HM_Display::process_automatic_main_tft(int rotary_select, int SW_KORREKTUR, int SW_FLUSS, int SW_MENU) {
  #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
    //TOP
    //Max
    if (pos != pos_old) {
      tft.setFreeFont(F08[font_typ]);
      tft.setTextDatum(MC_DATUM);
      tft.setTextColor(BACKGROUND);
      sprintf(output, "%s:%i°", MAX_2[lingo], angle_min + ((angle_max[fullmode-1] - angle_min) * pos_old / 100));
      tft.drawString(output, 160, 10);
      tft.setTextColor(TEXT);
      sprintf(output, "%s:%i°", MAX_2[lingo], angle_min + ((angle_max[fullmode-1] - angle_min) * pos / 100));
      tft.drawString(output, 160, 10);
      pos_old = pos;
    }
    //Ist
    if (angle != angle_ist_old) {
      tft.setTextDatum(CR_DATUM);
      tft.setFreeFont(F08[font_typ]);
      tft.setTextColor(BACKGROUND);
      sprintf(output, "%s:%i°", ACT[lingo], angle_ist_old);
      tft.drawString(output, 318, 10);
      tft.setTextColor(TEXT);
      sprintf(output, "%s:%i°", ACT[lingo], angle);
      tft.drawString(output, 318, 10);
      angle_ist_old = angle;
    }
    //MIDDLE
    //Glas aufstellen
    if (tare > 0) {
      if (weight < -20 and weight != weight_old and jar_on_scale == false) {
        jar_on_scale = true;
        y_pos_weight = false;
        tft.fillRect(70, 26, 250, 87, BACKGROUND);
        tft.setTextColor(TEXT, BACKGROUND);
        tft.setFreeFont(F18B[font_typ]);
        sprintf(output, "%s", PLEASE_PUT[lingo]);
        calculateStringMetrics(F18B[font_typ], "LC", output, 100, 69, x_y_h_w_b);
        tft.setCursor(x_y_h_w_b[0], x_y_h_w_b[1] - x_y_h_w_b[2]/2 - 2);
        tft.print(output);
        sprintf(output, "%s", UP_THE_JAR[lingo]);
        calculateStringMetrics(F18B[font_typ], "LC", output, 100, 69, x_y_h_w_b);
        tft.setCursor(x_y_h_w_b[0], x_y_h_w_b[1] + x_y_h_w_b[2]/2 + 2);
        tft.print(output);
        if (auto_enabled == 1) {
          int i;
          jar_old = -1;
          //nun ein bischen Overkill dass das Gewicht mit Overfill nicht ein mal flackert :-)
          tft.setFreeFont(F14[font_typ]);
          sprintf(output, "%dg ", (glaeser[fquantity_index].Gewicht));
          i = tft.textWidth(output);
          sprintf(output, "+%dg ", (overfill_gr));
          i = i + tft.textWidth(output);
          tft.fillRect(i, 125, 320 - i, 38, BACKGROUND);
        }
      }
      else if (weight >= -20 and weight != weight_old) {
        jar_on_scale = false;
        tft.setFreeFont(F28B[font_typ]);
        tft.setTextColor(TEXT, BACKGROUND);
        if (y_pos_weight == false) {
          y_pos_weight = true;
          calculateStringMetrics(F28B[font_typ], "RC", "0g", 310, 69, x_y_h_w_b);
          pos_y_weight = x_y_h_w_b[1];
          tft.fillRect(70, 26, 250, 87, BACKGROUND);
        }
        else {
          tft.fillRect(70, 26, 245 - tft.textWidth("g"), 87, BACKGROUND);
        }
        sprintf(output, "%ig", weight - tare_jar);
        calculateStringMetrics(F28B[font_typ], "RC", output, 315, 69, x_y_h_w_b);
        tft.setCursor(x_y_h_w_b[0], pos_y_weight);
        tft.print(output);
      }
      if (weight != weight_old) {
        progressbar = 314.0 * (((float)weight - tare_jar) / (float)(glaeser[fquantity_index].Gewicht));
        progressbar = constrain(progressbar, 0, 314);
        float adjusted_weight = weight - tare_jar;
        if (adjusted_weight > glaeser[fquantity_index].Gewicht + overfill_gr) {
          tft.fillRect(3, 116, progressbar, 8, TFT_ORANGE);  // Überfüllung
        } 
        else if (adjusted_weight >= glaeser[fquantity_index].Gewicht && adjusted_weight <= glaeser[fquantity_index].Gewicht + overfill_gr) {
          tft.fillRect(3, 116, progressbar, 8, TFT_GREEN);  // Innerhalb der akzeptablen Grenze
        } 
        else {
          tft.fillRect(3, 116, progressbar, 8, TFT_RED);  // Unterhalb des Zielgewichts
        }
        tft.fillRect(3 + progressbar, 116, 314 - progressbar, 8, BACKGROUND);
        weight_old = weight;  // Gewicht aktualisieren
      }
    }
    //kein Tara vorhanden
    else if (tare != tare_old_automatic) {
      jar_on_scale = false;
      y_pos_weight = false;
      tft.fillRect(70, 26, 250, 87, BACKGROUND);
      tft.setTextColor(TFT_RED, BACKGROUND);
      tft.setFreeFont(F18B[font_typ]);
      sprintf(output, "%s", NO_TARE[lingo]);
      calculateStringMetrics(F18B[font_typ], "MC", output, 195, 69, x_y_h_w_b);
      tft.setCursor(x_y_h_w_b[0], x_y_h_w_b[1]);
      tft.print(output);
      tft.setTextColor(TEXT);
      tare_old_automatic = tare;
    }
    //Glasauswahl
    if ((jar_old != fquantity_index and servo_enabled == 0 and weight <= glaeser[fquantity_index].Gewicht - tare)) {
      unsigned long  COLOR;
      if (rotary_select == SW_MENU and servo_enabled == 0) {
        COLOR = MARKER;
      }
      else {
        COLOR = TEXT;
      }
      int i;
      int j;
      tft.setTextDatum(TL_DATUM);
      if (jar_old != -1) {
        tft.setFreeFont(F14[font_typ]);
        tft.setTextColor(BACKGROUND, BACKGROUND);
        sprintf(output, "%dg ", (glaeser[jar_old].Gewicht));
        tft.drawString(output, 2, 128);
        i = tft.textWidth(output);
        tft.setFreeFont(F08[font_typ]);
        sprintf(output, "+%dg ", (overfill_gr));
        tft.drawString(output, i, 128);
        j = tft.textWidth(output);
        tft.setFreeFont(F14[font_typ]);
        if (GlasTypArray[glaeser[jar_old].GlasTyp] == "DIB") {
        tft.drawString("DE Imker Bund", i+j+10, 128);
        }
        else if (GlasTypArray[glaeser[jar_old].GlasTyp] == "TOF") {
          tft.drawString("TwistOff", i+j+10, 128);
        }
        else if (GlasTypArray[glaeser[jar_old].GlasTyp] == "DEE") {
          tft.drawString("DeepTwist", i+j+10, 128);
        }
        else if (GlasTypArray[glaeser[jar_old].GlasTyp] == "SPZ") {
          tft.drawString("Spezial", i+j+10, 128);
        }
      }
      tft.setFreeFont(F14[font_typ]);
      tft.setTextColor(COLOR, BACKGROUND);
      sprintf(output, "%dg ", (glaeser[fquantity_index].Gewicht));
      tft.drawString(output, 2, 128);
      i = tft.textWidth(output);
      tft.setFreeFont(F08[font_typ]);
      sprintf(output, "+%dg ", (overfill_gr));
      tft.drawString(output, i, 128);
      j = tft.textWidth(output);
      tft.setFreeFont(F14[font_typ]);
      if (GlasTypArray[glaeser[fquantity_index].GlasTyp] == "DIB") {
        tft.drawString("DE Imker Bund", i+j+10, 128);
      }
      else if (GlasTypArray[glaeser[fquantity_index].GlasTyp] == "TOF") {
        tft.drawString("TwistOff", i+j+10, 128);
      }
      else if (GlasTypArray[glaeser[fquantity_index].GlasTyp] == "DEE") {
        tft.drawString("DeepTwist", i+j+10, 128);
      }
      else if (GlasTypArray[glaeser[fquantity_index].GlasTyp] == "SPZ") {
        tft.drawString("Spezial", i+j+10, 128);
      }
      jar_old = fquantity_index;
      tare_old_automatic = -1;
      weight_old = -9999;
    }
    //Start/Stop Icon
    if (auto_enabled != auto_enabled_old) {
      tft.fillRoundRect(8, 43, 52, 52, 3, BACKGROUND);
      tft.setFreeFont(&icon26pt);
      tft.setTextDatum(MC_DATUM);
      if (auto_enabled == 1) {
        tft.setTextColor(TFT_GREEN);
        tft.drawString("A", 29, 69);
      }
      else {
        tft.setTextColor(TFT_RED);
        tft.drawString("B", 28, 69);
      }
      auto_enabled_old = auto_enabled;
      tft.setTextColor(TEXT);
    }
    //BOTTOM
    //INA219 Anzeige
    if (ina219_installed == 1 and current_mA != current_mA_old and (current_servo > 0 or show_current == 1)) {
      tft.setTextDatum(BR_DATUM);
      tft.setFreeFont(F08[font_typ]);
      if (current_mA_old < 1000) {
        sprintf(output, "%imA", current_mA_old);
      }
      else {
        sprintf(output, "%.3fA", current_mA_old / 1000.0);
      }
      tft.setTextColor(BACKGROUND, BACKGROUND);
      tft.drawString(output, 320, 185);
      tft.setTextColor(TEXT, BACKGROUND);
      if (current_mA > current_servo and current_servo > 0) {
        tft.setTextColor(TFT_RED, BACKGROUND);
      }
      if (current_mA < 1000) {
        sprintf(output, "%imA", current_mA);
      }
      else {
        sprintf(output, "%.3fA", current_mA / 1000.0);
      }
      tft.drawString(output, 320, 185);
      tft.setTextColor(TEXT);
      current_mA_old = current_mA;
    }
    //Korrektur Anzeige
    if (correction_old != correction + autocorrection_gr) {
      tft.setTextDatum(BR_DATUM);
      tft.setFreeFont(F08[font_typ]);
      tft.setTextColor(BACKGROUND, BACKGROUND);
      sprintf(output, "%ig", correction_old);
      tft.drawString(output, 320, 202);
      tft.setTextColor(TEXT);
      if (rotary_select == SW_KORREKTUR and servo_enabled == 0) {
        tft.setTextColor(MARKER);
      }
      sprintf(output, "%5ig", correction + autocorrection_gr);
      tft.drawString(output, 320, 202);
      tft.setTextColor(TEXT);
      correction_old = correction + autocorrection_gr;
    }
    //Autokorektur Anzeige
    if (autocorrection_gr_old != autocorrection_gr) {
      tft.setTextDatum(BR_DATUM);
      tft.setFreeFont(F08[font_typ]);
      tft.setTextColor(BACKGROUND);
      sprintf(output, "%ig", autocorrection_gr_old);
      tft.drawString(output, 320, 219);
      tft.setTextColor(TEXT);
      sprintf(output, "%ig", autocorrection_gr);
      tft.drawString(output, 320, 219);
      autocorrection_gr_old = autocorrection_gr;
    }
    //Fluss Anzeige
    if (intWeight_old != init_weight_f) {
      tft.setFreeFont(&icon10pt);
      tft.setTextDatum(BR_DATUM);
      if (intWeight_old == 0 or init_weight_f == 0) {
        if (init_weight_f == 0) {
          tft.setTextColor(TFT_RED, BACKGROUND);
          tft.drawString("D", 155, 236);
        }
        else {
          tft.setTextColor(TFT_GREEN, BACKGROUND);
          tft.drawString("C", 155, 236);
        }
      }
      tft.setTextDatum(BR_DATUM);
      tft.setFreeFont(F08[font_typ]);
      tft.setTextColor(BACKGROUND);
      sprintf(output, "%ig", intWeight_old);
      tft.drawString(output, 320, 236);
      tft.setTextColor(TEXT);
      if (rotary_select == SW_FLUSS and servo_enabled == 0) {
        tft.setTextColor(MARKER);
      }
      sprintf(output, "%ig", init_weight_f);
      tft.drawString(output, 320, 236);
      tft.setTextColor(TEXT);
      intWeight_old = init_weight_f;
    }    
  #endif
}

void tft_colors(void) {
    #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999
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
    #endif
}

void tft_marker(void) {   
    #if DISPLAY_TYPE == 993 or DISPLAY_TYPE == 999           
        if (color_marker == 0)          {MARKER = 0x05ff;}
        else if (color_marker == 1)     {MARKER = 0x021f;}
        else if (color_marker == 2)     {MARKER = 0x001f;}
        else if (color_marker == 3)     {MARKER = 0x4851;}
        else if (color_marker == 4)     {MARKER = 0xc050;}
        else if (color_marker == 5)     {MARKER = 0xf800;}
        else if (color_marker == 6)     {MARKER = 0xfbc0;}
        else if (color_marker == 7)     {MARKER = 0xfde0;}
        else if (color_marker == 8)     {MARKER = 0xd6e1;}
        else if (color_marker == 9)     {MARKER = 0xffe0;}
        else if (color_marker == 10)    {MARKER = 0x2724;}
        else if (color_marker == 11)    {MARKER = 0x0da1;}
    #endif
}

int StringLenght(String a) {
  a.trim();
  int res = a.length();
  return res;
}

int get_length(const char a[]) {
  int counter = 0;
  if (a != NULL) {
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
  }
  return counter;
}

int CenterPosX(const char a[], float font_width, int display_widht) {
  int res = 0;
  int counter = get_length(a);
  res = int(float(display_widht - font_width * counter) / 2);
  return res;
}

//TFT
uint16_t utf8ToUnicode(const char *&p) {
    uint16_t c = (uint8_t)*p;
    if (c < 0x80) {
        // ASCII-Zeichen
        return c;
    } else if ((c & 0xE0) == 0xC0) {
        // 2-Byte-Zeichen
        uint16_t c1 = (uint8_t)*(++p);
        return ((c & 0x1F) << 6) | (c1 & 0x3F);
    } else if ((c & 0xF0) == 0xE0) {
        // 3-Byte-Zeichen
        uint16_t c1 = (uint8_t)*(++p);
        uint16_t c2 = (uint8_t)*(++p);
        return ((c & 0x0F) << 12) | ((c1 & 0x3F) << 6) | (c2 & 0x3F);
    }
    return 0; // Ungültiges Zeichen
}

void calculateStringMetrics(const GFXfont *font, const char *datum, const char *text, int x, int y, uint16_t res[5]) {
  uint16_t width = 0;
  uint16_t height = 0;
  uint16_t baseline = 0;
  uint16_t topline = INT16_MAX;                             // Topline initial auf einen hohen Wert setzen
  int16_t maxAscent = 0;                                    // Oberhalb der Baseline
  int16_t maxDescent = 0;                                   // Unterhalb der Baseline
  for (const char *p = text; *p != '\0';) {
    uint16_t c = utf8ToUnicode(p);
    p++;
    if (c < font->first || c > font->last) {
      continue;                                             // Überspringe Zeichen außerhalb des Font-Bereichs
    }
    GFXglyph *glyph = &(font->glyph[c - font->first]);
    width += glyph->xAdvance;                               // Breite summieren
    // Höhe und Baseline berechnen
    int glyphAscent = -glyph->yOffset;
    int glyphDescent = glyph->height + glyph->yOffset;
    int glyphTopline = -glyph->yOffset;                     // Topline basierend auf yOffset
    if (glyphAscent > maxAscent) {
      maxAscent = glyphAscent;
    }
    if (glyphDescent > maxDescent) {
      maxDescent = glyphDescent;
    }
    if (glyphTopline < topline) {
      topline = glyphTopline;                               // Niedrigste Topline finden
    }
  }
  baseline = maxAscent;
  height = maxAscent + maxDescent;
  // Sicherstellen, dass die berechnete Höhe nicht die Font-Höhe (yAdvance) überschreitet
  if (height > font->yAdvance) {
    height = font->yAdvance;
  }
  if (datum == "MC") {
    res[0] = x - width / 2;
    res[1] = y + baseline / 2 - 1;
  }
  if (datum == "LC") {
    res[0] = x;
    res[1] = y + baseline / 2 - 1;
  }
  if (datum == "RC") {
    res[0] = x - width;
    res[1] = y + baseline / 2 - 1;
  }
  if (datum == "BL") {
    res[0] = x;
    res[1] = y;
  }
  if (datum == "BR") {
    res[0] = x - width;
    res[1] = y;
  }
  res[2] = height;
  res[3] = width;
  res[4] = baseline;
}