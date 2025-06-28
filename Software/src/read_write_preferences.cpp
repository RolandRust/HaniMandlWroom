//read_write_preferences.cpp

#include "Preferences.h"
#include "read_write_preferences.h"
#include "variables.h"
#include "variables_display.h"

Preferences preferences;

float old_factor;
int old_pos;
int old_weight_empty;
int old_correction;
int old_autostart;
int old_autocorrection;
int old_init_weight_f;
int old_overfill_gr;
int old_fquantity_index;
int old_angle_min;
int old_buzzermode;
int old_ledmode;
int old_showlogo;
int old_showcredits;
int old_cali_weight;
int old_jartolerance;
int old_current_servo;
int old_show_current;
int old_color_scheme;
int old_color_marker;
int old_use_turntable;
int old_lingo;
int old_wait_befor_fill;
int old_fullmode;
int old_font_typ;
int old_menu_rotation;
int old_squee_tap_left;
int old_servo_expanded;
int old_max_lc_weight;
int old_max_lc_cal;
int old_angle_max[5];
int old_angle_fine[5];
float old_finedos_weight[5];
glas old_jars[5];



HM_READ_WRITE_PREFERENCES::HM_READ_WRITE_PREFERENCES() {}

void HM_READ_WRITE_PREFERENCES::getPreferences() {
  char output[30];
  preferences.begin("EEPROM", false);                           // Parameter aus dem EEPROM lesen
  factor          = preferences.getFloat("factor", 0.0);
  pos             = preferences.getUInt("pos", 0);
  weight_empty    = preferences.getUInt("weight_empty", 0); 
  correction      = preferences.getUInt("correction", 0);
  autostart       = preferences.getUInt("autostart", 0);
  autocorrection  = preferences.getUInt("autocorrection", 0);
  init_weight_f   = preferences.getUInt("init_weight_f", init_weight_f);  // bei 0 aus.A.P.
  overfill_gr     = preferences.getUInt("overfill_gr", 5);
  fquantity_index = preferences.getUInt("fquantity_index", 3);
  angle_min       = preferences.getUInt("angle_min", angle_min);
  buzzermode      = preferences.getUInt("buzzermode", buzzermode);
  ledmode         = preferences.getUInt("ledmode", ledmode);
  showlogo        = preferences.getUInt("showlogo", showlogo);
  showcredits     = preferences.getUInt("showcredits", showcredits);
  cali_weight     = preferences.getUInt("cali_weight", cali_weight);
  jartolerance    = preferences.getUInt("jartolerance", jartolerance);
  current_servo   = preferences.getUInt("current_servo", current_servo);
  show_current    = preferences.getUInt("show_current", show_current);
  color_scheme    = preferences.getUInt("color_scheme", color_scheme);
  color_marker    = preferences.getUInt("color_marker", color_marker);
  use_turntable   = preferences.getUInt("use_turntable", use_turntable);
  lingo           = preferences.getUInt("lingo", lingo);
  wait_befor_fill = preferences.getUInt("wait_befor_fill", wait_befor_fill);
  fullmode        = preferences.getUInt("fullmode", fullmode);
  font_typ        = preferences.getUInt("font_typ", font_typ);
  menu_rotation   = preferences.getUInt("menu_rotation", menu_rotation);
  squee_tap_left  = preferences.getUInt("squee_tap_left", squee_tap_left);
  servo_expanded  = preferences.getUInt("servo_expanded", servo_expanded);
  max_lc_weight   = preferences.getUInt("max_lc_weight", max_lc_weight);
  max_lc_cal      = preferences.getUInt("max_lc_cal", max_lc_cal);
  ssid            = preferences.getString("ssid", ssid);                      //nicht im pref check drin
  password        = preferences.getString("password", password);              //nicht im pref check drin
  rotary_scale    = preferences.getUInt("rotary_scale", rotary_scale);        //nicht im pref check drin
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
  int i = 0;
  while(i < 5) {
    sprintf(output, "Gewicht%d", i);
    glaeser[i].Gewicht    = old_jars[i].Gewicht   = preferences.getInt(output, ResetGewichte[i]);
    sprintf(output, "GlasTyp%d", i);
    glaeser[i].GlasTyp    = old_jars[i].GlasTyp   = preferences.getInt(output, ResetGlasTyp[i]);
    sprintf(output, "Tara%d", i);
    glaeser[i].Tare       = old_jars[i].Tare      = preferences.getInt(output, -9999);
    sprintf(output, "TripCount%d", i);
    glaeser[i].TripCount  = old_jars[i].TripCount = preferences.getInt(output, 0);
    sprintf(output, "Count%d", i);
    glaeser[i].Count      = old_jars[i].Count     = preferences.getInt(output, 0);
    i++;
  }
  i = 0;
  while(i < 5) {
    sprintf(output, "angle_max%d", i);
    angle_max[i]      = old_angle_max[i]        = preferences.getInt(output, angle_max[i]);
    sprintf(output, "angle_fine%d", i);
    angle_fine[i]     = old_angle_fine[i]       = preferences.getInt(output, angle_fine[i]);
    sprintf(output, "finedos_weight%d", i);
    finedos_weight[i] =  old_finedos_weight[i]  = preferences.getFloat(output, finedos_weight[i]);
    i++;
  }
  preferences.end();
  setPreferencesCheck();
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
    Serial.print("squee_tap_left = ");        Serial.println(squee_tap_left);
    Serial.print("servo_expanded = ");        Serial.println(servo_expanded);
    Serial.print("max_lc_weight = ");         Serial.println(max_lc_weight);
    Serial.print("max_lc_cal = ");            Serial.println(max_lc_cal);
    Serial.print("rotary_scale = ");          Serial.println(rotary_scale);
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
  #endif
}

void HM_READ_WRITE_PREFERENCES::setPreferences() {
  char output[30];
  int winkel = getRotariesValue(SW_WINKEL);
  int i;
  bool changed = false;
  preferences.begin("EEPROM", false);
  // Winkel-Einstellung separat behandeln, ändert sich häufig
  if (winkel != preferences.getUInt("pos", 0)) {
    preferences.putUInt("pos", winkel);
    #if DEBUG_HM >= 1
      Serial.print("winkel gespeichert: ");
      Serial.println(winkel);
    #endif
  }
  // Counter separat behandeln, ändert sich häufig
  for ( i = 0 ; i < 5; i++ ) {
    sprintf(output, "TripCount%d", i);
    if (glaeser[i].TripCount != preferences.getInt(output, 0)) {
      preferences.putInt(output, glaeser[i].TripCount);
    }
    sprintf(output, "Count%d", i);
    if (glaeser[i].Count != preferences.getInt(output, 0)) {
      preferences.putInt(output, glaeser[i].Count);
    }
  }
  if (factor          != old_factor)          {preferences.putFloat("factor", factor);                  changed = true;}
  if (weight_empty    != old_weight_empty)    {preferences.putUInt("weight_empty", weight_empty);       changed = true;}
  if (correction      != old_correction)      {preferences.putUInt("correction", correction);           changed = true;}
  if (autostart       != old_autostart)       {preferences.putUInt("autostart", autostart);             changed = true;}
  if (autocorrection  != old_autocorrection)  {preferences.putUInt("autocorrection", autocorrection);   changed = true;}
  if (init_weight_f   != old_init_weight_f)   {preferences.putUInt("init_weight_f", init_weight_f);     changed = true;}
  if (overfill_gr     != old_overfill_gr)     {preferences.putUInt("overfill_gr", overfill_gr);         changed = true;}
  if (angle_min       != old_angle_min)       {preferences.putUInt("angle_min", angle_min);             changed = true;}
  if (fullmode        != old_fullmode)        {preferences.putUInt("fullmode", fullmode);               changed = true;}
  if (fquantity_index != old_fquantity_index) {preferences.putUInt("fquantity_index", fquantity_index); changed = true;}
  if (buzzermode      != old_buzzermode)      {preferences.putUInt("buzzermode", buzzermode);           changed = true;}
  if (ledmode         != old_ledmode)         {preferences.putUInt("ledmode", ledmode);                 changed = true;}
  if (showlogo        != old_showlogo)        {preferences.putUInt("showlogo", showlogo);               changed = true;}
  if (showcredits     != old_showcredits)     {preferences.putUInt("showcredits", showcredits);         changed = true;}
  if (cali_weight     != old_cali_weight)     {preferences.putUInt("cali_weight", cali_weight);         changed = true;}
  if (jartolerance    != old_jartolerance)    {preferences.putUInt("jartolerance", jartolerance);       changed = true;}
  if (current_servo   != old_current_servo)   {preferences.putUInt("current_servo", current_servo);     changed = true;}
  if (show_current    != old_show_current)    {preferences.putUInt("show_current", show_current);       changed = true;}
  if (color_scheme    != old_color_scheme)    {preferences.putUInt("color_scheme", color_scheme);       changed = true;}
  if (color_marker    != old_color_marker)    {preferences.putUInt("color_marker", color_marker);       changed = true;}
  if (use_turntable   != old_use_turntable)   {preferences.putUInt("use_turntable", use_turntable);     changed = true;}
  if (lingo           != old_lingo)           {preferences.putUInt("lingo", lingo);                     changed = true;}
  if (wait_befor_fill != old_wait_befor_fill) {preferences.putUInt("wait_befor_fill", wait_befor_fill); changed = true;}
  if (font_typ        != old_font_typ)        {preferences.putUInt("font_typ", font_typ);               changed = true;}
  if (menu_rotation   != old_menu_rotation)   {preferences.putUInt("menu_rotation", menu_rotation);     changed = true;}
  if (squee_tap_left  != old_squee_tap_left)  {preferences.putUInt("squee_tap_left", squee_tap_left);   changed = true;}
  if (servo_expanded  != old_servo_expanded)  {preferences.putUInt("servo_expanded", servo_expanded);   changed = true;}
  if (max_lc_weight   != old_max_lc_weight)   {preferences.putUInt("max_lc_weight", max_lc_weight);     changed = true;}
  if (max_lc_cal      != old_max_lc_cal)      {preferences.putUInt("max_lc_cal", max_lc_cal);           changed = true;}
  i = 0;
  while (i<5) {
    sprintf(output, "Gewicht%d", i);
    if (glaeser[i].Gewicht != old_jars[i].Gewicht) {preferences.putInt(output, glaeser[i].Gewicht); changed = true;}
    sprintf(output, "GlasTyp%d", i);
    if (glaeser[i].GlasTyp != old_jars[i].GlasTyp) {preferences.putInt(output, glaeser[i].GlasTyp); changed = true;}
    sprintf(output, "Tara%d", i);
    if (glaeser[i].Tare    != old_jars[i].Tare)    {preferences.putInt(output, glaeser[i].Tare);    changed = true;}
    i++;
  }
  i = 0;
  while (i<5) {
    sprintf(output, "angle_max%d", i);
    if (angle_max[i]      != old_angle_max[i])      {preferences.putInt(output, angle_max[i]);        changed = true;}
    sprintf(output, "angle_fine%d", i);
    if (angle_fine[i]     != old_angle_fine[i])     {preferences.putInt(output, angle_fine[i]);       changed = true;}
    sprintf(output, "finedos_weight%d", i);
    if (finedos_weight[i] != old_finedos_weight[i]) {preferences.putFloat(output, finedos_weight[i]); changed = true;}
    i++;
  }
  preferences.end();
  if (changed) {setPreferencesCheck();}
}

void HM_READ_WRITE_PREFERENCES::setPreferencesCheck() {
  #if DEBUG_HM >= 1
    Serial.println("Set preferences check");
  #endif
  old_factor          = factor;
  old_pos             = pos;
  old_weight_empty    = weight_empty;
  old_correction      = correction;
  old_autostart       = autostart;
  old_autocorrection  = autocorrection;
  old_init_weight_f   = init_weight_f;
  old_overfill_gr     = overfill_gr;
  old_fquantity_index = fquantity_index;
  old_angle_min       = angle_min;
  old_buzzermode      = buzzermode;
  old_ledmode         = ledmode;
  old_showlogo        = showlogo;
  old_showcredits     = showcredits;
  old_cali_weight     = cali_weight;
  old_jartolerance    = jartolerance;
  old_current_servo   = current_servo;
  old_show_current    = show_current;
  old_color_scheme    = color_scheme;
  old_color_marker    = color_marker;
  old_use_turntable   = use_turntable;
  old_lingo           = lingo;
  old_wait_befor_fill = wait_befor_fill;
  old_fullmode        = fullmode;
  old_font_typ        = font_typ;
  old_menu_rotation   = menu_rotation;
  old_squee_tap_left  = squee_tap_left;
  old_servo_expanded  = servo_expanded;
  old_max_lc_weight   = max_lc_weight;
  old_max_lc_cal      = max_lc_cal;
  int i = 0;
  while(i < 5) {
    old_jars[i].Gewicht    = glaeser[i].Gewicht;
    old_jars[i].GlasTyp    = glaeser[i].GlasTyp;
    old_jars[i].Tare       = glaeser[i].Tare;
    old_jars[i].TripCount  = glaeser[i].TripCount;
    old_jars[i].Count      = glaeser[i].Count;
    i++;
  }
  i = 0;
  while(i < 5) {
  
    old_angle_max[i]      = angle_max[i];
    old_angle_fine[i]     = angle_fine[i];
    old_finedos_weight[i] = finedos_weight[i];
    i++;
  }
}

void HM_READ_WRITE_PREFERENCES::set_rotary_scale_Preferences(int a) {
  preferences.begin("EEPROM", false);
  rotary_scale = a;
  preferences.putUInt("rotary_scale", rotary_scale);
  preferences.end();
}

void HM_READ_WRITE_PREFERENCES::clear_Preferences() {
  preferences.begin("EEPROM", false);
  preferences.clear();
  preferences.end();
}