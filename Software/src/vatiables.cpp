// variables.cpp

#include "variables.h"

//Scale, manual mode, automatic mode
int weight;
int tare;
int tare_jar;
long weight_empty;
float factor;
int fquantity;
int fquantity_index;
int mode = -1;
int auto_enabled = 0;
int scale_present = 0;
//Load Cell
int max_lc_weight = 5000;
int max_lc_cal = 500;
//Automatic
int correction;
int autostart;
int autocorrection;
int init_weight_f;
int init_weight_f_D = 1;
int angle_marker;
int loop1_c;
int overfill_gr;
int jartolerance = 20;
int wait_befor_fill = 0;
int stop_wait_befor_fill = 0;
int autocorrection_gr = 0;
int progressbar = 0;
int target_weight;
//Servo
int angle;
int angle_hard_min = 0;
int angle_hard_max = 180;
int angle_min = 0;
int angle_max[5] = {85,85,85,85,85};
int angle_fine[5] = {35,35,35,35,35};
float finedos_weight[5] = {60,60,60,60,60};
int fullmode = 1;
int servo_enabled = 0;
bool servo_live = false;
int squee_tap_left = 0;
int servo_expanded = 1;
//TFT
int color_scheme = 0;
int color_marker = 2;
bool make_changes;
int menuoffset_tft;
bool y_pos_weight;
//Language
int lingo = 0;
//Buzzer, LED, Logo, Credits
int buzzermode = 0;
int ledmode = 0;
int showlogo = 1;
int showcredits = 1;
//Rotary
int pos;
int pos_old;
int menu_rotation = 0;
int rotary_select = SW_WINKEL;
int rotary_scale = 0;
//Calibration scale
#if USER == 2
    int cali_weight = 2000;
#elif USER == 3
    int cali_weight = 500;
#else
    int cali_weight = 500;
#endif
//Turntable
int use_turntable = 0;
bool turntable_init = false;
bool turntable_moving = false;
bool drip_prodection = false;
bool turntable_ok = true;
bool turntable_init_check = true;
bool turntable_jar_full_flag = false;
int channel = 1;
String esp_now_wait_old; 
String esp_now_wait;
bool esp_now_change;
bool turntable_running = false;
bool center_jar_running = false;
int jar_center_pos = 0;
int speed_init = 0;
int speed_run = 0;
int speed_init_old = 0;
int speed_run_old = 0;
int ts_speed = 0;
int ts_waittime = 0;
int ts_angle_min = 0;
int ts_angle_max = 180;
int ota_status = 0;
char myReceivedMessage_text[30];
int myReceivedMessage_value;
//WiFI
String ssid = "";
String password = "";
const char* apSSID = "HaniMandl_Setup";
const char* apPassword = "12345678";
uint8_t baseMac[6];
char ip[30];
//OTA
int ota_done = 0;
int progress = 0;
char new_version[] = "";
//ESPnow
bool esp_now_ini = false;
char esp_now_textMsg[] = "";
bool esp_now_msg_recived = false;
bool esp_now_send_error = true;
//INA219
int ina219_installed = 0;
int current_servo = 0;
int current_mA;
int updatetime_ina219 = 500;
int last_ina219_measurement = 0;
int overcurrenttime = 1500;
int last_overcurrenttime = 0;
int alarm_overcurrent = 0;
int show_current = 0;
int inawatchdog = 1;
int offset_angle = 0;
//Diverses
float filling_weight = 0;
bool jar_on_scale = false;
//Variables for TFT Update
bool no_ina;
int scaletime;
int weight_old;
int angle_min_old;
int angle_ist_old;
int tare_old;
int tare_old_automatic;
int current_mA_old;
int servo_enabled_old;
int auto_enabled_old;
int jar_old;
int correction_old;
int autocorrection_gr_old;
int intWeight_old;
//Automatic mode
bool full = false;
//Jar struct and arrray
#if USER == 2
const char *GlasTypArray[2] = {"TOF", "Spezial"};
const char *GlasTypArrayTFT[2] = {"TwistOff", "SPZ"};
struct glas glaeser[5] = { 
                            {50, 0, -9999, 0, 0},
                            {125, 0, -9999, 0, 0},
                            {250, 0, -9999, 0, 0},
                            {500, 0, -9999, 0, 0},
                            {1000, 0, -9999, 0, 0} 
                          };
#elif USER == 3
const char *GlasTypArray[1] = {"TOF"};
const char *GlasTypArrayTFT[1] = {"TwistOff"};
struct glas glaeser[5] = { 
                            {250, 0, -9999, 0, 0},
                            {500, 0, -9999, 0, 0},
                            {0, 0, -9999, 0, 0},
                            {0, 0, -9999, 0, 0},
                            {0, 0, -9999, 0, 0} 
                          };
#else
const char *GlasTypArray[3] = {"DIB", "TOF", "DEE"};
const char *GlasTypArrayTFT[3] = {"DeepTwist", "TwistOff", "DE Imker Bund"};
struct glas glaeser[5] = { 
                            {125, 0, -9999, 0, 0},
                            {250, 1, -9999, 0, 0},
                            {250, 2, -9999, 0, 0},
                            {500, 1, -9999, 0, 0},
                            {500, 0, -9999, 0, 0} 
                          };
#endif