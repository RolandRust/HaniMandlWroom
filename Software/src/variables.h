//variables.h

#include <WString.h>

#ifndef __HM_VARIABLES__
#define __HM_VARIABLES__
    //Version
    //extern String version;
    //Scale, manual mode, automatic mode
    extern int weight;                                      // current weight
    extern int tare;                                        // tare for chosen jar, for automatic mode
    extern int tare_jar;                                    // tare for current jar, in case jar weight differs
    extern long weight_empty;                               // weight of empty scale
    extern float factor;                                    // scaling factor for scale values
    extern int fquantity;                                   // chosen fill quantity
    extern int fquantity_index;                             // index in jars[]
    extern int mode;                                        // wether to drive the servo to minimum on mode change
    extern int auto_enabled;                                // for automatic mode system on/off?
    extern int scale_present;                               // not talking to HX711 when no scale is connected, otherwise crash
    //Automatic
    extern int correction;                                  // correction value for fill quantity
    extern int autostart;                                   // full automatic on/off
    extern int autocorrection;                              // autocorrection on/off
    extern int init_weight_f;                               // A.P. Flow rate per run which should flow (g / unit of time) Filling speed, this should give a constant speed when filling, regardless of the filling level.
    extern int init_weight_f_D;                             // A.P. allowed deviation from intWeight  
    extern int angle_marker;                                // A.P. Marker for optimal angle for next filling
    extern int loop1_c;                                     // A.P. Counter for the passes  
    extern int overfill_gr;                                 // desired overfilling for autcorrection mode in grams
    extern int jartolerance;                                // weight for autostart may vary by +-20g, total 40g!
    extern int wait_befor_fill;                             // Flag if waiting is to be carried out in automatic mode after the tare
    extern int stop_wait_befor_fill;                        // Stop button is pressed while in wait_before_fill mode
    extern int autocorrection_gr;
    extern int progressbar;                                 // Variable for the progress bar
    extern int target_weight;                               // Jar + Correction
    //Servo
    extern int angle;                                       // current servo angle
    extern int angle_hard_min;                              // hard limit for servo
    extern int angle_hard_max;                              // hard limit for servo
    extern int angle_min;                                   // configurable in setup
    extern int angle_max[5];                                // array for the angle_max for the full mode
    extern int angle_fine[5];                               // array for the angle_fine for the full mode
    extern float finedos_weight[5];                         // array for the finedos_weight for the full mode
    extern int fullmode;                                    // Variable for the different preset angles
    extern int servo_enabled;                               // activate servo yes/no
    extern bool servo_live;                                 // activate servo in the servo setup menu
    //TFT
    extern int color_scheme;                                // 0 = dark, 1 = lite / Change of color scheme for the TFT display
    extern int color_marker;                                // Color for the marker for the TFT display
    extern bool change_scheme;                              // Helper for TFT display
    extern bool change_marker;                              // Helper for TFT display
    extern int value_old;                                   // Helper for TFT display
    extern bool change;                                     // Helper for TFT display
    extern bool make_changes;                               // Helper for TFT display
    extern String text_old;                                 // Helper for TFT display
    extern bool draw_frame;                                 // Helper for TFT display
    extern int menuoffset_tft;                              // Helper for TFT display, only used in the automatic menu
    extern bool y_pos_weight;                               // Helper for TFT display
    extern int font_typ;                                    // Font type
    //Language
    extern int lingo;                                       // Variable for the language
    //Buzzer, LED, Logo, Credits
    extern int buzzermode;                                  // 0 = off, 1 = on. TODO: button sounds as buzzermode 2?
    extern int ledmode;                                     // 0 = off, 1 = on.
    extern int showlogo;                                    // 0 = off, 1 = on
    extern int showcredits;                                 // 0 = off, 1 = on
    //Rotary
    extern int pos;                                         // current position of rotary
    extern int pos_old;                                     // Helper for TFT display
    extern int menu_rotation;                               // Changes the rotation from the main sctoll menu
    //Calibration scale
    extern int cali_weight;                                 // choosen weight for calibration
    //Turntable
    extern int use_turntable;                               // 0 = Turntable is not used, 1 = Turntable is used
    extern bool turntable_init;                             // false = Turntable Init not done, true = Turntable Init done.
    extern bool turntable_moving;                           // Turntable does not rotate --> false, Turntable is rotating --> true
    extern bool drip_prodection;                            // false = Drip protection is open, true = Drip protection is closed
    extern bool turntable_ok;                               // true if Glass is moved and drip guard is open. If the turntable is not used, the default value is true
    extern bool turntable_init_check;                       // Turntable is in the correct place --> true, Turntable is in the wrong place --> false
    extern bool turntable_jar_full_flag;                    // Set as true if the glass is full
    extern int channel;                                     // Standard channel number for ESPnow. If OTA is activated, this will be changed. The startup then takes longer because the channel has to be searched for. Only if turntable is activated
    extern String esp_now_wait_old;                         // Helper for TFT display
    extern String esp_now_wait;                             // Helper for TFT display
    extern bool esp_now_change;                             // Helper for TFT display
    //WiFI
    extern String ssid;                                     //WiFi SSID
    extern String password;                                 //WiFI password
    extern const char* apSSID;                              //Access point SSID
    extern const char* apPassword;                          //Access point Pasword
    //OTA
    extern int ota_done;                                    // Variable for OTA update
    //ESPnow
    extern bool esp_now_ini;                                // true if esp now is active
    extern char esp_now_textMsg[];                          // send string for ESP NOW
    extern bool esp_now_msg_recived;                        // true if something was received
    extern bool esp_now_send_error;                         // false = Data has been sent , true = Data was not sent
    //INA219
    extern int ina219_installed;                            // 0 = no INA219 installed, 1 = INA219 is isnatalled
    extern int current_servo;                               // 0 = INA219 is ignored, 1-1000 = Allowed maximum current from the servo in mA
    extern int current_mA;                                  // Current that the servo use
    extern int updatetime_ina219;                           // Time in ms in which the INA219 is read (500 -> A current measurement is taken every 0.5 seconds)
    extern int last_ina219_measurement;                     // Last time in which the current was measured
    extern int overcurrenttime;                             // Time in which the servo is allowed to draw more current than set in ms
    extern int last_overcurrenttime;                        // Last time in which no overcurrent was measured
    extern int alarm_overcurrent;                           // Alarm flag when the servo is drawing too much current 
    extern int show_current;                                // 0 = off, 1 = on / Show the current even if the INA is ignored
    extern int inawatchdog;                                 // 0 = off, 1 = on / s required to suspend INA measurement
    extern int offset_angle;                                // Offset in degrees from the closing angle when the servo had overcurrent (max +3 degrees from the set angle min)
    //Diverses
    extern float filling_weight;                            // only used in the travel jar values menu
    extern bool jar_on_scale;                               // Flag for display update in automatic mode
    //Variables for TFT Update
    extern bool no_ina;
    extern int scaletime;
    extern int weight_old;
    extern int angle_min_old;
    //extern int pos_old;
    extern int angle_ist_old;
    extern int tare_old;
    extern int tare_old_automatic;
    extern int current_mA_old;
    extern int servo_enabled_old;
    extern int auto_enabled_old;
    extern int jar_old;
    extern int correction_old;
    extern int autocorrection_gr_old;
    extern int intWeight_old;
    //Jar struct and arrray
    struct glas{ 
        int Gewicht;
        int GlasTyp;
        int Tare;
        int TripCount;
        int Count;
    };
    extern glas glaeser[5];
    #if USER == 2
        extern const char *GlasTypArray[2];
        extern const char *GlasTypArrayTFT[2];
    #elif USER == 3
        extern const char *GlasTypArray[1];
        extern const char *GlasTypArrayTFT[1];
    #else
        extern const char *GlasTypArray[3];
        extern const char *GlasTypArrayTFT[3];
    #endif
#endif