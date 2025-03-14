//display.h

#include <WString.h>

#ifndef __HM_Display__
#define __HM_Display__

class HM_Display {
    private:
	
    public:
        HM_Display();
        //Startup
        void begin();
        void print_logo();
        void print_credits();
        void not_calibrated();
        void no_scale();
        void empty_the_scale();
        //Setup
        void process_setup(const char *menuitems[], int posmenu[], int menuitems_number, int menuitem, int menuitem_old);
        void setup_parameter_oled(bool change_value, int menuitem, int menuitem_used);
        void setup_parameter_oled_exit(int menuitem);
        void setup_parameter_tft(int menuitems_number, bool change_value, const char *menuitems[]);
        void setup_parameter_tft_exit();
        void setup_language_oled(int pos, int menuitems_number, const char *menuitems[]);
        void setup_language_oled_exit();
        void setup_language_tft(int pos, int menuitems_number, const char *menuitems[]);
        void setup_language_tft_exit();
        void setup_calibration_oled(int a);
        void setup_calibration_tft(int a);
        void setup_tare_oled(int pos);
        void setup_tare_tft(int pos);
        void setup_fuellmenge_oled(int a);
        void setup_fuellmenge_tft(int a);
        void setup_automatik_oled (int menuoffset, bool change_value, int menuitem, int menuitem_used);
        void setup_automatik_oled_exit (int menuitem);
        void setup_automatik_tft (bool change_value, const char *menuitems[], int menuitems_number);
        void setup_automatik_tft_exit ();
        void setup_servoWinkel_oled(bool change_value, int menuitem, int menuitem_used);
        void setup_servoWinkel_oled_exit(int menuitem);
        void setup_servoWinkel_tft(bool change_value, int menu_items_number, const char *menuitems[]);
        void setup_servoWinkel_tft_exit();
        void setup_counter_oled(int a);
        void setup_counter_oled_exit();
        void setup_counter_tft(int a);
        void setup_counter_tft_exit();
        void setup_trip_counter_oled(int a);
        void setup_trip_counter_oled_exit();
        void setup_trip_counter_tft(int a);
        void setup_trip_counter_tft_exit();
        void setup_INA219_menu_oled(int menuitem, int menuitem_used, bool change_value);
        void setup_INA219_oled_save(int menuitem);
        void setup_INA219_cal1_oled(String calibration_status, bool, int menuitem, int menuitem_used);
        void setup_INA219_cal2_oled(String calibration_status, int cal_winkel, String quetschhan);
        void setup_INA219_cal3_oled(String calibration_status);
        void setup_INA219_menu_tft(bool change_value, int menu_items_number, const char *menuitems_1[], const char *menuitems_2[]);
        void setup_INA219_tft_save();
        void setup_INA219_cal1_tft(String calibration_status, bool change_value, int menuitem, int menuitem_used);
        void setup_INA219_cal2_tft(String calibration_status, int cal_winkel, String quetschhan);
        void setup_INA219_cal3_tft(String calibration_status);
        void setup_about_oled(uint8_t baseMac[]);
        void setup_about_tft(uint8_t baseMac[]);
        void setup_clear_prefs_oled(int menu_items_number);
        void setup_clear_prefs_tft(int menu_items_number, const char *menuitems[]);
        void setup_clear_prefs_oled_exit();
        void setup_clear_prefs_tft_exit();
        void setup_turntable_no_connection_oled();
        void setup_turntable_menu1_1_oled();
        void setup_turntable_menu1_2_oled(bool change_value, const char *menuitems_1[], int menuitem_1, int menu_items_number_1, int last_menu_pos_1, bool turntable_running, String esp_now_wait);
        void setup_turntable_menu1_oled_exit(int menuitem_1);
        void setup_turntable_menu2_1_oled();
        void setup_turntable_menu2_2_oled(bool change_value, const char *menuitems_2[], int menuitem_2, int menu_items_number_2, int ota_update);
        void setup_turntable_menu2_3_oled();
        void setup_turntable_menu2_4_oled();
        void setup_turntable_menu2_5_oled(char myReceivedMessage_text[]);
        void setup_turntable_menu2_6_oled(char myReceivedMessage_text[], int myReceivedMessage_value);
        void setup_turntable_menu2_7_oled();
        void setup_turntable_menu2_8_oled();
        void setup_turntable_menu2_9_oled();
        void setup_turntable_menu2_oled_exit(int menuitem_2);
        void setup_turntable_menu3_1_oled();
        void setup_turntable_menu3_2_oled(bool change_value, const char *menuitems_3[], int menuitem_3, int menu_items_number_3, String esp_now_wait, int jar_center_pos, int speed_init, int speed_run);
        void setup_turntable_menu3_oled_exit(int menuitem_3);
        void setup_turntable_menu4_1_oled();
        void setup_turntable_menu4_2_oled(bool change_value, const char *menuitems_4[], int menuitem_4, int menu_items_number_4, int ts_speed, int ts_waittime, int ts_angle_min, int ts_angle_max);
        void setup_turntable_menu4_oled_exit(int menuitem_4);
        void setup_setup_oled(const char *menuitems[], int menu_items_number);
        void setup_setup_oled_exit();
        void setup_turntable_frame();
        void setup_turntable_no_connection_tft();
        void setup_turntable_menu1_1_tft();
        void setup_turntable_menu1_2_tft(bool change_value, const char *menuitems_1[], int menu_items_number_1, bool turntable_running);
        void setup_turntable_menu1_tft_exit();
        void setup_turntable_menu2_1_tft();
        void setup_turntable_menu2_2_tft(bool change_value, const char *menuitems_2[], int menu_items_number_2, int ota_update); //hat ein problem 
        void setup_turntable_menu2_3_tft();
        void setup_turntable_menu2_4_tft();
        void setup_turntable_menu2_5_tft(char myReceivedMessage_text[]);
        void setup_turntable_menu2_6_tft(char myReceivedMessage_text[], int myReceivedMessage_value);
        void setup_turntable_menu2_7_tft();
        void setup_turntable_menu2_8_tft();
        void setup_turntable_menu2_9_tft(int a);
        void setup_turntable_menu2_tft_exit();
        void setup_turntable_menu3_1_tft();
        void setup_turntable_menu3_2_tft(bool change_value, const char *menuitems_3[], int menu_items_number_3, bool turntable_running, int jar_center_pos, int speed_init, int speed_run);
        void setup_turntable_menu3_tft_exit();
        void setup_turntable_menu4_1_tft();
        void setup_turntable_menu4_2_tft(bool change_value, const char *menuitems_4[], int menu_items_number_4, int ts_speed, int ts_waittime, int ts_angle_min, int ts_angle_max);
        void setup_turntable_menu4_tft_exit();
        void setup_setup_tft(const char *menuitems[], int menu_items_number);
        void setup_setup_tft_exit();
        //OTA
        void ota_setup1_oled();
        void ota_setup2_oled(char x[]);
        void ota_setup3_oled(char ip[]);
        void ota_setup4_oled();
        void on_ota_start_oled();
        void on_ota_progress_oled(size_t current, size_t final);
        void on_on_end1_oled(bool success);
        void on_on_end2_oled();
        void ota_disconnect_oled();
        void ota_setup1_tft();
        void ota_setup2_tft(char x[]);
        void ota_setup3_tft(char ip[]);
        void ota_setup4_tft();
        void ota_on_ota_start_tft();
        void ota_on_ota_progress_tft(size_t current, size_t final);
        void ota_on_on_end1_tft(bool success);
        void ota_on_on_end2_tft();
        void ota_disconnect_tft();
        //Manual mode
        void process_manualmode_oled();
        void process_manualmode_tft(int a);
        //Automatic mode
        void process_automatic_clear_oled();
        void process_automatic_connection_failed_oled();
        void process_automatic_init_turntable_nok_oled();
        void process_automatic_read_time_close_tripprodection_failed_oled();
        void process_automatic_start_oled();
        void process_automatic_wait_befor_fill_oled();
        void process_automatic_main_oled(int rotary_select, int SW_KORREKTUR, int SW_FLUSS, int SW_MENU, bool full, int y_offset_ina);
        void process_automatic_clear_tft();
        void process_automatic_connection_failed_tft();
        void process_automatic_init_turntable_nok_tft();
        void process_automatic_read_time_close_tripprodection_failed_tft();
        void process_automatic_init_screen_tft(int a);
        void process_automatic_start_tft();
        void process_automatic_set_tare_jar_tft();
        void process_automatic_wait_befor_fill_tft();
        void process_automatic_main_tft(int rotary_select, int SW_KORREKTUR, int SW_FLUSS, int SW_MENU);
        //WebIF
        void setup_webif_oled(int i);
        void setup_webif_tft(int i);
};
#endif