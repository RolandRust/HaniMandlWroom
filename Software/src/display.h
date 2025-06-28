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
        void clear();
        void set_rotary_scale();
        void print_logo();
        void print_credits();
        void not_calibrated();
        void no_scale();
        void empty_the_scale();
        //Manual mode
        void process_manualmode();
        //Automatic mode
        void process_automatic(int a);
        //Setup
        void process_setup();
        void setup_servoWinkel();
        void setup_automatik();
        void setup_parameter();
        void setup_language();
        void setup_clear_prefs();
        void setup_wifi();
        void setup_about();
        void setup_tare();
        void setup_scale();
        void setup_max_weight();
        
        void setup_counter(int a);
        void setup_trip_counter(int a);
        void setup_fuellmenge(int a);
        void setup_calibration(int a);
        void setup_INA219(int a);
        void setup_webif(int i);
        void onlineOTA(int i);
        void ota_setup(int a);
        void setup_turntable(int a);



        void setup_exit();
};
#endif