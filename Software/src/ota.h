//ota.h

#ifndef __HM_OTA__
#define __HM_OTA__

class HM_OTA {
    private:
	
    public:
        HM_OTA();
        void ota_search_chanel();
        void ota_setup(const int switch_setup_pin, const int button_stop_pin);
};
#endif