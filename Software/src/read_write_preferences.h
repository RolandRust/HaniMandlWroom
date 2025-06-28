//read_write_preferences.h

#ifndef __HM_READ_WRITE_PREFERENCES__
#define __HM_READ_WRITE_PREFERENCES__

class HM_READ_WRITE_PREFERENCES {
    private:
        void setPreferencesCheck();
    public:
        HM_READ_WRITE_PREFERENCES();
        void getPreferences();
        void setPreferences();
        void set_rotary_scale_Preferences(int a);
        void clear_Preferences();
};
#endif