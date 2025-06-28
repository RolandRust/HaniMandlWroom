#include <WString.h>
#include <vector>

#ifndef __HM_VARIABLES_DISPLAY__
#define __HM_VARIABLES_DISPLAY__

extern std::vector<const char*> menuitems;
extern size_t numItems;
extern std::vector<int> posmenu;
extern int font_typ;
extern bool restartMenu;
extern bool draw_frame;
extern int clear_tft;
extern int value_old;
extern bool change_value;
extern bool change_marker;
extern int menuitem;
extern int menuitem_old;
extern bool change;
extern bool change_menu;
extern String text_old;
extern char stars[];
extern size_t ota_current;
extern size_t ota_final;
extern String calibration_status;
extern String quetschhan;
extern int cal_winkel;
extern int y_offset_ina;
#endif