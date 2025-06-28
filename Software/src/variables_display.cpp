#include "variables_display.h"

//std::vector<const char*> menuitems = {};
std::vector<const char*> menuitems = {};
size_t numItems;
std::vector<int> posmenu;
int font_typ = 1;
bool restartMenu = false;
bool draw_frame = false;
int clear_tft = 0;
int value_old = -1;
bool change_value = false;
bool change_marker = false;
int menuitem = -1;
int menuitem_old = -1;
bool change = false;
bool change_menu = false;
String text_old = "";
char stars[] = "";
size_t ota_current;
size_t ota_final;
String calibration_status = "";
String quetschhan = "";
int cal_winkel = 0;
int y_offset_ina = 0;