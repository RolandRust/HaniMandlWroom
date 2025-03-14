// ** Definition der pins 
// ----------------------

#if HARDWARE_LEVEL == 1
  // Rotary Encoder
  const int outputA  = 33;
  const int outputB  = 26;
  const int outputSW = 32;
  // Servo
  const int servo_pin = 2;
  // 3x Schalter Ein 1 - Aus - Ein 2, VCC pins wurden direkt auf VCC gelegt
  const int switch_betrieb_pin = 15;
  const int switch_setup_pin   = 4;
  // Taster, VCC pins wurden direkt auf VCC gelegt
  const int button_start_pin     = 12;
  const int button_stop_pin      = 27;
  // Wägezelle-IC 
  const int hx711_sck_pin = 17;
  const int hx711_dt_pin  = 35;
  // Buzzer and LED
  static int buzzer_pin = 25;
  static int led_pin = 0;
#elif HARDWARE_LEVEL == 2 or HARDWARE_LEVEL == 3
  // Rotary Encoder
  const int outputA  = 47;  // Clk
  const int outputB  = 48;  // DT 
  const int outputSW = 26;
  // Servo
  const int servo_pin = 1;
  // 3x Schalter Ein 1 - Aus - Ein 2
  const int switch_betrieb_pin = 40;
  const int switch_vcc_pin     = 41;     // <- Vcc
  const int switch_setup_pin   = 42;
  const int vext_ctrl_pin      = 36;     // Vext control pin
  // Taster 
  const int button_start_vcc_pin =  7;  // <- Vcc
  const int button_start_pin     =  6;
  const int button_stop_vcc_pin  =  5;  // <- Vcc
  const int button_stop_pin      =  4;
  // Wägezelle-IC
  const int hx711_sck_pin = 38;
  const int hx711_dt_pin  = 39;
  // Buzzer - aktiver Piezo
  static int buzzer_pin = 2;
  static int led_pin = 35;
#else
  #error Hardware Level nicht definiert! Korrektes #define setzen!
#endif