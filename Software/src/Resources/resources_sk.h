char LANGUAGE1[]            = "Jazyk";
char LANGUAGE2[]            = "Slovenský";

//Process Setup
char TAREVALUES[]			  = "Tara hodnoty";
char SCALE[]			      = "Váha";	
char FILL_QUANTITY[]		= "Mn. náplne";				//Množstvo náplne
char AUTOMATIC[]			  = "Automaticky";
char SERVOSETTINGS[]		= "Nastav servo";			//Nastavenia serva
char PARAMETER[]			  = "Parameter";
char COUNTER[]				  = "Pocítadlo";
char COUNTER_TRIP[]			= "Pocet plnení";			//Počet naplnení
char CLEAR_PREFS[]			= "Vymaz predvol";			//Vymazať predvoľby
char INA219_SETUP[]			= "INA219 Setup";

// Setup Scale
char CALIBRATION[]			= "Kalibrácia";

//Setup Tarevalues
char TAREVALUES_JAR[]		= "Tara hodnoty pohárov";	//DISPLAY:	Tarevalues jars
char MISSING[]				= "chýbajú";				//			missing

//Setup Calibration
//Prosím vyprázdnite váhu a potvrďte s OK
char PLEASE_EMPTY_THE[]		= "Prosím, vyprázdnite";	//DISPLAY:	Please empty the
char SCALE_AND_CONFIRM[]	= " váhu a potvrdte";		//			scale and confirm
char WITH_OK_1[]			= " s OK";					//			with OK
//Prosím nastavte {hodnotu} a potvrďte OK
char PLEASE_SET_UP[]		= "Prosím, nastavte";		//DISPLAY:	Please set up
char AND_CONFIRM[]			= " a potvrdte";			//			{500g} and confirm
char WITH_OK_2[]			= " s OK";					//			with ok

//Nastavenie množstva pohárov
char FILL_QUANTITY_JARS[]	= "Naplnených pohárov";

//Setup Automatik
char AUTOSTART[]			= "Autostart";
char JAR_TOLERANCE[]		= "Toler. pohára";
char CORRECTION[]			= "Korekcia";
char AUTOCORRECTION[]		= "Autokorekcia";
char KINDNESS[]				= "Preplnenie";
char FLOW_G_OVER_TIME[]		= "Prietok g/cas";
char WAIT_BEFOR_FILL[]		= "Cakaj pred napl.";

//Setup Servo
char LIVESETUP[]			= "Nastav. nazivo";
char MINIMUM[]				= "Minimum";
char FINEDOSAGE[]			= "Dávkovanie";
char MAXIMUM[]				= "Maximum";

//Setup Parameter
char BUZZER[]				        = "Bzuciak";
char LED_1[]				        = "LED";					//Variable LED is used in the Library from the Heltec Module :-(
char SHOW_LOGO[]			      = "Zobraz logo";
char SHOW_CREDITS[]			    = "Zobraz zásluhy";
char COLORSCHEME[]			    = "Farebná schéma";
char MARKER_COLOR[]			    = "Farba znacky";
char CHANGE_ROTATION[]      = "Change rotation"

//Setup INA219
char SERVO_CURRENT[]		= "Prúd serva";
char CAL_HONEY_GATE[]		= "Cal. honey gate";		//Calibrate honey gate
char SHOW_CURRENT[]			= "Zobrazit prúd";
char MAX_CURRENT[]			= "max. prúd";
char MIN_ANGLE[]			= "min. uhol";
char CALIBRATION_RUNNING[]	= "Prebieha kalibrácia";
char CURRENT[]				= "Prúd";
char ANGLE[]				= "Uhol";
char HONEY_GATE[]			= "Honey gate";
char CALIBRATION_DONE[]		= "Kalibrácia je hotová";

//Setup Clear Preferenzes
char CLEAR_PREFERENCES[]	= "Vymazat predvolby";
char CLEAR_NVS_MEMORY[]		= "Vymazat NVS pamät";
char RESET_TURNTABLE[]	    = "Reset tocne";

//Setup točne (otočného stola)
char TURNTABLE[]            = "Tocna";
char INIT_TURNTABLE[]       = "Inic. tocne";
char SETUP_TURNTABLE[]      = "Setup tocne";
char SETUP_DRIPPRODECTION[] = "Setup OPO";				//Ochrana proti odkvapkávaniu (OPO)
//char ADJUST_JAR_POS[]       = "Adjust jar  pos.";			//Adjust jar  position
//char STEPS[]                = "Steps";
//char POSITION[]             = "Position";
char MOVE_JAR[]             = "Posun pohár";
char SPEED_INIT[]           = "Inic. rýchl.";
char SPEED_RUN[]            = "Rýchlost behu";
char MOVE_POS[]             = "Posun pozic.";
char SETUP_STEPPER[]        = "Setup steppera";
char SETUP_SERVO[]          = "Setup serva";
char OKKAY[]                = "ok";                         //ok [max 3 letters]
char NOKAY[]                = "nok";                        //not ok [max 3 letters]
char CENTER_JAR[]           = "Centr. pohár";
char OPEN_DRIPPROTECTION[]  = "Otvor OPO";             		//otvor ochranu proti odkvapkávaniu
char CLOSE_DRIPPROTECTION[] = "Zatvor OPO";       //zatvor ochranu proti odkvapkávaniu
char SPEED_DRIPPROTECTION[] = "Rýchlost OPO";       //rýchlosť ochrany proti odkvapkávaniu
char WAIT_TO_CLOSE_DP[]     = "Cakaj na otv. OPO";           //čakaj na otvorenie ochrany proti odkvapkávaniu
char DP_MIN_ANGLE[]         = "min uhol OPO";               //min uhol ochrany proti odkvapkávaniu
char DP_MAX_ANGLE[]         = "max uhol OPO";               //max uhol ochrany proti odkvapkávaniu
//Connection failed
char CONNECTION[]           = "Pripojenie";                 //Zobraz: Pripojenie
char FAILED[]               = " zlyhalo";                     //         zlyhalo
//Close drip protection wait time reading failed
//Zlyhalo čítanie čakacieho času na ochranu pred odkvapkávaním
char CLOSE_DRIP_DP[]        = "Zlyhalo cítanie";                 //Zobraz: Close drip
char PROTECTION_WAIT_DP[]   = "cakacieho casu";            //         protection wait
char TIME_READING_DP[]      = "na ochranu pred";               //         time reading
char FAILED_DP[]            = "odkvapkávaním";                     //         failed
char ENABLE_OTA_UPDATE[]    = "Povolit OTA Update";

//Setup About
char ABOUT[]                = "O";
char VERSION[]              = "Verzia";
char MAC[]                  = "MAC";                        //MAC adress
char WLAN_CHANNEL[]         = "WLAN kanál";
char USE_TURNTABLE[]        = "Pouzitie tocne";
char OTA_UPDATE[]           = "OTA update";

//Warnings
//bez váhy
char NO[] 					= "ziadne";							//DISPLAY:	no
char SCALE_1[]			= "váha!";						//			scale!
//nie je kalibrované
char NOT[]					= "nie je";						//DISPLAY:	not
char CALIBRATED[]			= "kalibrované";					//			calibrated
//prázdna váha
char EMPTY[]				= "prázdna";						//DISPLAY:	empty
char THE_SCALE[]			= "váha!";					//			the scale!

//Manual Modus
char MANUAL[]				= "Manuálne";
char SERVO[]				= "Servo";
char TARE[]					= "Tara";
char INA[]					= "INA";
char CURR[]					= "Prúd";                      //Current
char MAX_1[]				= "max";
char ACTUAL[]				= "Aktuálne";
char A_UPPER_CASE[]			= "U";							//[Angle - only OLED display]

//Automatic Modus
char INA219[]				= "INA219";
char AUTOCORR[]				= "Autokor.";					//[no space for Autocorrection]
char MIN[]					= "Min";						//[max 3 letters]
char MAX_2[]				= "Max";						//[max 3 letters]
char ACT[]					= "Akt";						//[max 3 letters - no space for Actual]
//Prosím vložte pohár
char PLEASE_PUT[]			= "Vlozte";					//DISPLAY:	Please put
char UP_THE_JAR[]			= " pohár";					//			up the jar
char TARE_JAR[]				= "Tara pohára";
//Pokračujte s tlačidlom štart
char CONTINUE_WITH[]        = "Pokracujte s";              //DISPLAY:  Continue with
char THE_START_BUTTON[]     = "tlacidlom Start";           //          the start button
char FLOW[]				    = "Prietok:";
char AS[]				    = "AS";							//[Autostart - only OLED Display]
char A_LOWER_CASE[]			= "a";							//[angle or auto - only OLED Display]
char C_LOWER_CASE[]			= "c";							//[correction - only OLED Display]
char F_LOWER_CASE[]			= "f";							//[flow - only OLED Display]


char NO_TARE[]				= "nie je tara";

char OFF[]					= "off";
char ON[]					= "on";
char DARK[]					= "tmavý";
char LIGHT[]				= "svetlý";
char TOTAL[]				= "Celkom";
char RESET[]				= "Reset";
char ABORT[]				= "Zrusit";
char START[]				= "Start";
char OPEN[]					= "otvorit";
char CLOSE[]				= "zatvorit";
char BACK[]					= "Spät";
char SAVE[]					= "Ulozit";