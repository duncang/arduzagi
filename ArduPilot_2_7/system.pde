/*****************************************************************************
The init_ardupilot function processes everything we need for an in-air restart
	We will determine later if we are actually on the ground and process a 
	ground start in that case.
	
Also in this function we will dump the log if applicable based on the slide switch
*****************************************************************************/
void init_ardupilot()
{

	#if GPS_PROTOCOL == -1
		Serial.begin(NO_GPS);
		Serial.println("NO GPS");
	#elif GPS_PROTOCOL == 0
		Serial.begin(NMEA_GPS);
		Serial.println("GPS: NMEA");
	#elif GPS_PROTOCOL == 1
		Serial.begin(EM406_GPS);
		Serial.println("GPS: EM406");
	#elif GPS_PROTOCOL == 2
		Serial.begin(UBLOX_GPS);
		Serial.println("GPS: uBlox");
	#elif GPS_PROTOCOL == 3
		Serial.begin(ARDU_IMU);  
		Serial.println("ARDU IMU");
	#elif GPS_PROTOCOL == 4
		Serial.begin(MTK_GPS);  
		Serial.println("MediaTek GPS");
	#elif GPS_PROTOCOL == 5
		Serial.begin(SIM_GPS);
		Serial.println("GPS: Sim");
	#endif

	Serial.println();
 	Serial.println("Init Ardupilot 2.7.1");
	
 	// ATMEGA ADC
 	// PC0 - ADC0 	- 23 - X sensor
 	// PC1 - ADC1 	- 24 - Y sensor
 	// PC2 - ADC2 	- 25 - Z sensor
 	// PC3 - ADC3 	- 26 - Pressure sensor
 	// PC4 - ADC4 	- 27 - 
 	// PC5 - ADC5 	- 28 - Battery Voltage
 	
 	// ATMEGA
	// PORTD
	// p0				// PD0 - RXD  		- Serial RX 	
	// p1				// PD1 - TXD  		- Serial TX 
	pinMode(2,INPUT);	// PD2 - INT0 		- Rudder in							- INPUT Rudder/Aileron
	pinMode(3,INPUT);	// PD3 - INT1 		- Elevator in 						- INPUT Elevator
	pinMode(4,INPUT);	// PD4 - XCK/T0 	- MUX pin							- Connected to Pin 2 on ATtiny
	pinMode(5,INPUT);	// PD5 - T0			- Mode pin							- Connected to Pin 6 on ATtiny   - Select on MUX
	pinMode(6,OUTPUT);	// PD6 - T1			- Ground start signaling Pin	
	pinMode(7,OUTPUT);	// PD7 - AIN0		- GPS Mux pin 
	// PORTB
	pinMode(8, OUTPUT); // PB0 - AIN1		- Servo throttle					- OUTPUT THROTTLE
	pinMode(9, OUTPUT);	// PB1 - OC1A		- Elevator PWM out					- Elevator PWM out
	pinMode(10,OUTPUT);	// PB2 - OC1B		- Rudder PWM out					- Aileron PWM out
	pinMode(11,INPUT); 	// PB3 - MOSI/OC2	-  
	pinMode(12,OUTPUT); // PB4 - MISO		- Blue LED pin  - GPS Lock			- GPS Lock
	pinMode(13,INPUT); 	// PB5 - SCK		- Yellow LED pin   					- INPUT Throttle

	// PORTC - Analog ports
	// PC0 - Thermopile - x
	// PC1 - Thermopile - y 
	// PC2 - Thermopile - z
	// PC3 - Airspeed
	// PC4 - CH4 OUT - Rudder output
	// PC5 - Battery

	// set Analog out 4 to output
	DDRC |= B00010000;
	

	digitalWrite(6,HIGH);
	
	// Enable GPS
	// ----------------
	setGPSMux();

	// setup control switch
	// ----------------
	initControlSwitch();
		
	// load launch settings from EEPROM
	// --------------------------------
	restore_EEPROM();

	// connect to radio 
	// ----------------
	init_radio();

	// setup PWM timers
	// ----------------
	init_PWM();

	// print the radio 
	// ---------------
	print_radio();
				
	// set the correct flight mode
	// ---------------------------
	reset_control_switch();

	Serial.print("freeRAM: ");
	Serial.println(freeRAM(),DEC);
	
	if(startup_check()){
		Serial.println("MSG Startup: Ground");
		startup_ground();
		startup_ground_event();
	}else{
		Serial.println("MSG Startup: AIR");
		takeoff_complete = true;
		ground_start_count = 0;
		
		// Load WP from memory
		// -------------------
		load_waypoint();
		startup_air_event();
	}
}

byte startup_check(void){
	if(DEBUG_SUBSYSTEM > 0){
		debug_subsystem();
	}else{
		if ((readSwitch() == 1) && (radio_in[CH_THROTTLE] < 1200)){
			// we are in manual
			return 1;
		}else{
			return 0;
		}
	}
}

//********************************************************************************
//This function does all the calibrations, etc. that we need during a ground start
//********************************************************************************
void startup_ground(void)
{
	#if USE_AUTO_LAUNCH == 1
		takeoff_complete	= false;			// Flag for using take-off controls
	#else
		takeoff_complete	= true;
	#endif

	// Configure GPS
	// -------------
	init_gps();

	// Output waypoints for confirmation
	// --------------------------------
	print_waypoints();

	//Signal the IMU to perform ground start
	//------------------------
	digitalWrite(6,LOW);
        
	// Makes the servos wiggle
	// step 1 = 1 wiggle
	// -----------------------
	demo_servos();

	// set a default reasonable ir_max val
	// -----------------------------------
	ir_max = 150;
		
	// read the radio to set trims
	// ---------------------------
	trim_radio();
	
	#if SET_RADIO_LIMITS == 1
	read_radio_limits();
	#endif

	#if AIRSPEED_SENSOR == 1
		// initialize airspeed sensor
		// --------------------------
		zero_airspeed();
	#else
		//Serial.println("NO airspeed");
	#endif
	
	// Number of reads before saving Home position
	// -------------------------------------------
	ground_start_count = 6;

	// Save the settings for in-air restart
	// ------------------------------------
	save_EEPROM_groundstart();

	// Lower signal pin in case of IMU power glitch
	// --------------------------------------------
	digitalWrite(6,HIGH);
	
	// Makes the servos wiggle - 3 times signals ready to fly
	// -----------------------
	demo_servos();
	demo_servos();
	demo_servos();

	print_launch_params();

	Serial.println(" ");
	Serial.println("MSG Ready to FLY. ");
}


void set_mode(byte mode)
{
	if(control_mode == mode){
		// don't switch modes if we are already in the correct mode.
		return;
	}
	#if AUTO_TRIM == 1
		if(control_mode == MANUAL) 
			trim_control_surfaces();
	#endif
	
	control_mode = mode;

	#if USE_AUTO_LAUNCH == 1
		if(control_mode == AUTO && takeoff_complete == false){
			control_mode = TAKEOFF;
		}
	#endif

	switch(control_mode)
	{
		case MANUAL:
			break;

		case STABILIZE:
			break;
		
		case FLY_BY_WIRE_A:
			break;
		
		case FLY_BY_WIRE_B:
			break;

		case AUTO:
			// reload the current WP and set control modes;
			load_waypoint();
			break;

		case RTL:
			return_to_launch();
			break;
		
		case LOITER:
			loiter_at_location();
			break;

		case TAKEOFF:
			break;

		case LAND:
			break;
	}
	
	// output control mode to the ground station
	send_message(MSG_HEARTBEAT);
}

void set_failsafe(boolean mode)
{
	// only act on changes
	// -------------------
	if(failsafe != mode){

		// store the value so we don't trip the gate twice
		// -----------------------------------------------
		failsafe = mode;

		if (failsafe == false){
			// We're back in radio contact
			// ---------------------------

			// re-read the switch so we can return to our preferred mode
			reset_control_switch();
			
			// Reset control integrators
			// ---------------------
			reset_I();

			// Release hardware MUX just in case it's not set
			// ----------------------------------------------
			set_servo_mux(false);
			
		}else{
			// We've lost radio contact
			// ------------------------
			// nothing to do right now
		}
		
		// Let the user know what's up so they can override the behavior
		// -------------------------------------------------------------
		failsafe_event();
	}
}



// This hack is to control the V2 shield so we can read the serial from 
// the XBEE radios - which is not implemented yet
void setGPSMux(void)
{
	#if SHIELD_VERSION < 1 || GPS_PROTOCOL == 3 // GPS_PROTOCOL == 3 -> With IMU always go low.
		digitalWrite(7, LOW); //Remove Before Fly Pull Up resistor
    #else
		digitalWrite(7, HIGH); //Remove Before Fly Pull Up resistor
	#endif
}


void setCommandMux(void)
{
	#if SHIELD_VERSION < 1
		digitalWrite(7, HIGH); //Remove Before Fly Pull Up resistor
    #else
		digitalWrite(7, LOW); //Remove Before Fly Pull Up resistor
	#endif
}

void update_GPS_light(void)
{
	// GPS LED on if we have a fix or Blink GPS LED if we are receiving data
	// ---------------------------------------------------------------------
	if(GPS_fix != VALID_GPS){
		GPS_light = !GPS_light;
		if(GPS_light){
			digitalWrite(BLUE_LED_PIN, HIGH);
		}else{
			digitalWrite(BLUE_LED_PIN, LOW);
		}		
	}else{
		if(!GPS_light){
			GPS_light = true;
			digitalWrite(BLUE_LED_PIN, HIGH);
		}
	}
}

/* This function gets the current value of the heap and stack pointers.
* The stack pointer starts at the top of RAM and grows downwards. The heap pointer
* starts just above the static variables etc. and grows upwards. SP should always
* be larger than HP or you'll be in big trouble! The smaller the gap, the more
* careful you need to be. Julian Gall 6-Feb-2009.
*/
unsigned long freeRAM() {
	uint8_t * heapptr, * stackptr;
	stackptr = (uint8_t *)malloc(4); // use stackptr temporarily
	heapptr = stackptr; // save value of heap pointer
	free(stackptr); // free up the memory again (sets stackptr to 0)
	stackptr = (uint8_t *)(SP); // save value of stack pointer
	return stackptr - heapptr;
}


//***********************************************************************************
//  The following functions are used during startup and are not for telemetry
//***********************************************************************************

void print_radio()
{
	#if GCS_PROTOCOL == 3
	Serial.print("MSG ");
	#endif
	Serial.print("Radio in A: ");
	Serial.print(radio_in[CH_ROLL],DEC);
	Serial.print("\tE: ");
	Serial.print(radio_in[CH_PITCH],DEC);
	Serial.print("\tT :");
	Serial.print(radio_in[CH_THROTTLE],DEC);
	Serial.print("\tR :");
	Serial.println(radio_in[CH_RUDDER],DEC);
}

void print_waypoints(){
	#if GCS_PROTOCOL == 3
	Serial.print("MSG ");
	#endif

	Serial.print("WP Total: ");
	Serial.println(wp_total, DEC);
	// create a location struct to hold the temp Waypoints for printing
	//Location tmp;
	struct Location tmp = get_wp_with_index(0);
	
	#if GCS_PROTOCOL == 3
	Serial.print("MSG ");
	#endif
	Serial.print("home: \t");
	Serial.print(tmp.lat, DEC);
	Serial.print("\t");
	Serial.print(tmp.lng, DEC);
	Serial.print("\t");
	Serial.println(tmp.alt,DEC);	

	for (int i = 1; i <= wp_total; i++){
		tmp = get_wp_with_index(i);
		#if GCS_PROTOCOL == 3
		Serial.print("MSG ");
		#endif
		Serial.print("wp #");
		Serial.print(i);
		Serial.print("\t");
		Serial.print(tmp.lat, DEC);
		Serial.print("\t");
		Serial.print(tmp.lng, DEC);
		Serial.print("\t");
		Serial.println(tmp.alt,DEC);
	}
}

void print_launch_params(void)
{
	/*
	Serial.println("LAUNCH PARAMETERS");
	Serial.println(wp_total,DEC);
	Serial.print("radius: ");
	Serial.println(wp_radius,DEC);
	Serial.print("radio_trim[CH_ROLL] = \t\t");
	Serial.println(radio_trim[CH_ROLL],DEC);
	Serial.print("radio_trim[CH_PITCH] = \t\t");
	Serial.println(radio_trim[CH_PITCH],DEC);
	Serial.print("radio_trim[CH_THROTTLE] = \t\t");
	Serial.println(radio_trim[CH_THROTTLE],DEC);	
	Serial.print("radio_min[CH_ROLL] = \t");
	Serial.println(radio_min[CH_ROLL],DEC);
	Serial.print("radio_max[CH_ROLL] = \t");
	Serial.println(radio_max[CH_ROLL],DEC);
	Serial.print("radio_min[CH_PITCH] = \t");
	Serial.println(radio_min[CH_PITCH],DEC);
	Serial.print("radio_max[CH_PITCH] = \t");
	Serial.println(radio_max[CH_PITCH],DEC);
	*/
}



