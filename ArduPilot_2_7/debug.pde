#if DEBUG_SUBSYSTEM == 1
void debug_subsystem()
{
	Serial.println("Begin Debug: Radio Subsystem ");
	while(1){
		delay(20);
		// Filters radio input - adjust filters in the radio.pde file
		// ----------------------------------------------------------
		read_radio();
		Serial.print("Radio in ch1: ");
		Serial.print(radio_in[CH_ROLL],DEC);
		Serial.print("\tch2: ");
		Serial.print(radio_in[CH_PITCH],DEC);
		Serial.print("\tch3:");
		Serial.print(ch3_raw,DEC);
		Serial.print("\tch4:");
		Serial.println(radio_in[CH_RUDDER],DEC);
	}
}
#endif

#if DEBUG_SUBSYSTEM == 2
void debug_subsystem()
{
	Serial.println("Begin Debug: Servo Subsystem ");
	Serial.print("Reverse ROLL - CH1: ");
	Serial.println(REVERSE_ROLL,DEC);
	Serial.print("Reverse PITCH - CH2: ");
	Serial.println(REVERSE_PITCH,DEC);
	Serial.print("Reverse THROTTLE - CH3: ");
	Serial.println(REVERSE_THROTTLE,DEC);
	Serial.print("Reverse RUDDER - CH4: ");
	Serial.println(REVERSE_RUDDER,DEC);

	// read the radio to set trims
	// ---------------------------
	trim_radio();
	/*
	Serial.print("radio_min ch1: ");
	Serial.print(radio_min[CH_ROLL],DEC);
	Serial.print(" radio_trim ch1: ");
	Serial.print(radio_trim[CH_ROLL],DEC);
	Serial.print(" radio_max ch1: ");
	Serial.println(radio_max[CH_ROLL],DEC);

	Serial.print("radio_min ch2: ");
	Serial.print(radio_min[CH_PITCH],DEC);
	Serial.print(" radio_trim ch2: ");
	Serial.print(radio_trim[CH_PITCH],DEC);
	Serial.print(" radio_max ch2: ");
	Serial.println(radio_max[CH_PITCH],DEC);

	Serial.print("radio_min ch3: ");
	Serial.print(radio_min[CH_THROTTLE],DEC);
	Serial.print(" radio_trim ch3: ");
	Serial.print(radio_trim[CH_THROTTLE],DEC);
	Serial.print(" radio_max ch3: ");
	Serial.println(radio_max[CH_THROTTLE],DEC);

	Serial.print("radio_min ch4: ");
	Serial.print(radio_min[CH_RUDDER],DEC);
	Serial.print(" radio_trim ch4: ");
	Serial.print(radio_trim[CH_RUDDER],DEC);
	Serial.print(" radio_max ch4: ");
	Serial.println(radio_max[CH_RUDDER],DEC);
	*/
	while(1){
		delay(20);
		// Filters radio input - adjust filters in the radio.pde file
		// ----------------------------------------------------------
		read_radio();
		servo_out[CH_ROLL]  = (float)((radio_in[CH_ROLL]  - radio_trim[CH_ROLL]) * 45  * REVERSE_ROLL) / 500;
		servo_out[CH_PITCH] = (float)((radio_in[CH_PITCH] - radio_trim[CH_PITCH]) * 45 * REVERSE_PITCH) / 500;
		servo_out[CH_RUDDER] = (float)((radio_in[CH_RUDDER] - radio_trim[CH_RUDDER]) * 45 * REVERSE_RUDDER) / 500;

		// write out the servo PWM values
		// ------------------------------
		set_servos_4();
		///*
		Serial.print("Servo_out ch1: ");
		Serial.print(servo_out[CH_ROLL],DEC);
		Serial.print("\tch2: ");
		Serial.print(servo_out[CH_PITCH],DEC);
		Serial.print("\tch3: ");
		Serial.print(servo_out[CH_THROTTLE],DEC);
		Serial.print("\tch4: ");
		Serial.print(servo_out[CH_RUDDER],DEC);

		Serial.print("\tRadio out ch1: ");
		Serial.print(radio_out[CH_ROLL],DEC);
		Serial.print("\tch2: ");
		Serial.print(radio_out[CH_PITCH],DEC);
		Serial.print("\tch3:");
		Serial.print(radio_out[CH_THROTTLE],DEC);
		Serial.print("\tch4:");
		Serial.println(radio_out[CH_RUDDER],DEC);
		//*/
	}
}
#endif


#if DEBUG_SUBSYSTEM == 3
void debug_subsystem()
{
	Serial.println("Begin Debug: Analog Sensor Subsystem ");
	
	Serial.print("Shield Version: ");
	Serial.println(SHIELD_VERSION,DEC);

	Serial.print("AirSpeed sensor enabled: ");
	Serial.println(AIRSPEED_SENSOR,DEC);
	
	Serial.print("ENABLE_Z_SENSOR: ");
	Serial.println(ENABLE_Z_SENSOR,DEC);
	
	Serial.print("Enable Battery: ");
	Serial.println(BATTERY_EVENT,DEC);
	zero_airspeed();
	
	Serial.print("Air pressure offset:");
	Serial.println(airpressure_offset,DEC);

	while(1){
		delay(20);
		#if ENABLE_Z_SENSOR == 1
		read_z_sensor();
		#endif
		read_XY_sensors();
		read_airspeed();
		read_battery();
		
		Serial.print("Analog IN:");
		Serial.print("  0:");
		Serial.print(analog0,DEC);
		Serial.print(", 1: ");
		Serial.print(analog1,DEC);
		Serial.print(", 2: ");
		Serial.print(analog2,DEC);
		Serial.print(", 3: ");
		Serial.print(airpressure_raw,DEC);
		
		Serial.print("\t\tSensors:");
		Serial.print("  ir_max:");
		Serial.print(ir_max,DEC);
		Serial.print("  roll:");
		Serial.print(roll_sensor/100,DEC);	
		Serial.print("  pitch:");
		Serial.print(pitch_sensor/100,DEC);
		Serial.print("  airSpeed:");
		Serial.print(airspeed,DEC);
		Serial.print("m \tbattV:");
		Serial.print(battery_voltage,DEC);
		Serial.println("volts ");
	}
}
#endif

#if DEBUG_SUBSYSTEM == 4
void debug_subsystem()
{
	Serial.println("Begin Debug: GPS Subsystem ");
	init_gps();
	
	while(1){
		delay(333);

		// Blink GPS LED if we don't have a fix
		// ------------------------------------
		update_GPS_light();
		
		decode_gps();

		if (GPS_new_data){
			Serial.print("Lat:");
			Serial.print(current_loc.lat,DEC);
			Serial.print(" Lon:");
			Serial.print(current_loc.lng,DEC);
			Serial.print(" Alt:");
			Serial.print(current_loc.alt/100,DEC);
			Serial.print("m, gs: ");
			Serial.print(ground_speed/100,DEC);
			Serial.print(" COG:");
			Serial.print(ground_course/1000000);
			Serial.print(" SAT:");
			Serial.print(NumSats,DEC);
			Serial.print(" FIX:");
			Serial.print(GPS_fix,DEC);
			Serial.print(" TIM:");
			Serial.print(iTOW);
			Serial.println();
		}
	}
}
#endif

#if DEBUG_SUBSYSTEM == 5
void debug_subsystem()
{
	Serial.println("Begin Debug: GPS Subsystem, RAW OUTPUT");
	init_gps();
	
	while(1){

		if(Serial.available()>0)	//Ok, let me see, the buffer is empty?
		{
			
			delay(60);	// wait for it all
			while(Serial.available()>0){
				byte incoming = Serial.read();
				//Serial.print(incoming,DEC);
				Serial.print(incoming,HEX); // will output Hex values
				Serial.print(",");
			}
			Serial.println(" ");
		}

	}
}
#endif

#if DEBUG_SUBSYSTEM == 6
void debug_subsystem()
{
	Serial.println("Begin Debug: IMU Subsystem ");
	
	while(1){
		delay(20);
		
		// We are using the IMU
		// ---------------------
		decode_gps();
		Serial.print("  roll:");
		Serial.print(roll_sensor/100,DEC);	
		Serial.print("  pitch:");
		Serial.print(pitch_sensor/100,DEC);
		Serial.print("  course:");
		Serial.println(ground_course/100,DEC);


		if (GPS_new_data){
			Serial.print("Lat:");
			Serial.print(current_loc.lat,DEC);
			Serial.print(" Lon:");
			Serial.print(current_loc.lng,DEC);
			Serial.print(" Alt:");
			Serial.print(current_loc.alt/100,DEC);
			Serial.print("m, gs: ");
			Serial.print(ground_speed/100,DEC);
			Serial.print(" TIM:");
			Serial.print(iTOW,DEC);
			Serial.print(" Health:");
			Serial.println(imu_health,DEC);
		}
	}
}
#endif

#if DEBUG_SUBSYSTEM == 7
void debug_subsystem()
{
	Serial.println("Begin Debug: Control Switch Test");
	
	while(1){
		delay(20);
		byte switchPosition = readSwitch();
		if (oldSwitchPosition != switchPosition){
			
			switch(switchPosition)
			{
				case 1: // First position
				Serial.println("Position 1 - Manual");

				break;
		
				case 2: // middle position
				Serial.println("Position 2");
				break;
		
				case 3: // last position
				Serial.println("Position 3");
				break;
			}
	
			oldSwitchPosition = switchPosition;
		}
	}
}
#endif

#if DEBUG_SUBSYSTEM == 8
void debug_subsystem()
{
	Serial.println("Begin Debug: Throttle Subsystem ");
	read_radio();
	
	uint16_t low_pwm = ch3_raw;
	uint16_t high_pwm = ch3_raw;
	
	while(1){
		delay(20);
		// Filters radio input - adjust filters in the radio.pde file
		// ----------------------------------------------------------
		read_radio();
		update_throttle();
		low_pwm = min(low_pwm, ch3_raw);
		high_pwm = max(high_pwm, ch3_raw);
		
		Serial.print("ch3: ");
		Serial.print(ch3_raw,DEC);
		Serial.print("\tRadio_in limited: ");
		Serial.print(radio_in[CH_THROTTLE],DEC);
		Serial.print("\tPWM output: ");
		Serial.print(radio_out[CH_THROTTLE],DEC);
		Serial.print("\tThrottle: ");
		Serial.print(servo_out[CH_THROTTLE],DEC);
		Serial.print("\tPWM Min: ");
		Serial.print(low_pwm,DEC);
		Serial.print("\tPWM Max: ");
		Serial.println(high_pwm,DEC);
	}
}
#endif

#if DEBUG_SUBSYSTEM == 9
void debug_subsystem()
{
	Serial.println("Begin Debug: Radio Min Max ");

	//example output
	//CH1: 1159|1895   CH2: 1108|1944   CH3: 1107|1927   CH4: 929|2099   

	uint16_t low_pwm[4];
	uint16_t high_pwm[4];
	uint8_t i;

	for(i = 0; i < 100;i++){
		delay(20);
		read_radio();
	}
	
	for(i = 0; i < 4; i++){
		radio_min[i]	= 0;
		radio_max[i]	= 2400;
	 	low_pwm[i]		= radio_in[i];
 		high_pwm[i]		= radio_in[i];
	}
	
	while(1){
		delay(100);
		// Filters radio input - adjust filters in the radio.pde file
		// ----------------------------------------------------------
		read_radio();
		for(i = 0; i < 4; i++){
			low_pwm[i] = min(low_pwm[i], radio_in[i]);
			high_pwm[i] = max(high_pwm[i], radio_in[i]);
		}
		
		
		
		for(i = 0; i < 4; i++){
			Serial.print("CH");
			Serial.print(i+1, DEC);
			Serial.print(": ");
			low_pwm[i] = min(low_pwm[i], radio_in[i]);
			Serial.print(low_pwm[i],DEC);
			Serial.print("|");
			high_pwm[i] = max(high_pwm[i], radio_in[i]);
			Serial.print(high_pwm[i],DEC);
			Serial.print("   ");
		}
		Serial.println(" ");
	}
}
#endif

#if DEBUG_SUBSYSTEM == 10
void debug_subsystem()
{
	Serial.println("Begin Debug: Display Waypoints");	
	delay(500);
	
	eeprom_busy_wait();
	int16_t alt_to_hold = eeprom_read_word((uint16_t *)	EE_ALT_HOLD_HOME);

	byte options = eeprom_read_byte((byte *) EE_CONFIG);

	// save the alitude above home option
	if(options & HOLD_ALT_ABOVE_HOME){
		Serial.println("Maintain current altitude ");
	}else{
		Serial.print("Hold this altitude over home ");
		Serial.print(alt_to_hold,DEC);
		Serial.println("meters");
	}
	
	Serial.print("Number of Waypoints: ");
	Serial.println(wp_total, DEC);

	Serial.print("Hit radius for Waypoints: ");
	Serial.println(wp_radius, DEC);

	Serial.print("Loiter radius around Waypoints: ");
	Serial.println(loiter_radius, DEC);

	struct Location tmp = get_wp_with_index(0);

	Serial.print("home: \t");
	Serial.print("\tlat: ");
	Serial.print((float)tmp.lat/t7, DEC);
	Serial.print("\tlong: ");
	Serial.print((float)tmp.lng/t7, DEC);
	Serial.print("\talt: ");
	Serial.println(tmp.alt/100,DEC);

	for (int i = 1; i <= wp_total; i++){
		tmp = get_wp_with_index(i);
		Serial.print("wp #");
		Serial.print(i);
		Serial.print("\tlat: ");
		Serial.print((float)tmp.lat/t7, DEC);
		Serial.print("\tlong: ");
		Serial.print((float)tmp.lng/t7, DEC);
		Serial.print("\talt: ");
		Serial.println(tmp.alt/100,DEC);
	}

}
#endif
