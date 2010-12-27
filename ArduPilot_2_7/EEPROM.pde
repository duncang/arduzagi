void save_EEPROM_groundstart(void)
{	
	// Sensor settings
	eeprom_busy_wait();
	eeprom_write_byte((uint8_t *)	EE_WP_INDEX, 	wp_index);					eeprom_busy_wait();
	eeprom_write_word((uint16_t *)	EE_AP_OFFSET, 	airpressure_offset);		eeprom_busy_wait();
	eeprom_write_word((uint16_t *)	EE_IR_MAX, 		ir_max);					eeprom_busy_wait();


	// Radio settings
	eeprom_write_word((uint16_t *)	EE_CH1_MIN, radio_min[CH_ROLL]);			eeprom_busy_wait();
	eeprom_write_word((uint16_t *)	EE_CH2_MIN, radio_min[CH_PITCH]);			eeprom_busy_wait();
	eeprom_write_word((uint16_t *)	EE_CH4_MIN, radio_min[CH_RUDDER]);			eeprom_busy_wait();

	eeprom_write_word((uint16_t *)	EE_CH1_TRIM, radio_trim[CH_ROLL]);			eeprom_busy_wait();
	eeprom_write_word((uint16_t *)	EE_CH2_TRIM, radio_trim[CH_PITCH]);			eeprom_busy_wait();
	eeprom_write_word((uint16_t *)	EE_CH4_TRIM, radio_trim[CH_RUDDER]);		eeprom_busy_wait();

	eeprom_write_word((uint16_t *)	EE_CH1_MAX, radio_max[CH_ROLL]);			eeprom_busy_wait();
	eeprom_write_word((uint16_t *)	EE_CH2_MAX, radio_max[CH_PITCH]);			eeprom_busy_wait();
	eeprom_write_word((uint16_t *)	EE_CH4_MAX, radio_max[CH_RUDDER]);			eeprom_busy_wait();

	eeprom_write_word((uint16_t *)	EE_ELEVON1_TRIM, 	elevon1_trim);			eeprom_busy_wait();
	eeprom_write_word((uint16_t *)	EE_ELEVON2_TRIM, 	elevon2_trim);
}

void restore_EEPROM(void)
{
	// if this is a first time use
	// check_eeprom_defaults();
	
	// Read out user options
	// ----------------------
	eeprom_busy_wait();
	wp_total 						= eeprom_read_byte((uint8_t *)	EE_WP_TOTAL);	eeprom_busy_wait();	//includes home
	wp_index 						= eeprom_read_byte((uint8_t *)  EE_WP_INDEX); 	eeprom_busy_wait();	// or return current waypoint

	wp_radius 						= eeprom_read_byte((uint8_t *)	EE_WP_RADIUS);	eeprom_busy_wait();
	airpressure_offset 				= eeprom_read_word((uint16_t *)	EE_AP_OFFSET);	eeprom_busy_wait(); 
	ir_max							= eeprom_read_word((uint16_t *)	EE_IR_MAX);		eeprom_busy_wait(); 

	// Radio settings
	/*
	radio_min[CH_ROLL] 				= eeprom_read_word((uint16_t *)	EE_CH1_MIN);	eeprom_busy_wait();
	radio_min[CH_PITCH] 			= eeprom_read_word((uint16_t *)	EE_CH2_MIN);	eeprom_busy_wait();
	radio_min[CH_THROTTLE] 			= eeprom_read_word((uint16_t *)	EE_CH3_MIN);	eeprom_busy_wait();
	radio_min[CH_RUDDER] 			= eeprom_read_word((uint16_t *)	EE_CH4_MIN);	eeprom_busy_wait();

	radio_trim[CH_ROLL] 			= eeprom_read_word((uint16_t *)	EE_CH1_TRIM);	eeprom_busy_wait();
	radio_trim[CH_PITCH] 			= eeprom_read_word((uint16_t *)	EE_CH2_TRIM);	eeprom_busy_wait();
	radio_trim[CH_THROTTLE] 		= eeprom_read_word((uint16_t *)	EE_CH3_TRIM);	eeprom_busy_wait();
	radio_trim[CH_RUDDER] 			= eeprom_read_word((uint16_t *)	EE_CH4_TRIM);	eeprom_busy_wait();

	radio_max[CH_ROLL] 				= eeprom_read_word((uint16_t *)	EE_CH1_MAX);	eeprom_busy_wait();
	radio_max[CH_PITCH] 			= eeprom_read_word((uint16_t *)	EE_CH2_MAX);	eeprom_busy_wait();
	radio_max[CH_THROTTLE] 			= eeprom_read_word((uint16_t *)	EE_CH3_MAX);	eeprom_busy_wait();
	radio_max[CH_RUDDER] 			= eeprom_read_word((uint16_t *)	EE_CH4_MAX);	eeprom_busy_wait();

	elevon1_trim 					= eeprom_read_word((uint16_t *)	EE_ELEVON1_TRIM);	eeprom_busy_wait();
	elevon2_trim 					= eeprom_read_word((uint16_t *)	EE_ELEVON2_TRIM);	eeprom_busy_wait();
	*/

	// lets fix broken values
	// ----------------------
	wp_index 						= constrain(wp_index, 0, wp_total);

	wp_radius 						= constrain(wp_radius, 					10, 	50);
	airpressure_offset 				= constrain(airpressure_offset, 		0, 		512);
	ir_max	 						= constrain(ir_max, 					40, 	512);

	radio_min[CH_ROLL] 				= constrain(radio_min[CH_ROLL], 		800, 	2200);
	radio_min[CH_PITCH] 			= constrain(radio_min[CH_PITCH], 		800, 	2200);
	radio_min[CH_RUDDER] 			= constrain(radio_min[CH_PITCH], 		800, 	2200);

	radio_trim[CH_ROLL] 			= constrain(radio_trim[CH_ROLL], 		800, 	2200);
	radio_trim[CH_PITCH] 			= constrain(radio_trim[CH_PITCH], 		800, 	2200);
	radio_trim[CH_RUDDER] 			= constrain(radio_trim[CH_RUDDER], 		800, 	2200);
 
 	radio_max[CH_ROLL] 				= constrain(radio_max[CH_ROLL], 		800, 	2200);
	radio_max[CH_PITCH] 			= constrain(radio_max[CH_PITCH],	 	800, 	2200);
	radio_max[CH_RUDDER] 			= constrain(radio_max[CH_PITCH],	 	800, 	2200);
	
	elevon1_trim 					= constrain(elevon1_trim, 				950, 	2050);
	elevon2_trim 					= constrain(elevon2_trim, 				950, 	2050);
	
	// load home latitude, long, alt
	// -----------------------------
	home = get_wp_with_index(0);
	
	// load next WP
	// ------------
	load_waypoint();
}

void check_eeprom_defaults(void)
{
	int test = eeprom_read_word((uint16_t *) EE_CH1_TRIM);	eeprom_busy_wait();
	
	if (test < 100){
		
		eeprom_busy_wait();
		eeprom_write_byte((uint8_t *)	EE_WP_RADIUS, 15);		eeprom_busy_wait();	// default WP radius
		eeprom_write_word((uint16_t *)	EE_AP_OFFSET,  250);	eeprom_busy_wait();	// airpressure_offset
		eeprom_write_word((uint16_t *)	EE_IR_MAX, 200);		eeprom_busy_wait();	// ir_max 

		eeprom_write_word((uint16_t *)	EE_CH1_TRIM, 1500);		eeprom_busy_wait();	// radio_trim[CH_ROLL]
		eeprom_write_word((uint16_t *)	EE_CH2_TRIM, 1500);		eeprom_busy_wait();	// radio_trim[CH_PITCH]
		eeprom_write_word((uint16_t *)	EE_CH4_TRIM, 1500);		eeprom_busy_wait(); // radio_trim[CH_RUDDER]

	}
}

void set_max_altitude_speed(void)
{
	if(get_altitude_above_home() > max_altitude) {
	  max_altitude = get_altitude_above_home();
	  eeprom_busy_wait();
	  eeprom_write_word((unsigned int*)EE_MAX_ALT, (max_altitude/100));
	}
	
	if(ground_speed > max_speed){
	  max_speed = ground_speed;
	  eeprom_busy_wait();
	  eeprom_write_word((unsigned int*)EE_MAX_SPEED,(max_speed/100));
	}
}
