void save_wp_index()
{
	eeprom_busy_wait();
	eeprom_write_byte((uint8_t *)EE_WP_INDEX, wp_index);
}

void loiter_at_location()
{
	next_WP = current_loc;
}

void return_to_launch()
{
	// home is WP 0
	// ------------
	wp_index = 0;

	// Loads WP from Memory
	// --------------------
	load_waypoint();

	// Altitude to hold over home
	// Set by configuration tool
	// -------------------------
	next_WP.alt = read_alt_to_hold();
}

long read_alt_to_hold()
{
	eeprom_busy_wait();
	byte options = eeprom_read_byte((byte *) EE_CONFIG);
	
	// save the alitude above home option
	if(options & HOLD_ALT_ABOVE_HOME){
		return current_loc.alt;
	}else{
		eeprom_busy_wait();
		int hold = eeprom_read_word((uint16_t *) EE_ALT_HOLD_HOME);
		return home.alt + ((long)hold * 100);
	}
}

void reached_waypoint()
{
	if (control_mode == RTL){
		set_mode(LOITER);
		return;
	}
	if(control_mode == LAND){
		// dont load a new waypoint
		return;
	}
	
	if (control_mode == AUTO){
	
		// load the next waypoint
		// ----------------------
		wp_index++;
		
		// Trigger landing mode
		// ----------------------
		if((wp_index == wp_total) && (USE_AUTO_LAND == 1)){
			set_mode(LAND);
		}
				
		if(wp_index > wp_total){
			set_mode(RTL);
			return;
		}else{
			Serial.print(iTOW, DEC);
			Serial.print(" Moving on to waypoint: ");
			Serial.println(wp_index,DEC);
		}
		
		// notify user of new index selection
		// -------------------------------------
		waypoint_event(EVENT_SET_NEW_WAYPOINT_INDEX);

		// grab new loaction from EEPROM
		// -----------------------------
		load_waypoint();			
	}
}

// run this whenever the wp_index changes
// -------------------------------------
void load_waypoint()
{
	// Save current waypoint index to EEPROM
	// -------------------------------------
	save_wp_index();

	// copy the current WP into the OldWP slot
	// ---------------------------------------
	prev_WP = current_loc;

	// Load the next_WP slot
	// ---------------------
	next_WP = get_wp_with_index(wp_index);

	// offset the altitude relative to home position
	// ---------------------------------------------
	//next_WP.alt += home.alt;

	if (control_mode == RTL) {
		// moved from return_to_launch()
		// Altitude to hold over home
		// Set by configuration tool
		// -------------------------
		next_WP.alt = read_alt_to_hold();
	} else {
		next_WP.alt += home.alt;
	}


	// used to control FBW and limit the rate of climb
	// -----------------------------------------------
	target_altitude = current_loc.alt;
	offset_altitude = next_WP.alt - current_loc.alt;

	// let them know we have loaded the WP
	// -----------------------------------
	waypoint_event(EVENT_LOADED_WAYPOINT);
	
	// do this whenever Old and new WP's change
	// ---------------------------------------------
	precalc_waypoint_distance();

	// zero out our loiter vals to watch for missed waypoints
	loiter_delta = 0;
	loiter_sum = 0;	
	
}
// Precalc for navigation algorithms
// called on each WP change
// ---------------------------------
void precalc_waypoint_distance(void)
{
	// this is used to offset the shrinking longitude as we go towards the poles	
	float rads = (abs(next_WP.lat)/t7) * 0.0174532925;
	//377,173,810 / 10,000,000 = 37.717381 * 0.0174532925 = 0.658292482926943		
	scaleLongDown = cos(rads);
	scaleLongUp = 1.0f/cos(rads);


	// this is handy for the groundstation
	wp_totalDistance 	= getDistance(&current_loc, &next_WP);
	wp_distance  		= wp_totalDistance;
	
	target_bearing 		= get_bearing(&current_loc, &next_WP);
	old_target_bearing 	= target_bearing;

	// set a new crosstrack bearing
	// ----------------------------
	reset_crosstrack();
	
	// output the new WP information to the Ground Station
	// ---------------------------------------------------
	#if GCS_PROTOCOL == 5
	print_new_wp_info();
	#endif
}


// run this at setup on the ground
// -------------------------------
void init_home()
{
	Serial.println("init home");
	// Copy our current location to home
	// ---------------------------------
	home = current_loc;
	
	// Save Home to EEPROM
	// -------------------
	set_wp_with_index(home, 0);

	// Save prev loc
	// -------------
	prev_WP = current_loc;
	
	// Set the current WP index to 1 (first WP)
	// ----------------------------------------
	wp_index = 1;
	
	// read wp 1 into memory
	// ---------------------
	load_waypoint();
}

struct Location set_wp_with_index(struct Location temp, int i)
{
	temp.lat /= 10;
	temp.lng /= 10;		// lat and long stored as * 1,000,000
	temp.alt /= 100; 	// altitude is stored as meters 
	
	if (i == 0){
		// Save Home location to EEPROM
		// ----------------------------
		eeprom_busy_wait();
		eeprom_write_dword((uint32_t *)	EE_HOME_LAT, temp.lat);
		eeprom_busy_wait();
		eeprom_write_dword((uint32_t *)	EE_HOME_LNG, temp.lng);
		eeprom_busy_wait();
		eeprom_write_word((uint16_t *)	EE_HOME_ALT, (int)temp.alt);

	}else{
	
		/* this is not being used right now
		// --------------------------------
		long mem_position = (long)(WP_START_BYTE + (i-1) * WP_SIZE);
    	eeprom_busy_wait();		
		eeprom_write_dword((uint32_t *)	mem_position, temp.lat);
		eeprom_busy_wait();
		mem_position += 4;
		eeprom_write_dword((uint32_t *)	mem_position, temp.lng);
		eeprom_busy_wait();
		mem_position += 4;
		eeprom_write_word((uint16_t *)	mem_position, (int)temp.alt);
		*/
	}
}


struct Location get_wp_with_index(int i)
{
	struct Location temp;
	long mem_position;

	// Find out proper location in memory by using the start_byte position + the index
	// --------------------------------------------------------------------------------
	if (i == 0) {
		// read home position 
		eeprom_busy_wait();
		temp.lat = (long)eeprom_read_dword((uint32_t*)EE_HOME_LAT);eeprom_busy_wait();
		temp.lng = (long)eeprom_read_dword((uint32_t*)EE_HOME_LNG);eeprom_busy_wait();
		temp.alt = 0;
		temp.alt = (long)eeprom_read_word((uint16_t*)EE_HOME_ALT);

		temp.lat *= 10;
		temp.lng *= 10;
		temp.alt *= 100;
		return temp;
		
	}else{
		// read WP position 
		mem_position = (long)(WP_START_BYTE + (i-1) * WP_SIZE);
		eeprom_busy_wait();
		temp.lat = (long)eeprom_read_dword((uint32_t*)mem_position);
		mem_position += 4;
		eeprom_busy_wait();
		temp.lng = (long)eeprom_read_dword((uint32_t*)mem_position);
		mem_position += 4;
		temp.alt = 0;
		eeprom_busy_wait();
		temp.alt = (long)eeprom_read_word((uint16_t*)mem_position);
		
		temp.lat *= 10;
		temp.lng *= 10;
		temp.alt *= 100;

		return temp;
	}
}

void readPoints()
{
    for (byte i = 0; i < wp_total; i++){
   		struct Location temp = get_wp_with_index(i);
		Serial.print("waypoint # ");
		Serial.print(i,DEC);
		Serial.print("\tlat: ");
		Serial.print(temp.lat,DEC);
		Serial.print("\tlong: ");
		Serial.print(temp.lng,DEC);
		Serial.print("\talt: ");
		Serial.println(temp.alt,DEC);
	}
}


byte get_waypiont_mode(void)
{
	eeprom_busy_wait();
	return eeprom_read_byte((uint8_t*)0x3E7);
}


// utility to reset WP index to 0 se we can restart mission
void reset_waypoint_index(void){
	//Serial.println("reset_waypoint_index");
	wp_index = 1;	// first WP
	load_waypoint();
}


