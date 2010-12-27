/*
	This event will be called when the failsafe changes
	boolean failsafe reflects the current state
*/
void startup_ground_event()
{
}

void startup_air_event()
{
}

void failsafe_event()
{
	if (failsafe == true){
		// This is how to handle a failsafe.
		// 
		switch(control_mode)
		{
			case MANUAL:
				set_mode(STABILIZE);
				break;
	
			case STABILIZE:
				if (GPS_fix != FAILED_GPS){
					set_mode(RTL);
				}else{
					set_mode(CIRCLE);
				}
				break;
	
			case FLY_BY_WIRE_A: 
				if (GPS_fix != FAILED_GPS){
					set_mode(RTL);
				}else{
					set_mode(CIRCLE);
				}
				break;
			
			case FLY_BY_WIRE_B:
				if (GPS_fix != FAILED_GPS){
					set_mode(RTL);
				}else{
					set_mode(CIRCLE);
				}
				break;
		
			case CIRCLE:
				break;

			case AUTO: 
			case LOITER: 
			case LAND: 
			case TAKEOFF: 
				if (FAILSAFE_ACTION == 1){
					set_mode(RTL);
					throttle_cruise = THROTTLE_CRUISE;				
				}
				break;

			case RTL:
				break;
			
		}
	}else{
		reset_I();
	}
}

/*
	This event will be called when the switch changes
	It is up to the user how to react to a swicth change event
	options are: MANUAL, STABILIZE, FLY_BY_WIRE_A, FLY_BY_WIRE_B, AUTO, RTL, LOITER 
	see: defines.h
	
	The three switch postions can be handled by most radios.
	Adjust your seetings to make sure all three positions work.
	If you don't have a 3 postion switch, try a two position one 
	and note which case below activates in each position.
*/

void waypoint_event(byte event)
{
	switch(event)
	{
		case EVENT_WILL_REACH_WAYPOINT:
			// called just before wp_index is incemented
			Serial.print("MSG Reached WP:");
			Serial.println(wp_index,DEC);
			break;
			
		case EVENT_SET_NEW_WAYPOINT_INDEX:
			// called just after wp_index is incemented

			/*
				// Dead Simple Egg drop example:
				// This doens't take into account height and airspeed, etc.
				if(wp_index = 5){
					servo_out[CH_RUDDER] = -45;
				}
				
			*/
			break;

		case EVENT_LOADED_WAYPOINT:
			//Serial.print("Loaded WP index:");
			//Serial.println(wp_index,DEC);
			print_current_waypoints();
			reset_I();			
			break;			
	}
}


// called after every single control loop
void mainLoop_event(void)
{
/*
	if (control_mode == LOITER){
		if (wp_index == 2 && elapsedTime > 120000 ){ // 2 minutes
			elapsedTime = 0;
			// our waypoints index is not altered during LOITER
			// All we need to do is reload the waypoint
			load_waypoint();
			// and return to Autopilot mode!
			set_mode(AUTO);
		}
	}
*/

			/*
				// Complex Simple Egg drop example:
				// This doens't take into account height and airspeed, etc.
				if(wp_index == 5){
					servo_out[CH_RUDDER] = -45;
				}
				
			*/
}

void mediumLoop_event(void)
{
	/*
	float temp  = (float)(current_loc.alt - home.alt) * .01;
	egg_dist = sqrt(temp / 4.903) * (float)ground_speed *.01;

	if(wp_index == 3){
		wp_radius = 10;
		if(wp_distance < egg_dist){
			servo_out[CH_RUDDER] = PAYLOAD_OPEN;
		}
	}else{
		wp_radius = 20;	
		servo_out[CH_RUDDER] = PAYLOAD_CLOSED;
	}
	*/
}


void low_battery_event(void)
{
	#if DEBUG_SUBSYSTEM != 3
		Serial.println("MSG Low Batt");
		set_mode(RTL);
	#endif
}



		
