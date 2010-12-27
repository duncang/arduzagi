
//****************************************************************
// Function that will calculate the desired direction to fly and altitude error
//****************************************************************
void navigate()
{
	// do not navigate with corrupt data
	// ---------------------------------
	/*if (GPS_fix == FAILED_GPS){
		if(control_mode != CIRCLE){
			nav_roll = 300;  // Set up a gentle bank
			nav_pitch = 0;
		}
		return;
	}*/
	
	// We only perform most nav computations if we have new gps data to work with
	// --------------------------------------------------------------------------
	if(GPS_new_data){
		GPS_new_data = false;
		
		// target_bearing is where we should be heading 
		// --------------------------------------------
		target_bearing 	= get_bearing(&current_loc, &next_WP);
	
		// nav_bearing will includes xtrac correction
		// ------------------------------------------
		nav_bearing = target_bearing;

		// waypoint distance from plane
		// ----------------------------
		wp_distance = getDistance(&current_loc, &next_WP);
		
		if (wp_distance <= 0){
			Serial.print("MSG WP error  ");
			Serial.println(wp_distance,DEC);
			print_current_waypoints();
			return;
		}

		// control mode specific updates to nav_bearing
		update_navigation();
	    
	}
}

/*
void calc_distance_error()
{	
	//distance_estimate 	+= (float)ground_speed * .0002 * cos(radians(bearing_error * .01));
	//distance_estimate 	-= DST_EST_GAIN * (float)(distance_estimate - GPS_wp_distance);
	//wp_distance  		= max(distance_estimate,10);
}
*/
void calc_bearing_error()
{
	bearing_error = nav_bearing - ground_course;
	if (bearing_error > 18000)	bearing_error -= 36000;
	if (bearing_error < -18000)	bearing_error += 36000;
	//bearing_error2 = bearing_error;
}

void calc_altitude_error() 
{
	// limit climb rates
	target_altitude = next_WP.alt - ((float)((wp_distance -20) * offset_altitude) / (float)(wp_totalDistance - 20));
	if(prev_WP.alt > next_WP.alt){
		target_altitude = constrain(target_altitude, next_WP.alt, prev_WP.alt);
	}else{
		target_altitude = constrain(target_altitude, prev_WP.alt, next_WP.alt);
	}
	/*
	#if AIRSPEED_SENSOR
		// special thanks to Ryan Beall for this one
		float pitch_angle 	= pitch_sensor - AOA; // pitch_angle = pitch sensor - angle of attack of your plane at level *100 (50 = .5°)
		pitch_angle			= constrain(pitch_angle, -2000, 2000);
		float scale			= sin(radians(pitch_angle * .01));
		altitude_estimate 	+= (float)airspeed * .0002 * scale;
		altitude_estimate 	-= ALT_EST_GAIN * (float)(altitude_estimate - current_loc.alt);
	
		// compute altitude error for throttle control
		altitude_error  = target_altitude - altitude_estimate;
	#else
		altitude_error 	= target_altitude - current_loc.alt;
	#endif
	*/
	altitude_error 	= target_altitude - current_loc.alt;
}


void wrap_bearing()
{
	if (nav_bearing > 36000)	nav_bearing -= 36000;
	if (nav_bearing < 0)		nav_bearing += 36000;
}

/*void update_loiter()
{
	loiter_delta = (target_bearing - old_target_bearing)/100;
	// reset the old value
	old_target_bearing = target_bearing;
	// wrap values
	if (loiter_delta > 170) loiter_delta -= 360;
	if (loiter_delta < -170) loiter_delta += 360;
	loiter_sum += loiter_delta;
}*/

void update_crosstrack(void)
{
	// Crosstrack Error
	// ----------------
	if (abs(target_bearing - crosstrack_bearing) < 4500) {   // If we are too far off or too close we don't do track following
		crosstrack_error = sin(radians((target_bearing - crosstrack_bearing)/100)) * wp_distance;   // Meters we are off track line
		nav_bearing += constrain(crosstrack_error * XTRACK_GAIN,-XTRACK_ENTRY_ANGLE,XTRACK_ENTRY_ANGLE);
	}
}

void reset_crosstrack()
{
	crosstrack_bearing 	= get_bearing(&current_loc, &next_WP);	// Used for track following
}

int get_altitude_above_home(void)
{
	// This is the altitude above the home location
	// The GPS gives us altitude at Sea Level
	// if you slope soar, you should see a negative number sometimes
	// -------------------------------------------------------------
	return current_loc.alt - home.alt;
}

long getDistance(struct Location *loc1, struct Location *loc2)
{
	if(loc1->lat == 0 || loc1->lng == 0) 
		return -1;
	if(loc2->lat == 0 || loc2->lng == 0) 
		return -1;
	float dlat 		= (float)(loc2->lat - loc1->lat);
	float dlong  	= ((float)(loc2->lng - loc1->lng)) * scaleLongDown;
	return sqrt(sq(dlat) + sq(dlong)) * .01113195;
}

long get_alt_distance(struct Location *loc1, struct Location *loc2)
{
	return abs(loc1->alt - loc2->alt);
}

long get_bearing(struct Location *loc1, struct Location *loc2)
{
	long off_x = loc2->lng - loc1->lng;
	long off_y = (loc2->lat - loc1->lat) * scaleLongUp;
	long bearing =  9000 + atan2(-off_y, off_x) * 5729.57795;
	if (bearing < 0) bearing += 36000;
	return bearing;
}
