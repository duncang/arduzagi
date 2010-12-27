//****************************************************************
// Function that controls aileron/rudder, elevator, rudder (if 4 channel control) and throttle to produce desired attitude and airspeed.
//****************************************************************

void stabilize()
{
	float ch1_inf = 1.0;
	float ch2_inf = 1.0;
	float ch4_inf = 1.0;
	
	if(crash_timer > 0){
		nav_roll = 0;
	}
		
	// Calculate dersired servo output for the roll 
	// servo_out has switched to degrees * 1
	// ---------------------------------------------
	servo_out[CH_ROLL]	= PID((nav_roll - roll_sensor), deltaMiliSeconds, CASE_SERVO_ROLL);
	servo_out[CH_PITCH] = PID((nav_pitch + abs(roll_sensor * PITCH_COMP) - pitch_sensor), deltaMiliSeconds, CASE_SERVO_PITCH);  
	
	// Mix Stick input to allow users to override control surfaces
	// -----------------------------------------------------------
	if ((control_mode < FLY_BY_WIRE_A) || (control_mode > FLY_BY_WIRE_B)) {
	
		ch1_inf = (float)radio_in[CH_ROLL] - (float)radio_trim[CH_ROLL];
		ch1_inf = abs(ch1_inf);
		ch1_inf = min(ch1_inf, 400.0);
		ch1_inf = ((400.0 - ch1_inf) /400.0);
		
		ch2_inf = (float)radio_in[CH_PITCH] - radio_trim[CH_PITCH];
		ch2_inf = abs(ch2_inf);									
		ch2_inf = min(ch2_inf, 400.0);							
		ch2_inf = ((400.0 - ch2_inf) /400.0);
		
		// scale the sensor input based on the stick input
		// -----------------------------------------------
		servo_out[CH_ROLL]		*= ch1_inf;
		servo_out[CH_PITCH]		*= ch2_inf;
	
		// Mix in stick inputs 
		// -------------------
		servo_out[CH_ROLL]	+=	REVERSE_ROLL * 	((radio_in[CH_ROLL] - radio_trim[CH_ROLL]) * 45.0f / 500);
		servo_out[CH_PITCH]	+=	REVERSE_PITCH * ((radio_in[CH_PITCH] - radio_trim[CH_PITCH]) * 45.0f / 500);
	}

	#if CH4_RUDDER == 1
		ch4_inf = (float)radio_in[CH_RUDDER] - (float)radio_trim[CH_RUDDER];
		ch4_inf = abs(ch4_inf);									
		ch4_inf = min(ch4_inf, 400.0);							
		ch4_inf = ((400.0 - ch4_inf) /400.0);
	
		servo_out[CH_RUDDER] = servo_out[CH_ROLL] * ADVERSE_ROLL;
		servo_out[CH_RUDDER] *= ch4_inf;
		servo_out[CH_RUDDER] += ((radio_in[CH_RUDDER] - radio_trim[CH_RUDDER]) * 45 * REVERSE_RUDDER) / 500;
	#endif
	
	// Call slew rate limiter if used
	// ------------------------------
	#if(ROLL_SLEW_LIMIT != 0)
		servo_out[CH_ROLL] = roll_slew_limit(servo_out[CH_ROLL]);
	#endif	
}

void crash_checker()
{
	if(pitch_sensor < -4500){
		crash_timer = 255;
	}
	if(crash_timer > 0)
		crash_timer--;
}


#if AIRSPEED_SENSOR == 0
void calc_throttle()
{
	// no airspeed sensor, we use nav pitch to determin the proper throttle output
	// AUTO, RTL, etc
	// ---------------------------------------------------------------------------
	if (nav_pitch >= 0) {
		servo_out[CH_THROTTLE] = throttle_cruise + (THROTTLE_MAX - throttle_cruise) * nav_pitch / PITCH_MAX;
	} else {
		servo_out[CH_THROTTLE] = throttle_cruise - (throttle_cruise - THROTTLE_MIN) * nav_pitch / PITCH_MIN;
	}
	servo_out[CH_THROTTLE] = max(servo_out[CH_THROTTLE], 0);
	
	// are we going too slow? up the throttle to get some more ground speed
	// move into PID loop in the future
	// lower the contstant value to limit the effect : 50 = default
	int gs_boost = 30 * (1.0 - ((float)ground_speed / (float)airspeed_cruise));
	gs_boost = max(0, gs_boost);
	servo_out[CH_THROTTLE] += gs_boost;
	servo_out[CH_THROTTLE] = constrain(servo_out[CH_THROTTLE], THROTTLE_MIN, THROTTLE_MAX);
}
#endif

#if AIRSPEED_SENSOR == 1
void calc_throttle()
{
	// throttle control with airspeed compensation	 
	// -------------------------------------------
	if(airspeed_error >= 0){
		energy_error = (.00005 * airspeed_error * airspeed_error) + (float)altitude_error * 0.098f;
	}else{
		energy_error = (-.00005 * airspeed_error * airspeed_error) + (float)altitude_error * 0.098f;
	}
	// positive energy errors make the throttle go higher
	servo_out[CH_THROTTLE] = throttle_cruise + PID(energy_error, dTnav, CASE_TE_THROTTLE);
	servo_out[CH_THROTTLE] = max(servo_out[CH_THROTTLE], 0);

	// are we going too slow? up the throttle to get some more ground speed
	// move into PID loop in the future
	// lower the contstant value to limit the effect : 50 = default
	int gs_boost = 30 * (1.0 - ((float)ground_speed / (float)airspeed_cruise));
	gs_boost = max(0,gs_boost);
	servo_out[CH_THROTTLE] += gs_boost;
	servo_out[CH_THROTTLE] = constrain(servo_out[CH_THROTTLE], THROTTLE_MIN, THROTTLE_MAX);
}
#endif



/*****************************************
 * Calculate desired roll angle (in medium freq loop)
 *****************************************/

#if AIRSPEED_SENSOR == 1
void calc_nav_pitch()
{
	// Calculate the Pitch of the plane
	// --------------------------------
	nav_pitch = -PID(airspeed_error, dTnav, CASE_NAV_PITCH_ASP);
	//nav_pitch = PID(altitude_error, dTnav, CASE_NAV_PITCH_ALT);
	nav_pitch = constrain(nav_pitch, PITCH_MIN, PITCH_MAX);
}
#endif

#if AIRSPEED_SENSOR == 0
void calc_nav_pitch()
{
	// Calculate the Pitch of the plane
	// --------------------------------
	nav_pitch = PID(altitude_error, dTnav, CASE_NAV_PITCH_ALT);
	nav_pitch = constrain(nav_pitch, PITCH_MIN, PITCH_MAX);
}
#endif


void calc_nav_roll()
{

	// Adjust gain based on ground speed - We need lower nav gain going in to a headwind, etc.
	// This does not make provisions for wind speed in excess of airframe speed
	nav_gain_scaler = (float)ground_speed / (float)(AIRSPEED_CRUISE * 100);
	nav_gain_scaler = constrain(nav_gain_scaler, 0.2, 1.0);

	// Doug to implement the high speed servo gain scale
	// use head max to limit turns, make a var
	
	//kp[CASE_NAV_ROLL] = NAV_ROLL_P * nav_gain_scaler;	//	 This will have to be redone for adjustable gains!!!!!!
	//ki[CASE_NAV_ROLL] = NAV_ROLL_I * nav_gain_scaler;	
	//kd[CASE_NAV_ROLL] = NAV_ROLL_D * nav_gain_scaler;
	
	// negative error = left turn
	// positive error = right turn
	// Calculate the required roll of the plane
	// ----------------------------------------
	nav_roll = PID(bearing_error, dTnav, CASE_NAV_ROLL);	//returns desired bank angle in degrees*100
	nav_roll = constrain(nav_roll,-HEAD_MAX, HEAD_MAX);
	nav_roll *= nav_gain_scaler;
}


/*****************************************
 * Roll servo slew limit
 *****************************************/
float roll_slew_limit(float servo)
{
	static float last;
	float temp = constrain(servo, last-ROLL_SLEW_LIMIT * deltaMiliSeconds/1000.f, last + ROLL_SLEW_LIMIT * deltaMiliSeconds/1000.f);
	last = servo;
	return temp;
}

/*****************************************
 * Throttle slew limit
 *****************************************/
float throttle_slew_limit(float throttle)
{
	static float last;
	float temp = constrain(throttle, last-THROTTLE_SLEW_LIMIT * deltaMiliSeconds/1000.f, last + THROTTLE_SLEW_LIMIT * deltaMiliSeconds/1000.f);
	last = throttle;
	return temp;
}


/*****************************************
 * Proportional Integrator Derivative Control 
 *****************************************/

float PID(long PID_error, long dt, int PID_case)
{

	// PID_case is used to keep track of which PID controller is being used - e.g.	PID_servo_out[CH_ROLL]
	float output;
 
	float derivative = 1000.0f * (PID_error-last_error[PID_case]) / (float)dt;
	last_error[PID_case] = PID_error;
	output = (kp[PID_case]*PID_error);    	//Compute proportional component
											//Positive error produces positive output

	integrator[PID_case] += (float)PID_error*(float)dt/1000.0f; 
	integrator[PID_case] = constrain(integrator[PID_case],-integrator_max[PID_case],integrator_max[PID_case]);
  
	//integrator[PID_case] = reset_I(integrator[PID_case]);   
  
	output += integrator[PID_case]*ki[PID_case];        	//Add the integral component

	output += derivative*kd[PID_case];                   	//Add the derivative component

	return output; //Returns the result     

}

// Zeros out navigation Integrators if we are changing mode, have passed a waypoint, etc.
// Keeps outdated data out of our calculations
void reset_I(void)
{
		integrator[CASE_NAV_ROLL] = 0;
		integrator[CASE_NAV_PITCH_ASP] = 0;
		integrator[CASE_NAV_PITCH_ALT] = 0;
		integrator[CASE_TE_THROTTLE] = 0;
		integrator[CASE_ALT_THROTTLE] = 0;

		last_error[CASE_NAV_ROLL] = 0;
		last_error[CASE_NAV_PITCH_ASP] = 0;
		last_error[CASE_NAV_PITCH_ALT] = 0;
		last_error[CASE_TE_THROTTLE] = 0;
		last_error[CASE_ALT_THROTTLE] = 0;
}

