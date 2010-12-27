#if GCS_PROTOCOL == 3
// An Xplane GCS

byte buf_len = 0;
byte out_buffer[32];

void output_Xplane(void)
{
	// output real-time sensor data
	Serial.print("AAA"); 		 						// 		Message preamble
	output_int((int)(servo_out[CH_ROLL]*100));			// 	0	bytes 0,1
	output_int((int)(servo_out[CH_PITCH]*100));			// 	1	bytes 2,3
	output_int((int)(servo_out[CH_THROTTLE]));			// 	2	bytes 4,5
	#if CH4_RUDDER == 1
	output_int((int)(servo_out[CH_RUDDER]*100));		// 	3	bytes 6,7
	#else
	output_int(0);										// 	3	bytes 6,7
	#endif
	output_int((int)wp_distance);						// 	4	bytes 8,9
	output_int((int)bearing_error);						// 	5	bytes 10,11
	output_int((int)altitude_error);					// 	6	bytes 12,13
	output_int((int)energy_error);						// 	7	bytes 14,15
	output_byte(wp_index);								// 	8	bytes 16
	output_byte(control_mode);							// 	9	bytes 17
	
	// print out the buffer and checksum
	// ---------------------------------
	print_buffer();
}

// this is for debugging only!
void output_Xplane_(void)
{
	// output real-time sensor data
	Serial.print("AAA"); 		 						// 		Message preamble
	output_int((int)(servo_out[CH_ROLL]*100));			// 	0	bytes 0,1
	output_int((int)(servo_out[CH_PITCH]*100));			// 	1	bytes 2,3
	output_int((int)(servo_out[CH_THROTTLE]));			// 	2	bytes 4,5
	#if CH4_RUDDER == 1
	output_int((int)(servo_out[CH_RUDDER]*100));		// 	3	bytes 6,7
	#else
	output_int(0);										// 	3	bytes 6,7
	#endif
	output_int((int)wp_distance);						// 	4	bytes 8,9
	output_int((int)bearing_error);						// 	5	bytes 10,11
	output_int((int)altitude_error);					// 	6	bytes 12,13
	output_int((int)energy_error);						// 	7	bytes 14,15
	output_byte(wp_index);								// 	8	bytes 16
	output_byte(control_mode);							// 	9	bytes 17
	
	// print out the buffer and checksum
	// ---------------------------------
	print_buffer();
}

void pipe()
{
	Serial.print("|");
}

void output_int(int value)
{
	//buf_len += 2;
	out_buffer[buf_len++]	= value & 0xff;
	out_buffer[buf_len++]	= (value >> 8) & 0xff;
}
void output_byte(byte value)
{
	out_buffer[buf_len++]	= value;
}

void print_buffer()
{
	byte ck_a = 0;
	byte ck_b = 0;
	for (byte i = 0;i < buf_len; i++){
		Serial.print (out_buffer[i]);
	}
	Serial.print('\r');
	Serial.print('\n');
	buf_len = 0;
}

void acknowledge(byte id, byte check1, byte check2)
{
}

void send_message(byte id) {
	send_message(id,0l);
}

void send_message(byte id, long param) {
	switch(id) {
		case MSG_ATTITUDE:								// ** Attitude message
			print_attitude();
			break;
		case MSG_LOCATION:								// ** Location/GPS message
			print_position();
			break;
		case MSG_HEARTBEAT:								// ** Location/GPS message
			print_control_mode();
			break;
		case MSG_PERF_REPORT:
			printPerfData();
	}	
}
void send_message(byte severity, const char *str)
{

}


// required by Groundstation to plot lateral tracking course 
void print_new_wp_info()
{
}

void print_control_mode(void)
{
	Serial.print("MSG: ");
	switch (control_mode){
		case MANUAL:
			Serial.println("MANUAL");
			break;
		case STABILIZE:
			Serial.println("STABILIZE");
			break;
		case CIRCLE:
			Serial.println("CIRCLE");
			break;
		case FLY_BY_WIRE_A:
			Serial.println("FBW A");
			break;
		case FLY_BY_WIRE_B:
			Serial.println("FBW B");
			break;
		case AUTO:
			Serial.println("AUTO");
			break;
		case RTL:
			Serial.println("RTL");
			break;
		case LOITER:
			Serial.println("LOITER");
			break;
		case TAKEOFF:
			Serial.println("TAKEOFF");
			break;
		case LAND:
			Serial.println("LAND");
			break;
	}
}


void print_current_waypoints()
{
	Serial.print("MSG: ");
	Serial.print("CUR:");
	Serial.print("\t");
	Serial.print(current_loc.lat,DEC);					
	Serial.print(",\t");
	Serial.print(current_loc.lng,DEC);					
	Serial.print(",\t");
	Serial.println(current_loc.alt,DEC);					
	
	Serial.print("MSG: ");
	Serial.print("NWP:");
	Serial.print(wp_index,DEC);
	Serial.print(",\t");
	Serial.print(next_WP.lat,DEC);
	Serial.print(",\t");
	Serial.print(next_WP.lng,DEC);
	Serial.print(",\t");
	Serial.println(next_WP.alt,DEC);
}

void print_position(void)
{
}

void printPerfData(void)
{
}

void print_attitude(void)
{
}
void print_tuning(void) {
}

#endif