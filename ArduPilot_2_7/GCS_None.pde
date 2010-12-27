#if GCS_PROTOCOL == -1
// A quieter GCS...

void acknowledge(byte id, byte check1, byte check2)
{
}

void send_message(byte id)
{
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

void print_current_waypoints()
{
	Serial.print("CUR:");
	Serial.print("\t");
	Serial.print(current_loc.lat,DEC);					
	Serial.print(",\t");
	Serial.print(current_loc.lng,DEC);					
	Serial.print(",\t");
	Serial.println(current_loc.alt,DEC);					
	
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
#endif