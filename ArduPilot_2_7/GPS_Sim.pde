#if GPS_PROTOCOL == 5
// The input buffer
#define BUF_LEN 30
char gps_buffer[BUF_LEN];
float dlat,dlng;

void init_gps(void)
{
	setCommandMux();
	GPS_fix 	= BAD_GPS;
	init_test_location();
}



void init_test_location(void)
{
	//Serial.println("init_test_location");
	GPS_fix 		= VALID_GPS;	
	GPS_new_data 	= true;
	
	current_loc = get_wp_with_index(1);
	current_loc.lat -= 9500;
	current_loc.lng -= 9500;

	//home = current_loc;
	//set_wp_with_index(home,0);
	print_current_waypoints();
}

void read_sim_sensors()
{
	roll_sensor = ((float)roll_sensor *.2f) + ((float)nav_roll *.8f);
	pitch_sensor = ((float)pitch_sensor *.2f) + ((float)nav_pitch *.8f);
	
	servo_out[CH_THROTTLE] = throttle_cruise;
	
	airspeed = throttle_cruise + ((float)airspeed *.9f) + ((float)servo_out[CH_THROTTLE] *.1f);
	airspeed = constrain(airspeed, 0, THROTTLE_MAX);
}


void decode_gps(void)
{
	static unsigned long GPS_timer = 0;
	
	// for testing only
	GPS_fix = VALID_GPS;
	
	//testing 1hz simulation
	if((millis() - GPS_timer) > 200) {
		GPS_timer = millis(); //Restarting timer...
		//readCommands();
		
		// ground speed from GPS
		// --------------------
		ground_speed = ((float)(radio_in[CH_THROTTLE] - 1000) * 2000) / 1000;
		ground_speed = constrain(ground_speed, 0, 1800);

		ground_speed = 1800;

		// groundcourse from GPS
		// ---------------------
		ground_course += (roll_sensor * (long)TURNRATE * (long)dTnav) / 90000L;
						// 4500 * 130 * 1000 / 90000 = 6500

		// wrap ground_course values
		// -------------------------
		if (ground_course > 36000)	ground_course -= 36000;
		if (ground_course < 0) 		ground_course += 36000;		


		// guess the climb rate
		// --------------------
		if(pitch_sensor >= 0){
			climb_rate = (pitch_sensor * CLIMBRATE_UP * (long)dTnav) / 90000L;
		}else{
			climb_rate = (pitch_sensor * CLIMBRATE_DOWN * (long)dTnav) / 90000L;
		}
		current_loc.alt 	+= climb_rate;
	
		// Estimate the location of the aircraft
		// -------------------------------------
		float pb_Rad 		= (float)ground_course * .0001745;
		float dist 			= ((float)(ground_speed * dTnav)) / 1000;
		current_loc.lat 	+= cos(pb_Rad) * dist;						// Latitude = Y part
		current_loc.lng 	+= (sin(pb_Rad) * dist) * scaleLongUp;		// Longitude = X part (scaled)


		if(GPS_fix == VALID_GPS){
			GPS_timer = DIYmillis(); //Restarting timer...
			GPS_new_data			= true;
		}
		
		if((millis() - GPS_timer) > 2000){
			if(GPS_fix != FAILED_GPS){
				GPS_fix = BAD_GPS;
			}
			
			if((millis() - GPS_timer) > 10000){
				GPS_fix = FAILED_GPS;
				GPS_timer = millis();
				Serial.println("no GPS, last 10s");
			}
		}
	}
}



void readCommands(void)
{
	static byte bufferPointer = 0;
	static byte header[2];
	const byte read_GS_header[] 	= {0x21, 0x21}; //!! Used to verify the payload msg header

	if(Serial.available()){
	
		Serial.println("Serial.available");
		bufferPointer = 0;
		
		byte tmp = Serial.read();
		header[0] = tmp;
		Serial.println(tmp);
		
		tmp = Serial.read();
		header[1] = tmp;
		Serial.println(tmp);
		Serial.println("Serial.available");
		
		//header[0] = Serial.read();
		//header[1] = Serial.read();
		
		byte test = 0;

		if(header[0] == read_GS_header[0]) test++;
		if(header[1] == read_GS_header[1]) test++;
		
		if(test == 2){
			// Block until we read full command 
			// --------------------------------
			delay(20);
			byte incoming_val = 0;

			// Ground Station communication 
			// ----------------------------
			while(Serial.available() > 0) 
			{
				incoming_val = Serial.read();		 

				if (incoming_val != 13 && incoming_val != 10 ) {	 
					gps_buffer[bufferPointer++] = incoming_val;	
				}

				if(bufferPointer >= BUF_LEN){
					Serial.println("Big buffer overrun");
					bufferPointer = 0;
					gps_buffer[0] = 1;
					Serial.flush();
					memset(gps_buffer,0,sizeof(gps_buffer));
					return;
				}
			}
			parseCommand(gps_buffer);
			
			// clear buffer of old data
			// ------------------------
			memset(gps_buffer,0,sizeof(gps_buffer));

		}else{
			Serial.flush();
		}
	}
}

// Commands can be sent as !!a:100|b:200|c:1
// -----------------------------------------
void parseCommand(char *buffer)
{
	Serial.println("got cmd ");
	char *token, *saveptr1, *saveptr2;
	
	for (int j = 1;; j++, buffer = NULL) {
		token = strtok_r(buffer, "|", &saveptr1);
		if (token == NULL) break;	
		
		char * cmd 		= strtok_r(token, ":", &saveptr2);
		long value		= strtol(strtok_r (NULL,":", &saveptr2), NULL,0);
		
		///*
		Serial.print("cmd ");
		Serial.print(cmd[0]);
		Serial.print("\tval ");
		Serial.println(value);
		Serial.println("");
		//*/
		///*
		switch(cmd[0]){
		
			case 'h':
			init_test_location();
			//Serial.println("init_test_location");
			break;

			case 'd':
			GPS_fix = BAD_GPS;
			Serial.println("disable-GPS");
			break;
			
			case 'g':
			GPS_fix = VALID_GPS;
			Serial.println("enable-GPS");
			break;
			
			case 'w':
			while (value >0){
				demo_servos();
				delay(200);
				value--;
			}
			break;
	
			case 'r': // return Home
			return_to_launch();
			break;

			case 'l': // return Home
			set_mode(LOITER);
			break;

			case 's':
			reached_waypoint();
			load_waypoint();
			break;
			
			case 'z'://reset
			wp_index = 1;
			load_waypoint();
			break;
			

		}
		//*/
	}
}
#endif

