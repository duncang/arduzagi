
void read_XY_sensors()
{
	analog0 		= analogRead(0);
	analog1 		= analogRead(1);
	roll_sensor  	= getRoll() + ROLL_TRIM;
	pitch_sensor 	= getPitch() + PITCH_TRIM;

	#if ENABLE_Z_SENSOR == 0
		if (analog0 > 511){
			ir_max = max((abs(511 - analog0) * IR_MAX_FIX), ir_max);
			ir_max = constrain(ir_max, 40, 600);
			if(ir_max > ir_max_save){
				eeprom_busy_wait();
				eeprom_write_word((uint16_t *)	EE_IR_MAX, ir_max);	// ir_max 
				ir_max_save = ir_max;
			}
		}
	#endif
}

void read_z_sensor(void)
{
	//Serial.print("ir_max: ");
	//Serial.println(ir_max,DEC);
	
	//Checks if the roll is less than 10 degrees to read z sensor
	if(abs(roll_sensor) <= 1500){
		analog2 = ((float)analogRead(2) * 0.10) + ((float)analog2 * .90);
		ir_max = abs(511 - analog2) * IR_MAX_FIX;
		ir_max = constrain(ir_max, 40, 600);
	}
}

// in M/S * 100
void read_airspeed(void)
{
	#if GCS_PROTOCOL != 3
	airpressure_raw = ((float)analogRead(AIRSPEED_PIN) * .10) + (airpressure_raw * .90);
	airpressure 	= (int)airpressure_raw - airpressure_offset;
	airpressure 	= max(airpressure, 0);
	airspeed 		= sqrt((float)airpressure / AIRSPEED_RATIO) * 100;
	#endif
	
	airspeed_error = airspeed_cruise - airspeed;
}

void read_battery(void)
{
	filter_batt_voltage = ((float)analogRead(BATTERY_PIN) * .05) + (filter_batt_voltage * .95);
	battery_voltage = BATTERY_VOLTAGE(filter_batt_voltage);
	if(battery_voltage < INPUT_VOLTAGE)
		low_battery_event();
}



// returns the sensor values as degrees of roll
//   0 ----- 511  ---- 1023    IR Sensor
// -90°       0         90°	    degree output * 100
// sensors are limited to +- 60° (6000 when you multply by 100)

long getRoll(void)
{
	#if XY_SENSOR_LOCATION ==1
	return constrain((x_axis() + y_axis()) / 2, -6000, 6000);
	#endif
	
	#if XY_SENSOR_LOCATION ==0
	return constrain((-x_axis() - y_axis()) / 2, -6000, 6000);
	#endif

	#if XY_SENSOR_LOCATION ==3
	return constrain((-x_axis() - y_axis()) / 2, -6000, 6000);
	#endif
	
	#if XY_SENSOR_LOCATION ==2
	return constrain((x_axis() + y_axis()) / 2, -6000, 6000);
	#endif
}

long getPitch(void)
{
  #if XY_SENSOR_LOCATION ==1
  return constrain((-x_axis() + y_axis()) / 2, -6000, 6000);
  #endif
  
  #if XY_SENSOR_LOCATION ==0
  return constrain((x_axis() - y_axis()) / 2, -6000, 6000);
  #endif

  #if XY_SENSOR_LOCATION ==3
  return constrain((-x_axis() + y_axis()) / 2, -6000, 6000);
  #endif
  
  #if XY_SENSOR_LOCATION ==2
  return constrain((x_axis() - y_axis()) / 2, -6000, 6000);
  #endif
}

long x_axis(void)// roll
{
	return ((analog1 - 511l) * 9000l) / ir_max;
	//      611 - 511 
	//         100 * 9000 / 100 = 90°  low = underestimate  = 36 looks like 90 = flat plane or bouncy plane
	//         100 * 9000 / 250 = 36°   				    = 36 looks like 36
	//		   100 * 9000 / 500 = 18°  high = over estimate = 36 looks like 18 = crash plane
}

long y_axis(void)// pitch
{
	return ((analog0 - 511l) * 9000l) / ir_max;
}


void zero_airspeed(void)
{
	airpressure_raw = analogRead(AIRSPEED_PIN);
	for(int c=0; c < 50; c++){
		delay(20);
		airpressure_raw = (airpressure_raw * .90) + ((float)analogRead(AIRSPEED_PIN) * .10);	
	}
	airpressure_offset = airpressure_raw;
	
}



/*

IR sesor looks at the difference of the two readings and gives a value the same = 511
differnce is +=40°
(temp dif / 40) * 511
if the top sees the ground its the dif is positive or negative


Analog 0 = Pitch sensor
Analog 0 = Roll sensor - unmarked
   	
	  					   ^ GROUND
		285			 	 713
					 	 P
			 \			/
			  \		  /
				\	/
				511
				/	\
			  /		 \
			/		  \
		 P
	  300				707
	 			||||
				||||
				||||
			 cable			 
				
				
 */



