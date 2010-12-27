//Function that will read the radio data, limit servos and trigger a failsafe
// ----------------------------------------------------------------------------
byte failsafeCounter = 0;		// we wait a second to take over the throttle and send the plain circling

unsigned int timer1count	= 0;
unsigned int timer2count	= 0;
unsigned int timer3count	= 0;
unsigned int timer4count    = 0;

unsigned int timer1diff		= 1500 * 2;
unsigned int timer2diff		= 1500 * 2;
unsigned int timer3diff		= 1100 * 2;
unsigned int timer4diff		= 1500 * 2;

byte ch_read = 0;
boolean ch1_read = 0;
boolean ch2_read = 0;
boolean ch3_read = 0;
boolean ch4_read = 0;

void read_radio()
{
	//Filter Radio input
	timer1diff -= 46;
	ch1_temp = timer1diff * .5;


	timer2diff -= 38;
	ch2_temp = timer2diff * .5;

	timer3diff += 10;
	ch3_raw = timer3diff * .5;
	radio_in[CH_THROTTLE] = (float)radio_in[CH_THROTTLE] *.9  + (float)timer3diff *.05;
	throttle_failsafe(timer3diff/2);
	radio_in[CH_THROTTLE] = constrain(radio_in[CH_THROTTLE],radio_min[CH_THROTTLE],radio_max[CH_THROTTLE]);
	
	// Rudder with filter
	//radio_in[CH_RUDDER] = (float)radio_in[CH_RUDDER] *.9  + ((float)timer4diff + 40)*.05;
	
	// Rudder with no filter
	radio_in[CH_RUDDER] = timer4diff * .5;

	#if MIXING_MODE == 0
		radio_in[CH_ROLL] = ch1_temp;
		radio_in[CH_PITCH] = ch2_temp;
	#endif
	
	#if  MIXING_MODE == 1
		radio_in[CH_ROLL] = REVERSE_ELEVONS * (REVERSE_CH2_ELEVON*(ch2_temp-elevon2_trim) - REVERSE_CH1_ELEVON*(ch1_temp-elevon1_trim))/2 + 1500;
		radio_in[CH_PITCH] = (REVERSE_CH2_ELEVON*(ch2_temp-elevon2_trim) + REVERSE_CH1_ELEVON*(ch1_temp-elevon1_trim))/2 + 1500;
	#endif

	#if REVERSE_THROTTLE
		radio_in[CH_THROTTLE] = 3000 - radio_in[CH_THROTTLE];
	#endif
		
	servo_out[CH_THROTTLE] = (float)(radio_in[CH_THROTTLE] - CH3_MIN) / (float)(CH3_MAX - CH3_MIN) *100;
	servo_out[CH_THROTTLE] = constrain(servo_out[CH_THROTTLE], 0, 100);	
}

#if RADIO_TYPE == 0
ISR(PCINT2_vect) {
	int cnt = TCNT1;
	
	if(PIND & B00000100){ 		// ch 1 (pin 2) is high
		if (ch1_read == 0){
			ch1_read = 1;
			timer1count = cnt;
		}
	}else if (ch1_read == 1){	// ch 1 (pin 2) is Low
		ch1_read = 0;
		if (cnt < timer1count)   // Timer1 reset during the read of this pulse
		   timer1diff = (cnt + 40000 - timer1count);    // Timer1 TOP = 40000
		else
		  timer1diff = (cnt - timer1count);
	}
	
	if(PIND & B00001000){ 		// ch 2 (pin 3) is high
		if (ch2_read==0){
			ch2_read = 1;
			timer2count = cnt;
		}
	}else if (ch2_read == 1){	// ch 2 (pin 3) is Low
		ch2_read = 0;
		if (cnt < timer2count)   // Timer1 reset during the read of this pulse
		   timer2diff = (cnt + 40000 - timer2count);    // Timer1 TOP = 40000
		else
		  timer2diff = (cnt - timer2count);
	}
}

ISR(PCINT0_vect)
{
	int cnt = TCNT1;
	
#if THROTTLE_PIN == 11
	if(PINB & 8){	// pin 11
#else
	if(PINB & 32){	// pin 13
#endif
		if (ch3_read==0){
			ch3_read = 1;
			timer3count = cnt;
		}
	}else if (ch3_read ==1){      //ch 3 (pin 5) is low
        ch3_read = 0;	
		if (cnt < timer3count)   // Timer1 reset during the read of this pulse
			timer3diff = (cnt + 40000 - timer3count);    // Timer1 TOP = 40000
		else
			timer3diff = (cnt - timer3count);
	}

#if THROTTLE_PIN == 11
	if(PINB & 32){	// pin 13
#else
	if(PINB & 8){	// pin 11
#endif
		if (ch4_read==0){
			ch4_read = 1;
			timer4count = cnt;
		}
	}else if (ch4_read == 1){    //ch 4 (pin 4) is low
		ch4_read = 0;
		if (cnt < timer4count)  //Timer1 reset during the read of this pulse
			timer4diff = (cnt + 40000 - timer4count);    //Timer1 TOP = 40000
		else
			timer4diff = (cnt - timer4count);
	}
}
#endif



#if RADIO_TYPE == 1
ISR(PCINT2_vect) {
	int cnt = TCNT1;
	
	if ((PIND & B00000100) && (PIND & B00001000)) {  //LE detected
		// start a new read
		ch_read = 0;
		ch1_read = 1;
		timer1count = cnt;
		
	} else if((PIND & B00000100) && (!(PIND & B00001000))){ // Ch1 high,Ch2 Low
		if (cnt < timer1count)	// Timer1 reset during the read of this pulse
			timer2diff = (cnt + 40000 - timer1count);		//Timer1 TOP = 40000
		else
			timer2diff = (cnt - timer1count);
			ch_read = 2;
			
	}else if ((!(PIND & B00000100)) && (PIND & B00001000)){ // Ch1 low,Ch2 high
		if (cnt < timer1count)	// Timer1 reset during theread of this pulse
			timer1diff = (cnt + 40000 - timer1count);		//Timer1 TOP = 40000
		else
		timer1diff = (cnt - timer1count);
		ch_read = 1;
		
	} else if ((!(PIND & B00000100)) && (!(PIND & B00001000))){ // Ch1low, Ch2 low
		if (ch_read == 0x00){
			if (cnt < timer1count){
				timer1diff = timer2diff = (cnt + 40000 -timer1count);
			}else{
				timer1diff = timer2diff = (cnt - timer1count);
			} 
		} else if (ch_read == 0x02) {
			if (cnt < timer1count){
				timer1diff = (cnt + 40000 - timer1count);
			}else{
				timer1diff = (cnt - timer1count);
			} 
		} else {
			if (cnt < timer1count){
				timer2diff = (cnt + 40000 - timer1count);
			}else{
				timer2diff = (cnt - timer1count);
			}
		}
	}
}

ISR(PCINT0_vect) {
	int cnt = TCNT1;
	#if THROTTLE_PIN == 11
	if( ch1_read && (!(PINB & 8)) ){	// pin 11
	#else
	if( ch1_read && (!(PINB & 32)) ){	// pin 13
	#endif
		ch1_read=0;
		if (cnt < timer1count)   // Timer1 reset during the read of this pulse
		   timer3diff = (cnt + 40000 - timer1count);    // Timer1 TOP = 40000
		else
		  timer3diff = (cnt - timer1count);
	}
}
#endif

void throttle_failsafe(int pwm)
{
	#if THROTTLE_FAILSAFE == 1
	//check for failsafe and debounce funky reads
	// ------------------------------------------
	if (pwm < THROTTLE_FS_VALUE){
		// we detect a failsafe from radio 
		// throttle has dropped below the mark
		failsafeCounter++;
		if (failsafeCounter == 9){
			Serial.print("MSG FS ON ");
			Serial.println(pwm,DEC);
		}else if(failsafeCounter == 10) {
			ch3_failsafe = true;
			//set_failsafe(true);
			//failsafeCounter = 10;
		}else if (failsafeCounter > 10){
			failsafeCounter = 11;
		}
		
	}else if(failsafeCounter > 0){
		// we are no longer in failsafe condition
		// but we need to recover quickly		
		failsafeCounter--;
		if (failsafeCounter > 3){
			failsafeCounter = 3;
		}		
		if (failsafeCounter == 1){
			Serial.print("MSG FS OFF ");
			Serial.println(pwm,DEC);
		}else if(failsafeCounter == 0) {
			ch3_failsafe = false;
			//set_failsafe(false);
			//failsafeCounter = -1;
		}else if (failsafeCounter <0){
			failsafeCounter = -1;
		}
	}
	#endif
}

void trim_control_surfaces()
{
	// Store control surface trim values
	// ---------------------------------
	#if  MIXING_MODE == 1
		elevon1_trim = ch1_temp;
		elevon2_trim = ch2_temp;
		//Recompute values here using new values for elevon1_trim and elevon2_trim 
		//We cannot use radio_in[CH_ROLL] and radio_in[CH_PITCH] values from read_radio() because the elevon trim values have changed
		radio_trim[CH_ROLL] = REVERSE_ELEVONS * (REVERSE_CH2_ELEVON*(ch2_temp-elevon2_trim) - REVERSE_CH1_ELEVON*(ch1_temp-elevon1_trim))/2 + 1500;
		radio_trim[CH_PITCH] = (REVERSE_CH2_ELEVON*(ch2_temp-elevon2_trim) + REVERSE_CH1_ELEVON*(ch1_temp-elevon1_trim))/2 + 1500;
	#endif
	#if MIXING_MODE == 0
		radio_trim[CH_ROLL] = radio_in[CH_ROLL];
		radio_trim[CH_PITCH] = radio_in[CH_PITCH];
	#endif
	
	radio_trim[CH_RUDDER] = radio_in[CH_RUDDER];
}

void trim_radio()
{
	// wait until we see the radio
	// ---------------------------
	while(radio_in[CH_ROLL] < 900 && radio_in[CH_PITCH] < 900){
		read_radio();
		delay(20);
	}

	// Warm up radio input filters
	// ---------------------------
	for(int c=0; c < 100; c++){
		delay(20);
		read_radio();
	}
	
	// trim ailerons/rudders/elevator
	// ---------------------------
	trim_control_surfaces();
	
	// constrain out of range values
	// -----------------------------
	radio_trim[CH_ROLL] 	= constrain(radio_trim[CH_ROLL], 950, 2050);
	radio_trim[CH_PITCH] 	= constrain(radio_trim[CH_PITCH], 950, 2050);
}


void init_radio()
{
	// enable in change interrupt on PB5 (digital pin 13)
	PCMSK0 = _BV(PCINT3) | _BV(PCINT5);
		
	// enable pin change interrupt on PD2,PD3 (digital pin 2,3)
	PCMSK2 = _BV(PCINT18) | _BV(PCINT19);
}

#if SET_RADIO_LIMITS == 1
void read_radio_limits()
{
	// set initial servo limits for calibration routine
	// -------------------------------------------------
	radio_min[CH_ROLL] = radio_in[CH_ROLL] - 150;
	radio_max[CH_ROLL] = radio_in[CH_ROLL] + 150;

	radio_min[CH_PITCH] = radio_in[CH_PITCH] - 150;
	radio_max[CH_PITCH] = radio_in[CH_PITCH] + 150;

	// vars for the radio config routine
	// ---------------------------------
	int counter 	= 0;
	long reminder;
    reminder 		= millis() - 10000;

	// Allows user to set stick limits and calibrate the IR
	// ----------------------------------------------------
	while(counter < 50){
  
        if (millis() - reminder >= 10000) {              //Remind user every 10 seconds what is going on
	        Serial.println("Reading radio limits:");
	        Serial.println("");
	        Serial.println("Sticks - up right and low Left");
	        Serial.println("");
	        Serial.println("Hold stick in corner- 2 sec");
	        print_radio();
	        demo_servos();
			reminder = millis();
		}
             
		delay(40);
		read_radio();

		// AutoSet servo limits
		// --------------------
		if (radio_in[CH_ROLL] > 1000 && radio_in[CH_ROLL] < 2000){
			radio_min[CH_ROLL] = min(radio_in[CH_ROLL], radio_min[CH_ROLL]);
			radio_max[CH_ROLL] = max(radio_in[CH_ROLL], radio_max[CH_ROLL]);
		}
		
		if (radio_in[CH_PITCH] > 1000 && radio_in[CH_PITCH]< 2000){
			radio_min[CH_PITCH] = min(radio_in[CH_PITCH], radio_min[CH_PITCH]);
			radio_max[CH_PITCH] = max(radio_in[CH_PITCH], radio_max[CH_PITCH]);
		}
		if(radio_in[CH_PITCH] < (radio_min[CH_PITCH] + 30) || radio_in[CH_PITCH] > (radio_max[CH_PITCH] -30)){
			Serial.print(".");
			counter++;
		}else{
			if (counter > 0)
				counter--;
		}
	}
	
	// contstrain min values
	// ---------------------
	radio_min[CH_ROLL] = constrain(radio_min[CH_ROLL], 800, 2200);
	radio_max[CH_ROLL] = constrain(radio_max[CH_ROLL], 800, 2200);
	radio_min[CH_PITCH] = constrain(radio_min[CH_PITCH], 800, 2200);
	radio_max[CH_PITCH] = constrain(radio_max[CH_PITCH], 800, 2200);
	
	Serial.println(" ");
}
#endif




