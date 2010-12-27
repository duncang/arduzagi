// swing the servos around to show them we're alive
// ------------------------------------------------
void demo_servos()
{
	delay(30);
	set_servo_mux(true);
	OCR1A = 1600 * 2;
	OCR1B = 1600 * 2;
	delay(400);
	OCR1A = 1400 * 2;
	OCR1B = 1400 * 2;
	delay(200);
	OCR1A = 1500 * 2;
	OCR1B = 1500 * 2;
	set_servo_mux(false);
	delay(30);
}

void set_servo_mux(boolean mode)
{
	while(TCNT1 < 20000){};
	if (mode){
		//take over the MUX
		pinMode(4, OUTPUT);
		digitalWrite(4, HIGH);
	}else{
		//release the MUX to allow Manual Control
		digitalWrite(4, LOW); 
		pinMode(4, INPUT);
	}
}
// wants +- 45°
void set_servos_4()
{
	#if GPS_PROTOCOL == 3
		if(imu_ok == false && control_mode > MANUAL){	        //  We have lost the IMU - Big trouble
			servo_out[CH_ROLL] = 0;   							//  If we have lost imu we will probably crash.  
			servo_out[CH_PITCH] = 0;							//  Neutralize controls, throttle off
			servo_out[CH_THROTTLE] = 0;
		}   
	#endif

	#if MIXING_MODE == 0
		set_ch1_degrees(servo_out[CH_ROLL]); // 45 ° = right turn (unless reversed)
		set_ch2_degrees(servo_out[CH_PITCH]);
	#endif
	
	  /*Elevon mode*/ // 
	#if MIXING_MODE == 1
		set_ch1_degrees(REVERSE_ELEVONS * (servo_out[CH_PITCH] - servo_out[CH_ROLL]));	
		set_ch2_degrees(servo_out[CH_PITCH] + servo_out[CH_ROLL]);
	#endif

	set_ch4_degrees(servo_out[CH_RUDDER]);
	update_throttle();
}



// requires +- 45°
void set_ch1_degrees(float deg){

#if MIXING_MODE == 0
	radio_out[CH_ROLL] = radio_trim[CH_ROLL] + ((float)REVERSE_ROLL * deg * 11.111f);
#endif

#if MIXING_MODE == 1
	radio_out[CH_ROLL] =  elevon1_trim + ((float)REVERSE_CH1_ELEVON  * deg * 11.111f);    //
#endif
	radio_out[CH_ROLL] = constrain(radio_out[CH_ROLL], 	radio_min[CH_ROLL], 	radio_max[CH_ROLL]);
	radio_out[CH_ROLL] = constrain(radio_out[CH_ROLL], 	1000, 	2000);
	OCR1A = radio_out[CH_ROLL] * 2;	//OCR1A is the channel 1 pulse width in half microseconds
}


void set_ch2_degrees(float deg){

#if MIXING_MODE == 0
	radio_out[CH_PITCH] = radio_trim[CH_PITCH] + ((float)REVERSE_PITCH * deg * 11.111f);
#endif

#if MIXING_MODE == 1
	radio_out[CH_PITCH] =  elevon2_trim + ((float)REVERSE_CH2_ELEVON * deg * 11.111f);
#endif

	radio_out[CH_PITCH] = constrain(radio_out[CH_PITCH], 	radio_min[CH_PITCH], 	radio_max[CH_PITCH]);
	radio_out[CH_PITCH] = constrain(radio_out[CH_PITCH], 	1000, 	2000);
	OCR1B = radio_out[CH_PITCH] * 2;
}

void set_ch4_degrees(float deg){

	//Serial.print("tdeg:");
	//Serial.print(deg,DEC);
	deg = constrain(deg, -45, 45);
	radio_out[CH_RUDDER] = radio_trim[CH_RUDDER] + ((float)REVERSE_RUDDER * deg * 11.111f);
	//Serial.print("\tradio_out: ");
	//Serial.print(radio_out[CH_RUDDER],DEC);
	radio_out[CH_RUDDER] = constrain(radio_out[CH_RUDDER], 	radio_min[CH_RUDDER], 	radio_max[CH_RUDDER]);
	//Serial.print("\tradio_out: ");
	//Serial.print(radio_out[CH_RUDDER],DEC);
	//Serial.print(" : ");
	
	uint16_t timer_out 	= radio_out[CH_RUDDER] % 512; 
	timer_ovf_b 		= radio_out[CH_RUDDER] / 512;
	timer_out >>= 1;

	if(timer_out != OCR2B)
		OCR2B = timer_out;
}

void no_throttle()
{
	//OCR2A = ch3_timer_min;
}

// sets the throttle timer value based on throttle percent
// -------------------------------------------------------
void update_throttle()
{
	#if THROTTLE_OUT == 1
		// convert 0 to 100% into PWM
		servo_out[CH_THROTTLE] = constrain(servo_out[CH_THROTTLE], 0, 100);	
		radio_out[CH_THROTTLE] = (servo_out[CH_THROTTLE] * (radio_max[CH_THROTTLE] - radio_min[CH_THROTTLE])) / 100;
		radio_out[CH_THROTTLE] += radio_min[CH_THROTTLE];
	#else	
		radio_out[CH_THROTTLE] = radio_min[CH_THROTTLE];
	#endif
	
	// Jason's fancy 2µs hack
	uint16_t timer_out 	= radio_out[CH_THROTTLE] % 512; 
	timer_ovf_a 		= radio_out[CH_THROTTLE] / 512;
	timer_out >>= 1;
	
	if(OCR2A != timer_out)
		OCR2A = timer_out;
}

// Throttle Timer Interrupt
// ------------------------
ISR(TIMER1_CAPT_vect) // Timer/Counter1 Capture Event
{
	//This is a timer 1 interrupts, executed every 20us 
	PORTB |= B00000001; //Putting the pin high!
	PORTC |= B00010000; //Putting the pin high!	
	TCNT2 = 0; //restarting the counter of timer 2
	timer_ovf = 0;
}

ISR(TIMER2_OVF_vect)
{
	timer_ovf++;
}

ISR(TIMER2_COMPA_vect) // Timer/Counter2 Compare Match A
{
	if(timer_ovf == timer_ovf_a){
		PORTB &= B11111110; //Putting the pin low
	}
}

ISR(TIMER2_COMPB_vect) // Timer/Counter2 Compare Match B Rudder Servo
{
	if(timer_ovf == timer_ovf_b){
		PORTC &= B11101111; //Putting the pin low!
	}
} 


void init_PWM()
{
	// Servo setup
	// -----------
	
	// Timer 1
	TCCR1A = ((1<<WGM11) | (1<<COM1B1) | (1<<COM1A1)); //Fast PWM: ICR1=TOP, OCR1x=BOTTOM,TOV1=TOP
	TCCR1B = (1<<WGM13) | (1<<WGM12) | (1<<CS11); // Clock scaler = 8, 2,000,000 counts per second
	OCR1A = 3000;	// Rudder  - multiply your value * 2; for example 3000 = 1500 = 45°; 4000 = 2000 = 90°
	OCR1B = 3000; 	// Elevator
	ICR1 = 40000; 	//50hz freq...Datasheet says	(system_freq/prescaler)/target frequency. So (16000000hz/8)/50hz = 40000,


	// enable pin change interrupt 2 - PCINT23..16
	PCICR = _BV(PCIE2);
	
	// enable pin change interrupt 0 -  PCINT7..0
	PCICR |= _BV(PCIE0);

	// Throttle;
	// Setting up the Timer 2 - 8 bit timer
	TCCR2A 	= 0x0; //Normal Mode            
	//TCCR2B 	= _BV(CS22); //prescaler 64, at 16mhz (64/16) = 4, the counter will increment 1 every 4us
	TCCR2B 	= _BV(CS21) |_BV(CS20); //prescaler 32, at 16mhz (32/16) = 2, the counter will increment 1 every 2us
	//OCR2A 	= (CH3_MIN-1000) / 4;
	//OCR2B  	= 125; // center the rudder
	servo_out[CH_THROTTLE] = 0;
	update_throttle();
	set_ch4_degrees(0);
	TIMSK1 |= _BV(ICIE1); 	// Timer/Counter1, Input Capture Interrupt Enable //PB0 - output throttle
	TIMSK2 	= _BV(TOIE1) | _BV(OCIE2A) | _BV(OCIE2B);	// Timer/Counter2 Compare Match A		
}
