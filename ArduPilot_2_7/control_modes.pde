byte switchPosition 	= 0;
byte oldSwitchPosition 	= 0;

void read_control_switch()
{
	byte switchPosition = readSwitch();
	if (oldSwitchPosition != switchPosition){
		
		switch(switchPosition)
		{
			case 1: // First position
			set_mode(POSITION_1);
			break;
	
			case 2: // middle position
			/*
				if (GPS_fix != FAILED_GPS){
					set_mode(RTL);
				}else{
					set_mode(CIRCLE);
				}
			*/
			set_mode(POSITION_2);
			break;
	
			case 3: // last position
			set_mode(POSITION_3);
			break;
		}

		oldSwitchPosition = switchPosition;

		// reset navigation integrators
		// -------------------------
		reset_I();
	}
}

void reset_control_switch()
{
	oldSwitchPosition = 0;
	read_control_switch();
}

byte readSwitch(void){
	if(digitalRead(4) == HIGH){
		if(digitalRead(5) == HIGH){

			// Middle Switch Position
			// ----------------------
			return 2;

		}else{
			// 3rd Switch Position
			// -------------------
			return 3;
		}
	}else{
		// 1st Switch Position
		// ----------------------
		return 1;
	}
}

void initControlSwitch()
{
	oldSwitchPosition = switchPosition = readSwitch();
}
