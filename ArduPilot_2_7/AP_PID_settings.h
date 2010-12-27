#define XTRACK_GAIN 10 					// Amount to compensate for crosstrack (degrees/100 per meter)
#define XTRACK_ENTRY_ANGLE 3000			// Max angle used to correct for track following	degrees*100

/***************************************/
//ATTITUDE: ROLL GAINS [Start with changes of no more than 25% at a time]
// IMPORTANT!!	Servo Gain values will be 100 times less than equivalent gains for ArduPilot 2.5
#define SERVO_ROLL_P .004				// Primary value to tune - overall proportional term determines how much rudder/aileron you use to turn
#define SERVO_ROLL_I .00				// roll PID integrator gain (value should generally be low)
#define SERVO_ROLL_D .00				// roll PID derivative gain (for advanced users - should be zero for most airframes)
#define SERVO_ROLL_INT_MAX 500	// Maximium integrator value in degrees * 100
#define ROLL_SLEW_LIMIT 0				// Use to limit slew rate of roll servo.	If zero then slew rate is not limited
										// Value is degree per second limit

/***************************************/
//ATTITUDE: PITCH GAINS [Start with changes of no more than 25% at a time]
//IMPORTANT!!	Servo Gain values will be 100 times less than equivalent gains for ArduPilot 2.5
#define SERVO_PITCH_P .005				// Pitch Proportional gain
#define SERVO_PITCH_I .0				// Pitch integrator gain	(value should generally be low)
#define SERVO_PITCH_D .0				// Pitch derivative gain	(for advanced users - should be zero for most airframes)
#define SERVO_PITCH_INT_MAX	500	// Maximum integrator value in degrees * 100
#define PITCH_COMP .20					// Pitch compensation vs. Roll bank angle. 
										// NOTE!!	The implementation of pitch compensation has been changed.
										// The optimal value for your airframe will likely differ between 2.5 and 2.6

/***************************************/
//ATTITUDE: RUDDER GAINS
// IMPORTANT!!	Servo Gain values will be 100 times less than equivalent gains for ArduPilot 2.5
#define SERVO_RUDDER_P .000				// 	Primary value to tune - overall proportional term determines how much rudder you use to coordinate turn
#define SERVO_RUDDER_I .0				// roll PID integrator gain (value should generally be low)
#define SERVO_RUDDER_D 0.0				// roll PID derivative gain (for advanced users - should be zero for most airframes)
#define SERVO_RUDDER_INT_MAX 500		//Maximium integrator value in degrees * 100
#define RUDDER_MIX 0.5

/***************************************/
//NAV: ROLL GAINS	[Start with changes of no more than 25% at a time]
#define NAV_ROLL_P .7					// Primary value to tune - overall proportional term determines how aggressively we bank to change heading
#define NAV_ROLL_I .01					// roll PID integrator gain (value should generally be low)
#define NAV_ROLL_D .02					// roll PID derivative gain (for advanced users - should be zero for most airframes)
#define NAV_ROLL_INT_MAX 500		// Maximium integrator value in degrees * 100


/***************************************/
//NAV: PITCH GAINS [Start with changes of no more than 25% at a time]
#define NAV_PITCH_ASP_P .65					// 	Overall proportional term determines how aggressively we change pitch to maintain airspeed
#define NAV_PITCH_ASP_I .0					// PID integrator gain (value should generally be low)
#define NAV_PITCH_ASP_D 0.0					// PID derivative gain (for advanced users - should be zero for most airframes)
#define NAV_PITCH_ASP_INT_MAX 500			// Maximium integrator value in degrees * 100

#define NAV_PITCH_ALT_P .65					// 	Overall proportional term determines how aggressively we change pitch to maintain airspeed
#define NAV_PITCH_ALT_I .0					// PID integrator gain (value should generally be low)
#define NAV_PITCH_ALT_D 0.0					// PID derivative gain (for advanced users - should be zero for most airframes)
#define NAV_PITCH_ALT_INT_MAX 500			// Maximium integrator value in degrees * 100

/***************************************/
//ENERGY HEIGHT: THROTTLE OUTPUT GAINS [Start with changes of no more than 25% at a time]
#define THROTTLE_TE_P .50					// Proportional
#define THROTTLE_TE_I .0					// Integrator
#define THROTTLE_TE_D .0					// Derivative
#define THROTTLE_TE_INT_MAX 20		// (0-100)  Integrator limit. 
#define THROTTLE_SLEW_LIMIT 0			// Use to limit slew rate of throttle output.	If zero then slew rate is not limited
										// Value is throttle value (0-100) per second limit

#define P_TO_T 2.5							// Pitch to Throttle feedforward gain (used when no airspeed sensor)

/***************************************/
//FLY BY WIRE AIRSPEED: THROTTLE OUTPUT GAINS 
#define THROTTLE_ALT_P .32 			//Proportional
#define THROTTLE_ALT_I .04 			//Integrator
#define THROTTLE_ALT_D 0.0 			//Derivative
#define THROTTLE_ALT_INT_MAX 20 // (0-100)  Integrator limit. 
