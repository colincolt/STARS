// python-build-start
// action, upload
// board, arduino:avr:uno
// port, /dev/ttyACM0
// ide, 1.6.3
// python-build-end

// controlling the speed of a DC motor based on the number of microseconds per revolution


//================
	// Link to the PID control code
	// first set the number of steps for the PID tables
#define numPIDtableSteps 3
	// then include the PID code file
#include "StepPID.h"
StepPID myStepPID;

//===============
	// Variables etc for Motor control

unsigned long targetMicros = 18000;
unsigned long stallMicros = 60000; // used when there are no pulses
unsigned long slowRevMicros = 40000; // used to get going
unsigned long setPoint;
char dir = 'F';

		// values to use when the error is out of range
long errorRange = 32000;
long errorOutOfRangeSlowOutput = 255;
long errorOutOfRangeFastOutput = 0;

		// variables for PWM
int PWMval = 0;
byte PWMpinCW = 5;
byte PWMpinCCW = 6;

	// variables used by the ISR
volatile unsigned long isrRevMicros = 0;
volatile unsigned long isrRevCount = 0;
volatile bool newRevMicros = true;

	// other motor control variables
bool pseudoRevMicros = false;
bool stallDetected = false;
byte afterStallCount = 0;

unsigned long revCount;

unsigned long revMicros;
unsigned long prevRevMicros;
unsigned long prevIsrMicros;
unsigned long latestIsrMicros;
long error;

//=================

void setup() {

	Serial.begin(115200);
	pinMode(11, INPUT_PULLUP);
	attachInterrupt(0, revDetectorISR, RISING);

	pinMode(PWMpinCW, OUTPUT);
	pinMode(PWMpinCCW, OUTPUT);
	analogWrite(PWMpinCCW, 0);
	analogWrite(PWMpinCW, 0);

	setupStepPID();

	Serial.println("StepPIDMotor.ino");

}

//===========

void setupStepPID() {
	myStepPID.errorRange = errorRange;
	myStepPID.dTermRange = errorRange / 4;
	myStepPID.outputMax = 255;
	myStepPID.outputMin = 0;

		// adjustments to use when speed is below target
		//  values should be in range 0 - outputRange
        //   for example 32 represents 12.5% if outputRange is 255
	myStepPID.iTermAdjMaxStepSlow = 32; // equivalent to kI
	myStepPID.pTermMaxStepSlow = 64; // equivalent to kP
	myStepPID.dTermMaxStepSlow = 0;  // equivalent to kD

		// and when speed is above target (in this case they are the same)
	myStepPID.iTermAdjMaxStepFast = myStepPID.iTermAdjMaxStepSlow ;
	myStepPID.pTermMaxStepFast = myStepPID.pTermMaxStepSlow;
	myStepPID.dTermMaxStepFast = myStepPID.dTermMaxStepSlow;

	myStepPID.updatePIDsteps();

	myStepPID.errorRangeStep[0] = 100; // treat errors below this as 0

	myStepPID.printStepTable();

}

//===========

void loop() {
	readPot();
	checkForStall();
	updatePWM();
	outputMotorPwr();
	showDebugData(507);
}

//================

void readPot() {
	int potVal = analogRead(A5);
	//~ potVal = 400; // for testing
	unsigned long newTargetMicros = 8000UL + potVal * 20;
	if (abs(newTargetMicros - targetMicros) > 200) {
		targetMicros = newTargetMicros;
	}
}


//================


void checkForStall() {
		// this causes a pseudo pulse time to be used if there is no real one
	if (micros() - latestIsrMicros >= stallMicros) {
		pseudoRevMicros = true;
		stallDetected = true;
		afterStallCount = 0;
		//~ Serial.print("Stalled "); Serial.println();
	}
	else if (stallDetected == true) {
		afterStallCount ++;
		if (afterStallCount > 8) {	// stay at low speed for a few revs
			stallDetected = false;
		}
	}
}


//============

void updatePWM() {

	if(newRevMicros == true or pseudoRevMicros == true) {

			// first get the setpoint
		setPoint = targetMicros;
		if (stallDetected == true) {
			setPoint = slowRevMicros;
		}

			// then update revMicros
		prevIsrMicros = latestIsrMicros;
		if (newRevMicros == true) {
			noInterrupts();
				latestIsrMicros = isrRevMicros;
				revCount = isrRevCount;
				newRevMicros = false;
			interrupts();
		}
		else { // did not get a newRevMicros value
			latestIsrMicros = micros();
			pseudoRevMicros = false;
		}
		revMicros = latestIsrMicros - prevIsrMicros;
			// ignore very short values of revMicros as aberrations
		if (revMicros < (setPoint >> 2)) {
			latestIsrMicros = prevIsrMicros; // restore the value
			return;
		}

			// get ready to call myStepPID.calc()
		int totalPower = 0;
			// calculate the error
		error = revMicros - setPoint;

			// if the error is out of range don't call StepPID
			//		just use the out-of-range values
		if (error > errorRange) {
			totalPower = errorOutOfRangeSlowOutput;
		}
		else if (error < -errorRange) {
			totalPower = errorOutOfRangeFastOutput;
		}
			// if the error is within range call StepPID
		else {
			unsigned long start = micros();
			totalPower = myStepPID.compute((int) error);
			unsigned long end = micros();
			//~ Serial.print("Calc Micros "); Serial.println(end - start); // 16 µsecs for 3 steps
		}
		PWMval = totalPower;
	}
}

//====================

void outputMotorPwr() {

		// finally, apply the PWM value
	if (dir == 'F') {
		analogWrite(PWMpinCCW, 0);
		analogWrite(PWMpinCW, (byte) PWMval);
	}
	else {
		analogWrite(PWMpinCW, 0);
		analogWrite(PWMpinCCW, (byte) PWMval);
	}
}

//=============

void showDebugData(int interval) {

	static unsigned long prevShowMillis;
	if ( millis() - prevShowMillis  >= interval ) {
		prevShowMillis = millis();

		Serial.print("setPoint "); Serial.print(setPoint);
		//~ Serial.print("\trevMic "); Serial.print(revMicros);
		//~ Serial.print("\terr "); Serial.print(error);
		//~ Serial.print("\ttotPWM "); Serial.print(PWMval);
		//~ Serial.println();

		myStepPID.printKeyVars();
		Serial.println();

	}
}

//===========

void revDetectorISR() {
	isrRevMicros = micros();
	isrRevCount ++;
	newRevMicros = true;
}
