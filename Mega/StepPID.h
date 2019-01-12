// data and code for StepPID

#include <Arduino.h>


class StepPID {

	public:
			// the control variables
		int errorRange;
		int dTermRange;
		int outputMax;
		int outputMin;
		
		int iTermAdjMaxStepSlow;
		int pTermMaxStepSlow;
		int dTermMaxStepSlow;
		
		int iTermAdjMaxStepFast;
		int pTermMaxStepFast;
		int dTermMaxStepFast;
		

	// private:	// having these public is easier for debugging
				// but they would not normally need to accessed by the user's program XXXXX
	
			// variables internal to the function
		
		static const byte PIDarraySize = numPIDtableSteps + 2;
		int errorRangeStep[PIDarraySize];
		int dTermRangeStep[PIDarraySize];
		
		int iTermAdjStepSlow[PIDarraySize] = {0, 1};
		int pTermStepSlow[PIDarraySize] = {0, 1};
		int dTermStepSlow[PIDarraySize];
		
		int iTermAdjStepFast[PIDarraySize] = {0, -1};
		int pTermStepFast[PIDarraySize] = {0, -1};
		int dTermStepFast[PIDarraySize];
		
		int error;
		int prevError = 0;

		int totalOutput;
		
		int iTerm;
		int iTermAdj;
		int pTerm;
		int dTerm;


//==============

	public:
		void updatePIDsteps() {
				// note that steps 0 and 1 were set when the arrays were initialized
			for (byte n = 2; n <= numPIDtableSteps + 1; n++) {
				iTermAdjStepSlow[n] =  ((float)iTermAdjMaxStepSlow / numPIDtableSteps) * (n - 1);
				pTermStepSlow[n] =     ((float)pTermMaxStepSlow    / numPIDtableSteps) * (n - 1);
				dTermStepSlow[n] =     ((float)dTermMaxStepSlow    / numPIDtableSteps) * (n - 1);
				iTermAdjStepFast[n] = -((float)iTermAdjMaxStepFast / numPIDtableSteps) * (n - 1);
				pTermStepFast[n] =    -((float)pTermMaxStepFast    / numPIDtableSteps) * (n - 1);
				dTermStepFast[n] =    -((float)dTermMaxStepFast    / numPIDtableSteps) * (n - 1);
			}
			updatePIDerrorRangeSteps();
		}

//=============

	public:
		void updatePIDerrorRangeSteps() {
			for (byte n = 1; n <= numPIDtableSteps + 1; n++) {
				errorRangeStep[n] =      ((float)errorRange      / (numPIDtableSteps + 1)) * n;
				dTermRangeStep[n] =      ((float)dTermRange      / (numPIDtableSteps + 1)) * n;
			}
		}

//==============

	public:
		int compute(int errVal) {

			error = errVal;
					
				// first the iTerm and the pTerm
				
				// select the values
			for (byte n = 0; n <= numPIDtableSteps; n++) {
				if (abs(error) <= errorRangeStep[n]) {
						// use different values for positive or negative errors
					if (error >= 0) {
						iTermAdj = iTermAdjStepSlow[n];
						pTerm = pTermStepSlow[n];
					}
					else {
						iTermAdj = iTermAdjStepFast[n];
						pTerm = pTermStepFast[n];
					}
					break;
				}
			}
				// update the iTerm
			iTerm += iTermAdj;
			if (iTerm > outputMax) {
				iTerm = outputMax;
			}
			if (iTerm < outputMin) {
				iTerm = outputMin;
			}

				// then figure out the dTerm
			int errChange = error - prevError;
			
			if (errChange >= 0) {
				for (byte n = 0; n <= numPIDtableSteps; n++) {
					if (errChange <= dTermRangeStep[n]) {
						dTerm = dTermStepSlow[n];
						break;
					}
				}
			}
			else {
				for (byte n = 0; n <= numPIDtableSteps; n++) {
					if (-errChange <= dTermRangeStep[n]) {
						dTerm = dTermStepFast[n];
						break;
					}
				}
			}
			prevError = error;

				// and finally add them all together
			totalOutput = iTerm + pTerm + dTerm;
			
				// apply the limits
			if (totalOutput > outputMax) {
				totalOutput = outputMax;
			}
			if (totalOutput < outputMin) {
				totalOutput = outputMin;
			}
			
			return totalOutput;
		}

//=================
	public:
		void printKeyVars() {
			Serial.print("  error "); Serial.print(error);
			Serial.print(" iTerm "); Serial.print(iTerm);
			Serial.print(" pTerm "); Serial.print(pTerm);
			Serial.print(" dTerm "); Serial.print(dTerm);
			Serial.print(" total "); Serial.print(totalOutput);
			Serial.println();
		}


//==================
	public:
		void printStepTable() {
			Serial.println("\tStep Table");
			Serial.println("\t\tErrStep\tdTstep\tiAdjS\tiAdjF\tpTermS\tpTermF\tdTermS\tdTermF");
			for (byte n = 0; n <= (numPIDtableSteps + 1); n++) {
				Serial.print('\t'); Serial.print(n);
				Serial.print('\t'); Serial.print(errorRangeStep[n]);
				Serial.print('\t'); Serial.print(dTermRangeStep[n]);
				Serial.print('\t'); Serial.print(iTermAdjStepSlow[n]);
				Serial.print('\t'); Serial.print(iTermAdjStepFast[n]);
				Serial.print('\t'); Serial.print(pTermStepSlow[n]);
				Serial.print('\t'); Serial.print(pTermStepFast[n]);
				Serial.print('\t'); Serial.print(dTermStepSlow[n]);
				Serial.print('\t'); Serial.print(dTermStepFast[n]);
				Serial.println();
			}

		}
};