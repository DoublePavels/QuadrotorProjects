#include <CompFilter.h>

// X direction
double GetPitchAngle(double pitchAngle, double Gx_Cur,double Gx_Prev,uint64_t x_Cur,uint64_t x_Prev,s16 AccX,s16 AccY,s16 AccZ){
	double h = 0.0;
	double ACCEL_XANGLE = 0.0;
	double dIx = 0.0;

	h   = (x_Cur - x_Prev) / 100.0; 		// get h integration step in sec
	dIx = h * (Gx_Prev + Gx_Cur) / 2.0; 	// get integral dx

	ACCEL_XANGLE = 57.295 * atan((float)AccY / sqrt(pow((float)AccZ, 2) + pow((float)AccX,2))); 	//get Accel angle by X

	pitchAngle = (1 - K) * (pitchAngle + dIx) + K * ACCEL_XANGLE; //Complementary filtration

	return pitchAngle;
}

// Y direction
double GetYawAngle(double yawAngle, double Gy_Cur,double Gy_Prev,uint64_t y_Cur,uint64_t y_Prev,s16 AccX,s16 AccY,s16 AccZ){
	double h = 0.0;
	double ACCEL_YANGLE = 0.0;
	double dIy = 0.0;

	h = (y_Cur-y_Prev)/100.0; // get h integration step in sec
	dIy = h * (Gy_Prev + Gy_Cur) / 2.0; // get integral dx

	ACCEL_YANGLE = 57.295*atan((float)-AccX/ sqrt(pow((float)AccZ,2)+pow((float)AccY,2))); //get Accel angle by Y

	yawAngle = (1-K)*(yawAngle+dIy)+K*ACCEL_YANGLE; //Complementary filtration

	return yawAngle;
}
