/*****************************************************************/
/*********** USER SETUP AREA! Set your options here! *************/
/*****************************************************************/

// OUTPUT OPTIONS
/*****************************************************************/

// Sensor data output interval in milliseconds
// This may not work, if faster than 20ms (=50Hz)
// Code is tuned for 20ms, so better leave it like that
#define DATA_INTERVAL 20 // in milliseconds

// SENSOR CALIBRATION
/*****************************************************************/
// How to calibrate? Read the tutorial at http://dev.qu.tu-berlin.de/projects/sf-razor-9dof-ahrs
// Put MIN/MAX and OFFSET readings for your board here!
// Accelerometer
// "accel x,y,z (min/max) = X_MIN/X_MAX  Y_MIN/Y_MAX  Z_MIN/Z_MAX"
#define ACCEL_X_MIN ((float)-250)
#define ACCEL_X_MAX ((float)250)
#define ACCEL_Y_MIN ((float)-250)
#define ACCEL_Y_MAX ((float)250)
#define ACCEL_Z_MIN ((float)-250)
#define ACCEL_Z_MAX ((float)250)

// Magnetometer (standard calibration mode)
// "magn x,y,z (min/max) = X_MIN/X_MAX  Y_MIN/Y_MAX  Z_MIN/Z_MAX"
#define MAGN_X_MIN ((float)-600)
#define MAGN_X_MAX ((float)600)
#define MAGN_Y_MIN ((float)-600)
#define MAGN_Y_MAX ((float)600)
#define MAGN_Z_MIN ((float)-600)
#define MAGN_Z_MAX ((float)600)

// Magnetometer (extended calibration mode)
// Uncommend to use extended magnetometer calibration (compensates hard & soft iron errors)
//#define CALIBRATION__MAGN_USE_EXTENDED true
//const float magn_ellipsoid_center[3] = {0, 0, 0};
//const float magn_ellipsoid_transform[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

// Gyroscope
// "gyro x,y,z (current/average) = .../OFFSET_X  .../OFFSET_Y  .../OFFSET_Z
#define GYRO_AVERAGE_OFFSET_X ((float)0.0)
#define GYRO_AVERAGE_OFFSET_Y ((float)0.0)
#define GYRO_AVERAGE_OFFSET_Z ((float)0.0)

/*
// Calibration example:

// "accel x,y,z (min/max) = -277.00/264.00  -256.00/278.00  -299.00/235.00"
#define ACCEL_X_MIN ((float) -277)
#define ACCEL_X_MAX ((float) 264)
#define ACCEL_Y_MIN ((float) -256)
#define ACCEL_Y_MAX ((float) 278)
#define ACCEL_Z_MIN ((float) -299)
#define ACCEL_Z_MAX ((float) 235)

// "magn x,y,z (min/max) = -511.00/581.00  -516.00/568.00  -489.00/486.00"
//#define MAGN_X_MIN ((float) -511)
//#define MAGN_X_MAX ((float) 581)
//#define MAGN_Y_MIN ((float) -516)
//#define MAGN_Y_MAX ((float) 568)
//#define MAGN_Z_MIN ((float) -489)
//#define MAGN_Z_MAX ((float) 486)

// Extended magn
#define CALIBRATION__MAGN_USE_EXTENDED true
const float magn_ellipsoid_center[3] = {91.5, -13.5, -48.1};
const float magn_ellipsoid_transform[3][3] = {{0.902, -0.00354, 0.000636}, {-0.00354, 0.9, -0.00599}, {0.000636, -0.00599, 1}};

// Extended magn (with Sennheiser HD 485 headphones)
//#define CALIBRATION__MAGN_USE_EXTENDED true
//const float magn_ellipsoid_center[3] = {72.3360, 23.0954, 53.6261};
//const float magn_ellipsoid_transform[3][3] = {{0.879685, 0.000540833, -0.0106054}, {0.000540833, 0.891086, -0.0130338}, {-0.0106054, -0.0130338, 0.997494}};

//"gyro x,y,z (current/average) = -40.00/-42.05  98.00/96.20  -18.00/-18.36"
#define GYRO_AVERAGE_OFFSET_X ((float) -42.05)
#define GYRO_AVERAGE_OFFSET_Y ((float) 96.20)
#define GYRO_AVERAGE_OFFSET_Z ((float) -18.36)
*/

// DEBUG OPTIONS
/*****************************************************************/
// When set to true, gyro drift correction will not be applied
#define DEBUG__NO_DRIFT_CORRECTION false
// Print elapsed time after each I/O loop
#define DEBUG__PRINT_LOOP_TIME false

/*****************************************************************/
/****************** END OF USER SETUP AREA!  *********************/
/*****************************************************************/

#include <Wire.h>

// Sensor calibration scale and offset values
#define ACCEL_X_OFFSET ((ACCEL_X_MIN + ACCEL_X_MAX) / 2.0f)
#define ACCEL_Y_OFFSET ((ACCEL_Y_MIN + ACCEL_Y_MAX) / 2.0f)
#define ACCEL_Z_OFFSET ((ACCEL_Z_MIN + ACCEL_Z_MAX) / 2.0f)
#define ACCEL_X_SCALE (GRAVITY / (ACCEL_X_MAX - ACCEL_X_OFFSET))
#define ACCEL_Y_SCALE (GRAVITY / (ACCEL_Y_MAX - ACCEL_Y_OFFSET))
#define ACCEL_Z_SCALE (GRAVITY / (ACCEL_Z_MAX - ACCEL_Z_OFFSET))

#define MAGN_X_OFFSET ((MAGN_X_MIN + MAGN_X_MAX) / 2.0f)
#define MAGN_Y_OFFSET ((MAGN_Y_MIN + MAGN_Y_MAX) / 2.0f)
#define MAGN_Z_OFFSET ((MAGN_Z_MIN + MAGN_Z_MAX) / 2.0f)
#define MAGN_X_SCALE (100.0f / (MAGN_X_MAX - MAGN_X_OFFSET))
#define MAGN_Y_SCALE (100.0f / (MAGN_Y_MAX - MAGN_Y_OFFSET))
#define MAGN_Z_SCALE (100.0f / (MAGN_Z_MAX - MAGN_Z_OFFSET))

// Gain for gyroscope (ITG-3200)
#define GYRO_GAIN 0.06957						   // Same gain on all axes
#define GYRO_SCALED_RAD(x) (x * TO_RAD(GYRO_GAIN)) // Calculate the scaled gyro readings in radians per second

// DCM parameters
#define Kp_ROLLPITCH 0.02f
#define Ki_ROLLPITCH 0.00002f
#define Kp_YAW 1.2f
#define Ki_YAW 0.00002f

// Stuff
#define GRAVITY 256.0f				  // "1G reference" used for DCM filter and accelerometer calibration
#define TO_RAD(x) (x * 0.01745329252) // *pi/180
#define TO_DEG(x) (x * 57.2957795131) // *180/pi

// Sensor variables
float accel[3]; // Actually stores the NEGATED acceleration (equals gravity, if board not moving).
float accel_min[3];
float accel_max[3];

float magnetom[3];
float magnetom_min[3];
float magnetom_max[3];
float magnetom_tmp[3];

float gyro[3];
float gyro_average[3];
int gyro_num_samples = 0;

// DCM variables
float MAG_Heading;
float Accel_Vector[3] = {0, 0, 0}; // Store the acceleration in a vector
float Gyro_Vector[3] = {0, 0, 0};  // Store the gyros turn rate in a vector
float Omega_Vector[3] = {0, 0, 0}; // Corrected Gyro_Vector data
float Omega_P[3] = {0, 0, 0};	   // Omega Proportional correction
float Omega_I[3] = {0, 0, 0};	   // Omega Integrator
float Omega[3] = {0, 0, 0};
float errorRollPitch[3] = {0, 0, 0};
float errorYaw[3] = {0, 0, 0};
float DCM_Matrix[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
float Update_Matrix[3][3] = {{0, 1, 2}, {3, 4, 5}, {6, 7, 8}};
float Temporary_Matrix[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

// Euler angles
float yaw;
float pitch;
float roll;

// DCM timing in the main loop
unsigned long timestamp;
unsigned long timestamp_old;
float G_Dt; // Integration time for DCM algorithm

// More output-state variables
int curr_calibration_sensor = 0;
boolean reset_calibration_session_flag = true;
int num_accel_errors = 0;
int num_magn_errors = 0;
int num_gyro_errors = 0;

void read_sensors()
{
	Read_Gyro();
	Read_Accel();
	Read_Magn();
}

// Read every sensor and record a time stamp
// Init DCM with unfiltered orientation
// TODO re-init global vars?
void reset_sensor_fusion()
{
	float temp1[3];
	float temp2[3];
	float xAxis[] = {1.0f, 0.0f, 0.0f};

	read_sensors();
	timestamp = millis();

	// GET PITCH
	// Using y-z-plane-component/x-component of gravity vector
	pitch = -atan2(accel[0], sqrt(accel[1] * accel[1] + accel[2] * accel[2]));

	// GET ROLL
	// Compensate pitch of gravity vector
	Vector_Cross_Product(temp1, accel, xAxis);
	Vector_Cross_Product(temp2, xAxis, temp1);
	// Normally using x-z-plane-component/y-component of compensated gravity vector
	// roll = atan2(temp2[1], sqrt(temp2[0] * temp2[0] + temp2[2] * temp2[2]));
	// Since we compensated for pitch, x-z-plane-component equals z-component:
	roll = atan2(temp2[1], temp2[2]);

	// GET YAW
	Compass_Heading();
	yaw = MAG_Heading;

	// Init rotation matrix
	init_rotation_matrix(DCM_Matrix, yaw, pitch, roll);
}

// Apply calibration to raw sensor readings
void compensate_sensor_errors()
{
	// Compensate accelerometer error
	accel[0] = (accel[0] - ACCEL_X_OFFSET) * ACCEL_X_SCALE;
	accel[1] = (accel[1] - ACCEL_Y_OFFSET) * ACCEL_Y_SCALE;
	accel[2] = (accel[2] - ACCEL_Z_OFFSET) * ACCEL_Z_SCALE;

	// Compensate magnetometer error
#if CALIBRATION__MAGN_USE_EXTENDED == true
	for (int i = 0; i < 3; i++)
		magnetom_tmp[i] = magnetom[i] - magn_ellipsoid_center[i];
	Matrix_Vector_Multiply(magn_ellipsoid_transform, magnetom_tmp, magnetom);
#else
	magnetom[0] = (magnetom[0] - MAGN_X_OFFSET) * MAGN_X_SCALE;
	magnetom[1] = (magnetom[1] - MAGN_Y_OFFSET) * MAGN_Y_SCALE;
	magnetom[2] = (magnetom[2] - MAGN_Z_OFFSET) * MAGN_Z_SCALE;
#endif

	// Compensate gyroscope error
	gyro[0] -= GYRO_AVERAGE_OFFSET_X;
	gyro[1] -= GYRO_AVERAGE_OFFSET_Y;
	gyro[2] -= GYRO_AVERAGE_OFFSET_Z;
}

// Reset calibration session if reset_calibration_session_flag is set
void check_reset_calibration_session()
{
	// Raw sensor values have to be read already, but no error compensation applied
	// Reset this calibration session?
	if (!reset_calibration_session_flag)
		return;

	// Reset acc and mag calibration variables
	for (int i = 0; i < 3; i++)
	{
		accel_min[i] = accel_max[i] = accel[i];
		magnetom_min[i] = magnetom_max[i] = magnetom[i];
	}

	// Reset gyro calibration variables
	gyro_num_samples = 0; // Reset gyro calibration averaging
	gyro_average[0] = gyro_average[1] = gyro_average[2] = 0.0f;

	reset_calibration_session_flag = false;
}

void iniciaRzr()
{
	// Init sensors
	delay(50); // Give sensors enough time to start
	I2C_Init();
	Accel_Init();
	Magn_Init();
	Gyro_Init();

	// Read sensors, init DCM algorithm
	delay(20); // Give sensors enough time to collect data
	reset_sensor_fusion();
}

bool calibrateMode = false;
void setCalibrateMode()
{
	calibrateMode = true;
	reset_calibration_session_flag = true;
}

void setCalibrateNextSensor()
{
	curr_calibration_sensor = (curr_calibration_sensor + 1) % 3;
	reset_calibration_session_flag = true;
}

float yprRoots[3];
float* getAnglesRoots()
{	
	yprRoots[0] = TO_DEG(yaw);
	yprRoots[1] = TO_DEG(pitch);
	yprRoots[2] = TO_DEG(roll);	
	return yprRoots;
}

void processaRzr()
{
	
	// Time to read the sensors again?
	if ((millis() - timestamp) >= DATA_INTERVAL)
	{
		timestamp_old = timestamp;
		timestamp = millis();
		if (timestamp > timestamp_old)
			G_Dt = (float)(timestamp - timestamp_old) / 1000.0f; // Real time of loop run. We use this on the DCM algorithm (gyro integration time)
		else
			G_Dt = 0;

		// Update sensor readings
		read_sensors();

		if (calibrateMode)
		{
			check_reset_calibration_session(); // Check if this session needs a reset
			calibrate_sensors(curr_calibration_sensor);
		}		
		else // Output sensor values
		{
			// Apply sensor calibration
			compensate_sensor_errors();

			// Run DCM algorithm
			Compass_Heading(); // Calculate magnetic heading
			Matrix_update();
			Normalize();
			Drift_correction();
			Euler_angles();
		}

#if DEBUG__PRINT_LOOP_TIME == true
		logging("loop time (ms) = " + (String)(millis() - timestamp), 1);
#endif
	}
#if DEBUG__PRINT_LOOP_TIME == true
	else
		logging("waiting...", 1);
#endif
}

// ===================================================================================

void Compass_Heading()
{
	float mag_x;
	float mag_y;
	float cos_roll;
	float sin_roll;
	float cos_pitch;
	float sin_pitch;

	cos_roll = cos(roll);
	sin_roll = sin(roll);
	cos_pitch = cos(pitch);
	sin_pitch = sin(pitch);

	// Tilt compensated magnetic field X
	mag_x = magnetom[0] * cos_pitch + magnetom[1] * sin_roll * sin_pitch + magnetom[2] * cos_roll * sin_pitch;
	// Tilt compensated magnetic field Y
	mag_y = magnetom[1] * cos_roll - magnetom[2] * sin_roll;
	// Magnetic Heading
	MAG_Heading = atan2(-mag_y, mag_x);
}


// ===================================================================================

// DCM algorithm

/**************************************************/
void Normalize(void)
{
	float error = 0;
	float temporary[3][3];
	float renorm = 0;

	error = -Vector_Dot_Product(&DCM_Matrix[0][0], &DCM_Matrix[1][0]) * .5; //eq.19

	Vector_Scale(&temporary[0][0], &DCM_Matrix[1][0], error); //eq.19
	Vector_Scale(&temporary[1][0], &DCM_Matrix[0][0], error); //eq.19

	Vector_Add(&temporary[0][0], &temporary[0][0], &DCM_Matrix[0][0]); //eq.19
	Vector_Add(&temporary[1][0], &temporary[1][0], &DCM_Matrix[1][0]); //eq.19

	Vector_Cross_Product(&temporary[2][0], &temporary[0][0], &temporary[1][0]); // c= a x b //eq.20

	renorm = .5 * (3 - Vector_Dot_Product(&temporary[0][0], &temporary[0][0])); //eq.21
	Vector_Scale(&DCM_Matrix[0][0], &temporary[0][0], renorm);

	renorm = .5 * (3 - Vector_Dot_Product(&temporary[1][0], &temporary[1][0])); //eq.21
	Vector_Scale(&DCM_Matrix[1][0], &temporary[1][0], renorm);

	renorm = .5 * (3 - Vector_Dot_Product(&temporary[2][0], &temporary[2][0])); //eq.21
	Vector_Scale(&DCM_Matrix[2][0], &temporary[2][0], renorm);
}

/**************************************************/
void Drift_correction(void)
{
	float mag_heading_x;
	float mag_heading_y;
	float errorCourse;
	//Compensation the Roll, Pitch and Yaw drift.
	static float Scaled_Omega_P[3];
	static float Scaled_Omega_I[3];
	float Accel_magnitude;
	float Accel_weight;

	//*****Roll and Pitch***************

	// Calculate the magnitude of the accelerometer vector
	Accel_magnitude = sqrt(Accel_Vector[0] * Accel_Vector[0] + Accel_Vector[1] * Accel_Vector[1] + Accel_Vector[2] * Accel_Vector[2]);
	Accel_magnitude = Accel_magnitude / GRAVITY; // Scale to gravity.
	// Dynamic weighting of accelerometer info (reliability filter)
	// Weight for accelerometer info (<0.5G = 0.0, 1G = 1.0 , >1.5G = 0.0)
	Accel_weight = constrain(1 - 2 * abs(1 - Accel_magnitude), 0, 1); //

	Vector_Cross_Product(&errorRollPitch[0], &Accel_Vector[0], &DCM_Matrix[2][0]); //adjust the ground of reference
	Vector_Scale(&Omega_P[0], &errorRollPitch[0], Kp_ROLLPITCH * Accel_weight);

	Vector_Scale(&Scaled_Omega_I[0], &errorRollPitch[0], Ki_ROLLPITCH * Accel_weight);
	Vector_Add(Omega_I, Omega_I, Scaled_Omega_I);

	//*****YAW***************
	// We make the gyro YAW drift correction based on compass magnetic heading

	mag_heading_x = cos(MAG_Heading);
	mag_heading_y = sin(MAG_Heading);
	errorCourse = (DCM_Matrix[0][0] * mag_heading_y) - (DCM_Matrix[1][0] * mag_heading_x); //Calculating YAW error
	Vector_Scale(errorYaw, &DCM_Matrix[2][0], errorCourse);								   //Applys the yaw correction to the XYZ rotation of the aircraft, depeding the position.

	Vector_Scale(&Scaled_Omega_P[0], &errorYaw[0], Kp_YAW); //.01proportional of YAW.
	Vector_Add(Omega_P, Omega_P, Scaled_Omega_P);			//Adding  Proportional.

	Vector_Scale(&Scaled_Omega_I[0], &errorYaw[0], Ki_YAW); //.00001Integrator
	Vector_Add(Omega_I, Omega_I, Scaled_Omega_I);			//adding integrator to the Omega_I
}

void Matrix_update(void)
{
	Gyro_Vector[0] = GYRO_SCALED_RAD(gyro[0]); //gyro x roll
	Gyro_Vector[1] = GYRO_SCALED_RAD(gyro[1]); //gyro y pitch
	Gyro_Vector[2] = GYRO_SCALED_RAD(gyro[2]); //gyro z yaw

	Accel_Vector[0] = accel[0];
	Accel_Vector[1] = accel[1];
	Accel_Vector[2] = accel[2];

	Vector_Add(&Omega[0], &Gyro_Vector[0], &Omega_I[0]);  //adding proportional term
	Vector_Add(&Omega_Vector[0], &Omega[0], &Omega_P[0]); //adding Integrator term

#if DEBUG__NO_DRIFT_CORRECTION == true // Do not use drift correction
	Update_Matrix[0][0] = 0;
	Update_Matrix[0][1] = -G_Dt * Gyro_Vector[2]; //-z
	Update_Matrix[0][2] = G_Dt * Gyro_Vector[1];  //y
	Update_Matrix[1][0] = G_Dt * Gyro_Vector[2];  //z
	Update_Matrix[1][1] = 0;
	Update_Matrix[1][2] = -G_Dt * Gyro_Vector[0];
	Update_Matrix[2][0] = -G_Dt * Gyro_Vector[1];
	Update_Matrix[2][1] = G_Dt * Gyro_Vector[0];
	Update_Matrix[2][2] = 0;
#else // Use drift correction
	Update_Matrix[0][0] = 0;
	Update_Matrix[0][1] = -G_Dt * Omega_Vector[2]; //-z
	Update_Matrix[0][2] = G_Dt * Omega_Vector[1];  //y
	Update_Matrix[1][0] = G_Dt * Omega_Vector[2];  //z
	Update_Matrix[1][1] = 0;
	Update_Matrix[1][2] = -G_Dt * Omega_Vector[0]; //-x
	Update_Matrix[2][0] = -G_Dt * Omega_Vector[1]; //-y
	Update_Matrix[2][1] = G_Dt * Omega_Vector[0];  //x
	Update_Matrix[2][2] = 0;
#endif

	Matrix_Multiply(DCM_Matrix, Update_Matrix, Temporary_Matrix); //a*b=c

	for (int x = 0; x < 3; x++) //Matrix Addition (update)
	{
		for (int y = 0; y < 3; y++)
		{
			DCM_Matrix[x][y] += Temporary_Matrix[x][y];
		}
	}
}

void Euler_angles(void)
{
	pitch = -asin(DCM_Matrix[2][0]);
	roll = atan2(DCM_Matrix[2][1], DCM_Matrix[2][2]);
	yaw = atan2(DCM_Matrix[1][0], DCM_Matrix[0][0]);
}


// ===================================================================================

// Computes the dot product of two vectors
float Vector_Dot_Product(const float v1[3], const float v2[3])
{
	float result = 0;

	for (int c = 0; c < 3; c++)
	{
		result += v1[c] * v2[c];
	}

	return result;
}

// Computes the cross product of two vectors
// out has to different from v1 and v2 (no in-place)!
void Vector_Cross_Product(float out[3], const float v1[3], const float v2[3])
{
	out[0] = (v1[1] * v2[2]) - (v1[2] * v2[1]);
	out[1] = (v1[2] * v2[0]) - (v1[0] * v2[2]);
	out[2] = (v1[0] * v2[1]) - (v1[1] * v2[0]);
}

// Multiply the vector by a scalar
void Vector_Scale(float out[3], const float v[3], float scale)
{
	for (int c = 0; c < 3; c++)
	{
		out[c] = v[c] * scale;
	}
}

// Adds two vectors
void Vector_Add(float out[3], const float v1[3], const float v2[3])
{
	for (int c = 0; c < 3; c++)
	{
		out[c] = v1[c] + v2[c];
	}
}

// Multiply two 3x3 matrices: out = a * b
// out has to different from a and b (no in-place)!
void Matrix_Multiply(const float a[3][3], const float b[3][3], float out[3][3])
{
	for (int x = 0; x < 3; x++) // rows
	{
		for (int y = 0; y < 3; y++) // columns
		{
			out[x][y] = a[x][0] * b[0][y] + a[x][1] * b[1][y] + a[x][2] * b[2][y];
		}
	}
}

// Multiply 3x3 matrix with vector: out = a * b
// out has to different from b (no in-place)!
void Matrix_Vector_Multiply(const float a[3][3], const float b[3], float out[3])
{
	for (int x = 0; x < 3; x++)
	{
		out[x] = a[x][0] * b[0] + a[x][1] * b[1] + a[x][2] * b[2];
	}
}

// Init rotation matrix using euler angles
void init_rotation_matrix(float m[3][3], float yaw, float pitch, float roll)
{
	float c1 = cos(roll);
	float s1 = sin(roll);
	float c2 = cos(pitch);
	float s2 = sin(pitch);
	float c3 = cos(yaw);
	float s3 = sin(yaw);

	// Euler angles, right-handed, intrinsic, XYZ convention
	// (which means: rotate around body axes Z, Y', X'')
	m[0][0] = c2 * c3;
	m[0][1] = c3 * s1 * s2 - c1 * s3;
	m[0][2] = s1 * s3 + c1 * c3 * s2;

	m[1][0] = c2 * s3;
	m[1][1] = c1 * c3 + s1 * s2 * s3;
	m[1][2] = c1 * s2 * s3 - c3 * s1;

	m[2][0] = -s2;
	m[2][1] = c2 * s1;
	m[2][2] = c1 * c2;
}


void calibrate_sensors(int calibration_sensor)
{
	if (calibration_sensor == 0) // Accelerometer
	{
		// Output MIN/MAX values
		logging("accel (min/max) = ", 1);
		for (int i = 0; i < 3; i++)
		{
			if (accel[i] < accel_min[i])
				accel_min[i] = accel[i];
			if (accel[i] > accel_max[i])
				accel_max[i] = accel[i];
			logging((String)accel_min[i], 1);
			logging("/", 1);
			logging((String)accel_max[i] + "\n", 1);
		}
	}
	else if (calibration_sensor == 1) // Magnetometer
	{
		// Output MIN/MAX values
		logging("magn (min/max) = ", 1);
		for (int i = 0; i < 3; i++)
		{
			if (magnetom[i] < magnetom_min[i])
				magnetom_min[i] = magnetom[i];
			if (magnetom[i] > magnetom_max[i])
				magnetom_max[i] = magnetom[i];
			logging((String)magnetom_min[i], 1);
			logging("/", 1);
			logging((String)magnetom_max[i] + "\n", 1);
		}
	}
	else if (calibration_sensor == 2) // Gyroscope
	{
		// Average gyro values
		for (int i = 0; i < 3; i++)
			gyro_average[i] += gyro[i];
		gyro_num_samples++;

		// Output current and averaged gyroscope values
		logging("gyro (current/average) = ", 1);
		for (int i = 0; i < 3; i++)
		{
			logging((String)gyro[i], 1);
			logging("/", 1);
			logging((String)(gyro_average[i] / (float)gyro_num_samples) + "\n", 1);
		}
		calibrateMode = false;
	}
}


// ===================================================================================
// I2C code to read the sensors

// Sensor I2C addresses
#define ACCEL_ADDRESS ((int16_t)0x53) // 0x53 = 0xA6 / 2
#define MAGN_ADDRESS ((int16_t)0x1E)  // 0x1E = 0x3C / 2
#define GYRO_ADDRESS ((int16_t)0x69)  // 0x68 = 0xD0 / 2

// Arduino backward compatibility macros
#define WIRE_SEND(b) Wire.write((byte)b)
#define WIRE_RECEIVE() Wire.read()

void I2C_Init()
{
	Wire.begin();
}

void Accel_Init()
{
	Wire.beginTransmission(ACCEL_ADDRESS);
	WIRE_SEND(0x2D); // Power register
	WIRE_SEND(0x08); // Measurement mode
	Wire.endTransmission();
	delay(5);
	Wire.beginTransmission(ACCEL_ADDRESS);
	WIRE_SEND(0x31); // Data format register
	WIRE_SEND(0x08); // Set to full resolution
	Wire.endTransmission();
	delay(5);

	// Because our main loop runs at 50Hz we adjust the output data rate to 50Hz (25Hz bandwidth)
	Wire.beginTransmission(ACCEL_ADDRESS);
	WIRE_SEND(0x2C); // Rate
	WIRE_SEND(0x09); // Set to 50Hz, normal operation
	Wire.endTransmission();
	delay(5);
}

// Reads x, y and z accelerometer registers
void Read_Accel()
{
	int i = 0;
	uint8_t buff[6];

	Wire.beginTransmission(ACCEL_ADDRESS);
	WIRE_SEND(0x32); // Send address to read from
	Wire.endTransmission();

	Wire.beginTransmission(ACCEL_ADDRESS);
	Wire.requestFrom(ACCEL_ADDRESS, 6); // Request 6 bytes
	while (Wire.available())			// ((Wire.available())&&(i<6))
	{
		buff[i] = WIRE_RECEIVE(); // Read one byte
		i++;
	}
	Wire.endTransmission();

	if (i == 6) // All bytes received?
	{
		// No multiply by -1 for coordinate system transformation here, because of double negation:
		// We want the gravity vector, which is negated acceleration vector.
		accel[0] = (int16_t)((((uint16_t)buff[3]) << 8) | buff[2]); // X axis (internal sensor y axis)
		accel[1] = (int16_t)((((uint16_t)buff[1]) << 8) | buff[0]); // Y axis (internal sensor x axis)
		accel[2] = (int16_t)((((uint16_t)buff[5]) << 8) | buff[4]); // Z axis (internal sensor z axis)
	}
	else
	{
		num_accel_errors++;
		logging("!ERR: reading accelerometer", 2);
	}
}

void Magn_Init()
{
	Wire.beginTransmission(MAGN_ADDRESS);
	WIRE_SEND(0x02);
	WIRE_SEND(0x00); // Set continuous mode (default 10Hz)
	Wire.endTransmission();
	delay(5);

	Wire.beginTransmission(MAGN_ADDRESS);
	WIRE_SEND(0x00);
	WIRE_SEND(0b00011000); // Set 50Hz
	Wire.endTransmission();
	delay(5);
}

void Read_Magn()
{
	int i = 0;
	uint8_t buff[6];

	Wire.beginTransmission(MAGN_ADDRESS);
	WIRE_SEND(0x03); // Send address to read from
	Wire.endTransmission();

	Wire.beginTransmission(MAGN_ADDRESS);
	Wire.requestFrom(MAGN_ADDRESS, 6); // Request 6 bytes
	while (Wire.available())		   // ((Wire.available())&&(i<6))
	{
		buff[i] = WIRE_RECEIVE(); // Read one byte
		i++;
	}
	Wire.endTransmission();

	if (i == 6) // All bytes received?
	{
		// MSB byte first, then LSB; Y and Z reversed: X, Z, Y
		magnetom[0] = -1 * (int16_t)(((((uint16_t)buff[4]) << 8) | buff[5])); // X axis (internal sensor -y axis)
		magnetom[1] = -1 * (int16_t)(((((uint16_t)buff[0]) << 8) | buff[1])); // Y axis (internal sensor -x axis)
		magnetom[2] = -1 * (int16_t)(((((uint16_t)buff[2]) << 8) | buff[3])); // Z axis (internal sensor -z axis)
																			  // 9DOF Sensor Stick SEN-10183 and SEN-10321 using HMC5843 magnetometer
	}
	else
	{
		num_magn_errors++;
		logging("!ERR: reading magnetometer", 1);
	}
}

void Gyro_Init()
{
	// CTRL_REG1 - 0x20
	// Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0
	// DR1  | DR0  | BW1  | BW0  | PD   | Zen  | Yen  | Xen
	Wire.beginTransmission(GYRO_ADDRESS);
	WIRE_SEND(0x20);
	// WIRE_SEND(0b00001111); // 100Hz
	WIRE_SEND(0b01101111); // 
	Wire.endTransmission();
	delay(5);

	// CTRL_REG2 - 0x21
	// Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0
	// 0    | 0    | HPM1 | HPM0 | HPCF3| HPCF2| HPCF1| HPCF0
	Wire.beginTransmission(GYRO_ADDRESS);
	WIRE_SEND(0x21);
	// WIRE_SEND(0b00000000);
	WIRE_SEND(0b00100000);
	Wire.endTransmission();
	delay(5);

	// CTRL_REG3 - 0x22
	// Bit7    | Bit6    | Bit5      | Bit4  | Bit3    | Bit2   | Bit1   | Bit0
	// I1_Int1 | I1_Boot | H_Lactive | PP_OD | I2_DRDY | I2_WTM |I2_ORun | I2_Empty
	Wire.beginTransmission(GYRO_ADDRESS);
	WIRE_SEND(0x22);
	WIRE_SEND(0b00001000);
	Wire.endTransmission();
	delay(5);

	// CTRL_REG4 - 0x23
	// Bit7 | Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0
	// BDU  | BLE  | FS1  | FS0  | -    | ST1  | ST0  | SIM
	Wire.beginTransmission(GYRO_ADDRESS);
	WIRE_SEND(0x23);
	//WIRE_SEND(0b00000000); // 250 dps
	//WIRE_SEND(0b00010000); // 500 dps
	// WIRE_SEND(0b00110000); // 2000 dps
	WIRE_SEND(0b10110000); // 2000 dps
	Wire.endTransmission();
	delay(5);

	// CTRL_REG5 - 0x24
	// Bit7 | Bit6    | Bit5 | Bit4 | Bit3      | Bit2      | Bit1     | Bit0
	// BOOT | FIFO_EN | --   | HPen | INT1_Sel1 | INT1_Sel0 | Out_Sel1 | Out_Sel0
	Wire.beginTransmission(GYRO_ADDRESS);
	WIRE_SEND(0x24);
	// WIRE_SEND(0b00011100);
	WIRE_SEND(0b00011010);
	Wire.endTransmission();
	delay(5);

	// STATUS_REG - 0x27
	// Bit7  | Bit6 | Bit5 | Bit4 | Bit3  | Bit2 | Bit1 | Bit0
	// ZYXOR | ZOR  | YOR  | XOR  | ZYXDA | ZDA  | YDA  | XDA
	Wire.beginTransmission(GYRO_ADDRESS);
	WIRE_SEND(0x27);
	WIRE_SEND(0b10000000);
	Wire.endTransmission();
	delay(5);
	
	// INT1_CFG - 0x30
	// Bit7   | Bit6 | Bit5 | Bit4 | Bit3 | Bit2 | Bit1 | Bit0
	// AND/OR | LIR  | ZHIE | ZLIE | YHIE | YLIE | XHIE | XLIE
	// Wire.beginTransmission(GYRO_ADDRESS);
	// WIRE_SEND(0x30);
	// WIRE_SEND(0b00000000);
	// Wire.endTransmission();
	// delay(5);

}

// Reads x, y and z gyroscope registers
void Read_Gyro()
{
	uint8_t buff[6];

	Wire.beginTransmission(GYRO_ADDRESS);
	Wire.write(0x28);
	Wire.endTransmission();
	Wire.requestFrom(GYRO_ADDRESS, 1);
	while (!Wire.available())
	{
		// waiting
	}
	buff[0] = Wire.read();

	Wire.beginTransmission(GYRO_ADDRESS);
	Wire.write(0x29);
	Wire.endTransmission();
	Wire.requestFrom(GYRO_ADDRESS, 1);
	while (!Wire.available())
	{
		// waiting
	}
	buff[1] = Wire.read();

	Wire.beginTransmission(GYRO_ADDRESS);
	Wire.write(0x2A);
	Wire.endTransmission();
	Wire.requestFrom(GYRO_ADDRESS, 1);
	while (!Wire.available())
	{
		// waiting
	}
	buff[2] = Wire.read();

	Wire.beginTransmission(GYRO_ADDRESS);
	Wire.write(0x2B);
	Wire.endTransmission();
	Wire.requestFrom(GYRO_ADDRESS, 1);
	while (!Wire.available())
	{
		// waiting
	}
	buff[3] = Wire.read();

	Wire.beginTransmission(GYRO_ADDRESS);
	Wire.write(0x2C);
	Wire.endTransmission();
	Wire.requestFrom(GYRO_ADDRESS, 1);
	while (!Wire.available())
	{
		// waiting
	}
	buff[4] = Wire.read();

	Wire.beginTransmission(GYRO_ADDRESS);
	Wire.write(0x2D);
	Wire.endTransmission();
	Wire.requestFrom(GYRO_ADDRESS, 1);
	while (!Wire.available())
	{
		// waiting
	}
	buff[5] = Wire.read();

	gyro[0] = -1 * (int16_t)(((((uint16_t)buff[1]) << 8) | buff[0])); // X axis (internal sensor -y axis)
	gyro[1] = -1 * (int16_t)(((((uint16_t)buff[3]) << 8) | buff[2])); // Y axis (internal sensor -x axis)
	gyro[2] = -1 * (int16_t)(((((uint16_t)buff[5]) << 8) | buff[4])); // Z axis (internal sensor -z axis)
}

// ===================================================================================
