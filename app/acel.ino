#include <Adafruit_ADXL345_U.h>


float acelVec[3] = {0.0, 0.0, 0.0};

/* define um ID único */
Adafruit_ADXL345_Unified acel = Adafruit_ADXL345_Unified(12345);

extern unsigned short int nivelLog;

void iniciaAcel(){
#ifndef ESP8266
    while (!Serial); // for Leonardo/Micro/Zero
#endif
	acel.setDataRate(ADXL345_DATARATE_800_HZ);
	
	if (!acel.begin())	{
	    escreveLog("ADXL345 nao detectado ....", 2);
		while (1);
	}

	acel.setRange(ADXL345_RANGE_2_G);

    if(nivelLog != 0){
        detalhesAcel();
        taxaDadosAcel();
        escalaAcel();
    }
}

long ultimaLeituraAcel = 0;
void processaAcel(long intervalo){
	long agora = millis();
	if ((agora - ultimaLeituraAcel) > intervalo)
	{
		sensors_event_t event;
		acel.getEvent(&event);

		/* escreve na serial x, y e z (m/s^2) */  
		acelVec[0] = event.acceleration.x;
		acelVec[1] = event.acceleration.y;
		acelVec[2] = event.acceleration.z;

		ultimaLeituraAcel = agora;
	}
}

float* obtemAcel(){    
	return acelVec;
}

float* obtemOffSetAcel(){
	float offSetVec[3] = {0.0, 0.0, 0.0};
	
	offSetVec[0] = acel.read16(ADXL345_REG_OFSX);
	offSetVec[1] = acel.read16(ADXL345_REG_OFSY);
	offSetVec[2] = acel.read16(ADXL345_REG_OFSZ);

	return offSetVec;
}

void detalhesAcel(void)
{
	sensor_t sensor;
	acel.getSensor(&sensor);
	escreveLog("------------------------------------\n", 1);
	escreveLog("Sensor:       ", 1);
	escreveLog(String(sensor.name), 1);
    escreveLog("\n", 1);
	escreveLog("Driver Ver:   ", 1);
	escreveLog(String(sensor.version), 1);
    escreveLog("\n", 1);
	escreveLog("Unique ID:    ", 1);
	escreveLog(String(sensor.sensor_id), 1);
    escreveLog("\n", 1);
	escreveLog("Max Value:    ", 1);
	escreveLog(String(sensor.max_value), 1);
	escreveLog(" m/s^2", 1);
    escreveLog("\n", 1);
	escreveLog("Min Value:    ", 1);
	escreveLog(String(sensor.min_value), 1);
	escreveLog(" m/s^2", 1);
    escreveLog("\n", 1);
	escreveLog("Resolution:   ", 1);
	escreveLog(String(sensor.resolution), 1);
	escreveLog(" m/s^2", 1);
    escreveLog("\n", 1);
	escreveLog("------------------------------------\n", 1);
	delay(500);
}

void taxaDadosAcel(void){
	escreveLog("Data Rate:    ", 1);

	switch (acel.getDataRate())
	{
	case ADXL345_DATARATE_3200_HZ:
		escreveLog("3200 ", 1);
		break;
	case ADXL345_DATARATE_1600_HZ:
		escreveLog("1600 ", 1);
		break;
	case ADXL345_DATARATE_800_HZ:
		escreveLog("800 ", 1);
		break;
	case ADXL345_DATARATE_400_HZ:
		escreveLog("400 ", 1);
		break;
	case ADXL345_DATARATE_200_HZ:
		escreveLog("200 ", 1);
		break;
	case ADXL345_DATARATE_100_HZ:
		escreveLog("100 ", 1);
		break;
	case ADXL345_DATARATE_50_HZ:
		escreveLog("50 ", 1);
		break;
	case ADXL345_DATARATE_25_HZ:
		escreveLog("25 ", 1);
		break;
	case ADXL345_DATARATE_12_5_HZ:
		escreveLog("12.5 ", 1);
		break;
	case ADXL345_DATARATE_6_25HZ:
		escreveLog("6.25 ", 1);
		break;
	case ADXL345_DATARATE_3_13_HZ:
		escreveLog("3.13 ", 1);
		break;
	case ADXL345_DATARATE_1_56_HZ:
		escreveLog("1.56 ", 1);
		break;
	case ADXL345_DATARATE_0_78_HZ:
		escreveLog("0.78 ", 1);
		break;
	case ADXL345_DATARATE_0_39_HZ:
		escreveLog("0.39 ", 1);
		break;
	case ADXL345_DATARATE_0_20_HZ:
		escreveLog("0.20 ", 1);
		break;
	case ADXL345_DATARATE_0_10_HZ:
		escreveLog("0.10 ", 1);
		break;
	default:
		escreveLog("???? ", 1);
		break;
	}
	escreveLog(" Hz\n", 1);
}

void escalaAcel(void){
	escreveLog("Range:         +/- ", 1);

	switch (acel.getRange())
	{
	case ADXL345_RANGE_16_G:
		escreveLog("16 ", 1);
		break;
	case ADXL345_RANGE_8_G:
		escreveLog("8 ", 1);
		break;
	case ADXL345_RANGE_4_G:
		escreveLog("4 ", 1);
		break;
	case ADXL345_RANGE_2_G:
		escreveLog("2 ", 1);
		break;
	default:
		escreveLog("?? ", 1);
		break;
	}
	escreveLog(" g\n", 1);
}