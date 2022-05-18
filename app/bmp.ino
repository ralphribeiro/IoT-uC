#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP085.h>

Adafruit_BMP085 bmp;

void iniciaBMP()
{
    if (!bmp.begin(BMP085_HIGHRES))
    {
        logging("Nao foi possivel conectar ao BMP085!\n", 2);
        while (1)
            ;
    }
}

float temperaturaBMP = 0;
float obtemTemperaturaBMP()
{
    return temperaturaBMP;
}

float altitude = 0;
float obtemAltitude()
{
    return altitude;
}

float pressao = 0;
float obtemPressao()
{
    return pressao;
}

long ultimaLeituraBMP = 0;
void processaBMP(int intervaloBMP)
{
    long agora = millis();

    if ((agora - ultimaLeituraBMP) > intervaloBMP)
    {
        temperaturaBMP = bmp.readTemperature();
        altitude = bmp.readAltitude();
        pressao = bmp.readPressure();

        pubBroker("sensor=BMP temperatura=" + (String)temperaturaBMP);
        pubBroker("sensor=BMP pressao=" + (String)pressao);

        ultimaLeituraBMP = agora;
    }
}