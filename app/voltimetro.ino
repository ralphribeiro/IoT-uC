#include <Arduino.h>

uint8_t analogPin = A0;

float r1 = 100300.0;
float r2 = 10140.0;

float tensao = 0;

void iniciaVoltimetro()
{
    pinMode(analogPin, INPUT);
}

long ultimaLeituraVolt = 0;

void processaVoltimetro(int intervalo)
{
    long agora = millis();

    if ((agora - ultimaLeituraVolt) > intervalo)
    {
        float valor = analogRead(analogPin);
        float vsaida = (valor * 3.3) / 1023.0;
        tensao = vsaida / (r2 / (r1 + r2));

        pubBroker("sensor=VBat tensao=" + (String)tensao);

        ultimaLeituraVolt = agora;
    }
}

float obtemNivelBateria()
{
    float max = 4.2 * 3;
    float min = 3.4 * 3;

    if (tensao < min)
        return 0;

    float qtd = tensao - min;

    return qtd * 100 / (max - min);
}
