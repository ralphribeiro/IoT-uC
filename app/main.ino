#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

unsigned short int nivelLog = 0;
/*
0 - nenhum
1 - debug
2 - erros
*/
void escreveLog(String msg, unsigned short int nivLog)
{
    if (nivLog == nivelLog)
        mostraLog(msg);
}

void setup()
{
    Serial.begin(9600);
    analogWriteFreq(10000);
    iniciaES();
}

void loop()
{
    processaES();
}

void iniciaES()
{
    iniciaDisplay();
    iniciaDHT();
    iniciaBMP();
    iniciaWIFI();
    iniciaMQTT();
}

uint16_t tempoDisplay = 100;
uint16_t tempoDht = 1000;
uint16_t tempoBmp = 1000;
uint16_t tempoWifi = 5000;

void processaES()
{
    processaDHT(tempoDht);
    processaBMP(tempoBmp);

    if (nivelLog == 0)
        processaDisplay(tempoDisplay);

    processaWIFI(tempoWifi);
    processaMQTT();
    processaSaida();
}
