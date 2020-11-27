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
uint16_t tempoBmp = 5000;
uint16_t tempoWifi = 5000;
uint16_t tempoMqtt = 10;

void processaES()
{
    processaDHT(tempoDht);
    processaBMP(tempoBmp);

    if (nivelLog == 0)
        processaDisplay(tempoDisplay);

    processaWIFI(tempoWifi);
    processaMQTT(tempoMqtt);
}
