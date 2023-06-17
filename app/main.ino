#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

unsigned short int nivelLog = 0;
/*
0 - nenhum
1 - debug
2 - erros
*/
void logging(String msg, unsigned short int nivLog)
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
    setup_pwm();
    iniciaDisplay();
    iniciaDHT();
    iniciaBMP();
    initServer();
    iniciaRzr();
}

uint16_t tempoVBat = 5000;
uint16_t tempoDisplay = 50;
uint16_t tempoBmp = 1000;
uint16_t tempoWifi = 5000;

void processaES()
{
    pwm_output_process();
    processaRzr();
    processaDHT();
    processaBMP(tempoBmp);

    loopServer();

    if (nivelLog == 0)
        processaDisplay(tempoDisplay);
}
