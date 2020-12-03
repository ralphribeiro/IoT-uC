#include <Led.h>
#define PWMPIN D5

uint8_t nivelMin = 100;
uint8_t nivelMax = 1023;

uint8_t intervaloEventoFade = 30;
uint8_t degrauFade = 5;

Led led = Led(PWMPIN, 1, nivelMin, nivelMax);

void iniciaSaida()
{
    pinMode(PWMPIN, OUTPUT);
}

void manipulaFade()
{
    if (!led.fade())
    {
        if (!led.aceso())
            led.ativaFade(HIGH, degrauFade, intervaloEventoFade);
        else
            led.ativaFade(LOW, degrauFade, intervaloEventoFade);
    }
}

int nivelPwm = 0;
int obtemNivelPwm()
{
    return nivelPwm;
}

void processaSaida()
{
    if (!led.processa())
        manipulaFade();
    // nivelPwm = map(led.obtemNivel(), nivelMin, nivelMax, 0, 100);
    nivelPwm = led.obtemNivel();
}
