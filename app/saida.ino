#include <Led.h>
#define PWMPIN D5

uint16_t nivelMin = 100;
uint16_t nivelMax = 800;

uint8_t intervaloEventoFade = 30;
uint8_t degrauFade = 20;

Led led = Led(PWMPIN, 1, nivelMin, nivelMax);

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
    nivelPwm = map(led.obtemNivel(), 0, nivelMax, 0, 100);
}
