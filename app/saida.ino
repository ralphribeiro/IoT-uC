#include <Arduino.h>

#define PWMPIN D5

void setup_pwm()
{
    analogWriteFreq(25000);
    pinMode(PWMPIN, OUTPUT);
    analogWrite(PWMPIN, 0);
}

int pwm_min = 0;
int pwm_max = 1023;
int pwm = 0;
const int pwm_step = 100;
int last_time_process = 0;

void to_down_pwm()
{
    if (pwm < pwm_step)
        pwm = pwm_min;
    else
        pwm -= pwm_step;
}

void to_up_pwm()
{
    if (pwm >= pwm_max - pwm_step)
        pwm = pwm_max;
    else
        pwm += pwm_step;
}

bool is_up_pwm = true;

void pwm_output_process()
{
    int now = millis();
    if (now - last_time_process > 500)
    {
        pwm = 254;
        // if (is_up_pwm)
        // {
        //     to_up_pwm();
        //     if (pwm == pwm_max)
        //         is_up_pwm = false;
        // }
        // else
        // {
        //     to_down_pwm();
        //     if (pwm == pwm_min)
        //         is_up_pwm = true;
        // }
        
        last_time_process = now;
        analogWrite(PWMPIN, pwm);
    }
}
