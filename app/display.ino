#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET LED_BUILTIN // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void iniciaDisplay()
{
    if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
    {
        escreveLog("Falha na inicialização da tela", 2);
        for (;;)
            ;
    }

    display.clearDisplay();
    display.setCursor(0, 0);
    display.setTextColor(SSD1306_WHITE);
    display.dim(true);
    display.display();

    delay(50);
}

long ultimaLeituraDisplay = 0;

void processaDisplay(int intervaloDisplay)
{
    long agora = millis();
    if ((agora - ultimaLeituraDisplay) > intervaloDisplay)
    {
        mostraTela1();
        display.display();

        ultimaLeituraDisplay = agora;
    }
}

void mostraTela1()
{
    float a = obtemTemperaturaDHT();
    float b = obtemUmidadeDHT();
    float c = obtemTemperaturaBMP();
    float d = obtemPressao();
    bool e = statusWifi();
    int8_t f = statusMqtt();
    uint8_t g = obtemNivelPwm();

    display.clearDisplay();
    display.setTextSize(2);
    display.setCursor(0, 0);
    display.print(a, 1);
    display.drawCircle(display.getCursorX() + 4, display.getCursorY() + 3, 3, WHITE);
    display.setCursor(display.getCursorX() + 9, 0);
    display.setTextSize(2);
    display.println("C");
    display.setTextSize(1);
    display.setCursor(0, 20);
    display.print(c, 1);
    display.drawCircle(display.getCursorX() + 1, display.getCursorY() + 2, 1, WHITE);
    display.setCursor(display.getCursorX() + 4, display.getCursorY());
    display.print("C");
    display.setCursor(0, 30);
    display.print(b, 1);
    display.print("%");
    display.setCursor(0, 40);
    display.print(d/1000, 2);
    display.print("kPa");

    display.setCursor(110, 0);
    display.setTextSize(1);
    display.print(e);
    display.setCursor(110, 10);
    display.print(f);

    display.setCursor(100, 32);
    display.print(g);
    display.print("%");
}

void mostraLog(String s)
{
    display.print(s);
    display.display();
}

// void mostraHora()
// {
//     display.clearDisplay();
//     display.setCursor(0, 0);
//     display.setTextSize(2);
//     display.print(hora);
//     display.print(":");
//     display.print(minuto);
//     display.print(":");
//     display.print(segundo);
//     display.display();
// }