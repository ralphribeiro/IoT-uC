#include "DHT.h"

#define DHTPIN D6
#define DHTTYPE DHT22

DHT dht(DHTPIN, DHTTYPE);

void iniciaDHT()
{
    dht.begin();
}

float temperaturaDHT = 0;
float obtemTemperaturaDHT()
{
    return temperaturaDHT;
}

float umidade = 0;
float obtemUmidadeDHT()
{
    return umidade;
}

float pontoOrvalho = 0;
float obtemPontoDeOrvalhoDHT()
{
    return pontoOrvalho;
}

long ultimaLeitura = 0;
void processaDHT(int intervaloDht)
{
    long agora = millis();

    if ((agora - ultimaLeitura) > intervaloDht)
    {
        float h = dht.readHumidity();
        float t = dht.readTemperature();

        if (isnan(h) || isnan(t))
        {
            escreveLog("Falha ao ler o sensor DHT", 2);
            return;
        }

        float hic = dht.computeHeatIndex(t, h, false);

        umidade = h;
        temperaturaDHT = t;
        pontoOrvalho = hic;

        publicaMQTT("DHT22", (String)t);

        ultimaLeitura = agora;
    }
}