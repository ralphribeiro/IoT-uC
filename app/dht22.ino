// #include "DHT.h"
#include "DHTesp.h"

#define DHTPIN D6

DHTesp dht;

void iniciaDHT()
{
    // dht.begin();
    dht.setup(DHTPIN, DHTesp::DHT22);
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

long ultimaLeituraDht = 0;
void processaDHT()
{
    long agora = millis();

    long dhtDelay = dht.getMinimumSamplingPeriod();

    if ((agora - ultimaLeituraDht) > dhtDelay)
    {
        // float h = dht.readHumidity();
        float h = dht.getHumidity();
        // float t = dht.readTemperature();
        float t = dht.getTemperature();

        if (isnan(h) || isnan(t))
        {
            logging("Falha ao ler o sensor DHT", 2);
            return;
        }

        float hic = dht.computeHeatIndex(t, h, false);

        umidade = h;
        temperaturaDHT = t;
        pontoOrvalho = hic;

        pubBroker("sensor=DHT temperatura=" + (String)temperaturaDHT);
        pubBroker("sensor=DHT umidade=" + (String)umidade);

        ultimaLeituraDht = agora;
    }
}