#include <ESP8266WiFi.h>
#include <PubSubClient.h>

int port = 1883;
IPAddress host(192, 168, 1, 9);

const char *topicoPub = "sensors";
const char *topicoSub = "comandos";
const char *idHW = "ESP_01";

bool _statusBroker = 0;
bool statusBroker()
{
    return _statusBroker;
}

WiFiClient clientWifi;
PubSubClient clientBroker(clientWifi);

void callback(char *topic, byte *payload, unsigned int length)
{
    String msg = "";
    if (topic == topicoSub)
    {
        for (int i = 0; i < length; i++)
        {
            msg.concat((char)payload[i]);
        }

        // if (msg == "liga")
        //     _topico = "Ligado";
        // if (msg == "desliga")
        //     _topico = "Desligado";
    }
}

long ultimaTentativeReconexao = 0;
unsigned long intervaloReconexao = 0;
unsigned int intervaloReconexaoMax = 5 * 60 * 1000;
unsigned int intervaloReconexaoMin = 2000;

void iniciaBroker()
{
    logging("Iniciando cliente Broker", 1);
    clientBroker.setServer(host, port);
    clientBroker.setCallback(callback);
    ultimaTentativeReconexao = 0;
    intervaloReconexao = intervaloReconexaoMin;
}

boolean reconecta()
{
    if (clientBroker.connect(idHW))
    {
        logging("Conectando ao Broker", 1);
        // clientBroker.publish(topicoPub, "conectado");
        clientBroker.subscribe(topicoSub);
        logging("Conectado", 1);
    }
    return clientBroker.connected();
}

void processaBroker()
{
    if (!_statusBroker)
    {
        long agora = millis();
        if (agora - ultimaTentativeReconexao > intervaloReconexao)
        {
            ultimaTentativeReconexao = reconecta() ? 0 : agora;
            if (intervaloReconexao <= intervaloReconexaoMax / 10)
                intervaloReconexao *= 10;
        }
    }
    else
    {
        intervaloReconexao = intervaloReconexaoMin;
        clientBroker.loop();
    }

    _statusBroker = clientBroker.connected();
}

void pubBroker(String msg)
{
    if (_statusBroker)
    {
        logging("Mensagem publicada", 1);
        String pub_msg = ((String)idHW + "," + msg);
        clientBroker.publish(topicoPub, pub_msg.c_str(), false);
    }
}