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

boolean reconecta()
{
    if (clientBroker.connect(idHW))
    {
        escreveLog("Conectando ao Broker", 1);
        // clientBroker.publish(topicoPub, "conectado");
        clientBroker.subscribe(topicoSub);
        escreveLog("Conectado", 1);
    }
    return clientBroker.connected();
}

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

void iniciaBroker()
{
    escreveLog("Iniciando cliente Broker", 1);
    clientBroker.setServer(host, port);
    clientBroker.setCallback(callback);
    ultimaTentativeReconexao = 0;
}

void processaBroker()
{
    if (!_statusBroker)
    {
        long agora = millis();
        if (agora - ultimaTentativeReconexao > 2000)
            ultimaTentativeReconexao = reconecta() ? 0 : agora;
    }
    else
        clientBroker.loop();

    _statusBroker = clientBroker.connected();
}

void pubBroker(String msg)
{
    if (_statusBroker)
    {
        escreveLog("Mensagem publicada", 1);
        String pub_msg = ((String)idHW + "," + msg);
        clientBroker.publish(topicoPub, pub_msg.c_str(), false);
    }
}