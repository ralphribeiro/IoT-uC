#include <PubSubClient.h>
#include <WiFiClient.h>
#include <Wire.h>

const char *mqtt_server = "192.168.1.5";
// const char *mqtt_server = "192.168.1.7";
const char *topicoPub = "sensors";
const char *topicoSub = "sensors";
const char *idHW = "ESP_01";

const uint16_t porta = 1883;

WiFiClient espClient;
PubSubClient MQTT(espClient);

void iniciaMQTT()
{
	MQTT.setServer(mqtt_server, porta);
	MQTT.setCallback(callback);
	gerenciaConexao();
	delay(500);
}

int8_t statusMq = 0;
int8_t statusMqtt()
{
	return statusMq;
}

void processaMQTT()
{
	gerenciaConexao();
	if (statusMq == 1)
		MQTT.loop();
}

long ultimaLeituraMq = 0;
int intervaloMqConn = 3000;
void gerenciaConexao()
{
	if(MQTT.connected())
		return;
	else
	{
		long agora = millis();
		if ((agora - ultimaLeituraMq) > intervaloMqConn || statusMq == 0)
		{
			statusMq = 0;
			escreveLog("Conectando ao Broker MQTT: ", 1);
			escreveLog(mqtt_server, 1);
			escreveLog("\n", 1);

			if (MQTT.connect(idHW))
			{
				escreveLog("Conectado com Sucesso ao Broker\n", 1);
				statusMq = 1;
				MQTT.subscribe(topicoSub);
			}
			else
			{
				escreveLog("Falha ao Conectar, rc=", 2);
				escreveLog(String(MQTT.state()), 2);
				escreveLog("\n", 2);
				escreveLog(" tentando se reconectar em 3 sugundos...\n", 2);
				statusMq = -1;
			}
			ultimaLeituraMq = agora;
		}
	}
}

void callback(char *topico, byte *mensagem, unsigned int tamanho)
{
	// String msg;

	// for (int i = 0; i < tamanho; i++)
	// {
	// 	msg += (char)mensagem[i];
	// }

	// if (String(topico) == topicoSub)
	// {
	// 	escreveLog("topico: ", 1);
	// 	escreveLog(topico, 1);
	// 	escreveLog("mensagem: ", 1);
	// 	escreveLog(msg, 1);
	// 	escreveLog("\n", 1);

	// 	if (msg.equals("on"))
	// 	{
	// 	}
	// 	else if (msg.equals("off"))
	// 	{
	// 	}
	// }
}

void publicaMQTT(String msg)
{
	gerenciaConexao();

	if (statusMq == 1)
	{
		String pub_msg = ((String)idHW + "," + msg);
		MQTT.publish(topicoPub, pub_msg.c_str(), false);		
	}
}
