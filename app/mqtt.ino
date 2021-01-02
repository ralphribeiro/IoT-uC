// #include <WiFiClient.h>
// #include <PubSubClient.h>

// // Update these with values suitable for your hardware/network.
// IPAddress server(192, 168, 1, 7);

// const char *topicoPub = "sensors";
// const char *topicoSub = "output";
// const char *idHW = "ESP_01";

// const uint16_t port = 1883;

// const uint16_t reconnectDelay = 5000;



// void callback(char *topic, byte *payload, unsigned int length)
// {
// 	// String msg;

// 	// for (int i = 0; i < length; i++)
// 	// {
// 	// 	msg += (char)payload[i];
// 	// }

// 	// if (String(topico) == topicoSub)
// 	// {
// 	// 	escreveLog("topico: ", 1);
// 	// 	escreveLog(topico, 1);
// 	// 	escreveLog("mensagem: ", 1);
// 	// 	escreveLog(msg, 1);
// 	// 	escreveLog("\n", 1);

// 	// 	if (msg.equals("on"))
// 	// 	{
// 	// 	}
// 	// 	else if (msg.equals("off"))
// 	// 	{
// 	// 	}
// 	// }
// }

// WiFiClient wifiClient;
// PubSubClient client(wifiClient);

// long lastReconnectAttempt = 0;

// boolean reconnect()
// {
// 	if (client.connect(idHW))
// 	{
// 		// client.publish(topicoPub, "conectando");
// 		client.subscribe(topicoSub);
// 	}
// 	return client.connected();
// }

// void initMQTT()
// {
// 	client.setServer(server, port);
// 	client.setCallback(callback);

// 	delay(500);
// 	lastReconnectAttempt = 0;
// }

bool statusMqtt()
{
	return 0;
	// return client.connected();
}

// bool processMQTT()
// {
// 	if (!client.connected())
// 	{
// 		long now = millis();
// 		if (now - lastReconnectAttempt > reconnectDelay)
// 		{
// 			lastReconnectAttempt = now;
// 			if (reconnect())
// 				lastReconnectAttempt = 0;
// 		}
// 	}
// 	else
// 	{
// 		client.loop();
// 		return true;
// 	}
// 	return false;
// }

// void pubMQTT(String msg)
// {
// 	if(processMQTT())
// 	{
// 		String pub_msg = ((String)idHW + "," + msg);
// 		client.publish(topicoPub, pub_msg.c_str(), false);
// 	}	
// }
