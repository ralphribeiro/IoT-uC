#include <ESP8266WiFi.h>

const char *ssid = "Vivas_Ribeiro"; // SSID
const char *password = "suupeR114"; // Senha

bool statusWf = 0;
bool statusWifi()
{
	return statusWf;
}

void iniciaWIFI()
{
	WiFi.begin(ssid, password);
	escreveLog("Conectando-se na rede: ", 0);
	escreveLog(String(ssid), 0);
	escreveLog("\n", 0);

	while (WiFi.status() != WL_CONNECTED)
	{
		escreveLog(".", 1);
		delay(500);
	}
	statusWf = 1;
	escreveLog("WiFi conectado!\n", 0);
}

long ultimaLeituraWf = 0;
void processaWIFI(int intervaloWf)
{
	long agora = millis();
	if ((agora - ultimaLeituraWf) > intervaloWf)
	{
		statusWf = WiFi.isConnected();
		ultimaLeituraWf = agora;
	}
}
