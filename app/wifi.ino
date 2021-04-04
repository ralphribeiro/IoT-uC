#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <NTPClient.h>

const char *ssid = "Vivas_Ribeiro"; // SSID
const char *password = "suupeR114"; // Senha
const int UTC = -3*60*60;


WiFiUDP ntpUDP;
NTPClient dataCliente(ntpUDP, "pool.ntp.org", UTC);

bool statusWf = 0;
bool statusWifi()
{
	return statusWf;
}

// String ipLocal()
// {
// 	return String(WiFi.localIP());
// }

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
	
	WiFi.setAutoReconnect(true);

	dataCliente.begin();
	dataCliente.update();
}


long ultimaLeituraWf = 0;
long ultimaLeituraData = 0;
int intervaloData = 60*1000;

void processaWIFI(int intervaloWf)
{
	long agora = millis();
	if ((agora - ultimaLeituraWf) > intervaloWf)
	{
		statusWf = WiFi.isConnected();
		ultimaLeituraWf = agora;
	}

	if ((agora - ultimaLeituraData) > intervaloData)
	{
		dataCliente.update();
		ultimaLeituraData = agora;
	}
}


char diasDaSemana[7][4] = {
	"Dom", "Seg", "Ter", "Qua", "Qui", "Sex", "Sab"
};

char* obtemDia()
{	
	return diasDaSemana[dataCliente.getDay()];
}

String obtemTempo()
{
	return dataCliente.getFormattedTime();
}