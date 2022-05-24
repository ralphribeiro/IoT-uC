#include <ArduinoJson.h>
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>

const char *ssid = "Vivas_Ribeiro"; // SSID
const char *password = "suupeR114"; // Senha

ESP8266WebServer server(80);

void handleRoot()
{
    server.send(200, "text/html", "<h1>ONLINE</h1>");
}

void handleNotFound()
{
    String message = "File Not Found\n\n";
    message += "URI: ";
    message += server.uri();
    message += "\nMethod: ";
    message += (server.method() == HTTP_GET) ? "GET" : "POST";
    message += "\nArguments: ";
    message += server.args();
    message += "\n";

    for (uint8_t i = 0; i < server.args(); i++)
    {
        message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
    }

    server.send(404, "text/plain", message);
}

bool statusWf = 0;
bool statusWifi()
{
    return statusWf;
}

void initWifi()
{
    logging("Conectando-se na rede: ", 0);
    logging(String(ssid), 0);
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED)
    {
        logging(".", 1);
        delay(500);
    }
    statusWf = 1;
    logging("WiFi conectado!\n", 0);
}

void initMDN()
{
    if (!MDNS.begin("nodemcu"))
    {
        logging("Erro ao iniciar DNS!", 2);
        while (1)
            ;
    }
    logging("DNS iniciado", 0);
}

long ultimaLeituraWf = 0;
long ultimaLeituraData = 0;

void checkWifi()
{
    long agora = millis();
    if ((agora - ultimaLeituraWf) > 3000)
    {
        statusWf = WiFi.isConnected();
        ultimaLeituraWf = agora;
    }
}

void handleJson()
{
    StaticJsonDocument<96> sensors;
    sensors["temperaturaDHT"] = obtemTemperaturaDHT();
    sensors["temperaturaBMP"] = obtemTemperaturaBMP();
    sensors["pressure"] = obtemPressao();
    sensors["humidity"] = obtemUmidadeDHT(); 
    
    String output;
    serializeJson(sensors, output);

    server.send(200, "application/json", output);
}

void initServer()
{
    initWifi();
    initMDN();

    server.on("/", handleRoot);
    server.on("/api", handleJson);
    server.onNotFound(handleNotFound);
    server.begin();
    logging("Servidor iniciado", 0);
}

void loopServer()
{
    checkWifi();
    server.handleClient();
    MDNS.update();
}
