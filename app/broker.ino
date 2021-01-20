#include <ESP8266WiFi.h>

int port = 5556;
char *host = "192.168.1.7";

bool _statusBroker = 0;
bool statusBroker()
{
    return _statusBroker;
}

WiFiClient clientBroker;
void iniciaBroker()
{
    _statusBroker = clientBroker.connect(host, port);
}

void processaBroker()
{
    _statusBroker = (clientBroker.available()) ? 1 : 0;

    if (_statusBroker)
        clientBroker.write("Ralph");
}