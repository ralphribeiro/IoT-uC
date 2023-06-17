void iniciaBT()
{
    // Serial.begin(9600);
}

void enviaBT(unsigned short int value)
{
    // switch (value)
    // {
    // case 1:
    //     // serialBT.print(obtemVoltagem());
    //     serialBT.println(" volts");
    //     break;

    // case 2:
    //     // serialBT.print(obtemCorrente(), 2);
    //     serialBT.println(" amperes");
    //     break;

    // case 3:
    //     // serialBT.print(obtemTemperatura());
    //     serialBT.println("C");
    //     break;

    // case 4:
    //     serialBT.println("max");
    //     break;

    // case 5:
    //     serialBT.println("min");
    //     break;

    // case 6:
    //     serialBT.print("vent :");
    //     // serialBT.print(ObtemCargaVentoinha());
    //     serialBT.println(" %");
    //     break;
    // case 7:
    //     // serialBT.print(ObtemIntervalo());
    //     serialBT.println(" step");
    //     break;

    // case 8:
    //     serialBT.print("modo estrobo: ");
    //     // if (ModoStrobo())
    //     //     serialBT.println("on");
    //     // else
    //     //     serialBT.println("off");

    //     break;

    // case 9:
    //     serialBT.println("off");
    //     break;

    // case 10:
    //     serialBT.print("intervalo: ");
    //     // serialBT.print(ObtemIntervaloStrobo());
    //     serialBT.println(" ms");
    //     break;

    // case 11:
    //     // serialBT.print("fan: ");
    //     // serialBT.print(map(Output, pwmFanMin, pwmFanMax, 5, 100));
    //     // serialBT.println("%");
    //     break;

    // default:
    //     break;
    // }
}


void processaBT()
{
    if (Serial.available())
    {
        char received = Serial.read();

        switch (received)
        {
        case 's':
            break;

        case 'd':
            break;
        default:
            break;
        }
    }
}
