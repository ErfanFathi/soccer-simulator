#include "sslworld.h"

SSLWorld* ssl;
RobotsFomation* forms;

int main()
{
    ssl = new SSLWorld();
    while (1)
    {
        ssl->receivePacket();
        ssl->sendPacket();
    }
}