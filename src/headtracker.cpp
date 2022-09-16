#include "headtracker.h"
#include <Arduino.h>
#include <DFRobot_QMC5883.h>

/////////// FUNCTION DEFS ///////////

extern void sendMSPViaEspnow(mspPacket_t *packet);

/////////////////////////////////////


DFRobot_QMC5883 compass(&Wire, /*I2C addr*/QMC5883_ADDRESS);



void
Headtracker::Init()
{
    delay(0);
    Wire.begin(17,16);

    while (!compass.begin())
    {
        Serial.println("Could not find a valid compass 5883 sensor, check wiring!");
        delay(500);
    }

}

void
Headtracker::Loop(uint32_t now)
{
    if((now-m_last)>500){


        float declinationAngle = (4.0 + (26.0 / 60.0)) / (180 / PI);
        compass.setDeclinationAngle(declinationAngle);
        sVector_t mag = compass.readRaw();
        compass.getHeadingDegrees();
        // Serial.print("Degress = ");
        // Serial.println(mag.HeadingDegress);


        mspPacket_t packet;
        packet.reset();
        packet.makeCommand();
        packet.function = MSP_ELRS_BACKPACK_HEADTRACKER;
        // static int n=0;
        int angle = mag.HeadingDegress*10;
        if(angle>2700) angle=angle-3600;
        else if(angle>1800) angle=-900;
        else if(angle>900) angle=900;
        Serial.print("Angle*10 = ");
        Serial.println(angle);
        // mapping from 0-360 to -900 - 900

        
        // packet.addByte((n>>8) %4);  // payload1
        // packet.addByte(n%256);  // payload2 x=p1*256+p2
        // packet.addByte((-n>>8) %4);  // payload3
        // packet.addByte(-n%256);  // payload4 y=p3*256+p4
        packet.addByte(angle>>8);  // payload1
        packet.addByte(angle);  // payload2 x=p1*256+p2
        packet.addByte(0);  // payload3
        packet.addByte(0);  // payload4 y=p3*256+p4
        sendMSPViaEspnow(&packet);

        // n++;

        m_last = now;
    }
}

Headtracker::Headtracker(Stream *port)
{
    m_port = port;
}