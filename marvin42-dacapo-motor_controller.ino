//#define DEBUG

#ifndef DEBUG
#include "ArduinoMotorShieldR3.h"
#else // Define dummy class if in debug/development mode
#include "src/ArduinoMotorShieldR3.h"
#endif

#include "src/crc.h"
#include "src/packet.h"
#include "src/custom_packets.h"

ArduinoMotorShieldR3 md;

static char tmpbuf[64 + 1];
static const char* hexstr(const void* const data, const size_t size)
{
    const unsigned char* ptr = (const unsigned char*)data;
    for(size_t i = 0U; i < size; i++)
    {
        snprintf(tmpbuf + (i * 2), 2 + 1, "%02hhX", ptr[i]);
    }
    return tmpbuf;
}

void setup()
{
    Serial.begin(115200);
    Serial.println("Arduino Motor Shield R3");
    md.init();
}

uint8_t buffer[128];
size_t len;
void loop()
{
    if(Serial.available())
    {
        len = Serial.readBytes(buffer, sizeof(buffer));
        if(len < sizeof(packet_header_t))
        {
            Serial.println("Insufficient header");
            return;
        }

        packet_header_t* hdr = (packet_header_t*)buffer;

        if(packet_verifyheader(hdr) == 0)
        {
            Serial.println("Header failed");
            return;
        }

        Serial.println("Header OK");
        if(len < hdr->size)
        {
            Serial.println("Insufficient data");
            return;
        }

        if(packet_verifydata(hdr) == 0)
        {
            Serial.println("Data Failed");
            return;
        }

        Serial.println("Data OK");

        Serial.print("Data: ");
        Serial.println(hexstr(buffer, len));
        Serial.print("Size: ");
        Serial.println(len);

        switch(hdr->type)
        {
            case CPT_MOTORRUN:
            {
                Serial.println("Run motor");
                packet_motorrun_t* pkt = (packet_motorrun_t*)buffer;
                float left, right;
                memcpy(&left, &pkt->left, sizeof(pkt->left));
                memcpy(&right, &pkt->right, sizeof(pkt->right));

                md.setM1Speed(400 * left);
                md.setM2Speed(400 * right);
                break;
            }
            case CPT_MOTORSTOP:
            {
                break;
            }
            default:
            {
                Serial.print("Unknown type: ");
                Serial.println(hdr->type);
                break;
            }
        }
    }

    /*Serial.println("M1 Speed 100% Forward");
    md.setM1Speed(400);
    Serial.println("M2 Speed 100% Forward");
    md.setM2Speed(400);
    Serial.print("M1 current: ");
    Serial.println(md.getM1CurrentMilliamps());
    Serial.print("M2 current: ");
    Serial.println(md.getM2CurrentMilliamps());
    delay(2000);

    Serial.println("M1 Speed 100% Backward");
    md.setM1Speed(-400);
    Serial.println("M2 Speed 100% Backward");
    md.setM2Speed(-400);
    Serial.print("M1 current: ");
    Serial.println(md.getM1CurrentMilliamps());
    Serial.print("M2 current: ");
    Serial.println(md.getM2CurrentMilliamps());
    delay(2000);

    Serial.println("M1 Speed 50% Forward");
    md.setM1Speed(200);
    Serial.println("M2 Speed 50% Forward");
    md.setM2Speed(200);
    Serial.print("M1 current: ");
    Serial.println(md.getM1CurrentMilliamps());
    Serial.print("M2 current: ");
    Serial.println(md.getM2CurrentMilliamps());
    delay(2000);

    Serial.println("M1 Speed 50% Backward");
    md.setM1Speed(-200);
    Serial.println("M2 Speed 50% Backward");
    md.setM2Speed(-200);
    Serial.print("M1 current: ");
    Serial.println(md.getM1CurrentMilliamps());
    Serial.print("M2 current: ");
    Serial.println(md.getM2CurrentMilliamps());
    delay(2000);

    Serial.println("M1 Speed 0%");
    md.setM1Speed(0);
    Serial.println("M2 Speed 0%");
    md.setM2Speed(0);
    Serial.print("M1 current: ");
    Serial.println(md.getM1CurrentMilliamps());
    Serial.print("M2 current: ");
    Serial.println(md.getM2CurrentMilliamps());
    delay(2000);*/
}
