#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(7, 8); // CE, CSN

const byte address[6] = "00001";

void setup()
{
    Serial.begin(9600);
    Serial.println("Booted!");
    if (!radio.begin())
    {
        Serial.println(F("radio hardware not responding!"));
        while (1){} // hold program in infinite loop to prevent subsequent errors
    }
}

void loop()
{
    const char text[] = "Hello World";
    radio.write(&text, sizeof(text));
    delay(1000);
}