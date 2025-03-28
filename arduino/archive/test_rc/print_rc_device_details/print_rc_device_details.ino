#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <printf.h>

// Use this code to print the details of the RF24 radio module
// to the Serial Monitor. This is useful for debugging and checking the
// configuration of the radio module.

RF24 radio(7, 8); // CE, CSN

const byte address[6] = "00001";

void setup()
{
    Serial.begin(115200);
    printf_begin();

    if (radio.begin())
    {
        Serial.println("Radio hardware is responding!");
    }
    else
    {
        Serial.println("Radio hardware is not responding!");
        while (1)
        {
            delay(1000);
        }
    }
    radio.openReadingPipe(0, address);
    radio.setPALevel(RF24_PA_MIN);
    radio.startListening();
    radio.printDetails();
}

void loop()
{
}