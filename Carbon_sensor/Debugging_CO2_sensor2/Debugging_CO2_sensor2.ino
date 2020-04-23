#include <SoftwareSerial.h>

SoftwareSerial mySerial(5, 6); // RX, TX

byte cmd[9] = {0xFF,0x01,0x86,0x00,0x00,0x00,0x00,0x00,0x79};

char response[9];

#define pwmPin 7

int prevVal = LOW;

long th, tl, h, l, ppm, ppm2 = 0.0;

void setup() {

    Serial.begin(9600);

    mySerial.begin(9600);

    pinMode(pwmPin, INPUT);

}

void loop(){

    Serial.println("I start now.");

    mySerial.write(cmd,9);

    mySerial.readBytes(response, 9);

    int responseHigh = (int) response[2];

    int responseLow = (int) response[3];

    ppm = (256*responseHigh)+responseLow;

    Serial.println("We have UART");

    //CO2 via pwm

    do {

        th = pulseIn(pwmPin, HIGH, 1004000) / 1000.0;

        tl = 1004 - th;

        ppm2 = 2000 * (th-2)/(th+tl-4);

        Serial.println("Loop PWM");

    } while (ppm2 < 0.0);

    Serial.println("We have PWM");

    Serial.println("PPM-UART:");
    
    Serial.println(ppm);

    Serial.println("PPM-PWM:");

    Serial.println(ppm2);

    Serial.println("-----------");

    delay(5000);

}
