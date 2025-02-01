#include <Arduino.h>
constexpr auto SA5 = 0;
constexpr auto SA4 = 1;
constexpr auto SA3 = 2;
constexpr auto SA2 = 3;
constexpr auto SA1 = 4;
constexpr auto SA0 = 5;
constexpr auto SOE = 6;
constexpr auto SWR = 7;
constexpr auto SD0 = 8;
constexpr auto SD1 = 9;
constexpr auto SD2 = 10;
constexpr auto SD3 = 11;
constexpr auto SD4 = 12;
constexpr auto SD5 = 13;
constexpr auto SD6 = 14;
constexpr auto SD7 = 15;
void setup() {
    pinMode(SA5, OUTPUT);
    pinMode(SA4, OUTPUT);
    pinMode(SA3, OUTPUT);
    pinMode(SA2, OUTPUT);
    pinMode(SA1, OUTPUT);
    pinMode(SA0, OUTPUT);
    pinMode(SOE, OUTPUT);
    pinMode(SWR, OUTPUT);
    pinMode(SD0, OUTPUT);
    pinMode(SD1, OUTPUT);
    pinMode(SD2, OUTPUT);
    pinMode(SD3, OUTPUT);
    pinMode(SD4, OUTPUT);
    pinMode(SD5, OUTPUT);
    pinMode(SD6, OUTPUT);
    pinMode(SD7, OUTPUT);
    digitalWrite(SA5, LOW);
    digitalWrite(SA4, LOW);
    digitalWrite(SA3, LOW);
    digitalWrite(SA2, LOW);
    digitalWrite(SA1, LOW);
    digitalWrite(SA0, LOW);
    digitalWrite(SOE, HIGH);
    digitalWrite(SWR, HIGH);
    digitalWrite(SD0, LOW);
    digitalWrite(SD1, LOW);
    digitalWrite(SD2, LOW);
    digitalWrite(SD3, LOW);
    digitalWrite(SD4, LOW);
    digitalWrite(SD5, LOW);
    digitalWrite(SD6, LOW);
    digitalWrite(SD7, LOW);
}

void loop() {

}
