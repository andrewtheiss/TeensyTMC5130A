// main.cpp
#include "TMC5130.h"

// Pin definitions
#define CLK16_PIN 8
#define EN_PIN 9
#define CS_PIN 10
#define BUTTON_PIN 2

// Create TMC5130 instance
TMC5130 stepper(CLK16_PIN, EN_PIN, CS_PIN, BUTTON_PIN);

void setup() {
    stepper.begin();
}

void loop() {
    stepper.update();
}