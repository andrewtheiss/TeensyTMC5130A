#include <Arduino.h>
#include <SPI.h>

// Pin definitions
#define CLK16_PIN 8
#define EN_PIN 9
#define CS_PIN 10

// TMC5130 Register Addresses
#define GCONF           0x00
#define GSTAT           0x01
#define IO_INPUT_OUTPUT 0x04
#define IHOLD_IRUN      0x10
#define TSTEP           0x12
#define RAMPMODE        0x20
#define XACTUAL         0x21
#define VSTART          0x23
#define A1              0x24
#define V1              0x25
#define AMAX            0x26
#define VMAX            0x27
#define DMAX            0x28
#define D1              0x2A
#define VSTOP           0x2B
#define XTARGET         0x2D
#define CHOPCONF        0x6C
#define COOLCONF        0x6D
#define DRVSTATUS       0x6F

// Function declarations
void resetDriver();
void initializeDriver();
void configureMotor();
void moveMotor(int32_t position);
uint32_t readRegister(uint8_t reg);
void writeRegister(uint8_t reg, uint32_t value);
void checkStatus();
void testRegisters();
void writeAndVerify(uint8_t reg, uint32_t value);

// Button/Switch Handling Variables
int buttonState = HIGH;         // Current state of the button
int lastButtonState = HIGH;     // Previous state of the button
unsigned long lastDebounceTime = 0;  // Last time the button state was toggled
unsigned long debounceDelay = 50;    // Debounce time in milliseconds

// Motor Movement State Machine Variables
bool motorMoving = false;
int moveCount = 0;              // Number of times to move the motor
int currentMove = 0;            // Current move number
int32_t targetPosition = 0;     // Target position for motor movement
unsigned long lastMoveTime = 0;  // Time when the last move was completed
unsigned long moveDelay = 500;   // Delay between moves in milliseconds
int pendingMoveRequests = 0;     // Number of pending movement requests

void resetDriver() {
    Serial.println("Resetting TMC5130...");
    writeRegister(GSTAT, 0x07);  // Clear all reset flags
    delay(100);
    uint32_t gstat = readRegister(GSTAT);
    Serial.print("GSTAT after reset: 0x");
    Serial.println(gstat, HEX);
}

void monitorDriver() {
    uint32_t gstat = readRegister(GSTAT);
    Serial.print("GSTAT: 0x");
    Serial.println(gstat, HEX);

    uint32_t drvstatus = readRegister(DRVSTATUS);
    Serial.print("DRVSTATUS: 0x");
    Serial.println(drvstatus, HEX);

    // Parse and display DRVSTATUS flags
    bool otpw = (drvstatus & 0x00000002) != 0;
    bool ot = (drvstatus & 0x00000001) != 0;
    bool s2ga = (drvstatus & 0x00000004) != 0;
    bool s2gb = (drvstatus & 0x00000008) != 0;
    bool ola = (drvstatus & 0x00000010) != 0;
    bool olb = (drvstatus & 0x00000020) != 0;

    Serial.print("Overtemperature warning: ");
    Serial.println(otpw ? "Yes" : "No");
    Serial.print("Overtemperature: ");
    Serial.println(ot ? "Yes" : "No");
    Serial.print("Short to GND coil A: ");
    Serial.println(s2ga ? "Yes" : "No");
    Serial.print("Short to GND coil B: ");
    Serial.println(s2gb ? "Yes" : "No");
    Serial.print("Open load A: ");
    Serial.println(ola ? "Yes" : "No");
    Serial.print("Open load B: ");
    Serial.println(olb ? "Yes" : "No");
}

void configureMotion() {
    Serial.println("Configuring motion profile...");
    writeRegister(RAMPMODE, 0x00);  // Positioning mode
    uint32_t rampmode = readRegister(RAMPMODE);
    Serial.print("Verified RAMPMODE: 0x");
    Serial.println(rampmode, HEX);
    writeRegister(VSTART, 10);       // Start velocity
    writeRegister(A1, 5000);         // First acceleration
    writeRegister(V1, 100000);       // First velocity
    writeRegister(AMAX, 10000);      // Maximum acceleration
    writeRegister(VMAX, 400000);     // Maximum velocity
    writeRegister(DMAX, 5000);       // Maximum deceleration
    writeRegister(D1, 1000);         // Final deceleration
    writeRegister(VSTOP, 10);        // Stop velocity
    writeRegister(RAMPMODE, 0x00);   // Positioning mode
    rampmode = readRegister(RAMPMODE);
    Serial.print("Reapplied RAMPMODE: 0x");
    Serial.println(rampmode, HEX);
}

void testSmallMovement() {
    writeRegister(XTARGET, 3 * 51200);  // Move 3 rotations
    delay(1000);
    while (true) {
        int32_t xactual = (int32_t)readRegister(XACTUAL);
        Serial.print("XACTUAL: ");
        Serial.println(xactual);

        if (abs(xactual - (3 * 51200)) < 10) {
            Serial.println("Target reached.");
            break;
        }
        delay(100);
    }
}

// Additional Functions for Motor Control
void configureChopper() {
    Serial.println("Configuring chopper...");
    writeRegister(CHOPCONF, 0x000100C3);  // SpreadCycle mode, TOFF=3
    uint32_t chopconf = readRegister(CHOPCONF);
    Serial.print("CHOPCONF: 0x");
    Serial.println(chopconf, HEX);
}

void reinitializeRegisters() {
    // Configure GCONF
    writeRegister(GCONF, 0x00000004);  // Internal reference voltage
    uint32_t gconf = readRegister(GCONF);
    Serial.print("GCONF: 0x");
    Serial.println(gconf, HEX);

    // Configure CHOPCONF
    writeRegister(CHOPCONF, 0x000100C3);  // SpreadCycle
    uint32_t chopconf = readRegister(CHOPCONF);
    Serial.print("CHOPCONF: 0x");
    Serial.println(chopconf, HEX);

    // Configure Motion Parameters
    configureMotion();

    // Reset XACTUAL to 0
    writeRegister(XACTUAL, 0);
    uint32_t xactual = readRegister(XACTUAL);
    Serial.print("XACTUAL after reset: ");
    Serial.println(xactual);
}

void debugTStep() {
    uint32_t tstep = readRegister(TSTEP);
    Serial.print("TSTEP: 0x");
    Serial.println(tstep, HEX);
}

void reapplyTargetPosition(int32_t target) {
    Serial.print("Reapplying XTARGET: ");
    Serial.println(target);

    writeRegister(XTARGET, target);
    uint32_t xtarget = readRegister(XTARGET);
    Serial.print("Reapplied XTARGET: 0x");
    Serial.println(xtarget, HEX);

    if (xtarget != target) {
        Serial.println("WARNING: XTARGET reapplication failed!");
    } else {
        Serial.println("XTARGET successfully reapplied.");
    }
}

void setup() {
    Serial.begin(9600);
    while (!Serial);

    SPI.begin();
    pinMode(CS_PIN, OUTPUT);
    pinMode(CLK16_PIN, OUTPUT);
    pinMode(EN_PIN, OUTPUT);
    digitalWrite(CS_PIN, HIGH);
    digitalWrite(CLK16_PIN, LOW);
    digitalWrite(EN_PIN, LOW);

    // Configure button pin with internal pull-up
    pinMode(2, INPUT_PULLUP);  // Enable internal pull-up resistor on pin 2
    pinMode(3, INPUT_PULLUP);  // Optionally enable pull-up on pin 3 for extra stability


    Serial.println("Testing SPI...");
    resetDriver();             // Force a full reset
    reinitializeRegisters();   // Reinitialize all relevant registers

    debugTStep();  // Check TSTEP value

    // Test small movement (optional)
    // Serial.println("Testing small movement...");
    // testSmallMovement();
}

void initializeDriver() {
    Serial.println("Initializing driver to 4...");
    writeRegister(GCONF, 0x00000004);  // Enable internal reference voltage
    delay(100);
    uint32_t gconf = readRegister(GCONF);
    Serial.print("GCONF after initialization: 0x");
    Serial.println(gconf, HEX);
}

void configureMotor() {
    Serial.println("Configuring motor...");
    writeRegister(IHOLD_IRUN, 0x000F1F10);  // Set IHOLD=16, IRUN=31, IHOLDDELAY=6
    delay(100);
    uint32_t ihold = readRegister(IHOLD_IRUN);
    Serial.print("IHOLD_IRUN: 0x");
    Serial.println(ihold, HEX);

    // Ensure driver status is healthy
    uint32_t gstat = readRegister(GSTAT);
    Serial.print("GSTAT: 0x");
    Serial.println(gstat, HEX);

    uint32_t drvstatus = readRegister(DRVSTATUS);
    Serial.print("DRVSTATUS: 0x");
    Serial.println(drvstatus, HEX);

    Serial.println("Motor configuration complete");
}

void moveMotor(int32_t position) {
    Serial.print("Moving motor to position: ");
    Serial.println(position);

    reapplyTargetPosition(position);  // Ensure XTARGET is applied

    while (true) {
        int32_t actual_pos = (int32_t)readRegister(XACTUAL);  // Read the current position
        Serial.print("Current position: ");
        Serial.println(actual_pos);

        // Break when the motor reaches the target (allowing for some error)
        if (abs(actual_pos - position) < 10) break;
        delay(100);  // Poll every 100ms
    }
    Serial.println("Target position reached.");
}

uint32_t readRegister(uint8_t reg) {
    uint32_t data = 0;
    uint8_t status;

    digitalWrite(CS_PIN, LOW);
    delayMicroseconds(100);

    status = SPI.transfer(reg & 0x7F);  // Clear write bit
    Serial.print("Status byte: 0x");
    Serial.println(status, HEX);

    data |= ((uint32_t)SPI.transfer(0x00) << 24);
    data |= ((uint32_t)SPI.transfer(0x00) << 16);
    data |= ((uint32_t)SPI.transfer(0x00) << 8);
    data |= SPI.transfer(0x00);

    digitalWrite(CS_PIN, HIGH);
    delayMicroseconds(100);

    return data;
}

void writeRegister(uint8_t reg, uint32_t value) {
    SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE3));
    digitalWrite(CS_PIN, LOW);
    delayMicroseconds(10);

    SPI.transfer(reg | 0x80);
    SPI.transfer((value >> 24) & 0xFF);
    SPI.transfer((value >> 16) & 0xFF);
    SPI.transfer((value >> 8) & 0xFF);
    SPI.transfer(value & 0xFF);

    digitalWrite(CS_PIN, HIGH);
    SPI.endTransaction();
    delayMicroseconds(10);

    Serial.print("Wrote to register 0x");
    Serial.print(reg, HEX);
    Serial.print(": 0x");
    Serial.println(value, HEX);
}

void writeAndVerify(uint8_t reg, uint32_t value) {
    // Write value
    digitalWrite(CS_PIN, LOW);
    delayMicroseconds(100);

    SPI.transfer(reg | 0x80);  // Set write bit
    SPI.transfer((value >> 24) & 0xFF);
    SPI.transfer((value >> 16) & 0xFF);
    SPI.transfer((value >> 8) & 0xFF);
    SPI.transfer(value & 0xFF);

    digitalWrite(CS_PIN, HIGH);
    delayMicroseconds(100);

    Serial.print("Wrote 0x");
    Serial.print(value, HEX);
    Serial.print(" to register 0x");
    Serial.println(reg, HEX);

    // Read back and verify
    uint32_t readback = readRegister(reg);
    Serial.print("Read back: 0x");
    Serial.println(readback, HEX);

    if (readback == value) {
        Serial.println("Write verified successfully!");
    } else {
        Serial.println("WARNING: Readback doesn't match written value!");
    }
}

void checkStatus() {
    Serial.println("\n--- Status Check ---");

    uint32_t drvstatus = readRegister(DRVSTATUS);
    Serial.print("DRVSTATUS: 0x");
    Serial.println(drvstatus, HEX);

    // Parse DRVSTATUS
    bool otpw = (drvstatus & 0x00000002) != 0;
    bool ot = (drvstatus & 0x00000001) != 0;
    bool s2ga = (drvstatus & 0x00000004) != 0;
    bool s2gb = (drvstatus & 0x00000008) != 0;
    bool ola = (drvstatus & 0x00000010) != 0;
    bool olb = (drvstatus & 0x00000020) != 0;

    Serial.print("Temperature warning: "); Serial.println(otpw ? "Yes" : "No");
    Serial.print("Overtemperature: "); Serial.println(ot ? "Yes" : "No");
    Serial.print("Short to GND coil A: "); Serial.println(s2ga ? "Yes" : "No");
    Serial.print("Short to GND coil B: "); Serial.println(s2gb ? "Yes" : "No");
    Serial.print("Open load A: "); Serial.println(ola ? "Yes" : "No");
    Serial.print("Open load B: "); Serial.println(olb ? "Yes" : "No");
}

void testRegisters() {
    Serial.println("\n--- Testing Registers ---");

    Serial.print("GCONF: 0x");
    Serial.println(readRegister(GCONF), HEX);

    Serial.print("GSTAT: 0x");
    Serial.println(readRegister(GSTAT), HEX);

    Serial.print("IHOLD_IRUN: 0x");
    Serial.println(readRegister(IHOLD_IRUN), HEX);

    Serial.print("CHOPCONF: 0x");
    Serial.println(readRegister(CHOPCONF), HEX);
}

void loop() {
    // Read the button
    int reading = digitalRead(2);

    if (reading != lastButtonState) {
        lastDebounceTime = millis();
    }

    if ((millis() - lastDebounceTime) > debounceDelay) {
        if (reading != buttonState) {
            buttonState = reading;

            // Button state has changed (toggle detected)
            pendingMoveRequests++;
            Serial.println("Button toggled. Movement request queued.");
        }
    }

    lastButtonState = reading;

    // Handle motor movement state machine
    if (!motorMoving && pendingMoveRequests > 0) {
        // Start a new movement
        moveCount = 3;        // Set the number of movements
        currentMove = 0;      // Reset current move counter
        motorMoving = true;   // Set motor moving flag
        targetPosition = 0;   // Reset target position
        pendingMoveRequests--; // Decrease pending requests
        Serial.println("Starting motor movement.");
    }

    if (motorMoving) {
        if (currentMove < moveCount) {
            // If we have not started moving yet, or moveDelay has passed
            if (targetPosition == 0 && (millis() - lastMoveTime >= moveDelay)) {
                int32_t xactual = (int32_t)readRegister(XACTUAL);
                targetPosition = xactual + (3 * 51200);  // Move 3 rotations from current position
                writeRegister(XTARGET, targetPosition);
                Serial.print("Moving to target position: ");
                Serial.println(targetPosition);
            } else if (targetPosition != 0) {
                // Check if target has been reached
                int32_t xactual = (int32_t)readRegister(XACTUAL);
                if (abs(xactual - targetPosition) < 10) {
                    // Target reached
                    Serial.println("Target reached.");
                    currentMove++;
                    targetPosition = 0;  // Reset target position for next move
                    lastMoveTime = millis();  // Record the time when the move completed
                } else {
                    // Motor is still moving
                    Serial.print("XACTUAL: ");
                    Serial.println(xactual);
                }
            }
        } else {
            // All movements completed
            motorMoving = false;
            Serial.println("All movements completed.");
        }
    }

    // Other code, e.g., monitor driver
    // monitorDriver();  // Uncomment if needed
    delay(50);  // Adjust delay as needed
}
