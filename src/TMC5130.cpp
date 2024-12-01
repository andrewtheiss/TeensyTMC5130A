// TMC5130.cpp
#include "TMC5130.h"

// Register definitions
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
#define VSTOP          0x2B
#define XTARGET         0x2D
#define CHOPCONF        0x6C
#define COOLCONF        0x6D
#define DRVSTATUS       0x6F

TMC5130::TMC5130(uint8_t clk16Pin, uint8_t enPin, uint8_t csPin, uint8_t buttonPin) :
    _clk16Pin(clk16Pin),
    _enPin(enPin),
    _csPin(csPin),
    _buttonPin(buttonPin),
    _buttonState(HIGH),
    _lastButtonState(HIGH),
    _lastDebounceTime(0),
    _debounceDelay(50),
    _motorMoving(false),
    _moveCount(0),
    _currentMove(0),
    _targetPosition(0),
    _lastMoveTime(0),
    _moveDelay(500),
    _pendingMoveRequests(0)
{
}

void TMC5130::begin() {
    Serial.begin(9600);
    while (!Serial);

    SPI.begin();
    pinMode(_csPin, OUTPUT);
    pinMode(_clk16Pin, OUTPUT);
    pinMode(_enPin, OUTPUT);
    pinMode(_buttonPin, INPUT_PULLUP);

    digitalWrite(_csPin, HIGH);
    digitalWrite(_clk16Pin, LOW);
    digitalWrite(_enPin, LOW);

    Serial.println("Testing SPI...");
    reset();
    reinitializeRegisters();
    debugTStep();
}

void TMC5130::reset() {
    Serial.println("Resetting TMC5130...");
    writeRegister(GSTAT, 0x07);
    delay(100);
    uint32_t gstat = readRegister(GSTAT);
    Serial.print("GSTAT after reset: 0x");
    Serial.println(gstat, HEX);
}

void TMC5130::initialize() {
    Serial.println("Initializing driver...");
    writeRegister(GCONF, 0x00000004);
    delay(100);
    uint32_t gconf = readRegister(GCONF);
    Serial.print("GCONF after initialization: 0x");
    Serial.println(gconf, HEX);
}

uint32_t TMC5130::readRegister(uint8_t reg) {
    uint32_t data = 0;
    uint8_t status;

    digitalWrite(_csPin, LOW);
    delayMicroseconds(100);

    status = SPI.transfer(reg & 0x7F);
    Serial.print("Status byte: 0x");
    Serial.println(status, HEX);

    data |= ((uint32_t)SPI.transfer(0x00) << 24);
    data |= ((uint32_t)SPI.transfer(0x00) << 16);
    data |= ((uint32_t)SPI.transfer(0x00) << 8);
    data |= SPI.transfer(0x00);

    digitalWrite(_csPin, HIGH);
    delayMicroseconds(100);

    return data;
}

void TMC5130::writeRegister(uint8_t reg, uint32_t value) {
    SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE3));
    digitalWrite(_csPin, LOW);
    delayMicroseconds(10);

    SPI.transfer(reg | 0x80);
    SPI.transfer((value >> 24) & 0xFF);
    SPI.transfer((value >> 16) & 0xFF);
    SPI.transfer((value >> 8) & 0xFF);
    SPI.transfer(value & 0xFF);

    digitalWrite(_csPin, HIGH);
    SPI.endTransaction();
    delayMicroseconds(10);
}

void TMC5130::configureMotor() {
    Serial.println("Configuring motor...");
    writeRegister(IHOLD_IRUN, 0x00061F10);  // IHOLDDELAY=6, IRUN=31, IHOLD=16
   // writeRegister(IHOLD_IRUN, 0x000F1F10);
    delay(100);
    
    uint32_t ihold = readRegister(IHOLD_IRUN);
    Serial.print("IHOLD_IRUN: 0x");
    Serial.println(ihold, HEX);
    
    configureChopper();
    configureMotion();
}

// Adjust the vsense Bit in CHOPCONF: The vsense bit determines the full-scale voltage of the sense resistor,
// affecting the maximum motor current. Setting vsense to 0 allows for a higher full-scale voltage, thereby 
// increasing the maximum current through the motor coils.
// Microstepping Resolution Values (MRES bits 24-27):
// Microsteps	MRES Value
// Full step	0b0000
// 1/2	0b0001
// 1/4	0b0010
// 1/8	0b0011
// 1/16	0b0100
// 1/32	0b0101
// 1/64	0b0110
// 1/128	0b0111
// 1/256	0b1000
void TMC5130::configureChopper() {
    Serial.println("Configuring chopper for lower microstepping...");
    uint32_t chopconf = 0x000000C3;  // Base configuration with vsense=0
    chopconf |= (0b1000 << 24);      // Set MRES bits (bits 24-27) to 0b0100 for 1/16 microstepping
    writeRegister(CHOPCONF, chopconf);
    chopconf = readRegister(CHOPCONF);
    Serial.print("CHOPCONF: 0x");
    Serial.println(chopconf, HEX);
}

void TMC5130::configureMotion() {
    Serial.println("Configuring motion profile...");
    writeRegister(RAMPMODE, 0x00);
    writeRegister(VSTART, 10);
    writeRegister(A1, 5000);
    writeRegister(V1, 100000);
    writeRegister(AMAX, 2000);
    writeRegister(VMAX, 400000);
    writeRegister(DMAX, 4000);
    writeRegister(D1, 1000);
    writeRegister(VSTOP, 10);
    writeRegister(RAMPMODE, 0x00);
}

void TMC5130::moveMotor(int32_t position) {
    Serial.print("Moving motor to position: ");
    Serial.println(position);
    
    reapplyTargetPosition(position);

    while (true) {
        int32_t actual_pos = (int32_t)readRegister(XACTUAL);
        Serial.print("Current position: ");
        Serial.println(actual_pos);

        if (abs(actual_pos - position) < 10) break;
        delay(100);
    }
    Serial.println("Target position reached.");
}

void TMC5130::update() {
    // Read the button
    int reading = digitalRead(_buttonPin);

    if (reading != _lastButtonState) {
        _lastDebounceTime = millis();
    }

    if ((millis() - _lastDebounceTime) > _debounceDelay) {
        if (reading != _buttonState) {
            _buttonState = reading;
            _pendingMoveRequests++;
            Serial.println("Button toggled. Movement request queued.");
        }
    }

    _lastButtonState = reading;

    // Handle motor movement state machine
    if (!_motorMoving && _pendingMoveRequests > 0) {
        _moveCount = 3;
        _currentMove = 0;
        _motorMoving = true;
        _targetPosition = 0;
        _pendingMoveRequests--;
        Serial.println("Starting motor movement.");
    }

    if (_motorMoving) {
        if (_currentMove < _moveCount) {
            if (_targetPosition == 0 && (millis() - _lastMoveTime >= _moveDelay)) {
                int32_t xactual = (int32_t)readRegister(XACTUAL);
                _targetPosition = xactual + (3 * 51200);
                writeRegister(XTARGET, _targetPosition);
                Serial.print("Moving to target position: ");
                Serial.println(_targetPosition);
            } else if (_targetPosition != 0) {
                int32_t xactual = (int32_t)readRegister(XACTUAL);
                if (abs(xactual - _targetPosition) < 10) {
                    Serial.println("Target reached.");
                    _currentMove++;
                    _targetPosition = 0;
                    _lastMoveTime = millis();
                } else {
                    Serial.print("XACTUAL: ");
                    Serial.println(xactual);
                }
            }
        } else {
            _motorMoving = false;
            Serial.println("All movements completed.");
        }
    }

    delay(50);
}

// Remaining method implementations for TMC5130.cpp

void TMC5130::writeAndVerify(uint8_t reg, uint32_t value) {
    // Write value
    digitalWrite(_csPin, LOW);
    delayMicroseconds(100);

    SPI.transfer(reg | 0x80);  // Set write bit
    SPI.transfer((value >> 24) & 0xFF);
    SPI.transfer((value >> 16) & 0xFF);
    SPI.transfer((value >> 8) & 0xFF);
    SPI.transfer(value & 0xFF);

    digitalWrite(_csPin, HIGH);
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

void TMC5130::reapplyTargetPosition(int32_t target) {
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

void TMC5130::debugTStep() {
    uint32_t tstep = readRegister(TSTEP);
    Serial.print("TSTEP: 0x");
    Serial.println(tstep, HEX);
}

void TMC5130::checkStatus() {
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

void TMC5130::testRegisters() {
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

void TMC5130::monitorDriver() {
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

void TMC5130::reinitializeRegisters() {
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