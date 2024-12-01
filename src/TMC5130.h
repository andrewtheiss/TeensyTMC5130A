// TMC5130.h
#ifndef TMC5130_H
#define TMC5130_H

#include <Arduino.h>
#include <SPI.h>

class TMC5130 {
private:
    // Pin configurations
    const uint8_t _clk16Pin;
    const uint8_t _enPin;
    const uint8_t _csPin;
    const uint8_t _buttonPin;

    // New private members
    uint8_t _microstepResolution;  // MRES value (0b0000 to 0b1000)
    uint32_t _stepsPerRevolution;  // Calculated steps per revolution based on MRES and motor steps

    // Button/Switch handling variables
    int _buttonState;
    int _lastButtonState;
    unsigned long _lastDebounceTime;
    const unsigned long _debounceDelay;

    // Motor movement state machine variables
    bool _motorMoving;
    int _moveCount;
    int _currentMove;
    int32_t _targetPosition;
    unsigned long _lastMoveTime;
    const unsigned long _moveDelay;
    int _pendingMoveRequests;

    // Private methods
    uint32_t readRegister(uint8_t reg);
    void writeRegister(uint8_t reg, uint32_t value);
    void writeAndVerify(uint8_t reg, uint32_t value);
    void configureChopper();
    void configureMotion();
    void reapplyTargetPosition(int32_t target);
    void debugTStep();
    void updateStepsPerRevolution();

public:
    // Constructor
    TMC5130(uint8_t clk16Pin, uint8_t enPin, uint8_t csPin, uint8_t buttonPin);

    // Public methods
    void begin();
    void reset();
    void initialize();
    void configureMotor();
    void moveMotor(int32_t position);
    void checkStatus();
    void testRegisters();
    void monitorDriver();
    void update();  // This will replace the loop() function
    void reinitializeRegisters();
    void setMicrostepResolution(uint8_t mres);
    uint8_t getMicrostepResolution() const;
    uint32_t getStepsPerRevolution() const;
};

#endif // TMC5130_H