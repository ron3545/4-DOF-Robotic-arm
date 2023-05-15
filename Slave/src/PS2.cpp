#include "PS2.hpp"

void PS2::sendCommand(byte command, byte &response)
{
    // Write out a byte command, or read a byte from the controller
    // When reading, the "command" is usually IDLE (0), thus no data gets written
    // The response is always the response from the previous command
    response = 0;
    for (int i=0; i<8; i++)
    {
        // Write command bit
        digitalWrite(_cmdPin, (command & (1 << i))? HIGH:LOW);
        digitalWrite(_clockPin, LOW);
        delayMicroseconds(_delay);
        // Read response bit
        if (digitalRead(_dataPin)) response |= 1 << i;
        digitalWrite(_clockPin, HIGH);
        delayMicroseconds(_delay);
    }
}

void PS2::setupPins(byte dataPin, byte cmdPin, byte attPin, byte clockPin, byte delay)
{
    // Assign pins
    _dataPin = dataPin;
    _cmdPin = cmdPin;
    _attPin = attPin;
    _clockPin = clockPin;
    _delay = delay;
    // Setup pins and pin states
    pinMode(dataPin, INPUT);
    digitalWrite(dataPin, HIGH);
    pinMode(cmdPin, OUTPUT);
    pinMode(attPin, OUTPUT);
    digitalWrite(attPin, HIGH);
    pinMode(clockPin, OUTPUT);
    digitalWrite(clockPin, HIGH);
}

int PS2::read(PS2DATA &psxdata)
{
    // Send data request
    digitalWrite(_attPin, LOW);
    byte response;
    sendCommand(PS2PROT_HANDSHAKE, response);
    sendCommand(PS2PROT_GETDATA, response);
    sendCommand(PS2PROT_IDLE, response);
    // Check response
    if(response == PS2PROT_STARTDATA)
    {
        // Decode response
        byte data[6];
        for(int i=0; i<6; i++) sendCommand(PS2PROT_IDLE, data[i]);
        digitalWrite(_attPin, HIGH);
        psxdata.buttons = ~(data[1] + (data[0] << 8));
        psxdata.JoyRightX = data[2];
        psxdata.JoyRightY = data[3];
        psxdata.JoyLeftX = data[4];
        psxdata.JoyLeftY = data[5];
        return PS2ERROR_SUCCESS;
    }
    else
    {
        digitalWrite(_attPin, HIGH);
        return PS2ERROR_NODATA;
    }
}

void PS2::config(byte mode)
{
    // Perform initial handshake with the controller
    // Enter config
    byte response;
    digitalWrite(_attPin, LOW);
    sendCommand(PS2PROT_HANDSHAKE, response);
    sendCommand(PS2PROT_CONFIG, response);
    sendCommand(PS2PROT_IDLE, response);
    sendCommand(PS2PROT_ENTERCONFIG, response);
    sendCommand(PS2PROT_ZERO, response);    
    digitalWrite(_attPin, HIGH);
    delayMicroseconds(_delay);
    // Set mode
    digitalWrite(_attPin, LOW); 
    sendCommand(PS2PROT_HANDSHAKE, response);
    sendCommand(PS2PROT_CONFIGMODE, response);
    sendCommand(PS2PROT_IDLE, response);
    sendCommand(mode, response);
    sendCommand(PS2PROT_MODELOCK, response);
    for(int i=0; i<4; i++) sendCommand(PS2PROT_ZERO, response);  
    digitalWrite(_attPin, HIGH);    
    delayMicroseconds(_delay);
    // Disable vibration motors
    digitalWrite(_attPin, LOW); 
    sendCommand(PS2PROT_HANDSHAKE, response);
    sendCommand(PS2PROT_CONFIGMOTOR, response);
    sendCommand(PS2PROT_IDLE, response);
    sendCommand(PS2PROT_ZERO, response);
    sendCommand(PS2PROT_MOTORMAP, response);
    for(int i=0; i<4; i++) sendCommand(PS2PROT_NONZERO, response);   
    digitalWrite(_attPin, HIGH);    
    delayMicroseconds(_delay);
    // Finish config
    digitalWrite(_attPin, LOW);
    sendCommand(PS2PROT_HANDSHAKE, response);
    sendCommand(PS2PROT_CONFIG, response);
    sendCommand(PS2PROT_IDLE, response);
    sendCommand(PS2PROT_EXITCONFIG, response);
    for(int i=0; i<5; i++) sendCommand(PS2PROT_EXITCFGCNT, response);        
    digitalWrite(_attPin, HIGH);
}
