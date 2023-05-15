#pragma once
#include "Arduino.h"

#define JOYSTICK_MAX_VAL    255
#define JOYSTICK_MID_VAL_X  123
#define JOYSTICK_MID_VAL_Y  132

#define PS2BTN_LEFT         0x8000
#define PS2BTN_DOWN         0x4000
#define PS2BTN_RIGHT        0x2000
#define PS2BTN_UP           0x1000
#define PS2BTN_START        0x0800
#define PS2BTN_SELECT       0x0100
#define PS2BTN_SQUARE       0x0080
#define PS2BTN_CROSS        0x0040
#define PS2BTN_CIRCLE       0x0020
#define PS2BTN_TRIANGLE     0x0010
#define PS2BTN_R1           0x0008
#define PS2BTN_L1           0x0004
#define PS2BTN_R2           0x0002
#define PS2BTN_L2           0x0001

// Protocol commands and constants

#define PS2PROT_IDLE        0x00
#define PS2PROT_NONZERO     0xFF
#define PS2PROT_ZERO        0x00
#define PS2PROT_HANDSHAKE   0x01
#define PS2PROT_GETDATA     0x42
#define PS2PROT_STARTDATA   0x5A
#define PS2PROT_CONFIG      0x43
#define PS2PROT_CONFIGMOTOR 0x4D
#define PS2PROT_MOTORMAP    0x01
#define PS2PROT_ENTERCONFIG 0x01
#define PS2PROT_EXITCONFIG  0x00
#define PS2PROT_EXITCFGCNT  0x5A
#define PS2PROT_CONFIGMODE  0x44
#define PS2PROT_MODELOCK    0x03
#define PS2MODE_ANALOG      0x01
#define PS2MODE_DIGITAL     0x00

// Read function return codes

#define PS2ERROR_SUCCESS    0
#define PS2ERROR_NODATA     1

class PS2
{
    public:

        /**
         * @brief Holds the state of the controller
         * 
         */
        struct PS2DATA
        {
            unsigned int buttons;   //!< The state of all buttons encoded as a bitfield
            byte JoyLeftX;          //!< The horizontal value of the left joystick
            byte JoyLeftY;          //!< The vertical value of the left joystick
            byte JoyRightX;         //!< The horizontal value of the right joystick
            byte JoyRightY;         //!< The vertical value of the right joystick
        };

        /**
         * @brief Construct a new PSX::PSX object
         * 
         */
        PS2() {}

        /**
         * @brief Assign the pins to be used for communication and set the protocol timing
         * 
         * @param dataPin The data pin (usually brown wire)
         * @param cmdPin The command pin (usually orange wire)
         * @param attPin The attention pin (usually yellow wire)
         * @param clockPin The clock pin (usually blue wire)
         * @param delay The protocol timing delay in microseconds (usually 10)
         */
        void setupPins(byte dataPin, byte cmdPin, byte attPin, byte clockPin, byte delay);

        /**
         * @brief Read the state of all joysticks and buttons from the controller
         * 
         * @param psxdata The state read from the controller
         * @return int Error code, PSXERROR_SUCCESS or PSXERROR_NODATA
         */
        int read(PS2DATA &psxdata);

        /**
         * @brief Configures the controller, disables vibration and sets either analog or digital mode
         * 
         * @param mode What mode to set. PSXMODE_ANALOG or PSXMODE_DIGITAL.
         */
        void config(byte mode);
        
    private:
    
        /**
         * @brief Send a command to the PSX controller
         * 
         * @param command The command to send
         * @param response The response from the controller
         */
        void sendCommand(byte command, byte &response);

        byte _dataPin, _cmdPin, _attPin, _clockPin, _delay;

};
