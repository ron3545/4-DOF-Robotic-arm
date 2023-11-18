# ROBOTICS
This is a project conducted and presented on June 6, 2023.
This 4DOF articulated Robotic arm uses dc motors as its main actuators. For sensor, it uses AS5600 magnetic encoder for reading the position of eanch joints.
Each motor is connetrelled by PID to ensure that the motor positions it self in a right manner, with its speed also controlled. The system uses ESP32, 2 l298n motor drivers
and A4988 stepper driver.


https://github.com/ron3545/ROBOTICS/assets/86136180/28dd57ee-4b28-4dc5-99a4-9b4f8e63d114

https://github.com/ron3545/ROBOTICS/assets/86136180/3583c4ff-21da-484d-be93-96dc9b02fafd


## Magnetic_Encoder Class
The Magnetic_Encoder class represents a magnetic encoder, providing methods for initialization, configuration, and data reading. Key functionalities include setting and getting the encoder location, initializing the encoder, reading raw and scaled angles, detecting the magnet, and burning configurations.

## AS5600 Class
The AS5600 class is a specific implementation of the magnetic encoder using the AS5600 sensor. It includes methods for getting the angle, initializing the sensor, and reading the raw and corrected angles. Additionally, it provides functions for checking magnet presence and adjusting the angle based on the magnet's position.

## Motor Class
The Motor class encapsulates the functionality of a motor in a robotic system. It includes methods for motor initialization, setting the motor direction, moving the motor to a specific angle, and reading the current position. The class supports different motor types, such as stepper and normal motors.

## PS2 class
This header file defines constants and a class, PS2, for interfacing with a PlayStation 2 (PS2) controller. The PS2 class provides methods for setting up communication pins, reading joystick and button values, and configuring the controller mode.



