# ROBOTICS
The articulated 4DOF robotic arm project, conducted and presented on June 6, 2023, showcases the implementation of a versatile robotic arm using DC motors as the primary actuators. This arm incorporates advanced control techniques, utilizing PID (Proportional-Integral-Derivative) control for precise positioning and speed regulation. With the integration of an AS5600 magnetic encoder and a range of electronic components such as the Arduino Mega, L298N motor drivers, and A4988 stepper driver, this project demonstrates a comprehensive approach to achieving accurate and controlled movements. The wirings are a bit messy due to the fact that the deadline was changed by my professor.


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

# Robotic Arm Design
The 4DOF (Degrees of Freedom) robotic arm consists of four interconnected joints that enable it to perform a wide range of motions. Each joint is equipped with a DC motor as the actuator, providing the necessary torque to move the arm segments. To accurately determine the position of each joint, an AS5600 magnetic encoder is employed, allowing for precise feedback and control.

## Control System:
The control system of the robotic arm utilizes PID control to ensure precise and stable positioning of each joint. PID control is a feedback mechanism that adjusts the motor's input based on the difference between the desired position and the measured position. This control loop allows for real-time adjustments, minimizing position errors and enhancing the arm's overall accuracy.

## Electronics and Components:
The core components of the project include an Arduino Mega, two L298N motor drivers, and an A4988 stepper driver. The Arduino Mega serves as the main controller, responsible for executing the control algorithms and coordinating the movements of the robotic arm. The L298N motor drivers are employed to drive the DC motors, providing sufficient power and control for their operation. Additionally, the A4988 stepper driver is utilized for accurate control of the AS5600 magnetic encoder, enabling precise position sensing.

## Project Goals:
The primary objective of this project is to develop an articulated robotic arm capable of accurately and smoothly performing various tasks. The integration of PID control ensures that the arm can position itself precisely and efficiently, while the use of DC motors offers the necessary strength and dexterity for a range of applications. By utilizing the AS5600 magnetic encoder, the arm can reliably sense its joint positions, facilitating enhanced control and flexibility.


