# ROS Arduino Firmware

This repository contains the firmware for interfacing **Arduino** with **ROS (Robot Operating System)**. The firmware enables communication between Arduino and ROS, allowing sensors and actuators connected to Arduino to be controlled and monitored via ROS topics and messages.

---

## Features

- **ROS Serial Integration:**
  - Enables communication between Arduino and ROS using the `rosserial` protocol.
  - Supports publishing sensor data to ROS topics and subscribing to ROS topics for actuator control.

- **Sensor and Actuator Support:**
  - Compatible with various sensors (e.g., ultrasonic, IR, temperature) and actuators (e.g., motors, servos).
  - Easily extendable for custom hardware configurations.

- **Example Sketches:**
  - Includes example Arduino sketches for common use cases like reading sensor data and controlling motors.

---

## Repository Structure
```bash
ros_arduino_firmware/
├── examples/ # Example Arduino sketches
├── src/ # Source code for custom firmware
├── README.md # This file
└── LICENSE # License file
```

---

## Installation

1. **Clone the Repository:**
   ```bash
   git clone https://github.com/zeynelerenkinali/ros_arduino_firmware.git
   cd ros_arduino_firmware
   ```
2. Install ROS Serial:
      - Install the rosserial package in your ROS environment:
      ```bash
      sudo apt-get install ros-noetic-rosserial-arduino
      sudo apt-get install ros-noetic-rosserial
      ```
3. Upload Firmware to Arduino:
     - Open the desired example sketch from the examples/ folder in the Arduino IDE.
     - Upload the sketch to your Arduino board.

---

## Usage
1. Run ROS Serial Server:
   - Start the ROS serial server on your machine:
        ```bash
        rosrun rosserial_python serial_node.py _port:=/dev/ttyUSB0 _baud:=57600
        ```
        Replace /dev/ttyUSB0 with the correct port for your Arduino.
2. Interact with ROS Topics:
    - Use ROS tools like rostopic and rviz to monitor and control the Arduino-connected devices.

---

## Example
Reading Sensor Data
1. Upload the sensor_read_example.ino sketch to your Arduino.
2. Run the ROS serial server.
3. View the sensor data published to the ROS topic:
    ```bash
    rostopic echo /sensor_data
    ```
Controlling Actuators
1. Upload the actuator_control_example.ino sketch to your Arduino.
2. Run the ROS serial server.
3. Publish commands to the ROS topic to control actuators:
    ```bash
    rostopic pub /actuator_control std_msgs/String "data: 'ON'"
    ```

---

## License

This project is licensed under the [MIT License](https://opensource.org/licenses/MIT).
