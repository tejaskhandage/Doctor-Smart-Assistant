**ESP8266 IoT Vital Signs Monitoring Project**

**Overview**
This project utilizes an ESP8266 microcontroller to create an IoT (Internet of Things) device capable of monitoring vital signs such as heart rate, blood oxygen saturation (SpO2), and body temperature. The device incorporates sensors, an LCD display, and motor-driven interaction to provide real-time data and user feedback.

**Features**
1.Vital Signs Monitoring: Utilizes a MAX30105 sensor for heart rate and SpO2 monitoring through photoplethysmography (PPG) techniques.
2.Body Temperature Sensing: Incorporates a DS18B20 digital temperature sensor for measuring body temperature.
3.LCD Display: Displays real-time vital sign data and user feedback on a 16x2 LCD display with I2C interface.
4.Motor-Driven Interaction: Provides user interaction through motor-driven actions, enhancing user experience and feedback.
5.Motion Sensor (Optional): Includes logic for motion detection using a passive infrared (PIR) sensor to trigger device operations.

**Setup Instructions**

**Hardware Setup:**
Connect the sensors, LCD display, and motors to the ESP8266 development board as per the schematic.

**Software Setup:**
Install the necessary libraries for the sensors and LCD display in your Arduino IDE or preferred development environment.
Upload the provided Arduino sketch (main.ino) to the ESP8266 development board.

**Usage:**
Power on the device and place your index finger on the MAX30105 sensor for vital sign monitoring.
Follow the on-screen instructions displayed on the LCD.
Optional: Enable motion sensor for automated device operations.
