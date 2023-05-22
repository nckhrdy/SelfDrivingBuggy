# Roadtrip

Authors: Authors: Marybel Boujaoude, Hassan Hijazi, Nicholas Hardy, Riya Deokar

Date: 2023-04-04



### Summary

To meet the requirements of the quest, we will designed a platform for autonomous driving that incorporated cruise control (through proportional, integral, differential control) and collision avoidance capabilities (through the ultrasonic and lidar sensors). We used an ESP32 microcontroller to interface with the car electronics and sensors, and implement PID control for maintaining a fixed speed setpoint and controlling steering. We tested each subsystem as it is built to ensure that our solution works as intended when integrated together.

To control steering, we used feedback from side range sensors to maintain a distance of +/- 15cm from the track center. We also used a wheel speed sensor, specifically and optical encoder, to measure the car's speed and implement PID control to maintain a fixed speed setpoint in the range of 0.1 to 0.4 m/s after start and before stop.

To prevent collisions, we detected objects in front of the car using a front-facing ultrasonic sensor and implement an emergency stop. The collision sensor is triggered at a range less than or equal to 20cm, stopping the car before collision. 

To determine when the wall/object is too close to the side of the car, we included Lidar sensors on the left side of the car in order for the car to sense and auto steer to the right. 

To calculate, display, and utilize speed, we calculated the speed through using the optical encoder to find the time of a revolution. Then we used the circumfrence and arithmetic to return/display the speed to the console and alphanumeric display. The speed also contributes to the PID increasing or decreasing the speed. 

To ensure that our software satisfies the timing of our control algorithm, we accommodated the timing limits of sensors and actuators, set PID timing, and use timers or interrupts to ensure consistent timing. We will also prioritize collision detection as a critical activity. Finally, we issued start and stop instructions wirelessly through wireless control. 

### Supporting Artifacts
- https://drive.google.com/file/d/1qet2Beorp1TNlXkDEIk6q_9u5V8p38tI/view?usp=sharing
- (Video1: follows wall + turn then fails ) https://drive.google.com/file/d/1G9TEW_tw2v6cyPlIAQ0eAq4RobSjFZ3y/view?usp=share_link


### Self-Assessment 

| Objective Criterion | Rating | Max Value  | 
|---------------------------------------------|:-----------:|:---------:|
| Objective One | 1  |  1     | 
| Objective Two | 1 |  1     | 
| Objective Three | 1 |  1     | 
| Objective Four | 1 |  1     | 
| Objective Five | 1 |  1     | 
| Objective Six | 1 |  1     | 
| Objective Seven | 1 |  1     | 


### Solution Design

We began with prepping and calibrating the buggy, by converting the RC control of the buggy to ESP32 control by wiring up the ESP32 to the electronic speed control (ESC) and the steering servo using the instructions provided in the recipe for wiring the buggy. Before powering up the car, we made sure the wheels were elevated to prevent the car from launching off the table if my code was not well-designed. To calibrate the ESC, we reviewed the calibration recipe provided for the Buggy and adopted the sample code to calibrate the ESC. This was an important step as the speed controller needed to know the set points (PWM widths) for the limits of "Fast," "Stop," and "Fast Reverse" to function properly. Using our prior experience and code for controlling a servo, we wrote a program to test the steering by cycling the steering servo from center to left to right and back to center. This ensured that the steering mechanism was properly wired and responsive to our code. Similarly, we wrote a program to test the ESC, which slowly cycled the ESC from stop to forward, to stop, to backwards, and back to stop. This ensured that the ESC was properly wired and able to control the buggy's speed. 

To measure wheel speed, we used an optical encoder attached to one of the wheels on the crawler. We mounted an LED/photo-sensor pair on the chassis and connected it to the ESP32 microcontroller. To wire up the optical encoder to the ESP32, we used female-female wires and male-male wires to connect the optical encoder to the breadboard but also have it be extended enough to reach the encoder pattern on the wheel. Once positioned correctly, the optical encoder is a reflective sensor that senses changes in intensity at the sensor output per unit time. Although our pattern did not have dark enough values in order to sense the difference, to combat this we used a Sharpie to darken the printed template which solved our issue. When coding for the change in speed we made sure all 6 transitions of the pattern have been processed using a 12 cycle pattern and then calculated RPM and displayed the speed. To display speed information on an alpha display, we used the I2C for communicatio n as usual and we're able to used this display as a debugging tool.

Additionally we used this speed information to implement PID control for maintaining a fixed speed setpoint in the range of 0.1 to 0.4. To implement a PID loop with only proportional control, we wired up 3 LEDs as shown in the diagram provided. We attached two LIDAR-lite V2 range sensors and reused our range code. Then we used and added to the PID design pattern to implement a timing loop with a 100ms period, with the help of UART and GPIO functions.
On each cycle of the loop, we calculated  the error signal and evaluated it by checking through thes following cases:
- If the error < 0, then turn on the red LED
- If the error = 0, then turn on the green LED
- If the error > 0, then turn on the blue LED
We set the setpoint at a distance of 40cm, therefore the car should  maintain a distance of 40 cm from the object being detected by the LIDAR sensor and demonstrated by a green LED sensor. We also used an interrupt for the timer to ensure consistent timing. If we move above or below the setpoint, we demonstrate the feedback through LED sensors turning red or blue. 

The Buggy remote, which is connected to our IP address as shown in sketches below, sends a message to the ESP which controls the wireless starting and stopping of the car. We were able to implement this by using our knowledge of Transferring Data Using User Datagram Protocol (UDP). We brought up the corresponding client at the laptop and server at the ESP, where the host laptop sent data to the ESP. This allowed us to remotely control the Buggy, which was connected to our IP address (192.168.1.36:8081). The remote sent a message to the ESP, which controlled the wireless starting and stopping of the car.



### Sketches/Diagrams

<img width="1326" alt="Screen Shot 2023-04-04 at 10 22 40 PM" src="https://user-images.githubusercontent.com/47408944/229964746-dbacb46b-2e3b-47f5-937b-1440d4219a2e.png">

<img width="1291" alt="Screen Shot 2023-04-04 at 9 43 08 PM" src="https://user-images.githubusercontent.com/47408944/229959682-683632f9-5608-4bb9-ae26-9a886794aef4.png">

<img width="1437" alt="Screen Shot 2023-04-04 at 8 53 54 PM" src="https://user-images.githubusercontent.com/47408944/229954137-8398f7ca-f594-4f64-b8b9-26a32e6acfe6.png">



### Modules, Tools, Source Used Including Attribution
HUZZAH32 ESP32 Feather Board, Raspberry Pi, QRD1114 Optical Detector, IP Address and E1200 V2 Router and UDP, LED, LIDAR-lite V3, ESC, Servo, Buggy, Adafruit 14-Segment Alphanumeric LED

### References

Calibrating ESC and Steering Servo to Buggy
https://github.com/BU-EC444/01-EBook/blob/main/docs/briefs/recipes/recipe-esc-buggy.md
https://github.com/BU-EC444/01-EBook/blob/main/docs/briefs/recipes/recipe-buggy-interfacing.md

Servo
https://github.com/espressif/esp-idf/tree/master/examples/peripherals/mcpwm/mcpwm_servo_control

Motor Control Pulse Width Modulator (MCPWM) Functions 
https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/mcpwm.html

Pulse Counter and Wiring
https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/pcnt.html
https://learn.sparkfun.com/tutorials/qrd1114-optical-detector-hookup-guide#example-circuit

LIDAR-lite V3 Readdressing: Daniel Paganelli
https://github.com/BU-EC444/01-EBook/blob/main/docs/briefs/recipes/recipe-lidarlite.md

PID
https://github.com/BU-EC444/01-EBook/blob/main/docs/briefs/design-patterns/dp-pid.md
