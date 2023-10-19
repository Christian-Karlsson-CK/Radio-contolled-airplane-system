# Radio controlled airplane system
Embedded C

<img src="https://github.com/Christian-Karlsson-CK/Radio-controlled-airplane-system/assets/106676664/3f31e514-5203-45c5-8047-099c538987e7" width="500" height="500">  

**Introduction**  
The following readme file outlines my approach to testing, learning, and developing a fully functional radio controlled airplane system.  
I will employ the ESP32 as the transmitter (TX) on the controller and the Elegoo UNO R3 as the receiver (RX) on the airplane, utilizing a pair of NRF24L01 transceivers for radio communication.  
In the current build the system is fully functional with the communication between the controller and airplane working in both directions. This enables both precise control over the airplanes control surfaces as well as enabling realtime information from the airplane to be displayed on the controllers LCD screen such as the current voltage of the battery, altitude, heading, speed, visible GPS satellites, longitude and latitude and more.  

In the project, I'm currently working on a RTH(Return-To-Home) function. This function will autonomously steer the airplane back home to you. Finding the bearing to home is implemented and continously updated as well as most of the autonomous steering logic. The last step is to add a gyroscope so that we can accurately control the airplanes roll, pitch and yaw axis as to not overrotate on any given axis.  


**Electronic Components:**  
The list of components have grown quite a lot over the course of the project. Bellow is a list of the most important components.  

<img src="https://github.com/Christian-Karlsson-CK/Radio-controlled-airplane-system/assets/106676664/81dbc280-9ac2-4534-98b2-617f00ca8722" width="400" height="400">  


**On the airplane:**  
Elegoo UNO R3 (microcontroller)  
1x NRF24L01+ Transceiver  
1x BMP-280 (airpressure & temperature sensor)  
1x GY-271 (3-axis magnetometer compass)  
1x GY-NEO6MV2 with a ceramic antenna (GPS)  
1x ESC  (Electronic speed controller, used to control motor)  
1x Electric motor  
2x Servo Ailerons  
1x Servo Elevator  
1x Servo Rudder  


**On the controller:**  
ESP32-WROOM-32 DevkitC v4  (microcontroller)  
1x NRF24L01+ Transceiver  
1x 1602 LCD display  
2x Joysticks  
2x 3-way switches  
2x Potentiometers  
1x Power module with On/Off Button for the battery  


**In the early stages:**  
I began by testing the functionality of joysticks and servos on the Elegoo UNO R3 to ensure they operated as intended. As I transitioned to working with the ESP32, I encountered the need to adapt to a different programming environment, as the ESP32 doesn't utilize AVR like the UNO R3. Instead, it relies on the Espressif IDF framework. To gain familiarity with the ESP32's pin programming, I initiated my learning journey by creating a simple LED blink program.


**Planing**  
I've drafted an initial TODO list for my workflow, with the understanding that it will evolve as I gain a deeper understanding of the project requirements and potentially add more functionality. The list is divided into two sections: one for completed tasks (DONE) and another for tasks yet to be tackled (TODO).


**DONE:**  
1. Successfully configured and tested the joystick's analog input with the ESP32 (for testing purposes).  
2. ~~Test servo on ESP32.~~ I initially attempted to test the servo on the ESP32, but it proved to be more challenging than expected and ultimately deemed unnecessary, so I skipped this step.  
3. Wrote a library for the NRF24L01 transceiver for ESP32.  
3.1 Invested significant time in ensuring proper register read/write functionality (ESP32).  
3.2 Verified the SPI transmission to the NRF24L01 was functioning correctly (ESP32).  
3.3 Adapted the NRF24L01 library for compatibility with an AVR MCU.  
3.4 Confirmed register read/write operations with the AVR MCU.  
3.5 Verified proper SPI transmission to the NRF24L01 for the AVR MCU.  
4. Established communication between the TX and RX MCUs using the NRF24L01 transceiver.  
4.1 Addressed sporadic message reception on RX to ensure consistent and expected performance.    
5. Properly sent joystick input from TX to RX using NRF24L01 transceiver, RX then move a servo as intended.  
5.1 Created a voltage divider with resistors to monitor battery voltage. (3-cell LIPO: ~9.9V = empty, ~12.8V = full)  
5.2 Transmitted battery voltage data from AVR to ESP32.  
5.3 Display voltage on LCD screen on from the ESP32. (This included changing a LCD lib from AVR to work in IDF environment)  
5.4 Enable TX and RX to switch between TX and RX mode to both send and receive data in a controlled manner. (mostly controlled, small servo fluctuations)  
6. Using the ESP32 test that ESC and motor works as intended.  
7. Transmitted joystick input from TX to RX for ESC control.  
7.1 Connected all buttons, switches, joysticks and everything else needed to test that the whole system functions as expected.  
7.2 Test complete system on an airplane and confirmed that everything works as expected. (All analog control data is received properly to RX but, atmega328p seems not be able to control more than 2 servo/ESC) I will continue to next steps and decide if i need another MCU at a later time.  
7.3 Wrote necessary LCD functions for esp32 and confirmed that they do work properly.  
8. Designed a custom circuit board for the transmitter, accommodating components such as joysticks, MCU, on/off button, switches, and more.  
8.1 Added an GY-BMP280 module that can measure temperature and atmospheric pressure. With the atomspheric pressure I can calculate the airplanes altitude and display on the LCD-display on the remote control.  
8.2 Added an GY-271 Compass module and read the bearing value.  
9. Added an GY-NEOM6MV2 GPS module on the airplane. Latitude, longitude, altitude, ground speed, fix and satellite count is being sent to the TX radio controller.    
10. Implemented an buzzer function.  
11. Implemented a Return-To-Home function. (For proper airplane steering, more sensors are need along with proper tuning)  


**TODO:**   
 12. Add a gyroscope.  
 13. Add a Airspeed sensor.  
 14. Upgrade from Atmega328p on the airplane to an MCU that has atleast 3 PWM outputs, OR add a PWM module.  
 15. Use built in WIFI module on ESP32 to send flight coordinates to a Cloudservice.  
 16. Display flightroutes on google maps or similar via an app or webpage.  
 17. Have the airplane fly a preconfigured route from google maps or similar.  



**Result and reflections:**  
So far I'm very satisfied with how the project has turned out. From not being sure that if my initial goal just to control the airplane with a controller was doable, to going far beyond what i thought I could accomplish.
I have learned so many valuable things from this project and it has been so much fun. At times it has gone smooth and progress have been going in a steady pace. But also many times have i come to a complete stop. Trying to find a out why something does not work. The most challenging part was to write the library for the radio modules, on one side the communication protocol had to be written on the IDF framework and on the other end it was AVR framework. Once i finally could communicate through SPI from both frameworks to the radio modules, I still had one hard nut to crack. That was to make the radio modules properly send and receive a radio transmission. With several important registers on both sides needing to be configured correctly with just 1 bit wrong could result that the whole communication would not work. Also not being able to debug and not even knowing which side or maybe even both was configured wrong made it extremely hard and time comsuming to set everything correct.  

**Images and videos:**  
Bellow I would like to present a few images of the airplane as well as the controller.
At the very bottom there is also few demonstration videos.

<img src="https://github.com/Christian-Karlsson-CK/Radio-controlled-airplane-system/assets/106676664/de396d31-eda9-464e-b436-7c4fca99fbd5" width=50% height=50%>  

<img src="https://github.com/Christian-Karlsson-CK/Radio-controlled-airplane-system/assets/106676664/364237dd-cc9a-4c2f-98fc-005221319f82" width=50% height=50%>  

<img src="https://github.com/Christian-Karlsson-CK/Radio-controlled-airplane-system/assets/106676664/7908696c-894b-46ea-b56c-403a6aa9a52f" width=50% height=50%>  

<img src="https://github.com/Christian-Karlsson-CK/Radio-controlled-airplane-system/assets/106676664/2a552810-6807-40c5-a5ee-77b1afed4c69" width=50% height=50%>  

<img src="https://github.com/Christian-Karlsson-CK/Radio-controlled-airplane-system/assets/106676664/2f7a56d4-3a23-4601-a90b-aca4b5db819c" width=50% height=50%>  

<img src="https://github.com/Christian-Karlsson-CK/Radio-controlled-airplane-system/assets/106676664/1dee5966-1cbb-4364-a7d8-595a3ea70df9" width=50% height=50%>  

<img src="https://github.com/Christian-Karlsson-CK/Radio-controlled-airplane-system/assets/106676664/acb2fb59-8a79-4707-b5f8-2bbc9c9f1501" width=50% height=50%>  

<img src="https://github.com/Christian-Karlsson-CK/Radio-controlled-airplane-system/assets/106676664/fb5a15d4-4220-4c2a-a64d-4a211aa24efe" width=50% height=50%>  


 

 

 


The radio communication is working:  
[![Watch the video](https://img.youtube.com/vi/rBgySWtwwQA/hqdefault.jpg)](https://www.youtube.com/embed/rBgySWtwwQA)






