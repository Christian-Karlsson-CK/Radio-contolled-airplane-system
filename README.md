# Radio controlled airplane system
Embedded C

The following text is my process of how I test, learn and try to develop a fully functional radio controlled RC airplane system.
I'm going to use the ESP32 as the Transmitter(TX) and Elegoo UNO R3 as the Receiver(RX) using two NRF24L01 transceivers.

Following list is all the electronic components:

PARTS:
-1 Elegoo UNO R3
-1 ESP32-­WROOM-­32 devkitC v4
-1 ESC + motor
-2x NRF24L01 transceiver
-2x Servo aileron
-1 Servo elevator
-1 Servo rudder
-2x Joysticks
-1 On/Off switch/button for transmitter

A few learning steps I have taken is to test that I can make joystick and servo work as intended on the Elegoo UNO R3.
Then figuring out how I even get started on the programming of a ESP32 as it does not use AVR. I found that it uses the Espressif IDF framework instead. I created a simple led blink program to get knowledge of how to program the pins on the ESP32.

I have written a rough TODO list for the workflow I intend to follow, the list will be subject to change as I learn more of what I need to do to or if more functionality is added.
The list will be separated into 2 sections. A DONE section and a TODO section.

DONE:
1. Make the joystick work and get its analog input into the ESP32.(Testing purposes)

TODO:

2. Make the servo move as intended from the input from the joystick on the ESP32.(Testing purposes)
3. Write/find a library for the NRF24L01 transceiver.
4. Test that I can send a message from TX to RX MCUs using the NRF24L01 transceiver.
5. Properly send joystick input from TX to RX using NRF24L01 transceiver, RX then move a servo as intended.
6. From the ESP32 test ESC and make it work with the motor.
7. Send joystick input from TX to RX for ESC control.
8. Create custom circuitboard for transmitter for all pieces like joysticks, MCU, on/off button, switches etc.
9. Create additional functions like a arming function, trims etc.
10. (OPTIONAL) GPS functionality.

PROBLEMS:
If the joystick y axis is pulled to the top reading 4095, x axis also reads 4096?? Should be ~2650.
If the joystick x axis is pulled to the right reading 4095, y axis also reads 4096?? Should be ~2650.

