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
3. Write/find a library for the NRF24L01 transceiver for ESP32.
3.1 Test that registers is properly read and written to.(ESP32) (This took way too many hours to get working) 
3.2 Test that the SPI transmitting part to the LRF24L01 is working properly (ESP32)
3.3 Rewrite the library to work with a AVR MCU.
3.4 Test that i can read/write to registers for an AVR MCU.
3.5 Test that the SPI transmitting part to the LRF24L01 is working properly for the AVR MCU.
4. Test that I can send a message from TX to RX MCUs using the NRF24L01 transceiver.
4.1 Message received sometimes on startup. Try to make it work everytime and as expected.
5. Properly send joystick input from TX to RX using NRF24L01 transceiver, RX then move a servo as intended.
5.1 Create a voltage divider with resistors so that i can read battery voltage into the AVR MCU. Battery = 3cell LIPO ~9.9V = Empty, ~12.8V = full.
5.2 Send data from AVR to ESP32 with battery voltage.

TODO:


5.3 Display voltage on LCD screen.
6. From the ESP32 test ESC and make it work with the motor.
7. Send joystick input from TX to RX for ESC control.
8. Create custom circuitboard for transmitter for all pieces like joysticks, MCU, on/off button, switches etc.
9. Create additional functions like a arming function, trims etc.
10. (OPTIONAL) GPS functionality.

PROBLEMS:

If the joystick y axis is pulled to the top reading 4095, x axis also reads 4095?? Should be ~2650. SOLVED: joystick voltage should not be 5V but use the 3.3v instead as the ADC can not read up to 5V.

If the joystick x axis is pulled to the right reading 4095, y axis also reads 4095?? Should be ~2650.SOLVED: joystick voltage should not be 5V but use the 3.3v instead as the ADC can not read up to 5V.

Learnings:
Clock frequenzy on the SPI communication needs to be set to the same value for both the Atmega328p and ESP32.

A big problem that I did not know about is that the NRF24L01 will always send back the FIFO_STATUS register first on every new command from the SPI transmitted to it. When the FIFO_STATUS is empty and it will be return a decimal value of 14. It took many hours to find out this.
