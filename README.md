# Radio controlled airplane system
Embedded C

The following text outlines my approach to testing, learning, and developing a fully functional radio controlled RC airplane system.
I will employ the ESP32 as the transmitter (TX) and the Elegoo UNO R3 as the receiver (RX), utilizing a pair of NRF24L01 transceivers.

**Electronic Components:**

Elegoo UNO R3  
ESP32-WROOM-32 DevkitC v4  
ESC + Motor  
2x NRF24L01+ Transceivers  
2x Servo Ailerons  
1x Servo Elevator  
1x Servo Rudder  
2x Joysticks  
1x On/Off Switch/Button for Transmitter  

In the initial stages of my project, I began by testing the functionality of joysticks and servos on the Elegoo UNO R3 to ensure they operated as intended. As I transitioned to working with the ESP32, I encountered the need to adapt to a different programming environment, as the ESP32 doesn't utilize AVR like the UNO. Instead, it relies on the Espressif IDF framework. To gain familiarity with the ESP32's pin programming, I initiated my learning journey by creating a simple LED blink program.

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


**TODO:**   
10. Implement additional functions like a arming function, trims, buzzer etc.  
11. Implement a Return-To-Home function.  

**MAIN PROBLEMS:**  

Initially, when pulling the joystick's y-axis to the top, both the x and y-axis readings showed 1023 instead of the expected ~512. This issue was resolved by adjusting the joystick voltage from 5V to 3.3V since the ADC cannot read up to 5V.

Similarly, when pulling the joystick's x-axis to the right, both the x and y-axis readings displayed 1023 instead of the expected ~512. This was also addressed by using 3.3V for the joystick voltage.

The GPIO 13 on the ESP32 was initially JTAG configured, requiring a reset to function as a normal GPIO. Configuring it as an output and setting it to logical HIGH resulted in a reading of ~2.84 volts with a multimeter, causing issues with the LCD. This problem was only identified after extensive troubleshooting including the use of a multimeter, which included accidentally damaging one ESP32 module.

A significant challenge encountered was that the NRF24L01 always returns the FIFO_STATUS register first with every new command though the SPI. When the FIFO_STATUS is empty, it returns a decimal value of 14. Identifying and understanding this behavior required a substantial amount of time and effort.

There seem to be disturbances in certain bytes in the TxData[32] array when sending from Rc controller to the airplane.
