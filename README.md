THIS REPOSITORY CONTAINS A COMPLETE BLUEPRINT AND FIRMWARE OF DIY CAR SPEEDO GAUGE AKA "I SHOW SPEED". 

YOU MAY REQUIRE 
- MAX7219 8-Digit 7 Segment Display Unit (Everyone can afford it, go search aliexpress if you want)
- STM32 Black Pill Board W/ STM32F411CEU6. You can use any board if it contains the same chip.
- NEO-M8N or compatible U-Blox UART GPS Breakout Board. (Chinese clone board with official chip will work)
- Passive Buzzer for sound feedback feature
- A tactile button switch. No H/W debouncing circuit required as the firmware handle it
- Dark transparent film. It works without this, but you may want to slap in on the LED segment display as these display have shitty contrast without a film
- 3D Printed case. PET-G filament is recommended as the unit is placed on the car dashboard so it should be UV and heat resistent.

CARGPSTOY_F411.ZIP File contains a full project directory for STM32CubeIDE. Import and build the project. 
Included STL file is a 3d print model for casing. 

FEATURES:
- SAT STATUS: A Number of visible satellite blinks at 5Hz when there is no fix / Used satellite number when gps fix is availiable
- Current / Avg / Maximum Speed
- GPS Heading / Grade / Altitude / Time / Distance
- Automatic brightness control based on sunset / sunrise time
- 1-Field / 2-Field / 0-100 Measurement mode
- System setup is stored in an internal flash w/ wear leveling + CRC
- Hardware (Flash, RAM) Test feature
- Single Button User Interface

Build and Enjoy! 
Bored? Add 
