THIS REPOSITORY CONTAINS A COMPLETE BLUEPRINT AND FIRMWARE OF DIY CAR SPEEDO GAUGE AKA "I SHOW SPEED". 

You see, local rental company is trying to permaban me from using car sharing service only because I love to speed, which is something that I will never give a middle finger.
Cars these days has decent performance, they easily hit 150kph with ease. and they claim that exceeding 150kph is a violation of their terms of service. 

So yeah.. I needed some device that I can easily carry, install, remove on a rental vehicle, and helps me avoid speeding. My initial thought was to make a device thatn warns speeding whenever I floor and sending her, with my leftover parts from one of my big main project.
However, the part that I use is clearly an overkill for this purpose. I mean, U-Blox M8 Series chip and STM32F4 for simple speed warning digital karen doesn't make any sense haha.
That's the start of the story.  Maybe I can integrate a whole lot of features that even a complete aftermarket HUD product doesn't provide? 

There are lots of similar product that falls into similar category - Tons of these are on sale from Aliexpress. They provide decent GPS sensitivity, some speed related feature. 
Pay more and you'll get a full pledged GIS based safe drive assistant system - clearly a waste of money, since at this point I would buy RAM mount instead..

And here I am - I am a well educated embedded engineer who can make things BETTER! 



YOU MAY REQUIRE 
- MAX7219 8-Digit 7 Segment Display Unit (Everyone can afford it, go search aliexpress if you want)
- STM32 Black Pill Board W/ STM32F411CEU6. You can use any board if it contains the same chip.
- NEO-M8N or compatible U-Blox UART GPS Breakout Board. (Chinese clone board with official chip will work)
- Passive Buzzer for sound feedback feature
- A tactile button switch. No H/W debouncing circuit required as the firmware handle it
- Dark transparent film. It works without this, but you may want to slap in on the LED segment display as these display have shitty contrast without a film
- 3D Printed case. PET-G filament is recommended as the unit is placed on the car dashboard so it should be UV and heat resistent.

CARGPSTOY_F411.ZIP File contains a full project directory for STM32CubeIDE HAL. Import and build the project, then upload. 
Included STL file is a 3d print model for casing. Print and slap everything in a case. 

FEATURES:
- SAT STATUS: A Number of visible satellite blinks at 5Hz when there is no fix / Used satellite number when gps fix is availiable
- Current / Avg / Maximum Speed
- GPS Heading / Grade / Altitude / Time / Distance
- Automatic brightness control based on sunset / sunrise time
- 1-Field / 2-Field / 0-100 Measurement mode
- System setup is stored in an internal flash w/ wear leveling + CRC
- Hardware (Flash, RAM) Test feature
- Single Button User Interface
- Warning if current driving speed exceeds 150 / 160 / 170km/h 

Build and Enjoy! 
Bored? Add features. Fix bugs. FIne tune everything. 
