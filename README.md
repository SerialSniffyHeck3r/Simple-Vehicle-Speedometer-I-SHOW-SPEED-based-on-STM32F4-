
# 'I Show Speed' Portable Car Speedometer with Super High Sensitivity. 

![Video](images/gifgif.gif)

## 'I show Speed' 차량 대시보드용 속도 게이지: 한국어 설명

이 레포지토리는 회로도와 펌웨어, 케이스 STL 파일을 포함하고 있습니다. 

00년대 초반, GPS 네비게이션 제품이 비쌌던 그 시절 잠깐이나마 시장에 풀려 영광을 누릴 뻔 했던 GPS 안전운전 보조장치를 재해석한 차량용 속도계 프로젝트입니다.

- 방위, 거리, 고도, 경사도, 속도 지원
- 제로백 측정 기능 지원
- U-Blox 포맷 호환
- 단일 버튼 인터페이스
- GMT / 비프음 볼륨 / 화면 밝기 조절 설정 기능 내장
- 일출 일몰 계산하여 자동 밝기 적용하는 알고리즘 내장
- 플래시 메모리에 설정 값 저장
- 하드웨어 테스트 모드 

## Introduction

THIS REPOSITORY CONTAINS A COMPLETE BLUEPRINT AND FIRMWARE OF DIY CAR SPEEDO GAUGE AKA "I SHOW SPEED". 

You see, local rental company is trying to permaban me from using car sharing service only because I love to speed, which is something that I will never give a middle finger.
Cars these days has decent performance, they easily hit 150kph with ease. and they claim that exceeding 150kph is a violation of their terms of service. 

So yeah.. I needed some device that I can easily carry, install, remove on a rental vehicle, and helps me avoid speeding. 
My initial thought was to make a device that warns speeding whenever I floor and sending her, with my leftover parts from one of my big main project.
However, the part that I use is clearly an overkill for this purpose. 
I mean, U-Blox M8 Series chip and STM32F4 for simple speed warning digital karen doesn't make any sense haha.
That's the start of the story.  Maybe I can integrate a whole lot of features that even a complete aftermarket HUD product doesn't provide? 

There are lots of similar product that falls into similar category - Tons of these are on sale from Aliexpress. 
They provide decent GPS sensitivity, some speed related feature. 
Pay more and you'll get a full pledged GIS based safe drive assistant system - clearly a waste of money, since at this point I would buy RAM mount instead..

And here I am - I am a well educated embedded engineer who can make things BETTER! 

## Design Origin

Here in korea, Car GPS was quite expensive on early 00s. Dedicated vehicle GPS unit is barely affordable.
Tech-savvy users can carry their PDAs that runs Windows CE or Windows Mobile 2003 with external GPS Module. 
Some companies even provide GPS Software like 'I-Navi' for those. However, Early PDAs all shared a similar fatal flaw - SRAM Backup. 
Once battery ran out, every settings and user applications installed on the PDA is completely gone, including GPS software.

So, small companies back then brought a simple idea. Most people use their GPS Navigation to check the location of speed cameras and avoid speeding. 
Then, Why don't we make a product that does not have an expensive processor and display, and just warns speed camera and show only necessary infos?

Those units have 8-bit MCU with SirfStar GPS chip. Most of them equipped with 4 digit segment display and very simple voice alert system, and was affordable.
People can install those small units on their dashboard and those device helped them to drive safe... and of course avoiding speed tickets, well... by driving carefully especially in front of speed cameras.

So my design was based on these unit, but now with more sensitive and fast (2Hz) GPS, 8 digit display and extended features.

## About the name of this project

I taught English as a part time job. and one of my student loved a youtube channel called I Show Speed.
And yeah.. This product pretty much literally shows a speed. That's it. 

## YOU MAY REQUIRE 
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


## Cortex-M4, Isn't it overkill??

100% Agreed. STM32F4 is like a supercar for microproceor. Its like a Porsche when most commercially available cheapo 8-bit MCUs are a shitbox. 
BOM is a real issue when its in production, but since its a personal DIY project that people can download and make themselves, I valued more for ease of programming than an extreme cost cutting.

