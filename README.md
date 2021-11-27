# Smart Traffic System

Supervised by Dr. Mohamed Shalan at The American University in Cairo

- [Smart Traffic System](#smart-traffic-system)
  - [Group Members](#group-members)
  - [Github Repository](#github-repository)
  - [Motivation](#motivation)
  - [Project Idea](#project-idea)
  - [Updated Block Diagram](#updated-block-diagram)
  - [Project Components](#project-components)
  - [Status Update](#status-update)
  - [Currently Working On:](#currently-working-on)
  - [Hardware Components](#hardware-components)
  - [Software Components](#software-components)

## Group Members

Karim Gemiey\
Mohamed El Awadly\
Nouran Abdelkarim\
Omar Elewa

## Github Repository

[Repo](https://github.com/omarelewa/ES-Smart-Traffic/tree/master)

## Motivation

We are using the golden hour rule as our motivation for this project. The golden hour rule says that there is a golden hour between life and death. If you are critically injured you have less than 60 minutes to survive. You might not die right then but something has happened in your body that is irreparable.

## Project Idea

Many emergencies/services can easily get delayed during rush hours, and while some of these services can afford the latency of waiting in traffic, other emergencies cannot, and thus, our project is a smart traffic light control system that utilizes a GSM module to aid in the efficient handling of these emergencies/services. It can aid in proper scheduling of arrival and tracking the location of the dispatched car.

## Updated Block Diagram

![](https://github.com/omarelewa/ES-Smart-Traffic/blob/master/Screen%20Shot%202021-11-26%20at%2011.03.13%20PM.png)

- An ambulance which acts as the vehicle. Its coordinates are checked by the web server to see if it nears the traffic lights, and the relevant steps are performed accordingly.

- The web server which receives emergency signals and keeps checking the coordinates of the vehicle in order to change the traffic lights.

- The traffic lights.

## Project Components

1. Ambulance: Tracked using a GPS module and coordinates are communicated to the webserver using GSM module.
2. Asynchronous Web Server: Sends signal to microcontroller specifying change to occur.
3. Traffic Light: Responsible for the timely response to the signal provided by the webserver.

## Status Update

1. Setting up ESP32 as a web server:

   - ESP32 board works on Arduino IDE
   - It can be accessed locally by reading over WiFi and responding over WiFi
   - Keeps listening for the traffic emergency signal that will be sent over using GSM.
   - After receiving the emergency signal, a loop starts where the webserver keeps accessing the GPS coordinates of the vehicle.
   - When the vehicle is near the traffic lights, the webserver sends a signal over to the microcontroller to turn the traffic light green for a predefined amount of time.

2. Traffic lights logic:

   - We have implemented the logic of the traffic lights in Keil Vision.
   - Through the use of three external LEDs (green, yellow, and red) connected to STM, the relevant traffic light is enabled according to the logic.
   - We also used CubeMx for the configuration of UART and the GPIO pins.

3. Working with GPS Module

   - In order to implement a practical solution that can be implemented in real life scenarios, we decided to integrate GPS module.
   - The GPS module connects to a microcontroller using a UART serial connection
   - It sends data in NMEA-0183 messages format.
   - NMEA-0183 have 2 checksum bytes at the end of most messages, which is of huge importance in our application.
   - We researched how to navigate NMEA-0183 messages
   - NMEA-0183 messages let external devices use selected data collected or computed by the GNSS receiver.
   - Currently considering several NMEA sentences to process, in order to find which one suits our needs best.
   - $GPRMC (Global Positioning Recommended Minimum Coordinates) provides the time, date, latitude, longitude, altitude and estimated velocity.
   - $GPGGA sentence provides essential fix data which provide 3D location and accuracy data, UTC of position fix, Latitude, Direction of latitude, Longitude, Direction of longitude

4. Working with GSM/GPRS Module

- We bought 3 GSM/GPRS modules to be used for public internet access.
- Also we figured out the APN (Access Point Name) and credentials to access the internet through the GPRS module.
- Started experimenting with GSM/GPRS with the AT commands

## Currently Working On:

- Making the webserver asynchronous so it handles multiple requests simultaneously.
- Buying a hosting plan and domain name for the server to be accessed outside the local network.
- Integrating GPRS into the project for communication within public networks because we now only use within WiFi which has limited range.

## Hardware Components

- STM32L432KC Microcontroller
- GSM/GPRS: A6 Quad band
- ESP32
- 3 Vodafone SIM Cards (for communication within the network)
- GPS: Ublox NEO 6m SKU 165 [GPS](https://github.com/omarelewa/ES-Smart-Traffic/tree/master)
- 3 LEDs (green, yellow and red, one for each traffic light)

## Software Components

- Keil Vision - used for traffic lights logic implementation
- STMCube32MX - used for configuration of UART and GPIO pins in traffic lights implementation
- Tera Term
- Arduino IDE - used for web server implementation
