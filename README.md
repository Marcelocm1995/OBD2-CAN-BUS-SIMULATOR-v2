# OBD2-CAN-Simulator-v2
This project is a new version of OBD2 CAN Simulator that simulates CAN BUS of OBD2 port, helping to develop OBD2 CAN BUS projects.

# What Changes
Now this project have a GUI that user can especify sensor values (last version works with random values).
Multiple PID request in one message, like a real ECU!

# How It Works:
This project get the second byte of data of received CAN message and determine the size of reponse, then return values(selected on GUI) on fourth-seventh data of CAN message.
To read OBD protocol our device need to have an ID = 0x7DF, ECM will return with ID = 0x7E8. Knowing this, we configure CAN filter to receive only ID = 0x7E8, with this filter any other ID will be ignored.

# Software:
It uses cubeMX for generate a pre-code. Compiler Keil uVision, but you can easily migrate to another suported compiler by cubeMX, like IAR.
For GUI it uses Nextion Editor.
NOTE: in folder "CAN_simulator" have an archive "Drivers.zip", you need to extract.

# Hardware:
This hardware uses a STM32F103C8T6 (bluepill board) with a CAN transceiver (MCP2251).
MCP2551 is 5V logic, because of that we need to use PB9 and PB8 pins (this pins are 5V tolerant). Pins PA11 and PA12 aren't 5V tolerant.
The display for GUI is a Nextion NX4024T032_11 connected on USART3 (PB10 and PB11).

![alt text for screen readers](/Images/2QnrXvz.jpg "Text to show on mouseover")

# CAN SPEED
Clock was configured at 72MHz, with this clock we set: 
* CAN preescaler = 9
* Time Quantum in Bit Segment 1 = 3TQ
* Time Quantum in Bit Segment 2 = 4TQ

This configuration gives a CAN speed = 500kbps, 2uS for each bit (most commom on OBD2).

# Enclosure
I made a 3D printed enclosure for this project, STL's and STEP's are avaiable for download on this repository.
![alt text for screen readers](/Images/foto1.PNG "Text to show on mouseover")
![alt text for screen readers](/Images/foto2.PNG "Text to show on mouseover")

# Final project
[![IMAGE ALT TEXT HERE](https://img.youtube.com/vi/TzrFJ_mD2jE/0.jpg)](https://www.youtube.com/watch?v=TzrFJ_mD2jE)

https://www.youtube.com/watch?v=TzrFJ_mD2jE
