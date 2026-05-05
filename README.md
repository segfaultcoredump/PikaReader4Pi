# PikaReader4Pi: Raspberry PI UI for PikaReader

PikaReader4Pi is a python based script that links the RasberryPi buttons and LCD with 
a local [PikaReader](https://github.com/PikaTimer/PikaReader) instance

## Current Feature Highlights:
* Momentary button to Start / Stop Reading 
* 20x4 LCD output for system and antenna power meter display
* LED showing reading status
* Display Support for up to two connected 4-port readers

## Installation and Usage: 
* Setup Raspberry Pi (Instructions TBD)
* Install [PikaReader](https://github.com/PikaTimer/PikaReader) 0.6 or newer
* Install PikaReader4Pi
* Update the PikaReader4Pi python script and update the GPIO and OS commands portion
* Setup systemd to auto-start PikaReader4Pi
* Press reading button to start reader
* Press and hold the reading button for 3+ seconds to stop the reader

## Future Features:
* Support for a momentary button for a Trigger
* Battery level via i2c voltage sensor (INA237?)
* Beep / Buzzer on read
* Guided Installation Script
* Wiki with build instructions 

# Required Python Libraries:
* socket
* asyncio
* time
* websockets 
* json
* datetime 
* os
* RPi.GPIO as GPIO
* threading 
* requests

