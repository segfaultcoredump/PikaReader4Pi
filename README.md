# PikaReader4Pi: Raspberry PI UI for PikaReader

PikaReader4Pi is a python based script that links the RasberryPi buttons and LCD with 
a local [PikaReader](https://github.com/PikaTimer/PikaReader) instance

## Current Feature Highlights:
* Momentary button to Start / Stop Reading 
* 20x4 LCD output for system and antenna power meter display
* LED showing reading status
* Display Support for up to two connected 4-port readers
* Trigger button support
* Active piezo buzzer for audible feedback

## Installation: 
* Setup Raspberry Pi (Instructions TBD)
* Install/Configure [PikaReader](https://github.com/PikaTimer/PikaReader) 0.7 or newer
* Install PikaReader4Pi
* Review the PikaReader4Pi python script and update the GPIO and OS commands portion
* Setup systemd to auto-start PikaReader4Pi

## Usage:
* Python Script should auto-start via systemd (See Installation)
* Press reading button to start reader
* Press and hold the reading button for 3+ seconds to stop the reader
* Press the trigger button to create a 'trigger' in the chip feed

## Future Features:
* Guided Installation Script
* Wiki with build instructions 

# Required Python Libraries:
* socket
* asyncio
* collections
* time
* websockets 
* json
* datetime 
* os
* RPi.GPIO (via lgpio)
* threading 
* requests
* numpy

