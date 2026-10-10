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
* Install/Configure [PikaReader](https://github.com/PikaTimer/PikaReader) 1.0 or newer
* Install PikaReader4Pi
* Run the PikaReader4Pi.py script and then review the config file at ~/.PikaReader4Pi.cfg to update the GPIO and OS commands as needed
* Setup systemd to auto-start PikaReader4Pi upon boot

## Usage:
* Press reading button to start reader
* Press and hold the reading button for 3+ seconds to stop the reader
* Press the trigger button to create a 'trigger' in the chip feed

## Future Features:
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
* RPi.GPIO (via lgpio)
* threading 
* requests
* numpy
* tomlkit
* pathlib

