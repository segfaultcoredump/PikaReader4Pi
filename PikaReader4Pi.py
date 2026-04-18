#!/usr/bin/env python3
#
# Copyright (C) 2026 John Garner <segfaultcoredump@gmail.com>
#
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program.  If not, see <http://www.gnu.org/licenses/>.
#

########
# Modify before using!
# Update the GPIO Assignments section below
# Update the command to start PikaReader
#

import socket
import asyncio
#from importlib.metadata import pass_none

import LCD2004
import time
import websockets
import json
from datetime import datetime
import os
import RPi.GPIO as GPIO
import threading 
import requests


###### 
# GPIO Assignments
BTN_START_READING = 16
BTN_TRIGGER = 18
LED_READING = 36 # GREEN

#####
# PikaReader Start command
PIKAREADER_START_CMD = "java -jar /opt/PikaReader/bin/PikaReader-0.6.jar > /tmp/pika.out &"


######
#
######



####
# Global Variables
reading_status = False
time_synced = 0
gps_lock = ''
last_chip_read = ''
read_count = 0

###
# GPIO Defaults and initial setup
GPIO.cleanup()
GPIO.setwarnings(False) # Ignore warning for now
GPIO.setmode(GPIO.BOARD) # Use physical pin numbering
GPIO.setup(LED_READING,GPIO.OUT) # set the reading led mode
GPIO.output(LED_READING,GPIO.LOW) # default it to off
GPIO.setup(BTN_START_READING, GPIO.IN, pull_up_down=GPIO.PUD_UP) # Set the start reading button to be an input button and set initial value to be pulled low (off)

lcd_lock = threading.Lock()

def set_reading_status(s):
   global reading_status
   global LED_READING

   if reading_status != s:
       reading_status = s
       blank_line = "                    "
       lcd_write(0,2,blank_line)
       lcd_write(0,3,blank_line)

       if reading_status:
          GPIO.output(LED_READING,GPIO.HIGH)
       else:
          GPIO.output(LED_READING,GPIO.LOW)

def lcd_write(x,y,str):
   global lcd_lock
   with lcd_lock:
      LCD2004.write(x,y,str)

def timesync_check():
    global time_synced
    global gps_lock
    now = datetime.now()
    time = now.strftime("%m/%d/%Y  %H:%M:%S")
    lcd_write(0, 2, time.center(20))
    print(now.strftime("%m/%d/%Y  %H:%M:%S.%f"))

    ntp_stratum = int(os.popen("chronyc  tracking | grep Stratum | awk \'{print $3}\'").read().strip())

    if int(now.timestamp()) % 2 > 0: 
        gps_lock = int(os.popen("gpscsv -f mode -n 1 --header 0").read().strip())

    status = f"NTP:{ntp_stratum} GPS:{gps_lock}"
    lcd_write(0, 3, status.center(20))

    if ntp_stratum > 0: 
        time_synced +=1


def clear_lcd():
    LCD2004.clear()

def get_outbound_ip():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        # Doesn't actually send data; used to find the local interface
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
    except Exception:
        ip = "127.0.0.1"
    finally:
        s.close()
    return ip


def timecheck_abort(channel):
    global time_synced
    print("Start Button Pressed!")
    time_synced = 11

# The RSSI values are typically -100 (nothing) to -20 (really, really good)
# So we will just add 100 to deal with positive numbers from 0 -> 80
# With two charachters, there are 18 possible levels (from all blank to all filled)
#
# The rssi is the raw rssi
# the position is if this is the top char or the bottom one.
def pwr_map(rssi, position):
    rssi = rssi + 100

    if position == "T":
        rssi -= 40

    if rssi <= 0: return " "

    return chr(int(rssi // 5) - 1)

    match rssi:
        case x if x > 35:
            return "\x07"
        case x if x > 30:
            return "\x06"
        case x if x > 25:
            return "\x05"
        case x if x > 20:
            return "\x04"
        case x if x > 15:
            return "\x03"
        case x if x > 10:
            return "\x02"
        case x if x > 5:
            return "\x01"
        case x if x > 0:
            return "\x00"
        case _:
            return " "

###
# this is for the old school read power meter display (as seen in movies from the 80's)
# The LCD can store up to 8 custom symbols.
# There are 8 lines and we will use a space for 0 resulting in a total of 9 levels.
def setup_custom_lcd_chars():
    LCD2004.create_char(0, bytes([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F]))
    LCD2004.create_char(1, bytes([0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F, 0x1F]))
    LCD2004.create_char(2, bytes([0x00, 0x00, 0x00, 0x00, 0x00, 0x1F, 0x1F, 0x1F]))
    LCD2004.create_char(3, bytes([0x00, 0x00, 0x00, 0x00, 0x1F, 0x1F, 0x1F, 0x1F]))
    LCD2004.create_char(4, bytes([0x00, 0x00, 0x00, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F]))
    LCD2004.create_char(5, bytes([0x00, 0x00, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F]))
    LCD2004.create_char(6, bytes([0x00, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F]))
    LCD2004.create_char(7, bytes([0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F]))


###
# Main Logic: 
# 
# Wait for a time sync
# Start the pikatimer java service
# Start the background thread to listen for button pushes
# Start the background thread to monitor the websocket for PikaReader
# Start the loop to update the time of day
###

###
# Step 0: housekeeping
#
LCD2004.init(0x27, 1)	# init(slave address, background light)
setup_custom_lcd_chars()

###
# Step 1: 
# Wait for a time sync (or a button push to skip that part)
GPIO.add_event_detect(BTN_START_READING,GPIO.RISING,callback=timecheck_abort,bouncetime=500)
lcd_write(0, 0, 'PikaReader'.center(20))
lcd_write(0,1, os.popen("cat /etc/timezone").read().strip().center(20))

try:
    time.sleep(1 - (time.time() % 1))
    while time_synced < 5:
        start_time = time.time(); 
        timesync_check()
        sleep_time = start_time + 1 - time.time()
        print(f"sleeping {sleep_time}")
        if sleep_time >0: 
            time.sleep(sleep_time)

except KeyboardInterrupt: 
    clear_lcd()
finally:
    clear_lcd()
    GPIO.remove_event_detect(BTN_START_READING)

###
# Step 2: 
# Start the PikaTimer java service
clear_lcd()
lcd_write(0,2,"Starting".center(20))
lcd_write(0,3,"PikaReader... ".center(20))
os.popen(PIKAREADER_START_CMD)
#print("Sleeping for 10 seconds")
#time.sleep(0)
#clear_lcd()

###
# Step 2:
# Setup button press handling
#

def reading_button_callback(channel):
    global reading_status
    global read_count
    global last_chip_read
    global lcd_lock

    print("Reading Button Pushed!")
   
    if reading_status:

        button_start_time = time.time()

        while GPIO.input(BTN_START_READING) == 0:
            pass

        button_time = time.time() - button_start_time

        print(f"Stop Button time {button_time}")

        if button_time >= 2:
            print("Stopping Readers...")
            with lcd_lock:
                LCD2004.write(0,2,"Stopping".center(20))
                LCD2004.write(0,3,"Readers... ".center(20))
                response = requests.get('http://localhost:8080/stop')
                print(response.text)
                time.sleep(2)

            read_count = 0
            last_chip_read = ""

    else:
        print("Starting Readers...")

        with lcd_lock:
            LCD2004.write(0,2,"Starting".center(20))
            LCD2004.write(0,3,"Readers... ".center(20))

            response = requests.get('http://localhost:8080/start')
            print(response.text)
            time.sleep(2)



GPIO.add_event_detect(BTN_START_READING,GPIO.FALLING,callback=reading_button_callback,bouncetime=500) # Setup event callback

###
# Step 3:
# Set background thread to monitor the websocket to PikaReader
# 
async def ws_event_monitor():
    global last_chip_read
    global read_count
    
    l_last_chip_read = ""
    l_read_count = 0

    uri = "ws://localhost:8080/events"
    unitID = "Pika01"
    while True:
        try:
            async with websockets.connect(uri) as websocket:
                print(f"Connected to {uri}")

                async for message in websocket:
                    start_time = time.time()

                    event = json.loads(message)
                    #print(f"Received: {message}")

                    # If we have a read event
                    if event["type"] == "READ":
                        last_chip_read = event["chip"] 
                        read_count += 1
                        #print(f"RC: {read_count} LC:{last_chip_read}")

                    # update the unitID
                    if event["type"] == "STATUS" and not event["reading"]:
                        #print("Status: Idle")

                        if reading_status:
                            set_reading_status(False)

                        if unitID != event["unitID"] or int(datetime.now().timestamp()) % 10 == 0:
                           unitID = event["unitID"]
                           lcd_write(0,0,unitID)
                        
                        # get the status for reader 1
                        r1 = event["readers"][0]
                        if r1["connected"]:
                            lcd_write(0,2,"R1 READY ")
                            ant="ANT: "
                            for p in r1["portStatus"]:
                                if r1["portStatus"][p] == "Disconnected":
                                    ant = ant + "x"
                                else: 
                                    ant = ant + "C"
                            lcd_write(9,2,ant)

                        else:
                            lcd_write(0,2,"R1 ERROR     ")

                        # get the status for reader 2
                        # (if it exists)
                        if len(event["readers"]) > 1: 
                            r2 = event["readers"][1]
                       
                            if r1["connected"]:
                                lcd_write(0,3,"R2 READY ")
                                ant="ANT: "
                                for p in r2["portStatus"]:
                                    if r2["portStatus"][p] == "Disconnected":
                                        ant = ant + "x"
                                    else: 
                                        ant = ant + "C"
                                lcd_write(9,3,ant)
                            else:
                                lcd_write(0,3,"R2 ERROR      ")
                        else: lcd_write(0,3,"                    ")

                    elif event["type"] == "STATUS" and event["reading"]:
                        #print("Status: Reading")
                        if not reading_status:
                            set_reading_status(True)

                        if unitID != event["unitID"] or int(datetime.now().timestamp()) % 10 == 0:
                           unitID = event["unitID"]
                           lcd_write(0,0,unitID)

                        # update the read count and last chip read
                        if l_last_chip_read != last_chip_read or l_read_count != read_count:
                            lcd_write(11,3,f"{last_chip_read:>9}")
                            lcd_write(15,2,f"{read_count:>5}")
                            l_last_chip_read = last_chip_read
                            l_read_count = read_count

                        # update the antenna power meters
                        pwr1 = ""
                        pwr2 = ""

                        if len(event["readers"] )> 0:
                            r1 = event["readers"][0]
                            r1_name = r1["name"]

                            pwr1 = pwr1 + "R"
                            pwr2 = pwr2 + "1"
                            r1_stats = event["readerPortStats"][r1_name]
                            for i,s in enumerate(r1_stats["status"]):
                                if s == "Disconnected":
                                    pwr1 = pwr1 + " "
                                    pwr2 = pwr2 + "x"
                                else:
                                    pwr1 = pwr1 + pwr_map(r1_stats["readStrength"][i],"T")
                                    pwr2 = pwr2 + pwr_map(r1_stats["readStrength"][i],"B")

                        if len(event["readers"] )> 1:
                            r1 = event["readers"][1]
                            r1_name = r1["name"]

                            pwr1 = pwr1 + " R"
                            pwr2 = pwr2 + " 2"
                            r1_stats = event["readerPortStats"][r1_name]
                            for i,s in enumerate(r1_stats["status"]):
                                if s == "Disconnected":
                                    pwr1 = pwr1 + " "
                                    pwr2 = pwr2 + "x"
                                else:
                                    pwr1 = pwr1 + pwr_map(r1_stats["readStrength"][i],"T")
                                    pwr2 = pwr2 + pwr_map(r1_stats["readStrength"][i],"B")

                        lcd_write(0,2,pwr1)
                        lcd_write(0,3,pwr2)


 
                    elapsed_time = time.time() - start_time
                    #print(f"WS Processing Time: {elapsed_time}")

        #except (websockets.WebSocketException, ConnectionRefusedError):
        # There is just way too much stuff that can go wrong here....
        # So let's just sleep for a bit and retry.
        except:
            print("Connection lost. Reconnecting in 5 seconds...")
            await asyncio.sleep(5)

def websocket_monitor():
    asyncio.run(ws_event_monitor())

ws_thread=threading.Thread(target=websocket_monitor,daemon=True)
ws_thread.start()


###
# Step 4:
# Start loop to update the lcd clock and IP address
# 
my_ip = get_outbound_ip()
lcd_write(7,0,f"{my_ip:>13}")

try: 
  while True:
    start_time = time.time()
    now = datetime.now()
    lcd_write(12,1, now.strftime("%H:%M:%S"))

    if (int(now.timestamp()) % 10) == 0:
        #print("IP / chronyc refresh...")
        ip = get_outbound_ip()
        #if my_ip != ip:
        my_ip = ip
        lcd_write(7,0,f"{my_ip:>13}")

        ntp_stratum = int(os.popen("chronyc  tracking | grep Stratum | awk \'{print $3}\'").read().strip())
        if ntp_stratum == 1: 
            lcd_write(11,1,'G')
        else:
            lcd_write(11,1,'N')
            
    sleep_time = start_time + 1 - time.time()
    #print(f"sleeping {sleep_time}")
    if sleep_time > 0:
        time.sleep(sleep_time)    


except KeyboardInterrupt:
    print("Closing PikaReader...")

finally: 
    print("Cleanup GPIO...")
    os.popen("pkill java")
    GPIO.cleanup()
    clear_lcd()
    print("Goodbye!")
