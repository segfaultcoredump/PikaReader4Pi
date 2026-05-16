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

######
# TODO:
# Show "0" and "No Reads" if we are in reading mode and don't have any reads yet

########
# Modify before using!
#  - GPIO Assignments section below
#  - Command to start PikaReader
#  - Chronyc and gpscsv commands to snag ntp and gps status
#  - Battery Capacity

###### 
# GPIO pin assignments
# These are PHYSICAL pin numbers, not logical!
BTN_START_READING = 40  # gpio 21
BTN_TRIGGER = 35  # gpio 19
LED_READING = 36  # gpio 27
BUZZER = 33 # gpio 13 / PWM1


#####
# OS Commands
# PikaReader Start command
# NTP Stratum
# GPS Lock

PIKAREADER_START_CMD = "java -jar /opt/PikaReader/bin/PikaReader-0.6.1.jar > /tmp/pika.out &"
NTP_STRATUM_CMD = "chronyc  tracking | grep Stratum | awk \'{print $3}\'"
GPS_STATUS_CMD = "gpscsv -f mode -n 1 --header 0" # set to "echo 0" if you don't have a gps

# Battery Size
BATTERY_CAP_WH = 160 # in Watt-Hours

###
# Beeper volume (0 for off, 100 for loud)
BEEPER_BUTTON_VOL = 75
BEEPER_READ_VOL = 40

# --- INA237 CONFIGURATION ---
I2C_BUS = 1
DEVICE_ADDR = 0x40  # Replace with your specific INA237 I2C address
R_SHUNT = 0.015  # 0.015 Ohms (15 mOhms)
WINDOW_SIZE = 10  # Average window length in seconds

#####
#
# End of user customizations
#
#####

import socket
import asyncio
from collections import deque
import numpy as np
import smbus2

import LCD2004
import time
import websockets
import json
from datetime import datetime
import os
import RPi.GPIO as GPIO
import threading
import requests


####
# Global Variables
reading_status = False
time_synced = 0
gps_lock = ''
last_chip_read = ''
read_count = 0

# --- INA237 REGISTER MAP ---
REG_CONFIG = 0x00
REG_ADC_CONFIG = 0x01
REG_VSHUNT = 0x04
REG_VBUS = 0x05

def estimate_remaining_capacity(voltage: float) -> float:
    """
    Calculates the approximate remaining capacity (%) of a 7S LiPo pack
    based on its resting voltage using linear interpolation.
    """
    # Reference data points from the 7S LiPo chart: (Voltage, Capacity %)
    voltage_points = [21.0, 25.2, 26.0, 26.6, 27.3, 29.4]
    capacity_points = [0.0, 10.0, 25.0, 50.0, 75.0, 100.0]

    # Clamp voltage within absolute safe physical limits
    if voltage >= 29.4:
        return 100
    if voltage <= 21.0:
        return 0.0

    # Linearly interpolate the capacity based on the reference points
    estimated_capacity = float(np.interp(voltage, voltage_points, capacity_points))

    #return round(estimated_capacity, 1)
    return round(estimated_capacity)

def estimate_remaining_time(cap: float, avg_current: float) -> str:
    wh_remaining = (BATTERY_CAP_WH * cap )/100
    hours_float = wh_remaining / avg_current
    hours = int(hours_float)
    minutes = int((hours_float - hours) * 60)

    # Format to HH:MM with leading zeros

    if hours>=10: time_string = ">10H"
    else: time_string = f"{hours:1d}:{minutes:02d}"

    return time_string


def read_word_ina237(bus, addr, reg):
    """Reads a 16-bit register from INA237 and fixes MSB/LSB ordering."""
    raw = bus.read_word_data(addr, reg)
    swapped = ((raw & 0xFF) << 8) | ((raw >> 8) & 0xFF)
    return swapped


def twos_complement(val, bits=16):
    """Converts raw unsigned binary to signed integer."""
    if (val & (1 << (bits - 1))) != 0:
        val = val - (1 << bits)
    return val


def init_ina237(bus, addr):
    """Initializes the INA237 with default continuous measurements."""
    bus.write_word_data(addr, REG_CONFIG, 0x0000)
    time.sleep(0.01)



###
# GPIO Defaults and initial setup
GPIO.cleanup()
GPIO.setwarnings(False) # Ignore warning for now
GPIO.setmode(GPIO.BOARD) # Use physical pin numbering
GPIO.setup(LED_READING,GPIO.OUT) # set the reading led mode
GPIO.output(LED_READING,GPIO.LOW) # default it to off
GPIO.setup(BTN_START_READING, GPIO.IN, pull_up_down=GPIO.PUD_UP) # Set the start reading button to be an input button and set initial value to be pulled low (off)
GPIO.setup(BTN_TRIGGER, GPIO.IN, pull_up_down=GPIO.PUD_UP) # Set the trigger button to be an input button and set initial value to be pulled low (off)
GPIO.setup(BUZZER, GPIO.OUT)
BUZZER_PWM = GPIO.PWM(BUZZER, 1000)

### Locks to keep us from walking over ourselves
lcd_lock = threading.Lock()
buzzer_lock = threading.Lock()


def do_beep(vol = BEEPER_BUTTON_VOL):
    global buzzer_lock
    if buzzer_lock.acquire(blocking=False):
        BUZZER_PWM.start(vol)
        time.sleep(.1)
        BUZZER_PWM.stop()
        buzzer_lock.release()


def set_reading_status(s):
   global reading_status
   global LED_READING

   if reading_status != s:
       reading_status = s
       blank_line = " " * 20
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

    ntp_stratum = int(os.popen(NTP_STRATUM_CMD).read().strip())

    if int(now.timestamp()) % 2 > 0: 
        gps_lock = int(os.popen(GPS_STATUS_CMD).read().strip())

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


def battery_monitor_thread():
    bus = smbus2.SMBus(I2C_BUS)
    init_ina237(bus, DEVICE_ADDR)

    # Deques automatically eject old values when maxlen is exceeded
    vbus_history = deque(maxlen=WINDOW_SIZE)
    vshunt_history = deque(maxlen=WINDOW_SIZE)
    current_history = deque(maxlen=WINDOW_SIZE)
    power_history = deque(maxlen=WINDOW_SIZE)

    try:
        while True:
            # 1. Read Raw Values from Sensor
            raw_vbus = read_word_ina237(bus, DEVICE_ADDR, REG_VBUS)
            vbus_signed = twos_complement(raw_vbus, 16)
            bus_voltage = vbus_signed * 0.003125  # 3.125 mV per LSB

            raw_vshunt = read_word_ina237(bus, DEVICE_ADDR, REG_VSHUNT)
            vshunt_signed = twos_complement(raw_vshunt, 16)
            shunt_voltage = vshunt_signed * 0.000005  # 5 uV per LSB

            # 2. Compute Instantaneous Stats
            current = shunt_voltage / R_SHUNT
            power = abs(current * bus_voltage)

            # 3. Append to History Buffers
            vbus_history.append(bus_voltage)
            vshunt_history.append(shunt_voltage)
            current_history.append(current)
            power_history.append(power)

            # 4. Calculate Averages
            avg_vbus = sum(vbus_history) / len(vbus_history)
            avg_vshunt = sum(vshunt_history) / len(vshunt_history)
            avg_current = sum(current_history) / len(current_history)
            avg_power = sum(power_history) / len(power_history)

            # 5. Print Rolling Averages
            # Shows a warning until the 5-second buffer fills up for the first time
            status = "" if len(vbus_history) == WINDOW_SIZE else f" (Buffering: {len(vbus_history)}/{WINDOW_SIZE}s)"

            cap = estimate_remaining_capacity(avg_vbus)
            eta = estimate_remaining_time(cap,avg_power)

            epoch = current_seconds = round(time.time())
            sec = epoch % 10

            if sec < 2: lcd_write(0,1, f"ETA:{eta}")
            else: lcd_write(0,1, f"BAT:{cap:>3}%")

            time.sleep(1)

    except KeyboardInterrupt:
        print("\nMeasurement stopped.")
    finally:
        bus.close()


def timecheck_abort(channel):
    global time_synced
    print("Timecheck Abort Button Pressed!")
    time_synced = 11

# The RSSI values are typically -100 (nothing) to -20 (really, really good)
# So we will just add 100 to deal with positive numbers from 0 -> 80
# With two characters, there are 18 possible levels (from all blank to all filled)
#
# The rssi is the raw rssi
# the position is if this is the top char or the bottom one.
def pwr_map(rssi, position):
    rssi = rssi + 100

    if position == "T":
        rssi -= 40

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

def trigger_button_callback(channel):
    do_beep()
    print("Trigger Button Pressed!")
    response = requests.get('http://localhost:8080/trigger')
    print(response.text)
    event = json.loads(response.text)
    timestamp = event["timestamp"].split()
    with lcd_lock:
        LCD2004.write(0, 2, "Trigger Pressed".center(20))
        LCD2004.write(0, 3, timestamp[1].center(20))
        time.sleep(3)

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
            do_beep()
            print("Stopping Readers...")
            with lcd_lock:
                LCD2004.write(0, 2, "Stopping".center(20))
                LCD2004.write(0, 3, "Readers...".center(20))
                response = requests.get('http://localhost:8080/stop')
                print(response.text)
                time.sleep(2)

            read_count = 0
            last_chip_read = ""

    else:
        do_beep()
        print("Starting Readers...")

        with lcd_lock:
            LCD2004.write(0, 2, "Starting".center(20))
            LCD2004.write(0, 3, "Readers...".center(20))

            response = requests.get('http://localhost:8080/start')
            print(response.text)
            time.sleep(2)


async def ws_event_monitor():
    global last_chip_read
    global read_count

    l_last_chip_read = ""
    l_read_count = -1

    uri = "ws://localhost:8080/events"
    unitID = "Pika01"
    while True:
        try:
            async with websockets.connect(uri) as websocket:
                print(f"Connected to {uri}")

                async for message in websocket:
                    start_time = time.time()

                    event = json.loads(message)
                    # print(f"Received: {message}")

                    if event["type"] == "READ": do_beep(BEEPER_READ_VOL)

                    # update the unitID
                    if event["type"] == "STATUS" and not event["reading"]:
                        # print("Status: Idle")

                        if reading_status:
                            set_reading_status(False)

                        if unitID != event["unitID"] or int(datetime.now().timestamp()) % 10 == 0:
                            unitID = event["unitID"]
                            lcd_write(0, 0, unitID)

                        # get the status for readers
                        for i in range(2):
                            line = ""
                            if len(event["readers"]) > i:
                                r = event["readers"][i]
                                if r["connected"]:
                                    line = f"R{i + 1} READY ANT: "

                                    for p in r["portStatus"]:
                                        if r["portStatus"][p] == "Disconnected":
                                            line = line + "X"
                                        elif r["portStatus"][p] == "Disabled":
                                            line = line + "-"
                                        else:
                                            line = line + "C"
                                else:
                                    line = f"R{i + 1} ERROR"

                            lcd_write(0, i + 2, f"{line:20}")


                    elif event["type"] == "STATUS" and event["reading"]:
                        # print("Status: Reading")
                        if not reading_status:
                            set_reading_status(True)

                        if unitID != event["unitID"] or int(datetime.now().timestamp()) % 10 == 0:
                            unitID = event["unitID"]
                            lcd_write(0, 0, unitID)

                        # update the read count and last chip read
                        if "lastChipRead" in event: last_chip_read = event["lastChipRead"]
                        if "totalReads" in event: read_count = event["totalReads"]
                        if l_last_chip_read != last_chip_read or l_read_count != read_count:
                            if read_count > 99999: read_count = 99999

                            lcd_write(11, 3, f"{last_chip_read:>9.9}")
                            lcd_write(15, 2, f"{read_count:>5}")
                            l_last_chip_read = last_chip_read
                            l_read_count = read_count

                        # update the antenna power meters
                        pwr1 = ""
                        pwr2 = ""

                        for i in range(2):

                            if len(event["readers"]) > i:
                                r1 = event["readers"][i]
                                r1_name = r1["name"]

                                if i > 0:
                                    pwr1 = pwr1 + " "
                                    pwr2 = pwr2 + " "

                                pwr1 = pwr1 + "R"
                                pwr2 = pwr2 + f"{i + 1}"
                                r1_stats = event["readerPortStats"][r1_name]
                                for i, s in enumerate(r1_stats["status"]):
                                    if s == "Disconnected":
                                        pwr1 = pwr1 + " "
                                        pwr2 = pwr2 + "x"
                                    elif s == "Disabled":
                                        pwr1 = pwr1 + " "
                                        pwr2 = pwr2 + "-"
                                    else:
                                        pwr1 = pwr1 + pwr_map(r1_stats["readStrength"][i], "T")
                                        pwr2 = pwr2 + pwr_map(r1_stats["readStrength"][i], "B")

                        lcd_write(0, 2, f"{pwr1:11}")
                        lcd_write(0, 3, f"{pwr2:11}")

                    elapsed_time = time.time() - start_time
                    # print(f"WS Processing Time: {elapsed_time}")

        # except (websockets.WebSocketException, ConnectionRefusedError):
        # There is just way too much stuff that can go wrong here....
        # So let's just sleep for a bit and retry.
        except Exception as ex:
            print(f"Exception: {ex}")
            print("Connection lost. Reconnecting in 5 seconds...")
            await asyncio.sleep(5)


def websocket_monitor():
    asyncio.run(ws_event_monitor())


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
GPIO.add_event_detect(BTN_TRIGGER,GPIO.RISING,callback=timecheck_abort,bouncetime=500)
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
    GPIO.remove_event_detect(BTN_TRIGGER)

###
# Step 2: 
# Start the PikaTimer java service
clear_lcd()
lcd_write(0,1,"Starting".center(20))
lcd_write(0,2,"PikaReader... ".center(20))
os.popen(PIKAREADER_START_CMD)
print("Started PikaReader!")
time.sleep(2)
clear_lcd()

###
# Step 3:
# Button press handling
#
GPIO.remove_event_detect(BTN_TRIGGER)
GPIO.add_event_detect(BTN_START_READING,GPIO.RISING,callback=reading_button_callback,bouncetime=500)
GPIO.add_event_detect(BTN_TRIGGER,GPIO.RISING,callback=trigger_button_callback,bouncetime=500)
print("Added Reading Button Callback")

###
# Step 4:
# Start a background thread to connect to PikaReader's event websocket
# 
ws_thread=threading.Thread(target=websocket_monitor,daemon=True)
ws_thread.start()
print("Started WebSocket Monitor")

###
# Step 5:
# Start a background thread to connect to monitor the battery level
#
bat_thread=threading.Thread(target=battery_monitor_thread, daemon=True)
bat_thread.start()
print("Started Battery Monitor")

###
# Step 6:
# Start loop to update the lcd clock and IP address
# 
my_ip = get_outbound_ip()
lcd_write(7,0,f"{my_ip:>13}")

print("Starting Clock Updater...")
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
        if my_ip == "127.0.0.1": my_ip = "No Network"
        lcd_write(7,0,f"{my_ip:>13}")

        ntp_stratum = int(os.popen(NTP_STRATUM_CMD).read().strip())
        if ntp_stratum == 1: # we have a pps/gps timesync
            lcd_write(11,1,'g')
        elif ntp_stratum == 0: # no real sync
            lcd_write(11,1,'x')
        else: # we have a sync from a remote ntp server
            lcd_write(11,1,'n')
            
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
    time.sleep(1)
    clear_lcd()
    lcd_write(0,1,"Goodbye!".center(20))
    print("Goodbye!")
