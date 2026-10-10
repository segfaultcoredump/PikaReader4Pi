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
# After the 1st run, check the ~/.PikaReader4Pi.cfg file for the following:
#  - GPIO Assignments section below
#  - Command to start PikaReader
#  - Chronyc and gpscsv commands to snag ntp and gps status
#  - Battery Capacity
#
#  NOTE: Battery capacity % and remaining is based on a 7P LIPO
#  Modify the array in the estimate_remaining_capacity function
#  below if you are using something different.

######
# TODO:
# * Move trigger to unused gpio to free up 2nd serial port
# * verify trigger no longer locks up the lcd
# * verify stopping no longer locks up the lcd

# Implement the python logging facility: write to /tmp, keep past 10 runs.
# Document gpio port usage better (I2C, Serial #1, Serial #2, etc) in wiki
# Send GPS -> external status every minute if ntp shows we have a lock
# Create update script that syncs from github latest
# Figure out a better way to lock the LCD for "starting..." and "stopping..." messages

import socket
import asyncio
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
import tomlkit
from pathlib import Path

CONFIG = {}
CONFIG_FILE =  Path("~/.PikaReader4Pi.cfg").expanduser()



def create_default_config():
    doc = tomlkit.document()

    # Global comments
    doc.add(tomlkit.comment("PikaReader4Pi Settings"))

    # GPIO
    gpio = tomlkit.table()
    gpio.add(tomlkit.comment("GPIO Settings"))
    gpio.add(tomlkit.comment("List the Physical Port, not logical."))
    gpio.add("start_button", 40)
    gpio.add("trigger_button", 35)
    gpio.add("reading_led", 36)
    gpio.add("buzzer_pwm", 33)
    doc.add("GPIO", gpio)

    # PikaReader
    pikareader = tomlkit.table()
    pikareader.add(tomlkit.comment("PikaReader Settings"))
    pikareader.add("start_cmd", "/opt/PikaReader/bin/PikaReader.sh")
    pikareader.add("port", 8080)
    doc.add("PikaReader", pikareader)

    # Battery
    battery = tomlkit.table()
    battery.add(tomlkit.comment("Battery Settings"))
    battery.add(tomlkit.comment("Capacity in Wh"))
    battery.add("capacity", 160)
    battery.add("status_file", "/tmp/pikareader.status")
    doc.add("Battery", battery)

    # System Commands
    system = tomlkit.table()
    system.add(tomlkit.comment("OS Status Commands"))
    system.add("ntp_stratum", "chronyc  tracking | grep Stratum | awk \'{print $3}\'")
    system.add(tomlkit.comment("Set gps_status to \"echo 0\" if you don't have a gps"))
    system.add("gps_status", "gpscsv -f mode -n 1 --header 0")
    doc.add("System", system)

    # Buzzer Volume
    buzzer = tomlkit.table()
    buzzer.add(tomlkit.comment("Buzzer Settings"))
    buzzer.add("button_vol", 100)
    buzzer.add("read_vol", 100)
    doc.add("Buzzer", buzzer)

    return doc

def load_config():
    # Create a reference map of defaults to heal against
    defaults = create_default_config()
    is_updated = False

    # Setup default if it doesn't exist
    if not os.path.exists(CONFIG_FILE):
        with open(CONFIG_FILE, "w") as f:
            f.write(defaults.as_string())

    # Load and read the TOML file
    with open(CONFIG_FILE, "r") as f:
        config = tomlkit.parse(f.read())

        # Check root keys
        for key in defaults:
            if key not in config:
                config.add(key, defaults[key])
                is_updated = True
            # Check sub-tables (like server and database)
            elif isinstance(defaults[key], tomlkit.items.Table):
                for sub_key in defaults[key]:
                    if sub_key not in config[key]:
                        # Append the missing key and its default value into the sub-table
                        config[key].add(sub_key, defaults[key][sub_key])
                        is_updated = True

        # Scenario 3: If missing items were injected, save the file back without ruining comments
        if is_updated:
            print("Updating config file with missing values...")
            with open(CONFIG_FILE, "w", encoding="utf-8") as f:
                f.write(config.as_string())

    # Update our top-level dictionary in place
    CONFIG.update(config)

####
# Global Variables
reading_status = False
time_synced = 0
gps_lock = ''
last_chip_read = ''
read_count = 0

# --- INA237 CONFIGURATION ---
I2C_BUS = 1
DEVICE_ADDR = 0x40  # Replace with your specific INA237 I2C address
R_SHUNT = 0.015  # 0.015 Ohms (15 mOhms)

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
    # 7S LiPO pack voltage points (Sorted for np.interp)
    # Additional steps below 40% to capture the discharge knee accurately.
    # Uniform 5% steps from 40% to 100%.
    voltage_points = np.array([
        21.00,  # 0%   (3.00V / cell) - Safe floor
        23.10,  # 5%   (3.30V / cell)
        24.50,  # 10%  (3.50V / cell) - Drop-off zone start
        24.71,  # 12%  (3.53V / cell)
        24.92,  # 15%  (3.56V / cell)
        25.13,  # 18%  (3.59V / cell)
        25.27,  # 20%  (3.61V / cell)
        25.41,  # 23%  (3.63V / cell)
        25.55,  # 25%  (3.65V / cell)
        25.69,  # 28%  (3.67V / cell)
        25.76,  # 30%  (3.68V / cell)
        25.90,  # 35%  (3.70V / cell) - Drop-off zone end
        26.25,  # 40%  (3.75V / cell)
        26.39,  # 45%  (3.77V / cell)
        26.53,  # 50%  (3.79V / cell) - Storage voltage
        26.60,  # 55%  (3.80V / cell)
        26.74,  # 60%  (3.82V / cell)
        27.06,  # 65%  (3.87V / cell)
        27.37,  # 70%  (3.91V / cell)
        27.69,  # 75%  (3.96V / cell)
        28.00,  # 80%  (4.00V / cell)
        28.35,  # 85%  (4.05V / cell)
        28.70,  # 90%  (4.10V / cell)
        29.05,  # 95%  (4.15V / cell)
        29.40  # 100% (4.20V / cell) - Fully Charged
    ])

    # Corresponding remaining capacity percentages (0.0 to 1.0)
    capacity_points = np.array([
        0.0, 5.0, 10.0, 12.0, 15.0, 18.0, 20.0, 23.0, 25.0, 28.0, 30.0, 35.0,
        40.0, 45.0, 50.0, 55.0, 60.0, 65.0, 70.0, 75.0, 80.0, 85.0, 90.0, 95.0,
        100.0
    ])

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
    wh_remaining = (CONFIG["Battery"]["capacity"] * cap )/100
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

    # 2. Setup ADC Config Register for Stability
    # Bits 0 -> 2: AVG (64 samples) = 011
    # Bits 3 - 5: VTCT (1.052ms) = 0b101
    # Bits 6 -> 8: VSHCT (1.052ms)   = 0b101
    # bits 9 -> 11: VBUSCT (1.052ms)  = 0b101
    # Bits 12 -> 15: MODE (Continuous) = 0b1111
    # Total ADC_CFG Value = 0xFB6D
    # Default is 0xFCDH which sets AVG to 1 sample

    # Note, 4.120ms conversion times sometimes don't play nicely with
    # the 5v step down transformer, so it is best to just stick with
    # the default 1.052ms and bump the number of samples up.
    adc_config_val = 0xBB6E # bus and shunt only, 1.052ms conversion times, 128x sampling

    # deal with the big-endian vs little-endian issue
    swapped_value = ((adc_config_val & 0xFF) << 8) | ((adc_config_val >> 8) & 0xFF)
    bus.write_word_data(addr, REG_ADC_CONFIG, swapped_value)
    time.sleep(0.01)


def init_gpio():
    ###
    # GPIO Defaults and initial setup
    global BUZZER_PWM

    GPIO.cleanup()
    GPIO.setwarnings(False) # Ignore warning for now
    GPIO.setmode(GPIO.BOARD) # Use physical pin numbering
    GPIO.setup(CONFIG["GPIO"]["reading_led"],GPIO.OUT) # set the reading led mode
    GPIO.output(CONFIG["GPIO"]["reading_led"],GPIO.LOW) # default it to off
    GPIO.setup(CONFIG["GPIO"]["start_button"], GPIO.IN, pull_up_down=GPIO.PUD_UP) # Set the start reading button to be an input button and set initial value to be pulled low (off)
    GPIO.setup(CONFIG["GPIO"]["trigger_button"], GPIO.IN, pull_up_down=GPIO.PUD_UP) # Set the trigger button to be an input button and set initial value to be pulled low (off)
    GPIO.setup(CONFIG["GPIO"]["buzzer_pwm"], GPIO.OUT)
    BUZZER_PWM = GPIO.PWM(CONFIG["GPIO"]["buzzer_pwm"], 1000)

### Locks to keep us from walking over ourselves
lcd_lock = threading.Lock()
buzzer_lock = threading.Lock()


def do_beep(vol = -1):
    global buzzer_lock
    global BUZZER_PWM
    if vol == -1: vol = CONFIG["Buzzer"]["button_vol"]
    if buzzer_lock.acquire(blocking=False):
        BUZZER_PWM.start(vol)
        time.sleep(.1)
        BUZZER_PWM.stop()
        buzzer_lock.release()


def set_reading_status(s):
   global reading_status

   if reading_status != s:
       reading_status = s
       blank_line = " " * 20
       lcd_write(0,2,blank_line)
       lcd_write(0,3,blank_line)

       if reading_status:
          GPIO.output(CONFIG["GPIO"]["reading_led"],GPIO.HIGH)
       else:
          GPIO.output(CONFIG["GPIO"]["reading_led"],GPIO.LOW)


def lcd_write(x,y,str):
   global lcd_lock
   # with lcd_lock:
   #   LCD2004.write(x,y,str)

   acquired = lcd_lock.acquire(timeout=0.5)

   if acquired:
       try:
           LCD2004.write(x,y,str)
       finally:
           lcd_lock.release()
   else:
       print("lcd_write: Failed to acquire lock: operation timed out.")


def timesync_check():
    global time_synced
    global gps_lock
    now = datetime.now()
    time = now.strftime("%m/%d/%Y  %H:%M:%S")
    lcd_write(0, 2, time.center(20))
    print(now.strftime("%m/%d/%Y  %H:%M:%S.%f"))

    ntp_stratum = int(os.popen(CONFIG["System"]["ntp_stratum"]).read().strip())

    if int(now.timestamp()) % 2 > 0: 
        gps_lock = int(os.popen(CONFIG["System"]["gps_status"]).read().strip())

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
        ip = "No Network"
    finally:
        s.close()
    return ip


def battery_monitor_thread():
    bus = smbus2.SMBus(I2C_BUS)
    init_ina237(bus, DEVICE_ADDR)
    ALPHA = 0.1

    avg_vbus = None
    avg_vshunt = None
    avg_current = None
    avg_power = None

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

            # 3. Smooth the data
            if avg_vbus is None: avg_vbus = bus_voltage
            else: avg_vbus = (ALPHA * bus_voltage) + ((1-ALPHA) * avg_vbus)

            if avg_vshunt is None: avg_vshunt = shunt_voltage
            else: avg_vshunt = (ALPHA * shunt_voltage) + ((1-ALPHA) * avg_vshunt)

            if avg_current is None: avg_current = current
            else: avg_current = (ALPHA * current) + ((1-ALPHA) * avg_current)

            if avg_power is None: avg_power = power
            else: avg_power = (ALPHA * power) + ((1-ALPHA) * avg_power)

            cap = estimate_remaining_capacity(avg_vbus)
            eta = estimate_remaining_time(cap,avg_power)

            epoch = current_seconds = round(time.time())
            sec = epoch % 10

            if sec < 2: lcd_write(0,1, f"ETA:{eta}")
            else: lcd_write(0,1, f"BAT:{cap:>3}%")

            batt_status = {"BATTERY":cap,"RUNTIME_REMAINING":eta}
            with open(CONFIG["Battery"]["status_file"], "w", encoding="utf-8") as file:
                json.dump(batt_status, file)

            time.sleep(2)

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
    global lcd_lock
    do_beep()
    print("Trigger Button Pressed!")
    response = requests.get(f"http://localhost:{CONFIG["PikaReader"]["port"]}/trigger")
    print(response.text)
    event = json.loads(response.text)
    timestamp = event["timestamp"].split()
    with lcd_lock:
        LCD2004.write(0, 2, "Trigger Pressed".center(20))
        LCD2004.write(0, 3, timestamp[1].center(20))
        time.sleep(2)
        LCD2004.write(0, 2, " ".center(20))
        LCD2004.write(0, 3, " ".center(20))

def reading_button_callback(channel):
    global reading_status
    global read_count
    global last_chip_read
    global lcd_lock

    print("Reading Button Pushed!")

    if reading_status:

        button_start_time = time.time()
        while GPIO.input(CONFIG["GPIO"]["start_button"]) == 0:
            pass
        button_time = time.time() - button_start_time

        print(f"Stop Button time {button_time}")

        if button_time >= 2:
            do_beep()
            print("Stopping Readers...")
            with lcd_lock:
                LCD2004.write(0, 2, "Stopping".center(20))
                LCD2004.write(0, 3, "Readers...".center(20))
                response = requests.get(f"http://localhost:{CONFIG["PikaReader"]["port"]}/stop")
                print(response.text)
                time.sleep(2)
                LCD2004.write(0, 2, " ".center(20))
                LCD2004.write(0, 3, " ".center(20))

            read_count = 0
            last_chip_read = ""

    else:
        # Start if the button is held down for more than 0.1 seconds
        # prevents random bumps from starting the unit
        button_start_time = time.time()

        while GPIO.input(CONFIG["GPIO"]["start_button"]) == 0:
            pass

        button_time = time.time() - button_start_time

        if button_time >= 0.1:
            do_beep()
            print("Starting Readers...")
            with lcd_lock:
                LCD2004.write(0, 2, "Starting".center(20))
                LCD2004.write(0, 3, "Readers...".center(20))

                response = requests.get(f"http://localhost:{CONFIG["PikaReader"]["port"]}/start")
                print(response.text)
                time.sleep(2)
                LCD2004.write(0, 2, " ".center(20))
                LCD2004.write(0, 3, " ".center(20))


async def ws_status_monitor():
    global last_chip_read
    global read_count

    l_last_chip_read = ""
    l_read_count = -1

    uri = f"ws://localhost:{CONFIG["PikaReader"]["port"]}/events"
    unitID = "PikaR"
    while True:
        try:
            async with websockets.connect(uri) as websocket:
                print(f"Connected to {uri}")

                async for message in websocket:
                    start_time = time.time()

                    event = json.loads(message)
                    # print(f"Received: {message}")

                    if event["type"] == "READ": asyncio.create_task(asyncio.to_thread(do_beep,CONFIG["Buzzer"]["read_vol"]))

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
                                    line = f"R{i + 1} Connecting..."

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
                        if read_count == 0: last_chip_read = "No Reads"
                        if l_last_chip_read != last_chip_read or l_read_count != read_count:
                            if read_count > 999999: read_count = " 100K+"

                            lcd_write(12, 2, f"R#{read_count:>6}")
                            lcd_write(12, 3, f"{last_chip_read:>8.8}")
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
    asyncio.run(ws_status_monitor())



if __name__ == "__main__":
    ###
    # Main Logic:
    #
    # Load config file
    # Wait for a time sync
    # Start the pikatimer java service
    # Start the background thread to listen for button pushes
    # Start the background thread to monitor the websocket for PikaReader
    # Start the loop to update the time of day
    ###

    ###
    # Step 0: housekeeping
    #
    load_config()
    LCD2004.init(0x27, 1)	# init(slave address, background light)
    setup_custom_lcd_chars()

    init_gpio()

    ###
    # Step 1:
    # Wait for a time sync (or a button push to skip that part)
    GPIO.add_event_detect(CONFIG["GPIO"]["trigger_button"],GPIO.RISING,callback=timecheck_abort,bouncetime=500)
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
        GPIO.remove_event_detect(CONFIG["GPIO"]["trigger_button"])

    ###
    # Step 2:
    # Start the PikaTimer java service
    clear_lcd()
    lcd_write(0,1,"Starting".center(20))
    lcd_write(0,2,"PikaReader... ".center(20))
    os.popen(CONFIG["PikaReader"]["start_cmd"])
    print("Started PikaReader!")
    time.sleep(2)
    clear_lcd()

    ###
    # Step 3:
    # Button press handling
    #
    GPIO.remove_event_detect(CONFIG["GPIO"]["trigger_button"])
    GPIO.add_event_detect(CONFIG["GPIO"]["start_button"],GPIO.RISING,callback=reading_button_callback,bouncetime=500)
    GPIO.add_event_detect(CONFIG["GPIO"]["trigger_button"],GPIO.RISING,callback=trigger_button_callback,bouncetime=500)
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
    # Start a background thread to monitor the battery level
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

            ntp_stratum = int(os.popen(CONFIG["System"]["ntp_stratum"]).read().strip())
            match ntp_stratum:
                case 1:
                    lcd_write(11, 1, 'G')
                case 0: # no real sync
                    lcd_write(11,1,'X')
                case 8: # local clock
                    lcd_write(11,1,'L')
                case _: # we have a sync from a remote ntp server
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
        time.sleep(1)
        clear_lcd()
        lcd_write(0,1,"Goodbye!".center(20))
        print("Goodbye!")
