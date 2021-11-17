# -*- coding: utf-8 -*-
"""
TI CC2650 SensorTag
-------------------

Adapted by Ashwin from the following sources:
 - https://github.com/IanHarvey/bluepy/blob/a7f5db1a31dba50f77454e036b5ee05c3b7e2d6e/bluepy/sensortag.py
 - https://github.com/hbldh/bleak/blob/develop/examples/sensortag.py

"""
import asyncio
import platform
import struct
import bleak
import os
import signal
import sys
import json
from commons.commons import (
    BLE_NAME_LIST,
    BLE_ADDR_LIST,
    BLE_ADDR_TO_NAME,
    BLE_NAME_SENSOR_NECK,
    BLE_NAME_SENSOR_BACK_MID,
    BLE_NAME_SENSOR_BACK_LOW,
    MQTT_CLASSIFICATIONS,
    MODE_EXPORT_DATA, 
    MODE_RT_SCAN,
    PROJ_DIR,
    IO_DIR,
    MQTT_TOPIC_PREDICT,
    DATA_KEY_CATEGORY, 
    DATA_KEY_TIMESTAMP,
    DATA_KEY_QLIST_BACK_MID,
    DATA_KEY_QLIST_BACK_LOW,
    DATA_KEY_QLIST_NECK,
    DATA_KEY_QLIST,
    BLE_ADDR_SENSOR_BACK_LOW, 
    BLE_ADDR_SENSOR_NECK
)

from mqtt.controller import (
    setup
)

import paho.mqtt.client as mqtt
import numpy as np
from os import listdir
from os.path import join
from time import (
    sleep
)
import pandas as pd
import csv
import fasteners

lock = fasteners.InterProcessLock('%s/tmp_lock_file' % IO_DIR)

TIME_BETWEEN_READINGS = 0.1 # seconds
PATH = "./samples"

led_and_buzzer = None
data_csv_name = "data_with_labels.csv"
data_csv_path = os.path.join(IO_DIR, data_csv_name)
ble_queue = asyncio.Queue()
mode = MODE_RT_SCAN
calibrate_count = 0
is_base_quat = True

base_quats = {}

from bleak import (
    BleakClient,
    discover
)

from utils.ble_utils import (
    discover_sensors,
    LEDAndBuzzer,
    QuatSensor
)

from utils.utils import (
    getTimeStamp,
    NP_Q
)

# On disconnect from a bleak client, this function is called
def on_disconnect(client: BleakClient):
    print(f"Disconnected from bleak {BLE_ADDR_TO_NAME[client.address]}!")
    

disconnected_event = asyncio.Event()

def disconnected_callback(client):
    print("Disconnected callback called!")
    disconnected_event.set()
    

async def run(address, postfix, flag, flags, mqtt_flag, warn_flag):
    """
    Main loop for each sensor. 
    Collects sensor data in quaternions to be processed.
    """
    global led_and_buzzer, calibrate_count

    ble_client = BleakClient(address, timeout = 15)
    await ble_queue.put(ble_client)

    async with ble_client as client:
        # Upon bleak client connect, register all the sensors
        if client.is_connected:
            client.set_disconnected_callback(on_disconnect)
            print(postfix + " bleak is connected!")
                                                                                                    
            # Set buzzer if flag is true.
            led_and_buzzer = LEDAndBuzzer()
            movement_sensor = QuatSensor()

            # Start listener for the device
            try:
                print("Starting listener for " + postfix)
                await movement_sensor.start_listener(client)

            except Exception as e:
                print("Fail to start bleak listener for " + postfix)
                print(e)

        # Once connected the client should not disconnect
        # If it does, exception is raised
        
        await asyncio.sleep(5)

        datetime_start = getTimeStamp()

        
            
        while True:

            if mode == MODE_RT_SCAN:
                delay = 10
                if calibrate_count < 5:
                    print("==================\n\nCollecting sample %d for calibration... Sit straight please!\n\n==================\n\n" % calibrate_count)
                    calibrate_count += 1                    

            elif mode == MODE_EXPORT_DATA: 
                delay = 1
                if (round(getTimeStamp()) - round(datetime_start) > int(sys.argv[2])):
                    print("Scan done!!!")
                    break 
                else:
                    print("Time elapsed: %d" % (round(getTimeStamp()) - round(datetime_start)))

            
            for i in range(delay):
                if warn_flag.is_set() or (calibrate_count > 2 and calibrate_count < 5):
                    print("haha")
                    await led_and_buzzer.notify(client, 0x05)
                else:
                    await led_and_buzzer.notify(client, 0x02)
                await asyncio.sleep(TIME_BETWEEN_READINGS) # Without await, the bleak listener cannot update the readings array, giving errors

            try:
                # Check if client is connected, if not raise Exception
                if not client.is_connected:
                    print("Bleak client for " + postfix + " not connected")
                    flag.clear()
                    raise Exception

                # Set bleak client's flag since bleak client is connected
                if not flag.is_set():
                    flag.set()
                
                print("--------------------")
                print("All flags: " + str(all(f.is_set() for f in flags)))
                # Wait for all the flags to be set before taking readings
                for f in flags:
                    await f.wait()

                # Print all the data collected for all the devices
                print("--------------------")
                print("All flags: " + str(all(f.is_set() for f in flags)))

                print(postfix + ":quat" + str(movement_sensor.readings))


                # Get all readings
                timestamp = movement_sensor.readings[0]

                # Create a dictionary of readings
                l = ["q0_", "q1_", "q2_", "q3_"]
                sensor_keys = [i + postfix for i in l]
                sensor_val = movement_sensor.readings[1:]
                zip_iter = zip(sensor_keys, sensor_val)
                datalist = dict(zip_iter)
                datalist[DATA_KEY_TIMESTAMP] = timestamp

                # Write to json file
                json_object = json.dumps(datalist)
                filename = postfix + ".json"
                filepath = os.path.join(IO_DIR, filename)
                with lock:
                    with open(filepath, "w") as outfile:
                        outfile.write(json_object)

                # Set mqtt flag after writing to json
                if not mqtt_flag.is_set():
                    mqtt_flag.set()

            except (Exception, KeyboardInterrupt) as e:
                print(e)
                raise e

# https://stackoverflow.com/questions/59073556/how-to-cancel-all-remaining-tasks-in-gather-if-one-fails

async def main(mqtt_client, category):
    """
    Sets up flags, switches and tasks to be run.
    """
    tasks = []
    while True:
    # try:
        # This finds the bluetooth devices and will not exit untill all devices are visible. However, this does not connect to the devices.
        while not await discover_sensors():
            print("waiting for sensors")

    
        # Create flags for each sensor to signal whene each bleak client is connected for each sensor
        neck_Flag = asyncio.Event()
        back_low_Flag = asyncio.Event()
        back_mid_Flag = asyncio.Event()
        # flags = [neck_Flag, back_Flag, shoulder_r_Flag, shoulder_l_Flag]

        # switcher is a switch case statement but in python
        switcher = {
            BLE_NAME_SENSOR_NECK: neck_Flag,
            BLE_NAME_SENSOR_BACK_MID: back_mid_Flag,
            BLE_NAME_SENSOR_BACK_LOW: back_low_Flag
        }

        switcher = {
            BLE_NAME_SENSOR_NECK: neck_Flag,
            BLE_NAME_SENSOR_BACK_MID: back_mid_Flag,
            BLE_NAME_SENSOR_BACK_LOW: back_low_Flag
        }

        flags = [switcher[BLE_ADDR_TO_NAME[address]]for address in BLE_ADDR_LIST]
        
        # Create flags for each sensor to signal whene each bleak client is connected for each sensor
        mqtt_neck_Flag = asyncio.Event()
        mqtt_back_low_Flag = asyncio.Event()
        mqtt_back_mid_Flag = asyncio.Event()

        # mqtt_switcher is a switch case statement but in python
        mqtt_switcher = {
            BLE_NAME_SENSOR_NECK: mqtt_neck_Flag,
            BLE_NAME_SENSOR_BACK_MID: mqtt_back_mid_Flag,
            BLE_NAME_SENSOR_BACK_LOW: mqtt_back_low_Flag
        }

        mqtt_flags = [mqtt_switcher[BLE_ADDR_TO_NAME[address]]for address in BLE_ADDR_LIST]

        buzzer_Flag = asyncio.Event()
        mqtt_client.user_data_set(buzzer_Flag)

        # Create a list of tasks using list comprehension
        tasks = [
            asyncio.ensure_future(
                run(
                    address, 
                    BLE_ADDR_TO_NAME[address], 
                    switcher.get(BLE_ADDR_TO_NAME[address]), 
                    [flags[i] for i in range(len(BLE_ADDR_LIST))], 
                    mqtt_switcher.get(BLE_ADDR_TO_NAME[address]),
                    buzzer_Flag
                )
            ) for address in BLE_ADDR_LIST]

        tasks.append(asyncio.ensure_future(post_processing_watcher(mqtt_client, mqtt_flags, category)))

        # Wait for all tasks
        await asyncio.gather(*tasks)

    # Any error, such as an unexpected disconnect from a device, will come here and restart the loop
    # This disconnects from every sensor safely and reconnects again

    # except (Exception, KeyboardInterrupt) as e:
    #     print("Something messed up. Cancelling everything")
    #     print(e)
    #     for t in tasks:
    #         t.cancel()
    #     if mode == MODE_RT_SCAN:
    #         print("Restarting loop in 5 seconds")
    #         await asyncio.sleep(5)
    #         print("Restarting...")
    #         is_base_quat = True
    #     else:
    #         asyncio.get_event_loop.close()


# Inteface with gateway ble functions
def get_data_func():
    """
    Combines exported sensor data in json and returns a single collated dictionary
    """
    filepaths = [os.path.join(IO_DIR, i+".json") for i in BLE_NAME_LIST]
        
    exists = [os.path.exists(filepath) for filepath in filepaths]
    # If all files exist
    if all(exist for exist in exists):
        # Merge the json files
        q_dict = {}
        send_dict = {}
        send_arr = None
        for i in BLE_NAME_LIST:
            filepath = os.path.join(IO_DIR, i+".json")
            with open(filepath, "r") as f:
                dict_data = json.load(f)
                q_dict.update(dict_data)
                print("Before:\n", q_dict)
                curr_quats = np.array(q_dict[q] for q in DATA_KEY_QLIST[i])
                send_arr = NP_Q.quad_diff(base_quats[i], curr_quats)
                send_dict = dict(zip(DATA_KEY_QLIST[i], send_arr.tolist()))
                print("After:\n", send_dict)
        return send_dict
    else:
        return

async def post_processing_watcher(mqtt_client, mqtt_flags, category):
    """
    Handles data when available.
    """
    global base_quat_back_low, base_quat_back_mid, base_quat_back_neck
    while True:
        for f in mqtt_flags:
            await f.wait()
        for f in mqtt_flags:
            f.clear()

        send_dict = get_data_func()
        print(send_dict)

        if mode == MODE_RT_SCAN:
            if is_base_quat:
                for name in BLE_NAME_LIST:
                    base_quats[name] = np.array([
                    send_dict[key] for key in DATA_KEY_QLIST[name]
                ])
                print("Calibrate: ", base_quats)
                # After first time, don't calibrate again.
                is_base_quat = False
            else:
                mqtt_send_data(mqtt_client, send_dict)
        else:
            export_to_csv()


def export_to_csv(send_dict):
    """
    This is for saving all the classification data to csv
    """
    # Convert from dict to df to csv row
    for key in send_dict.keys():
        val = send_dict.get(key)
        val_list = [val]
        send_dict.update({key: val_list})
    df = pd.DataFrame.from_dict(send_dict)
    df[DATA_KEY_CATEGORY] = category
    myCsvRow = df.to_numpy().flatten().tolist()

    if(os.path.exists(data_csv_path)):
        # If the file already exists, append the row, if not create a new file and save
        save_df = pd.read_csv(data_csv_path)
        with open(data_csv_path,'a', newline='', encoding='utf-8') as fd:
            wr = csv.writer(fd, delimiter=',')
            print(myCsvRow)
            wr.writerow(myCsvRow)
    else:
        # Append data to existing file.
        save_df = df
        save_df.to_csv(data_csv_path, index=False)


def mqtt_send_data(mqtt_client, send_dict):
    """
    Handles time-instant results by committing result to MQTT server
    """
    # Commit results to MQTT
    mqtt_client.publish(MQTT_TOPIC_PREDICT, json.dumps(send_dict))
    print("Published")


"""
Setting up Async Exception Handling
"""
async def handleException():
    while not ble_queue.empty():
        client = await ble_queue.get()
        await client.disconnect()

def handleException_(loop, context):
    print("lol")
    loop = asyncio.get_event_loop()
    loop.run_until_complete(handleException())
    loop.close()        

if __name__ == "__main__":
    """
    Main Function.
    """
    print(PROJ_DIR)
    # To find the address, once your sensor tag is blinking the green led after pressing the button, run the discover.py
    # file which was provided as an example from bleak to identify the sensor tag device
    category = None
    # KW: https://github.com/hbldh/bleak/issues/345 
    
    print(
        "Starting the program"
    )
    os.environ["PYTHONASYNCIODEBUG"] = str(1)

    if len(sys.argv) > 1:
        mode = MODE_EXPORT_DATA
        print("Collecting data for csv training!")
        print("Collecting ground truth labels for category %s" % (MQTT_CLASSIFICATIONS[int(sys.argv[1])]))
        category = sys.argv[1]
    

    loop = asyncio.get_event_loop()

    # mqtt_client = setup("127.0.0.1")
    
    # To interface with AWS MQTT
    mqtt_client = setup("13.59.198.52")

    # Runs the loop forever since main() has a while True loop
    try:
        # loop.set_exception_handler(handleException_)
        loop.run_until_complete(main(mqtt_client, category))
        loop.run_forever()
        
    # Something unexpected happened, whole program will close
    except (Exception, KeyboardInterrupt) as e:    
        print(e)
        handleException_(loop, None)
        loop.close()

