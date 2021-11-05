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
    BLE_ADDR_SENSOR_BACK,
    MQTT_CLASSIFICATION_BACKWARD, 
    MQTT_CLASSIFICATION_PROPER,
    BLE_ADDR_LIST,
    BLE_ADDR_TO_NAME,
    BLE_NAME_SENSOR_NECK,
    BLE_NAME_SENSOR_SHOULDER_L,
    BLE_NAME_SENSOR_SHOULDER_R,
    BLE_NAME_SENSOR_BACK,
    MQTT_TOPIC_CLASSIFY,
    MQTT_TOPIC_PREDICT
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

TIME_BETWEEN_READINGS = 5 # seconds
PATH = "./samples"

led_and_buzzer = None
from bleak import (
    BleakClient,
    discover
)

from utils.ble_utils import (
    discover_sensors
)

from utils.utils import (
    appendDataToDataframe,
    getDataframeFromDatalist, 
    loadDataframeFromCsv, 
    saveDataframeToCsv,
    getTimeStamp,
    getRowFromDatalists
)

class Service:
    """
    Here is a good documentation about the concepts in ble;
    https://learn.adafruit.com/introduction-to-bluetooth-low-energy/gatt

    In TI SensorTag there is a control characteristic and a data characteristic which define a service or sensor
    like the Light Sensor, Humidity Sensor etc

    Please take a look at the official TI user guide as well at
    https://processors.wiki.ti.com/index.php/CC2650_SensorTag_User's_Guide
    """

    def __init__(self):
        self.data_uuid = None
        self.ctrl_uuid = None


class Sensor(Service):

    def callback(self, sender: int, data: bytearray):
        raise NotImplementedError()

    async def start_listener(self, client, *args):
        # start the sensor on the device
        write_value = bytearray([0x01])
        await client.write_gatt_char(self.ctrl_uuid, write_value)

        # listen using the handler
        await client.start_notify(self.data_uuid, self.callback)


class MovementSensorMPU9250SubService:

    def __init__(self):
        self.bits = 0

    def enable_bits(self):
        return self.bits

    def cb_sensor(self, data):
        raise NotImplementedError


class MovementSensorMPU9250(Sensor):
    GYRO_XYZ = 7
    ACCEL_XYZ = 7 << 3
    MAG_XYZ = 1 << 6
    ACCEL_RANGE_2G  = 0 << 8
    ACCEL_RANGE_4G  = 1 << 8
    ACCEL_RANGE_8G  = 2 << 8
    ACCEL_RANGE_16G = 3 << 8

    def __init__(self):
        super().__init__()
        self.data_uuid = "f000aa81-0451-4000-b000-000000000000"
        self.ctrl_uuid = "f000aa82-0451-4000-b000-000000000000"
        self.ctrlBits = 0

        self.sub_callbacks = []

    def register(self, cls_obj: MovementSensorMPU9250SubService):
        self.ctrlBits |= cls_obj.enable_bits()
        self.sub_callbacks.append(cls_obj.cb_sensor)

    async def start_listener(self, client, *args):
        # start the sensor on the device
        await client.write_gatt_char(self.ctrl_uuid, struct.pack("<H", self.ctrlBits))

        # listen using the handler
        await client.start_notify(self.data_uuid, self.callback)

    def callback(self, sender: int, data: bytearray):
        unpacked_data = struct.unpack("<hhhhhhhhh", data)
        for cb in self.sub_callbacks:
            cb(unpacked_data)


class AccelerometerSensorMovementSensorMPU9250(MovementSensorMPU9250SubService):
    def __init__(self):
        super().__init__()
        self.bits = MovementSensorMPU9250.ACCEL_XYZ | MovementSensorMPU9250.ACCEL_RANGE_4G
        self.scale = 8.0/32768.0 # TODO: why not 4.0, as documented? @Ashwin Need to verify
        self.readings : list

    def cb_sensor(self, data):
        '''Returns (x_accel, y_accel, z_accel) in units of g'''
        rawVals = data[3:6]
        # print("[MovementSensor] Accelerometer:", tuple([ v*self.scale for v in rawVals ]))
        self.readings = [getTimeStamp(), tuple([ v*self.scale for v in rawVals ])]
        

class MagnetometerSensorMovementSensorMPU9250(MovementSensorMPU9250SubService):
    def __init__(self):
        super().__init__()
        self.bits = MovementSensorMPU9250.MAG_XYZ
        self.scale = 4912.0 / 32760
        self.readings : list
        # Reference: MPU-9250 register map v1.4

    def cb_sensor(self, data):
        '''Returns (x_mag, y_mag, z_mag) in units of uT'''
        rawVals = data[6:9]
        # print("[MovementSensor] Magnetometer:", tuple([ v*self.scale for v in rawVals ]))
        self.readings = [getTimeStamp(), tuple([ v*self.scale for v in rawVals ])]
        

class GyroscopeSensorMovementSensorMPU9250(MovementSensorMPU9250SubService):
    def __init__(self):
        super().__init__()
        self.bits = MovementSensorMPU9250.GYRO_XYZ
        self.scale = 500.0/65536.0
        self.readings : list

    def cb_sensor(self, data):
        '''Returns (x_gyro, y_gyro, z_gyro) in units of degrees/sec'''
        rawVals = data[0:3]
        # print("[MovementSensor] Gyroscope:", tuple([ v*self.scale for v in rawVals ]))
        self.readings = [getTimeStamp(), tuple([ v*self.scale for v in rawVals ])]
        

class OpticalSensor(Sensor):
    def __init__(self):
        super().__init__()
        self.data_uuid = "f000aa71-0451-4000-b000-000000000000"
        self.ctrl_uuid = "f000aa72-0451-4000-b000-000000000000"

    def callback(self, sender: int, data: bytearray):
        raw = struct.unpack('<h', data)[0]
        m = raw & 0xFFF
        e = (raw & 0xF000) >> 12
        print("[OpticalSensor] Reading from light sensor:", 0.01 * (m << e))



class HumiditySensor(Sensor):
    def __init__(self):
        super().__init__()
        self.data_uuid = "f000aa21-0451-4000-b000-000000000000"
        self.ctrl_uuid = "f000aa22-0451-4000-b000-000000000000"

    def callback(self, sender: int, data: bytearray):
        (rawT, rawH) = struct.unpack('<HH', data)
        temp = -40.0 + 165.0 * (rawT / 65536.0)
        RH = 100.0 * (rawH/65536.0)
        print(f"[HumiditySensor] Ambient temp: {temp}; Relative Humidity: {RH}")


class BarometerSensor(Sensor):
    def __init__(self):
        super().__init__()
        self.data_uuid = "f000aa41-0451-4000-b000-000000000000"
        self.ctrl_uuid = "f000aa42-0451-4000-b000-000000000000"

    def callback(self, sender: int, data: bytearray):
        (tL, tM, tH, pL, pM, pH) = struct.unpack('<BBBBBB', data)
        temp = (tH*65536 + tM*256 + tL) / 100.0
        press = (pH*65536 + pM*256 + pL) / 100.0
        print(f"[BarometerSensor] Ambient temp: {temp}; Pressure Millibars: {press}")


class LEDAndBuzzer(Service):
    """
        Adapted from various sources. Src: https://evothings.com/forum/viewtopic.php?t=1514 and the original TI spec
        from https://processors.wiki.ti.com/index.php/CC2650_SensorTag_User's_Guide#Activating_IO

        Codes:
            1 = red
            2 = green
            3 = red + green
            4 = buzzer
            5 = red + buzzer
            6 = green + buzzer
            7 = all
    """

    def __init__(self):
        super().__init__()
        self.data_uuid = "f000aa65-0451-4000-b000-000000000000"
        self.ctrl_uuid = "f000aa66-0451-4000-b000-000000000000"

    async def notify(self, client, code):
        # enable the config
        write_value = bytearray([0x01])
        await client.write_gatt_char(self.ctrl_uuid, write_value)

        # turn on the red led as stated from the list above using 0x01
        write_value = bytearray([code])
        await client.write_gatt_char(self.data_uuid, write_value)

# On disconnect from a bleak client, this function is called
def on_disconnect(client: BleakClient):
    print(f"Disconnected from bleak {BLE_ADDR_TO_NAME[client.address]}!")

async def run(address, postfix, flag, flags, mqtt_flag, warn_flag):
    global led_and_buzzer

    async with BleakClient(address, timeout = 15) as client:
        # Upon bleak client connect, register all the sensors
        if client.is_connected:
            client.set_disconnected_callback(on_disconnect)
            print(postfix + " bleak is connected!")

            # Set buzzer if flag is true.
            led_and_buzzer = LEDAndBuzzer()

            acc_sensor = AccelerometerSensorMovementSensorMPU9250()
            gyro_sensor = GyroscopeSensorMovementSensorMPU9250()
            magneto_sensor = MagnetometerSensorMovementSensorMPU9250()
            movement_sensor = MovementSensorMPU9250()

            movement_sensor.register(acc_sensor)
            movement_sensor.register(gyro_sensor)
            movement_sensor.register(magneto_sensor)

            # Start listener for the device
            try:
                print("Starting listener for " + postfix)
                await movement_sensor.start_listener(client)
            except Exception as e:
                print("Fail to start bleak listener for " + postfix)
                print(e)
        # Once connected the client should not disconnect
        # If it does, exception is raised
        while True:
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
                acc_readings = acc_sensor.readings
                gyro_readings = gyro_sensor.readings
                magneto_readings = magneto_sensor.readings
                print(postfix + ":acc " + str(acc_readings))
                print(postfix + ":gyro " + str(gyro_readings))
                print(postfix + ":mag " + str(magneto_readings))

                # Get all readings
                timestamp = acc_readings[0]
                acc = list(acc_readings[1])
                gyro = list(gyro_readings[1])
                magneto = list(magneto_readings[1])

                # Create a dictionary of readings
                l = ["accX_", "accY_", "accZ_", "magX_", "magY_", "magZ_", "gyroX_", "gyroY_", "gyroZ_"]
                sensor_keys = [i + postfix for i in l]
                sensor_val = []
                sensor_val.extend(acc)
                sensor_val.extend(magneto)
                sensor_val.extend(gyro)
                zip_iter = zip(sensor_keys, sensor_val)
                datalist = dict(zip_iter)
                datalist["Timestamp"] = timestamp

                # Write to json file
                json_object = json.dumps(datalist)
                filename = postfix + ".json"
                with open(filename, "w") as outfile:
                    outfile.write(json_object)

                # Set mqtt flag after writing to json
                if not mqtt_flag.is_set():
                    mqtt_flag.set()

                if warn_flag.is_set() and address == BLE_ADDR_SENSOR_BACK:
                    print("haha")
                    warn_flag.clear()
                    await led_and_buzzer.notify(client, 0x05)
                else:
                    await led_and_buzzer.notify(client, 0x00)

            except Exception as e:
                raise e

# https://stackoverflow.com/questions/59073556/how-to-cancel-all-remaining-tasks-in-gather-if-one-fails
async def main(mqtt_client):
    global client
    while True:
        # This finds the bluetooth devices and will not exit untill all devices are visible. However, this does not connect to the devices.
        while not await discover_sensors():
            print("waiting for sensors")

        try:
            # Create flags for each sensor to signal whene each bleak client is connected for each sensor
            neck_Flag = asyncio.Event()
            back_Flag = asyncio.Event()
            shoulder_r_Flag = asyncio.Event()
            shoulder_l_Flag = asyncio.Event()
            # flags = [neck_Flag, back_Flag, shoulder_r_Flag, shoulder_l_Flag]

            # switcher is a switch case statement but in python
            switcher = {
                BLE_NAME_SENSOR_NECK: neck_Flag,
                BLE_NAME_SENSOR_BACK: back_Flag,
                BLE_NAME_SENSOR_SHOULDER_L: shoulder_l_Flag,
                BLE_NAME_SENSOR_SHOULDER_R: shoulder_r_Flag
            }

            switcher = {
                BLE_NAME_SENSOR_NECK: neck_Flag,
                BLE_NAME_SENSOR_BACK: back_Flag,
                BLE_NAME_SENSOR_SHOULDER_L: shoulder_l_Flag,
                BLE_NAME_SENSOR_SHOULDER_R: shoulder_r_Flag
            }

            flags = [switcher[BLE_ADDR_TO_NAME[address]]for address in BLE_ADDR_LIST]
            
            # Create flags for each sensor to signal whene each bleak client is connected for each sensor
            mqtt_neck_Flag = asyncio.Event()
            mqtt_back_Flag = asyncio.Event()
            mqtt_shoulder_r_Flag = asyncio.Event()
            mqtt_shoulder_l_Flag = asyncio.Event()

            # mqtt_switcher is a switch case statement but in python
            mqtt_switcher = {
                BLE_NAME_SENSOR_NECK: mqtt_neck_Flag,
                BLE_NAME_SENSOR_BACK: mqtt_back_Flag,
                BLE_NAME_SENSOR_SHOULDER_L: mqtt_shoulder_l_Flag,
                BLE_NAME_SENSOR_SHOULDER_R: mqtt_shoulder_r_Flag
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

            tasks.append(asyncio.ensure_future(mqtt_watcher(mqtt_client, mqtt_flags)))
            # Wait for all tasks
            await asyncio.gather(*tasks)

        # Any error, such as an unexpected disconnect from a device, will come here and restart the loop
        # This disconnects from every sensor safely and reconnects again
        except Exception as e:
            print("Something messed up. Cancelling everything")
            print(e)
            for t in tasks:
                t.cancel()
            print("Restarting loop in 5 seconds")
            await asyncio.sleep(5)
            print("Restarting...")


# Inteface with gateway ble functions
def get_data_func():
    filenames = [BLE_NAME_SENSOR_NECK+".json", BLE_NAME_SENSOR_SHOULDER_L+".json", BLE_NAME_SENSOR_SHOULDER_R+".json", BLE_NAME_SENSOR_BACK+".json"]
    exists = [os.path.exists(filename) for filename in filenames]
    # If all files exist
    if all(exist for exist in exists):
        # Merge the json files
        send_dict = {}
        for filename in filenames:
            with open(filename, "r") as f:
                dict_data = json.load(f)
                send_dict.update(dict_data)
        return send_dict
    else:
        return

async def mqtt_watcher(mqtt_client, mqtt_flags):
    while True:
        for f in mqtt_flags:
            await f.wait()
        for f in mqtt_flags:
            f.clear()
        mqtt_send_data(mqtt_client, mqtt_flags)

def mqtt_send_data(mqtt_client, mqtt_flags):
    send_dict = get_data_func()
    mqtt_client.publish(MQTT_TOPIC_PREDICT, json.dumps(send_dict))
    print("Published")


if __name__ == "__main__":
    """
    To find the address, once your sensor tag is blinking the green led after pressing the button, run the discover.py
    file which was provided as an example from bleak to identify the sensor tag device
    """

    # KW: https://github.com/hbldh/bleak/issues/345 
    
    print(
        "Starting the program"
    )
    os.environ["PYTHONASYNCIODEBUG"] = str(1)

    loop = asyncio.get_event_loop()

    # mqtt_client = setup("127.0.0.1")
    
    # To interface with AWS MQTT
    mqtt_client = setup("13.59.198.52")

    # Runs the loop forever since main() has a while True loop
    try:
        loop.run_until_complete(main(mqtt_client))

    # Something unexpected happened, whole program will close
    except Exception as e:
        print(e)
        loop.close()

