"""
Scan/Discovery
--------------

Example showing how to scan for BLE devices.

Updated on 2019-03-25 by hbldh <henrik.blidh@nedomkull.com>

"""

import asyncio
from common import (
    BLE_ADDR_SENSOR_SHOULDER_L,
    BLE_ADDR_SENSOR_SHOULDER_R,
    BLE_ADDR_SENSOR_NECK,
    BLE_ADDR_SENSOR_BACK,
    BLE_ADDR_LIST
)


from bleak import discover

import asyncio

from bleak import BleakClient, discover


disconnected_event = asyncio.Event()

def disconnected_callback(client):
    print("Disconnected callback called!")
    disconnected_event.set()

async def discover_sensors():
    devices = await discover()
    list_of_ble_addr = [d.address for d in devices]
    set_of_devices_discovered = set(BLE_ADDR_LIST).intersection(set(list_of_ble_addr))
    print(set_of_devices_discovered)
    if len(set_of_devices_discovered) != 4:
        print("We cannot detect the following devices...")
        print(set(BLE_ADDR_LIST) - set_of_devices_discovered)
        return False
    else:
        print("All devices present!")
        return True

async def run():

    while not await discover_sensors():
        print("waiting")
        await asyncio.sleep(1.0)

    print("done")
    # client = BleakClient(
    #     list(set_of_devices_discovered)[0], disconnected_callback=disconnected_callback
    # ) 

    # async with client as client:
    #     print("Sleeping until device disconnects...")
    #     await client.disconnect()
    #     await disconnected_event.wait()
    #     print("Connected:", client.is_connected)


loop = asyncio.get_event_loop()
loop.run_until_complete(run())
