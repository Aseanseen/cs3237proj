import asyncio
from commons.commons import (
    BLE_ADDR_SENSOR_SHOULDER_L,
    BLE_ADDR_SENSOR_SHOULDER_R,
    BLE_ADDR_SENSOR_NECK,
    BLE_ADDR_SENSOR_BACK,
    BLE_ADDR_LIST,
    BLE_ADDR_TO_NAME
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
    print("We detect the following devices...")
    print([BLE_ADDR_TO_NAME[i] for i in set_of_devices_discovered])
    if set_of_devices_discovered != set(BLE_ADDR_LIST):
        print("We cannot detect the following devices...")
        print([BLE_ADDR_TO_NAME[i] for i in set(BLE_ADDR_LIST) - set_of_devices_discovered])
        return False
    else:
        print("All devices present!")
        return True

async def run():
    while not await discover_sensors():
        print("waiting in ble_utils")
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

if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(run())
