from commons.commons import (
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

import paho.mqtt.client as mqtt
import asyncio

import json

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected.")
        client.subscribe(MQTT_TOPIC_CLASSIFY)
    else:
        print("Failed to connect. Error code: %d" % (rc))

def on_message(client, userdata: asyncio.Event, msg):
    print("Received message from server.")
    resp_dict = json.loads(msg.payload)
    if (resp_dict['classification'] != MQTT_CLASSIFICATION_PROPER):
        userdata.set()     
    else:
        userdata.clear()
        
    print(
        resp_dict
    )
    
def setup(hostname):
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(hostname)
    client.loop_start()
    return client