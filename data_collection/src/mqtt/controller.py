from commons.commons import (
    MQTT_CLASSIFICATION_PROPER,
    MQTT_TOPIC_CLASSIFY,
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