from mqtt_broker.commons import DATA_KEY_CATEGORY, DATA_KEY_Q0_BACK_LOW, DATA_KEY_Q0_NECK, DATA_KEY_Q1_BACK_LOW, DATA_KEY_Q1_NECK, DATA_KEY_Q2_BACK_LOW, DATA_KEY_Q2_NECK, DATA_KEY_Q3_BACK_LOW, DATA_KEY_Q3_NECK, DATA_KEY_TIMESTAMP
import paho.mqtt.client as mqtt
import numpy as np
from PIL import Image
import json
from os import listdir
from os.path import join
from time import (
    sleep
)
from commons import (
    DATA_KEY_Q0_BACK_MID,
    DATA_KEY_Q1_BACK_MID, 
    DATA_KEY_Q2_BACK_MID,
    DATA_KEY_Q3_BACK_MID
)

PATH = "./samples"
MQTT_TOPIC_CLASSIFY = "Group_B2/IMAGE/classify"
MQTT_TOPIC_PREDICT = "Group_B2/IMAGE/predict"

SAMPLE_DATA_SIMPLE = [-0.06437778, -0.9050758, 0.42034901, -0.95735092,  0.03532022,  0.28676069,
    0.66192558, -0.56875487,  0.48823398, -0.49501592, -0.52682804,  0.69094968]

SAMPLE_DATA = {
    DATA_KEY_Q0_BACK_MID: 0.02391175925731659,
    DATA_KEY_Q1_BACK_MID: -0.01958831213414669,
    DATA_KEY_Q2_BACK_MID: -0.6926203370094299,
    DATA_KEY_Q3_BACK_MID: 0.7182962894439697,
    DATA_KEY_TIMESTAMP: 1637028607.169289,
    DATA_KEY_Q0_BACK_LOW: -0.17328114807605743,
    DATA_KEY_Q1_BACK_LOW: 0.3296964466571808,
    DATA_KEY_Q2_BACK_LOW: -0.7871379852294922,
    DATA_KEY_Q3_BACK_LOW:0.48842713236808777,
    DATA_KEY_Q0_NECK: 0.7584683299064636,
    DATA_KEY_Q1_NECK: -0.6471405029296875,
    DATA_KEY_Q2_NECK: 0.05283760279417038,
    DATA_KEY_Q3_NECK: -0.00035772108822129667,
    DATA_KEY_CATEGORY: 3
}

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected.")
        client.subscribe(MQTT_TOPIC_CLASSIFY)
    else:
        print("Failed to connect. Error code: %d" % (rc))

def on_message(client, userdata, msg):
    print("Received message from server.")
    resp_dict = json.loads(msg.payload)
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
       
def get_data_func():
    # To inteface with gateway ble functions
    return SAMPLE_DATA

def send_data(client):
    send_dict = get_data_func()
    client.publish(MQTT_TOPIC_PREDICT, json.dumps(send_dict))

def main():
    # The global constant variable PATH holds the 
    # relative path to the samples folder. 
    global PATH
    client = setup("13.59.198.52")
    # client = setup("127.0.0.1") 
    print("Sending data.")        

    send_data(client)
    # After all the files are sent over, 
    # the program is suspended in an infinite loop
    while True:
        pass

if __name__ == "__main__":
    main()




