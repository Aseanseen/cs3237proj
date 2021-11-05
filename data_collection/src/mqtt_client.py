import paho.mqtt.client as mqtt
import numpy as np
from PIL import Image
import json
from os import listdir
from os.path import join
from time import (
    sleep
)

PATH = "./samples"
MQTT_TOPIC_CLASSIFY = "Group_B2/IMAGE/classify"
MQTT_TOPIC_PREDICT = "Group_B2/IMAGE/predict"

SAMPLE_DATA_SIMPLE = [-0.06437778, -0.9050758, 0.42034901, -0.95735092,  0.03532022,  0.28676069,
    0.66192558, -0.56875487,  0.48823398, -0.49501592, -0.52682804,  0.69094968]

SAMPLE_DATA = {
    "Timestamp": 1635240442.913178,
    "accX_shoulder_l":1.3359375,
    "accY_shoulder_l":-1.38330078125,
    "accZ_shoulder_l":0.61767578125,
    "magX_shoulder_l":-1.16729736328125,
    "magY_shoulder_l":0.84686279296875,
    "magZ_shoulder_l":0.7171630859375,
    "gyroX_shoulder_l":-30.887423687423688,
    "gyroY_shoulder_l":-117.4021978021978,
    "gyroZ_shoulder_l":-20.99145299145299,
    "accX_back":-1.94970703125,
    "accY_back":0.0966796875,
    "accZ_back":0.35205078125,
    "magX_back":-0.9613037109375,
    "magY_back":1.53350830078125,
    "magZ_back":1.13677978515625,
    "gyroX_back":-93.56190476190476,
    "gyroY_back":65.67326007326007,
    "gyroZ_back":30.43760683760684,
    "accX_shoulder_r":-1.22412109375,
    "accY_shoulder_r":-1.34130859375,
    "accZ_shoulder_r":0.810546875,
    "magX_shoulder_r":-0.2288818359375,
    "magY_shoulder_r":-0.20599365234375,
    "magZ_shoulder_r":0.23651123046875,
    "gyroX_shoulder_r":236.15384615384613,
    "gyroY_shoulder_r":8.546520146520146,
    "gyroZ_shoulder_r":94.6114774114774,
    "accX_neck":-0.19189453125,
    "accY_neck":-1.42529296875,
    "accZ_neck":1.31298828125,
    "magX_neck":-0.83160400390625,
    "magY_neck":0.17547607421875,
    "magZ_neck":1.8768310546875,
    "gyroX_neck":10.645665445665443,
    "gyroY_neck":-29.388034188034187,
    "gyroZ_neck":7.796825396825397
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
    print("Sending data.")

    send_data(client)
    # After all the files are sent over, 
    # the program is suspended in an infinite loop
    while True:
        pass

if __name__ == "__main__":
    main()




