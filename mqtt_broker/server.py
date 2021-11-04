import paho.mqtt.client as mqtt
import numpy as np
import json
import requests
# import tensorflow as tf
# from tensorflow.python.keras.backend import set_session
# from tensorflow.keras.models import load_model
from os import listdir
from os.path import join
from model import (
    load_model,
    classify
)

model = None
MQTT_TOPIC_CLASSIFY = "Group_B2/IMAGE/classify"
MQTT_TOPIC_PREDICT = "Group_B2/IMAGE/predict"
url = "http://127.0.0.1:3237/classify"
url_post = "https://demoiot3237.herokuapp.com/add_data"

classes = [
    "daisy",
    "dandelion",
    "roses",
    "sunflowers",
    "tulips"
]

model = None
# MODEL_NAME = "flowers.hd5"

# session = tf.compat.v1.Session(
#     graph = tf.compat.v1.Graph()
# )

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Successfully connected to broker")
        client.subscribe(MQTT_TOPIC_PREDICT)
    else:
        print("Connection failed with code: %d." % rc)

def on_message(client, userdata, msg):
    print(msg.payload)
    recv_dict = json.loads(msg.payload)
    print(recv_dict)
    acc_dict = {
        "timecollect": round(recv_dict["Timestamp"]),
        "acc_x_neck": recv_dict["accX_neck"],
        "acc_y_neck": recv_dict["accY_neck"],
        "acc_z_neck": recv_dict["accZ_neck"], 
        "acc_x_back": recv_dict["accX_back"],
        "acc_y_back": recv_dict["accY_back"],
        "acc_z_back": recv_dict["accZ_back"], 
        "acc_x_shoulder_l": recv_dict["accX_shoulder_l"],
        "acc_y_shoulder_l": recv_dict["accY_shoulder_l"],
        "acc_z_shoulder_l": recv_dict["accZ_shoulder_l"], 
        "acc_x_shoulder_r": recv_dict["accX_shoulder_r"],
        "acc_y_shoulder_r": recv_dict["accY_shoulder_r"],
        "acc_z_shoulder_r": recv_dict["accZ_shoulder_r"], 
    }            

    """       
    MQTT      
    """                                                             
    print(acc_dict)                                  
    send_dict = classify(acc_list_str=acc_dict)         
    print(send_dict)     
    client.publish(MQTT_TOPIC_CLASSIFY, json.dumps(send_dict))
                                                               
    #Send the request and get back the response into result
    """
    HTTP
    """
    # response = requests.put(url_post, params = acc_dict)
    # print(response)





def setup(hostname):
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(hostname)
    client.loop_start()
    return client

def main():
    load_model()
    # setup("13.59.198.52")
    setup("127.0.0.1")
    while True:
        pass

if __name__ == "__main__":
    main()
    # url = "https://demoiot3237.herokuapp.com/add_data"
    # acc_dict = {
    #     'timecollect':1,
    #     'acc_x_neck':2,
    #     'acc_y_neck':2,
    #     'acc_z_neck':2,
    # }
    # requests.put(url, params = acc_dict)