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
from commons import (
    MQTT_TOPIC_PREDICT,
    MQTT_TOPIC_CLASSIFY,
    URL_POST,
    DATA_KEY_TIMESTAMP,
)

model = None

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
          

    """       
    MQTT      
    """                          

    class_dict = classify(recv_dict=recv_dict)         
    print(class_dict)     
    client.publish(MQTT_TOPIC_CLASSIFY, json.dumps(class_dict))
    # Send dict should contain all the quaternion values from all 4 sensors, as well as the predicted classification.
    send_dict = {**recv_dict, **class_dict}    

    send_dict = {
        "name" : "Kai Wen",
        "timecollect" : recv_dict[DATA_KEY_TIMESTAMP],
        "classification" : class_dict["classification"]
    }

    print(send_dict)
    #Send the request and get back the response into result
    """
    HTTP
    """
    response = requests.put(URL_POST, params = send_dict)
    print(response)

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
