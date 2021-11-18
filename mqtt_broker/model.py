from flask import Flask, request, render_template
import joblib
from commons import (
    CLASSIFICATION_TO_MQTT_CLASSIFICATION,
    MQTT_CLASSIFICATION_BACKWARD,
    MQTT_CLASSIFICATION_FORWARD,
    MQTT_CLASSIFICATION_LEFT,
    MQTT_CLASSIFICATION_PROPER,
    MQTT_CLASSIFICATION_RIGHT
)
import numpy as np
import json
import tensorflow as tf

from utils import (
    getEulerAngles
)

def string_to_float_list(string_of_list):
    list_of_strings = string_of_list.strip('][').split(', ')
    list_of_floats = [float(i) for i in list_of_strings]
    return list_of_floats    

def classify(recv_dict):
    label_dict = CLASSIFICATION_TO_MQTT_CLASSIFICATION
    label_dict[-1] = "Error has occured, model is not predicting..."
    answer_dict = {}
    postfix = ["_back_mid"]
    q_ls = ["q0", "q1", "q2", "q3"]

    for pf in postfix:
        y, p, r = getEulerAngles([recv_dict[i + pf] for i in q_ls])
    
    if abs(r) < 180 - 30:
        answer_dict['classification'] = MQTT_CLASSIFICATION_FORWARD
    elif abs(r) > 180 + 30:
        answer_dict['classification'] = MQTT_CLASSIFICATION_BACKWARD
    else:
        if p < 0 - 17:
            answer_dict['classification'] = MQTT_CLASSIFICATION_LEFT
        elif p > 0 + 17: 
            answer_dict['classification'] = MQTT_CLASSIFICATION_RIGHT
        else: 
            answer_dict['classification'] = MQTT_CLASSIFICATION_PROPER
    return answer_dict

def load_model():
    global model 
    import sys
    if sys.argv[1] == 1:
        model = joblib.load("modeldt1.joblib")
    elif sys.argv[1] == 6:
        filename = '../ml_model/model_nn_6'
        model = tf.keras.models.load_model(filename)
    elif sys.argv[1] == 5:
        filename = '../ml_model/model_nn_6'
        model = tf.keras.models.load_model(filename)                  

