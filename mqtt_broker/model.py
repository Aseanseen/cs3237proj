from flask import Flask, request, render_template
import joblib
import numpy as np
import json

from commons.commons import (
    MQTT_CLASSIFICATION_FORWARD,
    MQTT_CLASSIFICATION_LEFT,
    MQTT_CLASSIFICATION_RIGHT,
    MQTT_CLASSIFICATION_BACKWARD,
    MQTT_CLASSIFICATION_PROPER
)

def string_to_float_list(string_of_list):
    list_of_strings = string_of_list.strip('][').split(', ')
    list_of_floats = [float(i) for i in list_of_strings]
    return list_of_floats    

def classify(acc_list_str):
    label_dict = {CLASSIFICATION_FORWARD: MQTT_CLASSIFICATION_FORWARD, CLASSIFICATION_LEFT: MQTT_CLASSIFICATION_LEFT, CLASSIFICATION_RIGHT: MQTT_CLASSIFICATION_RIGHT, 
                  CLASSIFICATION_BACKWARD: MQTT_CLASSIFICATION_BACKWARD, CLASSIFICATION_PROPER: MQTT_CLASSIFICATION_PROPER}
    
    input_array = [
        acc_list_str["acc_x_neck"],
        acc_list_str["acc_y_neck"],
        acc_list_str["acc_z_neck"],
        acc_list_str["acc_x_back"],
        acc_list_str["acc_y_back"],
        acc_list_str["acc_z_back"],
        acc_list_str["acc_x_shoulder_l"],
        acc_list_str["acc_y_shoulder_l"],
        acc_list_str["acc_z_shoulder_l"],
        acc_list_str["acc_x_shoulder_r"],
        acc_list_str["acc_y_shoulder_r"],
        acc_list_str["acc_z_shoulder_r"], 
    ]
    
    model_input = np.array(input_array).reshape(1, -1)
    print(model)
    result = model.predict(model_input)[0]
    
    #Create an answer object
    answer_dict = {}
    answer_dict['classification'] = label_dict[result]
    
    return answer_dict
    # json_response = json.dumps(answer_dict, indent = 4) 
    
    # return str(json_response)

def load_model():
    global model 
    filename = 'rfc_model.sav'
    model = joblib.load(filename)
                                 

