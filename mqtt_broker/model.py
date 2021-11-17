from flask import Flask, request, render_template
import joblib
from commons import (
    CLASSIFICATION_TO_MQTT_CLASSIFICATION
)
import numpy as np
import json
import tensorflow as tf

def string_to_float_list(string_of_list):
    list_of_strings = string_of_list.strip('][').split(', ')
    list_of_floats = [float(i) for i in list_of_strings]
    return list_of_floats    

def classify(recv_dict):
    label_dict = CLASSIFICATION_TO_MQTT_CLASSIFICATION
    label_dict[-1] = "Error has occured, model is not predicting..."
    
    input_array = recv_dict
    
    model_input = np.array(input_array).reshape(1, -1)

    # try:
    result = model.predict(model_input)[0]
    # except:
        # result = -1
    
    max_index = np.argmax(result)

    #Create an answer object
    answer_dict = {}
    answer_dict['classification'] = label_dict[max_index]
    
    return answer_dict

def load_model():
    global model 
    filename = '../ml_model/model_nn_5'
    model = tf.keras.models.load_model(filename)
                                 

