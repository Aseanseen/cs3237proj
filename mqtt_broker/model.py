from flask import Flask, request, render_template
import joblib
import numpy as np
import json

def string_to_float_list(string_of_list):
    list_of_strings = string_of_list.strip('][').split(', ')
    list_of_floats = [float(i) for i in list_of_strings]
    return list_of_floats    

def classify(acc_list_str):
    label_dict = {0: 'Good posture', 1: 'Lean Forward', 2: 'Lean Backward', 
                  3: 'Lean Left', 4: 'Lean Right'}
    
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
                                 

