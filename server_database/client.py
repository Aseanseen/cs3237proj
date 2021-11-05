from flask import Flask, request, render_template
import requests
import json

def main():
    # url = "https://demoiot3237.herokuapp.com/add_data"
    url = "http://127.0.0.1:5000/add_data"
    acc_dict = {
        'name': "Looi Kai Wen",
        'timecollect': 1234, 
        'acc_x_neck': 0.333, 
        'acc_y_neck': 0.345, 
        'acc_z_neck': 0.567, 
        "classification": 1
    }
    acc_dict1 = {
        'name': "Sally",
        'timecollect': 1234, 
        'acc_x_neck': 0.333, 
        'acc_y_neck': 0.345, 
        'acc_z_neck': 0.567, 
        "classification": 1
    }
    
    #Send the request and get back the response into result
    requests.put(url, params=acc_dict)
    #Send the request and get back the response into result
    requests.put(url, params=acc_dict1)
    requests.post("http://127.0.0.1:5000/remove_data_all", params=acc_dict1)
if __name__ == '__main__':
    main()

