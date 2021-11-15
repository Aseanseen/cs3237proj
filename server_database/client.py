from flask import Flask, request, render_template
import requests
import json
import datetime
from commons import commons

def main():
    # url = "https://demoiot3237.herokuapp.com/add_data"
    url_add_data = "http://127.0.0.1:5000/add_data"
    url_get_data = "http://127.0.0.1:5000/get_data"
    acc_dict = {
        'name': "Looi Kai Wen",
        'timecollect': datetime.datetime.timestamp(datetime.datetime.now()), 
        'acc_x_neck': 0.333, 
        'acc_y_neck': 0.345, 
        'acc_z_neck': 0.567, 
        "classification": 1
    }
    acc_dict1 = {
        'name': "Sally",
        'timecollect': datetime.datetime.timestamp(datetime.datetime.now()), 
        'acc_x_neck': 0.333, 
        'acc_y_neck': 0.345, 
        'acc_z_neck': 0.567, 
        "classification": 1
    }

    # for i in range(10):
    #     dict_to_enter = {
    #         'name': "Sally",
    #         'timecollect': round(int(datetime.datetime.timestamp(datetime.datetime.now() + i * datetime.timedelta(seconds=5)))), 
    #         'acc_x_neck': 0.333, 
    #         'acc_y_neck': 0.345, 
    #         'acc_z_neck': 0.567, 
    #         "classification": commons.CLASSIFICATIONS[i % len(commons.CLASSIFICATIONS)]
    #     }
    #     requests.put(url_add_data, params=dict_to_enter)

    dict_to_enter = {
        "name" : "Sally",
        "start_time" : 1636145419,
        "end_time" : 1636146879
    }
    requests.get(url_get_data, params=dict_to_enter)
    #Send the request and get back the response into result
    # requests.put(url, params=acc_dict)
    # #Send the request and get back the response into result
    # requests.put(url, params=acc_dict1)
    # requests.post("http://127.0.0.1:5000/remove_data_all", params=acc_dict1)
if __name__ == '__main__':
    main()

