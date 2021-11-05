from flask import Flask, request, render_template
import requests
import json

def main():
    url = "https://demoiot3237.herokuapp.com/add_data"
    acc_dict = {'timecollect': 1234, 'acc_x_neck': 0.333, 'acc_y_neck': 0.345, 'acc_z_neck': 0.567}
    
    #Send the request and get back the response into result
    requests.put(url, params=acc_dict)

if __name__ == '__main__':
    main()

