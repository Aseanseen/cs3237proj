from flask import Flask, request, render_template
import requests
import json

def main():
    url = "http://127.0.0.1:3237/classify"
    
    #acc_x_neck, acc_y_neck, acc_z_neck, acc_x_back, acc_y_back, acc_z_back
    #acc_x_shoulder_l, ..., acc_x_shoulder_r, ..., acc_z_shoulder_r
    sample = [-0.06437778, -0.9050758, 0.42034901, -0.95735092,  0.03532022,  0.28676069,
    0.66192558, -0.56875487,  0.48823398, -0.49501592, -0.52682804,  0.69094968]
    payload = {'acc_list': json.dumps(sample)}
    
    #Send the request and get back the response into result
    result = requests.get(url, params=payload)
    print(result.content)

if __name__ == '__main__':
    main()

