## Note for MQTT
- This branch aims to reproduce an MVP with only MQTT. However, the code is capable of HTTP protocol to update the remote website <a href = "https:/demoiot3237.herokuapp.com">here </a>. 
- Instructions to run:
    - pip install requirements.txt
    - cd to mqtt_broker/ and run ```mosquitto``` and ```python3 server.py```
    - cd to data_collection/src and run ```python3 mqtt_broker.py```
- TODO:
    - Need to find a way to link ```mqtt_client.py``` with an async function in ```app.py```.  


## Note for Sensortag
- This code builds on the ```sensortag``` CCS project in the BLUstack, and you need to replace the right files in the project to write the code to your device. You can try to load the ccs project: ```sensortag/cs3237_ccs_proj.zip```. Otherwise, you need to install Blustack, follow the BLE guide uploaded, and replace the files with that in ```sensor_profile/``` and ```sensortag_cs3237```. 
- You should use drag and drop to transfer the following files into CCS project window, then create a virtual link when prompted. These files are changed from the BLE stack:
  - (Added)    ```Applications/sensortag_quat.c```, ```sensortag_quat.h```
  - (Modified) ```Applications/sensortag.c```, ```sensortag.h```
  - (Added) ```Applications/quat_utils.c```, ```quat_utils.h```
  - (Added) ```Profile/quatservice.c```, ```quatservice.h```

- I referred to <a href = "https://github.com/pruthvikar/SensorTag-Fusion/tree/master/projects/ble">this repository </a> to implement the BLE service. Eventually it replaces the movement service. It is identical except for uuid, and the preprocessing involved before actually writing to client.
- I referred to <a href = "https://github.com/kriswiner/MPU9250">this repository </a> to modify the filters for quaternions. We can try both the Mohany one and the Madgwick one.
- <a href = "https://thepoorengineer.com/en/quaternion/">This link</a> provides a good guide and the pygame code to simulate visually the quaternion orientation.
- To run, have two terminals up and run the following on one terminal each:
  - ```python app.py 3 1000```
  - ```python app_viz.py```
- Sometimes it fails, then just run again.