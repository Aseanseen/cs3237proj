# Connecting to the TI SensorTag device for CS3237

## Instructions

### Setting up the environment
1. Install the bleak library using `pip install bleak`;
1. Bleak is compatible with almost all operating systems but if there are issues please refer to the project link
at [Bleak PyPi Page](https://pypi.org/project/bleak/); Linux users might need to install BlueZ if that is not installed in the factory version


### Setting up the device
1. Please make sure your device is not paired with any device at the moment; this includes closing the SensorTag app on
your phone after use
1. Please make sure you reset your SensorTag device to factory settings using the instructions at
[TI Sensor Tag User Guide ](https://processors.wiki.ti.com/index.php/CC2650_SensorTag_User's_Guide#Firmware_Upgrade); This UI option is only
available in Windows;


### Running the code

1. After setting up the environment and the device you should be able to see the green led on the device blink once you
press the button.
1. To find the address of your device that is used in the code, once your sensor tag is blinking the green,
run `python discover.py` to search and display all available devices. You should be able to find the address of your
SensorTag device from the list with the name `CC2650 SensorTag`
1. Copy the address and paste it into the address variable in main block within the `cc2650.py` file
1. Upon running the file `cc2650.py` you should be able to see the output from each of the sensors available on the board. Currently
OpticalSensor, HumiditySensor, MovementSensor and BarometerSensor are available on the device.

## Notes
In the example provided, we have used the `client.start_notify` function provided by the library using callbacks,
sensor values can also be directly read using the `client.read_gatt_char` function.
<br><br>
Some additional code provided, from the bleak repository, is the `sensortag.py` which displays the basic info about the
device provides another way to read the GATT characteristic

## Credits
Lot of the code has been adapted for ease of use from the following sources:
1. https://github.com/hbldh/bleak/tree/develop/examples
1. https://github.com/IanHarvey/bluepy/blob/a7f5db1a31dba50f77454e036b5ee05c3b7e2d6e/bluepy/sensortag.py
1. [TI SensorTag User Guide # GATT Server](https://processors.wiki.ti.com/index.php/CC2650_SensorTag_User's_Guide#Gatt_Server)

