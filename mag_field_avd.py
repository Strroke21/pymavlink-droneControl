#!/usr/bin/env python

from __future__ import print_function
import time
import bmm150
import math
from droneControl import connect,VehicleMode
import numpy as np

vehicle = connect('/dev/ttyACM0')

def change_mode():
    VehicleMode(5,vehicle)
    print("Changing the mode to LOITER...")

def mag_field():
    device = bmm150.BMM150()  # Bus number will default to 1
    #print("Starting Magnatometer...")
    time.sleep(0.2)
    x, y, z = device.read_mag_data()
    x = np.round(x)
    y =  np.round(y)
    z = np.round(z,2)
    #print(x)

    heading_rads = math.atan2(x, y)

    heading_degrees = np.round(math.degrees(heading_rads),2)

    #print("Magnetometer x: {0:.2f}".format(x), end=' ')
    #print(" y: {0:.2f}".format(y), end=' ')
    #print(" z: {0:.2f}".format(z), end=' ')
    #print(" uT")

    #print('heading(axis_Y point to): {0:.2f} degree'.format(heading_degrees))
    #time.sleep(.250)
    return [heading_degrees,x,y,z]

while True:
    
    mag = mag_field()
    print(f" Heading: {mag[0]} Mag X: {mag[1]} Mag Y: {mag[2]} Mag Z: {mag[3]}")

    if (abs(mag[1]) or abs(mag[2]) or abs(mag[3]) >= 200):# or (mag[1] or mag[2] or mag[3]>= -200): for now considering only postive values using abs function

        VehicleMode(6,vehicle) #6 for RTL
        print("Changing mode to RTL...")
        time.sleep(2)
        break






