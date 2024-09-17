#!/usr/bin/env python3

# import multiprocessing.process
from time import time, sleep
import pyudev
from sender_rfd900x_multi import send_rfd900x_mavlink_data
from receiver_rfd900x_multi_v2 import mavlink_receiver_rfd900x, mavlink_data_queue
import threading

baud_rate = 57600
source_system_id = 4
# system_id = 4 
# target_system_id = 4

def find_device_address():
    global rfd900x_address
    context = pyudev.Context()
    devices = context.list_devices(subsystem = 'tty')
    address = list()
    for device in devices:
        # print(device)
        if device.get('ID_VENDOR_ID') == '10c4' and device.get('ID_MODEL_ID') == 'ea60' and device.device_path.split('/')[-4] == '1-1.1:1.0':
            # print(device.device_node, device.get('ID_VEDOR'), device.get('ID_VEDOR'), device.get('ID_MODEL'), device.get('ID_SERIAL_SHORT'), device.device_path)
            rfd900x_address = device.device_node 

if __name__ == "__main__":
    find_device_address()
    print(rfd900x_address)
    sender = send_rfd900x_mavlink_data(rfd900x_address, baud_rate, source_system_id)
    drone_data = mavlink_receiver_rfd900x(rfd900x_address, baud_rate, source_system_id)
    drone_data.start_receiving()
    
    while True:
        try:
            print(mavlink_data_queue.get())

        except KeyboardInterrupt:
            print("Stopping...")
            drone_data.stop_receiving()
        
    
    
        
    
    # while True:
    #     drone_data.receive_mavlink_data()
        # print(f"calling global position data")
        # print(drone_data.global_position_int_data())

    
    # while True:
    #     positions= global_position_data.receive_global_position_data()
    #     # print(positions)
    #     if positions:
    #         latitude, longitude, altitude = positions[0], positions[1], positions[2]
    #         print(f"Latitude:{latitude}, Longitude :{longitude}, Altitude:{altitude}")


    # global_position_data.spin()
    