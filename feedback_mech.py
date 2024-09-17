import cv2
import cv2.aruco as aruco
import numpy as np
import time
import serial
from droneControl import connect, flightMode, drone_takeoff, VehicleMode, distance_to_home, arm
import subprocess
#################################

############ARUCO/CV2############
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
parameters = aruco.DetectorParameters_create()
###########drone port##################
port = serial.Serial('/dev/ttyUSB0', baudrate=9600, timeout=1)

###### drone ####################
def front_camera():
    cap=cv2.VideoCapture(0)
    cap.set(3,640)
    cap.set(4,480)

    while True:

        cameraMatrix   = np.array([[266.82523151,   0,         340.24328138],
                                  [  0,         265.35396197, 241.02371401],
                                  [  0,           0,           1        ]])
        cameraDistortion   = np.array([-0.29218741,  0.15190219, -0.00076712, -0.00037857, -0.0637175])
        marker_size = 3
        id_to_find=80
        ret, frame = cap.read() #for Threaded webcam
        frame_np = np.array(frame)
        gray_img = cv2.cvtColor(frame_np,cv2.COLOR_BGR2GRAY)

        if ret:

            cv2.imshow('Aruco Tracker',frame)

        if cv2.waitKey(1) & 0xFF == ord('Q'):
            break

        ids=''
        corners, ids, rejected = aruco.detectMarkers(image=gray_img,dictionary=aruco_dict,parameters=parameters)
        if ids is not None:
            print("Found these IDs in the frame:")
            print(ids)
        if ids is not None and ids[0] == id_to_find:
            ret = aruco.estimatePoseSingleMarkers(corners,marker_size,cameraMatrix=cameraMatrix,distCoeffs=cameraDistortion)
            rvec,tvec = ret[0][0,0,:], ret[1][0,0,:]
            R, _ = cv2.Rodrigues(rvec)
            x="{:.2f}".format(tvec[0])
            y="{:.2f}".format(tvec[1])
            z="{:.2f}".format(tvec[2])
            print(f"Found Aruco: {id_to_find}")
            marker_position="MARKER POSITION: x="+x+" y="+y+" z="+z
            print(marker_position)
            #message to be sent to drone port#
            roll_rad = np.arctan2(R[2,1], R[2,2])
            roll_deg = abs(np.degrees(roll_rad))
            if (-20<float(x)<20) and (roll_deg<30):

                port.write(f"Front Position aligned: {marker_position}")
                break
            
            else:
                VehicleMode(vehicle,"GUIDED")
                print("Vehicle in GUIDED mode.")
                time.sleep(1)
                arm(vehicle)
                time.sleep(5)
                drone_takeoff(vehicle,5)
                print("Taking off again for position alignment")
                time.sleep(10)
                VehicleMode(vehicle,"LAND")
                print("Vehicle in LAND mode.")
                time.sleep(30)
            

        else:
            port.write("Marker not found")
            print("Marker not found")



def side_camera():
    cap=cv2.VideoCapture(1)
    cap.set(3,640)
    cap.set(4,480)

    while True:

        cameraMatrix   = np.array([[266.82523151,   0,         340.24328138],
                                  [  0,         265.35396197, 241.02371401],
                                  [  0,           0,           1        ]])
        cameraDistortion   = np.np.array([-0.29218741,  0.15190219, -0.00076712, -0.00037857, -0.0637175])
        marker_size = 3
        id_to_find=81
        ret, frame = cap.read() #for Threaded webcam
        frame_np = np.array(frame)
        gray_img = cv2.cvtColor(frame_np,cv2.COLOR_BGR2GRAY)

        if ret:

            cv2.imshow('Aruco Tracker',frame)

        if cv2.waitKey(1) & 0xFF == ord('Q'):
            break

        ids=''
        corners, ids, rejected = aruco.detectMarkers(image=gray_img,dictionary=aruco_dict,parameters=parameters)
        if ids is not None:
            print("Found these IDs in the frame:")
            print(ids)
        if ids is not None and ids[0] == id_to_find:
            ret = aruco.estimatePoseSingleMarkers(corners,marker_size,cameraMatrix=cameraMatrix,distCoeffs=cameraDistortion)
            rvec,tvec = ret[0][0,0,:], ret[1][0,0,:]
            R, _ = cv2.Rodrigues(rvec)
            x="{:.2f}".format(tvec[0])
            y="{:.2f}".format(tvec[1])
            z="{:.2f}".format(tvec[2])
            print(f"Found Aruco: {id_to_find}")
            marker_position="MARKER POSITION: x="+x+" y="+y+" z="+z
            print(marker_position)
            roll_rad = np.arctan2(R[2,1], R[2,2])
            roll_deg = abs(np.degrees(roll_rad))
            #message to be sent to drone port#
            if (-20<float(x)<20) and (roll_deg<30) :

                port.write(f"Side Position aligned: {marker_position}")
                break

        else:
            port.write("Marker not found")
            print("Marker not found")
            time.sleep(1)


def connect_to_wifi(ssid, password):
    try:
        # Check if the Wi-Fi network is available
        result = subprocess.run(['nmcli', 'dev', 'wifi', 'list'], capture_output=True, text=True)
        if ssid not in result.stdout:
            print(f"Network {ssid} not found.")
            return False
        
        # Connect to the Wi-Fi network
        subprocess.run(['nmcli', 'dev', 'wifi', 'connect', ssid, 'password', password], check=True)
        print(f"Connected to {ssid}.")
        return True
    except subprocess.CalledProcessError as e:
        print(f"Failed to connect to {ssid}: {e}")
        return False

ssid = 'NodeMCU_AP'
password = 'nopassword'

while True:
    conn = connect_to_wifi(ssid,password)
    if conn==True:
        break
    else:
        print("searching for wifi")

vehicle = connect('udp:localhost:14550')
while True:

    distance=distance_to_home(vehicle)
    print(f"Distance to Drone Port: {round(distance,2)} m.")
    if flightMode=="LAND" and distance<=1:
        front_camera()
        side_camera()
        

