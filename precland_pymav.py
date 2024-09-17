###########DEPENDENCIES################
import time
import math
from pymavlink import mavutil
import cv2
import cv2.aruco as aruco
import numpy as np
import subprocess
from droneControl import connect, flightMode, condition_yaw, VehicleMode, drone_takeoff, get_local_position,distance_to_home, send_land_message, arm_status, set_parameter, send_velocity_setpoint, get_rangefinder_data
#from imutils.video import WebcamVideoStream
#import imutils
#######VARIABLES####################
##Aruco
id_to_find = 72
marker_size = 16 #cm
takeoff_height = 6
velocity = 0.5

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)


parameters = aruco.DetectorParameters_create()
##

##Camera
horizontal_res = 640
vertical_res = 480
#cap = WebcamVideoStream(src=0, width=horizontal_res, height=vertical_res).start()
cap=cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, horizontal_res)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, vertical_res)

horizontal_fov = 62.2 * (math.pi / 180 ) ##Pi cam V1: 53.5 V2: 62.2
vertical_fov = 48.8 * (math.pi / 180)    ##Pi cam V1: 41.41 V2: 48.8

calib_path="/home/pi/video2calibration/calibrationFiles/"
cameraMatrix   = np.loadtxt(calib_path+'cameraMatrix.txt', delimiter=',')
cameraDistortion   = np.loadtxt(calib_path+'cameraDistortion.txt', delimiter=',')
##

##Counters and script triggers
found_count=0
notfound_count=0

first_run=0 #Used to set initial time of function to determine FPS
start_time=0
end_time=0
script_mode = 2##1 for arm and takeoff, 2 for manual LOITER to GUIDED land 
ready_to_land=0 ##1 to trigger landing

manualArm=False ##If True, arming from RC controller, If False, arming from this script.


#########FUNCTIONS#################

def lander():
    global first_run,notfound_count,found_count,marker_size,start_time
    if first_run==0:
        print("First run of lander!!")
        first_run=1
        start_time=time.time()
        
    ret, frame = cap.read()
    frame = cv2.resize(frame,(horizontal_res,vertical_res))
    frame_np = np.array(frame)    #array transformation 
    gray_img = cv2.cvtColor(frame_np,cv2.COLOR_BGR2GRAY)   #grey image conversion
    ids=''
    corners, ids, rejected = aruco.detectMarkers(image=gray_img,dictionary=aruco_dict,parameters=parameters)
    alt_int=int(altitude)
    align_yaw = None

    if flightMode(vehicle)!='LAND':
        VehicleMode(vehicle,"LAND")
        while flightMode(vehicle)!='LAND':
            print('WAITING FOR DRONE TO ENTER LAND MODE')
            time.sleep(1)
    try:
         
        
        if ids is not None and ids[0] == id_to_find:
             
            ############ markers position estimation from opencv############
            ret = aruco.estimatePoseSingleMarkers(corners,marker_size,cameraMatrix=cameraMatrix,distCoeffs=cameraDistortion) #markers position 
            (rvec, tvec) = (ret[0][0, 0, :], ret[1][0, 0, :])   # rotation and translation vectors
            
            ######heading calculation#######
            R, _ = cv2.Rodrigues(rvec) 
            yaw_rad = np.arctan2(R[1,0], R[0,0])
            yaw_deg = np.degrees(yaw_rad) 
            yaw = round((yaw_deg+360)%360,2)  #%360 sets limit of the yaw scale to 0-360  # 'round' makes heading yaw upto 2 decimals
            ################################
            
            
            ########### Roll #########
            roll_rad = np.arctan2(R[2,1], R[2,2])
            roll_deg = np.degrees(roll_rad)
            roll = round((roll_deg+360)%360,2)
            ##########################
            
            ########## Pitch #########
            pitch_rad = np.arctan2(-R[2,0],np.sqrt(R[2,1]**2 + R[2,2]**2))
            pitch_deg = np.degrees(pitch_rad)
            pitch = round((pitch_deg+360)%360,2)
            ##########################
            
            ########## marker position calculation ######
            x = '{:.2f}'.format(tvec[0])
            y = '{:.2f}'.format(tvec[1])
            z = '{:.2f}'.format(tvec[2])
            #############################################
            
            ######## fake rangefinder calculation ######
            z_f=float(z)
            z_int=int(z_f)
            #############################################
            
            ############ x,y angle calculation in radians #########
            y_sum = 0
            x_sum = 0
            
            x_sum = corners[0][0][0][0]+ corners[0][0][1][0]+ corners[0][0][2][0]+ corners[0][0][3][0]
            y_sum = corners[0][0][0][1]+ corners[0][0][1][1]+ corners[0][0][2][1]+ corners[0][0][3][1]
    
            x_avg = x_sum*.25
            y_avg = y_sum*.25
            
            x_ang = (x_avg - horizontal_res*.5)*(horizontal_fov/horizontal_res)
            y_ang = (y_avg - vertical_res*.5)*(vertical_fov/vertical_res)
            #########################################################
            
                
            if flightMode(vehicle)!='LAND':
                VehicleMode(vehicle,'LAND')
                
                while flightMode(vehicle)!='LAND':
                    time.sleep(1)
                print("------------------------")
                print("Vehicle now in LAND mode")
                print("------------------------")
                send_land_message(vehicle,x_ang,y_ang)
                #send_distance_message(z_int) #fake lidar message
            else:
                send_land_message(vehicle,x_ang,y_ang)
                #send_distance_message(z_int) #fake lidar message
                pass
            print("X CENTER PIXEL: "+str(x_avg)+" Y CENTER PIXEL: "+str(y_avg))
            print("FOUND COUNT: "+str(found_count)+" NOTFOUND COUNT: "+str(notfound_count))
            print("Marker Heading:"+str(yaw)+ " MARKER POSITION: x=" +x+" y= "+y+" z="+z)
            #print("Yaw:" +str(yaw)+ " Roll:" +str(roll)+ " Pitch:"+str(pitch), marker_position)
            found_count = found_count+1
            
            ########### yaw alignment command ###########
            
            if align_yaw is None and found_count==1:   # it will take yaw value only once and put it in condition_yaw() function
                align_yaw = yaw
                condition_yaw(vehicle,align_yaw,1)
                print("yaw alignment",align_yaw)
            else:
                pass
            #############################################
            
            print("Heading is aligned with marker")
        else:
            notfound_count = notfound_count+1
    except Exception as e:
        print('Target likely not found. Error: '+str(e))
        notfound_count=notfound_count+1
    
     

####################### MAIN DRONE PARAMETERS ###########################

########### main vehicle parameters #####
vehicle = connect('/dev/ttyACM0')
    ##SETUP PARAMETERS TO ENABLE PRECISION LANDING
set_parameter(vehicle,'PLND_ENABLED',1)
set_parameter(vehicle,'PLND_TYPE',1)
set_parameter(vehicle,'PLND_EST_TYPE',0)
set_parameter(vehicle,'LAND_SPEED',30)

if script_mode ==1:
    drone_takeoff(vehicle,takeoff_height)
    print(str(time.time()))
    send_velocity_setpoint(vehicle,0,0.5,0) ##Offset drone from target
    time.sleep(1)
    ready_to_land=1


elif script_mode ==2:
    
    while True:
       
        ######### distance_to_home calculation ########
        distance = distance_to_home(vehicle)
        altitude = -get_local_position(vehicle)[2]
          
        if (flightMode=='RTL' and distance<= 3 and altitude<=8) or (flightMode=='LAND'): 
            
            print("Landing Point Acquired...")
            ready_to_land=1
            break
        
        time.sleep(1)
        print("Distance to Home:"+str(distance)+ " Altitude:" +str(altitude)+" Waiting to acquire Landing Point...")
        
       
       
if ready_to_land==1:
    
    while True:
        altitude = get_rangefinder_data(vehicle)
        lander()
        if altitude<=0.3:
            break
        
    end_time = time.time()
    total_time = end_time - start_time
    total_time = abs(int(total_time))
    total_count = found_count + notfound_count
    freq_lander = total_count / total_time
    print("Total iterations: " + str(total_count))
    print("Total seconds: " + str(total_time))
    print("------------------")
    print("Lander function had a frequency of: " + str(freq_lander))
    print("------------------")
    print("Precision landing completed...")
    



           



