import cv2 as cv
import numpy as np
import cv2.aruco as aruco
from picamera2 import Picamera2
import time
import serial

arduino = serial.Serial(port='/dev/ttyACM0', baudrate=9600, timeout=0.1)
arduino.flush()

time.sleep(2)
arduino.write(b"START\n")
time.sleep(1)

def write_to_arduino(x):
    arduino.write(x.encode())
    time.sleep(0.1)
    #print(x)
   

def T_to_theta(T):
    yaw = np.arctan2(T[1,0],T[0,0])
    return yaw
    


picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"size": (640, 480)})
picam2.configure(config)
picam2.start()

dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50)
detector = cv.aruco.ArucoDetector(dictionary)
mtx=np.array([[583.39999978,   0. ,        317.26971098],
 [  0.    ,     568.87700109, 226.74554456],
 [  0.    ,       0.  ,         1.        ]])
dist = np.array( [[ 4.37606760e-01, -2.53165723e+00, -1.30981396e-03,  1.46723672e-02,
   4.64273442e+00]])
t=""

while True: 
    print(arduino.readline())
    frame = picam2.capture_array()
    frame_bgr = cv.cvtColor(frame,cv.COLOR_RGB2BGR)
    markerCorners, markerIds, _ = detector.detectMarkers(frame_bgr)
    if markerIds is not None:
        len_ids = len(markerIds)
        t = ""
        for i in range(len(markerIds)):

            print(arduino.readline())
            frame_bgr = cv.aruco.drawDetectedMarkers(frame_bgr, markerCorners, markerIds)
            
            display_images = []
            c=markerCorners[i][0]
            
            h=c[0][0]-c[1][0]
            w=c[1][1]-c[2][1]
            area=h*w
            objectPoints=np.array([[0,0,0],[0.096,0,0],[0.096,0.096,0],[0,0.096,0]])
            
            retval, rvecs, tvecs, inliers = cv.solvePnPRansac(
                objectPoints, 
                c, 
                mtx, 
                distCoeffs=dist, 
                flags=cv.SOLVEPNP_IPPE_SQUARE)      #Try IPPE and ITERATIVE check if they are better than IPPE_SQUARE
            if(retval):
                proj, _ = cv.projectPoints(objectPoints, rvecs, tvecs, mtx, dist)
                errors = np.linalg.norm(c-proj.squeeze(),axis=1)
                mean_error = np.mean(errors)
                print(area, mean_error)
                if ((area < 7100 and mean_error>1.4) or (area > 7100 and mean_error>4.5)):
                    len_ids -= 1
                    continue
                
                R,_=cv.Rodrigues(rvecs)
                Tct = np.eye(4)
                Tct[0:3,0:3] = R
                Tct[0:3,3] = tvecs.flatten()
                Tbc = np.array([[0,0,1,0.10],[-1,0,0,-0.02],[0,-1,0,0.022],[0,0,0,1]])
                T = Tbc@Tct
                
                xx = T[0,3]
                yy = T[1,3]
                theta = T_to_theta(T)
                t += f"x{round(xx,3)}y{round(yy,3)}t{round(theta,3)}i{int(markerIds[i][0])}" 

        if (t!=""):
            write_to_arduino(f"{len_ids}"+t+"#")
        
    
        
        
              
    cv.imshow('ArUco ', frame_bgr)  
    if cv.waitKey(1) == 27:
        break

   

cv.destroyAllWindows()
picam2.stop()