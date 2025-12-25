import cv2 as cv
import cv2.aruco as aruco
from picamera2 import Picamera2
import threading
import queue
from rplidar import RPLidar
import numpy as np
import time
import serial
import sys
import signal

arduino = serial.Serial(port='/dev/ttyACM0', baudrate=115200, timeout=0.1)
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

def scan_loop():
    for scan in lidar.iter_scans():
        if scan_queue.full():
            scan_queue.get_nowait()
        scan_queue.put(scan)


running = True 
def signal_handler(sig, frame):
    global running
    running = False
    time.sleep(0.1)
    sys.exit(0)

signal.signal(signal.SIGTERM, signal_handler)


# ============ Configuration ============
PORT = '/dev/ttyUSB0'
MAX_DIST =1500
MIN_DIST = 1
SECTOR_WIDTH = 5
QUEUE_SIZE = 3

# ============ Shared Data ============
scan_queue = queue.Queue(maxsize=QUEUE_SIZE)

# ============ Main ============
lidar = RPLidar(PORT, baudrate=115200, timeout=3)
t1 = threading.Thread(target=scan_loop, daemon=True)
t1.start()
j = 0
scan = []

picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"size": (640, 480)})
picam2.configure(config)
picam2.start()

dictionary = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50)
detector = cv.aruco.ArucoDetector(dictionary)
mtx=np.array([[583.39999978,   0. ,        317.26971098],
 [  0.    ,     568.87700109, 226.74554456],
 [  0.    ,       0.  ,         1.        ]])
distortion = np.array( [[ 4.37606760e-01, -2.53165723e+00, -1.30981396e-03,  1.46723672e-02,
   4.64273442e+00]])
t=""


try:
    while running:
        try:
            scan = scan_queue.get(timeout=1)     
        except queue.Empty:
            pass
        
        # Group into angle sectors
        sector_data = []
        for quality, angle, dist in scan:
            sector = (int((angle) // SECTOR_WIDTH) * SECTOR_WIDTH) % 360
            if MIN_DIST <= dist <= MAX_DIST:
                sector_data.append(sector)

        print(arduino.readline())
        ttt = ""
        circle = np.arange(0,360,5)
        for ss in sector_data:
            for angle in circle:
                a = ss - 20
                b = ss + 20
                if a < 0:
                    a += 360
                    if a<=angle<360 or 0<=angle<=b:
                        circle = circle[circle!=angle]
                elif b>355:
                    b -= 360
                    if 0<=angle<=b or a<=angle<360:
                        circle = circle[circle!=angle]
                else:
                    if a<=angle<=b:
                        circle = circle[circle!=angle]
        
        for angle in circle:    
            ttt = ttt + str(int(angle)) + 's'
        j += 1
        if (ttt!="" and j>=3):
            ww = 's'+ttt+'#'
            write_to_arduino(ww)
        
        if(j>=3):
            frame = picam2.capture_array()
            frame_bgr = cv.cvtColor(frame,cv.COLOR_RGB2BGR)
            markerCorners, markerIds, _ = detector.detectMarkers(frame_bgr)
            if markerIds is not None:
                len_ids = len(markerIds)
                t = ""
                for i in range(1):
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
                        distCoeffs=distortion, 
                        flags=cv.SOLVEPNP_IPPE_SQUARE)      #Try IPPE and ITERATIVE check if they are better than IPPE_SQUARE
                    
                    if(not retval):
                        len_ids -= 1
                        
                    
                    if(retval):
                        proj, _ = cv.projectPoints(objectPoints, rvecs, tvecs, mtx, distortion)
                        errors = np.linalg.norm(c-proj.squeeze(),axis=1)
                        mean_error = np.mean(errors)
                        if ((area < 7100 and mean_error>1.4) or (area > 7100 and mean_error>4.5)or area<1200):
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
                    rrr = f"{1}"+t+"#"
                    arduino.write(rrr.encode())
            
            cv.imshow('ArUco ', frame_bgr)  
            if cv.waitKey(1) == 27:
                break


        


finally:
    lidar.stop()
    lidar.stop_motor()
    lidar.disconnect()
    print("Lidar safely stopped.")
    cv.destroyAllWindows()
    picam2.stop()
    t1.join()