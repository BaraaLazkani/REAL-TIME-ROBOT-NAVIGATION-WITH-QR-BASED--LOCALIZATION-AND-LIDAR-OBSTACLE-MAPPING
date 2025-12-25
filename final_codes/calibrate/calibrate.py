import numpy as np
import cv2 as cv
import glob

# Termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Prepare object points (9x7 grid, 9.6cm width × 9.2cm height)
objp = np.zeros((4 * 4, 3), np.float32)  # (9 columns, 7 rows)
objp[:, :2] = np.mgrid[0:4, 0:4].T.reshape(-1, 2)# (width, height) in mm

# Arrays to store object and image points
objpoints = []  # 3D points in real world
imgpoints = []  # 2D points in image plane

images = glob.glob('*.jpg')
count = 0

for fname in images:
    img = cv.imread(fname)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    # Find chessboard corners (9x7 grid)
    ret, corners = cv.findChessboardCorners(gray, (4, 4), None)  # Update to (9,7)

    if ret:
        objpoints.append(objp)
        corners2 = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners2)
        count += 1

print(f"Successful calibrations: {count}")

# Calibrate camera
ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

# Compute reprojection error
mean_error = 0
for i in range(len(objpoints)):
    imgpoints2, _ = cv.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
    error = cv.norm(imgpoints[i], imgpoints2, cv.NORM_L2) / len(imgpoints2)
    mean_error += error

print(f"Total reprojection error: {mean_error / len(objpoints)}")
print("Camera matrix (K):\n", mtx)
print("Distortion coefficients (D):\n", dist)
"""
9.2 and 9.6 are considered cm
Successful calibrations: 42
Total reprojection error: 0.30997029591340963
Camera matrix (K):
 [[583.40005881   0.         317.26969773]
 [  0.         568.87705925 226.74550975]
 [  0.           0.           1.        ]]
Distortion coefficients (D):
 [[ 4.37606949e-01 -2.53165976e+00 -1.30983552e-03  1.46723945e-02
   4.64274007e+00]]
"""

"""
92 and 96 are considered mm
Successful calibrations: 42
Total reprojection error: 0.30997062206974185
Camera matrix (K):
 [[583.39999978,   0. ,        317.26971098],
 [  0.    ,     568.87700109, 226.74554456],
 [  0.    ,       0.  ,         1.        ]]
Distortion coefficients (D):
 [[ 4.37606760e-01, -2.53165723e+00, -1.30981396e-03,  1.46723672e-02,
   4.64273442e+00]]
"""