import numpy as np
import cv2 as cv
import glob

# Size of square in mm (needed for distance, not for intrinsic camera props)
square_size = 25

# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((6*9,3), np.float32)
objp[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2) * square_size

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

# Capture images
imglist = []
cap = cv.VideoCapture(0)
cap.set(cv.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv.CAP_PROP_FPS, 30)

if not cap.isOpened():
    print("cannot open camera")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("Can't recieve frame. Exiting.")
        break
    cv.imshow('frame', frame)
    key = cv.waitKey(1)
    if (key == ord(' ')):
        imglist.append(frame)
        print("Picture added to list")
    elif (key == ord('q')):
        break

cv.destroyAllWindows()

print()
for img in imglist:
    print("Processing image... ", end="")
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret, corners = cv.findChessboardCorners(gray, (9,6), None)

    # If found, add object points, image points (after refining them)
    if ret == True:
        print("Found corners. Using image.")
        objpoints.append(objp)
        corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners2)

        # Draw and display the corners
        cv.drawChessboardCorners(img, (9,6), corners2, ret)
        cv.imshow('img', img)
        cv.waitKey(500)
    else:
        print("Corners not found. Discarding.")

cv.destroyAllWindows()

print("Beginning calibration...")
ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
h, w = img.shape[:2]
newMtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (w,h), 0, (w,h))

print("Calibration done.\n\n")

print("Camera Intrinsics (Camera Matrix):")
print(mtx)
print("Meaning:")
print(f"fx: {int(round(mtx[0,0]))}")
print(f"fy: {int(round(mtx[1,1]))}")
print(f"cx: {int(round(mtx[0,2]))}")
print(f"cy: {int(round(mtx[1,2]))}")

print("\n\n")
print("Camera Extrinsics (Distortion Matrix):")
print(dist)
print("Meaning:")
print(f"k1 = {dist[0, 0]}")
print(f"k2 = {dist[0, 1]}")
print(f"p1 = {dist[0, 2]}")
print(f"p2 = {dist[0, 3]}")
print(f"k3 = {dist[0, 4]}")

print("\n\n")
print("Undistorted Camera Intrinsics (New Camera Matrix):")
print(newMtx)
print("Meaning:")
print(f"fx: {int(round(newMtx[0,0]))}")
print(f"fy: {int(round(newMtx[1,1]))}")
print(f"cx: {int(round(newMtx[0,2]))}")
print(f"cy: {int(round(newMtx[1,2]))}")
