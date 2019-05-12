'''
File: pnptest.py
Created on: 2019-09-05  11:19:43
Author: Johnny Sun (johnnysuns@163.com)
-----
Last Modified: 2019-09-05 23:43:42
Modified By: Johnny Sun (johnnysuns@163.com>)
-----
Copyright(C) 2019 - 2019 Networked Robotics and System Lab(NRSL)
'''
import numpy as np
import cv2 as cv
import glob
mtx = np.array([[400,0,512],[0,400,384],[0,0,1]],np.float64)
dist = np.array([0, 0, 0, 0], np.float64)

def draw(img, corners, imgpts):
    corner = tuple(corners[0].ravel())
    img = cv.line(img, corner, tuple(imgpts[0].ravel()), (255, 0, 0), 5)
    img = cv.line(img, corner, tuple(imgpts[1].ravel()), (0, 255, 0), 5)
    img = cv.line(img, corner, tuple(imgpts[2].ravel()), (0, 0, 255), 5)
    return img


criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
objp = np.zeros((6*7, 3), np.float32)
objp[:, :2] = np.mgrid[0:7, 0:6].T.reshape(-1, 2)
axis = np.float32([[3, 0, 0], [0, 3, 0], [0, 0, -3]]).reshape(-1, 3)
for fname in glob.glob('*.png'):
    img = cv.imread(fname)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    cv.imshow("gray", gray)
    cv.waitKey(100)
    ret, corners = cv.findChessboardCorners(gray, (7, 6), None)
    if ret == True:
        corners2 = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        # Find the rotation and translation vectors.
        ret, rvecs, tvecs = cv.solvePnP(objp, corners2, mtx, dist)
        # project 3D points to image plane
        imgpts, jac = cv.projectPoints(axis, rvecs, tvecs, mtx, dist)
        img = draw(img, corners2, imgpts)
        cv.imshow('img', img)
        k = cv.waitKey(0) & 0xFF
        if k == ord('s'):
            cv.imwrite(fname[:6]+'.png', img)
cv.destroyAllWindows()
