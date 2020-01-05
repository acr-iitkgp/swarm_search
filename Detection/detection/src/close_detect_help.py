import sys
import cv2
import numpy as np
import math

font = cv2.FONT_HERSHEY_COMPLEX
def f1(img):

    bm = 0
    frame_HSV = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
    frame_HSVr = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)[:, :, 0]
    frame_HSV = (cv2.GaussianBlur(frame_HSV, (11, 11), 0)).astype(np.int)
    sz = frame_HSV.shape[0] * frame_HSV.shape[1]
    z2 = np.sum(frame_HSV, axis=(0, 1)) / (sz - bm)
    z3 = (frame_HSV - z2).astype(np.float) / np.array([255.0, 255.0, 255.0])
    z3 = (abs(z3) > 0.22).any(axis=2).astype(np.uint8)
    z3 = cv2.erode(z3, None)

    z3 = cv2.dilate(z3, None, iterations=5)

    bm = bm + sz - np.sum(z3)
    z3 = np.repeat(z3[:, :, np.newaxis], 3, 2).astype(np.uint8)
    frame_HSV = z3 * frame_HSV
    z4 = (frame_HSVr > 40)*(frame_HSVr < 100)*1
    z4 = np.repeat(z4[:, :, np.newaxis], 3, 2).astype(np.uint8)

    img = z3 * z4 * img
    
    return img


def get_cnt(img):    
    
    img = f1(img)
    img = (img.all(2)!=0).astype(np.uint8)*255

    cnt2=[]
    contours, hierarchy = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    #cnt3.extend(contours)
    for cnt in contours:
        # Contours detection
        area = cv2.contourArea(cnt)

        approx = cv2.approxPolyDP(cnt, 0.08*cv2.arcLength(cnt, True), True) # 0.012 param      #########
        x = approx.ravel()[0]
        y = approx.ravel()[1]

        if area > 40 and area<1000:#param

            if len(approx) == 4:
                
                if len(cnt) > 4:
                    (cx,cy),(MA,ma),angle = cv2.fitEllipse(cnt)
                    ar = MA/ma
                    hull = cv2.convexHull(cnt)
                    hull_area = cv2.contourArea(hull)
                    solidity = float(area)/hull_area
                else:
                    ar = np.linalg.norm(approx[0] - approx[2])/np.linalg.norm(approx[1]-approx[3])
                    if ar > 1:
                        ar=1/ar
                    solidity = 1.0

                if solidity > 0.8:
                    cnt2.append(approx)
    return cnt2

      
