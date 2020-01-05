#!/usr/bin/env python


# sys.path.remove("/opt/ros/kinetic/lib/python2.7/dist-packages")

import cv2
import numpy as np

import numpy as np
import argparse
import roi_detect_help

import cv2
font = cv2.FONT_HERSHEY_COMPLEX


# from cv_bridge import CvBridge, CvBridgeError

def kalman_xy(x, P, measurement, R,
              motion=np.matrix('0. 0. 0. 0.').T,
              Q=np.matrix(np.eye(4))):
    """
    Parameters:    
    x: initial state 4-tuple of location and velocity: (x0, x1, x0_dot, x1_dot)
    P: initial uncertainty convariance matrix
    measurement: observed position
    R: measurement noise 
    motion: external motion added to state vector x
    Q: motion noise (same shape as P)
    """
    return kalman(x, P, measurement, R, motion, Q,
                  F=np.matrix('''
                      1. 0. 1. 0.;
                      0. 1. 0. 1.;
                      0. 0. 1. 0.;
                      0. 0. 0. 1.
                      '''),
                  H=np.matrix('''
                      1. 0. 0. 0.;
                      0. 1. 0. 0.'''))


def kalman(x, P, measurement, R, motion, Q, F, H):
    '''
    Parameters:
    x: initial state
    P: initial uncertainty convariance matrix
    measurement: observed position (same shape as H*x)
    R: measurement noise (same shape as H)
    motion: external motion added to state vector x
    Q: motion noise (same shape as P)
    F: next state function: x_prime = F*x
    H: measurement function: position = H*x

    Return: the updated and predicted new values for (x, P)

    See also http://en.wikipedia.org/wiki/Kalman_filter

    This version of kalman can be applied to many different situations by
    appropriately defining F and H 
    '''
    # UPDATE x, P based on measurement m
    # distance between measured and current position-belief
    y = np.matrix(measurement).T - H * x
    S = H * P * H.T + R  # residual convariance
    K = P * H.T * S.I    # Kalman gain
    x = x + K * y
    I = np.matrix(np.eye(F.shape[0]))  # identity matrix
    P = (I - K * H) * P

    # PREDICT x, P based on motion
    x = F * x + motion
    P = F * P * F.T + Q

    return x, P


def cntsearch(frameorg, state):
    list_prevkalman = state
    list_currentkalman = []
    list_matched = []
    frame = frameorg
    gray = cv2.cvtColor(frameorg, cv2.COLOR_BGR2GRAY)
    corners = cv2.goodFeaturesToTrack(gray, 0, 0.0001, 0.01)
    ctc = np.zeros(gray.shape)
    for i in corners:
        x, y = i.ravel()
        ctc[int(y)][int(x)] = 1

    t2 = roi_detect_help.get_cnt(frame)

    frame = cv2.GaussianBlur(frame, (11, 11), 0)
    contourlist = []
    for cnt in t2:
        rects_temp = cv2.boundingRect(cnt)

        rects = np.array([[rects_temp[0], rects_temp[1]], [rects_temp[0] + rects_temp[2], rects_temp[1]], [rects_temp[0] +
                                                                                                           rects_temp[2], rects_temp[1] + rects_temp[3]], [rects_temp[0], rects_temp[1] + rects_temp[3]]])
        rects = rects.reshape(cnt.shape)
        center = np.sum(rects, axis=0) / 4
        rects = np.maximum((0.5 * (rects - center) + center).astype(int), np.zeros(rects.shape)).astype(int)
        rects_temp = cv2.boundingRect(rects)

        rectl = np.maximum((3 * (rects - center) + center).astype(int), np.zeros(rects.shape)).astype(int)
        rectl_temp = cv2.boundingRect(rectl)
        rectl_temp = [rectl_temp[0], rectl_temp[1], rectl_temp[2], rectl_temp[3]]
        if rectl_temp[0] + rectl_temp[2] >= ctc.shape[1]:
            rectl_temp[2] = ctc.shape[1] - rectl_temp[0] - 1
        if rectl_temp[1] + rectl_temp[3] >= ctc.shape[0]:
            rectl_temp[3] = ctc.shape[0] - rectl_temp[1] - 1
        sel = np.ix_(np.arange(rects_temp[1], rects_temp[1] + rects_temp[3]).tolist(),
                     np.arange(rects_temp[0], rects_temp[0] + rects_temp[2]).tolist())
        selc = np.ix_(np.arange(rects_temp[1], rects_temp[1] + rects_temp[3]).tolist(),
                      np.arange(rects_temp[0], rects_temp[0] + rects_temp[2]).tolist(), [0, 1, 2])
        acs = np.sum(frameorg[selc], axis=(0, 1)).astype(np.float) / rects_temp[2] / rects_temp[3]
        fcs = np.sum(ctc[(sel)])
        vs = np.var(frame[selc])
        sel = np.ix_(np.arange(rectl_temp[1], rectl_temp[1] + rectl_temp[3]).tolist(),
                     np.arange(rectl_temp[0], rectl_temp[0] + rectl_temp[2]).tolist())
        selc = np.ix_(np.arange(rectl_temp[1], rectl_temp[1] + rectl_temp[3]).tolist(),
                      np.arange(rectl_temp[0], rectl_temp[0] + rectl_temp[2]).tolist(), [0, 1, 2])
        fcl = np.sum(ctc[(sel)])
        acl = np.sum(frameorg[selc], axis=(0, 1)).astype(np.float) / rectl_temp[2] / rectl_temp[3]

        d = fcs * rectl_temp[2] * rectl_temp[3] / fcl / rects_temp[2] / rects_temp[3]       # feature count
        t = (acs - acl) / 255.0

        t = t * t          # LAB SPace        https://sensing.konicaminolta.us/blog/identifying-color-differences-using-l-a-b-or-l-c-h-coordinates/
        # #print(np.sum(t))
        cv2.putText(frameorg, "a"+str(((int(np.sum(t)*100)/100.0))), (cnt[0][0][0],cnt[0][0][1]), font, 0.5, (0, 255, 0), 2, cv2.LINE_AA)

        if (d < 1 or fcl == 0) and np.sum(t) > 0.01:
            # print(vs)
            contourlist.append(cnt)
            cv2.drawContours(frameorg, [rectl, rects],-1, (0,255,255),10)

    for cnt in contourlist:
        ((cX, cY), radius) = cv2.minEnclosingCircle(cnt)
        cl = np.array([cX, cY])
        matched = False
        R = 0
        
        for (x, P), dc, mc, _ in list_prevkalman:
            br = False
            for (x2, P2) in list_matched:
                if (x == x2).all() and (P == P2).all():
                    br = True
                    break
            if br:
                continue
            lcv = P[:2, :2]
            t = np.array([cX - x[0][0], cY - x[1][0]]).reshape((1, 2))
            lpp = np.matmul(np.matmul(t, lcv), t.T)
            #print(lpp)
            if lpp < 5000:  # threshold for match       shouln'd be changed
                matched = True
                list_matched.append((x, P))
                list_currentkalman.append((kalman_xy(x, P, cl, R), dc + 1, 0, cnt))

        if not matched:
            x = np.matrix('0. 0. 0. 0.').T
            P = np.matrix(np.eye(4)) * 10
            list_currentkalman.append((kalman_xy(x, P, cl, R), 1, 0, cnt))
    contourlist = []

    for (x, P), dc, mc, cnt in list_prevkalman:
        br = False
        for (x2, P2) in list_matched:
            if (x == x2).all() and (P == P2).all():
                br = True
                break
        if not br:
            if mc <= 3:  # see this,     this is to remove the rong kalman
                list_currentkalman.append(((x, P), dc, mc + 1, cnt))
    for (x, P), dc, mc, cnt in list_currentkalman:
        if dc >= 2 and mc<=0:  # to finalise the kalman that it is correct
            contourlist.append(cv2.boundingRect(cnt))

    list_prevkalman = list_currentkalman[:]

    return contourlist, list_prevkalman
