import numpy as np
import cv2
import math
import time
from simple_pid import PID
from adafruit_servokit import ServoKit

def gstreamer_pipeline(
    capture_width=320,
    capture_height=240,
    display_width=320,
    display_height=240,
    framerate=60,
    flip_method=0,
):
    return (
        "nvarguscamerasrc ! "
        "video/x-raw(memory:NVMM), "
        "width=(int)%d, height=(int)%d, "
        "format=(string)NV12, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )
R=0
L=0
Line1=0
Line2=0
v=0
print(gstreamer_pipeline(flip_method=0))
cap = cv2.VideoCapture(gstreamer_pipeline(flip_method=2), cv2.CAP_GSTREAMER)
kit = ServoKit(channels=16)
pid = PID(0.85,0,0,setpoint=90)
def PID(Kp, Ki, Kd, MV_bar=0, beta=1, gamma=0):
	eD_prev = 0
	t_prev -100
	P = 0
	I = 0
	D = 0
	MV = MV_bar

while(cap.isOpened()):
    tic = time.perf_counter()
    ret, frame = cap.read()

    hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
            
    lower_red = np.array([30,80,50])
    upper_red = np.array([255,255,180])
    
    mask = cv2.inRange(hsv, lower_red, upper_red)
    #res = cv2.bitwise_and(frame, frame, mask = mask)
    
    b1 = False
    b2 = False
    b3 = False
    b4 = False
    
    for y in range(240):
        _Line = mask[239-y,300]
        if _Line>0:
            y2 = y
            b2 = True
            break  
    
    if b2 == False:
        for y in range(240):
            _Line = mask[239-y,200]
            if _Line>0:
                y1 = y
                b1 = True
                break
                
    for y in range(240):
        _Line = mask[239-y,20]
        if _Line>0:
            y4 = y
            b4 = True
            break
            
    if b4 == False:
        for y in range(240):
            _Line = mask[239-y,120]
            if _Line>0:
                y3 = y
                b3 = True
                break
    
    if b4 == True:
        L = y4
        Line1 = 20
    elif b3 == True:
        L = y3
        Line1 = 120
        
    if b2 == True:
        R = y2
        Line2 = 300
    elif b1 == True:
        R = y1
        Line2 = 200
    
    diff = R-L
    tang = abs(diff)/100
    
    if R>L:
        angle = (math.atan(tang) * 180 / math.pi)
    else:
        angle = 0 - (math.atan(tang) * 180 / math.pi)
    
    font = cv2.FONT_HERSHEY_SIMPLEX
    
    angle = round(angle,5)
    text = str(angle)
    
    
    mask = cv2.line(mask, (20,0), (20,240), (255,255,255), 2)
    mask = cv2.line(mask, (120,0), (120,240), (255,255,255), 2)
    mask = cv2.line(mask, (200,0), (200,240), (255,255,255), 2)
    mask = cv2.line(mask, (300,0), (300,240), (255,255,255), 2)
    
    
    frame = cv2.line(frame, (20,0), (20,240), (255,255,255), 2)
    frame = cv2.line(frame, (120,0), (120,240), (255,255,255), 2)
    frame = cv2.line(frame, (200,0), (200,240), (255,255,255), 2)
    frame = cv2.line(frame, (300,0), (300,240), (255,255,255), 2)
    
    frame = cv2.line(frame, (Line1,239-L), (Line2, 239-R), (255,0,0), 3)
    
    cv2.putText(mask, text, (20,210), font, 1, (255,255,255),2,cv2.LINE_4)
    
    
    test = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    
    test = cv2.line(test, (Line1,239-L), (Line2, 239-R), (255,0,0), 3)
    
    both = np.concatenate((frame,  test), axis=1)
    
    cv2.imshow('both',both)
    
    toc = time.perf_counter()
    
    #text = text.replace(".",",")
    #print((toc-tic))
    #print(text)
    
    #angle=(0-angle)+90
    v = angle
    MV = pid(v)
    #t, PV, SP, TR = yield MV
    PV = angle
    #I = TR - MV_bar - P - D
    #P = Kp*(beta*SP - PV)
    #I = I + Ki*(SP - PV)*(t - t_prev)
    #eD = gamma*SP - PV
    #D = Kd*(eD - eD_prev)/(t - t_prev)
    #MV = MV_bar + P + I + D
    #eD_prev = eD
    #t_prev = t
 
    if MV > 180:
    	MV = 179
    if 0 > MV:
    	MV = 1 

    kit.servo[8].angle = MV
    kit.continuous_servo[9].throttle = (abs(MV-90)/90)/3
            
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
        
cap.release()
cv2.destroyAllWindows()