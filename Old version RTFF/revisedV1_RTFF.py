# Import the necessary packages
import numpy as np
import cv2
import math
import time
import cv2.cuda as cvc
from simple_pid import PID
from adafruit_servokit import ServoKit


# Define a GStreamer pipeline string for capturing video from the camera
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

# Initialize some variables
R = 0
L = 0
Line1 = 0
Line2 = 0
v = 0

# Print the GStreamer pipeline string for debugging purposes
print(gstreamer_pipeline(flip_method=0))

# Create a VideoCapture object to read frames from the camera
cap = cv2.VideoCapture(gstreamer_pipeline(flip_method=2), cv2.CAP_GSTREAMER)

# Initialize a ServoKit object to control the motors
kit = ServoKit(channels=16)

# Initialize a PID controller to adjust the motor speed based on the angle of the red line
pid = PID(0.85, 0, 0, setpoint=90)

def PID(Kp, Ki, Kd, MV_bar=0, beta=1, gamma=0):
	eD_prev = 0
	t_prev -100 # undefined//commented in original version too?
	P = 0
	I = 0
	D = 0
	MV = MV_bar

while(cap.isOpened()):
    # Get the start time for measuring the frame rate
    tic = time.perf_counter()

    # Read a frame from the camera
    ret, frame = cap.read()

    # Convert RGB to BGR on the GPU
    frame_gpu = cvc.cvtColor(frame, cv2.COLOR_RGB2BGR)

    # Convert BGR to HSV on the GPU
    hsv_gpu = cvc.cvtColor(frame_gpu, cv2.COLOR_BGR2HSV)
            
    # Define the range of red colors in the HSV color space
    lower_red = np.array([30, 80, 50])
    upper_red = np.array([255, 255, 180])
    
    # Create the mask on the GPU
    mask_gpu = cv2.cuda.inRange(hsv_gpu, lower_red, upper_red)

    # Reduce the mask to find the position of the red line on the left and right sides of the frame
    mask_gpu_reduce = cvc.reduce(mask_gpu, 0, cv2.REDUCE_MAX)
    line1_gpu = cvc.reduce(mask_gpu_reduce[:, :200], 0, cv2.REDUCE_MAX)
    line2_gpu = cvc.reduce(mask_gpu_reduce[:, 200:], 0, cv2.REDUCE_MAX)

    # Download the results back to the CPU
    line1 = line1_gpu.download()
    line2 = line2_gpu.download()
    
    # Find the position of the red line on the left, right, bottom and top sides of the frame
    found_top_left = False
    found_top_right = False
    found_bottom_right = False
    found_bottom_left = False
    
    for y in range(240):
        _Line = line2[239-y]
        if _Line > 0:
            y2 = y
            found_top_right = True
            break  

    if found_top_right == False:
        for y in range(240):
            _Line = line1[y]
            if _Line > 0:
                y1 = y
                found_top_left = True
                break
            
    for y in range(240):
        _Line = line1[239-y]
    if _Line > 0:
        y4 = y
        found_bottom_left = True
        break
        
    if found_bottom_left == False:
        for y in range(240):
            _Line = line2[239-y]
            if _Line > 0:
                y3 = y
                found_bottom_right = True
                break

    if found_bottom_left == True:
        L = y4
        Line1 = 20
    elif found_bottom_right == True:
        L = y3
        Line1 = 120
    
    if found_top_right == True:
        R = y2
        Line2 = 300
    elif found_top_left == True:
        R = y1
        Line2 = 200

    diff = R - L
    tang = abs(diff) / 100

    if R > L:
        angle = (math.atan(tang) * 180 / math.pi)
    else:
        angle = 0 - (math.atan(tang) * 180 / math.pi)

    font = cv2.FONT_HERSHEY_SIMPLEX

    angle = round(angle, 5)
    text = str(angle)

    # Draw lines and angle text on the GPU
    mask_gpu = cvc.line(mask_gpu, (20,0), (20,240), (255,255,255), 2)
    mask_gpu = cvc.line(mask_gpu, (120,0), (120,240), (255,255,255), 2)
    mask_gpu = cvc.line(mask_gpu, (200,0), (200,240), (255,255,255), 2)
    mask_gpu = cvc.line(mask_gpu, (300,0), (300,240), (255,255,255), 2)

    frame_gpu = cvc.line(frame_gpu, (20,0), (20,240), (255,255,255), 2)
    frame_gpu = cvc.line(frame_gpu, (120,0), (120,240), (255,255,255), 2)
    frame_gpu = cvc.line(frame_gpu, (200,0), (200,240), (255,255,255), 2)
    frame_gpu = cvc.line(frame_gpu, (300,0), (300,240), (255,255,255), 2)

    frame_gpu = cvc.line(frame_gpu, (Line1, 239-L), (Line2, 239-R), (255,0,0), 3)

    text_gpu = cvc.putText(mask_gpu, text, (20,210), font, 1, (255,255,255),2,cv2.LINE_4)

    test_gpu = cvc.cvtColor(mask_gpu, cv2.COLOR_GRAY2BGR)

    test_gpu = cvc.line(test_gpu, (Line1, 239-L), (Line2, 239-R), (255,0,0), 3)

    # Concatenate the frames on the GPU
    frame_gpu[:, 320:, :] = test_gpu

    # Display the frame on the screen
    cv2.imshow('both', frame_gpu)

    toc = time.perf_counter()

    # Calculate the new motor speed with PID
    v = angle
    MV = pid(v)

    # Clip the motor speed to the range [1, 179]
    MV = np.clip(MV, 1, 179)

    # Set the servo position and motor speed
    kit.servo[8].angle = MV
    kit.continuous_servo[9].throttle = (abs(MV-90) / 90) / 3
        
    # Check if the user has pressed the 'q' key to exit the program
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
# Release the resources and close the windows
cap.release()
cv2.destroyAllWindows()
