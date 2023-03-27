 #Changes to revisedV1_RTFF.py
 * Used the cv2.cuda module to perform some image processing operations on the GPU.
 * Replaced the loop that scans for the position of the red line on the left and right sides of the frame with, 
 * cv2.cuda.reduce() calls to reduce the mask image to one-dimensional arrays representing the position of the line. (cvc.reduce)
 * Added GPU-accelerated drawing of lines and text to the mask and frame images.
 * Used np.clip() to limit the output of the PID controller to the range [1, 179] before setting the servo position.
 * Cleared 'unnecesary' comments and code. Cross reference the original code for more details. 'completeRTTF.py'
 * PID not fully functional atm.


 Required packages to install on: Jetson Nano
 pip install opencv-python-headless
 pip install numpy
 pip install simple-pid
 pip install adafruit-circuitpython-servokit

