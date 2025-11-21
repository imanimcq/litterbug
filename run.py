# --- System and Core Libraries ---
import time
import math
import os
import sys
import argparse
import glob
import cv2
import numpy as np
import threading  # Added for non-blocking buzzer operation

# --- Hardware and Vision Libraries ---
from picamera2 import Picamera2
from ultralytics import YOLO

# --- Custom Hardware Modules (Assumed to be defined externally) ---  
from ultrasonic import Ultrasonic
from motor import Ordinary_Car
from servo import Servo
from infrared import Infrared
from adc import ADC
from buzzer import Buzzer

# --- CONFIGURATION ---
model_path = '/home/imanimcquay/Freenove_4WD_Smart_Car_Kit_for_Raspberry_Pi/Code/Server/mahica_best_ncnn_model'
img_source = 'picamera0' # NOTE: This variable is for reference only; Picamera2 is initialized below.
min_thresh = 0.65
user_res = "640x480"
record = False # Currently disabled

# Check if model file exists and is valid
if (not os.path.exists(model_path)):
    print('ERROR: Model path is invalid or model was not found. Make sure the model filename was entered correctly.')
    sys.exit(0)

# Load the model into memory and get labelmap
model = YOLO(model_path, task='detect')
labels = model.names


# --- CAR CLASS DEFINITION ---
class Car:
    def __init__(self):
        # Hardware component references
        self.servo = None
        self.sonic = None
        self.motor = None
        self.infrared = None
        self.adc = None
        self.buzzer = None
        
        self.car_record_time = time.time()
        self.car_sonic_servo_angle = 30
        self.car_sonic_servo_dir = 1
        self.car_sonic_distance = [30, 30, 30]
        
        # Buzzer threading attributes for non-blocking operation
        self.buzzer_thread = None  # Reference to the active buzzer thread
        self.buzzer_active = False  # Flag to track if buzzer is currently sounding
        self.buzzer_lock = threading.Lock()  # Thread-safe lock to prevent race conditions
        
        # Obstacle detection attributes
        self.last_distance_check = time.time()
        self.distance_check_interval = 0.1  # Check distance every 0.1 seconds (10 Hz)
        
        self.time_compensate = 3 #Depend on your own car,If you want to get the best out of the rotation mode, change the value by experimenting.
        self.start()
    
    def start(self):  
        if self.servo is None:
            self.servo = Servo()
            self.servo.set_servo_pwm('0', 30) # Servo faces forward
        if self.sonic is None:
            self.sonic = Ultrasonic()
        if self.motor is None:
            self.motor = Ordinary_Car()
        if self.adc is None:
            self.adc = ADC() 
        if self.buzzer is None:
            self.buzzer = Buzzer()

    def close(self):
        """Cleanup all hardware components and threads before shutdown"""
        # Stop motors immediately
        self.motor.set_motor_model(0,0,0,0)
        
        # Close hardware components
        self.sonic.close()
        self.motor.close()
        self.adc.close_i2c()
        
        # Ensure buzzer is off and wait for thread to complete
        if self.buzzer is not None:
            self.buzzer.set_state(False)
            # Wait for buzzer thread to finish (max 2.5 seconds)
            if self.buzzer_thread is not None and self.buzzer_thread.is_alive():
                self.buzzer_thread.join(timeout=2.5)
        
        # Clear all references
        self.servo = None
        self.sonic = None
        self.motor = None
        self.adc = None
        self.buzzer = None
    
 
    def run_motor_ultrasonic(self, distance):
        L, M, R = distance[0], distance[1], distance[2]

        # Print current distances
        print(f"[DISTANCES] Left: {L:.1f} cm | Middle: {M:.1f} cm | Right: {R:.1f} cm")

        # --- MAIN OBSTACLE CONDITION ---
        if (L < 30 and M < 30 and R < 30) or M < 30:
            print("[ACTION] Obstacle detected in FRONT. Reversing.")
            self.motor.set_motor_model(-1450,-1450,-1450,-1450) 
            time.sleep(0.1)

            if L < R:
                print("[TURN] Turning RIGHT after reversing.")
                self.motor.set_motor_model(1450,1450,-1450,-1450)
            else:
                print("[TURN] Turning LEFT after reversing.")
                self.motor.set_motor_model(-1450,-1450,1450,1450)

        # --- LEFT & MIDDLE BLOCKED ---
        elif L < 30 and M < 30:
            print("[ACTION] Obstacle on LEFT + FRONT → Turning RIGHT.")
            self.motor.set_motor_model(1500,1500,-1500,-1500)

        # --- RIGHT & MIDDLE BLOCKED ---
        elif R < 30 and M < 30:
            print("[ACTION] Obstacle on RIGHT + FRONT → Turning LEFT.")
            self.motor.set_motor_model(-1500,-1500,1500,1500)

        # --- LEFT CLOSE ---
        elif L < 20:
            print("[ACTION] Obstacle on LEFT → Sliding RIGHT.")
            self.motor.set_motor_model(2000,2000,-500,-500)
            if L < 10:
                print("[ACTION] LEFT extremely close → HARD RIGHT TURN.")
                self.motor.set_motor_model(1500,1500,-1000,-1000)

        # --- RIGHT CLOSE ---
        elif R < 20:
            print("[ACTION] Obstacle on RIGHT → Sliding LEFT.")
            self.motor.set_motor_model(-500,-500,2000,2000)
            if R < 10:
                print("[ACTION] RIGHT extremely close → HARD LEFT TURN.")
                self.motor.set_motor_model(-1500,-1500,1500,1500)

        # --- CLEAR PATH ---
        else:
            print("[ACTION] Path clear → Moving FORWARD.")
            self.motor.set_motor_model(600,600,600,600)



    def mode_ultrasonic(self):
        if (time.time() - self.car_record_time) > 0.25:
            self.car_record_time = time.time()
            self.servo.set_servo_pwm('0', self.car_sonic_servo_angle)

            # Read distances based on servo angle
            if self.car_sonic_servo_angle == 30:
                self.car_sonic_distance[0] = self.sonic.get_distance()
            elif self.car_sonic_servo_angle == 90:
                self.car_sonic_distance[1] = self.sonic.get_distance()
            elif self.car_sonic_servo_angle == 150:
                self.car_sonic_distance[2] = self.sonic.get_distance()

            # Print current scan angle
            print(f"[SCAN] Servo at {self.car_sonic_servo_angle}° → "
                f"L:{self.car_sonic_distance[0]:.1f}  "
                f"M:{self.car_sonic_distance[1]:.1f}  "
                f"R:{self.car_sonic_distance[2]:.1f}")

            # Motor decision
            self.run_motor_ultrasonic(self.car_sonic_distance)

            # Sweep direction update
            if self.car_sonic_servo_angle <= 30:
                self.car_sonic_servo_dir = 1
            elif self.car_sonic_servo_angle >= 150:
                self.car_sonic_servo_dir = 0
            
            if self.car_sonic_servo_dir == 1:
                self.car_sonic_servo_angle += 60
            elif self.car_sonic_servo_dir == 0:
                self.car_sonic_servo_angle -= 60


    def detect_object(self, num_object):
        if num_object <= 0:
            return
        else: 
            self.motor.set_motor_model(0, 0, 0, 0)
            time.sleep(5.0)

    
    def sound_buzzer(self, duration=2.0):
        """
        Activate buzzer for specified duration without blocking the main program.
        Uses threading to run buzzer in background, allowing robot to continue
        moving and processing camera frames simultaneously.
        
        Args:
            duration (float): How long to sound the buzzer in seconds (default: 2.0)
        
        Returns:
            bool: True if buzzer started, False if already active
        """
        # Thread-safe check if buzzer is available and not already running
        with self.buzzer_lock:
            if self.buzzer is None:
                return False
            
            if self.buzzer_active:
                # Buzzer already sounding, skip to avoid overlap
                return False
            
            # Mark buzzer as active before starting thread
            self.buzzer_active = True
        
        # Create and start background thread for buzzer
        self.buzzer_thread = threading.Thread(
            target=self._buzzer_worker, 
            args=(duration,), 
            daemon=True  # Thread will auto-terminate when main program exits
        )
        self.buzzer_thread.start()
        return True
    
    def _buzzer_worker(self, duration):
        """
        Internal worker function that runs in a separate thread.
        This function does the actual buzzer on/off control without blocking.
        
        Args:
            duration (float): How long to keep buzzer on in seconds
        """
        try:
            # Turn buzzer on
            self.buzzer.set_state(True)
            
            # Sleep in this background thread (doesn't block main program)
            time.sleep(duration)
            
            # Turn buzzer off
            self.buzzer.set_state(False)
            
        except Exception as e:
            # Handle any errors gracefully
            print(f"Buzzer error: {e}")
            try:
                self.buzzer.set_state(False)
            except:
                pass
        finally:
            # Always mark buzzer as inactive when done
            with self.buzzer_lock:
                self.buzzer_active = False

# --- RESOLUTION SETUP ---
resW, resH = int(user_res.split('x')[0]), int(user_res.split('x')[1])

# --- CAMERA INITIALIZATION (Picamera2) ---
print(f"Initializing Picamera2 at resolution: {resW}x{resH}...")
cap = Picamera2()
cap.configure(cap.create_video_configuration(main={"format": 'RGB888', "size": (resW, resH)}))
cap.start()
print("Camera stream started.")

# --- VISUALIZATION SETUP ---
# Set bounding box colors (using the Tableu 10 color scheme)
bbox_colors = [(164,120,87), (68,148,228), (93,97,209), (178,182,133), (88,159,106), 
              (96,202,231), (159,124,168), (169,162,241), (98,118,150), (172,176,184)]

# Initialize control and status variables
avg_frame_rate = 0
frame_rate_buffer = []
fps_avg_len = 200 # Length of the buffer to average FPS over
img_count = 0

# Initialize Car (creates buzzer internally)
car = Car()

# --- MAIN PROCESSING LOOP ---
try: 
    while True: 
        
        car.mode_ultrasonic()

        # Capture frame-by-frame
        t_start = time.perf_counter()
        frame = cap.capture_array()
        if (frame is None):
            print('Unable to read frames from the Picamera. This indicates the camera is disconnected or not working. Exiting program.')
            break
        frame = cv2.resize(frame,(resW,resH))

        results = model(frame, verbose=False)

        # Extract results
        detections = results[0].boxes

        # Initialize variable for basic object counting example
        object_count = 0

        # Go through each detection and get bbox coords, confidence, and class
        for i in range(len(detections)):

            # Get bounding box coordinates
            # Ultralytics returns results in Tensor format, which have to be converted to a regular Python array
            xyxy_tensor = detections[i].xyxy.cpu() # Detections in Tensor format in CPU memory
            xyxy = xyxy_tensor.numpy().squeeze() # Convert tensors to Numpy array
            xmin, ymin, xmax, ymax = xyxy.astype(int) # Extract individual coordinates and convert to int

            # Get bounding box class ID and name
            classidx = int(detections[i].cls.item())
            classname = labels[classidx]

            # Get bounding box confidence
            conf = detections[i].conf.item()

            # Draw box if confidence threshold is high enough
            if conf > min_thresh:

                color = bbox_colors[classidx % 10]
                cv2.rectangle(frame, (xmin,ymin), (xmax,ymax), color, 2)

                label = f'{classname}: {int(conf*100)}%'
                labelSize, baseLine = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1) # Get font size
                label_ymin = max(ymin, labelSize[1] + 10) # Make sure not to draw label too close to top of window
                cv2.rectangle(frame, (xmin, label_ymin-labelSize[1]-10), (xmin+labelSize[0], label_ymin+baseLine-10), color, cv2.FILLED) # Draw white box to put label text in
                cv2.putText(frame, label, (xmin, label_ymin-7), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1) # Draw label text

                # Basic example: count the number of objects in the image
                object_count = object_count + 1

        # Sound buzzer ONCE per frame if any objects detected
        # This now runs in background thread, so it won't slow down the main loop
        if object_count > 0:
            buzzer_started = car.sound_buzzer(2.0)
            car.detect_object(object_count)
            if buzzer_started:
                print(f"Object(s) detected! Count: {object_count}. Buzzer activated.")
            else:
                print(f"Object(s) detected! Count: {object_count}. (Buzzer already active)")

        cv2.putText(frame, f'FPS: {avg_frame_rate:0.2f}', (10,20), cv2.FONT_HERSHEY_SIMPLEX, .7, (0,255,255), 2) # Draw framerate
        # Display detection results
        cv2.putText(frame, f'Number of objects: {object_count}', (10,40), cv2.FONT_HERSHEY_SIMPLEX, .7, (0,255,255), 2) # Draw total number of detected objects
        cv2.imshow('YOLO detection results',frame) # Display image
        key = cv2.waitKey(5)

        if key == ord('q') or key == ord('Q'): # Press 'q' to quit
            break
        elif key == ord('s') or key == ord('S'): # Press 's' to pause inference
            cv2.waitKey()
        elif key == ord('p') or key == ord('P'): # Press 'p' to save a picture of results on this frame
            cv2.imwrite('capture.png',frame)

        # Calculate FPS for this frame
        t_stop = time.perf_counter()
        frame_rate_calc = float(1/(t_stop - t_start))

        # Append FPS result to frame_rate_buffer (for finding average FPS over multiple frames)
        if len(frame_rate_buffer) >= fps_avg_len:
            temp = frame_rate_buffer.pop(0)
            frame_rate_buffer.append(frame_rate_calc)
        else:
            frame_rate_buffer.append(frame_rate_calc)

        # Calculate average FPS for past frames
        avg_frame_rate = np.mean(frame_rate_buffer)

        # print(f'Average pipeline FPS: {avg_frame_rate:.2f}')

except KeyboardInterrupt:
    car.close()
    cap.stop()
    cv2.destroyAllWindows()
    print("\nProgram interrupted by user.")
except Exception as e:
    print(f"An unexpected error occurred in the main loop: {e}")
    car.close()
    cap.stop()
    cv2.destroyAllWindows()
