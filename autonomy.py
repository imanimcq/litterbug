from Server.motor import Ordinary_Car
from Server.buzzer import Buzzer
from Server.servo import Servo
from Server.car import Car
from Server.test import *
from Server.ultrasonic import Ultrasonic
import cv2
import time
from ultralytics import YOLO
MODEL_PATH = "yolo/best_ncnn_model"

if (not os.path.exists(MODEL_PATH)):
    print('ERROR: Model path is invalid or model was not found. Make sure the model filename was entered correctly.')
    sys.exit(0)

model = YOLO(MODEL_PATH)

car = Car()
motor = Ordinary_Car()
ultrasonic = Ultrasonic()
buzzer = Buzzer()

FRAME_WIDTH = 640
FRAME_HEIGHT = 640

MOTOR_SPEED = 2000

cap = cv2.VideoCapture(0)
cap.set(3, FRAME_WIDTH)
cap.set(4, FRAME_HEIGHT)



def detect(frame):
    """Run YOLO and return largest box or None."""
    results = model(frame, verbose=False)
    boxes = results[0].boxes
    if len(boxes) == 0:
        return None

    largest = None
    max_area = 0

    for b in boxes:
        x1, y1, x2, y2 = b.xyxy[0]
        area = (x2 - x1) * (y2 - y1)
        if area > max_area:
            max_area = area
            largest = (x1, y1, x2, y2, area)

    return largest


def Servo_sweep():
    import time
    servo = Servo()
    try:
        print ("Program is starting ...")
        while True:
            for i in range(50,110,1):
                servo.set_servo_pwm('0',i)
                time.sleep(0.01)
                ret, frame = cap.read()
                if not ret:
                    continue
                box = detect(frame)
                if box:
                    print(f"Object found at servo angle {i}")
                    return i, box
            for i in range(110,50,-1):
                servo.set_servo_pwm('0',i)
                time.sleep(0.01)
                ret, frame = cap.read()
                if not ret:
                    continue
                box = detect(frame)
                if box:
                    print(f"Object found at servo angle {i}")
                    return i, box
            for i in range(80,150,1):
                servo.set_servo_pwm('1',i)
                time.sleep(0.01)
                ret, frame = cap.read()
                if not ret:
                    continue
                box = detect(frame)
                if box:
                    print(f"Object found at servo angle {i}")
                    return i, box
            for i in range(150,80,-1):
                servo.set_servo_pwm('1',i)
                time.sleep(0.01)
                ret, frame = cap.read()
                if not ret:
                    continue
                box = detect(frame)
                if box:
                    print(f"Object found at servo angle {i}")
                    return i, box   
    except KeyboardInterrupt:
        servo.set_servo_pwm('0',90)
        servo.set_servo_pwm('1',90)

while True:
    angle, box = Servo_sweep() #servo turns to detect object 

    if box is None:
        print("No object found in scan → rotating body to search...")
        car.mode_rotate(0)
        continue

    servoAngle = angle

    #ALIGN BODY TO SERVO DIRECTION

    if servoAngle < 90 - 10:
        print("Object is left → turning robot left")
        car.mode_rotate(90)
    elif servoAngle > 90 + 10:
        print("Object is right → turning robot right")
        car.mode_rotate(0)
    else:
        print("Object ahead → checking approach distance")
        distance = ultrasonic.get_distance()
        if distance is not None:
                    print(f"Ultrasonic distance: {distance}cm")  # Print the distance measurement
    
    time.sleep(0.4)
    motor.set_motor_model(0,0,0,0)

    # Get a fresh frame while facing object
    ret, frame = cap.read()
    box = detect(frame)
    if box is None:
        continue

    x1, y1, x2, y2, area = box

    print("Approaching target…")
    while True:
                distance = ultrasonic.get_distance()  # Get the distance measurement in centimeters
                if distance is not None:
                    print(f"Ultrasonic distance: {distance}cm")  # Print the distance measurement
                    motor.set_motor_model(MOTOR_SPEED, MOTOR_SPEED,MOTOR_SPEED, MOTOR_SPEED)
                    if distance < 30: # 1 foot away
                        motor.set_motor_model(0,0,0,0)
                        time.sleep(0.2)
                        buzzer.set_state(True)
                        time.sleep(0.5)
                        buzzer.set_state(False) 
                continue


piuu
                         
   
   


