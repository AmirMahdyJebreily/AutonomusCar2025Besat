import pyfirmata
import RPi.GPIO
from pyfirmata import Arduino,util
from pyfirmata.pyfirmata import Board
import cv2
import time
import numpy as np
import oop 
class Car():
    def __init__(self,motor11,motor12,motor21,motor22,ina1,ina2,Servo):
        port = Arduino('com3')
        self.motor11 = port.get_pin(motor11)
        self.motor12 = port.get_pin(motor12)
        self.motor21 = port.get_pin(motor21)
        self.motor22 = port.get_pin(motor22)
        self.ina1 = port.get_pin(ina1)
        self.ina2 = port.get_pin(ina2)
        self.Servo = port.get_pin(Servo)
        camera = cv2.VideoCapture(0)
        camera.set(cv2.CAP_PROP_FRAME_WIDTH,512)
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT,512)
        camera.set(cv2.CAP_PROP_FPS,60)
        self.TRING_LEFT = 23
        self.ECHO_LEFT = 24   
        self.TRIG_MID = 17    
        self.ECHO_MID = 27    
        self.TRIG_RIGHT = 22  
        self.ECHO_RIGHT = 10
        RPi.GPIO.setmode(RPi.GPIO.BCM)
        for trig in [self.TRIG_LEFT, self.TRIG_MID, self.TRIG_RIGHT]:
            RPi.GPIO.setup(trig, RPi.GPIO.OUT)
            RPi.GPIO.output(trig, False)
        for echo in [self.ECHO_LEFT, self.ECHO_MID, self.ECHO_RIGHT]:
            RPi.GPIO.setup(echo, RPi.GPIO.IN)
        return(motor11,motor12,motor21,motor22,camera,Servo)
    def forward(self,speed1=1,speed2=1,servo=90):
        self.motor11.write(1)
        self.motor12.write(0)
        self.motor21.write(0)
        self.motor22.write(1)
        self.ina1.write(speed1)
        self.ina2.write(speed2)
        self.Servo.write(servo)
    def back(self,speed1=1,speed2=1,servo=90):
        self.motor11.write(0)
        self.motor12.write(1)
        self.motor21.write(1)
        self.motor22.write(0)
        self.ina1.write(speed1)
        self.ina2.write(speed2)
        self.Servo.write(servo)
    def right(self,speed1=.8,speed2=.6,servo=110):
        self.motor11.write(1)
        self.motor12.write(0)
        self.motor21.write(0)
        self.motor22.write(1)
        self.ina1.write(speed1)
        self.ina2.write(speed2)
        self.Servo.write(servo)
    def left(self,speed1=.6,speed2=.8,servo=70):
        self.motor11.write(1)
        self.motor12.write(0)
        self.motor21.write(0)
        self.motor22.write(1)
        self.ina1.write(speed1)
        self.ina2.write(speed2)
        self.Servo.write(servo)
    def stop(self,speed1=0,speed2=0,servo=90):
        self.motor11.write(1)
        self.motor12.write(1)
        self.motor21.write(1)
        self.motor22.write(1)
        self.ina1.write(speed1)
        self.ina2.write(speed2)
        self.Servo.write(servo)
    #Attributes to kind of replicate a Pub-sub pattern messaging to request data  
    steering_value = 0
    speed_value = 0
    sensor_status = 1
    image_mode = 1
    get_Speed = 1
    sensor_angle = 30
    image =  None
    sensors = None
    current_speed = None
def setSteering(self, steering):
    # Limit steering input to -100 to +100
    if steering > 100:
        steering = 100
    elif steering < -100:
        steering = -100
    if steering >= 0:
        # Map 0 to +100 range to 90 to 70 degrees (right)
        servo_angle = 90 - (steering * 0.2)  # 0.2 = (90-70)/100
    else:
        # Map -100 to 0 range to 110 to 90 degrees (left) 
        servo_angle = 90 - (steering * 0.2)  # 0.2 = (110-90)/100
    self.Servo.write(servo_angle)
    self.steering_value = steering
    self.image_mode = 0
    self.sensor_status = 0
    self.updateData()
    self.sock.sendall(self.data_str.encode("utf-8"))
    time.sleep(0.01)
def setSpeed(self, speed):
    '''
    Sets the speed of both DC motors
    Parameters:
        speed: int between -100 (full reverse) and +100 (full forward)
        0 means stop
    '''
    # Limit speed input to -100 to +100
    if speed > 100:
        speed = 100
    elif speed < -100:
        speed = -100
        
    # Convert percentage to 0-1 range for motor control
    speed_value = abs(speed) / 100.0
    
    if speed >= 0:  # Forward
        self.motor11.write(1)
        self.motor12.write(0)
        self.motor21.write(0)
        self.motor22.write(1)
    else:  # Backward
        self.motor11.write(0)
        self.motor12.write(1)
        self.motor21.write(1)
        self.motor22.write(0)
    
    # Set motor speeds
    self.ina1.write(speed_value)  # Left motor
    self.ina2.write(speed_value)  # Right motor
    
    self.speed_value = speed
    self.image_mode = 0
    self.sensor_status = 0
    self.updateData()
    self.sock.sendall(self.data_str.encode("utf-8"))
    time.sleep(0.01)

def measure_distance(self, trigger, echo):
    RPi.GPIO.output(trigger, True)
    time.sleep(0.00001)
    RPi.GPIO.output(trigger, False)

    pulse_start = time.time()
    timeout_start = time.time()
    while RPi.GPIO.input(echo) == 0:
        pulse_start = time.time()
        if time.time() - timeout_start > 0.1:  # timeout after 100ms
            return 400.0
    pulse_end = time.time()
    while RPi.GPIO.input(echo) == 1:
        pulse_end = time.time()
        if time.time() - pulse_start > 0.1:  # timeout after 100ms
            return 400.0
    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17150  
    
    return round(min(distance, 100.0), 1)  
    def getImage(self):
        ret, frame = self.camera.read()
        if ret:
            return frame
        return None
    def getSensors(self):
        time.sleep(0.05) 
        left = self.measure_distance(self.TRIG_LEFT, self.ECHO_LEFT)
        middle = self.measure_distance(self.TRIG_MID, self.ECHO_MID)
        right = self.measure_distance(self.TRIG_RIGHT, self.ECHO_RIGHT)
    return [left, middle, right]
    def getSpeed(self):
        return self.current_speed
    def stop(self):
        stop()
        print("Process stopped successfully!")
    def __del__(self):
        self.stop()
        RPi.GPIO.cleanup()



