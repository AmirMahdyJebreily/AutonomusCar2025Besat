import time
from pyfirmata import Arduino,util
from time import sleep
from pyfirmata.pyfirmata import Board

port = Arduino('com3')

# motor11 = port.get_pin('d:2:o')
# motor12= port.get_pin('d:4:o')
# motor21 = port.get_pin('d:7:o')
# motor22 = port.get_pin('d:5:o')
#
# ina1 = port.get_pin('d:3:p')
# ina2 = port.get_pin('d:6:p')
#
# Servo = port.get_pin('d:9:s')

class control():
    def __init__(self,motor11,motor12,motor21,motor22,ina1,ina2,Servo):
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
        return(motor11,motor12,motor21,motor22,camera)
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


# while True:
#     x = int(input("m : "))
#     if x == 1:
#         robot.right()
#     elif x == 2:
#         robot.back()
#     elif x == 3:
#         robot.forward()
#     elif x == 4:
#         robot.left()

