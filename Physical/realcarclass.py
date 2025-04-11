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
        obot = Car('d:2:o','d:4:o','d:7:o','d:5:o','d:3:p','d:6:p','d:9:s')
        return(motor11,motor12,motor21,motor22,camera,Car)
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

    def setSpeed(self,speed):
        '''
        تنظیم سرعت خودرو
        ورودی:
            speed: عدد صحیح - مقدار سرعت مورد نظر
        عملکرد:
            - تنظیم مقدار سرعت در متغیر speed_value 
            - غیرفعال کردن حالت تصویر (image_mode = 0)
            - غیرفعال کردن وضعیت سنسور (sensor_status = 0) 
            - به‌روزرسانی داده‌ها با updateData()
            - ارسال داده‌ها به سرور از طریق سوکت
            - تاخیر 0.01 ثانیه‌ای
        '''
        self.speed_value = speed
        self.image_mode = 0
        self.sensor_status = 0
        self.updateData()
        self.sock.sendall(self.data_str.encode("utf-8"))
        time.sleep(0.01)

    def setSensorAngle(self, angle):
        '''
        تنظیم زاویه بین پرتوهای سنسور
        ورودی:
            angle: عدد صحیح - زاویه بر حسب درجه
        عملکرد:
            - غیرفعال کردن حالت تصویر
            - غیرفعال کردن وضعیت سنسور
            - تنظیم زاویه سنسور جدید
            - به‌روزرسانی و ارسال داده‌ها
        '''
        self.image_mode = 0
        self.sensor_status = 0
        self.sensor_angle = angle
        self.updateData()
        self.sock.sendall(self.data_str.encode("utf-8"))
        
    def getData(self):
        '''
        دریافت داده‌ها از شبیه‌ساز
        عملکرد:
            - فعال کردن حالت تصویر و سنسور
            - به‌روزرسانی داده‌ها
            - ارسال درخواست به سرور
            - دریافت و پردازش پاسخ شامل:
                * تصویر دوربین
                * داده‌های سنسور
                * سرعت فعلی
        '''
        self.image_mode = 1
        self.sensor_status = 1
        self.updateData()
        self.sock.sendall(self.data_str.encode("utf-8"))

        receive = self.recvall(self.sock)

        imageTagCheck = re.search('<image>(.*?)<\/image>', receive)
        sensorTagCheck = re.search('<sensor>(.*?)<\/sensor>', receive)
        speedTagCheck = re.search('<speed>(.*?)<\/speed>', receive)            
        
        try:
            if(imageTagCheck):
                imageData = imageTagCheck.group(1)
                im_bytes = base64.b64decode(imageData)
                im_arr = np.frombuffer(im_bytes, dtype=np.uint8)  # im_arr is one-dim Numpy array
                imageOpenCV = cv2.imdecode(im_arr, flags=cv2.IMREAD_COLOR)
                self.image = imageOpenCV
            
            if(sensorTagCheck):
                sensorData = sensorTagCheck.group(1)
                sensor_arr = re.findall("\d+", sensorData)
                sensor_int_arr = list(map(int, sensor_arr)) 
                self.sensors = sensor_int_arr
            else:
                self.sensors = [1500,1500,1500]

            if(speedTagCheck):
                current_sp = speedTagCheck.group(1)
                self.current_speed = int(current_sp)
            else:
                self.current_speed = 0
        except:
            print("Failed to receive data")


    def getImage(self):
        '''
        Returns the image from the camera
        '''
        return self.image

    def getSensors(self):
        '''
        Returns the sensor data
            A List: 
                [Left Sensor: int, Middle Sensor: int, Right Sensor: int]
        '''
        return self.sensors
    
    def getSpeed(self):
        '''
        Returns the speed of the car
        '''
        return self.current_speed
    
    def updateData(self):
        '''
        Updating the request data array and data string
        '''
        data = [self.speed_value,self.steering_value,self.image_mode,self.sensor_status,self.get_Speed, self.sensor_angle]
        self.data_str = self._data_format.format(data[0], data[1], data[2], data[3], data[4], data[5])
        
    def stop(self):
        '''
        Stoping the car and closing the socket
        '''
        self.setSpeed(0)
        self.setSteering(0)
        self.sock.sendall("stop".encode("utf-8"))
        self.sock.close()
        print("Process stopped successfully!")
    
    def __del__(self):
        self.stop()




