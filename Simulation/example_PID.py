"""
@ 2023, Copyright AVIS Engine
- An Example Compatible with AVISEngine version 2.0.1 / 1.2.4 (ACL Branch) or higher
"""

import avisengine
import config
import time
import cv2
import numpy as np

# Creating an instance of the Car class
car = avisengine.Car()

# Connecting to the server (Simulator)
car.connect(config.SIMULATOR_IP, config.SIMULATOR_PORT)

# Counter variable
counter = 0

debug_mode = True


# PID Controller (Proportional integral derivative): it's trying to decreese error, the error is the steering angle errors
class PID:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp      # ضریب تناسبی
        self.Ki = Ki      # ضریب انتگرالی
        self.Kd = Kd      # ضریب مشتقی
        self.integral = 0.0
        self.prev_error = 0.0

    def update(self, error):
        # Proportional
        P = self.Kp * error
        # integral: انتگرال همان عمل جمع است. با توجه به اینکه ما در هر لحظه تابع را فراخوانی میکنیم نیازی به ضرب دیفرانسیل زمان نیست!
        self.integral += error
        I = self.Ki * self.integral
        # Derivative: در مشتق هم نیازی به تقسیم دیفرانسیل زمان نداریم
        derivative = (error - self.prev_error)
        D = self.Kd * derivative
        self.prev_error = error
        # خروجی خام PID (این خروجی ممکن است بسیار بزرگ شود)
        raw_output = P + I + D
        return raw_output


# یک تبدیل هندسی است که یک ذوزنقه را روی کل عکس می‌کشد و سپس تصویر را به اندازۀ یک مستطیل می‌کشد تا تصویر به گونه ای شود که انگار یک پرنده از بالا در حال نگاه کردن است
# به این نما، نمای بالا سر پرنده میگویند
# bird eye view (BEV)
def bird_eye_view_transform(image, src_points, dst_points, output_size):
    transformed = cv2.getPerspectiveTransform(src_points, dst_points)
    warped = cv2.warpPerspective(image, transformed, output_size)
    return warped # شکل تبدیل شده

# استفاده از اچ‌اس‌وی برای پیدا کردن خطوط سفید در تصویر 
def hsv_mask(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    # حد بالا و پایین سفید بودن رنگ خطوط
    lower_white = np.array([0, 0, 200])
    upper_white = np.array([180, 30, 255])
    mask = cv2.inRange(hsv, lower_white, upper_white)
    return mask


def detect_edges(image):
    blurred = cv2.GaussianBlur(image, (7, 7), 3)
    edges = cv2.Canny(blurred, 50, 150)
    return edges # اینو دیگه میدونین :)



def detect_lines_hough(
    edges, rho=1, theta=np.pi / 180, threshold=50, min_line_length=50, max_line_gap=20
):
    lines = cv2.HoughLinesP(
        edges,
        rho,
        theta,
        threshold,
        np.array([]),
        minLineLength=min_line_length,
        maxLineGap=max_line_gap,
    )
    return lines # اینم دیگه میدونین ;)


# محاسبه میانگین نقاط خطوط در محور افقی برای برازش نقطه ای روی خط سمت راست جاده
def compute_right_lane_point(lines, y_target, image_width):
    right_x = []
    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                # انتخاب خطوطی که احتمالاً مربوط به خط راست هستند؛ فرض بر این است که آنها در سمت راست قرار دارند.
                # لذا قطعا آنها در سه چهارم دیگر صفحه هستند
                if x1 > image_width / 4 and x2 > image_width / 4:
                    # اگر خط از y_target عبور می‌کند (تفاوت علامتی)
                    if (y1 - y_target) * (y2 - y_target) <= 0:
                        # محاسبه نقطه‌ی تقاطع خط با y_target
                        if (y2 - y1) != 0:
                            x_intersect = x1 + (x2 - x1) * ((y_target - y1) / (y2 - y1))
                            right_x.append(x_intersect)
    if len(right_x) == 0:
        return image_width + 10  # در صورت عدم شناسایی، فرض می‌کنیم خط در سمت راست تصویر قرار دارد و مقداری را هم به آن اضافه میکنیم
    return np.mean(right_x) # در نهایت میانگین میگیریم


<<<<<<< Updated upstream
=======
def compute_left_lane_point(lines, y_target, image_width):
    left_x = []
    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                # انتخاب خطوطی که احتمالاً مربوط به خط راست هستند؛ فرض بر این است که آنها در سمت راست قرار دارند.
                # لذا قطعا آنها در سه چهارم دیگر صفحه هستند
                if x1 < image_width * (0.75) and x2 < image_width * (0.75):
                    # اگر خط از y_target عبور می‌کند (تفاوت علامتی)
                    if (y1 - y_target) * (y2 - y_target) <= 0:
                        # محاسبه نقطه‌ی تقاطع خط با y_target
                        if (y2 - y1) != 0:
                            x_intersect = x1 + (x2 - x1) * ((y_target - y1) / (y2 - y1))
                            left_x.append(x_intersect)
    if len(left_x) == 0:
        return 10  # در صورت عدم شناسایی، فرض می‌کنیم خط در سمت راست تصویر قرار دارد و مقداری را هم به آن اضافه میکنیم
    return np.mean(left_x) # در نهایت میانگین میگیریم

pid = PID(Kp=4, Ki=0.005, Kd=2.5)
>>>>>>> Stashed changes
# تابع اصلی پردازش تصویر و محاسبه زاویه فرمان
def process_image_and_compute_steering(image, offset=50):
    """
    - offset: فاصله مطلوب از خط راست جاده به پیکسل (به گونه‌ای تنظیم می‌شود که خودرو در فاصله‌ای امن نسبت به خط راست قرار گیرد)
    """
    if image is None:
        print("تصویر بارگذاری نشد.")
        return None

    height, width = image.shape[:2]

    obstacles : bool = (offset > 0)
    
    # با فیلتر اچ اس وی دیگر نیازی به خاکستری کردن رنگ نیست
    hsv_filtered = hsv_mask(image)

    # مرحله ۲: تبدیل به نمای بالای سر
    # نقاط منبع و مقصد باید بر اساس تنظیمات دوربین و ROI انتخاب شوند.
    src_points = np.float32(
        [
            [
<<<<<<< Updated upstream
                (width // 2 - 85, (height // 2) - 28),
                ((width // 2) + 85, (height // 2) - 28),
=======
                (width // 2 - 80, (height // 2)),
                ((width // 2) + 85, (height // 2)),
>>>>>>> Stashed changes
                (width, height - 100),
                (0, height - 100),
            ]
        ]
    )
    dst_points = np.float32([[0, 0], [width, 0], [width, height], [0, height]])
    bev_image = bird_eye_view_transform(
        hsv_filtered, src_points, dst_points, (width, height)
    )

    edges = detect_edges(bev_image)


    lines = detect_lines_hough(edges)
    if lines is None:
        print("هیچ خطی شناسایی نشد.")

    y_target = int(height * 0.875)
    right_lane_point = compute_left_lane_point(lines, y_target, width) if obstacles else compute_right_lane_point(lines, y_target, width)

    # موقعیت مطلوب خودرو: فاصله ایمن از خط راست (خط مطلوب = نقطه روی خط راست - offset)
    desired_position = right_lane_point + offset

    # مرکز تصویر به عنوان موقعیت فعلی خودرو
    center_position = width / 2.0

    # خطای مسیر (CTE) برابر با اختلاف بین موقعیت مطلوب و مرکز تصویر
    cte = desired_position - center_position

    # مرحله ۵: استفاده از کنترل‌کننده PID برای محاسبه زاویه فرمان بر اساس CTE
    pid = PID(Kp=0.6, Ki=0.01, Kd=2.0)
    steering_angle = pid.update(np.arctan(cte / y_target) * (180 / np.pi))

    # نمایش نتایج برای دیداری کردن
    output = cv2.cvtColor(bev_image, cv2.COLOR_GRAY2BGR)
    # رسم خطوط تشخیص داده شده
    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(output, (x1, y1), (x2, y2), (0, 255, 0), 2)
    # رسم نقطه روی خط راست
    cv2.circle(output, (int(right_lane_point), y_target), 5, (0, 0, 255), -1)
    # رسم خط مرکز تصویر
    cv2.line(
        output,
        (int(center_position), y_target),
        (int(center_position), height),
        (255, 0, 0),
        2,
    )
    # رسم نقطه موقعیت مطلوب خودرو
    cv2.circle(output, (int(desired_position), y_target), 5, (255, 0, 255), -1)
    cv2.imshow("output", output)
    print("CTE:", cte)
    print("PID:", steering_angle)
    return steering_angle


# Sleep for 3 seconds to make sure that client connected to the simulator
time.sleep(3)


try:
    prev_steering = 1
    while True:
        # Counting the loops
        counter = counter + 1

        # Set the power of the engine the car to 20, Negative number for reverse move, Range [-100,100]
        car.setSpeed(3000)

        # Set the Steering of the car -10 degree from center, results the car to steer to the left
        # car.setSteering(0)

        # Set the angle between sensor rays to 45 degrees, Use this only if you want to set it from python client
        # Notice: Once it is set from the client, it cannot be changed using the GUI
        car.setSensorAngle(1)

        # Get the data. Need to call it every time getting image and sensor data
        car.getData()

        # Start getting image and sensor data after 4 loops
        if counter > 4:
            counter = counter + 1
            # Returns a list with three items which the 1st one is Left sensor data\
            # the 2nd one is the Middle Sensor data, and the 3rd is the Right one.
            sensors = car.getSensors()

            image = car.getImage()

            m, r= sensors[1], sensors[2]
            kir = -185

            if m <= 1499 or r <= 1200:
                car.setSensorAngle(10)
                kir = 185
            else:
                kir = -185

            steering = process_image_and_compute_steering(image, kir)
            
            if abs(steering - prev_steering) > 2.5 or abs(steering) > 20:
<<<<<<< Updated upstream
                car.setSpeed(20)
=======
                car.setSpeed(50)
>>>>>>> Stashed changes
                print("stop")
            else:
                car.setSpeed(3000)
            prev_steering = steering

            
            car.setSteering(((steering / 90.0) * 100))

            # Showing the opencv type image

            if cv2.waitKey(10) == ord("q"):
                break

            
finally:
    car.stop()
