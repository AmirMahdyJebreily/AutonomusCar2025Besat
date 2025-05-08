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

# # تابع شبیه‌سازی دریافت فاصله از مرکز خودرو نسبت به مرجع (d_ref)
# def get_distance_from_road():
#     # در سیستم واقعی، این داده از دوربین یا حسگرهای دیگر خوانده می‌شود.
#     return 1.0  # متر (مثال)

# # تابع شبیه‌سازی دریافت موقعیت خط سمت راست جاده (r_right)
# def get_right_road_edge():
#     return 3.0  # متر (مثال)

# def initialize_kalman(dt, process_noise, measurement_noise):
#     """
#     مقداردهی اولیه فیلتر کالمن برای بردار حالت [CTE, cte_dot].
#     ابعاد: dim_x=2 و dim_z=1.
#     """
#     kf = KalmanFilter(dim_x=2, dim_z=1)
    
#     # ماتریس انتقال حالت: مدل خطی ساده
#     # e_{k+1} = e_k + dt * cte_dot_k
#     # cte_dot_{k+1} = cte_dot_k
#     kf.F = np.array([[1, dt],
#                      [0,  1]])
    
#     # ورودی کنترل نداریم؛ بنابراین B صفر است
#     kf.B = np.zeros((2, 1))
    
#     # مدل اندازه‌گیری: حسگر فقط CTE (اولین مؤلفه) را اندازه می‌گیرد
#     kf.H = np.array([[1, 0]])
    
#     # ماتریس نویز فرآیندی Q: فرض می‌کنیم نویزها قطری هستند
#     # process_noise=[sigma_cte, sigma_cte_dot]
#     kf.Q = np.diag([process_noise[0]**2, process_noise[1]**2])
    
#     # ماتریس نویز اندازه‌گیری R
#     kf.R = np.array([[measurement_noise**2]])
    
#     # ماتریس کوواریانس اولیه P
#     kf.P = np.eye(2)
    
#     # حالت اولیه: فرض می‌کنیم CTE و cte_dot ابتدا صفر باشند.
#     kf.x = np.array([[0],
#                      [0]])
    
#     return kf

# def update_kalman(kf, measurement):
#     """
#     به‌روزرسانی فیلتر کالمن با اندازه‌گیری جدید (CTE).
#     """
#     kf.predict()
#     kf.update(np.array([[measurement]]))
#     return kf.x  # بازگرداندن تخمین حالت به صورت بردار ستونی


class FeedforwardWeightController:
    def __init__(
        self, ramp_up_speed=0.05, ramp_down_speed=0.05, max_weight=1.0, min_weight=0.0
    ):
        """
        ramp_up_speed: نرخ افزایش وزن در هر به‌روزرسانی (مثلاً 0.05)
        ramp_down_speed: نرخ کاهش وزن در هر به‌روزرسانی (مثلاً 0.05)
        max_weight: حداکثر وزنی که می‌توان به دست آورد (مثلاً 1.0 برای 100 درصد)
        min_weight: حداقل وزن (معمولاً 0.0)
        """
        self.weight = min_weight  # مقدار اولیه weight
        self.ramp_up_speed = ramp_up_speed
        self.ramp_down_speed = ramp_down_speed
        self.max_weight = max_weight
        self.min_weight = min_weight

    def update(self, obstacle_detected: bool):
        """
        این تابع در هر حلقه فراخوانی شده و وضعیت تشخیص مانع (obstacle_detected) را دریافت می‌کند.
        اگر مانع تشخیص داده شده باشد، وزن feedforward به تدریج افزایش می‌یابد،
        و در غیر این صورت، به آرامی کاهش می‌یابد.

        ورودی:
            obstacle_detected: یک مقدار بولی؛ True به معنای شناسایی مانع، False به معنای عدم شناسایی.

        خروجی:
            وزن feedforward جاری (مقدار بین min_weight تا max_weight)
        """
        if obstacle_detected:
            # افزایش وزن به سمت max_weight
            self.weight = min(self.weight + self.ramp_up_speed, self.max_weight)

        else:
            # کاهش وزن به سمت min_weight
            self.weight = max(self.weight - self.ramp_down_speed, self.min_weight)
            print("weight = ", self.weight)

        return self.weight


# PID Controller (Proportional integral derivative): it's trying to decreese error, the error is the steering angle errors
class PID:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp  # ضریب تناسبی
        self.Ki = Ki  # ضریب انتگرالی
        self.Kd = Kd  # ضریب مشتقی
        self.integral = 0.0
        self.prev_error = 0.0

    def update(self, error):
        # Proportional
        P = self.Kp * error
        # integral: انتگرال همان عمل جمع است. با توجه به اینکه ما در هر لحظه تابع را فراخوانی میکنیم نیازی به ضرب دیفرانسیل زمان نیست!
        self.integral += error
        I = self.Ki * self.integral
        # Derivative: در مشتق هم نیازی به تقسیم دیفرانسیل زمان نداریم
        derivative = error - self.prev_error
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
    return warped  # شکل تبدیل شده


# استفاده از اچ‌اس‌وی برای پیدا کردن خطوط سفید در تصویر
def hsv_mask(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # حد بالا و پایین سفید بودن رنگ خطوط
    lower_white = np.array([0, 0, 190], dtype=np.uint8)
    upper_white = np.array([180, 50, 255], dtype=np.uint8)
    mask = cv2.inRange(hsv, lower_white, upper_white)
    return mask


def detect_edges(image):
    blurred = cv2.GaussianBlur(image, (7, 7), 3)
    edges = cv2.Canny(blurred, 50, 150)
    return edges  # اینو دیگه میدونین :)


def detect_lines_hough(
    edges, rho=2, theta=np.pi / 180, threshold=70, min_line_length=50, max_line_gap=30
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
    return lines  # اینم دیگه میدونین ;)


# محاسبه میانگین نقاط خطوط در محور افقی برای برازش نقطه ای روی خط سمت راست جاده
def compute_right_lane_point(lines, y_target, right_lane_point, image_width):
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
                # else:
                #     return right_lane_point
    if len(right_x) == 0:
        return right_lane_point  # در صورت عدم شناسایی، فرض می‌کنیم خط در سمت راست تصویر قرار دارد و مقداری را هم به آن اضافه میکنیم
    res = np.mean(right_x) # در نهایت میانگین میگیریم
    return res


pid_r = PID(Kp=4, Ki=0.00000025, Kd=11)

dt = 0.001  # گام زمانی
process_noise = [0.1, 0.05]  # برای [CTE, cte_dot]
measurement_noise = 0.01
# Kf = initialize_kalman(dt, process_noise, measurement_noise)
# if not hasattr(Kf, "Kf"):
#     kf = initialize_kalman(dt, process_noise, measurement_noise)
#     Kf = kf


# تابع اصلی پردازش تصویر و محاسبه زاویه فرمان
def process_image_and_compute_steering(
    image, lane_point,cte_prev, y_target, offset=50, obstT: bool = False
):
    """
    - offset: فاصله مطلوب از خط راست جاده به پیکسل (به گونه‌ای تنظیم می‌شود که خودرو در فاصله‌ای امن نسبت به خط راست قرار گیرد)
    """
    if image is None:
        print("تصویر بارگذاری نشد.")
        return None

    height, width = image.shape[:2]

    # با فیلتر اچ اس وی دیگر نیازی به خاکستری کردن رنگ نیست
    hsv_filtered = hsv_mask(image)

    # مرحله ۲: تبدیل به نمای بالای سر
    # نقاط منبع و مقصد باید بر اساس تنظیمات دوربین و ROI انتخاب شوند.
    src_points = (
        np.float32(
            [
                [
                    ((width // 2) - 110 , (height // 2) + 50),
                    ((width // 2) + 200, (height // 2) + 50),
                    (width, height - 100),
                    (0, height - 100),
                ]  # This part of the code is performing the following operations:
            ]
        )
    )


    dst_points = np.float32([[0, 0], [width, 0], [width, height], [0, height]])
    bev_image = bird_eye_view_transform(
        hsv_filtered, src_points, dst_points, (width, height)
    )

    edges = detect_edges(bev_image)

    lines = detect_lines_hough(edges)

    lane_point = (
        compute_right_lane_point(lines, y_target, lane_point, width)
    )

    # موقعیت مطلوب خودرو: فاصله ایمن از خط راست (خط مطلوب = نقطه روی خط راست - offset)
    desired_position = lane_point + offset

    # مرکز تصویر به عنوان موقعیت فعلی خودرو
    center_position = width / 2.0

    # خطای مسیر (CTE) برابر با اختلاف بین موقعیت مطلوب و مرکز تصویر
    cte = desired_position - center_position

    # # به‌روزرسانی فیلتر کالمن با اندازه‌گیری cte
    # Kf.predict()
    # Kf.update(np.array([[cte]]))
    # cte_est = kf.x[0, 0]  # تخمین صاف شده CTE
    # *** END: ادغام فیلتر کالمن ***



    # مرحله کنترل: استفاده از کنترل‌کننده PID برای محاسبه فرمان feedback بر مبنای تخمین CTE
    # در این مثال، اگر obstT True باشد از pid_l استفاده می‌کنیم، در غیر این صورت از pid_r.
    diffrence = pid_r.update(cte)
    steering_angle = np.arctan(diffrence / y_target) * (180 / np.pi)

    # نمایش نتایج برای دیداری کردن
    output = cv2.cvtColor(bev_image, cv2.COLOR_GRAY2BGR)
    # رسم خطوط تشخیص داده شده
    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(output, (x1, y1), (x2, y2), (0, 255, 0), 2)
    # رسم نقطه روی خط راست
    cv2.circle(output, (int(lane_point), y_target), 5, (0, 0, 255), -1)
    # رسم خط مرکز تصویر
    cv2.line(
        output,
        (int(center_position), y_target),
        (int(center_position), height),
        (255, 0, 0),
        2,
    )

    cv2.line(
        output,
        (int(center_position), y_target),
        (int(desired_position), y_target),
        (0, 255, 0),
        2,
    )

    cv2.line(
        output,
        (int(center_position), height),
        (int(desired_position), y_target),
        (0, 0, 255),
        2,
    )

    un_bev = bird_eye_view_transform(output, dst_points, src_points, (width, height))

    # ۳. ترکیب با وزن – مثل overlay
    alpha = 0.5  # میزان شفافیت لایه بالا (HSV)
    beta = 0.9 - alpha

    overlay = cv2.addWeighted(image, beta, un_bev, alpha, 1)

    # رسم نقطه موقعیت مطلوب خودرو
    cv2.circle(output, (int(desired_position), y_target), 5, (255, 0, 255), -1)
    cv2.imshow("output", output)
    cv2.imshow("edges", edges)
    cv2.imshow("output1", overlay)
    # print("CTE:", cte)
    # print("PID:", steering_angle)
    return steering_angle, lane_point, cte


def get_steering_at_progress(
    x,
    L_eff,
    A,
    v,
    tolerance=0.3,
    alpha=0.5,
    k=2.0,
    v_thresh=6.0,
    decel_factor=0.5,
    min_speed_factor=2,
):
    """
    محاسبه درصد فرمان در لحظه‌ای با پیشرفت x بر روی مسیر سینوسی پیش تعریف‌شده.

    ورودی‌ها:
      x     : پیشرفت طی شده (متر)
      L_eff : طول کل مسیر برای عبور از مانع (متر)؛ در این مثال دو برابر فاصله تا مانع
      A     : دامنه موج (به عنوان نصف عرض جاده)

    خروجی:
      درصد فرمان در بازه -100 تا +100
    """
    import math

    # محاسبه زاویه حداکثر فرمان (مرجع نرمال‌سازی)
    theta_max = 100

    # محاسبه مشتق تابع مسیر در موقعیت x
    derivative = A * (math.pi / L_eff) * math.cos((math.pi / L_eff) * x)

    # محاسبه زاویه فرمان لازم با استفاده از arctan از مشتق
    theta = math.atan(derivative)
    scale = alpha + (1 - alpha) * (1 + math.tanh(k * (x - L_eff / 2))) / 2

    if v <= v_thresh:
        speed_factor = 1.0
    else:
        # کاهش خطی بر اساس اختلاف سرعت، اما به طور حداقل به min_speed_factor محدود شود.
        speed_factor = max(1.0 - decel_factor * (v - v_thresh), min_speed_factor)

    theta_final = speed_factor * theta

    theta_degree = theta_final * (180 / np.pi)

    # تبدیل زاویه به درصد نسبت به θ_max
    steering_percent = (theta_degree / theta_max) * 100
    steering_percent = max(min(steering_percent, 100), -100)
    reached = abs(x - L_eff) <= tolerance
    at_obstacle = abs(x - (L_eff / 2)) <= 0.001
    return -steering_percent, not reached, not at_obstacle


# Sleep for 3 seconds to make sure that client connected to the simulator
time.sleep(3)

RIGHT_OFFSET = -160
LEFT_OFFSET = 220
D_T = 0.0001

obsterCount = 0

ff_controller = FeedforwardWeightController(
    ramp_up_speed=0.25, ramp_down_speed=0.4, max_weight=1.0, min_weight=0.0
)
try:
    prev_steering = 1
    lane_point = 256
    steering = 0
    speed = 100
    obstacle: bool = False
    y_target = int(405)
    x_total = 0
    L_eff = 0
    prev_speed = 0
    prev_time = 0
    steering_PID = 0
    steering_Sine = 0
    cte_prev = 0


    # مقداردهی اولیه فیلتر کالمن
    # kf = initialize_kalman(D_T, process_noise, measurement_noise)
    
    n_steps = 100  # تعداد گام‌های شبیه‌سازی

    while True:
        # Counting the loops
        counter = counter + 1
        # 1. دریافت داده‌های حسگری:
        # Set the power of the engine the car to 20, Negative number for reverse move, Range [-100,100]
        # Set the Steering of the car -10 degree from center, results the car to steer to the left
        # car.setSteering(0)

        # Set the angle between sensor rays to 45 degrees, Use this only if you want to set it from python client
        # Notice: Once it is set from the client, it cannot be changed using the GUI
        car.setSensorAngle(5)

        # Get the data. Need to call it every time getting image and sensor data
        car.getData()

        # Start getting image and sensor data after 4 loops
        if counter > 4:
            thisTime = time.time()

            # Returns a list with three items which the 1st one is Left sensor data\
            # the 2nd one is the Middle Sensor data, and the 3rd is the Right one.
            sensors = car.getSensors()

            # int (between 0 and 1500cm (if its les than 1500, there is a obstacle ahed))
            image = car.getImage()

            state = 1
            offset = RIGHT_OFFSET
            # Obstacle managmant mode
            
        

            steering, lane_point,cte_prev = process_image_and_compute_steering(
                image, lane_point,cte_prev, y_target, offset, False
            )

            abs_steering = abs(steering)
            if abs_steering > 2:
                car.setSpeed(-100)
            else: car.setSpeed(100)

            #w = ff_controller.update(at_obstacle)
            #print("w = ", w)
            #steering = ((1 - w) * steering_PID) + (w * steering_Sine)
            print("PID steering = ", steering)
            prev_steering = steering

            # if abs(steering - prev_steering) > 20.5:
            #     speed = 50
            #     print("stop")
            car.setSteering(((steering / 90.0) * 100))
            car.setSpeed(speed)
            # Showing the opencv type image

            if cv2.waitKey(10) == ord("q"):
                break

            prev_time = thisTime

            #time.sleep(D_T)


finally:
    car.stop()