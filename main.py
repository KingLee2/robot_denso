#!/usr/bin/env pybricks-micropython
# from tkinter.tix import ButtonBox
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from math import atan, degrees

# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# --- Khởi tạo EV3 và thiết bị ---
ev3 = EV3Brick()

# Hai động cơ bánh xe
left_motor = Motor(Port.A)
right_motor = Motor(Port.B)

clamp_motor = Motor(Port.D)
arm_motor = Motor(Port.C)

# Cảm biến siêu âm 
ultra_f = UltrasonicSensor(Port.S1)
ultra_r = UltrasonicSensor(Port.S2)
# gyro
gyro_sensor = GyroSensor(Port.S4)
#color
color_sensor = ColorSensor(Port.S3)
#driver
robot_driver = DriveBase(left_motor, right_motor, 56, 185)
#var
step = 0
complete = 0
complete_step_0 = 0

count_red = 0
DISTANCE_F = 70 #70
DISTANCE_R = 300
DISTANCE_BALL = 250
###
CLAMP_SPEED = 100
MOVE_ARM_SPEED = 500
REVOLUTION_MOVE_ARM = 2.25

ROTATION_ANGLE_CLAMP = 50
ROTATION_ANGLE_MARGIN = 65

ROTATION_ANGLE_MOVE_ARM = REVOLUTION_MOVE_ARM * 360
# --- PID ---
KP = 2.0
KI = 0.01
KD = 0.8
KP_DIST = 0.8
integral = 0
last_error = 0

# --- Hàm hỗ trợ ---
def init_robot():
    robot_driver.settings(150,300, 60, 120)
    gyro_sensor.reset_angle(0)
    clamp_motor.reset_angle(0)
    arm_motor.reset_angle(0)
    init_pick_ball()

def align_with_wall(move_dist=100):
    """
    Xoay nhẹ qua lại để căn robot song song với tường bên phải.
    Dựa trên biến thiên của cảm biến ultrasonic.
    """
    ev3.screen.clear()
    ev3.screen.print("Aligning...")
    d1 = ultra_r.distance()
    ev3.screen.print(d1)
    robot_driver.reset()
    robot_driver.drive(100, 0)
    while robot_driver.distance() < move_dist:
        wait(10)
    robot_driver.stop()
    wait(100)
    d2 = ultra_r.distance()
    ev3.screen.print(d2)
    delta_d = d2 - d1
    theta = degrees(atan(delta_d / move_dist))  # độ lệch
    ev3.screen.print("Angle:", theta)
    robot_driver.drive(-100, 0)
    while robot_driver.distance() > 0:
        wait(10)
    robot_driver.stop()
    wait(100)
    gyro_sensor.reset_angle(0)
    target = theta  
    if target > 2:
        while gyro_sensor.angle() < target:
            robot_driver.drive(0, 20)
            wait(10)
    elif target < -2:
        while gyro_sensor.angle() > target:
            robot_driver.drive(0, -20)
            wait(10)
    robot_driver.stop()
    ev3.screen.print("Aligned to wall")
    gyro_sensor.reset_angle(0)

def run_forward(speed, k=2):
    # error = gyro_sensor.angle()
    # correction = k * error
    # robot_driver.drive(speed, -correction)
    global integral, last_error
    error = gyro_sensor.angle()
    ev3.screen.print(error)
    integral += error
    derivative = error - last_error
    correction = KP * error + KI * integral + KD * derivative

    # Limit correction
    correction = max(min(correction, 100), -100)

    # left_motor.run(speed - correction)
    # right_motor.run(speed + correction)
    robot_driver.drive(speed, -int(correction))
    last_error = error
    wait(1)

def turn_to_angle(target_angle, max_speed=100):
    """
    Quay robot tới góc target_angle (độ) dựa vào cảm biến gyro.
    Dùng PID nhẹ để đảm bảo không bị overshoot.
    """
    gyro_sensor.reset_angle(0)
    last_error = 0

    while True:
        current_angle = gyro_sensor.angle()
        error = target_angle - current_angle
        derivative = error - last_error
        turn_rate = KP * error + KD * derivative

        # Giới hạn tốc độ quay
        turn_rate = max(min(turn_rate, max_speed), -max_speed)

        robot_driver.drive(0, turn_rate)

        # Dừng khi gần đạt góc mục tiêu
        if abs(error) < 1:
            break

        last_error = error
        wait(10)

    robot_driver.stop()

def rotate_left(angle, omega): #truyen vao tham so am
    gyro_sensor.reset_angle(0)
    while abs(gyro_sensor.angle() - angle) > 2:
    # while gyro_sensor.angle() - angle > -2:
        robot_driver.drive(0,omega)
        ev3.screen.print(gyro_sensor.angle())
        wait(10)
    robot_driver.stop()

def rotate_right(angle, omega): #truyen vao tham so duong
    gyro_sensor.reset_angle(0)
    while abs(gyro_sensor.angle() - angle) > 2:
    # while gyro_sensor.angle() - angle < 2:
        robot_driver.drive(0,omega)
        ev3.screen.print(gyro_sensor.angle())
        wait(10)
    robot_driver.stop()
    
def rotate_circle(angle, R):
    gyro_sensor.reset_angle(0)
    if(angle < 0):
        while abs(gyro_sensor.angle() - angle) > 2:
            left_motor.run(R-90)
            right_motor.run(R+90)
            wait(10)
        robot_driver.stop()
    elif(angle > 0):
        while abs(gyro_sensor.angle() - angle) > 2:
            left_motor.run(R+90)
            right_motor.run(R-90)
            wait(10)
        robot_driver.stop()

def move_forward(vel, dis): #truyen vao tham so duong
    robot_driver.reset()
    while(robot_driver.distance() < dis):
        robot_driver.drive(vel, 0)
        wait(10)
    robot_driver.stop()

def move_back(vel, dis): #truyen vao tham so am
    robot_driver.reset()
    while(robot_driver.distance() > dis):
        robot_driver.drive(vel, 0)
        wait(10)
    robot_driver.stop()

def correct_to_zero(base_omega):
    while abs(gyro_sensor.angle()) > 1:
        error_angle = gyro_sensor.angle()
        # Quyết định hướng quay
        if error_angle > 0:
            robot_driver.drive(0, base_omega)
        else:
            robot_driver.drive(0, -base_omega)
        wait(10)
    robot_driver.stop()

def move_after_wall():
    global distanse_move
    global ultra_r
    robot_driver.stop()
    # robot_driver.straight(50)
    # robot_driver.stop()
    distanse_move = robot_driver.distance()
    ev3.screen.print("dis: ",distanse_move)
    if(distanse_move >1200):
        # robot_driver.straight(DISTANCE_F - 15) #25
        # wait(10)
        # # robot_driver.straight(-(DISTANCE_F - 30 + 450))
        # robot_driver.straight(-(DISTANCE_F - 10)) #40
        # rotate_right(180,45)
        # robot_driver.turn(90)
        # wait(10)
        # robot_driver.turn(90)
        # wait(10)
        robot_driver.straight(-500)
        robot_driver.stop()
        if (ultra_r.distance() < DISTANCE_R):
            # rotate_left(-90,-45)
            robot_driver.turn(-90)
        else:
            # rotate_right(90,45)
            robot_driver.turn(90)
    else:
        # robot_driver.straight(DISTANCE_F - 15) #25
        # wait(10)
        # robot_driver.straight(-(DISTANCE_F - 30)) #40
        if (ultra_r.distance() < DISTANCE_R):
            # rotate_left(-90,-45)
            robot_driver.turn(-90)
        else:
            # rotate_right(90,45)
            robot_driver.turn(90)

def scan_for_ball(angle=15, omega=15, detect_distance=200):
    gyro_sensor.reset_angle(0)
    found = False

    # --- Quay trái ---
    while gyro_sensor.angle() < angle:
        robot_driver.drive(0, -omega)  # quay trái
        # ev3.screen.print("angle: ", gyro_sensor.angle())
        if ultra_f.distance() < detect_distance:
            found = True
            robot_driver.stop()
            ev3.speaker.beep()
            break
        wait(10)
    robot_driver.stop()

    # Nếu chưa thấy → quét sang phải
    if not found:
        while gyro_sensor.angle() > -angle:
            robot_driver.drive(0, omega)  # quay phải
            # ev3.screen.print("angle: ", gyro_sensor.angle())
            if ultra_f.distance() < detect_distance:
                found = True
                robot_driver.stop()
                ev3.speaker.beep()
                break
            wait(10)
        robot_driver.stop()
    if not found:
        # Quay về góc 0
        while abs(gyro_sensor.angle()) > 1:
            error = gyro_sensor.angle()
            robot_driver.drive(0, 2 * error)  # nhẹ nhàng quay về giữa
            # ev3.screen.print("angle: ", gyro_sensor.angle())
            wait(10)
        robot_driver.stop()
    return found

def check_color(sensor, samples=3, delay=20):
    colors = []
    for _ in range(samples):
        c = sensor.color()
        colors.append(c)
        wait(delay)

    # Đếm tần suất xuất hiện của từng màu
    most_color = max(set(colors), key=colors.count)
    count = colors.count(most_color)
    
    # Nếu màu này chiếm đa số (>=2/3) thì coi là hợp lệ
    if count >= (samples * 2 / 3):
        return most_color
    else:
        return None

def clamp(dir, speed, rotate_angle, wait=True):
    sign = 0
    if dir == "out":
        sign = 1
    elif dir == "in":
        sign = -1
    clamp_motor.run_angle(speed, sign * rotate_angle, then=Stop.HOLD, wait=wait)

def move_arm(dir: str, isHalf: bool = False):
    rotation_angle = ROTATION_ANGLE_MOVE_ARM / 2 if isHalf else ROTATION_ANGLE_MOVE_ARM
    sign = 0
    if dir == "up":
        sign = 1
    elif dir == "down":
        sign = -1
    arm_motor.run_angle(MOVE_ARM_SPEED, int(rotation_angle * sign))


def pick_down():
    clamp(
        dir="out",
        speed=CLAMP_SPEED * 0.5,
        rotate_angle=ROTATION_ANGLE_MARGIN,
        wait=False,
    )
    move_arm(dir="down")
    clamp(
        dir="in",
        speed=CLAMP_SPEED,
        rotate_angle=ROTATION_ANGLE_CLAMP + ROTATION_ANGLE_MARGIN,
    )


def pick_up():
    move_arm(dir="up", isHalf=False)
    clamp(dir="out", speed=CLAMP_SPEED, rotate_angle=ROTATION_ANGLE_CLAMP)


def init_pick_ball():
    clamp(dir="out", speed=CLAMP_SPEED, rotate_angle=ROTATION_ANGLE_CLAMP)


def pick_ball():
    pick_down()
    pick_up()
# --- Chương trình chính ---
ev3.speaker.beep()
ev3.screen.clear()
ev3.screen.print("Press ENTER to start")

# Đợi người dùng nhấn nút ENTER
while Button.CENTER not in ev3.buttons.pressed():
    wait(50)

init_robot()
ev3.speaker.beep()
ev3.screen.clear()
ev3.screen.print("robot run")
wait(1000)
###
# while(True):
#     print("Angle: ", gyro_sensor.angle())
#     wait(500)
###### CHECK COLOR SENSOR
# while (True):
#     run_forward(150)
#     detected_color = check_color(color_sensor)
#     ev3.screen.print(detected_color)
#     if(detected_color == Color.YELLOW):
#         robot_driver.stop()
#         robot_driver.straight(60)
#         if (ultra_r.distance() < DISTANCE_R):
#             rotate_left(-90,-45)
#         else:
#             rotate_right(90,45)
#         gyro_sensor.reset_angle(0)
#         robot_driver.reset()
#         break
###### CHECK TO GOAL
# while (True):
#     run_forward(150)
#     detected_color = check_color(color_sensor)
#     ev3.screen.print(detected_color)
#     if(detected_color == Color.RED):
#         ev3.screen.print("RED")
#         robot_driver.stop()
#         robot_driver.straight(-300)
#         robot_driver.stop()
#         rotate_right(180,45)
#         robot_driver.straight(-310)
#         robot_driver.stop()
#         wait(3000)
#         robot_driver.straight(250)
#         if (ultra_r.distance() < DISTANCE_R):
#             rotate_left(-90,-45)
#         else:
#             rotate_right(90,45)
#         complete_step_0 = 1
#         break
###### MAIN
robot_driver.reset()
robot_driver.straight(200)
while (True):
    if(step == 0): #grip ball
        robot_driver.reset()
        gyro_sensor.reset_angle(0)
        while ultra_f.distance() > DISTANCE_F:
            # run_forward(150)
            robot_driver.drive(150,0)
            ev3.screen.print(robot_driver.distance())
            # check ball
            # ev3.screen.print(gyro_sensor.angle())
            detected_color = check_color(color_sensor)
            if(detected_color == Color.BLACK):
                ev3.screen.print("BLACK")
                robot_driver.stop()
                if scan_for_ball():
                    # ev3.screen.print("angle: ", gyro_sensor.angle())
                    robot_driver.straight(100)
                    pick_ball()
                    # complete_step_0 = complete_step_0 + 1
                else:
                    # ev3.screen.print("angle: ", gyro_sensor.angle())
                    robot_driver.straight(100)
                    robot_driver.stop()
            elif(detected_color == Color.YELLOW):
                ev3.screen.print("YELLOW")
                robot_driver.stop()
                robot_driver.straight(60)
                break
                # if (ultra_r.distance() < DISTANCE_R):
                #     rotate_left(-90,-45)
                # else:
                #     rotate_right(90,45)
                # gyro_sensor.reset_angle(0)
                # robot_driver.reset()
            elif(detected_color == Color.RED):
                ev3.screen.print("RED")
                robot_driver.stop()
                # robot_driver.straight(-300)
                # robot_driver.stop()
                # rotate_right(180,45)
                robot_driver.turn(-180)
                while(True):
                    robot_driver.drive(-150,0)
                    detected_color = check_color(color_sensor)
                    if(detected_color == Color.GREEN):
                        break
                # robot_driver.straight(-315)
                robot_driver.stop()
                wait(1000)
                robot_driver.straight(350)
                robot_driver.stop()
                robot_driver.turn(90)
                # rotate_right(90,45)
                # rotate_circle(90, 300)
                # if (ultra_r.distance() < DISTANCE_R):
                #     rotate_left(-90,-45)
                # else:
                #     rotate_right(90,45)
                complete_step_0 = 1
                break

        # correct_to_zero(15)
        if(complete_step_0 == 1):
            # move_after_wall()
            step = 1
        else:
            correct_to_zero(15)
            move_after_wall()
    else:
        robot_driver.reset()
        gyro_sensor.reset_angle(0)
        while ultra_f.distance() > DISTANCE_F:
            # run_forward(150)
            robot_driver.drive(150,0)
            ev3.screen.print(robot_driver.distance())
            detected_color = check_color(color_sensor)        
            if(detected_color == Color.RED):
                robot_driver.stop()
                robot_driver.straight(200)
                complete = 1
                break
            elif(detected_color == Color.GREEN):
                ev3.screen.print("GREEN")
                robot_driver.stop()
                robot_driver.straight(200)
                # rotate_right(90,45)
                # if (ultra_r.distance() < DISTANCE_R):
                #     rotate_left(-90,-45)
                # else:
                #     rotate_right(90,45)
                # gyro_sensor.reset_angle(0)
                # robot_driver.reset()
                break

        # correct_to_zero(15)
        if(complete == 1):
            break
        else:
            correct_to_zero(15)
            move_after_wall()
####
ev3.screen.print("COMPLETE TASK")
ev3.speaker.say("Done")
####################################