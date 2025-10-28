#!/usr/bin/env pybricks-micropython
# from tkinter.tix import ButtonBox
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# --- Khởi tạo EV3 và thiết bị ---
ev3 = EV3Brick()

# Hai động cơ bánh xe
left_motor = Motor(Port.C)
right_motor = Motor(Port.B)

clamp_motor = Motor(Port.A)
flip_motor = Motor(Port.D)

# Cảm biến siêu âm 
ultra_f = UltrasonicSensor(Port.S1)
ultra_r = UltrasonicSensor(Port.S3)
# gyro
gyro_sensor = GyroSensor(Port.S4)
#color
color_sensor = ColorSensor(Port.S2)
#driver
robot_driver = DriveBase(left_motor, right_motor, 56, 178)
#var
step = 0
ball_count = 0
complete = 0
complete_step_0 = 0
complete_step_1 = 0
complete_step_2 = 0
DISTANCE_F = 180
DISTANCE_R = 300
DISTANCE_BALL = 250
###
CLAMP_SPEED = 200
FLIP_SPEED = 500
ROTATION_ANGLE_CLAMP = 80
REVOLUTION_FLIP = 2.2
ROTATION_ANGLE_FLIP = REVOLUTION_FLIP * 360
# --- PID ---
KP = 2.0
KI = 0.01
KD = 0.8

integral = 0
last_error = 0

# --- Hàm hỗ trợ ---
def init_robot():
    robot_driver.settings(200,50, 30, 30)
    gyro_sensor.reset_angle(0)

def run_forward(speed, k=2):
    # error = gyro_sensor.angle()
    # correction = k * error
    # robot_driver.drive(speed, -correction)
    global integral, last_error
    error = gyro_sensor.angle() 
    integral += error
    derivative = error - last_error
    correction = KP * error + KI * integral + KD * derivative

    # Limit correction
    correction = max(min(correction, 100), -100)

    # left_motor.run(speed - correction)
    # right_motor.run(speed + correction)
    robot_driver.drive(speed, -int(correction))
    last_error = error
    wait(10)

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
            robot_driver.drive(0, -base_omega)
        else:
            robot_driver.drive(0, base_omega)
        wait(10)
    robot_driver.stop()

def move_after_wall():
    global distanse_move
    global ultra_r
    robot_driver.stop()
    distanse_move = robot_driver.distance()
    if(distanse_move >1200):
        move_back(-50, -50)
        # rotate_right(180,30)
        turn_to_angle(180)
        move_forward(200,400)
        if (ultra_r.distance() < DISTANCE_R):
            # rotate_left(-90,-30)
            turn_to_angle(-90)
        else:
            # rotate_right(90,30)
            turn_to_angle(90) 
    else:
        if (ultra_r.distance() < DISTANCE_R):
            # rotate_left(-90,-30)
            turn_to_angle(-90)
        else:
            # rotate_right(90,30)
            turn_to_angle(90)

def scan_for_ball(angle=15, omega=15, detect_distance=250):
    gyro_sensor.reset_angle(0)
    found = False

    # --- Quay trái ---
    while gyro_sensor.angle() > -angle:
        robot_driver.drive(0, -omega)  # quay trái
        if ultra_f.distance() < detect_distance:
            found = True
            robot_driver.stop()
            ev3.speaker.beep()
            break
        wait(10)
    robot_driver.stop()

    # Nếu chưa thấy → quét sang phải
    if not found:
        while gyro_sensor.angle() < angle:
            robot_driver.drive(0, omega)  # quay phải
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
            robot_driver.drive(0, -2 * error)  # nhẹ nhàng quay về giữa
            wait(10)
        robot_driver.stop()
    return found

def clamp(state):
    sign_clamp = 1 if state == "out" else -1 if state == "in" else 0
    clamp_motor.run_angle(CLAMP_SPEED, sign_clamp * ROTATION_ANGLE_CLAMP)

def flip(state, isHalf: bool = False):
    revolution = ROTATION_ANGLE_FLIP / 2 if isHalf else ROTATION_ANGLE_FLIP
    sign_flip = 1 if state == "up" else -1 if state == "down" else 0
    flip_motor.run_angle(FLIP_SPEED, int(revolution * sign_flip))

def picking_down():
    clamp("in")
    flip("down", True)
    clamp("out")
    flip("down", True)
    clamp("in")

def picking_up():
    flip("up", False)
    clamp("out")

def picking():
    picking_down()
    picking_up()

# --- Chương trình chính ---
ev3.speaker.beep()
ev3.screen.clear()
init_robot()
ev3.screen.print("Press ENTER to start")

# Đợi người dùng nhấn nút ENTER
while Button.CENTER not in ev3.buttons.pressed():
    wait(50)

ev3.speaker.beep()
ev3.screen.clear()
ev3.screen.print("robot run")
wait(1000)
# turn_to_angle(-90)
# wait(1000)
# turn_to_angle(90)
######
while (True):
    robot_driver.reset()
    gyro_sensor.reset_angle(0)
    while ultra_f.distance() > DISTANCE_F:
        run_forward(150)
        ev3.screen.print(gyro_sensor.angle())
        # robot_driver.drive(100, 0)
        # check ball
        ev3.screen.print(gyro_sensor.angle())
        if(color_sensor.color() == Color.BLACK):
            robot_driver.stop()
            if scan_for_ball():
                move_forward(100,170)
                picking()
                complete_step_0 = complete_step_0 + 1
        if(color_sensor.color() == Color.RED):
            robot_driver.stop()
            move_back(-100,-300)
            turn_to_angle(-90)
            # rotate_left(-90,-30)

    correct_to_zero(15)
    if(complete_step_0 == 4):
        break
    move_after_wall()
####
# robot_driver.reset()
# gyro_sensor.reset_angle(0)
# while ultra_f.distance() > DISTANCE_F:
#     run_forward(150)
#     ev3.screen.print(gyro_sensor.angle())
#     if(color_sensor.color() == Color.BLACK):
#         robot_driver.stop()
#         while(True):
#             ev3.screen.print(ultra_f.distance())
            

ev3.screen.print("COMPLETE TASK")
ev3.speaker.say("Done")
#####################################

