from math import isclose, pow

from gpiozero import Button, LED, PWMLED

from Managers import Timer
from line_cam import camera_y
from mp_manager import *

print_obstacle = False
speed_zone = False

# program variables
wait_time_red = 9

max_turn_angle = 110

last_turn_dir = "l"

obstacle_dir = ["l", "r"]
obstacle_count = 0

obstacle_on_ramp = True
turn_around_ramp_side = True

turn_around_45 = False

# average time variables
time_last_angles = empty_time_arr()

time_last_gyro_y = empty_time_arr()
time_last_gyro_x = empty_time_arr()
time_last_gyro_z = empty_time_arr()

time_sensor_one = empty_time_arr()
time_sensor_two = empty_time_arr()
time_sensor_three = empty_time_arr()
time_sensor_four = empty_time_arr()
time_sensor_five = empty_time_arr()
time_sensor_six = empty_time_arr()
time_sensor_seven = empty_time_arr()

time_silver_detected = empty_time_arr()
time_silver_angle = empty_time_arr()
time_line_similarity = fill_array(0, 1200)

timer = Timer()

run = True

# gpio pins
in_1 = 0  # forwards right
in_2 = 1  # backwards right
in_3 = 5  # forwards left
in_4 = 6  # backwards right
en_a = 12  # speed right
en_b = 13  # speed left

led = 20  # lighting pin
button_pin = 21  # button pin

# corrections due to different motor speeds
left_correction = 1
right_correction = 1

# update limit for sensor values
last_update_time = time.perf_counter()


def switch_lights(light_on):
    if light_on:
        light.off()
    else:
        light.on()


def steer(angle=190., speed=0.8):
    speed_left.value = 0
    speed_right.value = 0

    # stop
    if angle == 190:
        forward_right.off()
        backward_right.off()
        forward_left.off()
        backward_left.off()
        speed_left.value = 0
        speed_right.value = 0

    # backward
    elif angle == 200:
        forward_right.on()
        backward_right.off()
        forward_left.on()
        backward_left.off()
        speed_left.value = max(speed * left_correction, 0)
        speed_right.value = max(speed * right_correction, 0)

    # forward
    elif angle in range(-180, 181):
        forward_right.off()
        backward_right.on()
        forward_left.off()
        backward_left.on()

        # right
        if angle >= 0:
            if angle > max_turn_angle:
                forward_right.on()
                backward_right.off()
                forward_left.off()
                backward_left.on()
                speed_left.value = min(speed * left_correction * 1.2, 1)
                speed_right.value = min(speed * right_correction * 1.2, 1)
            else:
                speed_left.value = min(speed * left_correction, 1)
                speed_right.value = min(speed * right_correction * ((max_turn_angle - angle) / (max_turn_angle - 1)), 1)

        # left
        else:
            if angle < -max_turn_angle:
                forward_right.off()
                backward_right.on()
                forward_left.on()
                backward_left.off()
                speed_left.value = min(speed * left_correction * 1.2, 1)
                speed_right.value = min(speed * right_correction * 1.2, 1)
            else:
                speed_left.value = min(speed * left_correction * ((max_turn_angle + angle) / (max_turn_angle - 1)), 1)
                speed_right.value = min(speed * right_correction, 1)


def program_continue():
    switch.value = True if button.value == 1 else False
    return switch.value and not terminate.value


def update_sensor_average():
    global time_sensor_one, time_sensor_two, time_sensor_three, time_sensor_four, time_sensor_five, time_sensor_six, time_sensor_seven, time_last_gyro_y, time_last_gyro_x, time_last_gyro_z, time_silver_detected, time_silver_angle, time_line_similarity, last_update_time

    if time.perf_counter() - last_update_time > 1 / 90:
        if sensor_one.value > 25:
            time_sensor_one = add_time_value(time_sensor_one, sensor_one.value)
        if sensor_two.value > 25:
            time_sensor_two = add_time_value(time_sensor_two, sensor_two.value)
        time_sensor_three = add_time_value(time_sensor_three, sensor_three.value)
        time_sensor_four = add_time_value(time_sensor_four, sensor_four.value)
        if sensor_five.value > 25:
            time_sensor_five = add_time_value(time_sensor_five, sensor_five.value)
        time_sensor_six = add_time_value(time_sensor_six, sensor_six.value)
        time_sensor_seven = add_time_value(time_sensor_seven, sensor_seven.value)

        time_last_gyro_y = add_time_value(time_last_gyro_y, sensor_y.value)
        time_last_gyro_x = add_time_value(time_last_gyro_x, sensor_x.value)
        time_last_gyro_z = add_time_value(time_last_gyro_z, sensor_z.value)

        time_silver_detected = add_time_value(time_silver_detected, silver_value.value)
        time_silver_angle = add_time_value(time_silver_angle, silver_angle.value)

        time_line_similarity = add_time_value(time_line_similarity, line_similarity.value)

        last_update_time = time.perf_counter()


def round_angle(angle, direction=0, rounding_value=90, final_addition=0, round_45_only=False):
    angle = (angle + direction) % 360
    if round_45_only:
        possible_angles = [45, 135, 225, 315]
        angle = min(possible_angles, key=lambda x: abs(x - angle))
    else:
        angle = round(angle / rounding_value) * rounding_value % 360
    return (angle + final_addition) % 360


def get_rotation():
    if rotation_y.value == "ramp_up":
        if get_time_average(time_last_gyro_y, .7) > 10:
            timer.set_timer("was_ramp_up", .7)
            return "ramp_up"
        elif get_time_average(time_last_gyro_y, .5) < -5.5:
            return "ramp_down"
        else:
            return "none"
    else:
        if get_time_average(time_last_gyro_y, .5) > 15:
            timer.set_timer("was_ramp_up", .7)
            return "ramp_up"
        elif get_time_average(time_last_gyro_y, .5) < -11:
            return "ramp_down"
        else:
            return "none"


def get_speed(angle):
    if rotation_y.value == "ramp_up":
        if abs(angle) > max_turn_angle:
            if not timer.get_timer("stuck_detected"):
                return .75
            else:
                return .6
        elif abs(angle) > max_turn_angle / 2:
            if not timer.get_timer("stuck_detected"):
                return .9
            else:
                return .75
        else:
            if not timer.get_timer("stuck_detected"):
                return .9
            else:
                return .65

    elif rotation_y.value == "ramp_down":
        if abs(angle) > max_turn_angle:
            return .65 if timer.get_timer("stuck_detected") else .85
        elif abs(angle) > max_turn_angle / 2:
            return .28 if timer.get_timer("stuck_detected") else .65
        else:
            return .15 if timer.get_timer("stuck_detected") else .3

    elif ramp_ahead.value or not timer.get_timer("ramp_ahead"):
        if ramp_ahead.value:
            timer.set_timer("ramp_ahead", 2)

        if abs(angle) > max_turn_angle:
            return .65
        elif abs(angle) > max_turn_angle / 2:
            return .4
        else:
            return .3

    return 1


def add_angle(angle, addition):
    return (angle + addition) % 360


def sub_angle(angle, subtraction):
    return (angle - subtraction) % 360


def turn_to_angle(angle, tolerance=1.5, stop_on_black=False, direction="n", speed=0, correct_overturn=True):
    global time_last_gyro_x

    start_angle = sensor_x.value
    last_angle = 400
    timer.set_timer("detect_stuck", 1.5)
    while abs((angle - sensor_x.value + 540) % 360 - 180) > tolerance:
        update_sensor_average()

        if direction == "n":
            angle_to_turn = (angle - sensor_x.value + 540) % 360 - 180
        elif direction == "r":
            angle_to_turn = (angle - sensor_x.value) % 360
            if last_angle < (angle_to_turn - 10):
                direction = "n"
        elif direction == "l":
            angle_to_turn = (angle - sensor_x.value) % -360
            if -last_angle > (angle_to_turn + 10):
                direction = "n"

        turn_direction = 180 if angle_to_turn > 0 else -180

        if angle_to_turn <= 0 and turn_direction == 180:
            time.sleep(.2)
            turn_direction = -180
        elif angle_to_turn > 0 and turn_direction == -180:
            time.sleep(.2)
            turn_direction = 180

        if speed == 0:
            steer(turn_direction, max(1 - pow(abs(abs(angle_to_turn) / 360 - 1), 7), 0.4))
        else:
            steer(turn_direction, speed)

        last_angle = abs(angle_to_turn)

        # if stuck
        if isclose(get_time_average(time_last_gyro_x, .5), sensor_x.value, abs_tol=.5) and not get_time_average(time_last_gyro_x, 1) == -1 and abs((angle - sensor_x.value + 540) % 360 - 180) > 20 and timer.get_timer("detect_stuck"):
            steer(-turn_direction, .9)
            time.sleep(.5)
            steer(turn_direction, .9)
            time.sleep(.8)
            steer()

            timer.set_timer("detect_stuck", 2)

        if stop_on_black and line_detected.value:
            steer()
            return "black"

        if not correct_overturn and abs((angle - sensor_x.value + 540) % 360 - 180) < (2 * tolerance):
            steer()
            return "none"

        if not program_continue():
            return

    steer()
    return "none"


def drive_back_until_line(max_time, speed=.7):

    timer.set_timer("find_line_again", max_time)
    while not line_detected.value and not timer.get_timer("find_line_again"):
        steer(200, speed)
        time.sleep(.001)
    min_line_size.value = 3000

    steer(0, .7)
    time.sleep(.2)
    steer()

    return line_detected.value


def ensure_line_detected():
    time.sleep(.25)
    if not line_detected.value:
        steer(200, .7)
        time.sleep(.15)
        steer()
        time.sleep(.1)

        if not line_detected.value:
            return False

    return True


def orientate_gap():
    update_sensor_average()
    if (not line_detected.value or line_detected.value and line_size.value < 17000) and not silver_detected():
        status.value = f'Validating gap'

        steer(200, .7)
        time.sleep(.15)
        steer()
        time.sleep(.2)

        update_sensor_average()
        if silver_detected():
            steer(0, .7)
            time.sleep(.15)
            steer()
            time.sleep(.2)
            return False

        if obstacle_detected():
            return False

        steer(200, .7)
        time.sleep(.3)
        if not line_detected.value:
            time.sleep(.2)

        steer(0, .7)
        time.sleep(.25)
        steer()

    update_sensor_average()
    if line_detected.value and black_average.value < 40 and not silver_detected():
        status.value = f'Orientating at gap'

        angle = gap_angle.value
        x_gap = gap_center_x.value
        y_gap = gap_center_y.value

        correction_counter = 0
        while correction_counter < 7:

            if y_gap < 10:
                return False

            time_foreward = .25

            if (0 < angle < 173 and x_gap < 0) or (angle < -7 and x_gap > 0) or (0 < angle < 155) or (angle < -25):
                min_time = .35

                if (0 < angle < 173 and x_gap < 0) or (angle < -7 and x_gap > 0):
                    x_gap_perc = pow(abs(x_gap) / (180 + 40), .7)
                else:
                    x_gap_perc = 0

                y_gap_perc = y_gap / camera_y

                if angle > 0:
                    angle_perc = (angle - 90) / 90
                else:
                    angle_perc = abs(-angle / 90)

                time_foreward = min_time + .3 * x_gap_perc + .1 * y_gap_perc + .3 * angle_perc

            if not (0 >= angle > -1 or angle > 179):
                steer(0, .7)
                time.sleep(time_foreward)

                if not program_continue():
                    return False

                if angle > 0:
                    steer(180, .65)
                    time.sleep(abs(.9 - .85 * ((angle - 90) / 90)))
                else:
                    steer(-180, .65)
                    time.sleep(abs(.05 + .85 * abs(-angle / 90)))

                steer()

                if not program_continue():
                    return False

                min_line_size.value = 9000
                steer(200, .7)
                time.sleep(time_foreward + np.clip(((time_foreward - .25) / .4) * .15, 0, .15))

                if not drive_back_until_line(.6, .7):
                    return False

                if line_size.value > 17000:
                    steer(200, .7)
                    time.sleep(.2)
                    steer()
                    return False

            if not ensure_line_detected() or not program_continue():
                return False

            angle = gap_angle.value
            x_gap = gap_center_x.value

            if y_gap < 10:
                return False

            if abs(x_gap) > 55:
                steer(180 if x_gap > 0 else -180, .6)
                time.sleep(.4)
                steer()
                time.sleep(.2)

                time_foreward = .35 + .35 * ((abs(x_gap) - 55) / 100)

                steer(0, .7)
                time.sleep(time_foreward)
                steer()

                if not program_continue():
                    return False

                steer(-180 if x_gap > 0 else 180, .6)
                time.sleep(.3)
                steer()
                time.sleep(.2)

                if not program_continue():
                    return False

                min_line_size.value = 9000
                steer(200, .7)
                time.sleep(time_foreward + np.clip(((time_foreward - .35) / .35) * .2, 0, .2))

                if not drive_back_until_line(.5, .7):
                    return False

                if line_size.value > 17000:
                    steer(200, .7)
                    time.sleep(.2)
                    steer()
                    return False

                if not ensure_line_detected() or not program_continue():
                    return False

                angle = gap_angle.value
                x_gap = gap_center_x.value

                if y_gap < 10:
                    return False

            if (0 >= angle > -1 or angle > 179) and abs(x_gap) < 140:
                break

            if not program_continue():
                return False

            correction_counter += 1

        status.value = f'Gap orientated'
        line_status.value = "gap_avoid"
        min_line_size.value = 4000
        steer(0, .7)
        time.sleep(.8)
        return True

    elif (line_detected.value and black_average.value > 40) or silver_detected():
        status.value = f'Validation failed'
        if line_detected.value and black_average.value > 40:
            steer(200, .7)
            time.sleep(.2)
            steer()
        return False

    else:
        status.value = f'Searching for the line'

        start_angle = sensor_x.value

        reason = turn_to_angle(add_angle(sensor_x.value, 45), 1.5, True, False, "r", 0.6, False)

        if reason == "black" or not program_continue():
            return False

        reason = turn_to_angle(sub_angle(sensor_x.value, 90), 1.5, True, False, "l", 0.6, False)

        if reason == "black" or not program_continue():
            return False

        turn_to_angle(start_angle, 1.5)

        timer.set_timer("line_search", 1.2)
        while not line_detected.value and not timer.get_timer("line_search") and program_continue():
            steer(0, .7)
            time.sleep(.001)

        steer()
        return False


def turn_around():
    average_sensor_z = get_time_average(time_last_gyro_z, 1)
    if (-135 > average_sensor_z > -165 or 130 < average_sensor_z < 160) and turn_around_ramp_side and rotation_y.value == "none":
        steer(0, .7)
        time.sleep(.15)
        steer(-180, .6)
        time.sleep(.15)
        steer(0, .8)
        time.sleep(.15)
        steer(-180, .6)
        time.sleep(.2)
        steer(0, .8)
        time.sleep(.2)
        steer(-180, .6)
        time.sleep(.4)
        steer(0, .8)
        time.sleep(.3)
        steer(-180, .6)
        time.sleep(.4)
        steer(0, .8)
        time.sleep(.6)
        steer(-180, .6)
        time.sleep(.4)
        steer(0, .8)
        time.sleep(.4)
        steer(-180, .6)
        time.sleep(.4)
        steer(0, .8)
        time.sleep(.2)
        steer(-180, .6)
        time.sleep(.4)

    else:
        was_ramp_up = rotation_y.value == "ramp_up" or not timer.get_timer("was_ramp_up")
        steer(0, .7)
        time.sleep(.85 if was_ramp_up else .55)

        turn_to_angle(round_angle(sensor_x.value, 180, 90 if not turn_around_45 else 45), direction=last_turn_dir)

        steer(200, .7)
        time.sleep(.2 if was_ramp_up else .3)
        steer()

        if line_size.value < 5500:
            steer(200, .7)
            time.sleep(.4)
            steer()

    timer.set_timer("stuck_cooldown", 5)

    return "r" if last_turn_dir == "l" else "l"


def obstacle_detected():
    if ((20 < get_time_average(time_sensor_one, 0.25) < 75 or 20 < get_time_average(time_sensor_two, 0.25) < 55 or 20 < get_time_average(time_sensor_five, 0.25) < 55) and (rotation_y.value == "none" or obstacle_on_ramp)) and timer.get_timer("obstacle_detect_cooldown") and print_obstacle:
        print("Obstacle: ", get_time_average(time_sensor_one, 0.25), get_time_average(time_sensor_two, 0.25), get_time_average(time_sensor_five, 0.15))
    return ((20 < get_time_average(time_sensor_one, 0.25) < 75 or 20 < get_time_average(time_sensor_two, 0.25) < 55 or 20 < get_time_average(time_sensor_five, 0.25) < 55) and (rotation_y.value == "none" or obstacle_on_ramp)) and timer.get_timer("obstacle_detect_cooldown")


def obstacle_detected_again():
    return ((20 < get_time_average(time_sensor_one, 0.25) < 75 or 20 < get_time_average(time_sensor_two, 0.25) < 55 or 20 < get_time_average(time_sensor_five, 0.25) < 95) and (rotation_y.value == "none" or obstacle_on_ramp)) and timer.get_timer("obstacle_detect_cooldown")


def turn_for_obstacle():
    global time_sensor_one, time_sensor_two, time_sensor_five

    if rotation_y.value == "none":
        sensor_one_avg = get_time_average(time_sensor_one, 0.15)
        sensor_two_avg = get_time_average(time_sensor_two, 0.15)

        steer(200, .7)
        time.sleep(.15)
        steer()

        # centering in front of obstacle
        update_sensor_average()
        if get_time_average(time_sensor_five, 0.15) > 130:
            status.value = f'Centering in front of obstacle'

            turn_direction = 180 if sensor_one_avg < sensor_two_avg else -180

            timer.set_timer("obstacle", 5)
            while get_time_average(time_sensor_five, 0.15) > 130:
                update_sensor_average()

                steer(turn_direction, .55)

                if timer.get_timer("obstacle"):
                    return False

                if not program_continue():
                    return False

            steer(-turn_direction, .55)
            time.sleep(.15)

        steer()
        time.sleep(.5)

        # correcting distance to obstacle
        update_sensor_average()
        if 0 < get_time_average(time_sensor_five, 0.15) < 180:
            status.value = f'Correcting distance to obstacle'

            timer.set_timer("obstacle", 3)
            while not 75 < get_time_average(time_sensor_five, 0.15) < 90:
                update_sensor_average()

                if get_time_average(time_sensor_five, 0.15) > 90:
                    steer(0, .55)
                elif get_time_average(time_sensor_five, 0.15) < 75:
                    steer(200, .55)

                if timer.get_timer("obstacle"):
                    return False

                if not program_continue():
                    return False

            steer()

        turn_direction = -180 if obstacle_dir[obstacle_count % len(obstacle_dir)] == "l" else 180

        # turning to avoid obstacle
        timer.set_timer("obstacle", 5)
        if 70 < get_time_average(time_sensor_five, 0.15) < 95:
            while 0 < get_time_average(time_sensor_five, 0.15) < 280:
                status.value = f'Turning to avoid obstacle'

                update_sensor_average()

                steer(turn_direction, .65)

                if timer.get_timer("obstacle"):
                    return False

                if not program_continue():
                    return False

            steer(turn_direction, .65)
            time.sleep(.3)

            steer(0, .55)
            time.sleep(.45)

            steer()
            return True

        else:
            return False

    else:

        if rotation_y.value == "ramp_down":
            steer(200, .5)
            time.sleep(.3)
            steer(200, .2)
        elif rotation_y.value == "ramp_up":
            steer()
            time.sleep(.5)
            steer(200, .25)
            time.sleep(.3)
            steer(0, .15)

        time.sleep(.5)

        update_sensor_average()
        if obstacle_detected_again():
            turn_direction = -180 if obstacle_dir[obstacle_count % len(obstacle_dir)] == "l" else 180

            if rotation_y.value == "ramp_down":
                steer(200, .7)
                time.sleep(.7)

            steer(turn_direction, .75)

            if rotation_y.value == "ramp_up":
                time.sleep(.5)
            else:
                time.sleep(.65)

            if rotation_y.value == "ramp_up":
                steer(0, .8)
                time.sleep(.3)

            steer()
            return True

        else:
            return False


def return_after_failed_obstacle(start_angle):
    turn_to_angle(start_angle, 1.5)
    steer(200, .7)
    time.sleep(.25)
    steer()


def orientate_after_obstacle(direction):
    steer(0, .7)
    time.sleep(.1)
    steer()

    start_angle = sensor_x.value

    steer(180 if direction == "l" else -180, .7)
    time.sleep(.4)
    steer()
    time.sleep(.2)

    steer(0, .7)
    time.sleep(.75)
    steer()
    time.sleep(.2)

    steer(-180 if direction == "l" else 180, .7)
    time.sleep(.4)
    timer.set_timer("obstacle_turn", 3)
    while not line_detected.value and not timer.get_timer("obstacle_turn"):
        time.sleep(.01)

        if not program_continue():
            return

    if not timer.get_timer("obstacle_turn"):
        time.sleep(.35)
        steer()
        time.sleep(.2)

        if line_detected.value:
            return

        steer(180 if direction == "l" else -180, .7)
        time.sleep(.2)
        steer()
        time.sleep(.2)

        if line_detected.value:
            return

    turn_to_angle(start_angle, 1.5)

    if line_detected.value:
        return

    steer(200, .7)
    time.sleep(.85)
    steer()
    time.sleep(.2)
    return


def seesaw_detected():
    return get_time_average(time_last_gyro_y, .6) > 6.5 and sensor_y.value < -10


def avoid_seesaw():
    steer(200, .7)
    time.sleep(.15)
    steer(200, .1)
    time.sleep(2)

    steer(200, .7)
    time.sleep(.2)
    if not line_detected.value:
        time.sleep(.3)


def avoid_stuck():
    status.value = f'Line similarity too high, stuck detected'

    angle = line_angle.value

    if rotation_y.value == "none" and line_status.value == "line_detected" and abs(angle) > 120:
        steer()
        time.sleep(1)
        steer(180 if angle < 0 else -180, .7)
        time.sleep(.35)
        steer(0, .7)
        time.sleep(.45)
        steer(-180 if angle < 0 else 180, .7)
        time.sleep(.45)
        steer(200, .7)
        time.sleep(.5)

    elif rotation_y.value == "ramp_down":
        steer()
        time.sleep(1)
        steer(200, .7)
        time.sleep(.5)
        steer()

    else:
        steer()
        time.sleep(.5)

    timer.set_timer("stuck_detected", 1.2 if rotation_y.value == "ramp_up" else .85)


def stop_for_red():
    steer()
    for i in range(wait_time_red):
        if not program_continue():
            break

        if i == 5:
            run_start_time.value = -1

        status.value = f'Waiting for red: {wait_time_red - i} seconds left'
        time.sleep(1)

        if i == wait_time_red - 1:
            steer(0, 55)
            time.sleep(.5)
            steer()


def silver_detected():
    return get_time_average(time_silver_detected, .15 if rotation_y.value == "ramp_down" else .25) > .7 and timer.get_timer("silver_cooldown")


def validate_silver():
    status.value = f'Validating silver line'

    if rotation_y.value == "ramp_down":
        steer(200, .5)
        time.sleep(.3)
        steer(200, .2)
        time.sleep(.7)
        steer(200, .6)
        time.sleep(.45)
        steer()
    elif rotation_y.value == "ramp_up":
        pass
    else:
        steer(200, .7)
        time.sleep(.15)
        steer()

    if rotation_y.value == "ramp_down":
        steer(200, .2)
    elif rotation_y.value == "ramp_up":
        steer(0, .2)
    else:
        steer()

    time.sleep(.25 if speed_zone else .5)

    prev_line_size = black_average.value

    line_status.value = "check_silver"

    switch_lights(False)

    time.sleep(1.5 if speed_zone else 1.7)

    # print(f"Prev: {prev_line_size}, Current: {black_average.value}")
    if black_average.value > max(prev_line_size - 23, 0):
        status.value = f'Validating silver line failed'
        line_status.value = "line_detected"
        switch_lights(True)

        time.sleep(1)

        return False
    else:
        status.value = f'Validating silver line successful'
        if not speed_zone:
            time.sleep(.4)
        return True


def calculate_distance_nearest_90(current_angle):
    nearest_90 = round(current_angle / 90) * 90
    return nearest_90 - current_angle


def position_for_entry():
    tolerance = 10
    status.value = f'Centering silver line'

    if not line_detected.value or line_angle_y.value < camera_y * .1:
        timer.set_timer("entry", .75)
        while not line_detected.value and line_angle_y.value < camera_y * .1 and not timer.get_timer("entry") and program_continue():
            steer_direction = 200 if not line_detected.value else 0
            steer(steer_direction, .4)

        steer(0, .1)
        time.sleep(.3)
        steer()

    direction = 0
    if abs(calculate_distance_nearest_90(sensor_x.value)) > 15:
        status.value = f'Rotation not straight, determining silver angle'

        line_status.value = "position_entry_1"
        time.sleep(1.5)
        line_status.value = "position_entry"
        switch_lights(True)
        time.sleep(1)
        line_status.value = "position_entry_2"
        time.sleep(1.3)

        start_time = time.perf_counter()
        while not time.perf_counter() - start_time > .7:
            update_sensor_average()

        angle_silver = get_time_average(time_silver_angle, .25)
        line_status.value = "position_entry"

        if angle_silver > 20:
            direction = -35
        elif 0 > angle_silver > -160:
            direction = 35

        status.value = f'Got silver angle: {round(angle_silver, 2)}°'
        switch_lights(False)
        time.sleep(1)

    status.value = f'Centering black line'

    last_line_angle_y = line_angle_y.value
    timer.set_timer("center_black", 2)
    while not -tolerance < line_angle.value < tolerance and not timer.get_timer("center_black"):
        if line_detected.value:
            last_line_angle_y = line_angle_y.value

        if line_angle_y.value < camera_y * .1 or (not line_detected.value and last_line_angle_y < camera_y / 2):
            steer(0, .30)
        elif line_angle_y.value > camera_y * .9 or (not line_detected.value and last_line_angle_y > camera_y / 2):
            steer(200, .30)
        else:
            turn_direction = 180 if line_angle.value > 0 else -180

            steer(turn_direction, max(1 - pow(abs(abs(abs(line_angle.value)) / 180 - 1), 1.7), 0.4))

        if not program_continue():
            return False

    steer(0, .65)
    time.sleep(.35)

    status.value = f'Turning into entry'
    turn_to_angle(round_angle(sensor_x.value, direction=direction))

    return True


# debug methods
def calibrate_turn_time():
    gyro_x_offset(0)

    last_gyro = 0
    for i in range(1, 300, 1):
        direction = 180 if i % 2 == 0 else -180
        steer(direction, .8)
        time.sleep(i / 100)
        steer()
        time.sleep(.5)

        print(f"{i / 100}: {round(((sensor_x.value - last_gyro + 540) % 360 - 180), 2)}")

        if not program_continue():
            break

        last_gyro = sensor_x.value


########################################################################################################################
# Main Control Loop
########################################################################################################################


def control_loop():
    global forward_right, backward_right, forward_left, backward_left, speed_right, speed_left, light, button
    global run, last_turn_dir, obstacle_count, time_last_gyro_y, time_last_gyro_x, time_last_gyro_z, time_last_angles, time_sensor_one, time_sensor_two, time_sensor_three, time_sensor_four, time_sensor_five, time_sensor_six, time_sensor_seven, time_silver_detected, time_line_similarity

    # gpio setup
    forward_right = LED(in_1)
    backward_right = LED(in_2)
    forward_left = LED(in_3)
    backward_left = LED(in_4)

    speed_right = PWMLED(en_a, frequency=1000)
    speed_left = PWMLED(en_b, frequency=1000)

    light = LED(led)
    button = Button(button_pin)

    time.sleep(.5)

    switch_lights(True)

    calibration_switched_light = False
    obstacle_is_ramp = "none"
    iteration_limit_time = time.perf_counter()
    max_iterations = 60
    iteration_time = time.perf_counter()
    counter = 0

    timer.set_timer("silver_cooldown", .01)
    timer.set_timer("ramp_ahead", .01)
    timer.set_timer("stuck_detected", .01)
    timer.set_timer("obstacle_detect_cooldown", .01)
    timer.set_timer("stuck_cooldown", 5)
    timer.set_timer("was_ramp_up", .01)

    while not terminate.value:
        if calibrate_color_status.value == "none":
            if calibration_switched_light:
                switch_lights(True)
                calibration_switched_light = False

            # update average time values
            update_sensor_average()

            # update runtime variables
            switch.value = True if button.value == 1 else False
            rotation_y.value = get_rotation()

            if not switch.value and run and objective.value == "debug":
                status.value = f'Stopped (debug)'
                steer()
                switch_lights(True)

                run = False

            if not switch.value and run and not objective.value == "debug":
                status.value = f'Stopped'
                steer()

                switch_lights(True)

                objective.value = "follow_line"  # follow_line
                line_status.value = "line_detected"  # line_detected

                run = False

            # debug switch reset
            if switch.value and not run and objective.value == "debug":
                gyro_x_offset(0)
                gyro_y_offset(0)
                gyro_z_offset(0)
                run = True

            # lop reset
            if switch.value and not run and not objective.value == "debug":

                # start timer
                if run_start_time.value == -1:
                    run_start_time.value = time.perf_counter()

                gyro_x_offset(0)
                gyro_y_offset(0)
                gyro_z_offset(0)

                # reset all average time arrays
                time_last_angles = empty_time_arr()

                time_last_gyro_y = empty_time_arr()
                time_last_gyro_x = empty_time_arr()
                time_last_gyro_z = empty_time_arr()

                time_sensor_one = empty_time_arr()
                time_sensor_two = empty_time_arr()
                time_sensor_three = empty_time_arr()
                time_sensor_four = empty_time_arr()
                time_sensor_five = empty_time_arr()
                time_sensor_six = empty_time_arr()
                time_sensor_seven = empty_time_arr()

                time_line_similarity = fill_array(0, 1200)
                timer.set_timer("stuck_cooldown", 5)

                time.sleep(.25)

                run = True

            if run:
                if objective.value == "follow_line":

                    if seesaw_detected():
                        status.value = f'Avoiding seesaw'

                        avoid_seesaw()
                        time_last_gyro_y = fill_array(0)
                        time_sensor_one = fill_array(0)
                        time_sensor_two = fill_array(0)
                        time_sensor_five = fill_array(0)

                        timer.set_timer("obstacle_detect_cooldown", 1.5)
                        timer.set_timer("stuck_cooldown", 4)
                        continue

                    # detected line on last frame
                    if line_status.value == "line_detected":

                        if not line_detected.value and rotation_y.value == "none" and not ramp_ahead.value:
                            line_status.value = "gap_detected"

                        if red_detected.value:
                            line_status.value = "stop"

                        if obstacle_detected() or (obstacle_detected_again() and rotation_y.value == "ramp_up"):
                            line_status.value = "obstacle_detected"

                        if silver_detected():
                            if validate_silver():
                                if program_continue():
                                    line_status.value = "position_entry"
                            else:
                                steer(0, .7)
                                time.sleep(.3)
                                steer()
                                line_status.value = "line_detected"
                            timer.set_timer("silver_cooldown", 1.3)
                            timer.set_timer("stuck_cooldown", 6)
                            continue

                    # still line detected
                    if line_status.value == "line_detected":
                        if turn_dir.value == "turn_around":
                            status.value = f'Turning around {"right" if last_turn_dir == "r" else "left"}'

                            last_turn_dir = turn_around()
                            continue

                        status.value = f'Following Line'

                        steer(line_angle.value, get_speed(line_angle.value))

                        time_silver_detected = add_time_value(time_silver_detected, silver_value.value)
                        time_last_angles = add_time_value(time_last_angles, line_angle.value)

                        if get_time_average(time_line_similarity, 15) > .88 and timer.get_timer("stuck_cooldown"):
                            avoid_stuck()
                            timer.set_timer("stuck_cooldown", 4 if rotation_y.value == "none" else 8)


                    elif line_status.value == "stop":
                        stop_for_red()
                        line_status.value = "line_detected"
                        continue


                    elif line_status.value == "gap_detected":
                        verified_gap = orientate_gap()

                        if verified_gap:
                            timer.set_timer("gap_avoid", .4)
                        else:
                            line_status.value = "line_detected"
                            min_line_size.value = 3000
                            time.sleep(.1)

                        timer.set_timer("stuck_cooldown", 4)
                        continue


                    elif line_status.value == "gap_avoid":
                        status.value = f'Avoiding gap'

                        if line_detected.value or silver_detected() or obstacle_detected():
                            min_line_size.value = 3000
                            line_status.value = "line_detected"
                            timer.set_timer("stuck_cooldown", 4)
                            continue
                        else:
                            steer(0, .6)

                        if timer.get_timer("gap_avoid"):
                            min_line_size.value = 4500
                            steer(200, .6)
                            time.sleep(1.35)
                            drive_back_until_line(.3, .6)

                            line_status.value = "line_detected"
                            time.sleep(.1)
                            timer.set_timer("stuck_cooldown", 4)
                            continue


                    elif line_status.value == "obstacle_detected":
                        status.value = f'Obstacle detected'

                        start_angle = sensor_x.value
                        obstacle_is_ramp = rotation_y.value
                        if turn_for_obstacle():
                            line_status.value = "obstacle_avoid"

                            if rotation_y.value == "none":
                                timer.set_timer("obstacle_avoid", 1.6)
                            elif rotation_y.value == "ramp_up":
                                timer.set_timer("obstacle_avoid", 3.5)
                            elif rotation_y.value == "ramp_down":
                                timer.set_timer("obstacle_avoid", 1)

                            timer.set_timer("obstacke_cooldown", .45)

                            obstacle_direction.value = obstacle_dir[obstacle_count % len(obstacle_dir)]

                            time_line_similarity = fill_array(0, 1200)
                            min_line_size.value = 6500
                            time.sleep(.1)
                        else:
                            status.value = f'Obstacle failed, returning to {start_angle}°'
                            if program_continue():
                                return_after_failed_obstacle(start_angle)
                            line_status.value = "line_detected"
                        continue


                    elif line_status.value == "obstacle_avoid":
                        status.value = f'Avoiding obstacle'

                        if obstacle_dir[obstacle_count % len(obstacle_dir)] == "l" and not timer.get_timer("obstacle_avoid"):
                            if rotation_y.value == "none":
                                steer(max_turn_angle, .7)
                            elif rotation_y.value == "ramp_up":
                                steer(max_turn_angle - 40, 1)
                            elif rotation_y.value == "ramp_down":
                                steer(max_turn_angle, .3)

                        elif obstacle_dir[obstacle_count % len(obstacle_dir)] == "l" and timer.get_timer("obstacle_avoid") and not rotation_y.value == "ramp_up":
                            steer(180, .6)
                            time.sleep(.1)
                            if rotation_y.value == "none":
                                timer.set_timer("obstacle_avoid", .7)
                            elif rotation_y.value == "ramp_up":
                                timer.set_timer("obstacle_avoid", 1.9)
                            elif rotation_y.value == "ramp_down":
                                timer.set_timer("obstacle_avoid", .75)

                        elif obstacle_dir[obstacle_count % len(obstacle_dir)] == "r" and not timer.get_timer("obstacle_avoid"):
                            if rotation_y.value == "none":
                                steer(-max_turn_angle, .7)
                            elif rotation_y.value == "ramp_up":
                                steer(-max_turn_angle + 40, 1)
                            elif rotation_y.value == "ramp_down":
                                steer(-max_turn_angle, .3)

                        elif obstacle_dir[obstacle_count % len(obstacle_dir)] == "r" and timer.get_timer("obstacle_avoid") and not rotation_y.value == "ramp_up":
                            steer(-180, .6)
                            time.sleep(.1)
                            if rotation_y.value == "none":
                                timer.set_timer("obstacle_avoid", .7)
                            elif rotation_y.value == "ramp_up":
                                timer.set_timer("obstacle_avoid", 1.9)
                            elif rotation_y.value == "ramp_down":
                                timer.set_timer("obstacle_avoid", .75)

                        if get_time_average(time_line_similarity, 15) > .88 and timer.get_timer("stuck_cooldown"):
                            steer(180 if obstacle_dir[obstacle_count % len(obstacle_dir)] == "r" else -180, .7)
                            time.sleep(.4)
                            steer()
                            time.sleep(.2)
                            timer.set_timer("stuck_cooldown", 10)

                        if line_detected.value and timer.get_timer("obstacke_cooldown"):
                            min_line_size.value = 3000
                            line_status.value = "obstacle_orientate"

                            if obstacle_is_ramp == "none":
                                orientate_after_obstacle(obstacle_dir[obstacle_count % len(obstacle_dir)])

                                if line_detected.value:
                                    steer(200, .7)
                                    time.sleep(1)
                                    steer()
                            else:
                                steer(0, .3 if obstacle_is_ramp == "ramp_down" else .8)
                                time.sleep(.6 if obstacle_is_ramp == "ramp_down" else 1)
                                steer(180 if obstacle_dir[obstacle_count % len(obstacle_dir)] == "r" else -180, .7)
                                time.sleep(.3)

                            time_sensor_one = fill_array(300)
                            time_sensor_two = fill_array(300)
                            time_sensor_five = fill_array(300)

                            obstacle_count += 1
                            line_status.value = "line_detected"
                            timer.set_timer("stuck_cooldown", 4)
                            continue


                    elif line_status.value == "position_entry":
                        status.value = f'Positioning for entry'
                        if not speed_zone:
                            time.sleep(.5)
                        if rotation_y.value == "none":
                            if position_for_entry():
                                status.value = f'Reached the evacuation zone entrance (zone handling not included in this build)'
                                line_status.value = "line_detected"
                                run = False
                                steer()
                        else:
                            status.value = f'Reached the evacuation zone entrance (zone handling not included in this build)'
                            line_status.value = "line_detected"
                            run = False
                            steer()

                elif objective.value == "debug":
                    status.value = f'Debugging'

                    time.sleep(1)


        elif calibrate_color_status.value == "calibrate":
            if not calibration_switched_light:
                if calibration_color.value == "l-bz" or calibration_color.value == "l-bv":
                    switch_lights(False)
                else:
                    switch_lights(True)
                calibration_switched_light = True

        if time.perf_counter() - iteration_limit_time < 1 / max_iterations:
            time.sleep(abs(1 / max_iterations - (time.perf_counter() - iteration_limit_time)))
        iteration_limit_time = time.perf_counter()

        counter += 1
        if time.perf_counter() - iteration_time > 1:
            iterations_control.value = int(counter / (time.perf_counter() - iteration_time))
            iteration_time = time.perf_counter()
            counter = 0

    steer()
