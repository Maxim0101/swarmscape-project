from pololu_3pi_2040_robot import robot
from pololu_3pi_2040_robot.extras import editions
import time
from machine import Pin

display = robot.Display()
motors = robot.Motors()
line_sensors = robot.LineSensors()
encoders = robot.Encoders()
button_a = robot.ButtonA()
button_b = robot.ButtonB()
button_c = robot.ButtonC()

GAP_DISTANCE = 375 
MOTOR_SPEED = motors.MAX_SPEED // 12

MAX_SPEED = 600  
CALIBRATION_SPEED = 800
CALIBRATION_COUNT = 100

def move_forward(target_counts):
    left_start, right_start = encoders.get_counts()
    motors.set_speeds(MOTOR_SPEED, MOTOR_SPEED)
    
    while True:
        left_current, right_current = encoders.get_counts()
        left_diff = left_current - left_start
        right_diff = right_current - right_start
        
        if left_diff >= target_counts and right_diff >= target_counts:
            break

        display.fill_rect(40, 48, 64, 16, 0)
        display.text(f"L: {left_diff:>8}", 0, 48)
        display.text(f"R: {right_diff:>8}", 0, 56)
        display.show()
    
    motors.set_speeds(0, 0)
    time.sleep(0.5)
    
def turn_around():
    left_start, right_start = encoders.get_counts()
    
    motors.set_speeds(MOTOR_SPEED, -MOTOR_SPEED)
    
    target_counts = 450
    
    while True:
        left_current, right_current = encoders.get_counts()
        left_diff = left_current - left_start
        right_diff = right_start - right_current

        if left_diff >= target_counts and right_diff >= target_counts:
            break

        display.fill_rect(40, 48, 64, 16, 0)
        display.text(f"L: {left_diff:>8}", 0, 48)
        display.text(f"R: {right_diff:>8}", 0, 56)
        display.show()
    
    motors.set_speeds(0, 0)
    time.sleep(0.5)
    
def calibrate_line_sensors():
    display.fill(0)
    display.text("Calibrating...", 0, 20)
    display.show()
    time.sleep_ms(500)

    motors.set_speeds(CALIBRATION_SPEED, -CALIBRATION_SPEED)
    for i in range(CALIBRATION_COUNT // 4):
        line_sensors.calibrate()

    motors.off()
    time.sleep_ms(200)

    motors.set_speeds(-CALIBRATION_SPEED, CALIBRATION_SPEED)
    for i in range((CALIBRATION_COUNT // 2) - 3):
        line_sensors.calibrate()

    motors.off()
    time.sleep_ms(200)

    motors.set_speeds(CALIBRATION_SPEED, -CALIBRATION_SPEED)
    for i in range(CALIBRATION_COUNT // 4):
        line_sensors.calibrate()
    
    display.fill(0)
    display.show()
    
    motors.off()
    
    
    
t1 = 0
t2 = time.ticks_us()
p = 0
line = []
run_motors = True 
last_update_ms = 0

            
def follow_line():
    last_p = 0
    global p, t1, t2, line, MAX_SPEED
    run_motors = True
    
    display.fill(0)
    display.text("Following Line...", 0, 20)
    display.show()

    while run_motors:
        # Read calibrated line sensor values
        line = line_sensors.read_calibrated()[:]
        line_sensors.start_read()
        t1 = t2
        t2 = time.ticks_us()

        # Check if all sensors are below the threshold (indicating a gap)
        if all(sensor < 300 for sensor in line):
            run_motors = False
            motors.off()
            break  # Stop the robot and exit the loop

        # Estimate the position of the line
        l = (1000 * line[1] + 2000 * line[2] + 3000 * line[3] + 4000 * line[4]) // sum(line)
        p = l - 2000
        d = p - last_p
        last_p = p
        pid = p * 90 + d * 2000

        # Calculate motor speeds using the PID value
        min_speed = 0
        left = max(min_speed, min(MAX_SPEED, MAX_SPEED + pid))
        right = max(min_speed, min(MAX_SPEED, MAX_SPEED - pid))

        # Only move the motors if the robot is supposed to be running
        if run_motors:
            motors.set_speeds(left, right)
        else:
            motors.off()
            break
            
def take_photo():
    display.fill(0)
    display.text("Taking Photo", 0, 20)
    display.show()
    time.sleep(1)
    display.fill(0)
    display.show()
    
    # IN DEVELOPMENT
    #CODE FOR GIVING RASPBERRYPI PHOTOS (Work in process)
    #RASPBERRYPI SENDS PHOTOS TO VERTEXAI
#     # Set GPIO pin 18 as an output
#     pause_signal_pin = Pin(18, Pin.OUT)
# 
#     while True:
#         pause_signal_pin.value(1)  # Send HIGH signal
#         time.sleep(1)  # Signal stays HIGH for 1 second
#         pause_signal_pin.value(0)  # Reset to LOW
#         time.sleep(1)  # Wait for 1 second before sending the next signal
    
            
def find_num_plants(num_plants, current_gap):
    line_finished = False
    num_plants = 0
    
    while not line_finished:
        run_motors = True
        calibrate_line_sensors()
        line = line_sensors.read_calibrated()
        line_sensors.start_read()
        if any(sensor > 300 for sensor in line):
            run_motors = True
            display.fill(0)
            display.text("Line Detected", 0, 20)
            display.show()
            time.sleep(0.5)
            follow_line()
            time.sleep(0.5)
            display.fill(0)
            display.text("Gap Detected", 0, 20)
            display.show()
            time.sleep(0.5)
            current_gap += 1
            num_plants += 1
            display.fill(0)
            display.text(f"Total Plants: {num_plants}", 0, 20)
            display.show()
            time.sleep(0.5)
            move_forward(GAP_DISTANCE)
        else:
            line_finished = True
    
    turn_around()
    display.fill(0)
    display.text("Heading Home", 0, 20)
    display.show()
    time.sleep(0.5)
    move_forward(GAP_DISTANCE)
        
    while current_gap > 0:
        run_motors = True
        calibrate_line_sensors()
        follow_line()
        time.sleep(0.5)
        current_gap -= 1
        if current_gap > 0:
            move_forward(GAP_DISTANCE)
        else:
            turn_around()

        motors.off()
        display.fill(0)
        display.text("Mission Complete", 0, 20)
        display.show()
    
    return num_plants

def run_operation(num_plants, current_gap):
        while current_gap < num_plants:
            run_motors = True
            calibrate_line_sensors()
            follow_line()
            time.sleep(0.5)
            take_photo()
            current_gap += 1
            if current_gap < num_plants:
                move_forward(GAP_DISTANCE)

        turn_around()
        display.fill(0)
        display.text("Heading Home", 0, 20)
        display.show()
        time.sleep(0.5)
        
        while current_gap > 0:
            run_motors = True
            calibrate_line_sensors()
            follow_line()
            time.sleep(0.5)
            current_gap -= 1
            if current_gap > 0:
                move_forward(GAP_DISTANCE)
            else:
                turn_around()

        motors.off()
        display.fill(0)
        display.text("Mission Complete", 0, 20)
        display.show()
            
def main():
    num_plants = 0
    current_gap = 0
    global run_motors
    
    display.fill(0)
    display.text("Press A, B, or C", 0, 20)
    display.show()

    while True:
        # Check for button A press to count plants
        if button_a.check():
            display.fill(0)
            display.text("Running Exploration...", 0, 20)
            display.show()
            num_plants = find_num_plants(num_plants, current_gap)

        # Check for button B press to run the photo-taking operation
        elif button_b.check():
            display.fill(0)
            display.text("Running Operation...", 0, 20)
            display.show()
            run_operation(num_plants, current_gap)

        # Check for button C press to end the program
        elif button_c.check():
            display.fill(0)
            display.text("Exiting...", 0, 20)
            display.show()
            motors.off()
            break

        # Add a small delay to prevent CPU overuse
        time.sleep(0.1)

main()
