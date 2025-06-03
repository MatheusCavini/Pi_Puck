import paho.mqtt.client as mqtt
import json
import time
from pipuck.pipuck import PiPuck
from random import randint
import math


#================= GAME DEFINITIONS ===================#
msg = None
MY_ID = "14"
MY_X = None
MY_Y = None
MY_ANGLE = None
CHASERS_IDS = ["14", "00", "01"]
RUNNER_ID = "44"
#============== END OF GAME DEFINITIONS ================#

##============= UTILITY FUNCTIONS =============##
# Arena safety bounds
X_lower = 0.1
X_upper = 1.9
Y_lower = 0.1
Y_upper = 0.9


# Led blinking
def blink_3_times(color):
    for i in range(3):
        pipuck.set_leds_colour(color)
        time.sleep(0.05)
        pipuck.set_leds_colour("off")
        time.sleep(0.05)
    pipuck.set_leds_colour("off")

# Stop motors
def stop():
    pipuck.epuck.set_motor_speeds(0,0)

# Go forward at costante speed
def drive_forward(speed):
    pipuck.epuck.set_motor_speeds(speed, speed)

# Turn an angular amount at a given speed
def steer(amount, speed):
    global MY_ANGLE
    stop()
    current_angle = MY_ANGLE
    target_angle = (current_angle + amount) % 360
    while abs(MY_ANGLE - target_angle) > 10: #10 deg tolerance for steering
        if amount > 0:
            pipuck.epuck.set_motor_speeds(speed, -speed)
        else:
            pipuck.epuck.set_motor_speeds(-speed, speed)
        time.sleep(0.1)
    stop()


# Check if the robot is out of bounds
def is_out_of_bounds():
    global MY_X, MY_Y
    if MY_X < X_lower or MY_X > X_upper or MY_Y < Y_lower or MY_Y > Y_upper:
        print(f"Robot {MY_ID} is getting out of bounds!")
        return True
    return False

# Check if the robot is close to other robots
def is_close_to_other_robots(distance_threshold=0.15):
    global msg, MY_X, MY_Y
    for robot_id in msg:
        if robot_id != MY_ID and msg[robot_id] is not None:
            robot_x = msg[robot_id]['position'][0]
            robot_y = msg[robot_id]['position'][1]
            distance = ((MY_X - robot_x) ** 2 + (MY_Y - robot_y) ** 2) ** 0.5
            if distance < distance_threshold:
                print(f"Robot {MY_ID} is too close to robot {robot_id}!")
                return True
    return False


# Send hello to robot under a certain distance
def send_hello_to_near_robots(distance_threshold=0.30):
    global msg
    for robot_id in msg:
        if robot_id != MY_ID and msg[robot_id] is not None:
            robot_x = msg[robot_id]['position'][0]
            robot_y = msg[robot_id]['position'][1]
            distance = ((MY_X - robot_x) ** 2 + (MY_Y - robot_y) ** 2) ** 0.5
            if distance < distance_threshold:
                print(f"Sending hello to robot {robot_id}!")
                client.publish("robot/" + robot_id, f"Hello from robot {MY_ID}!")

# Drive to especified arena coordinates
def drive_to(x_to, y_to, rot_to):
    global X_pos, Y_pos, angle
    dX = x_to - X_pos
    dY = y_to - Y_pos

    # While not at target (10cm tolerance)
    while(abs(dX) > 0.1 or abs(dY) > 0.1):
        # Calculates distances and heading
        dX = x_to - X_pos
        dY = y_to - Y_pos
        heading_angle = math.atan2(dX, dY) * 180 / math.pi 
        heading_angle = (heading_angle + 360) % 360
        # Calculates angle to turn based on heading
        d_Theta = heading_angle - angle

        # Angular tolerance of 10 degrees
        # If more than 10 degrees from heading, turn
        if abs(d_Theta) > 10:
            steer(d_Theta, 150)

        # Otherwise, go forward
        else:
            drive_forward(500)

        # Check if out of bounds or too close to other robots
        # Then, turn away and get some distance
        if is_out_of_bounds() or is_close_to_other_robots():
            steer(randint(150,210), 250)
            drive_forward(700)
            time.sleep(1)
            break
        time.sleep(0.1)

# Use proportional control to get to a target point
def control_to(x_to, y_to, dt):
    global MY_X, MY_Y, MY_ANGLE
    kp = 100
    kh = 3
    dX = x_to - MY_X
    dY = y_to - MY_Y
    heading_angle = math.atan2(dX, dY) * 180 / math.pi
    heading_angle = (heading_angle + 360) % 360
    distance = ((dX) ** 2 + (dY) ** 2) ** 0.5

    d_Theta = heading_angle - MY_ANGLE
    # control motors based on distance and angle
    right_speed = kp * distance - kh * d_Theta
    left_speed = kp * distance + kh * d_Theta
    # set motor speeds
    pipuck.epuck.set_motor_speeds(int(left_speed), int(right_speed))
    return distance, d_Theta

##============= END OF UTILITY FUNCTIONS =============##



##==================== MQTT SETUP ======================##

# Define variables and callbacks
Broker = "192.168.178.56"  # Replace with your broker address
Port = 1883 # standard MQTT port

# function to handle connection
def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))
    client.subscribe("robot_pos/all")
    client.subscribe(f"robot/{MY_ID}")  # Subscribe to the robot's own topic
    
# function to handle incoming messages
def on_message(client, userdata, message):
    global msg, MY_X, MY_Y, MY_ANGLE
    try:
        if message.topic == f"robot/{MY_ID}":
            print(f"Message received on {message.topic}: {message.payload.decode()}")
            blink_3_times("cyan")
            
            
        elif message.topic == "robot_pos/all":
            data = json.loads(message.payload.decode())
            msg = data
            if MY_ID in msg and msg[MY_ID] is not None:
                MY_X = msg[MY_ID]['position'][0]
                MY_Y = msg[MY_ID]['position'][1]
                MY_ANGLE = msg[MY_ID]['angle']
            else:
                MY_X = None
                MY_Y = None
                MY_ANGLE = None
    except json.JSONDecodeError:
        print(f"Invalid JSON: {message.payload.decode()}")
        msg = None

# Initialize MQTT client
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

client.connect(Broker, Port, 60)

client.loop_start() # Start listening loop in separate thread

##================= END OF MQTT SETUP ==================##

# Initialize the PiPuck
pipuck = PiPuck(epuck_version=2)

##================== MAIN LOOP ===================##
CATCHED_FLAG = False

# Wai until the robot is in the arena
while MY_X is None or MY_Y is None:
    stop()
    client.publish(f"robot/{MY_ID}", f"Robot {MY_ID} is not in arena...")
    time.sleep(1)

print(f"Robot {MY_ID} is in arena! X: {MY_X}, Y: {MY_Y}")

while True:
    # Changes behavior according to the role
    if MY_ID == RUNNER_ID: #ROLE RUNNER
        # Gets chasers positions
        chasers_positions = []
        for chaser in CHASERS_IDS:
            if chaser in msg and msg[chaser] is not None:
                chaser_x = msg[chaser]['position'][0]
                chaser_y = msg[chaser]['position'][1]
                chasers_positions.append((chaser_x, chaser_y))

        if len(chasers_positions) > 0:
            print(f"Runner {MY_ID} sees chasers {CHASERS_IDS} at positions {chasers_positions}")
            # Calculate the point furthest from the chasers

        else:
            print(f"No chasers detected in the arena...")

        

       
    else: #ROLE CHASER
        # Simple strategy: get runner postion and try to go there
        runner_position = []
        if RUNNER_ID in msg and msg[RUNNER_ID] is not None:
            runner_x = msg[RUNNER_ID]['position'][0]
            runner_y = msg[RUNNER_ID]['position'][1]
            runner_position.append((runner_x, runner_y))
            print(f"Chaser {MY_ID} sees runner {RUNNER_ID} at position {runner_position}")
            
            # Check if the chaser is close to the runner
            distance = ((MY_X - runner_x) ** 2 + (MY_Y - runner_y) ** 2) ** 0.5
            while distance > 0.20:
                dt = 0.1
                distance, d_Theta = control_to(runner_x, runner_y, dt)
                print(f"Chaser {MY_ID} is moving towards the runner. Distance: {distance:.2f}; Deviation: {d_Theta:.2f}º")
                time.sleep(dt)

                
            print(f"Chaser {MY_ID} caught the runner!")
            blink_3_times("green")
            CATCHED_FLAG = True
            break
        else:
            print("No runner detected in the arena...")

        

# Stop the MQTT client loop
pipuck.epuck.set_motor_speeds(0,0)
client.loop_stop() 