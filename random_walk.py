import paho.mqtt.client as mqtt
import json
import time
from pipuck.pipuck import PiPuck
from random import randint
MY_ID = "18"

# Define variables and callbacks
Broker = "192.168.178.56"  # Replace with your broker address
Port = 1883 # standard MQTT port

# Variable to save the position message
msg = None
X_pos = None
Y_pos = None
angle = None



# function to handle connection
def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))
    client.subscribe("robot_pos/all")
    client.subscribe(f"robot/{MY_ID}")  # Subscribe to the robot's own topic
    
def blink_3_times(color):
    for i in range(3):
        pipuck.set_leds_colour(color)
        time.sleep(0.05)
        pipuck.set_leds_colour("off")
        time.sleep(0.05)
    pipuck.set_leds_colour("off")

# function to handle incoming messages
def on_message(client, userdata, message):
    global msg, X_pos, Y_pos, angle
    try:
        if message.topic == f"robot/{MY_ID}":
            print(f"Message received on {message.topic}: {message.payload.decode()}")
            blink_3_times("cyan")
            
            
        elif message.topic == "robot_pos/all":
            data = json.loads(message.payload.decode())
            msg = data
            if MY_ID in msg and msg[MY_ID] is not None:
                X_pos = msg[MY_ID]['position'][0]
                Y_pos = msg[MY_ID]['position'][1]
                angle = msg[MY_ID]['angle']
            else:
                X_pos = None
                Y_pos = None
                angle = None
    except json.JSONDecodeError:
        print(f"Invalid JSON: {message.payload.decode()}")
        msg = None
        


# Initialize MQTT client
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

client.connect(Broker, Port, 60)

client.loop_start() # Start listening loop in separate thread

# Initialize the PiPuck
pipuck = PiPuck(epuck_version=2)


## Define the arena boundaries
X_lower = 0.1
X_upper = 1.9
Y_lower = 0.1
Y_upper = 0.9

def stop():
    pipuck.epuck.set_motor_speeds(0,0)

def drive_forward(speed):
    pipuck.epuck.set_motor_speeds(speed, speed)



def steer(amount, speed):
    global angle
    stop()
    current_angle = angle
    target_angle = (current_angle + amount) % 360
    while abs(angle - target_angle) > 10: #10 deg tolerance for steering
        if amount > 0:
            pipuck.epuck.set_motor_speeds(speed, -speed)
        else:
            pipuck.epuck.set_motor_speeds(-speed, speed)
        time.sleep(0.1)
    stop()

def is_out_of_bounds():
    global X_pos, Y_pos
    if X_pos < X_lower or X_pos > X_upper or Y_pos < Y_lower or Y_pos > Y_upper:
        print(f"Robot {MY_ID} is getting out of bounds!")
        return True
    return False
    
def is_close_to_other_robots():
    global msg, X_pos, Y_pos
    for robot_id in msg:
        if robot_id != MY_ID and msg[robot_id] is not None:
            robot_x = msg[robot_id]['position'][0]
            robot_y = msg[robot_id]['position'][1]
            distance = ((X_pos - robot_x) ** 2 + (Y_pos - robot_y) ** 2) ** 0.5
            if distance < 0.15:
                print(f"Robot {MY_ID} is too close to robot {robot_id}!")
                return True
    return False

def send_hello_to_near_robots():
    global msg
    for robot_id in msg:
        if robot_id != MY_ID and msg[robot_id] is not None:
            robot_x = msg[robot_id]['position'][0]
            robot_y = msg[robot_id]['position'][1]
            distance = ((X_pos - robot_x) ** 2 + (Y_pos - robot_y) ** 2) ** 0.5
            if distance < 0.30:
                print(f"Sending hello to robot {robot_id}!")
                client.publish("robot/" + robot_id, f"Hello from robot {MY_ID}!")
    

# Wai until the robot is in the arena
while X_pos is None or Y_pos is None:
    stop()
    client.publish(f"robot/{MY_ID}", f"Robot {MY_ID} is not in arena...")
    time.sleep(1)

print(f"Robot {MY_ID} is in arena! X: {X_pos}, Y: {Y_pos}")


# Main Loop: random walk logic
for _ in range(10): # 10 iterations of random walk

    # Get a random time to wait before changing direction
    time_before_change = randint(1, 10)
    drive_forward(700)

    # While is driving forward, check for bounds and colision every 0.5 seconds
    for i in range(time_before_change * 2):
        time.sleep(0.5) #check position every 0.5 seconds

        # Check if robot is getting out of bounds or too close to other robots
        if is_out_of_bounds() or is_close_to_other_robots():
            steer(randint(120,240), 250)
            drive_forward(500)
            time.sleep(2)

        if i%2 == 0:
            send_hello_to_near_robots()


    angle_to_steer = randint(-180,180)
    steer(angle_to_steer, 250)
	
    
# Stop the MQTT client loop
pipuck.epuck.set_motor_speeds(0,0)
client.loop_stop()  
