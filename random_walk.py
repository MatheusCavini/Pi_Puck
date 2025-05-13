import paho.mqtt.client as mqtt
import json
import time
from pipuck.pipuck import PiPuck
from random import randint
MY_ID = "33"

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

# function to handle incoming messages
def on_message(client, userdata, message):
    global msg, X_pos, Y_pos, angle
    try:
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
    stop()
    current_angle = angle
    while (angle -current_angle) < amount:
        pipuck.epuck.set_motor_speeds(speed, -speed)
        time.sleep(0.1)
    stop()
    
    

    

# Wai until the robot is in the arena
while X_pos is None or Y_pos is None:
    stop()
    print(f"Robot {MY_ID} is not in arena...")
    time.sleep(1)

print(f"Robot {MY_ID} is in arena! X: {X_pos}, Y: {Y_pos}")




def change_direction():
    # Randomly change direction
    pipuck.epuck.set_motor_speeds(0, 0)
    time_rotating = randint(1,5)
    pipuck.epuck.set_motor_speeds(250, -250)
    time.sleep(time_rotating)
    pipuck.epuck.set_motor_speeds(0, 0)

for _ in range(10):
    time_before_change = randint(1, 10)
    drive_forward(500)
    for i in range(time_before_change * 2):
        time.sleep(0.5) #check position every 0.5 seconds
        if X_pos < X_lower or X_pos > X_upper or Y_pos < Y_lower or Y_pos > Y_upper:
            print(f"Robot {MY_ID} is getting out of bounds! X: {X_pos}, Y: {Y_pos}")
            break
    steer(90, 250)
	
    
# Stop the MQTT client loop
pipuck.epuck.set_motor_speeds(0,0)
client.loop_stop()  
