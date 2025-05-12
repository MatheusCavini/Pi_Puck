import paho.mqtt.client as mqtt
import json
import time
from pipuck.pipuck import PiPuck
from random import randint
MY_ID = "2"

# Define variables and callbacks
Broker = "192.168.178.56"  # Replace with your broker address
Port = 1883 # standard MQTT port

# Variable to save the message
msg = None
X_pos = None
Y_pos = None
msg = None

# function to handle connection
def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))
    client.subscribe("robot_pos/all")

# function to handle incoming messages
def on_message(client, userdata, message):
    global msg, X_pos, Y_pos
    try:
        data = json.loads(message.payload.decode())
        msg = data
        if MY_ID in msg and msg[MY_ID] is not None:
            print(f"Robot {MY_ID} is on the arena")
            X_pos = msg[MY_ID]['position'][0]
            Y_pos = msg[MY_ID]['position'][1]
        else:
            print(f"Unable to get robot {MY_ID} position")
    except json.JSONDecodeError:
        print(f"Invalid JSON: {message.payload.decode()}")
        msg = None
        X_pos = None
        Y_pos = None


# Initialize MQTT client
client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

client.connect(Broker, Port, 60)

client.loop_start() # Start listening loop in separate thread

# Initialize the PiPuck
pipuck = PiPuck(epuck_version=2)

X_lower = 0.1
X_upper = 1.9
Y_lower = 0.1
Y_upper = 0.9

# Random walk logic
for _ in range(10):
    time_before_change = randint(1, 10)
    pipuck.epuck.set_motor_speeds(500, 500)
    time.sleep(time_before_change)
    pipuck.epuck.set_motor_speeds(0, 0)
    # Randomly change direction
    time_rotating = randint(1,5)
    pipuck.epuck.set_motor_speeds(250, -250)
    time.sleep(time_rotating)
    pipuck.epuck.set_motor_speeds(0, 0)
	
    
# Stop the MQTT client loop
pipuck.epuck.set_motor_speeds(0,0)
client.loop_stop()  
