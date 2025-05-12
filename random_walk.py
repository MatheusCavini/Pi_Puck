import paho.mqtt.client as mqtt
import json
import time
from pipuck.pipuck import PiPuck
from random import randint

MY_ID = "33"

# Define variables and callbacks
Broker = "192.168.178.56"  # Replace with your broker address
Port = 1883 # standard MQTT port

# Variable to save the message
msg = None
X_pos = None
Y_pos = None


# function to handle connection
def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))
    client.subscribe("robot_pos/all")

# function to handle incoming messages
def on_message(client, userdata, msg):
    try:
        data = json.loads(msg.payload.decode())
        msg = data
        if msg[MY_ID] != None:
            print(f"Robot {MY_ID} is on the arena")
        else:
            print(f"Unable to get robot {MY_ID} position")
    except json.JSONDecodeError:
        print(f'invalid json: {msg.payload}')
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

# Get info from the robot
for _ in range(10):
    time.sleep(1)
	
    
# Stop the MQTT client loop
pipuck.epuck.set_motor_speeds(0,0)
client.loop_stop()  
