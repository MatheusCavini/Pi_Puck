import paho.mqtt.client as mqtt
import json
import time
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

msg = None
MY_ID = "32"
RUNNER_ID = "44"


def compute_potential_field(x, y, alpha =20.0, beta =0.15, beta0 = 0.005):
    global msg
    if msg is None:
        
        return 0.0
    

    # Get runner positiion as Goal
    if RUNNER_ID in msg and msg[RUNNER_ID] is not None:
        Xg = msg[RUNNER_ID]['position'][0]
        Yg = msg[RUNNER_ID]['position'][1]

    # Calculate attractive potential caused by th goal
    Ug = alpha * ((x - Xg)**2 + (y - Yg)**2) 

    # Calculate repulsive potential from any other robots in the arena
    U_obs = 0.0
    for robot_id in msg:
        if robot_id != MY_ID and robot_id != RUNNER_ID and msg[robot_id] is not None:
            robot_x = msg[robot_id]['position'][0]
            robot_y = msg[robot_id]['position'][1]
            dist = np.sqrt((x - robot_x)**2 + (y - robot_y)**2)
            if dist< 0.035:
                dist = 0.035
            U_obs += beta / (beta0 + dist)
    
    # Return total potential
    return Ug + U_obs

def compute_gradient(U_func, x, y, h=1e-3):
    dU_dx = (U_func(x + h, y) - U_func(x - h, y)) / (2 * h)
    dU_dy = (U_func(x, y + h) - U_func(x, y - h)) / (2 * h)
    return np.array([dU_dx, dU_dy])


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
        if message.topic == "robot_pos/all":
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

client.loop_start()

arena_width = 2.0
arena_height = 1.0
resolution = 200

x = np.linspace(0, arena_width, resolution)
y = np.linspace(0, arena_height, resolution)
X, Y = np.meshgrid(x, y)
eta = 0.1

while True:
    # compute and execute potential field in all arena points
    Z = np.zeros_like(X)
    for i in range(resolution):
        for j in range(resolution):
            Z[i, j] = compute_potential_field(X[i, j], Y[i, j])
    # Plotting the potential field in 2D
    plt.figure(figsize=(12, 6))
    cp = plt.contourf(X, Y, Z, levels=50, cmap='viridis')
    plt.colorbar(cp, label='Potential Field Value')
    plt.title('Potential Field (2D)')
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.xlim(0, arena_width)
    plt.ylim(0, arena_height)
    # plot goal position
    if msg is not None and RUNNER_ID in msg and msg[RUNNER_ID] is not None:
        Xg = msg[RUNNER_ID]['position'][0]
        Yg = msg[RUNNER_ID]['position'][1]
        plt.scatter(Xg, Yg, color='red', marker='*', s=100, label='Goal')
    # Show the gradient direction at the robot's position
    if 'MY_X' in globals() and MY_X is not None and MY_Y is not None:
        grad = compute_gradient(compute_potential_field, MY_X, MY_Y)
        plt.quiver(MY_X, MY_Y, -eta*grad[0]/np.linalg.norm(grad), -eta*grad[1]/np.linalg.norm(grad), angles='xy', scale_units='xy', scale=1, color='red', label='Gradient Direction')
    plt.legend()
    plt.show()
    time.sleep(0.1)


client.loop_stop() 