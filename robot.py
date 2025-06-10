
class Robot:
    def __init__(self, sim: bool = False, epuck_version: int = 2,
                 tof_sensors=(False, False, False, False, False, False),
                 yrl_expansion=False):
        self.sim = sim
        if not self.sim:
            from pipuck.pipuck import PiPuck  # only used for real robot
            self.robot = PiPuck(epuck_version=epuck_version,
                                tof_sensors=tof_sensors,
                                yrl_expansion=yrl_expansion)
            self.epuck = self.robot.epuck
        else:
            # For Webots simulated e-puck
            from controller import Robot as WebotsRobot, LED
            self.robot = WebotsRobot()
            self.epuck = epuck(sim=True, robot=self.robot)
            self.time_step = int(self.robot.getBasicTimeStep())
            self.leds = [
                self.robot.getDevice("led0"),
                self.robot.getDevice("led1"),
                self.robot.getDevice("led2"),
            ]
            

    def step(self, time=0):
        if time == 0:
            time = self.time_step
        if self.sim:
            return self.robot.step(time)
        else:
            raise NotImplementedError("step() can only be used in simulation")

    #--- Skipped other led methods ---#

    def set_led_color(self, led_index: int, color: str):
        if self.sim:
            # Webots simulated LED control (need to access actual LED devices)
            self.leds[led_index].set(1 if color != 'off' else 0)
        else:
            self.robot.set_led_colour(led_index, color)

    def set_leds_colour(self, colour: str):
        if self.sim:
            # Map colour names to RGB values supported by Webots LED API
            colour_map = {
                'off': 0x000000,
                'red': 0xFF0000,
                'green': 0x00FF00,
                'blue': 0x0000FF,
                'yellow': 0xFFFF00,
                'cyan': 0x00FFFF,
                'magenta': 0xFF00FF,
                'white': 0xFFFFFF,
                'black': 0x000000,
            }
            value = colour_map.get(colour.lower(), 0x000000)
            for led in self.leds:
                led.set(value)
        else:
            self.robot.set_leds_colour(colour)

        
    def get_battery_state(self):
        if self.sim:
            return False, 4.0, 1.0  # fake values for sim
        return self.robot.get_battery_state()

    def speaker_enable(self, state: bool):
        if not self.sim:
            self.robot.speaker_enable(state)

    def cleanup(self):
        if not self.sim:
            del self.robot

class epuck():
    def __init__(self, sim: bool = False, robot: Robot = None):
        self.sim = sim
        self.robot = robot
        self.left_motor = self.robot.getDevice('left wheel motor')
        self.right_motor = self.robot.getDevice('right wheel motor')
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)

    def set_motor_speeds(self, left_speed: int, right_speed: int):
        if self.sim:
            factor = 6/1000
            self.left_motor.setVelocity(left_speed*factor)
            self.right_motor.setVelocity(right_speed*factor)
        else:
            self.robot.epuck.set_motor_speeds(left_speed, right_speed)


class Time():
    def __init__(self, robot):
        self.robot = robot

    def sleep(self, seconds: float):
        if self.robot.sim:
            self.robot.step(int(seconds * 1000))
        