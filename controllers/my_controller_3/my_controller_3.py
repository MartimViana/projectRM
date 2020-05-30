"""my_controller_3 controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot

"""CLASSES"""
class PIDController():
    def __init__(self, P, I, D, initial, final):
        self.P = P
        self.I = I
        self.D = D
        self.previousError = 0
        self.previousIntegral = 0
        self.initial = initial
        self.final = final
    
    def iterate(self, current, dt):
        error = final - current
        integral = self.previousIntegral + error * dt
        derivative = (error - self.previousError) / dt
        result = self.P * error + self.I * integral + self.D * derivative
        self.previousError = error
        self.previousIntegral = integral
        return result

class Controller(PIDController):
    def __init_(self, P, I, D, initial, final, sensor, actuators):
        super().__init__(P, I, D, initial, final)


# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())


while robot.step(timestep) != -1:
    pass