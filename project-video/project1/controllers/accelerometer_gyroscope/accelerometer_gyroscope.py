"""accelerometer_gyroscope controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

acc_sensor = robot.getDevice('accelerometer')
acc_sensor.enable(timestep)
gyro_sensor = robot.getDevice('gyro')
gyro_sensor.enable(timestep)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    acc_value = acc_sensor.getValues()
    gyro_value = gyro_sensor.getValues()
    
    round_acc_value = [f"{num:.2f}" for num in acc_value]
    round_gyro_value = [f"{num:.2f}" for num in gyro_value]
    
    print("IMU Sensor value: {} {}".format(round_acc_value, round_gyro_value))
    
    left_motor.setVelocity(2)
    right_motor.setVelocity(-2)

# Enter here exit cleanup code.
