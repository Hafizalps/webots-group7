"""odometer_calculation controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Accelerometer, Gyro, LED

def run_robot(robot):
    # get the time step of the current world.
    timestep = 64
    max_speed = 0.5
    
    left_motor = robot.getDevice('left wheel motor')
    right_motor = robot.getDevice('right wheel motor')
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)
    right_motor.setVelocity(0.0)
    
    acc = robot.getDevice('accelerometer')
    acc.enable(timestep)
    #gyro = robot.getDevice('gyro')
    #gyro.enable(timestep)    
    
    led_front = robot.getDevice('front led')
    led_back = robot.getDevice('back led')
    led_left = robot.getDevice('left led')
    led_right = robot.getDevice('right led')            
    
    # Main loop:
    # - perform simulation steps until Webots is stopping the controller
    while robot.step(timestep) != -1:
        acc_value = acc.getValues()
        #gyro_value = gyro.getValues()

        round_acc_value = [f"{num:.2f}" for num in acc_value]
        #round_gyro_value = [f"{num:.2f}" for num in gyro_value]

        #print("----------------------------")
        #print("IMU sensor values: {} {}".format(round_acc_value, round_gyro_value))
        print("IMU sensor values: {}".format(round_acc_value))
        
        left_motor.setVelocity(max_speed)
        right_motor.setVelocity(-max_speed)
        
        if(abs(acc_value[0]) > abs(acc_value[2])):
            led_front.set(False)
            led_back.set(False)
            led_left.set(acc_value[0] > 0.0)
            led_right.set(acc_value[0] < 0.0)
        else:
            led_front.set(acc_value[2] > 0.0)
            led_back.set(acc_value[2] < 0.0)
            led_left.set(False)
            led_right.set(False)            

if __name__ == "__main__":
    my_robot = Robot()
    run_robot(my_robot)