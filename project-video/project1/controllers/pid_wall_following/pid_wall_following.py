from controller import Robot

def calculate_motor(signal):
    return (signal/100)*6.28

def run_robot(robot):
    #timestep = int(robot.getBasicTimeStep())
    timestep = 64
    
    #define sensor
    sensor0 = robot.getDevice('ps0')
    sensor0.enable(timestep)
    sensor1 = robot.getDevice('ps1')
    sensor1.enable(timestep)
    sensor2 = robot.getDevice('ps2')
    sensor2.enable(timestep)
    sensor3 = robot.getDevice('ps3')
    sensor3.enable(timestep)
    sensor4 = robot.getDevice('ps4')
    sensor4.enable(timestep)
    sensor5 = robot.getDevice('ps5')
    sensor5.enable(timestep)
    sensor6 = robot.getDevice('ps6')
    sensor6.enable(timestep)
    sensor7 = robot.getDevice('ps7')
    sensor7.enable(timestep)
    
    #define motor
    left_motor = robot.getDevice('left wheel motor')
    right_motor = robot.getDevice('right wheel motor')
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))
    left_motor.setVelocity(0)
    right_motor.setVelocity(0)
    
    #parameter
    #normal_speed = 3.6
    #fast_speed = 6.28
    #pid_parameter = [0.33, 0.00015, 0.27]
    pid_parameter = [0.1, 0.02, 2]
    error = [0, 0, 0]
    set_point = 120
    control = [0, 0, 0]
    pid_control = 0
    
    while robot.step(timestep) != -1:
        normal_speed = 30
        fast_speed = 100
        
        sensor0_val = sensor0.getValue()
        sensor1_val = sensor1.getValue()
        sensor2_val = sensor2.getValue()
        sensor3_val = sensor3.getValue()
        sensor4_val = sensor4.getValue()
        sensor5_val = sensor5.getValue()
        sensor6_val = sensor6.getValue()
        sensor7_val = sensor7.getValue()
        
        error[0] = max(sensor2_val, sensor1_val);
        error[0] = set_point - error[0]
        control[0] = error[0]*pid_parameter[0]
        
        error[1] = error[1] + error[0]
        if error[1] > 100:
            error[1] = 100
        if error[1] <= -100:
            error[1] = -100
        control[1] = error[1]*pid_parameter[1]
        
        control[2] = error[2]*pid_parameter[2]   
        error[2] = error[0]
        
        pid_control = control[0]+control[1]-control[2]     
        
        if pid_control >= (100-normal_speed):
            pid_control = (100-normal_speed)
        if pid_control <= -(100-normal_speed):
            pid_control = -(100-normal_speed)
    
        if (sensor0_val >= 80 or sensor7_val >= 80):
            max_speed = calculate_motor(fast_speed)
            left_motor.setVelocity(-max_speed)
            right_motor.setVelocity(max_speed)
        else:
            #if error[0] >= -3 and error[0] <= 3:
            #    max_speed = calculate_motor(fast_speed)
            #    left_motor.setVelocity(max_speed)
            #    right_motor.setVelocity(max_speed)  
            #else:
            speed = calculate_motor(normal_speed)
            pid_control = calculate_motor(pid_control)
            
            left_motor.setVelocity(speed+pid_control)
            right_motor.setVelocity(speed-pid_control)
        
        #round_error = [f"{num:.2f}" for num in error]
        #round_control = [f"{num:.2f}" for num in control]
        
        #print("Error: {} Control: {}".format(round_error, round_control))
        print("Error: {:.2f} Control: {:.2f}".format(error[0], pid_control))
    
if __name__ == "__main__":
    my_robot = Robot()
    run_robot(my_robot)