from controller import Robot

def run_robot(robot):
    timestep = int(robot.getBasicTimeStep())

    # Deklarasi Motor
    motor = []
    motor_nama =['motor_1', 'motor_2']
    for i in range(2):
        motor.append(robot.getDevice(motor_nama[i]))
        motor[i].setPosition(float('inf'))
        motor[i].setVelocity(0.0)

    # Deklarasi Sensor Odometry
    motor_pos = []
    motor_pos_nama =['ps_1', 'ps_2']
    for i in range(2):
        motor_pos.append(robot.getDevice(motor_pos_nama[i]))
        motor_pos[i].enable(timestep)

    # Deklarasi IR Sensor
    sensor_ir = []
    sensor_ir_nama = ['IRL2', 'IRL1', 'IRCL', 'IRCR', 'IRR1', 'IRR2']
    for i in range(6):
        sensor_ir.append(robot.getDevice(sensor_ir_nama[i]))
        sensor_ir[i].enable(timestep)
    
    # Deklarasi Sensor Proximity
    sensor_jarak = []
    sensor_jarak_nama =['ds_left', 'ds_front', 'ds_right']
    for i in range(3):
        sensor_jarak.append(robot.getDevice(sensor_jarak_nama[i]))
        sensor_jarak[i].enable(timestep)

    # Deklarasi Camera
    camera = robot.getDevice('CAM')
    camera.enable(timestep)

    motor_pos_val = [0, 0]
    sensor_ir_val = [0, 0, 0, 0, 0, 0]
    sensor_jarak_val = [0, 0, 0]

    # Main Loop
    while robot.step(timestep) != -1:
        for i in range(2):
            motor_pos_val[i] = motor_pos[i].getValue()
        
        for i in range(6):
            sensor_ir_val[i] = sensor_ir[i].getValue()

        for i in range(3):
            sensor_jarak_val[i] = sensor_jarak[i].getValue()


        #print("Odometry sensor values: {:.3f} {:.3f}".format(motor_pos_val[0], motor_pos_val[1]))

        #print("Proximity sensor values: {:.3f} {:.3f} {:.3f}".format(sensor_jarak_val[0], sensor_jarak_val[1], sensor_jarak_val[2]))

        #print("IR sensor values: {:.3f} {:.3f} {:.3f} {:.3f} {:.3f} {:.3f}".format(sensor_ir_val[0], sensor_ir_val[1], sensor_ir_val[2], sensor_ir_val[3], sensor_ir_val[4], sensor_ir_val[5]))

        motor[0].setVelocity(0)
        motor[1].setVelocity(0)

if __name__ == "__main__":
    my_robot = Robot()
    run_robot(my_robot)