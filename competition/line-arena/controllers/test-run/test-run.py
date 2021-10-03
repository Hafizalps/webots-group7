from controller import Robot

def kalkulasi_motor(signal):
    return (signal/100)*30

nilai_posisi = 0

""""def pid_calc():
    rata_rata = 0
    sum_ir = 0
    lastErrorLF = 0
    pid_control = 0

    for i in range(6):
        rata_rata += sensor_ir_val[i] * i * 1000
        sum_ir += sensor_ir_val[i]

    posisi = rata_rata/sum_ir #Posisi saat ini
    error = posisi - set_point_lf
    p = pid_parameter[0] * error
    d = pid_parameter[1] * (error - lastErrorLF)
    pid_control = p + d
    lastErrorLF = error"""

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

    # Deklarasi Parameter
    motor_pos_val = [0, 0]
    sensor_ir_val = [0, 0, 0, 0, 0, 0]
    sensor_jarak_val = [0, 0, 0]
    
    #pid_parameter = [0.35, 0.00001, 2.2] #Kp Ki Kd
    #pid_control = 0

    # Konversi nilai Ir sensor ke biner
    def read_ir():
        for i in range(6):
            if sensor_ir_val[i] < 600:
                sensor_ir_val[i] = 1
            else:
                sensor_ir_val[i] = 0

    # Deklarasi posisi robot
    def linefollow_pid():
        global nilai_posisi
        if sensor_ir_val == [1, 0, 0, 0, 0, 0] :
            nilai_posisi = 7
        elif sensor_ir_val == [1, 1, 1, 0, 0, 0] :
            nilai_posisi = 6
        elif sensor_ir_val == [1, 1, 0, 0, 0, 0] :
            nilai_posisi = 5
        elif sensor_ir_val == [0, 1, 0, 0, 0, 0] :
            nilai_posisi = 4
        elif sensor_ir_val == [0, 1, 1, 1, 0, 0] :
            nilai_posisi = 3
        elif sensor_ir_val == [0, 1, 1, 0, 0, 0] :
            nilai_posisi = 2
        elif sensor_ir_val == [0, 0, 1, 0, 0, 0] :
            nilai_posisi = 1
        elif sensor_ir_val == [0, 0, 1, 1, 0, 0] :
            nilai_posisi = -0
        elif sensor_ir_val == [0, 0, 0, 1, 0, 0] :
            nilai_posisi = -1
        elif sensor_ir_val == [0, 0, 0, 1, 1, 0] :
            nilai_posisi = -2
        elif sensor_ir_val == [0, 0, 1, 1, 1, 0] :
            nilai_posisi = -3
        elif sensor_ir_val == [0, 0, 0, 0, 1, 0] :
            nilai_posisi = -4
        elif sensor_ir_val == [0, 0, 0, 0, 1, 1] :
            nilai_posisi = -5
        elif sensor_ir_val == [0, 0, 0, 1, 1, 1] :
            nilai_posisi = -6
        elif sensor_ir_val == [0, 0, 0, 0, 0, 1] :
            nilai_posisi = -7       

    # Main Loop
    while robot.step(timestep) != -1:
        for i in range(2):
            motor_pos_val[i] = motor_pos[i].getValue()
        
        for i in range(6):
            sensor_ir_val[i] = sensor_ir[i].getValue()
        read_ir()
        linefollow_pid()
        #pid_calc()

        for i in range(3):
            sensor_jarak_val[i] = sensor_jarak[i].getValue()

        #if sensor_ir_val == [0, 0, 1, 1, 0, 0] :
            #motor[0].setVelocity(0)
            #motor[1].setVelocity(0)
            
        #print("Odometry sensor values: {:.3f} {:.3f}".format(motor_pos_val[0], motor_pos_val[1]))

        #print("Proximity sensor values: {:.3f} {:.3f} {:.3f}".format(sensor_jarak_val[0], sensor_jarak_val[1], sensor_jarak_val[2]))

        #print("IR sensor values: {:.3f} {:.3f} {:.3f} {:.3f} {:.3f} {:.3f}".format(sensor_ir_val[0], sensor_ir_val[1], sensor_ir_val[2], sensor_ir_val[3], sensor_ir_val[4], sensor_ir_val[5]))
        
        print(nilai_posisi)
        
        #print(*sensor_ir_val)

        #print("PD Control: {}".format(pid_control))

        motor[0].setVelocity(0)
        motor[1].setVelocity(0)

if __name__ == "__main__":
    my_robot = Robot()
    run_robot(my_robot)