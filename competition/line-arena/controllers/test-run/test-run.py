from controller import Robot

def kalkulasi_motor(signal):
    return (signal/100)*30

nilai_posisi = 0
kecepatan_lambat = 20
kecepatan_normal = 60
kecepatan_cepat = 100
pid_control = 0

def run_robot(robot):
    global kecepatan_lambat, kecepatan_normal, kecepatan_cepat
    timestep = 16

    # Deklarasi Parameter
    motor_pos_val = [0, 0]
    sensor_ir_val = [0, 0, 0, 0, 0, 0]
    sensor_jarak_val = [0, 0, 0]
    
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
    
    # Konversi nilai Ir sensor ke biner
    def read_ir():
        for i in range(6):
            if sensor_ir_val[i] < 600:
                sensor_ir_val[i] = 1
            else:
                sensor_ir_val[i] = 0

    # Deklarasi posisi robot
    def robot_pos():
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

    def kalkulasi_pid():
        global pid_control
        pid_parameter = [43, 0.2, 145] #Kp Ki Kd
        error = [0, 0, 0]
        control = [0, 0, 0]
        set_point = 0

        # Kendali Proporsional (P)
        error[0] = set_point - nilai_posisi
        control[0] = error[0] * pid_parameter[0]

        # Kendali Integral (I)
        error[1] = error[1] + error[0]
        if error[1] > 7:
            error[1] = 7
        if error[1] <= -8:
            error[1] = -7
        control[1] = error[1] * pid_parameter[1]

        # Kendali Differensial (D)
        control[2] = (error[0] - error[2]) * pid_parameter[2]   
        error[2] = error[0]

        pid_control = control[0] + control[1] + control[2]
        pid_control = kalkulasi_motor(pid_control)
        
    # Main Loop
    while robot.step(timestep) != -1:
        for i in range(2):
            motor_pos_val[i] = motor_pos[i].getValue()
        
        for i in range(6):
            sensor_ir_val[i] = sensor_ir[i].getValue()
        read_ir()
        robot_pos()

        for i in range(3):
            sensor_jarak_val[i] = sensor_jarak[i].getValue()

        kalkulasi_pid()
        kecepatan = kalkulasi_motor(kecepatan_normal)
        kiri = kecepatan + pid_control
        kanan = kecepatan - pid_control

        if kiri > kecepatan_cepat :
            kiri = kecepatan_cepat
        if kiri < kecepatan_lambat :
            kiri = kecepatan_lambat
        if kanan > kecepatan_cepat :
            kanan = kecepatan_cepat
        if kanan < kecepatan_lambat :
           kanan = kecepatan_lambat

        if sensor_ir_val == [1, 1, 1, 1, 1, 0] or sensor_ir_val == [1, 1, 1, 1, 0, 0]:
            kiri = 0
            kanan = kecepatan_cepat
        elif sensor_ir_val == [0, 1, 1, 1, 1, 1] or sensor_ir_val == [0, 0, 1, 1, 1, 1]:
            kiri = kecepatan_cepat
            kanan = 0

        

        #if sensor_ir_val == [0, 0, 1, 1, 0, 0] :
            #motor[0].setVelocity(0)
            #motor[1].setVelocity(0)
            
        #print("Odometry sensor values: {:.3f} {:.3f}".format(motor_pos_val[0], motor_pos_val[1]))

        #print("Proximity sensor values: {:.3f} {:.3f} {:.3f}".format(sensor_jarak_val[0], sensor_jarak_val[1], sensor_jarak_val[2]))

        #print("IR sensor values: {:.3f} {:.3f} {:.3f} {:.3f} {:.3f} {:.3f}".format(sensor_ir_val[0], sensor_ir_val[1], sensor_ir_val[2], sensor_ir_val[3], sensor_ir_val[4], sensor_ir_val[5]))
        
        print(kiri)
        print(kanan)

        #print(*sensor_ir_val)

        #print("PD Control: {}".format(pid_control))

        motor[0].setVelocity(kalkulasi_motor(kiri))
        motor[1].setVelocity(kalkulasi_motor(kanan))

if __name__ == "__main__":
    my_robot = Robot()
    run_robot(my_robot)