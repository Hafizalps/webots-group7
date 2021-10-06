from controller import Robot

def kalkulasi_motor(signal):
    return (signal/100)*30

nilai_posisi = 0
kecepatan_lambat = 15 #15
kecepatan_normal = 65 #65
kecepatan_cepat = 100
pid_control_line = 0

def run_robot(robot):
    global kecepatan_lambat, kecepatan_normal, kecepatan_cepat
    timestep = 8

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
            if sensor_ir_val[i] < 570:
                sensor_ir_val[i] = 1
            else:
                sensor_ir_val[i] = 0

    # Deklarasi posisi robot
    def robot_pos():
        global nilai_posisi
        if sensor_ir_val == [1, 0, 0, 0, 0, 0] :
            nilai_posisi = 10
        elif sensor_ir_val == [1, 1, 1, 0, 0, 0] :
            nilai_posisi = 15
        elif sensor_ir_val == [1, 1, 0, 0, 0, 0] :
            nilai_posisi = 5
        elif sensor_ir_val == [0, 1, 0, 0, 0, 0] :
            nilai_posisi = 4
        elif sensor_ir_val == [0, 1, 1, 1, 0, 0] :
            nilai_posisi = 3
            print("Belok Kiri")
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
            print("Belok Kanan")
        elif sensor_ir_val == [0, 0, 0, 0, 1, 0] :
            nilai_posisi = -4
        elif sensor_ir_val == [0, 0, 0, 0, 1, 1] :
            nilai_posisi = -5
        elif sensor_ir_val == [0, 0, 0, 1, 1, 1] :
            nilai_posisi = -15
        elif sensor_ir_val == [0, 0, 0, 0, 0, 1] :
            nilai_posisi = -10
        elif sensor_ir_val == [0, 0, 1, 1, 1, 1] : 
            nilai_posisi = -30
        elif sensor_ir_val == [0, 1, 1, 1, 1, 1] : 
            nilai_posisi = -35
            print("Belok Kanan Tajam")
        elif sensor_ir_val == [1, 1, 1, 1, 0, 0] : 
            nilai_posisi = 30
        elif sensor_ir_val == [1, 1, 1, 1, 1, 0] : 
            nilai_posisi = 35
            print("Belok Kiri Tajam")

    def kalkulasi_pid_line():
        global pid_control_line
        pid_parameter = [20, 0.2, 70] #Kp Ki Kd 43, 0.2, 145 ; 20, 0.2, 70 ; 10, 0.2, 80
        error = [0, 0, 0]
        control = [0, 0, 0]
        set_point = 0

        # Kendali Proporsional (P)
        error[0] = set_point - nilai_posisi
        control[0] = error[0] * pid_parameter[0]

        # Kendali Integral (I)
        error[1] = error[1] + error[0]
        if error[1] > 35:
            error[1] = 35
        if error[1] <= -35:
            error[1] = -35
        control[1] = error[1] * pid_parameter[1]

        # Kendali Differensial (D)
        control[2] = (error[0] - error[2]) * pid_parameter[2]
        error[2] = error[0]

        pid_control_line = control[0] + control[1] + control[2]
        pid_control_line = kalkulasi_motor(pid_control_line)

    # Parameter Untuk Kendali PD (Wall Follow)
    pd_parameter = [0.5, 0.1] #Kp Kd
    pd_control_wall = 0
    leftPrevWall = 0
    midPrevWall = 0
    rightPrevWall = 0
    distanceToWall = 990
    
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

        if sensor_jarak_val[0] < 990 or sensor_jarak_val[2] < 990 :
            print("Algoritma Wall Following")
            left_wall_error = sensor_jarak_val[0] - 780
            right_wall_error = sensor_jarak_val[2] - 780

            # Deteksi tembok kanan
            if sensor_jarak_val[2] < distanceToWall :
                pd_control_wall = pd_parameter[0] * right_wall_error + pd_parameter[1] * (right_wall_error - rightPrevWall)
                rightPrevWall = right_wall_error
                kiri = kecepatan_lambat + pd_control_wall
                kanan = kecepatan_lambat - pd_control_wall
                print("Telusur Tembok Kanan")
            # Deteksi tembok kiri
            elif sensor_jarak_val[0] < distanceToWall :
                pd_control_wall = pd_parameter[0] * left_wall_error + pd_parameter[1] * (left_wall_error - leftPrevWall)
                leftPrevWall = left_wall_error
                kiri = kecepatan_lambat - pd_control_wall
                kanan = kecepatan_lambat + pd_control_wall
                print("Telusur Tembok Kiri")
            # Robot diantara tembok kanan dan kiri (Tengah)
            elif sensor_jarak_val[0] < distanceToWall and sensor_jarak_val[2] < distanceToWall :
                pd_control_wall = pd_parameter[0] * (sensor_jarak_val[2] - sensor_jarak_val[0]) + pd_parameter[1] * (sensor_jarak_val[2] - sensor_jarak_val[0] - midPrevWall)
                midPrevWall = sensor_jarak_val[2] - sensor_jarak_val[0]
                kiri = kecepatan_lambat + pd_control_wall
                kanan = kecepatan_lambat - pd_control_wall
        else :
            # Line follow
            kalkulasi_pid_line()
            kecepatan = kalkulasi_motor(kecepatan_normal)
            kiri = kecepatan + pid_control_line
            kanan = kecepatan - pid_control_line
        
        # Pembatas Kecepatan            
        if kiri > kecepatan_cepat :
            kiri = kecepatan_cepat
        if kiri < kecepatan_lambat :
            kiri = kecepatan_lambat
        if kanan > kecepatan_cepat :
            kanan = kecepatan_cepat
        if kanan < kecepatan_lambat :
            kanan = kecepatan_lambat
            
        #print("Odometry sensor values: {:.3f} {:.3f}".format(motor_pos_val[0], motor_pos_val[1]))
        #print("Proximity sensor values: {:.3f} {:.3f} {:.3f}".format(sensor_jarak_val[0], sensor_jarak_val[1], sensor_jarak_val[2]))
        #print("IR sensor values: {:.3f} {:.3f} {:.3f} {:.3f} {:.3f} {:.3f}".format(sensor_ir_val[0], sensor_ir_val[1], sensor_ir_val[2], sensor_ir_val[3], sensor_ir_val[4], sensor_ir_val[5]))
        
        #print(kiri)
        #print(kanan)

        #print(*sensor_ir_val)
        #print("PID Control: {}".format(pid_control_line))

        motor[0].setVelocity(kalkulasi_motor(kiri))
        motor[1].setVelocity(kalkulasi_motor(kanan))

if __name__ == "__main__":
    my_robot = Robot()
    run_robot(my_robot)