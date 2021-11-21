from controller import Robot

robot = Robot()

def kalkulasi_motor(signal):
    return (signal/100)*180

nilai_posisi = 0
sensor_ir_val = [0, 0, 0, 0, 0, 0, 0, 0]

timestep = int(robot.getBasicTimeStep())

# Deklarasi Motor
motor = []
motor_nama =['left wheel motor', 'right wheel motor']
for i in range(2):
    motor.append(robot.getDevice(motor_nama[i]))
    motor[i].setPosition(float('inf'))
    motor[i].setVelocity(0.0)

# Deklarasi Ground Sensor
sensor_ir = []
sensor_ir_nama = ['gs0', 'gs1', 'gs2', 'gs3', 'gs4', 'gs5', 'gs6', 'gs7']
for i in range(8):
    sensor_ir.append(robot.getDevice(sensor_ir_nama[i]))
    sensor_ir[i].enable(timestep)
    
# Konversi nilai Ir sensor ke biner
def read_ir():
    for i in range(8):
        if sensor_ir_val[i] < 300:
            sensor_ir_val[i] = 1
        else:
            sensor_ir_val[i] = 0

# Deklarasi posisi robot
def robot_pos():
    global nilai_posisi
    if sensor_ir_val == [1, 0, 0, 0, 0, 0, 0, 0] :
        nilai_posisi = 10
    elif sensor_ir_val == [1, 1, 1, 0, 0, 0] :
        nilai_posisi = 15
    elif sensor_ir_val == [1, 1, 0, 0, 0, 0] :
        nilai_posisi = 5
    elif sensor_ir_val == [0, 1, 0, 0, 0, 0] :
        nilai_posisi = 4
    elif sensor_ir_val == [0, 1, 1, 1, 0, 0] :
        nilai_posisi = 3
        #print("Belok Kiri")
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
        #print("Belok Kanan")
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

# Main Loop
while robot.step(timestep) != -1:
    for i in range(8):
        sensor_ir_val[i] = sensor_ir[i].getValue()

        
    read_ir()
    robot_pos()
                
    # Line follow
    kalkulasi_pid_line()
    kecepatan = kalkulasi_motor(0)
    kiri = kecepatan + pid_control_line
    kanan = kecepatan - pid_control_line

            
    print("IR sensor values: {} {} {} {} {} {} {} {}".format(sensor_ir_val[0], sensor_ir_val[1], sensor_ir_val[2], sensor_ir_val[3], sensor_ir_val[4], sensor_ir_val[5], sensor_ir_val[6], sensor_ir_val[7]))

    motor[0].setVelocity(kalkulasi_motor(0))
    motor[1].setVelocity(kalkulasi_motor(0))