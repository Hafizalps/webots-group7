from controller import Robot
import time

def kalkulasi_motor(signal):
    return (signal/100)*6.28

robot = Robot()
timestep = 64
#timestep = int(robot.getBasicTimeStep())
robot.step(timestep)
    
# Deklarasi Sensor Proximity
sensor_jarak = []
sensor_nama =['ps0', 'ps1', 'ps2', 'ps3', 'ps4', 'ps5', 'ps6', 'ps7']
for i in range(8):
    sensor_jarak.append(robot.getDevice(sensor_nama[i]))
    sensor_jarak[i].enable(timestep)
    
# Deklarasi Ground Sensor
sensor_ir = []
sensor_ir_nama = ['gs0', 'gs1', 'gs2']
for i in range(3):
    sensor_ir.append(robot.getDevice(sensor_ir_nama[i]))
    sensor_ir[i].enable(timestep)
    
# Deklarasi Motor
motor = []
motor_nama =['left wheel motor', 'right wheel motor']
for i in range(2):
    motor.append(robot.getDevice(motor_nama[i]))
    motor[i].setPosition(float('inf'))
    motor[i].setVelocity(0.0)

motor_sensor = []
motor_sensor_nama =['left wheel sensor', 'right wheel sensor']
for i in range(2):
    motor_sensor.append(robot.getDevice(motor_sensor_nama[i]))
    motor_sensor[i].enable(timestep)

# Konversi nilai Ir sensor ke biner
def read_ir():
    for i in range(3):
        if sensor_ir_val[i] > 750:
            sensor_ir_val[i] = 1
        else:
            sensor_ir_val[i] = 0

def Kiri():
    global kiri, kanan
    kiri = -6.28
    kanan = 6.28

def Kanan():
    global kiri, kanan
    kiri = 6.28
    kanan = -6.28

def Lurus():
    global kiri, kanan
    kiri = 6.28
    kanan = 6.28

def Kembali():
    global kiri, kanan
    kiri = 6.28
    kanan = -6.28


    
"""def Berhenti():
    global kiri, kanan
    kiri = 0
    kanan = 0
    time.sleep(1)"""       
# Deklarasi Parameter
#pid_parameter = [0.55, 0.00004, 2.6]
"""pid_parameter = [0.35, 0.00001, 2.2] #Kp Ki Kd
error = [0, 0, 0]
set_point = 140
control = [0, 0, 0]
pid_control = 0
sensorJarak_val = [0, 0, 0, 0, 0, 0, 0, 0]"""
    
sensor_ir_val = [0, 0, 0]
motor_sensor_val = [0, 0]
t = robot.getTime()

# Main loop:
while robot.step(timestep) != -1:
    for i in range(3):
        sensor_ir_val[i] = sensor_ir[i].getValue()
    read_ir()

    for i in range(2):
        motor_sensor_val[i] = motor_sensor[i].getValue()
        

    #algortima Line Follow
    if sensor_ir_val == [1, 1, 0]:
        Kiri()
        if (robot.getTime() - t) * 1000 > 10000:
            Kiri()
        else:
            Lurus()
    else:
            Lurus()

        
        
    """if sensor_ir_val == [1, 1, 1] :
            sensor_ir_val == [1, 1, 1]
    elif sensor_ir_val == [1, 1, 1] :
        Kiri()
        if sensor_ir_val == [0, 1, 1] :
            sensor_ir_val == [0, 1, 1]
    elif sensor_ir_val == [0, 1, 1] :
        Kiri()
        if sensor_ir_val == [0, 0, 1] :
            sensor_ir_val == [0, 0, 1]
    elif sensor_ir_val == [0, 0, 1] :
        Kanan()"""
    """if sensorBawah_val >= 11 and sensorBawah_val < 10000:
        sensorBawah_val = 0
        
    for i in range(8):
        sensorJarak_val[i] = sensor_jarak[i].getValue()
        
    # Kendali Proporsional (P)
    error[0] = set_point - sensorJarak_val[2]
    control[0] = error[0]*pid_parameter[0]
        
    # Kendali Integral (I)
    error[1] = error[1] + error[0]
    if error[1] > 150:
        error[1] = 150
    if error[1] <= -150:
        error[1] = -150
    control[1] = error[1]*pid_parameter[1]
        
    # Kendali Differensial (D)
    control[2] = (error[0]-error[2])*pid_parameter[2]   
    error[2] = error[0]
        
    pid_control = control[0]+control[1]+control[2]     
        
    if pid_control >= (kecepatan_cepat-kecepatan_normal-1):
        pid_control = (kecepatan_cepat-kecepatan_normal-1)
    if pid_control <= -(kecepatan_cepat-kecepatan_normal-1):
        pid_control = -(kecepatan_cepat-kecepatan_normal-1)
    
    kecepatan_maksimal = kalkulasi_motor(kecepatan_cepat)

    print("gs0={} ".format(sensorBawah_val))
    if sensorBawah_val >= 7:
        kiri = kalkulasi_motor(0)
        kanan = kalkulasi_motor(0)
    else:
    #cek dinding depan
        if sensorJarak_val[0] > 80:
            #print("Front")
            kiri = -kecepatan_maksimal
            kanan = kecepatan_maksimal
        else:
            # Error kecil = Jalan lurus. Robot booster
            if error[0] >= -5 and error[0] <= 5:
                kiri = kecepatan_maksimal
                kanan = kecepatan_maksimal
            else:
                if sensorJarak_val[2] > 80:
                    #print("Wall")
                    kecepatan = kalkulasi_motor(kecepatan_normal)
                    pid_control = kalkulasi_motor(pid_control)
                    kiri = kecepatan+pid_control
                    kanan = kecepatan-pid_control
                else:
                    #print("No Wall")
                    kiri = kecepatan_maksimal
                    kanan = kecepatan_maksimal/6"""
    #print("IR sensor values: {:.3f} {:.3f} {:.3f}".format(sensor_ir_val[0], sensor_ir_val[1], sensor_ir_val[2]))
    print("IR sensor values: {} {} {}".format(sensor_ir_val[0], sensor_ir_val[1], sensor_ir_val[2]))
    print(t)
    motor[0].setVelocity(kiri)
    motor[1].setVelocity(kanan)