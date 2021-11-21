from controller import Robot

robot = Robot()

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

def Maju():
    global kiri, kanan
    kiri = 25
    kanan = 25
def Kiri():
    global kiri, kanan
    kiri = 5
    kanan = 25
def Kanan():
    global kiri, kanan
    kiri = 25
    kanan = 5
def Berhenti():
    global kiri, kanan
    kiri = 0
    kanan = 0




kiri = 0
kanan = 0
sensor_ir_val = [0, 0, 0, 0, 0, 0, 0, 0]

# Main loop:
while robot.step(timestep) != -1:
    for i in range(8):
        sensor_ir_val[i] = sensor_ir[i].getValue()

    garis_kiri  = sensor_ir_val[0] + sensor_ir_val[1] + sensor_ir_val[2] + sensor_ir_val[3] + sensor_ir_val[4] # 1100 | 3050
    garis_kiri2 = sensor_ir_val[0] + sensor_ir_val[1] + sensor_ir_val[2] + sensor_ir_val[3] # 880 | 2440
    garis_kiri3 = sensor_ir_val[0] + sensor_ir_val[1] + sensor_ir_val[2] # 660 | 1830
    garis_kiri4 = sensor_ir_val[0] + sensor_ir_val[1] # 440 | 1220

    garis_kanan = sensor_ir_val[7] + sensor_ir_val[6] + sensor_ir_val[5] + sensor_ir_val[4] + sensor_ir_val[3] # 1100 | 3050
    garis_kanan2 = sensor_ir_val[7] + sensor_ir_val[6] + sensor_ir_val[5] + sensor_ir_val[4] # 880 | 2440
    garis_kanan3 = sensor_ir_val[7] + sensor_ir_val[6] + sensor_ir_val[5]  # 660 | 1830
    garis_kanan4 = sensor_ir_val[7] + sensor_ir_val[6] # 440 | 1220

    garis_tengah = sensor_ir_val[3] + sensor_ir_val[4] # 440

    garis = garis_kiri2 + garis_kanan2 

    if (garis_kiri < 1300 or garis_kiri2 < 1080 or garis_kiri3 < 860 or garis_kiri4 < 640) and (garis_kanan3 > 1830 or garis_kanan2 > 2440 or garis_kanan > 3050 or garis_kanan4 > 1220) :
        Kiri()
    elif (garis_kanan < 1300 or garis_kanan2 < 1080 or garis_kanan3 < 860 or garis_kanan4 < 640) and (garis_kiri3 > 1830 or garis_kiri2 > 2440 or garis_kiri > 3050 or garis_kiri4 > 1220) :
        Kanan()
    elif garis > 4880 :
        Maju()
    elif (garis_tengah < 440) :
        Maju()
        

    print("IR sensor values: {:.2f} {:.2f} {:.2f} {:.2f} {:.2f} {:.2f} {:.2f} {:.2f}".format(sensor_ir_val[0], sensor_ir_val[1], sensor_ir_val[2], sensor_ir_val[3], sensor_ir_val[4], sensor_ir_val[5], sensor_ir_val[6], sensor_ir_val[7]))


    motor[0].setVelocity(kiri)
    motor[1].setVelocity(kanan)


