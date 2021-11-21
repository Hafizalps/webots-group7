from controller import Supervisor

# create the Robot instance.
robot = Supervisor()

robotNode = robot.getSelf()

# get the time step of the current world.
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

# Deklarasi LEDs
leds = []
for i in range(5):
    led = robot.getDevice('led' + str(i))
    leds.append(led)
    led.set(1)

def Error_Position(Pos):
    online = 0
    PosX = 0
    for i in range(8):
        # Read Sensor and if > 200
        if sensor_ir[i].getValue() > 250:
            PosX += i
            online += 1

    # If outline sensor, return the latest position
    if online == 0:
        return Pos
    return PosX / online -3.5

def posLED():
    # Center green LED
    if P > -1 and P < 1:
        leds[1].set(1)
    else:
        leds[1].set(0)
        
    # left blue LED
    if P < -0.8:
        leds[0].set(1)
    else:
        leds[0].set(0)

    # Right blue LED
    if P > 0.8:
        leds[2].set(1)
    else:
        leds[2].set(0)

# Konversi nilai Ir sensor ke biner
def read_ir():
    for i in range(8):
        if sensor_ir_val[i] < 250:
            sensor_ir_val[i] = 1
        else:
            sensor_ir_val[i] = 0

sensor_ir_val = [0, 0, 0, 0, 0, 0, 0, 0]
Kp = 20
Ki = 0.03 
Kd = 1

P = 0
I = 0
D = 0

oldP = 0
maxS = 20
maxV = 0

# Main loop:
while robot.step(timestep) != -1:
    for i in range(8):
        sensor_ir_val[i] = sensor_ir[i].getValue()
    
    # Error Position Calculation & PID
    P = Error_Position(P)
    I += P * timestep / 1000
    D = D * 0.5 + (P - oldP) / timestep * 1000
    
    PID = Kp * P + Ki * I + Kd * D
    
    oldP = P
    
    medS = maxS - abs(PID)
    left_speed = medS - PID
    right_speed = medS + PID

    posLED()

    #print("IR sensor values: {} {} {} {} {} {} {} {}".format(sensor_ir_val[0], sensor_ir_val[1], sensor_ir_val[2], sensor_ir_val[3], sensor_ir_val[4], sensor_ir_val[5], sensor_ir_val[6], sensor_ir_val[7]))
    print(D)

    motor[0].setVelocity(-20)
    motor[1].setVelocity(20)