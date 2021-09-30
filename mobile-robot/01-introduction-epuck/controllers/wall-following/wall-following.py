from controller import Robot

robot = Robot()
timestep = int(robot.getBasicTimeStep())

leftmotor = robot.getDevice('left wheel motor')
rightmotor = robot.getDevice('right wheel motor')
leftmotor.setPosition(float('inf'))
rightmotor.setPosition(float('inf'))

sensor_ps0 = robot.getDevice('ps0')
sensor_ps1 = robot.getDevice('ps1')
sensor_ps7 = robot.getDevice('ps7')
sensor_ps6 = robot.getDevice('ps6')

sensor_ps0.enable(timestep)
sensor_ps1.enable(timestep)
sensor_ps7.enable(timestep)
sensor_ps6.enable(timestep)

while robot.step(timestep) != -1:  
    value_ps0 = sensor_ps0.getValue()
    value_ps1 = sensor_ps1.getValue()
    value_ps7 = sensor_ps7.getValue()
    value_ps6 = sensor_ps6.getValue()
    
    #robot belok ke kiri karena di kanan ada objek
    if value_ps0 > 120 and value_ps1 > 120:
        rightmotor.setVelocity(4)
        leftmotor.setVelocity(-4)
    
    #robot belok ke kanan karena di kiri ada objek
    if value_ps7 > 120 and value_ps6 > 120:
        rightmotor.setVelocity(-4)
        leftmotor.setVelocity(4)
    
    #robot lurus karena tidak ada halangan
    else:
        rightmotor.setVelocity(3)
        leftmotor.setVelocity(3)