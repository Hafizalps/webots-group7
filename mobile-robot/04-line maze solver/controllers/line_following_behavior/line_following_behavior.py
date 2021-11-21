"""line_following_behavior controller."""

# This code was tested on Webots R2020b, revision 1, on Windows 10 running
# Python 3.7.7 64-bit

from controller import Robot, DistanceSensor, Motor
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl

# Initialize variables
TIME_STEP = 64
MAX_SPEED = 6.28
speed = 1 * MAX_SPEED

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())   # [ms]

# Inisialisasi ir sensor
gs = []
gsNames = ['gs0', 'gs1', 'gs2']
for i in range(3):
    gs.append(robot.getDistanceSensor(gsNames[i]))
    gs[i].enable(timestep)

# Initialize motors    
leftMotor = robot.getMotor('left wheel motor')
rightMotor = robot.getMotor('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0.0)
rightMotor.setVelocity(0.0)

# Inisialisasi Himpunan Fuzzy
garisKiri = ctrl.Antecedent(np.arange(0, 801, 1), 'garisKiri')
garisTengah = ctrl.Antecedent(np.arange(0, 801, 1), 'garisTengah')
garisKanan = ctrl.Antecedent(np.arange(0, 801, 1), 'garisKanan')


motorKiri = ctrl.Consequent(np.arange(0, 61, 1), 'motorKiri')
motorKanan = ctrl.Consequent(np.arange(0, 61, 1), 'motorKanan')


garisKiri['Sedikit'] = fuzz.trapmf(garisKiri.universe, [0,0,200,600])
garisKiri['Banyak'] = fuzz.trapmf(garisKiri.universe, [200,600,800,800])

garisTengah['Sedikit'] = fuzz.trapmf(garisTengah.universe, [0,0,200,600])
garisTengah['Banyak'] = fuzz.trapmf(garisTengah.universe, [200,600,800,800])

garisKanan['Sedikit'] = fuzz.trapmf(garisKanan.universe, [0,0,200,600])
garisKanan['Banyak'] = fuzz.trapmf(garisKanan.universe, [200,600,800,800])

motorKiri['Lambat'] = fuzz.trapmf(motorKiri.universe, [6,12,24,30])
motorKiri['Sedang'] = fuzz.trapmf(motorKiri.universe, [24,30,42,48])
motorKiri['Cepat'] = fuzz.trapmf(motorKiri.universe, [42,48,54,60])

motorKanan['Lambat'] = fuzz.trapmf(motorKanan.universe, [6,12,24,30])
motorKanan['Sedang'] = fuzz.trapmf(motorKanan.universe, [24,30,42,48])
motorKanan['Cepat'] = fuzz.trapmf(motorKanan.universe, [42,48,54,60])

#Rule Base
rule1a = ctrl.Rule(garisKanan['Sedikit'] & garisKiri['Sedikit'] & garisTengah['Sedikit'], motorKiri['Sedang']) #Lurus
rule2a = ctrl.Rule(garisKanan['Sedikit'] & garisKiri['Sedikit'] & garisTengah['Banyak'], motorKiri['Sedang'])  #Lurus
rule3a = ctrl.Rule(garisKanan['Sedikit'] & garisKiri['Banyak'] & garisTengah['Sedikit'], motorKiri['Lambat'])  #Belok Kiri
rule4a = ctrl.Rule(garisKanan['Sedikit'] & garisKiri['Banyak'] & garisTengah['Banyak'], motorKiri['Lambat'])  #Belok Kiri
rule5a = ctrl.Rule(garisKanan['Banyak'] & garisKiri['Sedikit'] & garisTengah['Sedikit'], motorKiri['Cepat']) #Belok Kanan
rule6a = ctrl.Rule(garisKanan['Banyak'] & garisKiri['Sedikit'] & garisTengah['Banyak'], motorKiri['Cepat'])  #Belok Kanan
rule7a = ctrl.Rule(garisKanan['Banyak'] & garisKiri['Banyak'] & garisTengah['Sedikit'], motorKiri['Sedang'])  #Lurus
rule8a = ctrl.Rule(garisKanan['Banyak'] & garisKiri['Banyak'] & garisTengah['Banyak'], motorKiri['Sedang'])  #Lurus

rule1b = ctrl.Rule(garisKanan['Sedikit'] & garisKiri['Sedikit'] & garisTengah['Sedikit'], motorKanan['Sedang']) #Lurus
rule2b = ctrl.Rule(garisKanan['Sedikit'] & garisKiri['Sedikit'] & garisTengah['Banyak'], motorKanan['Sedang'])  #Lurus
rule3b = ctrl.Rule(garisKanan['Sedikit'] & garisKiri['Banyak'] & garisTengah['Sedikit'], motorKanan['Cepat'])  #Belok Kiri
rule4b = ctrl.Rule(garisKanan['Sedikit'] & garisKiri['Banyak'] & garisTengah['Banyak'], motorKanan['Cepat'])  #Belok Kiri
rule5b = ctrl.Rule(garisKanan['Banyak'] & garisKiri['Sedikit'] & garisTengah['Sedikit'], motorKanan['Lambat']) #Belok Kanan
rule6b = ctrl.Rule(garisKanan['Banyak'] & garisKiri['Sedikit'] & garisTengah['Banyak'], motorKanan['Lambat'])  #Belok Kanan
rule7b = ctrl.Rule(garisKanan['Banyak'] & garisKiri['Banyak'] & garisTengah['Sedikit'], motorKanan['Sedang'])  #Lurus
rule8b = ctrl.Rule(garisKanan['Banyak'] & garisKiri['Banyak'] & garisTengah['Banyak'], motorKanan['Sedang'])  #Lurus


# Main loop:
while robot.step(timestep) != -1:
    # Update sensor readings
    gsValues = []
    for i in range(3):
        gsValues.append(gs[i].getValue())

    motor_control = ctrl.ControlSystem([rule1a, rule2a, rule3a, rule4a, rule5a, rule6a, rule7a, rule8a])
    motor = ctrl.ControlSystemSimulation(motor_control)
    motor.input['garisKiri'] = gsValues[2] # 0 - 800
    motor.input['garisTengah'] = gsValues[1] # 0 - 800
    motor.input['garisKanan'] = gsValues[0] # 0 - 800
    motor.compute() # 0 - 60

    motor_control_2 = ctrl.ControlSystem([rule1b, rule2b, rule3b, rule4b, rule5b, rule6b, rule7b, rule8b])
    motor_2 = ctrl.ControlSystemSimulation(motor_control_2)
    motor_2.input['garisKiri'] = gsValues[2] # 0 - 800
    motor_2.input['garisTengah'] = gsValues[1] # 0 - 800
    motor_2.input['garisKanan'] = gsValues[0] # 0 - 800
    motor_2.compute() # 0 - 60
    
    speedKiri = motor.output['motorKiri']
    speedKanan = motor_2.output['motorKanan']
    print('Kecepatan Motor Kiri=',speedKiri,'RPM')
    print('Kecepatan Motor Kanan=',speedKanan,'RPM')
    
    leftMotor.setVelocity((speedKiri*speed)/60)
    rightMotor.setVelocity((speedKanan*speed)/60)
    
    # if the left ground line sensor detect black line 
    if gsValues[2] > 600:  
        print('Belok Kiri', gsValues[0], gsValues[1], gsValues[2])
    
    # if the left ground line sensor detect black line
    elif gsValues[0] > 600:     
        print('Belok Kanan', gsValues[0], gsValues[1], gsValues[2])
    
    # if the left ground line sensor detect black line
    elif gsValues[1] > 600:  
        print('Lurus', gsValues[0], gsValues[1], gsValues[2])        
    
    else:
        print('Lurus', gsValues[0], gsValues[1], gsValues[2])
    
