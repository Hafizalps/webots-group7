# Arena
![Arena](https://github.com/2black0/webots-laboratory/blob/6821642d7449c7fbabcc026cab05f311bd9c2706/competition/line-arena/img/full-arena.png)

# List Component of Robot
## Sensor
![Inertial Unit ](https://github.com/2black0/webots-laboratory/blob/6821642d7449c7fbabcc026cab05f311bd9c2706/competition/line-arena/img/inertial-unit.png)
1. Inertial Unit 
```
"inertial unit"
```
![Distance Sensor](https://github.com/2black0/webots-laboratory/blob/e5155ba50e504a4b3106b1218210571894c044d8/competition/line-arena/img/sensor.png)
2. Line Sensor
```
"IRL2"
"IRL1"
"IRCL"
"IRCR"
"IRR1"
"IRR2"
```
3. Distance Sensor
```
"ds_right"
"ds_left"
"ds_front"
"ds_right_maze"
```
4. Wheel position
```
"ps_1"
"ps_2"
```
![Camera](https://github.com/2black0/webots-laboratory/blob/6821642d7449c7fbabcc026cab05f311bd9c2706/competition/line-arena/img/camera.png)
5. Camera
```
"CAM"
```

## Actuator
1. Wheel
```
"motor_1"
"motor_2"
```
2. Arm
```
"r_motor"
"l_motor"
"base_motor"
"arm1_motor"
"arm2_motor"
```
# Rule
1. robot start dari tempat start
2. pada arena yagn tidak terdapat jalur harus mengikuti dinding 
3. terdapat kubus berwarna merah-hijau-biru pada arena yang tidak boleh ditabrak
4. tidak boleh terbalik
5. robot boleh finish pada finish 1 atau 2