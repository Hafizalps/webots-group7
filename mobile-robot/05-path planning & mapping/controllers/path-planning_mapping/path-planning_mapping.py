from controller import Robot, Camera, CameraRecognitionObject, InertialUnit, DistanceSensor, PositionSensor
import math
import random
from collections import deque

robot = Robot()

timestep = int(robot.getBasicTimeStep())

# Deklarasi Sensor Jarak/Ultrasonik
fds = robot.getDevice('front_ds')
lds = robot.getDevice('left_ds')
rds = robot.getDevice('right_ds')
fds.enable(timestep)
lds.enable(timestep)
rds.enable(timestep)

# Deklarasi Sensor Odometry
lps = robot.getDevice('left wheel sensor')
rps = robot.getDevice('right wheel sensor')
lps.enable(timestep)
rps.enable(timestep)

# Deklarasi Camera & Recognition 
camera = robot.getDevice('camera1')
camera.enable(timestep)
camera.recognitionEnable(timestep)

# Deklarasi IMU Sensor (Accelerometer dan Gyroscope)
imu = robot.getDevice('inertial unit')
imu.enable(timestep)

# Deklarasi Motor
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftMotor.setVelocity(0)
rightMotor.setVelocity(0)

# Parameter-parameter yang dibutuhkan Robot
wheel_radius = 1.6 / 2.0
wheel_circ = 2 * 3.14 * wheel_radius
enc_unit = wheel_circ / 6.28
max_speed = 4
# Jarak antar roda dalam satuan inch dan titik tengah robot (d_mid)
d = 2.28
d_mid = d / 2.0

# Nilai pada arena (baris, kolom)
# n = indeks + 1 dari kombinasi yang terkait baris dan kolom dibawah ini.
# Contohnya, (0,0) adalah baris=0, kolom=0, Sehingga "n" merupakan indeks dari list, yaitu 0. Jadi ditambah 1, n=0+1=1
n_rc = [(0,0), (0,1), (0,2), (0,3),
        (1,0), (1,1), (1,2), (1,3),
        (2,0), (2,1), (2,2), (2,3),
        (3,0), (3,1), (3,2), (3,3)]

# Kondisi awal robot.
dir = "North"
# Variabel yang berfungsi untuk melacak posisi x,y yang baru ketika robot terus bergerak.
new_pos = [15.0, -15.0]
# Variabel yang berfungsi untuk melacak nilai terakhir dari sensor posisi (IMU) pada robot, sehingga perubahan posisinya dapat disimpan pada variabel ini. 
last_vals = [0, 0]
# Deklarasi posisi x,y robot, posisi awal robot berada di grid sel ke-berapa (n), dan orientasi theta (q)
robot_pose = [15.0, -15.0, 16, 180]

# Konfigurasi mapping arena labirin. 
# Jika salah satu West, North, East, South (WNES) bernilai 1, maka pada sel/grid dan arah tersebut terdapat dinding.
grid_maze = [[0, 1, 1, 0, 0], [0, 0, 1, 0, 1], [0, 0, 1, 1, 0], [0, 1, 1, 1, 0],
             [0, 1, 0, 0, 1], [0, 0, 1, 1, 1], [0, 1, 0, 0, 0], [0, 0, 0, 1, 1], 
             [0, 1, 1, 0, 0], [0, 0, 1, 0, 1], [0, 0, 0, 1, 0], [0, 1, 1, 1, 0],
             [0, 1, 0, 1, 1], [0, 1, 1, 0, 1], [0, 0, 0, 0, 1], [0, 0, 0, 1, 1]]

# List/array yang berfungsi untuk menyimpan/melacak semua sel yang tidak dikunjungi (.) dan yang sudah dikunjungi (X).
visited_cells = ['.', '.', '.', '.',
                 '.', '.', '.', '.',
                 '.', '.', '.', '.',
                 '.', '.', '.', 'X']  
                 
# Jumlah sel/grid yang ada di arena. Jika arena 4 x 4 maka jumlah sel = 16.
num_cells = 16

# Deklarasi tujuan akhir robot. Dengan format [Sel, Menghadap ke arah (West, North, East, South)]
goal_pose = [8, "West"]        
                 
# Fungsi untuk mengkonversi satuan meter ke inchi.
def m_to_i(meters):
    return meters * 39.3701

def pos_s_to_inches(val):
    return math.fabs(val * wheel_radius)
        
# Fungsi untuk menampilkan nilai sensor odometry pada roda kiri dan kanan, serta nilai Yaw Axis pada imu sensor
def print_measurements():
    print(f"left: {pos_s_to_inches(lps.getValue())}, right: {pos_s_to_inches(rps.getValue())}, imu: {(imu.getRollPitchYaw()[2] * 180) / 3.14159}")
    
def get_p_sensors_vals():
    # Konversi nilai sensor odometry pada roda kiri dan kanan ke satuan inchi.
    return pos_s_to_inches(lps.getValue()), pos_s_to_inches(rps.getValue())

def get_d_sensors_vals():
    # Konversi nilai sensor jarak/ultrasonik ke satuan inchi
    return m_to_i(lds.getValue()), m_to_i(fds.getValue()), m_to_i(rds.getValue())

def get_time(distance, speed):
    # Rumus untuk mengetahui waktu. Waktu = Jarak/Kecepatan
    return distance / speed

# Fungsi untuk robot bergerak (Motor berputar/berhenti)
def move(inches, timestep):
    seconds = get_time(inches, max_speed)
    end_time = seconds + robot.getTime()
    while robot.step(timestep) != -1:
        update_robot()
        if robot.getTime() < end_time:
            leftMotor.setVelocity(max_speed/wheel_radius)
            rightMotor.setVelocity(max_speed/wheel_radius)
        else:
            stop_motors()
            break

# Fungsi untuk mendapatkan kecepatan robot
def get_rot_speed_rad(degrees, seconds):
    circle = d_mid * 2 * math.pi
    dist = (degrees / 360) * circle
    linear_vel = dist / seconds
    left_wheel_speed = linear_vel / wheel_radius
    right_wheel_speed = -1 * linear_vel / wheel_radius
    return left_wheel_speed, right_wheel_speed

# Fungsi Belok pada Robot  
def rotate(degrees, seconds, timestep, direction):
    global last_vals
    # Mendapatkan nilai kecepatan rotasi kiri dan kanan untuk berputar sebesar "x" derajat dalam "y" detik.
    left, right = get_rot_speed_rad(degrees, seconds)
    end_time = seconds + robot.getTime()
    while robot.step(timestep) != -1:
        # update and print the robot's details
        update_robot(rotating=True)
        # still update the last vals
        vals = get_p_sensors_vals()
        last_vals = vals
        print(f"Rotating {direction}...")
        if robot.getTime() < end_time:
            leftMotor.setVelocity(left)
            rightMotor.setVelocity(right)
        else:
            stop_motors()
            break    

# Fungsi Robot Berhenti
def stop_motors():
    leftMotor.setVelocity(0)
    rightMotor.setVelocity(0)
    print("Motors stopped.")

# Fungsi Belok Kiri            
def turn_left(ts, degrees=-90.5):
    rotate(degrees, 1.5, ts, "left")           
            
# Fungsi Belok Kanan            
def turn_right(ts, degrees=90.5):
    rotate(degrees, 1.5, ts, "right")
    
# Fungsi untuk Mengecek semua sel pada arena, apakah terdapat nilai 0 atau tidak, dan apabila terdapat nilai 0, Sel yang belum dikunjungi tersebut masuk ke dalam daftar path planning atau tidak.
def check_if_robot_should_stop():
    for el in grid_maze:
        # Walaupun jika hanya ada satu nilai 0 dalam sel yg belum dikunjungi, maka robot akan terus berjalan.
        if el[0] == 0:
            return False
    # Jika pada sel yang terdaftar pada path planning semua selnya sudah bernilai 1/sudah dikunjungi maka robot harus berhenti di sel tersebut.
    return True      

def update_robot(rotating=False):
    # Berikut format parameter posisi dan gerak robot: (x,y,n,q), Dimana “x,y” merupakan posisi robot dalam koordinat, “n” merupakan posisi robot pada sel grid ke-berapa, and “q” merupakan orientasi theta.
    # Contohnya. robot_pose = (11.5, 2.3, 8, 1.1).
    global robot_pose
    # Menampilkan waktu
    print(80*"-")
    print(f"Time: {robot.getTime()}")
    # Mendapatkan nilai terbaru dari sensor odometry.
    vals = get_p_sensors_vals()
    
    if not rotating:
        x, y = get_robot_x_y(vals)
        n = get_current_grid_cell(x, y)
    else:
        x = robot_pose[0]
        y = robot_pose[1]
        n = robot_pose[2]
    q = (imu.getRollPitchYaw()[2] * 180) / 3.14159
        
    # Variabel aksi pada robot
    robot_pose = [x, y, n, q]
        
    # Menampilkan aksi yang terbaru (update) pada robot
    print(f"Pose: {robot_pose}")
        
    # Menampilkan posisi dan arah robot menghadap yang terbaru (update)
    update_direction()
    print(f"Robot current grid cell: {robot_pose[2]}, Direction: {dir}")
        
    # Memanggil fungsi menampilkan mapping sel arena yang telah dikunjungi oleh robot berupa tanda "." dan "X"
    print_visited_cells()
        
    # Memanggil fungsi hasil mapping arena labirin yang terbaru
    print_maze(grid_maze)

# Fungsi untuk mendapatkan nilai posisi robot dalam koordinat.
def get_robot_x_y(vals):
    global new_pos
    global last_vals
    diff = [vals[0] - last_vals[0], vals[1] - last_vals[1]]
    for i in range(len(diff)):
        diff[i] = math.fabs(diff[i])
    
    if math.fabs(diff[0]) >= .3 or math.fabs(diff[1]) >= .3:
            diff[0] = 0.3
            diff[1] = 0.3
            
    # Perbedaan rata-rata dari roda kiri dan kanan
    diff_avg = (diff[0]+ diff[1]) / 2.0
    
    # Nilai posisi robot dalam koordinat (x dan y) bergantung pada arah robot menghadap.
    if dir == "North":
        x = new_pos[0]
        y = new_pos[1] + diff_avg
    elif dir == "West":
        x = new_pos[0] - diff_avg
        y = new_pos[1] 
    elif dir == "East":
        x = new_pos[0] + diff_avg
        y = new_pos[1] 
    elif dir == "South":
        x = new_pos[0] 
        y = new_pos[1] - diff_avg
    # Variabel untuk menyimpann nilai koordinat robot yang sebelumnya
    last_vals = vals
    # Menyimpan nilai koordinat robot yang terbaru ke variabel new_pos
    new_pos = x, y
    return x, y


def get_current_grid_cell(x, y):
    n = 0
    row = 0
    col = 0
    # Cara menentukan baris pada arena.
    if y >= -20 and y < -10:
        # Baris paling bawah
        row = 3
    elif y >= -10 and y < 0:
        # Baris di atas baris paling bawah
        row = 2
    elif y >= 0 and y < 10:
        # Baris di bawah baris paling atas
        row = 1 
    elif y >= 10 and y <= 20:
        # Baris paling atas
        row = 0 
        
    # Cara menentukan kolom pada arena.
    if x >= -20 and x <= -10:
        # Kolom Pertama/paling kiri
        col = 0
    elif x > -10 and x <= 0:
        # Kolom Kedua
        col = 1
    elif x > 0 and x <= 10:
        # Kolom Ketiga
        col = 2 
    elif x > 10 and x <= 20:
        # Kolom Keempat/paling kanan
        col = 3 
        
    # Cara menentukan indeks pada setiap sel berdasarkan kombinasi baris dan kolomg yang telah didapatkan pada step sebelumnya
    for i in range(len(n_rc)):
        if n_rc[i][0] == row and n_rc[i][1] == col:
            n = i + 1
            break
    return n
    
# Fungsi untuk update arah robot
def update_direction():
    global dir
    dir = get_direction(robot_pose[3]) 

# Fungsi untuk menentukan dimana arah robot (dalam arah mata angin) berdasarkan nilai dari IMU sensor
def get_direction(imu_val):
    if (imu_val <= -135 and imu_val >= -180) or (135 <= imu_val <= 180):
        dir = "North"
    elif imu_val <= -45 and imu_val > -135:
        dir = "West"
    elif 45 <= imu_val <= 135:
        dir = "East"
    elif (-45 < imu_val <= 0) or (0 <= imu_val < 45):
        dir = "South"
    return dir 


def get_available_turns(walls):
    # Nilai 0 pada walls[i] berarti bahwa tidak ada dinding pada arah tersebut 0. Informasi tersebut disimpan pada "available" yang dituliskan secara berurutan yaitu [Kiri, Depan, Kanan]. Pada variabel "available", 0 berarti arah tersebut tidak boleh dilewati (ada dinding), sedangkan 1 berarti arah tersebut boleh dilewati (tidak ada dinding).
    # Variabel "count" merupakan jumlah robot akan berputar arah
    count = 0
    available = [0, 0, 0]
    for i in range(len(walls)):
        if walls[i] == 0:
            available[i] = 1
            count += 1
    return available, count

# Fungsi untuk menampilkan grid sel yang telah dikunjungi, dengan tampilan tanda "." dan "X"
def print_visited_cells():
    pr_vc = ""
    for i in range(len(visited_cells)):
        if i % 4 == 0 and i != 0:
            pr_vc += "\n"
        pr_vc += visited_cells[i]
    print("Visited cells:")
    print(pr_vc)

# Fungsi untuk menambahkan tanda/marking "X" sebagai grid sel yang telah dikunjungi
def mark_cell_visited(cell):
    global grid_maze
    global visited_cells
    grid_maze[cell-1][0] = 1
    visited_cells[cell-1] = 'X'


def face_north(ts):
    global dir
    while robot.step(ts) != -1:
        if dir == "West":
            # belok kanan menghadap utara/north lagi
            turn_right(ts)
        elif dir == "South":
            # berputar/belok kiri 2 kali untuk menghadap utara/north lagi
            turn_left(ts)
            turn_left(ts)
        elif dir == "East":
            # belok kiri menghadap utara/north lagi
            turn_left(ts)
        if dir == "North":
            break
    dir = "North"


def face_dir(ts, direction="North"):
    # Fungsi untuk mendapatkan nilai sudut gerak robot pada masing-masing arah mata angin
    if direction == "North":
        dir_angle = 180.0
    elif direction == "East":
        dir_angle = 90.0
    elif direction == "South":
        dir_angle = 0.0
    elif direction == "West":
        dir_angle = -90.0

    speed = 2
    tolerance = 0.1
    while robot.step(ts) != -1:
        q = (imu.getRollPitchYaw()[2] * 180) / 3.14159
        diff = dir_angle - q
        abs_diff = math.fabs(diff)
        print(80*"-")
        print(f"Time: {robot.getTime()}")
        print(f"q: {q}")
        print(f"diff: {diff}")
        print(f"abs_diff: {abs_diff}")
        print(f"Direction: {get_direction(q)}")
        print(f"Rotating until facing: {direction}..")
        print(f"speed: {speed}")
        
        # Kendali kecepatan roda, atau berbalik arah berdasarkan nilai abs_diff 
        if diff < 0 or (180 <= abs_diff <= 360):
            if abs_diff < 0.2:
                speed = -0.05
            elif abs_diff < 5:
                speed = -0.2
            else:
                speed = -2.0
        elif abs_diff < 0.2:
            speed = 0.05
        elif abs_diff < 5:
            speed = 0.2 
        elif abs_diff < 180:
            speed = 2.0
            
        # Jika nilai abs_diff telah terpenuhi, maka robot berhenti
        if abs_diff < tolerance:
            stop_motors()
            break
        else:
            leftMotor.setVelocity(-1 * speed)
            rightMotor.setVelocity(speed)   


# Fungsi untuk menampilkan hasil mapping arena labirin 4 x 4
def print_maze(maze):
    print("________________________________________")
    for i in range(4):
        x = i*4
        if (maze[x][0] == 0):
            v1 = "?"
        elif (maze[x][0] == 1):
            v1 = "V"
        else:
            v1 = str(maze[x][0])
            
        if (maze[x+1][0] == 0):
            v2 = "?"
        elif (maze[x+1][0] == 1):
            v2 = "V"
        else:
            v2 = str(maze[x+1][0])
            
        if (maze[x+2][0] == 0):
            v3 = "?"
        elif (maze[x+2][0] == 1):
            v3 = "V"
        else:
            v3 = str(maze[x+2][0])
            
        if (maze[x+3][0] == 0):
            v4 = "?"
        elif (maze[x+3][0] == 1):
            v4 = "V"
        else:
            v4 = str(maze[x+3][0])
         
        print("|  "+ str(maze[x][2]) +"\t  " +str(maze[x+1][2])+"\t  " +str(maze[x+2][2])
              +"\t  " +str(maze[x+3][2])+ "    |")
              
        print("|" +str(maze[x][1]) + " " +v1+" " + str(maze[x][3])+"\t" +str(maze[x+1][1])+ " " +v2+" " + str(maze[x+1][3])
              +"\t" +str(maze[x+2][1])+ " " +v3+" " + str(maze[x+2][3])
              +"\t" +str(maze[x+3][1]) + " " +v4+" " + str(maze[x+3][3]) +"  |")
              
        print("|  "+str(maze[x][4]) +"\t  " +str(maze[x+1][4])+"\t  " +str(maze[x+2][4])
              +"\t  " +str(maze[x+3][4])+"    |")
              
        if(i==3):
            print("|_______________________________________|\n")
        else:
            print("|                                       |")


def wave_front(ts, goal_cell):
    # Mendapatkan indeks dari sel yang pertama dan terakhir (tujuan)
    goal_cell_ind = goal_cell - 1
    start_cell_ind = robot_pose[2] - 1
    
    # Memberikan tanda ke sel yang terakhir (tujuan)
    grid_maze[goal_cell_ind][0] = 2
    
    # set the wave count to start at the next value from the goal
    count = grid_maze[goal_cell_ind][0] + 1
    
    print("Start wave front:")
    print_maze(grid_maze)
    
    while robot.step(ts) != -1:
        # Menampilkan waktu
        print(80*"-")
        print(f"Time: {robot.getTime()}")
        
        # Menampilkan wave count 
        print(f"Wave count: {count}")
        
        # get the cells to get the walls of 
        cells = []
        for i in range(num_cells):
            # if the cell has the previous wave count's value,
            # get that cell so we can add the current wave count to the grid_maze
            if grid_maze[i][0] == count - 1:
                cells.append(i)
        print(f"Cells: {cells}")
        
        # Mengambil nilai dinding pada setiap indeks sel dan kemudian menambahkan hasil wave count ke grid maze
        for c in cells:
            # get the walls surrounding the current cell
            walls = grid_maze[c]
            print(f"Cell: {c}, Walls: {walls}")
            
            if walls[1] == 0 and grid_maze[c-1][0] == 0:
                grid_maze[c-1][0] = count
            if walls[2] == 0 and grid_maze[c-4][0] == 0:
                grid_maze[c-4][0] = count
            if walls[3] == 0 and grid_maze[c+1][0] == 0:
                grid_maze[c+1][0] = count
            if walls[4] == 0 and grid_maze[c+4][0] == 0:
                grid_maze[c+4][0] = count
        
        # Menampilkan mapping labirin (grid maze) untuk melihat hasil perubahannya
        print_maze(grid_maze)
        
        # Jika sel yang pertama/awal sama dengan nilai wave count, maka perhitungan selesai
        if grid_maze[start_cell_ind][0] == count:
            break
        # Increment wave count
        count += 1


def plan_path(ts, goal_cell):
    # Robot menyusun untuk perencanaan jalur yang akan dilewati (path planning)
    plan = deque() 
    
    # Mendapatkan indeks dari sel yang pertama (mulai) dan terakhir (tujuan)
    goal_cell_ind = goal_cell - 1
    start_cell_ind = robot_pose[2] - 1
    
    # Nilai sel yang pertama
    start_val = grid_maze[start_cell_ind][0]
    
    # Nilai sel yang sekarang
    current_val = start_val - 1
    
    # the index of the cell currently being examined for its neighbors
    current_cell = start_cell_ind
    
    while robot.step(ts) != -1:
        # Pindah dari sel saat ini ke sel berikutnya
        mov = ""
        # Menampilkan waktu
        print(80*"-")
        print(f"Time: {robot.getTime()}")
        
        # Mendapatkan nilai keberadaan dinding yang mengelilingi sel saat ini, dengan format [V, W, N, E, S]
        walls = grid_maze[current_cell]
        
        # Periksa nilai sel saat ini untuk mendapatkan tindakan/aksi berikutnya (Maju, Kiri, Kanan, Putar balik)
        if current_cell > 3:
            if grid_maze[current_cell - 4][0] == current_val and walls[2] == 0:
                mov = "forward"
        if current_cell > 0 :
            if grid_maze[current_cell - 1][0] == current_val and walls[1] == 0:
                mov = "left"
        if current_cell < 15:
            if grid_maze[current_cell + 1][0] == current_val and walls[3] == 0:
                mov = "right"
        if current_cell < 11:
            if grid_maze[current_cell + 4][0] == current_val and walls[4] == 0:
                mov = "down"   
        
        # append the move to the stack
        plan.append((mov, current_val+1, current_cell))
        
        # Memperbarui nilai sel saat ini berdasarkan tindakan/aksi yang telah dilakukan
        if mov == "forward":
            current_cell -= 4
        elif mov == "left":
            current_cell -= 1
        elif mov == "right":
            current_cell += 1
        elif mov == "down":
            current_cell += 4
            
        # Menampilkan susunan dari path planning
        print(f"plan of moves: {plan}")
        
        # Memperbarui nilai current_val
        current_val -= 1
        
        # Jika di sel tujuan, makan susunan path planning akan berhenti.
        if current_cell == goal_cell_ind:
            break
    # Mengembalikan informasi susunan path planing
    return plan
    
# Melaksanakan path planing yang telah disusun, dari awal hingga akhir (Finish)    
def execute_plan(ts, plan):
    while robot.step(ts) != -1:
        # Menggerakkan robot ke arah Utara jika Belum
        if dir != "North":
            face_north(ts)
        
        # Menggunakan langkah paling kiri dalam plan deque untuk mendapatkan langkah saat ini yang harus dilakukan.
        mov = plan.popleft()
        # Menampilkan aksi/gerakannya dan arah robot
        #print(f"current move: {mov}")
        m = mov[0]
        
        # Jika robot mneghadap ke Utara
        if dir == "North":
            # Menggunakan gerakkan "m" untuk memutar robot
            if m == "left":
                turn_left(ts)
            elif m == "right":
                turn_right(ts)
            elif m == "down":
                turn_right(ts)
                turn_right(ts)
                
            # Setelah kondisi robot disesuaikan menurut gerakkan "m", pindah ke sel berikutnya sesuai dengan path planning yang telah disusun.
            move(10.0, ts)
            visited_cells[robot_pose[2]-1] = "X"
        
        # Jika robot sudah berada di sel tujuan (Akhir)
        if robot_pose[2] == goal_pose[0]:
            # Membuat robot menghadap ke Utara terlebih dahulu
            face_north(ts)
            # Membuat robot berputar sesuai dengan arah yang ada di variabel "goal_pose"
            if goal_pose[1] == "West":
                turn_left(ts)
            elif goal_pose[1] == "East":
                turn_right(ts)
            elif goal_pose[1] == "West":
                turn_right(ts)
                turn_right(ts)
                
            # Mengecek apakah arah robot sudah sesuai dengan arah yang ada di variabel "goal_pose"
            if dir == goal_pose[1]:
                break  
    return 0        

# Program utama robot
def main():
    if goal_pose[0] < 1:
        goal_pose[0] = 1
    elif goal_pose[0] > 16:
        goal_pose[0] = 16
    
    while robot.step(timestep) != -1:
        # Mengubah grid_maze sehingga dapat digunakan untuk membuat jalur/path
        wave_front(timestep, goal_pose[0])
        
        # Membuat path planning atau rencana berdasarkan grid_maze yang telah diubah
        plan = plan_path(timestep, goal_pose[0])
        
        # Mengeksekusi path planning
        val = execute_plan(timestep, plan)
        
        # Menampilkan sel yang telah dikunjungi
        print_visited_cells()
        
        # Jika nilai val=0 yang berarti bahwa robot sudah berada di kondisi tujuannya/finish, maka robot akan berhenti dan Sukses mencapai tujuan yang sudah diatur.
        if val == 0:
            print(80*"-")
            print(f"Time: {robot.getTime()}")
            print("GOAL")
            break       

if __name__ == "__main__":
    main()