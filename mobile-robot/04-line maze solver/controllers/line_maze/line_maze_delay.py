from controller import Supervisor
supervisor = Supervisor
def waktu():
    global t
    t = supervisor.getTime()
    print(t)