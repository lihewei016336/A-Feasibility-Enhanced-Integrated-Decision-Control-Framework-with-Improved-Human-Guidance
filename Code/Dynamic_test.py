from DOF14_1 import Dynamic_step
import random 
import pandas as pd


DYS=Dynamic_step()

DYS.set_init_Velocity(5)
delta_f = 0
Tt = 0
Yaw = []
Vy = []
ay = []
del_f = []
Vx = []
ax = []
X_pos = []
Y_pos = []
beta = []
for i in range(1000):
    del_delta_f = random.uniform(-5,5)
    del_Tt = random.uniform(-50,50)
    delta_f = delta_f + del_delta_f
    Tt = Tt + del_Tt
    DYS.set_Steering_wheel_Angle(delta_f)
    DYS.set_Torque(Tt/4,Tt/4,Tt/4,Tt/4)
    result=DYS.step(start=0,end=0,step=0.01)
    del_f.append(delta_f)
    Vx.append(result["Vx"])
    Vy.append(result["Vy"])
    ax.append(result["ax"])
    ay.append(result["ay"])
    X_pos.append(result["X_position"])
    Y_pos.append(result["Y_position"])
    Yaw.append(result["Yaw"])
    beta.append(result["beta"])

data = {
    'Column1': del_f,
    'Column2': Vx,
    'Column3': Vy,
    'Column4': ax,
    'Column5': ay,
    'Column6': X_pos,
    'Column7': Y_pos,
    'Column8': Yaw,
    'Column9': beta,
}
df = pd.DataFrame(data)
excel_filename = 'data.xlsx'
df.to_excel(excel_filename, index=False) 

a = 1