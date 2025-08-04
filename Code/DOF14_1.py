import ctypes
import os 
path = os.path.dirname(__file__)
invoker = ctypes.WinDLL(path + "\\Dll_DOF14_V2.dll", winmode=0)

# 加载动态链接库


# 定义Vehicle类和State类
class Vehicle(ctypes.Structure):
    _fields_ = [
        ("vehicle_Mt", ctypes.c_double),
        ("vehicle_M", ctypes.c_double),
        ("vehicle_Jx", ctypes.c_double),
        ("vehicle_Jy", ctypes.c_double),
        ("vehicle_Jz", ctypes.c_double),
        ("vehicle_R", ctypes.c_double),
        ("vehicle_steerRatio", ctypes.c_double),
        ("vehicle_g", ctypes.c_double),
        ("vehicle_cb_tlf", ctypes.c_double),
        ("vehicle_cb_tlr", ctypes.c_double),
        ("vehicle_cb_trf", ctypes.c_double),
        ("vehicle_cb_trr", ctypes.c_double),
        ("vehicle_cb_slf", ctypes.c_double),
        ("vehicle_cb_slr", ctypes.c_double),
        ("vehicle_cb_srf", ctypes.c_double),
        ("vehicle_cb_srr", ctypes.c_double),
        ("vehicle_ck_slf", ctypes.c_double),
        ("vehicle_ck_slr", ctypes.c_double),
        ("vehicle_ck_srr", ctypes.c_double),
        ("vehicle_um", ctypes.c_double),
        ("vehicle_a", ctypes.c_double),
        ("vehicle_b", ctypes.c_double),
        ("vehicle_c", ctypes.c_double),
        ("vehicle_Lu1", ctypes.c_double),
        ("vehicle_Lu2", ctypes.c_double),
        ("vehicle_Lu3", ctypes.c_double),
        ("vehicle_Lu4", ctypes.c_double),
        ("vehicle_Lu5", ctypes.c_double),
        ("vehicle_Lu6", ctypes.c_double),
        ("vehicle_Lu7", ctypes.c_double),
        ("vehicle_Lu8", ctypes.c_double),
        ("vehicle_Lu9", ctypes.c_double),
        ("vehicle_Lu10", ctypes.c_double),
        ("vehicle_Lu11", ctypes.c_double),
        ("vehicle_Lu12", ctypes.c_double),
        ("vehicle_FH_lf", ctypes.c_double),
        ("vehicle_FH_rf", ctypes.c_double),
        ("vehicle_FH_lr", ctypes.c_double),
        ("vehicle_FH_rr", ctypes.c_double),
    ]

class State(ctypes.Structure):
    _fields_ = [
        ("vehicle_wheel_Angle", ctypes.c_double),
        ("vehicle_Velocity_0", ctypes.c_double),
        ("vehicle_Mu_fl", ctypes.c_double),
        ("vehicle_Mu_fr", ctypes.c_double),
        ("vehicle_Mu_rl", ctypes.c_double),
        ("vehicle_Mu_rr", ctypes.c_double),
        ("vehicle_Torque_of_FL", ctypes.c_double),
        ("vehicle_Torque_of_FR", ctypes.c_double),
        ("vehicle_Torque_of_RL", ctypes.c_double),
        ("vehicle_Torque_of_RR", ctypes.c_double),
        ("vehicle_V1", ctypes.c_double),
        ("vehicle_V2", ctypes.c_double),
        ("vehicle_V3", ctypes.c_double),
        ("vehicle_be", ctypes.c_double),
        ("vehicle_d_be", ctypes.c_double),
        ("vehicle_ax", ctypes.c_double),
        ("vehicle_Pits_lf", ctypes.c_double),
        ("vehicle_Pits_lr", ctypes.c_double),
        ("vehicle_Pits_rf", ctypes.c_double),
        ("vehicle_Pits_rr", ctypes.c_double),
        ("vehicle_Fu_lf", ctypes.c_double),
        ("vehicle_Fu_lr", ctypes.c_double),
        ("vehicle_Fu_rf", ctypes.c_double),
        ("vehicle_Fu_rr", ctypes.c_double),
        ("vehicle_Fua_lf", ctypes.c_double),
        ("vehicle_Fua_lr", ctypes.c_double),
        ("vehicle_Fua_rf", ctypes.c_double),
        ("vehicle_Fua_rr", ctypes.c_double),
        ("vehicle_ux_glf", ctypes.c_double),
        ("vehicle_ux_glr", ctypes.c_double),
        ("vehicle_ux_grf", ctypes.c_double),
        ("vehicle_ux_grr", ctypes.c_double),
        ("vehicle_vx_glf", ctypes.c_double),
        ("vehicle_vx_glr", ctypes.c_double),
        ("vehicle_vx_grf", ctypes.c_double),
        ("vehicle_vx_grr", ctypes.c_double),
        ("vehicle_phax_lf", ctypes.c_double),
        ("vehicle_phax_lr", ctypes.c_double),
        ("vehicle_phax_rf", ctypes.c_double),
        ("vehicle_phax_rr", ctypes.c_double),
        ("vehicle_Sub1_glf", ctypes.c_double),
        ("vehicle_Sub1_glr", ctypes.c_double),
        ("vehicle_Sub1_grf", ctypes.c_double),
        ("vehicle_Sub1_grr", ctypes.c_double),
        ("vehicle_Sub2_glf", ctypes.c_double),
        ("vehicle_Sub2_glr", ctypes.c_double),
        ("vehicle_Sub2_grf", ctypes.c_double),
        ("vehicle_Sub2_grr", ctypes.c_double),
        ("vehicle_Sub1_clf", ctypes.c_double),
        ("vehicle_Sub1_clr", ctypes.c_double),
        ("vehicle_Sub1_crf", ctypes.c_double),
        ("vehicle_Sub1_crr", ctypes.c_double),
        ("vehicle_Sub2_clf", ctypes.c_double),
        ("vehicle_Sub2_clr", ctypes.c_double),
        ("vehicle_Sub2_crf", ctypes.c_double),
        ("vehicle_Sub2_crr", ctypes.c_double),
        ("vehicle_Sub3_clf", ctypes.c_double),
        ("vehicle_Sub3_clr", ctypes.c_double),
        ("vehicle_Sub3_crf", ctypes.c_double),
        ("vehicle_Sub3_crr", ctypes.c_double),
        ("vehicle_Vlf", ctypes.c_double),
        ("vehicle_Vrf", ctypes.c_double),
        ("vehicle_Vlr", ctypes.c_double),
        ("vehicle_Vrr", ctypes.c_double),
        ("vehicle_du_ulf", ctypes.c_double),
        ("vehicle_du_urf", ctypes.c_double),
        ("vehicle_du_ulr", ctypes.c_double),
        ("vehicle_du_urr", ctypes.c_double),
        ("vehicle_u_clf", ctypes.c_double),
        ("vehicle_u_crf", ctypes.c_double),
        ("vehicle_u_clr", ctypes.c_double),
        ("vehicle_u_crr", ctypes.c_double),
        ("vehicle_x_tlf", ctypes.c_double),
        ("vehicle_x_trf", ctypes.c_double),
        ("vehicle_x_tlr", ctypes.c_double),
        ("vehicle_x_trr", ctypes.c_double),
        ("vehicle_out_xv_0", ctypes.c_double),
        ("vehicle_out_xv_1", ctypes.c_double),
        ("vehicle_out_xv_2", ctypes.c_double),
        ("vehicle_out_xv_3", ctypes.c_double),
        ("vehicle_out_xv_4", ctypes.c_double),
        ("vehicle_out_xv_5", ctypes.c_double),
        ("vehicle_out_xv_6", ctypes.c_double),
        ("vehicle_out_xv_7", ctypes.c_double),
        ("vehicle_dx_tlf", ctypes.c_double),
        ("vehicle_dx_trf", ctypes.c_double),
        ("vehicle_dx_tlr", ctypes.c_double),
        ("vehicle_dx_trr", ctypes.c_double),
        ("vehicle_rdp_lf", ctypes.c_double),
        ("vehicle_rdp_rf", ctypes.c_double),
        ("vehicle_rdp_lr", ctypes.c_double),
        ("vehicle_rdp_rr", ctypes.c_double),
        ("vehicle_ldp_slf", ctypes.c_double),
        ("vehicle_ldp_srf", ctypes.c_double),
        ("vehicle_ldp_slr", ctypes.c_double),
        ("vehicle_ldp_srr", ctypes.c_double),
        ("vehicle_udp_ulf", ctypes.c_double),
        ("vehicle_udp_urf", ctypes.c_double),
        ("vehicle_udp_ulr", ctypes.c_double),
        ("vehicle_udp_urr", ctypes.c_double),
        ("vehicle_dx_slf", ctypes.c_double),
        ("vehicle_dx_srf", ctypes.c_double),
        ("vehicle_dx_slr", ctypes.c_double),
        ("vehicle_dx_srr", ctypes.c_double),
        ("vehicle_x_slf", ctypes.c_double),
        ("vehicle_x_srf", ctypes.c_double),
        ("vehicle_x_slr", ctypes.c_double),
        ("vehicle_x_srr", ctypes.c_double),
        ("vehicle_Sub3_lf", ctypes.c_double),
        ("vehicle_Sub3_rf", ctypes.c_double),
        ("vehicle_Sub3_lr", ctypes.c_double),
        ("vehicle_Sub3_rr", ctypes.c_double),
        ("vehicle_Sub1_lf", ctypes.c_double),
        ("vehicle_Sub1_rf", ctypes.c_double),
        ("vehicle_Sub1_lr", ctypes.c_double),
        ("vehicle_Sub1_rr", ctypes.c_double),
        ("vehicle_Sub2_lf", ctypes.c_double),
        ("vehicle_Sub2_rf", ctypes.c_double),
        ("vehicle_Sub2_lr", ctypes.c_double),
        ("vehicle_Sub2_rr", ctypes.c_double),
        ("vehicle_F1", ctypes.c_double),
        ("vehicle_F2", ctypes.c_double),
        ("vehicle_Vel_1", ctypes.c_double),
        ("vehicle_Vel_2", ctypes.c_double),
        ("vehicle_Acc_0", ctypes.c_double),
        ("vehicle_Acc_1", ctypes.c_double),
        ("vehicle_Acc_2", ctypes.c_double),
        ("vehicle_Rot_Vel_0", ctypes.c_double),
        ("vehicle_Rot_Vel_1", ctypes.c_double),
        ("vehicle_Rot_Vel_2", ctypes.c_double),
        ("vehicle_Rot_Acc_0", ctypes.c_double),
        ("vehicle_Rot_Acc_1", ctypes.c_double),
        ("vehicle_Rot_Acc_2", ctypes.c_double),
        ("vehicle_Rot_Ang_0", ctypes.c_double),
        ("vehicle_Rot_Ang_1", ctypes.c_double),
        ("vehicle_Rot_Ang_2", ctypes.c_double),
        ("vehicle_dynamics_response_0", ctypes.c_double),
        ("vehicle_dynamics_response_1", ctypes.c_double),
        ("vehicle_dynamics_response_2", ctypes.c_double),
        ("vehicle_dynamics_response_3", ctypes.c_double),
        ("vehicle_dynamics_response_4", ctypes.c_double),
        ("vehicle_dynamics_response_5", ctypes.c_double),
        ("vehicle_dynamics_response_6", ctypes.c_double),
        ("vehicle_dynamics_response_7", ctypes.c_double),
        ("vehicle_dynamics_response_8", ctypes.c_double),
        ("vehicle_dynamics_response_9", ctypes.c_double),
        ("vehicle_dynamics_response_10", ctypes.c_double),
        ("vehicle_dynamics_response_11", ctypes.c_double),
        ("vehicle_dynamics_response_12", ctypes.c_double),
        ("vehicle_dynamics_response_13", ctypes.c_double),
        ("vehicle_dynamics_response_14", ctypes.c_double),
        ("vehicle_X0", ctypes.c_double),
        ("vehicle_Y0", ctypes.c_double),
        ("vehicle_Z0", ctypes.c_double),
        ("vehicle_ay", ctypes.c_double),
        ("vehicle_az", ctypes.c_double),
        ("vehicle_yaw", ctypes.c_double),
        ("vehicle_Pitch", ctypes.c_double),
        ("vehicle_Roll", ctypes.c_double),
        ("vehicle_RollRate", ctypes.c_double),
        ("vehicle_PitchRate", ctypes.c_double),
        ("vehicle_YawRate", ctypes.c_double),
        ("vehicle_RollAcceleration", ctypes.c_double),
        ("vehicle_PitchAcceleration", ctypes.c_double),
        ("vehicle_YawAcceleration", ctypes.c_double),
        ("vehicle_domega_lf", ctypes.c_double),
        ("vehicle_domega_rf", ctypes.c_double),
        ("vehicle_domega_lr", ctypes.c_double),
        ("vehicle_domega_rr", ctypes.c_double),
        ("vehicle_res_1", ctypes.c_double),
        ("vehicle_res_2", ctypes.c_double),
        ("vehicle_res_3", ctypes.c_double),
        ("vehicle_res_4", ctypes.c_double),
        ("vehicle_res_5", ctypes.c_double),
        ("vehicle_res_6", ctypes.c_double),
        ("vehicle_res_7", ctypes.c_double),
        ("vehicle_res_8", ctypes.c_double),
        ("vehicle_res_9", ctypes.c_double),
        ("vehicle_res_10", ctypes.c_double),
        ("vehicle_res_11", ctypes.c_double),
        ("vehicle_res_12", ctypes.c_double),
        ("vehicle_res_13", ctypes.c_double),
        ("vehicle_res_14", ctypes.c_double),
        ("vehicle_dPits_lf", ctypes.c_double),
        ("vehicle_dPits_rf", ctypes.c_double),
        ("vehicle_dPits_lr", ctypes.c_double),
        ("vehicle_dPits_rr", ctypes.c_double),
        ("vehicle_X0_slf_old", ctypes.c_double),
        ("vehicle_X0_srf_old", ctypes.c_double),
        ("vehicle_X0_slr_old", ctypes.c_double),
        ("vehicle_X0_srr_old", ctypes.c_double),
        ("vehicle_X0_tlf_old", ctypes.c_double),
        ("vehicle_X0_trf_old", ctypes.c_double),
        ("vehicle_X0_tlr_old", ctypes.c_double),
        ("vehicle_X0_trr_old", ctypes.c_double),
        ("vehicle_d1", ctypes.c_double),
        ("vehicle_d2", ctypes.c_double),
        ("vehicle_d3", ctypes.c_double),
    ]

my_lib = ctypes.CDLL('Dll_DOF14_V2.dll',winmode=0)

# 设置Vehicle构造函数的原型
my_lib.init_veh.argtypes = [ctypes.c_double, ctypes.c_double, ctypes.c_double, ctypes.c_double, ctypes.c_double, ctypes.c_double, ctypes.c_double]
my_lib.init_veh.restype = ctypes.POINTER(Vehicle)

my_lib.init_state.argtypes = [ctypes.POINTER(Vehicle), ctypes.c_double, ctypes.c_double, ctypes.c_double, ctypes.c_double, ctypes.c_double, ctypes.c_double, ctypes.c_double, ctypes.c_double, ctypes.c_double, ctypes.c_double]
my_lib.init_state.restype = ctypes.POINTER(State)

my_lib.step.argtypes = [ctypes.POINTER(Vehicle), ctypes.POINTER(State), ctypes.c_double]
my_lib.step.restype = State

my_lib.get_Position.argtypes = [ctypes.POINTER(State)]
my_lib.get_Position.restype = ctypes.POINTER(ctypes.c_double)

my_lib.get_Velocity.argtypes = [ctypes.POINTER(State)]
my_lib.get_Velocity.restype = ctypes.POINTER(ctypes.c_double)

my_lib.get_Acceleration.argtypes = [ctypes.POINTER(State)]
my_lib.get_Acceleration.restype = ctypes.POINTER(ctypes.c_double)

my_lib.get_Angle_rate.argtypes = [ctypes.POINTER(State)]
my_lib.get_Angle_rate.restype = ctypes.POINTER(ctypes.c_double)

my_lib.get_Angle.argtypes = [ctypes.POINTER(State)]
my_lib.get_Angle.restype = ctypes.POINTER(ctypes.c_double)

my_lib.get_Omega.argtypes = [ctypes.POINTER(State)]
my_lib.get_Omega.restype = ctypes.POINTER(ctypes.c_double)

my_lib.get_Fx.argtypes = [ctypes.POINTER(State)]
my_lib.get_Fx.restype = ctypes.POINTER(ctypes.c_double)

my_lib.get_Fy.argtypes = [ctypes.POINTER(State)]
my_lib.get_Fy.restype = ctypes.POINTER(ctypes.c_double)

my_lib.get_Fz.argtypes = [ctypes.POINTER(State)]
my_lib.get_Fz.restype = ctypes.POINTER(ctypes.c_double)

my_lib.get_Sx.argtypes = [ctypes.POINTER(State)]
my_lib.get_Sx.restype = ctypes.POINTER(ctypes.c_double)

my_lib.get_beta.argtypes = [ctypes.POINTER(State)]
my_lib.get_beta.restype = ctypes.POINTER(ctypes.c_double)

my_lib.reset.argtypes = [ctypes.POINTER(State)]
my_lib.reset.restype = State

my_lib.set_Steering_wheel_Angle.argtypes = [ctypes.POINTER(Vehicle), ctypes.POINTER(State), ctypes.c_double]
my_lib.set_Steering_wheel_Angle.restype = State

my_lib.set_Torque.argtypes = [ctypes.POINTER(Vehicle), ctypes.POINTER(State), ctypes.c_double, ctypes.c_double, ctypes.c_double, ctypes.c_double]
my_lib.set_Torque.restype = State

my_lib.set_init_Velocity.argtypes = [ctypes.POINTER(Vehicle), ctypes.POINTER(State), ctypes.c_double]
my_lib.set_init_Velocity.restype = State

my_lib.set_init_Omg.argtypes = [ctypes.POINTER(Vehicle), ctypes.POINTER(State)]
my_lib.set_init_Omg.restype = State

class Dynamic_step():

    def __init__(self):
        
        self.vehicles =self.vehicle(Mt=1590.0,M=1430.0,Jx=900.0,Jy=2000.0,Jz=2000.0,R=0.285,steerRatio=1.0/26.0)
        self.states=self.state(wheel_Angle=0,Velocity_0=5*3.6,Mu_fl=0.8,Mu_fr=0.8,Mu_rl=0.8,Mu_rr=0.8,
                        Torque_of_FL=0.0,Torque_of_FR=0.0,Torque_of_RL=0.0,Torque_of_RR=0.0)

    def vehicle(self, Mt:float=1590.0, M:float=1430.0, Jx:float=900.0, Jy:float = 2000.0,Jz:float = 2000.0, R:float=0.285,steerRatio:float=1.0/26.0):
        '''
        车辆初始化
        '''
        vehicle = my_lib.init_veh(Mt, M, Jx, Jy, Jz, R, steerRatio)
        return vehicle

    def state(self,wheel_Angle:float=0.0, Velocity_0:float=40.0, Mu_fl:float=1.0, Mu_fr:float = 1.0,Mu_rl:float = 1.0, Mu_rr:float=1.0,
            Torque_of_FL:float=200.0,Torque_of_FR:float=200.0,Torque_of_RL:float=200.0,Torque_of_RR:float=200.0):
        '''
        车辆状态初始化
        wheel_Angle[deg], Velocity_0[km/h], Mu_fl, Mu_fr, Mu_rl, Mu_rr, 
        Torque_of_FL[Nm], Torque_of_FR[Nm], Torque_of_RL[Nm], Torque_of_RR[Nm]
        '''
        vehicle = self.vehicles
        state = my_lib.init_state(vehicle,wheel_Angle, Velocity_0, Mu_fl,Mu_fr,Mu_rl,Mu_rr,
                                Torque_of_FL,Torque_of_FR, Torque_of_RL, Torque_of_RR)
        my_lib.set_init_Omg(vehicle,state)
        return state


    def step(self, start: float=0.0, end: float=10.0, step: float=0.001):
            '''
            车辆仿真函数
            return: 
            {X_position[m], Y_position[m], 
            Vx[m/s], Vy[m/s], Vz[m/s],
            ax[m/s^2], ay[m/s^2], az[m/s^2], 
            PitchRate[rad/s], RollRate[rad/s], YawRate[rad/s], 
            Pitch[rad], Roll[rad], Yaw[rad], 
            Fx_lf[N], Fx_rf[N], Fx_lr[N], Fx_rr[N], 
            Fy_lf[N], Fy_rf[N], Fy_lr[N], Fy_rr[N], 
            Fz_lf[N], Fz_rf[N], Fz_lr[N], Fz_rr[N], 
            Sx_lf, Sx_rf, Sx_lr, Sx_rr, 
            Omega_lf[rad/s], Omega_rf[rad/s], Omega_lr[rad/s], Omega_rr[rad/s], beta[rad]}
            '''

            vehicle = self.vehicles
            state = self.states
            steps = int((end - start) / step) + 1
            for _ in range(steps):
                my_lib.step(vehicle, state, step) 
            X_position=my_lib.get_Position(state)[0]
            Y_position=my_lib.get_Position(state)[1]

            Vx=my_lib.get_Velocity(state)[0]
            Vy=my_lib.get_Velocity(state)[1]
            Vz=my_lib.get_Velocity(state)[2]

            ax = my_lib.get_Acceleration(state)[0]
            ay = my_lib.get_Acceleration(state)[1]
            az = my_lib.get_Acceleration(state)[2]

            PitchRate = my_lib.get_Angle_rate(state)[0]
            RollRate = my_lib.get_Angle_rate(state)[1]
            YawRate = my_lib.get_Angle_rate(state)[2]

            Pitch = my_lib.get_Angle(state)[0]
            Roll = my_lib.get_Angle(state)[1]
            Yaw = my_lib.get_Angle(state)[2]

            Fx_lf=my_lib.get_Fx(state)[0]
            Fx_rf=my_lib.get_Fx(state)[1]
            Fx_lr=my_lib.get_Fx(state)[2]
            Fx_rr=my_lib.get_Fx(state)[3]

            Fy_lf=my_lib.get_Fy(state)[0]
            Fy_rf=my_lib.get_Fy(state)[1]
            Fy_lr=my_lib.get_Fy(state)[2]
            Fy_rr=my_lib.get_Fy(state)[3]

            Fz_lf=my_lib.get_Fz(state)[0]
            Fz_rf=my_lib.get_Fz(state)[1]
            Fz_lr=my_lib.get_Fz(state)[2]
            Fz_rr=my_lib.get_Fz(state)[3]

            Sx_lf=my_lib.get_Sx(state)[0]
            Sx_rf=my_lib.get_Sx(state)[1]
            Sx_lr=my_lib.get_Sx(state)[2]
            Sx_rr=my_lib.get_Sx(state)[3]

            Omega_lf=my_lib.get_Omega(state)[0]
            Omega_rf=my_lib.get_Omega(state)[1]
            Omega_lr=my_lib.get_Omega(state)[2]
            Omega_rr=my_lib.get_Omega(state)[3]

            beta=my_lib.get_beta(state)[0] 

            output = {"X_position": X_position, 
                    "Y_position": Y_position, 
                    "Vx": Vx, 
                    "Vy": Vy, 
                    "Vz": Vz, 
                    "ax": ax, 
                    "ay": ay, 
                    "az": az, 
                    "PitchRate": PitchRate, 
                    "RollRate": RollRate, 
                    "YawRate": YawRate, 
                    "Pitch": Pitch, 
                    "Roll": Roll, 
                    "Yaw": Yaw, 
                    "Fx_lf": Fx_lf, 
                    "Fx_rf": Fx_rf, 
                    "Fx_lr": Fx_lr, 
                    "Fx_rr": Fx_rr, 
                    "Fy_lf": Fy_lf, 
                    "Fy_rf": Fy_rf, 
                    "Fy_lr": Fy_lr, 
                    "Fy_rr": Fy_rr, 
                    "Fz_lf": Fz_lf,
                    "Fz_rf": Fz_rf, 
                    "Fz_lr": Fz_lr, 
                    "Fz_rr": Fz_rr, 
                    "Sx_lf": Sx_lf, 
                    "Sx_rf": Sx_rf, 
                    "Sx_lr": Sx_lr, 
                    "Sx_rr": Sx_rr, 
                    "Omega_lf": Omega_lf, 
                    "Omega_rf": Omega_rf, 
                    "Omega_lr": Omega_lr, 
                    "Omega_rr": Omega_rr, 
                    "beta": beta}

            return output

    def reset(self):
        '''
        车辆状态重置函数
        '''
        state = self.states
        my_lib.reset(state)


    def set_Steering_wheel_Angle(self,wheel_Angle):
        '''
        车辆转角重新赋值函数
        '''
        vehicle = self.vehicles
        state = self.states
        my_lib.set_Steering_wheel_Angle(vehicle, state, wheel_Angle)#重新赋值方向盘转角[deg]


    def set_Torque(self,Torque_of_FL, Torque_of_FR, Torque_of_RL, Torque_of_RR): 
        '''
        车辆力矩重新赋值函数
        '''
        vehicle = self.vehicles
        state = self.states   
        my_lib.set_Torque(vehicle, state,Torque_of_FL, Torque_of_FR, Torque_of_RL, Torque_of_RR)#重新赋值四轮力矩[N/m]


    def set_init_Velocity(self,Velocity_0):
        '''
        车辆初始速度重新赋值函数
        '''
        vehicle = self.vehicles
        state = self.states
        my_lib.set_init_Velocity(vehicle, state, Velocity_0*3.6)#重新赋值初始速度[km/h]
        my_lib.set_init_Omg(vehicle, state)#重新赋值轮速


