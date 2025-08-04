import numpy as np
import gym
import ps
from gym import spaces
from Random_flow import Random_flow
from Env_feature import Feature_ego, sur_veh_feature, sur_per_feature, collosion_safe
import random
import optparse
import traci
import time
from txdpy import get_num
from DOF14_1 import Dynamic_step
from R_slip import r_slip
import warnings
import os
import math
from sumolib import checkBinary
from gym.wrappers.time_limit import TimeLimit
from gym.utils import seeding
from Bezier import bezier, find_closest
from network2python import my_neural_network_function as Slip_ratio_cons

class _GymSumo_without(gym.Env):

    dys = None
    metadata = {
        'render.modes': ['human', 'rgb_array'],
        'video.frames_per_second': 50
    }
    
    def __init__(self, args, evalutor):
        ## 是否打开视景
        if evalutor:
            self.sumo_gui = args.show_gui_evaluator
        else:
            self.sumo_gui = args.show_gui
        if self.sumo_gui:
            sumoBinary = checkBinary('sumo-gui')
        else:
            sumoBinary = checkBinary('sumo')
        sumocfgfile = args.sumocfgfile
        traci.start([sumoBinary, "-c", sumocfgfile])
        self.veh_nei = args.veh_nei
        self.bik_nei = args.bik_nei
        self.per_nei = args.per_nei
        self.test = args.test
        only_3 = args.Only_3
        self.veh_nei_real = args.veh_nei_real

        ## 初始化
        self.steps = 0
        if self.test:
            RandomEntryTime = random.randint(600, 800)
        else:
            RandomEntryTime = random.randint(800, 1300)
        self.random_flow = Random_flow(veh=self.veh_nei,bik=self.bik_nei,per=self.per_nei,Only_3=only_3)
        self.steps = self.random_flow.initialize(RandomEntryTime, self.steps)
        traci.vehicle.add(vehID = 'ego', typeID = 'self_car', routeID = "DL",  departLane='first', depart='now',departSpeed='0')

        ## 动作、状态空间(T_fl,T_fr,T_rl,T_rr,delta_f,choose)
        self.action_space = spaces.Box(np.array([-1 for _ in range(3)]), np.array([1 for _ in range(3)]), dtype=np.float32)

        self.high_state = np.array([30,2,180,0.8,0.8,0.8,0.8,1,1.6,20,1,1,1])
        self.low_state = np.array([-1,-2,-180,-0.8,-0.8,-0.8,-0.8,0,-1.6,-20,-1,-1,-1])
        # low_state = np.array([-150,-150,-1,-1,-180,-150,-150,-150,-150,-0.8,-0.8,-0.8,-0.8,0,-1.6,-20,-1] + sur_low_state)
        # self.observation_space = spaces.Box(np.array([-1 for _ in range(len(self.high_state))]), np.array([1 for _ in range(len(self.high_state))]), dtype=np.float32)
        self.observation_space = spaces.Box(self.low_state, self.high_state)

        ## 动作空间上下限
        # self.min_vehphy = np.array([-400, -400, -400, -400, -180])
        # self.max_vehphy = np.array([400, 400, 400, 400, 180])
        self.action_low0 = np.array([-2800, -400, -60], dtype=np.float32)
        self.action_high0 = np.array([1600, 400, 60], dtype=np.float32)
        self.action_low1 = np.array([-2800, -400, -180], dtype=np.float32)
        self.action_high1 = np.array([1600, 400, 180], dtype=np.float32)

        ## 起点及终点
        self.start_point = args.start_point
        self.end_point = args.end_point
        if self.end_point[0] > 20:
            self.phase1_obj = [14.4, -20.5]
            self.phase2_obj = [-20.5, 1.6]
            self.phase3_obj = self.end_point
        elif self.end_point[0] < -20:
            self.phase1_obj = [1.6, -20.5]
            self.phase2_obj = [-20.5, 1.6]
            self.phase3_obj = self.end_point
        else:
            self.phase1_obj = [random.choice([11.2,8,4.8]), -20.5]
            self.phase2_obj = [random.choice([1.6,8,4.8]), 20.5]
            self.phase3_obj = self.end_point
        self.Bezier = bezier(self.phase1_obj)
        initial_shape0 = self.Bezier[2]
        if self.sumo_gui:
            shape0 = [ [initial_shape0[0][i], initial_shape0[1][i]] for i in range(len(initial_shape0[0]))]
            traci.polygon.add('bezier0', shape=shape0, color=(255, 0, 0), layer=10, lineWidth=0.1)

            Straight = [self.start_point, [self.start_point[0], self.start_point[1] + 2]]
            traci.polygon.add('Straight', shape=Straight, color=(255, 0, 0), layer=10, lineWidth=0.1)

            left = [self.start_point, [self.start_point[0] + 1.6, self.start_point[1] + 1]]
            traci.polygon.add('left', shape=left, color=(255, 0, 0), layer=10, lineWidth=0.1)

            right = [self.start_point, [self.start_point[0] - 1.6, self.start_point[1] + 1]]
            traci.polygon.add('right', shape=right, color=(255, 0, 0), layer=10, lineWidth=0.1)

            traci.gui.addView('ego_view')
        
        ## 附加初值
        self.max_veh = [150,150,20,360,1]
        self.min_veh = [-150,-150,0,0,0]
        self.max_per = [150,150,2,360,1]
        self.min_per = [-150,-150,0,0,0]
        self.max_bik = [150,150,5,360,1]
        self.min_bik = [-150,-150,0,0,0]
        self.n = 10

    def step(self, action):
        self.truncated = False
        self.terminate = False
        action = np.array(action)
        try:
            ego_lane = traci.vehicle.getLaneID('ego')
        except:
            print('Error')
        if ego_lane == '':
            ego_lane = 'E3_2'

        if ego_lane[0] == 'E' or ego_lane[0] == 'D':
            action_real = action * (np.array(self.action_high0) - np.array(self.action_low0)) / 2.0 + (np.array(self.action_high0) + np.array(self.action_low0)) / 2.0
        else:
            action_real = action * (np.array(self.action_high1) - np.array(self.action_low1)) / 2.0 + (np.array(self.action_high1) + np.array(self.action_low1)) / 2.0
        
        if self.states[2] < 0.5:
            T_fl = action_real[0] / 4
            T_fr = action_real[0] / 4
            T_rl = action_real[0] / 4
            T_rr = action_real[0] / 4
        else:   
            T_fl = action_real[0] / 3.64 - action_real[1] * 0.303 / 1.341
            T_fr = action_real[0] / 3.64 + action_real[1] * 0.303 / 1.346
            T_rl = 0.82 * T_fl
            T_rr = 0.82 * T_fr

        ## delet
        if self.states[2] < 0.5 and self.states[17] == 1:
            action_real[2] = 0
        elif self.states[2] < 0.5:
            action_real[2] = self.delta_f

        if ego_lane[0] == 'D' and self.states[13] == 0:
            action_real[2] = - action_real[2]
        

        ##
        self.vehcura = [T_fl,T_fr,T_rl,T_rr,action_real[2]]
        del_Tt = abs(action_real[0] - self.Tt)
        del_delta_f = abs(action_real[2] - self.delta_f)
        self.delta_f = action_real[2]
        self.Tt = action_real[0]
        reward = self.Reward(self.vehcura, del_delta_f, del_Tt) 
        if ego_lane[0] == 'D':
            yaw = self.states[4] - 90
        else:
            yaw = self.states[4]
        yaw_radians = math.radians(yaw)
        if action_real[2] > 0 and self.sumo_gui:
            Straight = [[self.states[0], self.states[1]], [self.states[0] + 2 * math.sin(yaw_radians), self.states[1] + 2 * math.cos(yaw_radians)]]
            traci.polygon.setShape('Straight', shape=Straight)
            right = [[self.states[0],self.states[1]], [self.states[0] - 1.6 * math.cos(yaw_radians) + 1 * math.sin(yaw_radians), self.states[1] + 1 * math.cos(yaw_radians) + 1.6 * math.sin(yaw_radians)]]
            traci.polygon.setShape('right', shape=right)
            left = [[self.states[0],self.states[1]], [self.states[0] + 1.6 * math.cos(yaw_radians) + 1 * math.sin(yaw_radians), self.states[1] + 1 * math.cos(yaw_radians) - 1.6 * math.sin(yaw_radians)]]
            traci.polygon.setShape('left', shape=left)
            if self.states[13]:
                traci.polygon.setColor('Straight', (0, 255, 0))
            else:
                traci.polygon.setColor('Straight', (255, 0, 0))
            traci.polygon.setColor('right', (0, 255, 0))
            traci.polygon.setColor('left', (255, 0, 0))
        elif self.sumo_gui:
            Straight = [[self.states[0],self.states[1]], [self.states[0] + 2 * math.sin(yaw_radians), self.states[1] + 2 * math.cos(yaw_radians)]]
            traci.polygon.setShape('Straight', shape=Straight)
            right = [[self.states[0],self.states[1]], [self.states[0] - 1.6 * math.cos(yaw_radians) + 1 * math.sin(yaw_radians), self.states[1] + 1 * math.cos(yaw_radians) + 1.6 * math.sin(yaw_radians)]]
            traci.polygon.setShape('right', shape=right)
            left = [[self.states[0],self.states[1]], [self.states[0] + 1.6 * math.cos(yaw_radians) + 1 * math.sin(yaw_radians), self.states[1] + 1 * math.cos(yaw_radians) - 1.6 * math.sin(yaw_radians)]]
            traci.polygon.setShape('left', shape=left)
            if self.states[13]:
                traci.polygon.setColor('Straight', (0, 255, 0))
            else:
                traci.polygon.setColor('Straight', (255, 0, 0))
            traci.polygon.setColor('right', (255, 0, 0))
            traci.polygon.setColor('left', (0, 255, 0))

        # self.vehcura = np.array(self.vehcura) + np.array(self.daction_real)
        # self.vehcura = np.clip(self.vehcura, self.min_vehphy, self.max_vehphy)
        self.states, state = self.stepPhysics(self.vehcura, action[0])
        self.random_flow.add_flow(self.veh_nei)
        ego_edge = traci.vehicle.getRoadID('ego')
        if self.sumo_gui:
            traci.gui.trackVehicle('ego_view','ego')

        if ego_edge == 'E0' and (self.states[1] > 16 or self.states[1] < 0):
            self.truncated = True
        elif ego_edge == 'D0' and (self.states[1] > 0 or self.states[1] < -9.6):
            self.truncated = True
        elif ego_edge == 'E1' and (self.states[0] < -16 or self.states[0] > 0):
            self.truncated = True
        elif ego_edge == 'D1' and (self.states[0] < 0 or self.states[0] > 9.6):
            self.truncated = True
        elif ego_edge == 'E2' and (self.states[1] < -16 or self.states[1] > 0):
            self.truncated = True
        elif ego_edge == 'D2' and (self.states[1] < 0 or self.states[1] > 9.6):
            self.truncated = True
        elif ego_edge == 'E3' and (self.states[0] < 0 or self.states[0] > 16):
            self.truncated = True
        elif ego_edge == 'D3' and (self.states[0] < -9.6 or self.states[0] > 0):
            self.truncated = True

        To_Des_dis = (self.states[7] ** 2 + self.states[8] ** 2) ** 0.5
        if  To_Des_dis < 1:
            self.terminate = True
            reward = 5

        info = {"Dis_to_des": To_Des_dis,
                "Reward": reward,
                "T_fl": self.vehcura[0],
                "T_fr": self.vehcura[1],
                "T_rl": self.vehcura[2],
                "T_rr": self.vehcura[3],
                "delta_f": self.vehcura[4],
                "X_position": self.states[0],
                "Y_position": self.states[1],
                }

        self.steps += 1

        # state_code = (2 * (state - self.low_state) / (self.high_state - self.low_state) - 1)
        state_code = state

        return np.array(state_code), reward, self.terminate, self.truncated, info
        

    def stepPhysics(self, action, Tt):
        # if action[5] < 0.3:
        edge = self.Bezier[2]
        # elif action[5] > 0.6:
        #     edge = self.Bezier[2]
        # else:
        #     edge = self.Bezier[1]
        edge_x = edge[0]
        edge_y = edge[1]
        x_ego = Feature_ego([action[0],action[1],action[2],action[3]],action[4],self.start_point,_GymSumo_without.Dys)
        # x_ego = Feature_ego([50,50,50,50],action[4],self.start_point,_GymSumo_without.Dys)
        time.sleep(0.01)
        traci.simulationStep(self.steps/100)
        X,Y,Vx,Vy,angle,Yaw,Slip_FL,Slip_FR,Slip_RL,Slip_RR,Length,Width,Yaw_rate = x_ego
        try:
            ego_lane = traci.vehicle.getLaneID('ego')
        except:
            print('Error')
        if ego_lane == '':
            ego_lane = 'E3_2'
        NearVehIDs, NearBikIDs, NearPerIDs = self.random_flow.getNearbyIDs(ego_pos=traci.vehicle.getPosition('ego'))       
        if ego_lane[0] == 'E':
            phase_obj = self.phase1_obj
        elif ego_lane[0] == 'D':
            phase_obj = self.phase3_obj
        else:
            phase_obj = self.phase2_obj
        
        
        # sur_vehicle_feature = sur_veh_feature(NearVehIDs,self.max_veh,self.min_veh,X,Y,Vx,Vy,Yaw,self.n)
        # sur_bike_feature = sur_veh_feature(NearBikIDs,self.max_bik,self.min_bik,X,Y,Vx,Vy,Yaw,self.n)
        # sur_person_feature = sur_per_feature(NearPerIDs,self.max_per,self.min_per,X,Y,Vx,Vy,Yaw,self.n)
        self.collosion, self.veh_del, self.bik_del, self.per_del, self.neiberId = collosion_safe(NearVehIDs, NearBikIDs, NearPerIDs,X,Y,Yaw)

        if ego_lane[0] == 'D':
            delta_x = Y - phase_obj[1]
            delta_y = X - phase_obj[0]
        else:
            delta_x = X - phase_obj[0]
            delta_y = Y - phase_obj[1]

        # if self.lane_keep == 1:
        #     nei_ChangeLane, Vx_max = self.random_flow.NeiChangeLane(self.neiberId,delta_x,delta_y,Vx)
        # else:
        #     nei_ChangeLane = 0
        #     Vx_max = 25

        if ego_lane == 'E3_6' or ego_lane == 'D2_2':
            nei_ChangeLane = 0
            Vx_max = 25
        else:
            nei_ChangeLane, Vx_max = self.random_flow.NeiChangeLane(self.neiberId,delta_x,delta_y,Vx)


        dis2des_x = X - self.end_point[0]
        dis2des_y = Y - self.end_point[1]
        if ego_lane[0] == 'E':
            ego_Lateral_pos = traci.vehicle.getLateralLanePosition('ego')
            junction = 1
        elif ego_lane[0] == 'D':
            ego_Lateral_pos = traci.vehicle.getLateralLanePosition('ego')
            junction = 1
        else:
            # if Yaw > -45 and Yaw < 45:
            #     close_num = self.find_closest(edge_y,Y)
            #     ego_Lateral_pos = edge_x[close_num] - X
            # else:
            #     close_num = self.find_closest(edge_x,X)
            #     ego_Lateral_pos = edge_y[close_num] - Y
            close_num = self.find_closest(edge_y,Y)
            ego_Lateral_pos = edge_x[close_num] - X

            junction = 0

        if ego_lane[0] == 'D':
            Yaw = Yaw + 90

        if self.lane_keep and junction == 1 and abs(Yaw) < 2 and abs(delta_x) > 1.6 and abs(ego_Lateral_pos) / 1.6 < 0.3 and nei_ChangeLane and Vx > 1:
            self.lane_keep = 0
        
        if self.ego_lane != ego_lane and abs(ego_Lateral_pos) / 1.6 < 0.95:
            self.lane_keep = 1
            self.ego_lane = ego_lane

        del_ego_Lateral_pos = abs(ego_Lateral_pos) - abs(self.ego_Lateral_pos)

        self.ego_Lateral_pos = ego_Lateral_pos
        
        # sur_state = tuple(sur_vehicle_feature+sur_bike_feature+sur_person_feature)

        if ego_lane[0] == 'E':
            if ego_lane[3] == '2':  
                max_Vx = 10 
            else:
                if abs(delta_y) > 50:
                    max_Vx = 20 
                else:
                    max_Vx = 0.2 * abs(delta_y) + 10
        elif ego_lane[0] == 'D':
            dis2des = (dis2des_x ** 2 + dis2des_y ** 2) ** 0.5
            if dis2des > 50:
                max_Vx = 20
            else:
                max_Vx = 0.4 * dis2des
        else:
            max_Vx = 10

        if self.neiberId[0] == None:
            front_veh = False
            front_veh_dis = 20
        else:
            front_veh = True
            front_veh_dis = self.neiberId[0][1]

        if front_veh and front_veh_dis < 20 and junction == 1:
            Vx_del = front_veh_dis * 8 / 19  - 8 / 19
            if Vx_del < 0:
                Vx_del = 0
            max_Vx = traci.vehicle.getSpeed(self.neiberId[0][0]) * math.cos(math.radians(traci.vehicle.getAngle(self.neiberId[0][0]))) + Vx_del - 0.5
        
        if max_Vx > Vx_max:
            max_Vx = Vx_max


        return (X,Y,Vx,Vy,Yaw,delta_x,delta_y,dis2des_x,dis2des_y,Slip_FL,Slip_FR,Slip_RL,Slip_RR,self.lane_keep,ego_Lateral_pos,Vx - max_Vx,del_ego_Lateral_pos,junction,Tt), (Vx,Vy,Yaw,Slip_FL,Slip_FR,Slip_RL,Slip_RR,self.lane_keep,ego_Lateral_pos,Vx - max_Vx,del_ego_Lateral_pos,junction,Tt) 


    def reset(self):
        start_X = self.start_point[0]
        start_Y = self.start_point[1]
        _GymSumo_without.Dys = Dynamic_step()
        self.veh_nei = self.veh_nei_real
        result=_GymSumo_without.Dys.step(start=0,end=0,step=0.001)
        Slip_FL = result["Sx_lf"]
        Slip_FR = result["Sx_rf"]
        Slip_RL = result["Sx_lr"]
        Slip_RR = result["Sx_rr"]
        ego_Vx = result["Vx"]
        ego_Vy = result["Vy"]
        ego_Y = start_Y
        ego_X = start_X 
        delta_x = ego_X - self.phase1_obj[0]
        delta_y = ego_Y - self.phase1_obj[1]
        dis2des_x = ego_X - self.end_point[0]
        dis2des_y = ego_Y - self.end_point[1]
        ego_Angle = 0 
        traci.vehicle.moveToXY(vehID = 'ego', edgeID = 'E3', lane = 4, x = start_X, y = start_Y, angle=ego_Angle, keepRoute = 0, matchThreshold=100)
        time.sleep(0.01)
        traci.simulationStep(self.steps/100)
        if self.sumo_gui:
            traci.gui.setZoom('ego_view',500)
        NearVehIDs, NearBikIDs, NearPerIDs = self.random_flow.getNearbyIDs(ego_pos=(ego_X, ego_Y)) 
        sur_vehicle_feature = sur_veh_feature(NearVehIDs,self.max_veh,self.min_veh,ego_X,ego_Y,ego_Vx,ego_Vy,ego_Angle,self.n)
        sur_bike_feature = sur_veh_feature(NearBikIDs,self.max_bik,self.min_bik,ego_X,ego_Y,ego_Vx,ego_Vy,ego_Angle,self.n)
        sur_person_feature = sur_per_feature(NearPerIDs,self.max_per,self.min_per,ego_X,ego_Y,ego_Vx,ego_Vy,ego_Angle,self.n)   
        self.collosion, self.veh_del, self.bik_del, self.per_del, self.neiberId = collosion_safe(NearVehIDs, NearBikIDs, NearPerIDs,ego_X,ego_Y,ego_Angle)
        self.steps += 1
        self.lane_keep = 1
        self.Vx_reset_time = 0
        self.ego_lane = traci.vehicle.getLaneID('ego')
        self.ego_Lateral_pos = traci.vehicle.getLateralLanePosition('ego')
        if self.ego_lane[0] == 'E':
            if self.ego_lane[3] == '2':
                max_Vx = 6
            elif self.ego_lane[3] == '3':
                max_Vx = 8   
            elif self.ego_lane[3] == '4':
                max_Vx = 12     
            elif self.ego_lane[3] == '5':
                max_Vx = 16 
            else:
                max_Vx = 20 
        elif self.ego_lane[0] == 'D':
            dis2des = (dis2des_x ** 2 + dis2des_y ** 2) ** 0.5
            if dis2des > 50:
                max_Vx = 20
            else:
                max_Vx = 0.4 * dis2des
        else:
            max_Vx = 10
        if self.neiberId[0] == None:
            front_veh = False
            front_veh_dis = 20
        else:
            front_veh = True
            front_veh_dis = self.neiberId[0][1]

        if front_veh and front_veh_dis < 5:
            max_Vx = traci.vehicle.getSpeed(self.neiberId[0][0]) * math.cos(math.radians(traci.vehicle.getAngle(self.neiberId[0][0])))

        # self.states = [ego_X,ego_Y,ego_Vx,ego_Vy,ego_Angle,delta_x,delta_y,dis2des_x,dis2des_y,Slip_FL,Slip_FR,Slip_RL,Slip_RR,self.lane_keep,ego_Lateral_pos,ego_Vx - max_Vx,0] + sur_vehicle_feature+sur_bike_feature+sur_person_feature
        self.states = [ego_X,ego_Y,ego_Vx,ego_Vy,ego_Angle,delta_x,delta_y,dis2des_x,dis2des_y,Slip_FL,Slip_FR,Slip_RL,Slip_RR,self.lane_keep,self.ego_Lateral_pos,ego_Vx - max_Vx,0,1,0]
        
        state = [ego_Vx,ego_Vy,ego_Angle,Slip_FL,Slip_FR,Slip_RL,Slip_RR,self.lane_keep,self.ego_Lateral_pos,ego_Vx - max_Vx,0,1,0]
        self.delta_f = 0
        self.Tt = 0
        # state_code = (2 * (state - self.low_state) / (self.high_state - self.low_state) - 1)
        state_code = state
       
        return np.array(state_code), {}
    

    def Reward(self, action, del_delta_f, del_Tt):
        delta_f = action[4]
        Tt = action[0] + action[1] + action[2] + action[3]

        ego_lane = traci.vehicle.getLaneID('ego')
        if ego_lane == '':
            ego_lane = 'E3_6'
        
        # if self.states[13]:
        #     Vx_reward = 1 - abs(self.states[15]) / 20 
        # else:
        #     Vx_reward = 0.1
        Vx_reward = 1 - abs(self.states[15]) / 20 

        # if ((Tt > 0 and self.states[15] < 0)) and self.states[13]:
        #     Tt_reward = abs(Tt / 3200)
        # elif ((Tt < 0 and self.states[15] > 0)) and self.states[13]:
        #     Tt_reward = abs(Tt / 5000)    
        # elif self.states[13]:
        #     Tt_reward = 0
        # else:
        #     Tt_reward = 0.5 - abs(Tt / 3200)

        if ((Tt > 0 and self.states[15] < 0)):
            Tt_reward = 0.5
        elif ((Tt < 0 and self.states[15] > 0)):
            Tt_reward = abs(Tt / 5000)    
        else:
            Tt_reward = 0
        
        if abs(del_Tt) > 400:
            del_Tt_reward = -0.5
        else:
            del_Tt_reward = 0.5



        if abs(self.states[14])/1.6 < 0.1 and self.states[13]:
            KeepLane_reward = 0.6
        elif self.states[13] and self.states[16] < 0:
            KeepLane_reward = 0.55
        elif ((self.states[14] > 0 and delta_f < 0) or (self.states[14] < 0 and delta_f > 0)) and self.states[13] and self.states[17] == 1:
            KeepLane_reward = 0.55 - abs(delta_f) / 600
        elif ((self.states[14] > 0 and delta_f < 0) or (self.states[14] < 0 and delta_f > 0)) and self.states[13] and self.states[17] == 0:
            KeepLane_reward = 0.5
        elif delta_f * self.states[5] > 0 and self.states[17] == 1 and not self.states[13]:
            KeepLane_reward = abs(delta_f) / 120
        else:
            KeepLane_reward = 0


        if self.states[17] == 1:
            delta_f_reward = - abs(self.states[4] / 90)
        else:
            if self.states[4] < 0 and self.states[4] > -90 and delta_f < 0:
                delta_f_reward = 0
            elif self.states[4] < 0 and self.states[4] > -90 and delta_f >= 0:
                delta_f_reward = 0.032 * abs(delta_f / 57.3)
            elif self.states[4] > 0 and delta_f > 0:
                delta_f_reward = 0.032 * abs(delta_f / 57.3) - 0.1
            elif self.states[4] < -90 and delta_f < 0:
                delta_f_reward = 0.032 * abs(delta_f / 57.3) - 0.1
            else:
                delta_f_reward = -0.1 - 0.032 * abs(delta_f / 57.3)

        if self.states[13]:
            factor_delta_f = 2
        else:
            factor_delta_f = 0.5

        if abs(self.states[4]) < 3 and self.states[17] == 1 and self.states[13]:
            correct_KeepLane_reward = 1.5 - abs(delta_f) / 120 - abs(self.states[4]) / 90
        elif self.states[13] and self.states[4] * delta_f > 0 and self.states[17] == 1 and abs(self.states[4]) > 3:
            correct_KeepLane_reward = abs(delta_f) / 60
        else:
            correct_KeepLane_reward = 0


        R_slip = 0.0003 * r_slip(self.states[2], delta_f/26, self.states[9:13], mu=1)

        reward = Vx_reward + delta_f_reward * factor_delta_f + KeepLane_reward + Tt_reward + correct_KeepLane_reward + R_slip + del_Tt_reward
        # reward = lane_reward * 2 + delta_f_reward + KeepLane_reward + S_reward + reward_1

        if self.states[2] < 0.1:
            self.Vx_reset_time += 1
            if self.Vx_reset_time > 500:
                self.truncated = True
        else:
            self.Vx_reset_time = 0

        if self.collosion == 1:
            self.truncated = True

        if self.states[14] / 1.6 > 0.95 and self.lane_keep and (ego_lane[0] == 'E' or ego_lane[0] == 'D') and ego_lane[3] != 6:
            self.truncated = True
        
        if self.states[1] > 9.5 and self.end_point[0] < -20:
            self.truncated = True
        elif self.states[1] > 0 and self.end_point[0] > 20:
            self.truncated = True

        if ego_lane[0] == 'E':
            if self.end_point[0] > 20 and self.states[1] > -21 and ego_lane[3] != '2':
                self.truncated = True
            elif self.end_point[0] < 20 and self.end_point[0] > -20 and self.states[1] > -21 and (ego_lane[3] == '2' or ego_lane[3] == '6'):
                self.truncated = True
            elif self.end_point[0] < -20 and self.states[1] > -21 and ego_lane[3] != '6':
                self.truncated = True

        if abs(self.states[9]) > 0.8 and abs(self.states[10]) > 0.8 and abs(self.states[11]) > 0.8 and abs(self.states[12]) > 0.8 and self.states[2] > 1:
            self.truncated = True

        if self.states[0] < -150:
            self.truncated = True

        if self.states[2] > 25:
            self.truncated = True
            
        return reward
    
    def find_closest(self, lst, target):
        closest = 0
        for i in range(len(lst)):
            if abs(lst[i] - target) < abs(lst[closest] - target):
                closest = i
        return closest


    
    # def check_dangerous_driving(self):

    def traci_close(self):
        traci.close()



def env_creator(**kwargs):
    return TimeLimit(_GymSumo_without(**kwargs), 10000)
    
if __name__ == '__main__':
    pass