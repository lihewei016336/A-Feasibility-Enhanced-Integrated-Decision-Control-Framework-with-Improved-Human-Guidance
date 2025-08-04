

#  Copyright (c). All Rights Reserved.
#  Institution: Changchun Univ. of Tech.
#  Leader: Bin Zhao
#  Member: Hewei Li & Xinyue Chen & Zhenxing Ding & Xiangjin Liu & Qingxuan Wang & Yongle Gao & Ze Wang & Junming Zhang
#  Creator: Hewei Li
#  Description: Output of SUMO

import  traci
from txdpy import get_Bletter,get_Sletter,get_num
import re
import cmath
import math
import random


def sur_veh_feature(IDs,max,min,X,Y,Vx,Vy,Yaw,n):
    sur_vehicle = []
    if IDs == []:
        Feature_X = 20
        Feature_y = 20
        Feature_V = 0
        Feature_angle = 0
        Feature_mask = 1
        x = [Feature_X, Feature_y, Feature_V, Feature_angle, Feature_mask]
        sur_vehicle.append(x)
    else:
        for i in IDs:
            pos = traci.vehicle.getPosition(i)
            V = traci.vehicle.getSpeed(i)
            angle = traci.vehicle.getAngle(i)
            Feature_X = X - pos[0]
            Feature_y = Y - pos[1]
            Feature_V = (Vx ** 2 + Vy ** 2) ** 0.5 - V
            Feature_angle = Yaw - angle
            Feature_mask = 0
            x = [Feature_X, Feature_y, Feature_V, Feature_angle, Feature_mask]
            sur_vehicle.append(x)
    x_veh = [list(row) for row in zip(*sur_vehicle)]
    En_norm_veh = []
    for i in range(len(x_veh)):
        max_ = max[i]
        min_ = min[i]
        X_norm = []
        for j in range(len(x_veh[i])):
            X_norm.append((x_veh[i][j] - min_) / (max_ - min_))
        for k in range(n):
            En = 0
            for l in X_norm:
                En = En + l ** (k + 1)
            En_norm_veh.append(En)
    En_norm_veh.append(len(IDs))
    return En_norm_veh
    
def sur_per_feature(IDs,max,min,X,Y,Vx,Vy,Yaw,n):
    sur_person = []
    if IDs == []:
        Feature_X = 20
        Feature_y = 20
        Feature_V = 0
        Feature_angle = 0
        Feature_mask = 1
        x = [Feature_X, Feature_y, Feature_V, Feature_angle, Feature_mask]
        sur_person.append(x)
    else:
        for i in IDs:
            pos = traci.person.getPosition(i)
            V = traci.person.getSpeed(i)
            angle = traci.person.getAngle(i)
            Feature_X = X - pos[0]
            Feature_y = Y - pos[1]
            Feature_V = (Vx ** 2 + Vy ** 2) ** 0.5 - V
            Feature_angle = Yaw - angle
            Feature_mask = 0
            x = [Feature_X, Feature_y, Feature_V, Feature_angle, Feature_mask]
            sur_person.append(x)
    x_per = [list(row) for row in zip(*sur_person)]
    En_norm_per = []
    for i in range(len(x_per)):
        max_ = max[i]
        min_ = min[i]
        X_norm = []
        for j in range(len(x_per[i])):
            X_norm.append((x_per[i][j] - min_) / (max_ - min_))
        for k in range(n):
            En = 0
            for l in X_norm:
                En = En + l ** (k + 1)
            En_norm_per.append(En)
    En_norm_per.append(len(IDs))
    return En_norm_per

# Feature of ego
def Feature_ego(T,delta_f,Starting_point,Dys):
    x_ego = []
    Dys.set_Steering_wheel_Angle(delta_f)
    Dys.set_Torque(T[0],T[1],T[2],T[3])
    result=Dys.step(start=0,end=0.01,step=0.001)
    X0 = Starting_point[0]
    Y0 = Starting_point[1]
    ego_Vx = result["Vx"]
    ego_Vy = result["Vy"]     
    ego_Yaw = result["Yaw"] * -57.3
    ego_Yaw_rate = result["YawRate"] * -1
    ego_Length = traci.vehicle.getLength('ego')
    if ego_Yaw < 0:
        ego_Angle = 360 + ego_Yaw
    else:
        ego_Angle = 0 + ego_Yaw
    Angle = traci.vehicle.getAngle('ego')
    X = X0 - result["Y_position"]
    Y = Y0 + result["X_position"] 
    ego_edgeID = traci.vehicle.getRoadID('ego')
    ego_laneID = traci.vehicle.getLaneID('ego')
    if ego_laneID == '':
        traci.vehicle.moveToXY(vehID = 'ego', edgeID = ego_edgeID, lane = 1, x = X, y = Y, angle=ego_Angle, keepRoute = 2, matchThreshold=100)
    else:
        ego_lane = get_num(ego_laneID)[1]
        traci.vehicle.moveToXY(vehID = 'ego', edgeID = ego_edgeID, lane = ego_lane, x = X, y = Y, angle=ego_Angle, keepRoute = 2, matchThreshold=100)
    Slip_FL = result["Sx_lf"]
    Slip_FR = result["Sx_rf"]
    Slip_RL = result["Sx_lr"]
    Slip_RR = result["Sx_rr"]   
    ego_Width = traci.vehicle.getWidth('ego')
    x_ego = [X,Y,ego_Vx,ego_Vy,Angle,ego_Yaw,Slip_FL,Slip_FR,Slip_RL,Slip_RR,ego_Length,ego_Width,ego_Yaw_rate]

    return x_ego

def collosion_safe(vehid,bikeid,perid,ego_X,ego_Y,ego_yaw):
    collosion = 0
    ego_x1 = ego_X - 1.25 * math.sin(math.radians(ego_yaw))
    ego_y1 = ego_Y - 1.25 * math.cos(math.radians(ego_yaw))
    ego_x2 = ego_X - 3.75 * math.sin(math.radians(ego_yaw))
    ego_y2 = ego_Y - 3.75 * math.cos(math.radians(ego_yaw))
    ids = vehid + bikeid
    veh_sort = []
    bik_sort = []
    per_sort = []
    if ids != []:
        for i in ids:
            [x,y] = traci.vehicle.getPosition(i)
            sur_veh_angle = traci.vehicle.getAngle(i)
            sur_veh_x1 = x - 1.25 * math.sin(math.radians(sur_veh_angle))
            sur_veh_y1 = y - 1.25 * math.cos(math.radians(sur_veh_angle))
            sur_veh_x2 = x - 3.75 * math.sin(math.radians(sur_veh_angle))
            sur_veh_y2 = y - 3.75 * math.cos(math.radians(sur_veh_angle))
            del11 = ((ego_x1 - sur_veh_x1) ** 2 + (ego_y1 - sur_veh_y1) ** 2) ** 0.5
            del12 = ((ego_x1 - sur_veh_x2) ** 2 + (ego_y1 - sur_veh_y2) ** 2) ** 0.5
            del21 = ((ego_x2 - sur_veh_x1) ** 2 + (ego_y2 - sur_veh_y1) ** 2) ** 0.5
            del22 = ((ego_x2 - sur_veh_x2) ** 2 + (ego_y2 - sur_veh_y2) ** 2) ** 0.5
            del_list = [del11, del12, del21, del22]
            sort_del_list = sorted(del_list)
            if traci.vehicle.getTypeID(i) == 'vehicle0':
                veh_sort.append(sort_del_list[0])
                if sort_del_list[0] < 2:
                    collosion = 1
                    traci.vehicle.remove(vehID = i, reason=3)
            else:
                bik_sort.append(sort_del_list[0])
                if sort_del_list[0] < 2:
                    collosion = 1
                    traci.vehicle.remove(vehID = i, reason=3)      
    if perid != []:
        for i in perid:
            [x,y] = traci.person.getPosition(i)
            sur_veh_angle = traci.person.getAngle(i)
            sur_veh_x1 = x - 1.25 * math.sin(math.radians(sur_veh_angle))
            sur_veh_y1 = y - 1.25 * math.cos(math.radians(sur_veh_angle))
            sur_veh_x2 = x - 3.75 * math.sin(math.radians(sur_veh_angle))
            sur_veh_y2 = y - 3.75 * math.cos(math.radians(sur_veh_angle))
            del11 = ((ego_x1 - sur_veh_x1) ** 2 + (ego_y1 - sur_veh_y1) ** 2) ** 0.5
            del12 = ((ego_x1 - sur_veh_x2) ** 2 + (ego_y1 - sur_veh_y2) ** 2) ** 0.5
            del21 = ((ego_x2 - sur_veh_x1) ** 2 + (ego_y2 - sur_veh_y1) ** 2) ** 0.5
            del22 = ((ego_x2 - sur_veh_x2) ** 2 + (ego_y2 - sur_veh_y2) ** 2) ** 0.5
            del_list = [del11, del12, del21, del22]
            sort_del_list = sorted(del_list)
            per_sort.append(sort_del_list[0])
            if sort_del_list[0] < 1.9:
                collosion = 1
                traci.person.remove(personID = i, reason=3)

    veh_sort_ = sorted(enumerate(veh_sort), key=lambda x: x[1])
    bik_sort_ = sorted(enumerate(bik_sort), key=lambda x: x[1])
    per_sort_ = sorted(enumerate(per_sort), key=lambda x: x[1])
    
    if veh_sort_ == []:
        veh_del = 28
    else:
        veh_del = veh_sort_[0][1]
    if bik_sort_ == []:
        bik_del = 28
    else:
       bik_del = bik_sort_[0][1]
    if per_sort_ == []:
        per_del = 28
    else:
        per_del = per_sort_[0][1]

    neiberId = [traci.vehicle.getLeader('ego'), traci.vehicle.getFollower('ego'), traci.vehicle.getLeftLeaders('ego'), traci.vehicle.getLeftFollowers('ego'), traci.vehicle.getRightLeaders('ego'), traci.vehicle.getRightFollowers('ego')]     

    return collosion, veh_del, bik_del, per_del, neiberId