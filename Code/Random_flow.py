import traci
import random
import time


class Random_flow():
    def __init__(self, veh = 20, bik = 10, per = 10, Only_3 = 0):
        self.edge_id = (['E3','E0'],['E3','E1'],['E3','E2'],['E0','E1'],['E0','E2'],['E0','E3'],['E1','E0'],['E1','E2'],['E1','E3'],['E2','E1'],['E2','E0'],['E2','E3'])
        self.ID_bike = []
        self.ID_veh = []
        self.ID_per = []
        my_veh_list = [i * 0.01 for i in range(0, 500)]
        self.new_veh_list = [round(x, 2) for x in my_veh_list]
        self.veh_id = list(map(str, self.new_veh_list))
        my_bike_list = [i * 0.01 for i in range(501, 1000)]
        self.new_bike_list = [round(x, 2) for x in my_bike_list]
        self.bike_id = list(map(str, self.new_bike_list))
        self.person_id = self.veh_id
        self.veh_no = veh
        self.bik_no = bik
        self.per_no = per
        self.only_3 = Only_3

    def initialize(self, RandomEntryTime, steps):
        Route_id = traci.route.getIDList()
        Route = []
        if self.only_3:
            for i in Route_id:
                if i[0] != '!' and (i[0] == 'D'):
                    Route.append(i)
        else:
            for i in Route_id:
                if i[0] != '!':
                    Route.append(i)
        self.Bik_Route = []
        for i in Route_id:
            if i[0] != '!':
                self.Bik_Route.append(i)
        for i in range(RandomEntryTime):
            if len(self.veh_id) < 5:
                self.veh_id = list(map(str, self.new_veh_list))
            if len(self.bike_id) < 5:
                self.bike_id = list(map(str, self.new_bike_list))
            if len(self.person_id) < 5:
                self.person_id = list(map(str, self.new_veh_list))
            veh_id = traci.vehicle.getIDList()
            
            edgeID = traci.edge.getIDList()
            edge = []
            vehicle_id=[]
            bike_id=[]
            person_id=[]
            for i in edgeID:
                if i[0] != ':':
                    edge.append(i)
            for i in veh_id:
                veh_type = traci.vehicle.getTypeID(i)
                if veh_type != 'bike':
                    vehicle_id.append(i)
                else:
                    bike_id.append(i)
            person_id = traci.person.getIDList()
            if steps > 5:
                if len(vehicle_id) < self.veh_no:
                    self.generate_random_traffic(Route)
                else:
                    self.ID_veh = []  
            if steps > 5:
                if len(bike_id) < self.bik_no:
                    self.generate_random_bike(self.Bik_Route)
                else:
                    self.ID_bike = []
            if steps > 5:
                if len(person_id) < self.per_no:
                    self.generate_random_person()      
                else:
                    self.ID_per = []
            time.sleep(0.01)
            traci.simulationStep(steps/100)
            steps += 1
        return steps
    
    def generate_random_traffic(self,Route):
        veh_id = traci.vehicle.getIDList()
        if self.ID_veh != []:
            if self.ID_veh not in veh_id:
                traci.vehicle.remove(vehID = self.ID_veh, reason=3)
        result = [x for x in self.veh_id if x not in veh_id]
        self.ID_veh = random.choice(result)
        try:
            traci.vehicle.add(vehID = self.ID_veh, typeID = 'vehicle0', routeID = random.choice(Route), departLane = "best", departPos = str(0), departSpeed='random', depart = 0)
        except:
            print('The vehicle ' + self.ID_veh + ' already exists.') 

    def generate_random_person(self):
        per_id = traci.person.getIDList()
        Edge_id = random.choice(self.edge_id)
        Edge = Edge_id[0]
        if list(self.ID_per) != []:
            if self.ID_per not in per_id:
                traci.person.remove(personID = self.ID_per, reason=3)
        result = [x for x in self.person_id if x not in per_id]
        self.ID_per = random.choice(result)
        ID_per = self.ID_per
        try:
            traci.person.add(personID = self.ID_per, edgeID = Edge, pos = str(random.uniform(10, 100)), depart = -3)
            traci.person.appendWalkingStage(personID = ID_per, edges = Edge_id, arrivalPos = 1, duration=-1, speed=-1, stopID='')
        except:
            print('The person ' + self.ID_per + ' already exists.') 

    def generate_random_bike(self,Route):
        veh_id = traci.vehicle.getIDList()
        if self.ID_bike != []:
            if self.ID_bike not in veh_id:
                traci.vehicle.remove(vehID = self.ID_bike, reason=3)
        result = [x for x in self.bike_id if x not in veh_id]
        self.ID_bike = random.choice(result)
        try:
            traci.vehicle.add(vehID = self.ID_bike, typeID = 'bike', routeID = random.choice(Route), departLane = "random", departPos = str(random.uniform(10, 100)), departSpeed='random', depart = 0)
        except:
            print('The vehicle ' + self.ID_bike + ' already exists.') 

    def getNearbyIDs(self, ego_pos):
        X_width = 20
        Y_width = 20
        veh_id = traci.vehicle.getIDList()
        veh_id = list(veh_id)
        if veh_id.count('ego'):
            veh_id.remove('ego')
        NearVehIDs = []
        NearBikIDs = []
        NearPerIDs = []
        for i in veh_id:
            [pos_x, pos_y] = traci.vehicle.getPosition(i)
            if abs((pos_x) - ego_pos[0]) < X_width and abs((pos_y) - ego_pos[1]) < Y_width:
                veh_type = traci.vehicle.getTypeID(i)
                if veh_type == 'bike':
                    NearBikIDs.append(i)
                else:
                    NearVehIDs.append(i)
        per_id = traci.person.getIDList()
        for i in per_id:
            [pos_x, pos_y] = traci.person.getPosition(i)
            if abs((pos_x) - ego_pos[0]) < X_width and abs((pos_y) - ego_pos[1]) < Y_width:
                NearPerIDs.append(i)

        return NearVehIDs, NearBikIDs, NearPerIDs
    
    def add_flow(self, veh):
        veh_id = traci.vehicle.getIDList()    

        Route_id = traci.route.getIDList()
        Route = []
        if self.only_3:
            for i in Route_id:
                if i[0] != '!' and (i[0] == 'D'):
                    Route.append(i)
        else:
            for i in Route_id:
                if i[0] != '!':
                    Route.append(i)


        edgeID = traci.edge.getIDList()
        edge = []
        vehicle_id=[]
        bike_id=[]
        person_id=[]
        if len(self.veh_id) < 5:
            self.veh_id = list(map(str, self.new_veh_list))
        if len(self.bike_id) < 5:
            self.bike_id = list(map(str, self.new_bike_list))
        if len(self.person_id) < 5:
            self.person_id = list(map(str, self.new_veh_list))
        for i in edgeID:
            if i[0] != '!':
                edge.append(i)
        for i in veh_id:
            veh_type = traci.vehicle.getTypeID(i)
            if veh_type != 'bike':
                vehicle_id.append(i)
            else:
                bike_id.append(i)
        person_id = traci.person.getIDList()
        if len(vehicle_id) < veh:
            self.generate_random_traffic(Route)
        else:
            self.ID_veh = []  
        if len(bike_id) < self.bik_no:
            self.generate_random_bike(self.Bik_Route)
        else:
            self.ID_bike = []
        if len(person_id) < self.per_no:
            self.generate_random_person()      
        else:
            self.ID_per = []

    def NeiChangeLane(self, neiberId, delta_x, delta_y, Vx):
        edgeid = ['D0','D1','D2','D3','E0','E1','E2','E3']
        nei_ChangeLane = 1
        edge_vehid = []
        veh_id = set(traci.vehicle.getIDList())
        for i in edgeid:
            edge_vehid.append(traci.edge.getLastStepVehicleIDs(i))
        edgevehid = set([item for sublist in edge_vehid for item in sublist])
        jun_veh = list(veh_id - edgevehid - {'ego'})
        Vx_max = 25
        try:
            ego_lane = traci.vehicle.getLaneID('ego')
        except:
            print('Error')
        if ego_lane == '':
            ego_lane = 'E3_2'
        nei = [0,0]
        if ego_lane[0] == 'E':
            if delta_x > 1.6:
                nei[0] = neiberId[2]
                nei[1] = neiberId[3]
            elif delta_x < -1.6:
                nei[0] = neiberId[4]
                nei[1] = neiberId[5]
        elif ego_lane[0] == 'D':
            if delta_x > 1.6:
                nei[0] = neiberId[2]
                nei[1] = neiberId[3]
            elif delta_x < -1.6:
                nei[0] = neiberId[4]
                nei[1] = neiberId[5]
        # elif ego_lane[0] == 'D':
        #     if delta_y > 1.6:
        #         nei[0] = neiberId[2]
        #         nei[1] = neiberId[3]
        #     elif delta_y < -1.6:
        #         nei[0] = neiberId[4]
        #         nei[1] = neiberId[5]

        if ego_lane[0] == 'E' or ego_lane[0] == 'D':
            if nei[0] == () and nei[1] == ():
                nei_ChangeLane = 1
            elif nei[0] == ():
                try:
                    VehVx1 = traci.vehicle.getSpeed(nei[1][0][0])
                except:
                    print('eroor')
                if nei[1][0][1] < 0:
                    nei_ChangeLane = 0
                    Vx_max = VehVx1 - 5
                elif nei[1][0][1] < 5 and VehVx1 > Vx:
                    nei_ChangeLane = 0
            elif nei[1] == ():
                try:
                    VehVx0 = traci.vehicle.getSpeed(nei[0][0][0])
                except:
                    print('eroor')
                if nei[0][0][1] < 0:
                    nei_ChangeLane = 0
                    Vx_max = VehVx0 - 5
                elif nei[0][0][1] < 5 and VehVx0 < Vx:
                    nei_ChangeLane = 0
                    Vx_max = VehVx0 - 5
            elif nei[0] == 0:
                nei_ChangeLane = 0
            else:
                try:
                    VehVx0 = traci.vehicle.getSpeed(nei[0][0][0])
                    VehVx1 = traci.vehicle.getSpeed(nei[1][0][0])
                except:
                    print('eroor')
                if nei[0][0][1] < 0:
                    nei_ChangeLane = 0
                    Vx_max = VehVx0 - 5
                elif nei[1][0][1] < 0:
                    nei_ChangeLane = 0
                    Vx_max = VehVx1 - 5
                elif nei[0][0][1] < 5 and VehVx0 < Vx:
                    nei_ChangeLane = 0
                    Vx_max = VehVx0 - 5
                elif nei[1][0][1] < 2 and VehVx1 > Vx:
                    nei_ChangeLane = 0
        else:
            nei_ChangeLane = 0
            if jun_veh != []:
                ego_x, ego_y = traci.vehicle.getPosition('ego')
                VxMax = []
                for i in jun_veh:
                    x,y = traci.vehicle.getPosition(i)
                    delta = ((ego_x - x) ** 2 + (ego_y - y) ** 2) ** 0.5
                    if x < ego_x and y > ego_y:
                        VxMax_value = 0 + delta * 0.5 - 0.5
                        if VxMax_value < 0:
                            VxMax_value = 0
                        VxMax.append(VxMax_value)
                if VxMax != []:
                    Vx_max = min(VxMax)
        

        return nei_ChangeLane, Vx_max
    

    def NeiChangeLaneVx(self, neiberId, delta_x):
        edgeid = ['D0','D1','D2','D3','E0','E1','E2','E3']
        edge_vehid = []
        for i in edgeid:
            edge_vehid.append(traci.edge.getLastStepVehicleIDs(i))
        Vx_max = 25
        try:
            ego_lane = traci.vehicle.getLaneID('ego')
        except:
            print('Error')
        if ego_lane == '':
            ego_lane = 'E3_2'
        nei = [0,0]
        if ego_lane[0] == 'E':
            if delta_x > 1.6:
                nei[0] = neiberId[2]
                nei[1] = neiberId[3]
            elif delta_x < -1.6:
                nei[0] = neiberId[4]
                nei[1] = neiberId[5]
        elif ego_lane[0] == 'D':
            if delta_x > 1.6:
                nei[0] = neiberId[2]
                nei[1] = neiberId[3]
            elif delta_x < -1.6:
                nei[0] = neiberId[4]
                nei[1] = neiberId[5]

        if nei[0] != () and nei[0] != 0:

            Vx_del = nei[0][0][1] * 8 / 19  - 8 / 19
            VehVx1 = traci.vehicle.getSpeed(nei[0][0][0])
            Vx_max = VehVx1 - 3 + Vx_del

        return  Vx_max


        


