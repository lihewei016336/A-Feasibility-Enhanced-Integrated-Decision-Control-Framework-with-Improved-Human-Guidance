import argparse
# from gym_sumo_Surrounding_vehicles import _GymSumo
from Env_Final_without import _GymSumo_without
from Env_Final import _GymSumo
import numpy as np
from stable_baselines3 import A2C
from stable_baselines3.common.vec_env import SubprocVecEnv, DummyVecEnv
from stable_baselines3.common.callbacks import BaseCallback
import os
from datetime import datetime
from torch.utils.tensorboard import SummaryWriter
from tensorboard_tools import start_tensorboard
DEFAULT_TB_PORT = 6002


def get_args():
    parser = argparse.ArgumentParser()
    # SUMO config
    parser.add_argument("--show_gui", type=bool, default=False, help="The flag of show SUMO gui.")
    parser.add_argument("--show_gui_evaluator", type=bool, default=True, help="The flag of show SUMO gui.")
    parser.add_argument("--sumocfgfile", type=str, default="C:\\Users\\Dell\\anaconda3\\envs\\Self_RL_l\\Lib\\site-packages\\gym\\envs\\user_defined\\sumo_files\\cross1.sumocfg", help="The path of the SUMO configure file.")
    parser.add_argument('--learning_rate', type=int, default=5e-5) 
    parser.add_argument('--batch_size', type=int, default=128) 
    parser.add_argument('--gamma', type=int, default=0.99) 
    parser.add_argument('--end_point', type=int, default=[-145, 8]) # 定义终点
    parser.add_argument('--start_point', type=int, default=[8, -115]) # 定义初始点
    parser.add_argument('--start_point_E', type=int, default=[1.6, -20.6]) # 定义初始点
    parser.add_argument('--start_point_D', type=int, default=[-20.6, 0]) # 定义初始点
    parser.add_argument('--Max_train_steps', type=int, default=5e5, help='Max training steps')
    parser.add_argument('--local_num', type=int, default=12) # 子进程个数
    parser.add_argument('--veh_nei', type=int, default=20) # 定义环境中车辆数量
    parser.add_argument('--veh_nei_real', type=int, default=30) # 定义环境中车辆数量
    parser.add_argument('--bik_nei', type=int, default=10) # 定义环境中自行车数量
    parser.add_argument('--per_nei', type=int, default=10) # 定义环境中行人数量
    parser.add_argument('--Only_3', type=int, default=1) # 随即车辆是否仅从第三车道发出
    parser.add_argument('--test', type=int, default=1) # 定义环境中行人数量

    args = parser.parse_args()

    return args

def make_myenv(env_id, param1, evalutor, rank, seed=0):
    def _init():
        if env_id > 3:
            env = _GymSumo(param1, evalutor) 
        else:
            env = _GymSumo_without(param1, evalutor)  # 创建环境实例
        return env
    return _init

class CustomCallback(BaseCallback):
    def __init__(self, writer, verbose=0):
        super(CustomCallback, self).__init__(verbose)
        self.writer = writer

    def _on_step(self) -> bool:
        # 获取环境返回的信息
        infos = self.locals['infos'][0]
        self.writer.add_scalar('environment_info/distance', infos['Dis_to_des'], self.num_timesteps)
        return True

if __name__ == '__main__':
    args = get_args()
    evalutor = False
    max_step = args.Max_train_steps
    local_num = args.local_num
    D2D = 200
    num = 1
    envs = SubprocVecEnv([make_myenv(i, args, evalutor, i) for i in range(local_num)])
    current_date = datetime.now().strftime('%Y-%m-%d')
    full_path = os.path.join("C:\\Users\\Dell\\Desktop\\New_final\\logs\\A2C_0", current_date)
    # model = A2C.load("C:\\Users\\Dell\\Desktop\\New_final\\model\\RL1.zip",env=envs)
    model = A2C('MlpPolicy', envs, 
                policy_kwargs=dict(net_arch=[64, 64, 32, 16]),
                learning_rate=args.learning_rate,
                gamma=args.gamma,
                verbose=1,
                tensorboard_log=full_path,
                )
    log_dir = os.path.join("C:\\Users\\Dell\\Desktop\\New_final\\plot_folder")
    writer = SummaryWriter(log_dir)
    start_tensorboard(logdir = log_dir, port=DEFAULT_TB_PORT)
    # callback = CustomCallback(writer=writer) #####
    for i in range(100):
        # model.learn(total_timesteps=max_step, reset_num_timesteps=False, callback=callback)
        model.learn(total_timesteps=max_step, reset_num_timesteps=False)
        print("Test eposide: {}".format(i))
        evalutor = True
        env_evalutor = _GymSumo(args, evalutor)
        des_list = []
        for j in range(5):
            obs, _ = env_evalutor.reset()
            done = 0
            r_all = []
            while not done:
                action, _state = model.predict(obs, deterministic=True)
                obs, reward, terminate, truncated, info = env_evalutor.step(action)
                done = terminate or truncated
                des_to_dis = info['Dis_to_des']
                r = info['Reward']
                r_all.append(r)
            des_list.append(des_to_dis)
        des_to_dis = min(des_list)
        writer.add_scalar('environment_info/distance', info['Dis_to_des'], i)
        writer.add_scalar('environment_info/TAR', np.mean(r_all), i)
        dis = round(des_to_dis)
        if dis < D2D:
            model.save("C:\\Users\\Dell\\Desktop\\New_final\\model\\RL{}".format(dis))
            D2D = dis 
        elif dis < 20:
            name = str(dis) + '_' + str(num)
            model.save("C:\\Users\\Dell\\Desktop\\New_final\\model\\RL"+name)
            num += 1
        env_evalutor.traci_close()
        
        # model.save("C:\\Users\\Dell\\Desktop\\RL\\Model_RL\\2023_3_24")


    print("The train is over!")