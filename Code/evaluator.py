from stable_baselines3 import A2C
from Env_Final import _GymSumo
import argparse
import matplotlib.pyplot as plt
from stable_baselines3.common.callbacks import BaseCallback
from tensorboard_tools import start_tensorboard
from torch.utils.tensorboard import SummaryWriter
import time


def first_order_filter(u, y_previous, dt, tau):
    return (u + tau * y_previous) / (1 + dt * tau)


def get_args():
    parser = argparse.ArgumentParser()
    # SUMO config
    parser.add_argument("--show_gui", type=bool, default=False, help="The flag of show SUMO gui.")
    parser.add_argument("--show_gui_evaluator", type=bool, default=True, help="The flag of show SUMO gui.")
    parser.add_argument("--sumocfgfile", type=str, default="C:\\Users\\Dell\\anaconda3\\envs\\Self_RL\\Lib\\site-packages\\gym\\envs\\user_defined\\sumo_files\\cross1.sumocfg", help="The path of the SUMO configure file.")
    parser.add_argument('--learning_rate', type=int, default=1e-4) 
    parser.add_argument('--batch_size', type=int, default=128) 
    parser.add_argument('--gamma', type=int, default=0.99) 
    parser.add_argument('--end_point', type=int, default=[-145, 8]) # 定义终点
    parser.add_argument('--start_point', type=int, default=[8, -115]) # 定义初始点
    parser.add_argument('--Max_train_steps', type=int, default=5e5, help='Max training steps')
    parser.add_argument('--local_num', type=int, default=1) # 子进程个数
    parser.add_argument('--veh_nei', type=int, default=20) # 定义环境中车辆数量
    parser.add_argument('--veh_nei_real', type=int, default=30) # 定义环境中车辆数量
    parser.add_argument('--bik_nei', type=int, default=20) # 定义环境中自行车数量
    parser.add_argument('--per_nei', type=int, default=20) # 定义环境中行人数量
    parser.add_argument('--Only_3', type=int, default=1) # 定义环境中行人数量
    parser.add_argument('--test', type=int, default=1) # 定义环境中行人数量


    args = parser.parse_args()
    return args

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
    start_tensorboard('info', port= 6001)
    writer = SummaryWriter('info')
    evalutor = True
    env_evalutor = _GymSumo(args, evalutor)
    model = A2C.load("C:\\Users\\Dell\\Desktop\\ISR_EG\\model3\\RL1_8.zip",env=env_evalutor)
    obs, _ =  env_evalutor.reset()
    done = 0
    reward_ = []
    times = []
    delta_f = []
    t = 0
    tau = 0.1  # 时滞时间常数
    dt = 0.01  # 假设的时间步长
    log_dir = "./eva_plot"
    writer = SummaryWriter(log_dir)
    # start_tensorboard(logdir = full_path + '\\A2C_0', port=DEFAULT_TB_PORT)
    callback = CustomCallback(writer=writer)
    terminate = False
    truncated = False
    action_last = [0,0,0,0,0]
    while not terminate or not truncated:
        # model.learn(total_timesteps=100000, reset_num_timesteps=False, callback=callback)
        start = time.time()
        action, _state = model.predict(obs, deterministic=True)
        end = time.time()
        e_time = end - start
        if obs[-1]:
            for i in range(len(action)):
                action[i] = first_order_filter(action[i], action_last[i], dt, tau)
        # action = first_order_filter(action, action_last, dt, tau)
        obs, reward, terminate, truncated, info = env_evalutor.step(action)
        action_last = action
        # done = terminate or truncated
        writer.add_scalar('Vx', obs[0], t)
        writer.add_scalar('Vy', obs[1], t)
        writer.add_scalar('Yaw', obs[2], t)
        writer.add_scalar('S_fl', obs[3], t)
        writer.add_scalar('S_fr', obs[4], t)
        writer.add_scalar('S_rl', obs[5], t)
        writer.add_scalar('S_rr', obs[6], t)
        writer.add_scalar('V_target', obs[0] - obs[9], t)
        writer.add_scalar('info/X_position', info["X_position"], t)
        writer.add_scalar('info/Y_position', info["Y_position"], t)
        writer.add_scalar('info/T_fl', info["T_fl"], t)
        writer.add_scalar('info/T_fr', info["T_fr"], t)
        writer.add_scalar('info/T_rl', info["T_rl"], t)
        writer.add_scalar('info/T_rr', info["T_rr"], t)
        writer.add_scalar('info/delta_f', info["delta_f"], t)
        writer.add_scalar('info/reward', reward, t)


        t += 1

    print("evaluation is done!")

