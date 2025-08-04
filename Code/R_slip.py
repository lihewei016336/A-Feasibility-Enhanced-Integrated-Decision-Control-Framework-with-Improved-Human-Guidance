import torch
import torch.nn as nn
import numpy as np

def normalize(x, lower_bound = np.array([16, 0, 0.1]), upper_bound = np.array([33, 5, 0.9])):
    x_normalize = (np.array(x)-lower_bound) / (upper_bound - lower_bound)
    return x_normalize


class Reward_Model(nn.Module):
    """ 
        input_dim的维度数是state_dim + action_dim

        output_dim一般默认为1维

    """
    def __init__(self, input_dim = 4, hidden0=128, hidden1 = 128, hidden2 = 128, output_dim = 1):
        super(Reward_Model, self).__init__()
        # torch.manual_seed(2023)
        self.func0 = nn.Linear(input_dim, hidden0)
        nn.init.normal_(self.func0.weight, 0, 0.5)
        nn.init.constant_(self.func0.bias, 0.1)

        self.func1 = nn.Linear(hidden0, hidden1)
        nn.init.normal_(self.func1.weight, 0, 0.5)
        nn.init.constant_(self.func1.bias, 0.1)

        self.func2 = nn.Linear(hidden1, hidden2)
        nn.init.normal_(self.func2.weight, 0, 0.5)
        nn.init.constant_(self.func2.bias, 0.1)

        self.func3 = nn.Linear(hidden2, output_dim, bias=False)
        nn.init.normal_(self.func3.weight, 0, 0.5)

    def forward(self, x):
        x = nn.functional.tanh(self.func0(x))
        x = nn.functional.relu(self.func1(x))   
        x = nn.functional.relu(self.func2(x))
        reward = self.func3(x)

        return reward 
    

def r_slip(vx: float, delta_f: float, slip: list, mu: float = 0.8) -> float:
    '''
        the input value is raw
    '''
    state = [vx, abs(delta_f), mu]
    state_nor = normalize(state)
    state_nor = np.repeat(np.expand_dims(state_nor,0), len(slip), 0)
    s = np.expand_dims(np.array(slip)/0.08 - 1, 1)#from[0,0.16]to[-1,1]
    r_input = np.append(state_nor, s, 1)
    r_input = torch.FloatTensor(r_input)
    reward = Reward_Model(hidden0=32, hidden1=16, hidden2=8)
    reward.load_state_dict(torch.load('C:\\Users\\Dell\\Desktop\\New_final\\rewards.pth'))
    with torch.no_grad():
        reward_value = reward(r_input)
    return np.sum(reward_value.numpy())