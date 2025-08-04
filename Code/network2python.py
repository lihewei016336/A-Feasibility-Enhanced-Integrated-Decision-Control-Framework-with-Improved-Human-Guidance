import numpy as np
import pandas as pd
from openpyxl import load_workbook
import matplotlib.pyplot as plt

def my_neural_network_function(X):
    # ===== NEURAL NETWORK CONSTANTS =====

    # Input 1
    x1_step1_remove = 1
    x1_step1_keep = [1, 2]
    x1_step2_xoffset = np.array([[15], [0]])
    x1_step2_gain = np.array([[0.111111111111111], [0.4]])
    x1_step2_ymin = -1

    # Layer 1
    b1 = np.array([-4.0430253768072761, -4.2752841626461917, -2.6332820185384769, -0.43699298764036232, -2.9191293883861595, -0.081412735213707066, -1.7378900824658148, 2.4748622811281678, -4.3345332775779664, 4.3092910068040737])
    IW1_1 = np.array([[4.5596780582649243, -2.505145330237958], [3.5298202299007833, 1.3427093231660194], [4.1499181913143426, -1.1270208184038757], [0.78633345441066038, -5.2475736438457989], [5.8486272440303857, 5.3633695776915422], [3.148940463423302, 0.7510487357725647], [-0.80155004609409142, 3.4126523165568039], [3.8636994828788311, 0.83742186918867423], [-2.428250615529032, 1.4673833440767139], [4.242391019649788, 0.26801928747897041]])

    # Layer 2
    b2 = -0.37193700866478213
    LW2_1 = np.array([0.025226386000329216, -0.094040173657705062, 0.013611903565707732, 0.066910703669057284, -0.42881694357983779, -0.12712518924794489, -0.057717610563586708, -0.12408189784800323, -0.42618993788815729, -0.19636470492496783])

    # Output 1
    y1_step1_ymin = -1
    y1_step1_gain = 18.348623853211
    y1_step1_xoffset = 0.036

    # ===== SIMULATION ========

    # Format Input Arguments
    if isinstance(X, np.ndarray):
        X = [X]

    # Dimensions
    TS = len(X)  # timesteps
    if len(X) > 0:
        Q = X[0].shape[1]  # samples/series
    else:
        Q = 0

    # Allocate Outputs
    Y = [None] * TS

    # Time loop
    for ts in range(TS):

        # Input 1
        temp = remove_constant_rows_apply(X[ts], x1_step1_keep, x1_step1_remove)
        Xp1 = map_min_max_apply(temp, x1_step2_gain, x1_step2_xoffset, x1_step2_ymin)

        # Layer 1
        a1 = tansig_apply(np.matmul(IW1_1, Xp1) + np.tile(b1, (1, Q)).reshape(-1, 1))

        # Layer 2
        a2 = np.matmul(LW2_1, a1) + np.tile(b2, (1, Q))

        # Output 1
        Y[ts] = map_min_max_reverse(a2, y1_step1_gain, y1_step1_xoffset, y1_step1_ymin)

    return Y[0][0][0]

# ===== MODULE FUNCTIONS ========

# Map Minimum and Maximum Input Processing Function
def map_min_max_apply(x, settings_gain, settings_xoffset, settings_ymin):
    y = np.subtract(x, settings_xoffset)
    y = np.multiply(y, settings_gain)
    y = np.add(y, settings_ymin)
    return y

# Remove Constants Input Processing Function
def remove_constant_rows_apply(x, settings_keep, settings_remove):
    if settings_remove is None:
        return x
    else:
        return x[settings_keep, :]

# Sigmoid Symmetric Transfer Function
def tansig_apply(n):
    return 2 / (1 + np.exp(-2 * n)) - 1

# Map Minimum and Maximum Output Reverse-Processing Function
def map_min_max_reverse(y, settings_gain, settings_xoffset, settings_ymin):
    x = np.subtract(y, settings_ymin)
    x = np.divide(x, settings_gain)
    x = np.add(x, settings_xoffset)
    return x

#plot slip ratio for different delta_f 0~5,Vx 15~34
def main():
    # mu = 0.8
    # x = np.linspace(15, 34, 100)
    # for delta_f in np.arange(6):
    #     y = []
    #     for Vx in x:
    #         slip = slipration(mu, Vx, delta_f)
    #     y.append(slip)
    #     plt.plot(x, y, label='delta_f = {}'.format(delta_f))
    # plt.xlabel('Vx (m/s)')
    # plt.ylabel('Slip ratio')
    # plt.legend()
    # plt.show()
    # 准备输入数据
    # TS = 3  # 时间步数
    # Q = 1  # 样本数量

    # # 创建一个大小为 (1, TS) 的单元格数组 X
    # X = [[]]
    # for ts in range(TS):
    #     # 创建一个大小为 (3, Q) 的输入矩阵 Xp
    #     Xp = np.array([[0.8], [25], [2]])  # 这里使用提供的示例数据
    #     X[0].append(Xp)
    # # 将 X 传递给 my_neural_network_function 函数
    mu_range = np.arange(0.1, 1, 0.1)
    Vx_range = np.arange(16, 33.5, 0.5)
    deltaf_range = np.arange(0, 5.5, 0.5)
    # Y_mu = []
    result = []
    for i in range(len(mu_range)):
        # Y_deltaf = []
        for j in range(len(Vx_range)):
            # Y_deltaf = []
            for k in range(len(deltaf_range)):
                Y = []
                inputdata = [[mu_range[i]], [Vx_range[j]], [deltaf_range[k]]]
                inputdata = np.array(inputdata)
                y = my_neural_network_function(inputdata) / 0.8 * mu_range[i]
                Y = [Vx_range[j], deltaf_range[k], mu_range[i], (y - 0.08) / 0.08]
                result.append(Y)
        #         Y_deltaf.append(y)
        #     Y_deltaf.append(Y_deltaf)
        # Y_mu.append(Y_deltaf)

    df = pd.DataFrame(result, columns=['Vx', 'deltaf', 'mu', 'slipratio'])
    df.to_excel('result.xlsx')

    # colums = ['deltaf='+str(i) for i in deltaf_range]
    # index = Vx_range
    # df = pd.DataFrame()
    # df.to_excel('expert.xlsx')
    # for g in range(len(Y_mu)):
    #     df = pd.DataFrame(np.array(Y_mu[g]).T, index=index, columns=colums)
    #     with pd.ExcelWriter('expert.xlsx', engine='openpyxl', mode='a') as writer:
    #         df.to_excel(writer, sheet_name='mu='+str(mu_range[g]))

    # #将文件expert.xlsx中的数据读出来，重新排列
    # slipratio = np.array([])
    # all_sheet = pd.read_excel('expert.xlsx', sheet_name=None, index_col=0)
    # for sheet_name, data in all_sheet.items():
    #     slipratio = np.append(slipratio, data.values.flatten())
    
    # with pd.ExcelWriter('expert.xlsx', engine='openpyxl', mode='a') as writer:
    #     df = pd.DataFrame(slipratio)
    #     df.to_excel(writer, sheet_name='slip')
        
    #     X = np.arange(16, 33, 0.5)
    #     plt.plot(X, Y, label='delta_f = {}'.format(delta_f))
    #     Y = []
    # plt.xlabel('Vx (m/s)')
    # plt.ylabel('Slip ratio')
    # plt.legend()
    # plt.show()

if __name__ == '__main__':  
    main()