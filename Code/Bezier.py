import numpy as np

def B_nx(n, i, x):
    if i > n:
        return 0
    elif i == 0:
        return (1-x)**n
    elif i == 1:
        return n*x*((1-x)**(n-1))
    return B_nx(n-1, i, x)*(1-x) + B_nx(n-1, i-1, x)*x

def get_value(p, para):
    sumx = 0.
    sumy = 0.
    length = len(p)-1
    for i in range(0, len(p)):
        sumx += (B_nx(length, i, para) * p[i][0])
        sumy += (B_nx(length, i, para) * p[i][1])
    return sumx, sumy

def get_newxy(p,x):
    xx = [0] * len(x)
    yy = [0] * len(x)
    for i in range(0, len(x)):
        a, b = get_value(p, x[i])
        xx[i] = a
        yy[i] = b
    return xx, yy

def bezier(phase1_obj):
    if phase1_obj[0] < 2:
        p0 = [1.6, -20.5]
        p3_1 = [-20.9, 8] 
        p3_2 = [-20.9, 4.8] 
        p3_3 = [-20.9, 1.6] 
        p1_1 = [p0[0],p0[1] + 21]
        p1_2 = [p0[0],p0[1] + 21]
        p1_3 = [p0[0],p0[1] + 12]
        p2_1 = [p3_1[0] + 19, p3_1[1]]
        p2_2 = [p3_2[0] + 19, p3_2[1]]
        p2_3 = [p3_3[0] + 12, p3_3[1]]
        P1 = np.array([p0,p1_1,p2_1,p3_1])
        P2 = np.array([p0,p1_2,p2_2,p3_2])
        P3 = np.array([p0,p1_3,p2_3,p3_3])
        x = np.linspace(0, 1, 2000)
        xx1, yy1 = get_newxy(P1, x)
        xx2, yy2 = get_newxy(P2, x)
        xx3, yy3 = get_newxy(P3, x)
        Bezier = [[xx1,yy1],[xx2,yy2],[xx3,yy3]]
    elif phase1_obj[0] > 12:
        p0 = [11.2, -20.5]
        p3_1 = [20.9, -8] 
        p3_2 = [20.9, -4.8] 
        p3_3 = [20.9, -1.6] 
        p1_1 = [p0[0],p0[1] + 30]
        p1_2 = [p0[0],p0[1] + 30]
        p1_3 = [p0[0],p0[1] + 30]
        p2_1 = [p3_1[0] + 28.5, p3_1[1]]
        p2_2 = [p3_2[0] + 28.5, p3_2[1]]
        p2_3 = [p3_3[0] + 12.5, p3_3[1]]
        P1 = np.array([p0,p1_1,p2_1,p3_1])
        P2 = np.array([p0,p1_2,p2_2,p3_2])
        P3 = np.array([p0,p1_3,p2_3,p3_3])
        x = np.linspace(0, 1, 2000)
        xx1, yy1 = get_newxy(P1, x)
        xx2, yy2 = get_newxy(P2, x)
        xx3, yy3 = get_newxy(P3, x)
        Bezier = [[xx1,yy1],[xx2,yy2],[xx3,yy3]]
    else:
        p0 = [phase1_obj, -20.5]
        p3_1 = [1.6, 20.9] 
        p3_2 = [4.8, 20.9] 
        p3_3 = [8, 20.9] 
        p1_1 = [p0[0],p0[1] + 30]
        p1_2 = [p0[0],p0[1] + 30]
        p1_3 = [p0[0],p0[1] + 20]
        p2_1 = [p3_1[0] + 28.5, p3_1[1]]
        p2_2 = [p3_2[0] + 28.5, p3_2[1]]
        p2_3 = [p3_3[0] + 8.5, p3_3[1]]
        P1 = np.array([p0,p1_1,p2_1,p3_1])
        P2 = np.array([p0,p1_2,p2_2,p3_2])
        P3 = np.array([p0,p1_3,p2_3,p3_3])
        x = np.linspace(0, 1, 2000)
        xx1, yy1 = get_newxy(P1, x)
        xx2, yy2 = get_newxy(P2, x)
        xx3, yy3 = get_newxy(P3, x)
        Bezier = [[xx1,yy1],[xx2,yy2],[xx3,yy3]]

    return Bezier

def find_closest(lst, target):
    closest = 0
    for i in range(len(lst)):
        if abs(lst[i] - target) < abs(lst[closest] - target):
            closest = i
    return closest