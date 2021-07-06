import numpy as np
import pandas as pd
import matplotlib as mpl
import matplotlib.pyplot as plt

# READ MAP
with open('../dat/uav_obs.map') as f:
# with open('../dat/uav_obs_rand.map') as f:
    lines = f.readline()
    lines = f.readline()
    lines = f.readline()
    lines = f.readline()
    mapdata = np.array([list(line.rstrip()) for line in f])
mapdata.reshape((100,100))
mapdata[mapdata == '.'] = 0
mapdata[mapdata == 'T'] = 1
mapdata = mapdata.astype(int)
print(mapdata[0])

# READ UAV TRAJECTORY
solution = pd.read_csv("../dat/uavsol.txt", header=None)
X = solution[0]
Y = solution[1]
Theta = solution[2]
Vels = solution[3]

# DRAW
fig = plt.figure()
ax = plt.gca()
col_map = plt.get_cmap('RdBu_r')

# map
ax.imshow(mapdata, cmap='Greys')

# trajectory
traj = ax.scatter(Y, X, s=0.1, c=Vels, cmap=col_map)
start = ax.scatter(Y[0], X[0], s=20, c='r')
goal = ax.scatter(Y[len(Y)-1], X[len(X)-1], s=20, c='c')

# legend, colorbar
plt.legend([start, goal], ["Start", "Goal"])
fig.colorbar(traj, ax=ax)

plt.show()
