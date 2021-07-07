import numpy as np
import pandas as pd
import matplotlib as mpl
import matplotlib.pyplot as plt

# READ MAP
# with open('../dat/testmap.map') as f:
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

# READ UAV TRAJECTORY
solution = pd.read_csv("../dat/solutions/uavsol.txt", header=None)
X = solution[0]
Y = solution[1]
Theta = solution[2]
Vels = solution[3]

# DRAW
fig, (ax1, ax2) = plt.subplots(1,2)
col_map = plt.get_cmap('RdBu_r')

# map
ax1.imshow(mapdata, cmap='Greys')
ax1.grid(color='Grey', linestyle='-', linewidth=0.1)
# ax1.set_xticks(np.arange(0, 100, 1))
# ax1.set_yticks(np.arange(0, 100, 1))
# ax1.set_xticklabels(np.arange(0, 100, 1))
# ax1.set_yticklabels(np.arange(0, 100, 1))

# trajectory
traj = ax1.scatter(Y, X, s=0.2, c=Vels, cmap=col_map)
start = ax1.scatter(Y[0], X[0], s=20, c='r')
goal = ax1.scatter(Y[len(Y)-1], X[len(X)-1], s=20, c='c')

# legend, colorbar
plt.legend([start, goal], ["Start", "Goal"])
fig.colorbar(traj, ax=ax1)

ax2.plot(range(0, len(Theta)), Theta)

plt.show()
