import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

# READ MAP
with open('../dat/uav_obs.map') as f:
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
ax.scatter(X,Y, s=0.5, c='red')
ax.imshow(mapdata)
plt.show()