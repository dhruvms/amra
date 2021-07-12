# Generates a 50x50 map with a 10-unit thick cul-de-sac
import numpy as np
import matplotlib.pyplot as plt

ysize = 200 # height
xsize = 200 # width

mapfile = "../dat/culdesac_np_large.map"
file = open(mapfile, "w")
file.write("type octile\n")
file.write("height " + str(ysize) + "\n")
file.write("width " + str(xsize) + "\n")
file.write("map\n")

# obstacle cell coordinates
# x (right), y (down)
obs1 = set((x,y) for x in range(50,150) for y in range(50,70))
obs2 = set((x,y) for x in range(130,150) for y in range(70,140))
obs3 = set((x,y) for x in range(50,150) for y in range(140,160))
obstacles = obs1 | obs2 | obs3

for x in range(80,85):
    for y in range(50,70):
        obstacles.remove((x,y))

for y in range(ysize):
    for x in range(xsize):
        if (x,y) in obstacles:
            file.write("T")
        else:
            file.write(".")
    file.write("\n")
file.close()

with open(mapfile) as f:
    line = f.readline()
    line = f.readline()
    line = f.readline()
    line = f.readline()
    mapdata = np.array([list(line.rstrip()) for line in f])

mapdata.reshape((xsize,ysize))
mapdata[mapdata == '.'] = 0
mapdata[mapdata == 'T'] = 1
mapdata = mapdata.astype(int)

plt.imshow(mapdata, cmap='Greys')
plt.show()