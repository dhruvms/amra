# Generates a 50x50 map with a 10-unit thick cul-de-sac

import random

ysize = 100 # height
xsize = 100 # width

file = open("../dat/uav_obs.map", "w")
file.write("type octile\n")
file.write("height " + str(ysize) + "\n")
file.write("width " + str(xsize) + "\n")
file.write("map\n")

for y in range(ysize):
    for x in range(xsize):
        if random.randrange(40) == 5:
            file.write("T")
        else:
            file.write(".")
    file.write("\n")

file.close()