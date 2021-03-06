# Generates a 50x50 map with a 10-unit thick cul-de-sac

ysize = 50 # height
xsize = 50 # width

file = open("culdesac.map", "w")
file.write("type octile\n")
file.write("height " + str(ysize) + "\n")
file.write("width " + str(xsize) + "\n")
file.write("map\n")

# obstacle cell coordinates
# x (right), y (down)
obs1 = set((x,y) for x in range(10,30) for y in range(10,20))
obs2 = set((x,y) for x in range(10,30) for y in range(30,40))
obs3 = set((x,y) for x in range(30,40) for y in range(10,40))
obstacles = obs1 | obs2 | obs3

for y in range(ysize):
    for x in range(xsize):
        if (x,y) in obstacles:
            file.write("T")
        else:
            file.write(".")
    file.write("\n")

file.close()