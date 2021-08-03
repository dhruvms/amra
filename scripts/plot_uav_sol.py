import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

plt.rcParams.update({"text.usetex": True})
plt.rc('font', family='serif')

PLOT_THETA = 0

xsize = 0
ysize = 0

# READ MAP
with open('../dat/culdesac_np_large.map') as f:
    line = f.readline()
    line = f.readline()
    xsize = int(line.split(' ')[1])
    line = f.readline()
    ysize = int(line.split(' ')[1])
    line = f.readline()
    mapdata = np.array([list(line.rstrip()) for line in f])

mapdata.reshape((xsize,ysize))
mapdata[mapdata == '.'] = 0
mapdata[mapdata == 'T'] = 1
mapdata = mapdata.astype(int)

all_solutions = []
all_expansions = []
iterations = []

done = False
with open('../dat/solutions/uavsol.txt') as f:
    while True:
        if done:
            break
        line = f.readline()[:-1]  # discard newline character
        if line == "":
            break
        field = line.split(',')[0]
        if field == "iter":
            iteration = int(line.split(',')[1])
            iterations.append(iteration)
        elif field == "solstart":
            current_solution = []
            while True:
                line = f.readline()[:-1]  # discard newline character
                field = line.split(',')[0]
                if field == "solend":
                    all_solutions.append(current_solution)
                    break
                state = [float(x) for x in line.split(',')]
                current_solution.append(state)
            continue

expdf = pd.read_csv("../dat/solutions/uavexp.txt")
exp_iters = expdf["iter"]
exp_hidxs = expdf["hidx"]
exp_Xs = expdf["x"]
exp_Ys = expdf["y"]

# DRAW
figure_rows = len(iterations)
fig, axs = plt.subplots(ncols=4, nrows=figure_rows)
spec = gridspec.GridSpec(ncols=4, nrows=figure_rows, figure=fig, hspace=0, wspace=0)

for i in range(figure_rows):

    # fig, axs = plt.subplots(1, 4)

    sol = all_solutions[i]

    X = [s[0] for s in sol]
    Y = [s[1] for s in sol]
    Theta = [s[2] for s in sol]
    Vels = [s[3] for s in sol]

    # TRAJECTORY
    # ax = plt.subplot(gs1[i,0])
    ax = axs[i,0]
    traj = ax.scatter(Y, X, s=0.2, c=Vels)
    start = ax.scatter(Y[0], X[0], s=20, c='r')
    goal = ax.scatter(Y[len(Y)-1], X[len(X)-1], s=20, c='c')

    ax.imshow(mapdata, cmap='Greys')
    # ax.grid(color='Grey', linestyle='-', linewidth=0.1)
    ax.tick_params(axis='both', which='major', labelsize=4)
    ax.tick_params(axis='both', which='minor', labelsize=0)

    # ANCHOR EXPANSIONS
    df = expdf[(expdf["iter"] == i) & (expdf["hidx"] == 0)]
    X = df["x"]
    Y = df["y"]
    # Th = df["theta"]

    # ax = plt.subplot(gs1[i,1])
    ax = axs[i,1]
    ax.imshow(mapdata, cmap='Greys')
    # ax.grid(color='Grey', linestyle='-', linewidth=0.1)
    exps = ax.scatter(Y, X, s=1, alpha=0.1, color='b')
    ax.tick_params(axis='both', which='major', labelsize=3)
    ax.tick_params(axis='both', which='minor', labelsize=0)
    ax.set_title(r'Anchor', fontsize=8)

    # HEUR 1 EXPANSIONS
    df = expdf[(expdf["iter"] == i) & (expdf["hidx"] == 1)]
    X = df["x"]
    Y = df["y"]

    # ax = plt.subplot(gs1[i,2])
    ax = axs[i,2]
    ax.imshow(mapdata, cmap='Greys')
    # ax.grid(color='Grey', linestyle='-', linewidth=0.1)
    exps = ax.scatter(Y, X, s=1, alpha=0.1, color='b')
    ax.tick_params(axis='both', which='major', labelsize=3)
    ax.tick_params(axis='both', which='minor', labelsize=0)
    ax.set_title(r'3m res.', fontsize=8)

    # HEUR 2 EXPANSIONS
    df = expdf[(expdf["iter"] == i) & (expdf["hidx"] == 2)]
    X = df["x"]
    Y = df["y"]

    # ax = plt.subplot(gs1[i,3])
    ax = axs[i,3]
    ax.imshow(mapdata, cmap='Greys')
    # ax.grid(color='Grey', linestyle='-', linewidth=0.1)
    exps = ax.scatter(Y, X, s=1, alpha=0.1, color='b')
    ax.tick_params(axis='both', which='major', labelsize=3)
    ax.tick_params(axis='both', which='minor', labelsize=0)
    ax.set_title(r'9m res.', fontsize=8)

plt.show()
# filename = "solution_iter_" + str(i) + ".pdf"
# plt.savefig(filename, dpi=600)
