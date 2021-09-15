import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

plt.rcParams.update({"text.usetex": True})
plt.rc('font', family='serif')

PLOT_THETA = 0
IMG_DIR = '../dat/imgs/'

xsize = 0
ysize = 0

# READ MAP
with open('../dat/Boston_0_1024.map') as f:
    line = f.readline()
    line = f.readline()
    xsize = int(line.split(' ')[1])
    line = f.readline()
    ysize = int(line.split(' ')[1])
    line = f.readline()
    mapdata = np.array([list(line.rstrip()) for line in f])

mapdata.reshape((xsize,ysize))
mapdata[mapdata == '.'] = 0
mapdata[mapdata == '@'] = 1
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
# fig, axs = plt.subplots(ncols=5, nrows=figure_rows)
# spec = gridspec.GridSpec(ncols=5, nrows=figure_rows, figure=fig, hspace=0, wspace=0)
fig = plt.figure(figsize=(10,10))
ax = plt.gca()

for i in range(figure_rows):

    # fig, axs = plt.subplots(1, 4)

    sol = all_solutions[i]

    X = [s[0] for s in sol]
    Y = [s[1] for s in sol]
    Theta = [s[2] for s in sol]
    Vels = [s[3] for s in sol]

    # TRAJECTORY
    # ax = plt.subplot(gs1[i,0])
    # ax = axs[i,0]
    traj = ax.scatter(Y, X, s=20, c=Vels, cmap='plasma')
    start = ax.scatter(Y[0], X[0], s=40, c='g')
    goal = ax.scatter(Y[len(Y)-1], X[len(X)-1], s=40, c='r')

    ax.imshow(mapdata, cmap='Greys')
    # ax.grid(color='Grey', linestyle='-', linewidth=0.1)
    ax.tick_params(axis='both', which='major', labelsize=4)
    ax.tick_params(axis='both', which='minor', labelsize=0)
    plt.savefig(IMG_DIR + str(i) + '_0_traj.png', bbox_inches='tight')
    plt.cla()

    # ANCHOR EXPANSIONS
    df = expdf[(expdf["iter"] == i) & (expdf["hidx"] == 0)]
    X = df["x"]
    Y = df["y"]
    # Th = df["theta"]

    # ax = plt.subplot(gs1[i,1])
    # ax = axs[i,1]
    ax.imshow(mapdata, cmap='Greys')
    # ax.grid(color='Grey', linestyle='-', linewidth=0.1)
    exps = ax.scatter(Y, X, s=5, alpha=0.25, color='b')
    ax.tick_params(axis='both', which='major', labelsize=3)
    ax.tick_params(axis='both', which='minor', labelsize=0)
    ax.set_title(r'Anchor', fontsize=8)
    plt.savefig(IMG_DIR + str(i) + '_1_ANCHOR.png', bbox_inches='tight')
    plt.cla()

    # HEUR 1 EXPANSIONS
    df = expdf[(expdf["iter"] == i) & (expdf["hidx"] == 1)]
    X = df["x"]
    Y = df["y"]

    # ax = plt.subplot(gs1[i,2])
    # ax = axs[i,2]
    ax.imshow(mapdata, cmap='Greys')
    # ax.grid(color='Grey', linestyle='-', linewidth=0.1)
    exps = ax.scatter(Y, X, s=5, alpha=0.25, color='b')
    ax.tick_params(axis='both', which='major', labelsize=3)
    ax.tick_params(axis='both', which='minor', labelsize=0)
    ax.set_title(r'3m res.', fontsize=8)
    plt.savefig(IMG_DIR + str(i) + '_2_3m.png', bbox_inches='tight')
    plt.cla()

    # HEUR 2 EXPANSIONS
    df = expdf[(expdf["iter"] == i) & (expdf["hidx"] == 2)]
    X = df["x"]
    Y = df["y"]

    # ax = plt.subplot(gs1[i,3])
    # ax = axs[i,3]
    ax.imshow(mapdata, cmap='Greys')
    # ax.grid(color='Grey', linestyle='-', linewidth=0.1)
    exps = ax.scatter(Y, X, s=5, alpha=0.25, color='b')
    ax.tick_params(axis='both', which='major', labelsize=3)
    ax.tick_params(axis='both', which='minor', labelsize=0)
    ax.set_title(r'9m res.', fontsize=8)
    plt.savefig(IMG_DIR + str(i) + '_3_9m.png', bbox_inches='tight')
    plt.cla()

    # HEUR 3 EXPANSIONS
    df = expdf[(expdf["iter"] == i) & (expdf["hidx"] == 3)]
    X = df["x"]
    Y = df["y"]

    # ax = plt.subplot(gs1[i,3])
    # ax = axs[i,4]
    ax.imshow(mapdata, cmap='Greys')
    # ax.grid(color='Grey', linestyle='-', linewidth=0.1)
    exps = ax.scatter(Y, X, s=5, alpha=0.25, color='b')
    ax.tick_params(axis='both', which='major', labelsize=3)
    ax.tick_params(axis='both', which='minor', labelsize=0)
    ax.set_title(r'Dijkstra.', fontsize=8)
    plt.savefig(IMG_DIR + str(i) + '_4_Dijkstra.png', bbox_inches='tight')
    plt.cla()

# plt.show()
# filename = "solution_iter_" + str(i) + ".pdf"
# plt.savefig(filename, dpi=600)
