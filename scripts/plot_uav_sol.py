import numpy as np
import pandas as pd
import matplotlib as mpl
import matplotlib.pyplot as plt

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

# READ SOLUTIONS
# solution = pd.read_csv("../dat/solutions/uavsol.txt", header=None)
# X = solution[0]
# Y = solution[1]
# Theta = solution[2]
# Vels = solution[3]

all_solutions = []
all_expansions = []
iterations = []

done = False
with open('../dat/solutions/uavsol.txt') as f:
    while True:
        if done:
            break
        line = f.readline()[:-1]  # discard newline character
        print(line)
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

# exp_df = pd.read_csv("../dat/solutions/uavexp.txt")
# exp_iters = exp_df[0]
# exp_hidxs = exp_df[1]

# # READ EXPANSIONS
# exp_hidxs = expansions[0]
# index = expansions.index
# condition = expansions[0] == 0
# hidx_0_indices = index[condition]
# hidx_0_X = expansions.iloc[hidx_0_indices, 1]
# hidx_0_Y = expansions.iloc[hidx_0_indices, 2]

# condition = expansions[0] == 1
# hidx_1_indices = index[condition]
# hidx_1_X = expansions.iloc[hidx_1_indices, 1]
# hidx_1_Y = expansions.iloc[hidx_1_indices, 2]

# condition = expansions[0] == 2
# hidx_2_indices = index[condition]
# hidx_2_X = expansions.iloc[hidx_2_indices, 1]
# hidx_2_Y = expansions.iloc[hidx_2_indices, 2]

# condition = expansions[0] == 3
# hidx_3_indices = index[condition]
# hidx_3_X = expansions.iloc[hidx_3_indices, 1]
# hidx_3_Y = expansions.iloc[hidx_3_indices, 2]

# DRAW

fig, axs = plt.subplots(len(iterations), 1)
col_map = plt.get_cmap('RdBu_r')

for i in range(len(iterations)):
    ax = axs[i]
    sol = all_solutions[i]

    X = [s[0] for s in sol]
    Y = [s[1] for s in sol]
    Theta = [s[2] for s in sol]
    Vels = [s[3] for s in sol]

    # traj = ax1.scatter(Y, X, s=0.2, c=Vels, cmap=col_map)
    traj = ax.scatter(Y, X, s=0.2, c=Vels)
    start = ax.scatter(Y[0], X[0], s=20, c='r')
    goal = ax.scatter(Y[len(Y)-1], X[len(X)-1], s=20, c='c')

    ax.imshow(mapdata, cmap='Greys')
    ax.grid(color='Grey', linestyle='-', linewidth=0.1)

plt.show()

# # map
# ax1.imshow(mapdata, cmap='Greys')
# ax1.grid(color='Grey', linestyle='-', linewidth=0.1)

# # trajectory
# traj = ax1.scatter(Y, X, s=0.2, c=Vels, cmap=col_map)
# start = ax1.scatter(Y[0], X[0], s=20, c='r')
# goal = ax1.scatter(Y[len(Y)-1], X[len(X)-1], s=20, c='c')
# ax1.legend([start, goal], ["Start", "Goal"])
# # legend, colorbar
# # fig.colorbar(traj, ax=ax1)

# # PLOT EXPANSIONS
# ax2.imshow(mapdata, cmap='Greys')
# ax2.grid(color='Grey', linestyle='-', linewidth=0.1)
# exps = ax2.scatter(hidx_0_Y, hidx_0_X, s=10, c='b')
# ax2.set_title('ANCHOR')

# # ax3.imshow(mapdata, cmap='Greys')
# # ax3.grid(color='Grey', linestyle='-', linewidth=0.1)
# # exps = ax3.scatter(hidx_1_Y, hidx_1_X, s=10, c='r')
# # ax3.set_title('3m resolution')

# ax3.imshow(mapdata, cmap='Greys')
# ax3.grid(color='Grey', linestyle='-', linewidth=0.1)
# exps = ax3.scatter(hidx_2_Y, hidx_2_X, s=10, c='r')
# ax3.set_title('3m resolution')

# ax4.imshow(mapdata, cmap='Greys')
# ax4.grid(color='Grey', linestyle='-', linewidth=0.1)
# exps = ax4.scatter(hidx_3_Y, hidx_3_X, s=10, c='g')
# ax4.set_title('9m resolution')


# if PLOT_THETA:

#     fig, (ax1, ax2) = plt.subplots(1,2)
#     col_map = plt.get_cmap('RdBu_r')

#     # map
#     ax1.imshow(mapdata, cmap='Greys')
#     ax1.grid(color='Grey', linestyle='-', linewidth=0.1)
#     # ax1.set_xticks(np.arange(0, 100, 1))
#     # ax1.set_yticks(np.arange(0, 100, 1))
#     # ax1.set_xticklabels(np.arange(0, 100, 1))
#     # ax1.set_yticklabels(np.arange(0, 100, 1))

#     # trajectory
#     traj = ax1.scatter(Y, X, s=0.2, c=Vels, cmap=col_map)
#     start = ax1.scatter(Y[0], X[0], s=20, c='r')
#     goal = ax1.scatter(Y[len(Y)-1], X[len(X)-1], s=20, c='c')

#     # legend, colorbar
#     plt.legend([start, goal], ["Start", "Goal"])
#     fig.colorbar(traj, ax=ax1)

#     ax2.plot(range(0, len(Theta)), Theta)

#     plt.show()