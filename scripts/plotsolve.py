import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

import os
import sys

def two_scales(ax1, time, data1, data2, c1, c2):
	ax2 = ax1.twinx()
	ax1.plot(time, data1, color=c1, label='f-values')
	ax1.set_xlabel('path id')
	ax1.set_ylabel('f-values')
	ax2.plot(time, data2, color=c2, label='expansion delay')
	ax2.set_ylabel('expansion delay')
	return ax1, ax2


EXPS_DIR = '../dat/expansions/'
SOL_DIR = '../dat/solutions/'
IMG_DIR = '../dat/imgs/'

# map must be input through command line
MAP = sys.argv[1]
MAP = MAP.split('/')[-1].split('.')[0]

# start and goal states may be input through command line
S = None
G = None
if (len(sys.argv) > 2):
	S = np.array([int(sys.argv[2]), int(sys.argv[3])])
	G = np.array([int(sys.argv[4]), int(sys.argv[5])])

nrows = ncols = -1
for f in os.listdir(EXPS_DIR):
	if (f == '.gitignore'):
		continue

	fields = f.split('_')

	iters = int(fields[0])
	queue = int(fields[1])

	if (iters > nrows):
		nrows = iters
	if (queue > ncols):
		ncols = queue

nrows += 1
ncols += 1

qnames = {
	0: 'anchor',
	1: 'high res',
	2: 'mid res',
	3: 'low res',
	}

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10,10))

movingai = False
flipped = False
for f in os.listdir(EXPS_DIR):
	if (f == '.gitignore'):
		continue

	fields = f.split('_')
	iters = int(fields[0])
	queue = int(fields[1])

	# read expansions and adjust scale
	E = np.genfromtxt(EXPS_DIR + f, delimiter=',')
	expansions = None
	if 'costs' in MAP:
		E = E / 10
	if 'Cauldron' in MAP or 'TheFrozenSea' in MAP:
		if not movingai:
			movingai = True

		# E[E >= 1000] = 10
		E[E == 0] = -2
		expansions = np.argwhere(E == 1003)
		E = E.transpose()

	# plot start and goal if input through command line
	if S is not None and G is not None and movingai and not flipped:
		S = np.flipud(S)
		G = np.flipud(G)
		flipped = True
	if S is not None:
		ax1.scatter(S[0], S[1], s=10, c='g', zorder=10)
	if G is not None:
		ax1.scatter(G[0], G[1], s=10, c='r', zorder=10, marker='*')

	# explicitly plot expansions for movingai maps
	if movingai and expansions is not None:
		ax1.scatter(expansions[:, 1], expansions[:, 0], s=5, c='b')

	# plot solution path
	P = np.genfromtxt(SOL_DIR + MAP + '_path.map', delimiter=',')
	if movingai:
		P[:, [0, 1]] = P[:, [1, 0]]
	ax1.plot(P[:, 0], P[:, 1], 'salmon', lw=1, alpha=1.0)

	# plot arrows along solution path
	N = 10
	step = max(P.shape[0]//N, 1)
	for p in range(1, P.shape[0], step):
		dx = min(abs(P[p, 0]-P[p-1, 0]), 0.1) * np.sign(P[p, 0]-P[p-1, 0])
		dy = min(abs(P[p, 1]-P[p-1, 1]), 0.1) * np.sign(P[p, 1]-P[p-1, 1])
		ax1.arrow(P[p-1, 0]+dx, P[p-1, 1]+dy, dx, dy, shape='full', lw=0.5, length_includes_head=True, head_width=0.5+movingai*10, zorder=8, head_starts_at_zero=True, facecolor='yellow')

	# display map
	im = None
	if 'costs' in MAP:
		im = ax1.imshow(E.transpose(), vmin=0.9, vmax=26, cmap=plt.get_cmap('plasma'))
		im.cmap.set_under('k')
		im.cmap.set_over('cyan')
	else:
		im = ax1.imshow(E.transpose(), vmin=-1.1, vmax=1.1, cmap=plt.get_cmap('gray'))
		im.cmap.set_under('g')
		im.cmap.set_over('b')

	# ax1.set_ylabel('({0:2.2f}, {1:2.2f})'.format(float(fields[2]), float(fields[3])))
	ax1.set_title(qnames[queue])

	# plot expansion delay and f-value graphs
	EDS = np.genfromtxt(SOL_DIR + MAP + '_texpands.map', delimiter=',')
	FS = np.genfromtxt(SOL_DIR + MAP + '_fvals.map', delimiter=',') / 100
	if movingai:
		EDS[:, [0, 1]] = EDS[:, [1, 0]]
		FS[:, [0, 1]] = FS[:, [1, 0]]
	ax2, ax2a = two_scales(ax2, range(len(EDS) - 1), FS[1:], np.diff(EDS), 'r', 'b')
	ax2.legend(loc='upper right')
	ax2a.legend(loc='lower right')

	plt.show()
	# plt.savefig(IMG_DIR + f + '.png', bbox_inches='tight')
	plt.cla()

