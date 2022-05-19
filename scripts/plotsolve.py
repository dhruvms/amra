import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

import os
import sys

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

fig = plt.figure(figsize=(10,10))

movingai = False
flipped = False
for f in os.listdir(EXPS_DIR):
	if (f == '.gitignore'):
		continue

	ax = plt.gca()

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
		ax.scatter(S[0], S[1], s=100, c='g', zorder=10)
	if G is not None:
		ax.scatter(G[0], G[1], s=100, c='r', zorder=10, marker='*')

	# explicitly plot expansions for movingai maps
	if movingai and expansions is not None:
		ax.scatter(expansions[:, 1], expansions[:, 0], s=5, c='b')

	# plot solution path
	P = np.genfromtxt(SOL_DIR + '{0:04d}'.format(iters) + '_' + MAP + '_path.map', delimiter=',')
	if movingai:
		P[:, [0, 1]] = P[:, [1, 0]]
	ax.plot(P[:, 0], P[:, 1], 'salmon', lw=4, alpha=1.0)

	# plot arrows along solution path
	N = 10
	step = max(P.shape[0]//N, 1)
	for p in range(1, P.shape[0], step):
		dx = min(abs(P[p, 0]-P[p-1, 0]), 0.1) * np.sign(P[p, 0]-P[p-1, 0])
		dy = min(abs(P[p, 1]-P[p-1, 1]), 0.1) * np.sign(P[p, 1]-P[p-1, 1])
		ax.arrow(P[p-1, 0]+dx, P[p-1, 1]+dy, dx, dy, shape='full', lw=0.5, length_includes_head=True, head_width=0.5+movingai*10, zorder=8, head_starts_at_zero=True, facecolor='yellow')

	# display map
	im = None
	if 'costs' in MAP:
		im = ax.imshow(E.transpose(), vmin=0.9, vmax=26, cmap=plt.get_cmap('plasma'))
		im.cmap.set_under('k')
		im.cmap.set_over('cyan')
	else:
		im = ax.imshow(E.transpose(), vmin=-1.1, vmax=1.1, cmap=plt.get_cmap('gray'))
		im.cmap.set_under('g')
		im.cmap.set_over('b')

	ax.set_ylabel('({0:2.2f}, {1:2.2f})'.format(float(fields[2]), float(fields[3])))
	ax.set_title(qnames[queue])

	# plt.show()
	plt.savefig(IMG_DIR + f + '.png', bbox_inches='tight')
	plt.cla()

