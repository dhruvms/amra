import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

import os
import sys

EXPS_DIR = '../dat/expansions/'
SOL_DIR = '../dat/solutions/'

MAP = sys.argv[1]
MAP = MAP.split('/')[-1].split('.')[0]

nrows = ncols = -1
for f in os.listdir(EXPS_DIR):
	fields = f.split('_')

	iters = int(fields[0])
	queue = int(fields[1])

	if (iters > nrows):
		nrows = iters
	if (queue > ncols):
		ncols = queue

nrows += 1
ncols += 1

fig = plt.figure(figsize=(ncols+1, nrows+1))
gs = gridspec.GridSpec(nrows, ncols,
	wspace=0.0, hspace=0.0,
	top=1.-0.5/(nrows+1), bottom=0.5/(nrows+1),
	left=0.5/(ncols+1), right=1-0.5/(ncols+1))

for f in os.listdir(EXPS_DIR):
	fields = f.split('_')
	iters = int(fields[0])
	queue = int(fields[1])

	E = np.genfromtxt(EXPS_DIR + f, delimiter=',')
	P = np.genfromtxt(SOL_DIR + '{0:04d}'.format(iters) + '_' + MAP + '_path.map', delimiter=',')

	E[P[:, 0].astype(np.int), P[:, 1].astype(np.int)] += (E[P[:, 0].astype(np.int), P[:, 1].astype(np.int)] // 10) * 10

	ax = plt.subplot(gs[iters*ncols + queue])
	ax.plot(P[:, 0], P[:, 1], 'r', lw=1, alpha=0.25)
	ax.imshow(E.transpose(), vmin=-1, vmax=20, cmap=plt.get_cmap('twilight'))

	ax.set_xticklabels([])
	ax.set_yticklabels([])
	ax.axis('tight')

	if (queue == 0):
		ax.set_ylabel('({0:2.2f}, {1:2.2f})'.format(float(fields[2]), float(fields[3])))

plt.subplot(gs[0]).set_title('anchor')
plt.subplot(gs[1]).set_title('high res')
plt.subplot(gs[2]).set_title('mid res')
plt.subplot(gs[3]).set_title('low res')

plt.show()
