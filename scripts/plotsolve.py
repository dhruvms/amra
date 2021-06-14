import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

import os
import sys

EXPS_DIR = '../dat/expansions/'
SOL_DIR = '../dat/solutions/'
IMG_DIR = '../dat/imgs/'

MAP = sys.argv[1]
MAP = MAP.split('/')[-1].split('.')[0]

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

qnames = {0: 'anchor', 1: 'high res', 2: 'mid res', 3: 'low res'}

fig = plt.figure()

for f in os.listdir(EXPS_DIR):
	if (f == '.gitignore'):
		continue

	ax = plt.gca()

	fields = f.split('_')
	iters = int(fields[0])
	queue = int(fields[1])

	E = np.genfromtxt(EXPS_DIR + f, delimiter=',')
	P = np.genfromtxt(SOL_DIR + '{0:04d}'.format(iters) + '_' + MAP + '_path.map', delimiter=',')

	ax.plot(P[:, 0], P[:, 1], 'r', lw=1, alpha=0.25)
	ax.imshow(E.transpose(), vmin=-1, vmax=20, cmap=plt.get_cmap('twilight'))

	ax.set_xticklabels([])
	ax.set_yticklabels([])
	ax.set_ylabel('({0:2.2f}, {1:2.2f})'.format(float(fields[2]), float(fields[3])))
	ax.set_title(qnames[queue])
	ax.axis('equal')

	# plt.show()
	plt.savefig(IMG_DIR + f + '.png', bbox_inches='tight')
	plt.cla()

