import sys
import numpy as np
import matplotlib.pyplot as plt
import scipy.ndimage as ndimage

seeds = [4, 64, 144, 441, 3375, 4096, 4261]
np.random.seed(seed=np.random.choice(seeds))

MAP = sys.argv[1]
mapname = MAP.split('/')[-1].split('.')[0]

M = np.genfromtxt(MAP, delimiter=',')
m, n = M.shape
noise = np.random.randn(m, n) * 5
noise = ndimage.gaussian_filter(noise, sigma=(5, 5), order=0, mode='wrap')
noise = np.abs(noise)
costs = 97 + (122-97) * ((noise - np.min(noise)) / (np.max(noise) - np.min(noise)))
costs = costs * M
costs = costs.astype(np.int)

with open('../dat/' + mapname + '_costs.map', 'w') as F:
	F.write("type octile\n")
	F.write("height " + str(m) + "\n")
	F.write("width " + str(n) + "\n")
	F.write("map\n")

	for d1 in range(m):
		for d2 in range(n):
			if costs[d1, d2] == 0:
				F.write("T")
			else:
				F.write(chr(costs[d1, d2]))
		F.write("\n")

im = plt.imshow(costs.transpose(), cmap=plt.get_cmap('rainbow'), vmin=97, vmax=122)
im.cmap.set_under('k')
# plt.colorbar()
plt.savefig('../dat/imgs/bmps/' + mapname + '.png', bbox_inches='tight')
