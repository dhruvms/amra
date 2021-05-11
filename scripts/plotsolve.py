import numpy as np
import matplotlib.pyplot as plt

P = np.genfromtxt('lak303d_path.map', delimiter=',')
D = np.genfromtxt('lak303d_solution.map', delimiter=',')
plt.imshow(D.transpose(), vmin=-1, vmax=8)
plt.plot(P[:,0], P[:,1], 'y', alpha=0.25)
# plt.colorbar()
plt.show()
