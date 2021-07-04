import pandas as pd
import matplotlib.pyplot as plt

data = pd.read_csv("uavexps.txt", header=None)

fig = plt.figure()
ax = plt.gca()

ax.scatter(9, 9, color='r')
ax.scatter(36, 27, color='b')

ax.scatter(data[0], data[1], color='g', alpha=0.5)
plt.show()