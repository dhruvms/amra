import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

solution = pd.read_csv("../dat/uavsol.txt", header=None)
X = solution[0]
Y = solution[1]
Theta = solution[2]
Vels = solution[3]

fig = plt.figure()
ax = plt.gca()

ax.scatter(X,Y)
plt.show()