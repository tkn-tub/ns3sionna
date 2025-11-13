import matplotlib.pyplot as plt
import numpy as np
import sys
import os

# load: time, node-id, snr, posx, posy, posz
fname = sys.argv[1]
data = np.loadtxt(fname, delimiter=',').astype(float)

d0 = data[data[:, 1] == 0]
d1 = data[data[:, 1] == 1]

plt.scatter(d0[:,3], d0[:,4], s=10, c='b', alpha=0.5)
plt.scatter(d1[:,3], d1[:,4], marker='p', s=25, c='r', alpha=0.5)
plt.grid()

plt.tight_layout()
plt.show()


plt.plot(d0[:,0], d0[:,2])
plt.grid()
plt.tight_layout()
plt.show()
