import matplotlib.pyplot as plt
import numpy as np
import sys

cstyle = ['r', 'b']

for ii in range(1, len(sys.argv)):
	fname = sys.argv[ii]

	csi = np.asarray([complex(line.strip()) for line in open(fname, 'r')])

	y = np.abs(csi) ** 2
	#y_db = 10 * np.log10(y)

	freq = np.arange(1,csi.shape[0]+1)

	plt.plot(freq, y, marker='', linestyle='-', color=cstyle[ii-1], label=fname)  # Line plot with markers


plt.xlim([1, max(freq)])
plt.xlabel('Freq bin')
plt.ylabel('|h|^2')
plt.title('CFR')
plt.grid(True)
plt.legend()

plt.show()
