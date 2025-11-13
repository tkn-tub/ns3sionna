import matplotlib.pyplot as plt
import numpy as np
import sys
import os

# load CSI
fname = sys.argv[1]
csi = np.asarray([complex(line.strip()) for line in open(fname, 'r')])

fname_pl = sys.argv[2]
pathLossDb = np.asarray([float(line.strip()) for line in open(fname_pl, 'r')])

pathloss_lin = np.power(10.0, (-pathLossDb) / 10.0)

fname_time_pos = sys.argv[3]
time_pos = np.loadtxt(fname_time_pos, delimiter=',').astype(float)

# reshape
NFFT = 3073
N = 10
all_csi = csi.reshape(N, NFFT)

# plot mag squared
mag_csi = np.abs(all_csi) ** 2

# create coordinate grid
x = np.arange(mag_csi.shape[1])
y = np.arange(mag_csi.shape[0])
X, Y = np.meshgrid(x, y)

# plot mag squared with pathloss
#mag_csi_pl = np.abs(all_csi * pathloss_lin[:, np.newaxis]) ** 2
mag_csi_pl = (np.abs(all_csi) ** 2) * pathloss_lin[:, np.newaxis]

mag_csi_pl_db = 10 * np.log10(mag_csi_pl)

# create 3D plot
fig = plt.figure()
ax = fig.add_subplot(121, projection='3d')

# Surface plot
surf = ax.plot_surface(X, Y, mag_csi, cmap='viridis')

# Labels and colorbar
ax.set_xlabel('OFDM subcarrier [idx]')
ax.set_ylabel('CSI sample [id]')
ax.set_zlabel('|h|^2')
fig.colorbar(surf, ax=ax, shrink=0.5, aspect=10)

ax2 = fig.add_subplot(122, projection='3d')

# Surface plot
surf2 = ax2.plot_surface(X, Y, mag_csi_pl_db, cmap='viridis')

# Labels and colorbar
ax2.set_xlabel('OFDM subcarrier [idx]')
ax2.set_ylabel('CSI sample [id]')
ax2.set_zlabel('PL * |h|^2 [dB]')
fig.colorbar(surf2, ax=ax2, shrink=0.5, aspect=10)

plt.tight_layout()
plt.show()

plt.scatter(1.0, 2.0, marker='p', s=25, c='r', alpha=0.5)
plt.scatter(time_pos[:,1], time_pos[:,2], s=15, c='b', alpha=0.5)

plt.tight_layout()
plt.show()

rx_power_dbm = 20 - pathLossDb
rx_power_dbm = 20 - pathLossDb
plt.plot(time_pos[:,0], rx_power_dbm)
plt.ylabel('Mean RX power [dBm]')
plt.xlabel('Time [s]')

plt.tight_layout()
plt.show()
