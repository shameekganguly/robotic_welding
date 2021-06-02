# plot weld trajectory
import numpy as np
import pandas
pandas.set_option("mode.chained_assignment", None)
from pandas import read_csv
import sys
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


MIN_TIME = 305000000

POSITION_VAR_NAME = "oppos" # 3D position vector 
POSITION_VAR_LEN = 3

data_fname = sys.argv[1]
print "Plotting from ", data_fname

""" load data """
data = read_csv(data_fname, low_memory=False)
data = data.rename(columns=lambda x: x.strip()) # this removes white spaces padding the column names
data.loc[data.index.size-1] = [0] * data.columns.size # clean up last row
data = data[(data['timestamp'] > MIN_TIME)]
data['timestamp'] -= MIN_TIME

print "Loaded data contains ", len(data), " points"

""" get position and force data """
names = list(data.columns)
names = [''.join(n.split('_')[:-1]) for n in names if n.split('_')[:-1]]
names = set(names)
print "Loaded data contains following variables: ", list(names)

if POSITION_VAR_NAME not in names:
	print "Data does not contain end effector position!"

pos_inds = [POSITION_VAR_NAME+'_'+str(a) for a in range(POSITION_VAR_LEN)]

fig, [ax_X, ax_Y, ax_Z] = plt.subplots(3, sharex=True)
ax_X.plot(data['timestamp']*1e-6, data[pos_inds[0]], 'r')
ax_X.set_ylabel('X')
ax_Y.plot(data['timestamp']*1e-6, data[pos_inds[1]], 'g')
ax_Y.set_ylabel('Y')
ax_Z.plot(data['timestamp']*1e-6, data[pos_inds[2]], 'b')
ax_Z.set_ylabel('Z')
ax_Z.set_xlabel('time')

fig2 = plt.figure()
ax = fig2.add_subplot(111, projection='3d')
ax.set_aspect('equal')
ax.plot(data[pos_inds[0]], data[pos_inds[1]], data[pos_inds[2]])

plt.show()
