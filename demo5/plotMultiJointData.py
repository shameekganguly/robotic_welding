# plot multiple tactile datasets from same geometry

import numpy as np
from numpy.linalg import norm
import scipy.spatial.transform as strans
import pandas
pandas.set_option("mode.chained_assignment", None)
from pandas import read_csv
import argparse
from itertools import combinations
import json
import matplotlib
matplotlib.use("Agg")
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import animation
import random
import pprint
from functools import partial
from multiprocessing import Pool
import os.path as osp
import time
import sys

POSITION_VAR_NAME = "oppos" # 3D position vector 
POSITION_VAR_LEN = 3
FORCE_VAR_NAME = "force" # 6D force-torque vector
FORCE_VAR_LEN = 3
TORQUE_VAR_LEN = 3

MIN_TIME = 0
# MIN_TIME = 5000000 # 5 seconds, to get rid of data while moving arm manually

# FORCE_BIAS = np.array([-1.3556, 1.5259, -188.47]) # NOTE: this is configuration dependent, but we use fixed orientation
# TORQUE_BIAS = np.array([-1.3983, 3.51893, 0.5698])
FORCE_BIAS = np.zeros(3)

# hyperparameters
FORCE_CONTACT_THRES = 1 #N
INTER_POINT_THRES = 0.01 #m
DIST_VOTE_THRES = 0.01 #m
MAX_PLANES = 100 # maximum number of planes to consider for exhaustive search
RAND_FIT_SAMPLES = 6 # maximum number of points to fit plane to

'''
parse arguments
'''
def parse_opts():
	parser = argparse.ArgumentParser(description='Analyze, filter and plot log')
	parser.add_argument('paths', metavar='filepath', type=str, nargs='+',
                    help='Paths to input log')
	return parser.parse_args()

def set_axes_equal(ax):
    '''Make axes of 3D plot have equal scale so that spheres appear as spheres,
    cubes as cubes, etc..  This is one possible solution to Matplotlib's
    ax.set_aspect('equal') and ax.axis('equal') not working for 3D.

    Input
      ax: a matplotlib axis, e.g., as output from plt.gca().
    '''

    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()

    x_range = abs(x_limits[1] - x_limits[0])
    x_middle = np.mean(x_limits)
    y_range = abs(y_limits[1] - y_limits[0])
    y_middle = np.mean(y_limits)
    z_range = abs(z_limits[1] - z_limits[0])
    z_middle = np.mean(z_limits)

    # The plot bounding box is a sphere in the sense of the infinity
    # norm, hence I call half the max range the plot radius.
    plot_radius = 0.5*max([x_range, y_range, z_range])

    ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
    ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
    # ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])

'''
main
'''
opts = parse_opts()

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

for path in opts.paths:
	print "File: ", path
	data_fname = path

	""" load data """
	data = read_csv(data_fname, low_memory=False)
	data = data.rename(columns=lambda x: x.strip()) # this removes white spaces padding the column names
	data.loc[data.index.size-1] = [0] * data.columns.size # clean up last row
	data = data[(data['timestamp'] > MIN_TIME)]

	print "Loaded data contains ", len(data), " points"

	""" get position and force data """
	names = list(data.columns)
	names = [''.join(n.split('_')[:-1]) for n in names if n.split('_')[:-1]]
	names = set(names)
	print "Loaded data contains following variables: ", list(names)

	if POSITION_VAR_NAME not in names:
		print "Data does not contain end effector position!"

	if FORCE_VAR_NAME not in names:
		print "Data does not contain end effector force/torque!"

	pos_inds = [POSITION_VAR_NAME+'_'+str(a) for a in range(POSITION_VAR_LEN)]
	force_inds = [FORCE_VAR_NAME+'_'+str(a) for a in range(FORCE_VAR_LEN)]

	""" label data by whether or not point is in contact """
	data['in_contact'] = np.linalg.norm(data[force_inds] - FORCE_BIAS, axis=1) > FORCE_CONTACT_THRES
	contact_data = data[(data['in_contact'])]

	""" resample points to avoid crowding """
	point_ind = 1
	filtered_inds = [0]
	while point_ind < len(contact_data):
		diff = np.linalg.norm(contact_data[pos_inds].iloc[point_ind] - contact_data[pos_inds].iloc[filtered_inds[-1]])
		if diff > INTER_POINT_THRES:
			filtered_inds.append(point_ind)
		point_ind += 1
	filtered_contact_data = contact_data.iloc[filtered_inds]
	# print filtered_contact_data.iloc[1]
	filtered_contact_data = filtered_contact_data.iloc[1:] # this is done to remove the initial 0's

	ax.plot(filtered_contact_data[pos_inds[0]], filtered_contact_data[pos_inds[1]], filtered_contact_data[pos_inds[2]], 'o')

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

set_axes_equal(ax)

def init():
    return fig,

def animate(i):
    ax.view_init(elev=30., azim=i)
    return fig,

# Set up formatting for the movie files
Writer = animation.writers['ffmpeg']
writer = Writer(fps=15, metadata=dict(artist='Me'), bitrate=1800)

# Animate
anim = animation.FuncAnimation(fig, animate, init_func=init,
                               frames=360, interval=20, blit=True)
# Save
anim.save(osp.join(osp.expanduser('~'), 'Downloads/basic_animation.mp4'), writer=writer)

plt.show()

