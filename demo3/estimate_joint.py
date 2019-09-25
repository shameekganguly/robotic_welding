import numpy as np
import pandas
pandas.set_option("mode.chained_assignment", None)
from pandas import read_csv
import argparse
from itertools import combinations
import json
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import random
import pprint
from functools import partial
from multiprocessing import Pool
import time

POSITION_VAR_NAME = "oppos" # 3D position vector 
POSITION_VAR_LEN = 3
FORCE_VAR_NAME = "force" # 6D force-torque vector
FORCE_VAR_LEN = 3
TORQUE_VAR_LEN = 3

# MIN_TIME = 0
MIN_TIME = 5000000 # 5 seconds, to get rid of data while moving arm manually

FORCE_BIAS = np.array([-1.3556, 1.5259, -188.47]) # NOTE: this is configuration dependent, but we use fixed orientation
# TORQUE_BIAS = np.array([-1.3983, 3.51893, 0.5698])

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
	parser.add_argument('input_path', help='Path to input log')
	parser.add_argument('-f', '--fit_type', choices=["exhaust", "random"], default="exhaust", help='Choose plane fit type.')
	parser.add_argument('-o', '--output_path', help='Path to output log', default='')
	parser.add_argument('-ro', '--rel_output_path', help='Filename or path to write out to. Assumed related to the basedir for the input log file.', default='')
	parser.add_argument('-nf', '--skip_filter', help='Do not filter the data', default=False, action='store_true')
	parser.add_argument('-np', '--skip_plots', help='Do not plot any data', default=False, action='store_true')
	return parser.parse_args()

'''
get minimum distance between 3 points and their centroid
'''
def min_dist(positions):
	centroid = (positions.sum()/3.0)
	# print positions
	# print centroid
	# print np.linalg.norm(positions - centroid, axis=1)
	return np.min(np.linalg.norm(positions - centroid, axis=1))

'''
get plane normal
'''
def plane_normal(ind_tuple, positions):
	temp1 = np.array(positions.iloc[ind_tuple[0]] - positions.iloc[ind_tuple[1]])
	temp2 = np.array(positions.iloc[ind_tuple[2]] - positions.iloc[ind_tuple[1]])
	temp3 = np.cross(temp1, temp2)
	temp3_norm = np.linalg.norm(temp3)
	if temp3_norm < 1e-6:
		return [np.nan, np.nan, np.nan]
	else:
		return temp3/temp3_norm

'''
generate planes
note: planes are represented as tuples of 3 points
input: 
	positions: pandas df with rows containing x, y, z positions
'''
def get_planes_exhaustive(positions):
	planes = []
	N_points = len(positions)
	ind_comb = list(combinations(range(N_points), 3))
	for ind in ind_comb:
		# discard planes where points are too close
		if min_dist(positions.iloc[list(ind)]) > INTER_POINT_THRES:
			normal = plane_normal(ind, positions)
			if True not in np.isnan(normal):
				planes.append({'inds': ind, 'normal': normal})
	return planes

def random_combination(iterable):
    "Random selection from itertools.combinations(iterable, r)"
    pool = tuple(iterable)
    n = len(pool)
    return pool[random.sample(xrange(n), 1)[0]]

def get_planes_random(positions):
	planes = []
	N_points = len(positions)
	combs = list(combinations(range(N_points), 3))
	while len(planes) < MAX_PLANES:
		# print "try plane combo"
		ind = combs[random.randrange(len(combs))]
		if ind in [p['inds'] for p in planes]:
			# print "plane already found, ", ind
			continue
		# discard planes where points are too close
		if min_dist(positions.iloc[list(ind)]) > INTER_POINT_THRES:
			normal = plane_normal(ind, positions)
			if True not in np.isnan(normal):
				planes.append({'inds': ind, 'normal': normal})
				# print len(planes)
			else:
				# print "points almost linear ", ind
				pass
		else:
			# print "points too close ", ind
			pass
	return planes

'''
get distance from a plane to a point
'''
def plane_point_dist(point, plane, all_points):
	# print plane['normal']
	dist = np.dot(plane['normal'], point-all_points.iloc[plane['inds'][1]])
	# print dist
	return abs(dist)

def plane_points_dist(points, plane, all_points):
	return points.apply(lambda x: plane_point_dist(x, plane, all_points), axis=1)

'''
check if two planes are equal
'''
def arePlanesEqual(plane1, plane2, all_points):
	normal_dot = abs(np.dot(plane1['normal'], plane2['normal']))
	if normal_dot > 0.98:
		# print "Normal close."
		dist_arr = []
		for i in range(3):
			dist_arr.append(plane_point_dist(all_points.iloc[plane2['inds'][i]], plane1, all_points))
		if min(dist_arr) < 0.005:
			# print "Dist: ", max(dist_arr)
			return True
	return False

'''
parallel apply on dataframe
'''
def parallel_point_plane_dist(points, plane, n_cores=4):
    df_split = np.array_split(points, n_cores)
    pool = Pool(n_cores)
    res_df = pandas.concat(pool.map(partial(plane_points_dist, plane=plane, all_points=points), df_split))
    pool.close()
    pool.join()
    return res_df

'''
main
'''
opts = parse_opts()

data_fname = opts.input_path

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

if not opts.skip_plots:
	fig = plt.figure()
	ax = fig.add_subplot(111, projection='3d')
	ax.plot(data[pos_inds[0]], data[pos_inds[1]], data[pos_inds[2]], 'o')
	ax.set_xlabel('X')
	ax.set_ylabel('Y')
	ax.set_zlabel('Z')
	# plt.show()

# print pos_inds, force_inds

""" label data by whether or not point is in contact """
data['in_contact'] = np.linalg.norm(data[force_inds] - FORCE_BIAS, axis=1) > FORCE_CONTACT_THRES
contact_data = data[(data['in_contact'])]

if not opts.skip_plots:
	plt.figure()
	plt.plot(data['timestamp'], data[force_inds] - FORCE_BIAS)
	# plt.show()

print "Contact data contains ", len(contact_data), " points"

if not opts.skip_plots:
	fig = plt.figure()
	ax = fig.add_subplot(111, projection='3d')
	ax.plot(contact_data[pos_inds[0]], contact_data[pos_inds[1]], contact_data[pos_inds[2]], 'o')
	ax.set_xlabel('X')
	ax.set_ylabel('Y')
	ax.set_zlabel('Z')
	# plt.show()

# if not opts.skip_plots:
# 	fig, axes = plt.subplots(2,1)
# 	axes[0].plot(contact_data['timestamp'], contact_data[pos_inds[1]],'.')
# 	axes[0].set_ylabel('Y')
# 	axes[1].plot(contact_data['timestamp'], contact_data[pos_inds[2]],'.')
# 	axes[1].set_ylabel('Z')
# 	# plt.show()

""" resample points to avoid crowding """
if opts.skip_filter:
	filtered_contact_data = contact_data
else:
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
# print filtered_contact_data.iloc[[0,1,2]]
# print filtered_contact_data
print "Filtered data after unclustering contains ", len(filtered_contact_data), " points"

if not opts.skip_plots:
	fig = plt.figure()
	ax = fig.add_subplot(111, projection='3d')
	ax.plot(filtered_contact_data[pos_inds[0]], filtered_contact_data[pos_inds[1]], filtered_contact_data[pos_inds[2]], 'o')
	ax.set_xlabel('X')
	ax.set_ylabel('Y')
	ax.set_zlabel('Z')
	plt.show()

""" fit planes """
# planes are represented as tuples of 3 indices, where each index corresponds
#   to a point in filtered_contact_data
# we use a voting system across all possible planes in the dataset
if opts.fit_type == "exhaust":
	planes = get_planes_exhaustive(filtered_contact_data[pos_inds])
elif opts.fit_type == "random":
	planes = get_planes_random(filtered_contact_data[pos_inds])
print "Fitted ", len(planes), " planes"

""" merge planes """
merged_planes = [planes[0]]
for plane1 in planes:
	planes_are_same = False
	for plane2 in merged_planes:
		if arePlanesEqual(plane1, plane2, filtered_contact_data[pos_inds]):
			planes_are_same = True
			# print "planes very similar, merged"
			# print plane1
			# print plane2
	if not planes_are_same:
		merged_planes.append(plane1)
print "Merged down to ", len(merged_planes), " planes"

# t0 = time.time()

for plane in merged_planes:
	distances = plane_points_dist(filtered_contact_data[pos_inds], plane, filtered_contact_data[pos_inds])
	votes = np.exp(distances*np.log(0.01)/DIST_VOTE_THRES)
	plane['vote'] = votes.sum()

# t1 = time.time()

# print "Voting took ", "%.3f"%(t1 - t0), " secs" 

RESULT_FILE = 'result.json'

sorted_planes = sorted(merged_planes, key = lambda i: i['vote'], reverse=True)

# pprint.pprint(sorted_planes)

# pprint.pprint(merged_sorted_planes)

print "Voting done. Top 2 planes are "
pprint.pprint(sorted_planes[0])
pprint.pprint(sorted_planes[1])

for plane in sorted_planes:
	plane['normal'] = plane['normal'].tolist()

json_result = json.dumps(sorted_planes)
with open(RESULT_FILE,'w') as f:
	f.write(json_result)


print "Done sorting planes. Output in ", RESULT_FILE


