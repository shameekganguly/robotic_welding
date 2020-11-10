# stitch piece-wise linear and circular trajectories together

import numpy as np
import json
from matplotlib import pyplot as plt

LINE = 'line'
CIRCLE = 'circle'

ordered_piecewise_result_list = [
	{'type': LINE, 'filename': 'result_edge1.json'},
	{'type': CIRCLE, 'filename': 'result_circular.json'},
	{'type': LINE, 'filename': 'result_edge2.json'}
]

f_closed_path = False

output_path_info = []

RESULT_FILE = 'Result_stitched.json'

def getEndPointForLineSegmentOnCircle(line_start, line_dir, circle_center, circle_radius, circle_normal):
	line_start_proj = line_start + np.dot(circle_center - line_start, circle_normal)*circle_normal
	line_dir_proj = line_dir - np.dot(line_dir, circle_normal)*circle_normal
	line_dir_proj /= np.linalg.norm(line_dir_proj)
	if np.linalg.norm(line_start_proj - circle_center) < circle_radius:
		raise Exception("Line start lies inside circle")

	c = circle_center
	r = circle_radius
	ls = line_start_proj
	ld = line_dir_proj
	p = c - ls
	p_dot_ld = np.dot(p, ld)
	if p_dot_ld < 0:
		ld *= -1
		p_dot_ld *= -1
	punit = p / np.linalg.norm(p)
	m = (p_dot_ld)**2 - (np.dot(p, p) - r**2)
	lend = ls
	if m < 0:
		# two imaginary roots, need to use tangent
		theta = np.arcsin(r/np.linalg.norm(p))
		st = np.sin(theta)
		ct = np.cos(theta)
		tan1 = np.dot(np.array([[ct, -st, 0],[st, ct, 0], [0, 0, 1]]), punit)
		tan = tan1
		tan2 = np.dot(np.array([[ct, st, 0],[-st, ct, 0], [0, 0, 1]]), punit)
		if np.dot(tan1, ld) > np.dot(tan2, ld):
			tan = tan1
		else:
			tan = tan2
		lend = ls + tan*np.sqrt(np.linalg.norm(p)**2 - r**2)
	else:
		# two real roots
		t = p_dot_ld - np.sqrt(m)
		# (p - ld*t).(p - ld*t) = r^2 = p.p - 2(p.ld)t + t^2
		lend = ls + t * ld
	return lend

i = 0
last_segment_info = {}
for i in range(len(ordered_piecewise_result_list)):
	segment_info = {}
	with open(ordered_piecewise_result_list[i]['filename']) as f:
		segment_info = json.loads(f.read())
		segment_info['type'] = ordered_piecewise_result_list[i]['type']
	
	if segment_info['type'] == LINE:
		print "Segment type: ", segment_info['type']
		print "Start: ", segment_info['joint_start']
		print "Stop: ", segment_info['joint_end']
		print "Flip start and stop points? 1: yes, 2: no"
		x = input()
		if x == 1:
			temp = segment_info['joint_start']
			segment_info['joint_start'] = segment_info['joint_end']
			segment_info['joint_end'] = temp
	else:
		# TODO: for circle, allow flipping plane?
		pass
	output_path_info.append({'type': segment_info['type']})
	if i == 0:
		if segment_info['type'] == CIRCLE:
			raise Exception("Not supported for first path yet, as we need a starting point for path.")
		output_path_info[i]['joint_start'] = segment_info['joint_start']
		output_path_info[i]['plane1_normal'] = segment_info['plane1']['normal']
		output_path_info[i]['plane2_normal'] = segment_info['plane1']['normal']
		last_segment_info = segment_info
		continue
	else:
		if segment_info['type'] == CIRCLE:
			output_path_info[i]['circle_normal'] = segment_info['circle_normal']
			output_path_info[i]['circle_radius'] = segment_info['circle_radius']
			output_path_info[i]['circle_center'] = segment_info['circle_center']
			center = np.array(segment_info['circle_center'])
			radius = segment_info['circle_radius']
			if last_segment_info['type'] == CIRCLE: # circle followed by circle
				raise Exception("Not supported circle to circle path")
			else: # line segment followed by circle
				line_dir = np.array(last_segment_info['joint_end']) - np.array(last_segment_info['joint_start'])
				# compute end point for line segment
				line_end = getEndPointForLineSegmentOnCircle(
					np.array(last_segment_info['joint_start']),
					line_dir,
					center,
					radius,
					np.array(segment_info['circle_normal'])
				)
				# print line_end
				output_path_info[i-1]['joint_end'] = line_end.tolist()
		elif segment_info['type'] == LINE:
			output_path_info[i]['plane1_normal'] = segment_info['plane1']['normal']
			output_path_info[i]['plane2_normal'] = segment_info['plane2']['normal']
			if i == len(ordered_piecewise_result_list) - 1:
				output_path_info[i]['joint_end'] = segment_info['joint_end']
			if last_segment_info['type'] == CIRCLE: # circle followed by line
				center = np.array(last_segment_info['circle_center'])
				radius = last_segment_info['circle_radius']
				line_dir = np.array(segment_info['joint_end']) - np.array(segment_info['joint_start'])
				line_start = getEndPointForLineSegmentOnCircle(
					np.array(segment_info['joint_end']),
					line_dir,
					center,
					radius,
					np.array(last_segment_info['circle_normal'])
				)
				output_path_info[i]['joint_start'] =  line_start.tolist()
			else: # line followed by line
				pass
		last_segment_info = segment_info

with open(RESULT_FILE, 'w') as f:
	f.write(json.dumps(output_path_info))

# visualize
plt.figure()
plt.plot(output_path_info[0]['joint_start'][0], output_path_info[0]['joint_start'][1],'or')
plt.plot(output_path_info[0]['joint_end'][0], output_path_info[0]['joint_end'][1],'or')
plt.plot(output_path_info[2]['joint_start'][0], output_path_info[2]['joint_start'][1],'or')
plt.plot(output_path_info[2]['joint_end'][0], output_path_info[2]['joint_end'][1],'or')
circ_theta = np.linspace(0, 2*np.pi, 100)
circ_x = np.cos(circ_theta)*output_path_info[1]['circle_radius'] + output_path_info[1]['circle_center'][0]
circ_y = np.sin(circ_theta)*output_path_info[1]['circle_radius'] + output_path_info[1]['circle_center'][1]
plt.plot(circ_x, circ_y, '.b')

plt.show()
