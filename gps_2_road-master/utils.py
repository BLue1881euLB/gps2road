import pandas as pd
import numpy as np


def overlap(box, boxVect):
	a = boxVect[:,2] <= box[0]
	b = boxVect[:,0] >= box[2]
	c = boxVect[:,3] <= box[1]
	d = boxVect[:,1] >= box[3]
	cond = np.any([a,b,c,d], axis = 0)

	return ~cond

def point_in_box(pt, boxVect):
	a = boxVect[:,0] <= pt[0]
	b = boxVect[:,2] >= pt[0]
	c = boxVect[:,1] <= pt[1]
	d = boxVect[:,3] >= pt[1]

	cond = np.all([a,b,c,d], axis = 0)

	return cond

def sample(times, interval = 5):
	select = []
	s = times[0]

	for time in times[1:]:
		if (time - s).seconds > interval:
			select.append(False)
		else:
			select.append(True)
			s = time

	return select
