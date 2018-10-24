import numpy as np


def get_dis(x1,y1,z1,x2,y2,z2):
	p1 = np.array((x1, y1, z1))
	p2 = np.array((x2, y2, z2))
	return np.linalg.norm(p1 - p2)