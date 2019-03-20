#!/usr/bin/env python
import numpy as np
from Astar_lib import *
working_matrix=np.load('resources/my_map_final.npy')
working_matrix = np.reshape(working_matrix, (150,155))
print(np.shape(working_matrix))
for i in range(np.shape(working_matrix)[0]):
	for j in range(np.shape(working_matrix)[1]):

		if (working_matrix[i][j] < 0.5):
			working_matrix[i][j] = 0

		else:
			working_matrix[i][j] = 1

working_matrix = working_matrix.T

x1, y1=130, 140
x2, y2= 10, 140

path=astar(working_matrix, (x2,y2), (x1,y1))
print(path)
show_path(working_matrix)

