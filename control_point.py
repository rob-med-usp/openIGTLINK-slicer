import pickle
import os
import vtk

#pointA_Beta = alphaToBetaMatrix.MultiplyFloatPoint(pointA_Alpha)

cur_path = os.path.dirname(__file__)
path = cur_path + 'registration/out/matrix.p'

with open(path, 'rb') as file:
    matrix = pickle.load(file)

matrix = matrix.split()
matrix = [float(i) for i in matrix]


transform = vtk.vtkMatrix4x4()
transform.DeepCopy(matrix)

print(transform)

