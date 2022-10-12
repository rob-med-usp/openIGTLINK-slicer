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

# print(transform)
pontoInicio = [4.23, 92.39, -69.25, 1]
pontoInicio2 = [6.38, 112.87, -48.19, 1 ]
pontoInicio3 = [4.34, 67.36, -114.16, 1]
pontoInicio4 = [1.88, 85.75, 73.98, 1]


pointA_Beta = transform.MultiplyFloatPoint(pontoInicio)
pointA_Beta2 = transform.MultiplyFloatPoint(pontoInicio2)
pointA_Beta3 = transform.MultiplyFloatPoint(pontoInicio3)
pointA_Beta4 = transform.MultiplyFloatPoint(pontoInicio4)

print(pointA_Beta)
print(pointA_Beta2)
print(pointA_Beta3)
print(pointA_Beta4)

