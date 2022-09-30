# CODIGO ADAPTADO DE https://gist.github.com/ungi/d34094acb2a08601b51321d7334cf158
# autor do codigo bruto: Tamas Ungi (https://gist.github.com/ungi)

#OBS: Este codigo não executará caso não o abra em um ambiente do 3D Slicer devido aos modulos e funções deste
# para executá-lo, use o start_script.sh (antes, altere o PATH para o diretório de seu 3D Slicer)

# OBS-2: Apenas execute este código caso ja tenha obtido todos os pontos do 3D Slicer E robô

# Create transform node for registration result (optional)


import numpy
import os
from datetime import datetime
# import registration.py
# from scripts.registration.registration import Registration
from registration import Registration
import pickle

reg = Registration(3)
# reg.slicer_read()
# reg.robot_read()



if reg.slicer_read() and reg.robot_read(): # and reg.robot_read

  alphaToBeta = slicer.vtkMRMLLinearTransformNode()
  alphaToBeta.SetName('AlphaToBeta')
  slicer.mrmlScene.AddNode(alphaToBeta)

  # Experiment parameters (start from here if you have alphaToBeta already)

  N = 3         # Number of fiducials
  Scale = 100.0  # Size of space where fiducial are placed
  Sigma = 2.0    # Radius of random error

  # Create first fiducial list

  fromNormCoordinates = numpy.random.rand(N, 3) # An array of random numbers
  noise = numpy.random.normal(0.0, Sigma, N*3)

  # Create the two fiducial lists

  alphaFids = slicer.vtkMRMLMarkupsFiducialNode()
  alphaFids.SetName('Alpha')
  slicer.mrmlScene.AddNode(alphaFids)

  betaFids = slicer.vtkMRMLMarkupsFiducialNode()
  betaFids.SetName('Beta')
  slicer.mrmlScene.AddNode(betaFids)
  betaFids.GetDisplayNode().SetSelectedColor(1,1,0)

  # vtkPoints type is needed for registration

  alphaPoints = vtk.vtkPoints()
  betaPoints = vtk.vtkPoints()

  for i in range(N):
    # 3d slicer nos da estes pontos
    # x = (fromNormCoordinates[i, 0] - 0.5) * Scale
    # y = (fromNormCoordinates[i, 1] - 0.5) * Scale
    # z = (fromNormCoordinates[i, 2] - 0.5) * Scale

    # le os pontos recebidos pelo 3d slicer 
    x = float(reg.s_points[i][0]) 
    y = float(reg.s_points[i][1])
    z = float(reg.s_points[i][2])
    

    numFids = alphaFids.AddFiducial(x, y, z)
    numPoints = alphaPoints.InsertNextPoint(x, y, z)

    # espaço destino (robo)
    # xx = x+noise[i*3]
    # yy = y+noise[i*3+1]
    # zz = z+noise[i*3+2]
    xx = float(reg.r_points[i][0])
    yy = float(reg.r_points[i][1])
    zz = float(reg.r_points[i][2])

    numFids = betaFids.AddFiducial(xx, yy, zz)
    numPoints = betaPoints.InsertNextPoint(xx, yy, zz)

  # Create landmark transform object that computes registration

  landmarkTransform = vtk.vtkLandmarkTransform()
  landmarkTransform.SetSourceLandmarks( alphaPoints )
  landmarkTransform.SetTargetLandmarks( betaPoints )
  landmarkTransform.SetModeToRigidBody()
  landmarkTransform.Update()

  alphaToBetaMatrix = vtk.vtkMatrix4x4()
  landmarkTransform.GetMatrix( alphaToBetaMatrix )

  det = alphaToBetaMatrix.Determinant()
  if det < 1e-8:
    print('Unstable registration. Check input for collinear points.')

  alphaToBeta.SetMatrixTransformToParent(alphaToBetaMatrix)

  print(alphaToBetaMatrix) # retorna informações gerais sobre a matriz


  #print(alphaToBeta)


  # Compute average point distance after registration

  average = 0.0
  numbersSoFar = 0

  for i in range(N):
    numbersSoFar = numbersSoFar + 1
    a = alphaPoints.GetPoint(i)
    print("a", a)
    pointA_Alpha = numpy.array(a)
    pointA_Alpha = numpy.append(pointA_Alpha, 1)

    # multiplicação que transforma os pontos:
    pointA_Beta = alphaToBetaMatrix.MultiplyFloatPoint(pointA_Alpha)


    print("b", pointA_Beta)
    b = betaPoints.GetPoint(i)
    pointB_Beta = numpy.array(b)
    pointB_Beta = numpy.append(pointB_Beta, 1)
    distance = numpy.linalg.norm(pointA_Beta - pointB_Beta)
    average = average + (distance - average) / numbersSoFar

  print("Average distance after registration: " + str(average))

  
  ####################### saida em txt (armazenamento da matriz)

  txt = ''
  for i in range(4):
      for j in range(4):
        ele = alphaToBetaMatrix.GetElement(i, j)
        print(ele) # retorna elemento por elemento
        txt = f"{txt} {ele}"
      # txt = txt + " "


  now = datetime.now().time().strftime("%H:%M:%S") # time object
  date = datetime.now().strftime("%Y-%m-%d") # date object
  # print("date:",date)
  # print("time =", now)


  txtM = txt + f'Matriz transformação gerada em -> Horário: {now}; Data (ano-mês-dia): {date}'
  print(txtM)


  cur_path = os.path.dirname(__file__)
  new_path = cur_path + 'out/matrix-reg.txt'
  # new_path = os.path.relpath('/out/matrix-reg.txt', cur_path)

  # salva matriz como txt para visualizaçao e checagem
  with open(new_path, 'w') as file:
    file.write(txtM)
  
  # salva os dados da matrix em vtk em binario para importaçao no codigo de controle
  new_path = cur_path + 'out/matrix.p'
  

  with open( new_path, "wb" ) as f:


	  pickle.dump( txt, f)
      

else:
  print('ERRO: Não foi possível obter os pontos da classe Registration.')
