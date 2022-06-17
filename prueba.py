from pickle import TRUE
from tkinter import Y
import sim 
import simConst 
import numpy as np 
import sympy as sp  
from functions import fabrik, pendiente, connect


clientID = connect(19999)
ret, joint1_handle = sim.simxGetObjectHandle(clientID, "joint1", sim.simx_opmode_blocking)
ret, joint2_handle = sim.simxGetObjectHandle(clientID, "joint2", sim.simx_opmode_blocking)
#ret, joint3 = sim.simxGetObjectHandle(clientID, "joint3", sim.simx_opmode_blocking)
ret, pos_joint0 = sim.simxGetJointPosition(clientID, joint1_handle, sim.simx_opmode_blocking)
ret, pos_joint1 = sim.simxGetJointPosition(clientID, joint2_handle, sim.simx_opmode_blocking)
#ret, pos_join3 = sim.simxGetJointPosition(clientID, joint3, sim.simx_opmode_blocking)
print(f"Posicion Joint1: {pos_joint0}, Posicion Joint2: {pos_joint1}")

# Condiciones iniciales: 
p0 = {
  "x": 0, 
  "y": 0
}
p1 = {
  "x": 1, 
  "y": 0
}
p2 = {
  "x": 1.5, 
  "y": 0
}

#FABRIK IK
retrunCode, target_handle = sim.simxGetObjectHandle(clientID, "target", sim.simx_opmode_blocking)
returnCode, pos_target = sim.simxGetObjectPosition(clientID, target_handle, -1, sim.simx_opmode_blocking)
d1 = 1
d2 = 0.5
tgt = {
  "x": pos_target[0], 
  "y": pos_target[1], 
}

dist_vectores = [d2, d1]
segments = [p2, p1, p0] 
base = segments[len(segments) - 1]

for i in range(2): 
  p_2prima = fabrik(dist_vectores, segments, tgt, base)
  segments = p_2prima 
  dist_vectores = [d2, d1]
  print("Segments news: ", segments)
  
#Calculo de los angulos entre los eslabones: 
#q = producto_punto(p_2prima)
#q = pendiente(p_2prima)
y = float(p_2prima[1]["y"])
q = np.arcsin(y)
print("Valor de q1: ", q)
eje_y = d1*np.sin(q)
eje_x = d1*np.cos(q)
print("cinematica directa eslabon 1 eje_x: ", eje_x)
print(f"cinematica directa eslabon 1: eje_y {eje_y} \n")

y = float(p_2prima[2]["y"])
print("Valor usado por y: ", y)
q2 = np.arcsin((y - d1*np.sin(q))/d2) - q

print("Valor de q2: ", q2)
eje_x2 = d2*np.cos(q + q2) + d1*np.cos(q)
eje_y2 = d2*np.sin(q + q2) + d1*np.sin(q)
print("cinematica directa eslabon 2 eje_x: ", eje_x2)
print(f"cinematica directa eslabon 2: eje_y {eje_y2}")
print(f"Segments: {p_2prima} \n") 

#Movimiento de los motores:  
ret = sim.simxSetJointTargetPosition(clientID, joint1_handle, q, sim.simx_opmode_blocking)
ret = sim.simxSetJointTargetPosition(clientID, joint2_handle, q2, sim.simx_opmode_blocking)


retrunCode, pos_final_handle = sim.simxGetObjectHandle(clientID, "pos_final", sim.simx_opmode_blocking)
retCode, pos_final = sim.simxGetObjectPosition(clientID, pos_final_handle, -1, sim.simx_opmode_blocking)
etrunCode, eslabon1_handle = sim.simxGetObjectHandle(clientID, "eslabon1", sim.simx_opmode_blocking)
retCode, pos_eslabon1 = sim.simxGetObjectPosition(clientID, eslabon1_handle, -1, sim.simx_opmode_blocking)
print("Posicion del extremo final: ", pos_final)
print("Target: [x,y,z] ", tgt)
print("Eslabon1: ", pos_eslabon1)
print("Segmentos encontrados por Fabrik: ")
print("Ultimo segmento: ", p_2prima[0])
