import sim 
import simConst 
import numpy as np 
import sympy as sp  

def connect(port): 
    sim.simxFinish(-1) #Desconecta todos los puertos. 
    clientID = sim.simxStart('127.0.0.1', port, True, True, 2000, 5)
    if clientID == 0: 
        print("Conectado a : ", port)
    else: 
        print("No se pudo conectar a: ", port)
    return clientID  

def cinematica_directa(): 
    ## Longitud de los eslabones: 
    d1 = 1
    d2 = 0.5
    q1 = np.radians(90)
    q2 = np.radians(90)
    ret = sim.simxSetJointTargetPosition(clientID, joint1_handle, q1, sim.simx_opmode_blocking)
    ret = sim.simxSetJointTargetPosition(clientID, joint2_handle, q2, sim.simx_opmode_blocking)
    #Cinematica directa:
    # Si conozco las posiciones de los angulos puedo conocer
    # la pos final del robot.  
    # x = d3*np.sin(q0)*np.sin(q1)
    # y = -d3*np.cos(q0)*np.sin(q1)
    # z = d1+d2+d3*np.cos(q1)
    # print(f"Pos con cinematica en x: {x}, pos en y: {y}, pos en z: {z}") 
    retrunCode, pos_final_handle = sim.simxGetObjectHandle(clientID, "pos_final", sim.simx_opmode_blocking)
    pos_final = sim.simxGetObjectPosition(clientID, pos_final_handle, -1, sim.simx_opmode_blocking)
    print("Posicion del extremo final: ", pos_final)

def pendiente(segments):
    #[p2, p1, p0]
    segments.reverse()
    print("Encontrar los angulos entre los sig puntos: ", segments)
    eje_y = []
    eje_x = [] 
    q = []
    for i in range(len(segments) - 1): 
        eje_y.append(segments[i]["y"] - segments[i + 1]["y"])
        eje_x.append(segments[i]["x"] - segments[i + 1]["x"])

    for y, x in zip(eje_y, eje_x): 
        if y/x < 0.0:
            print("La pendiente es negativa") 
            q.append(np.arctan2(y,x))
        else: 
            print("La pendiente es positiva") 
            q.append(np.arctan2(y,x))
        print("q: ", q)
    return q
      

#Forward
def fabrik(dist_vectores, segments, tgt, base):     
    base = base 
    segments = segments 
    dist_vectores = dist_vectores
    p_prima = [tgt]
    
    for i in range(len(segments) - 1): 
        eje_x = segments[i+1]["x"] - p_prima[i]["x"] 
        eje_y = segments[i+1]["y"] - p_prima[i]["y"]
        ux = eje_x/(np.sqrt(eje_x**2 + eje_y**2))
        uy = eje_y/(np.sqrt(eje_x**2 + eje_y**2))

        prima_x = p_prima[i]["x"] + dist_vectores[i]*ux
        prima_y = p_prima[i]["y"] + dist_vectores[i]*uy
        prima = {
            "x": prima_x, 
            "y": prima_y
        }
        p_prima.append(prima)

    #Backward:
    # [d1 ,d2]
    dist_vectores.reverse()
    p_prima.reverse()
    p_2prima = [base] 

    for i in range(len(segments) - 1): 
        eje_x = p_prima[i+1]["x"] - p_2prima[i]["x"] 
        eje_y = p_prima[i+1]["y"] - p_2prima[i]["y"]
        ux = eje_x/(np.sqrt(eje_x**2 + eje_y**2))
        uy = eje_y/(np.sqrt(eje_x**2 + eje_y**2))

        prima_x = p_2prima[i]["x"] + dist_vectores[i]*ux
        prima_y = p_2prima[i]["y"] + dist_vectores[i]*uy
        prima = {
            "x": prima_x, 
            "y": prima_y
        }
        p_2prima.append(prima)

    print("p_2prima:", p_2prima)
    
    return p_2prima   
