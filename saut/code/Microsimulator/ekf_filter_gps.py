import numpy as np
import math
import matplotlib.pyplot as plt
import os


def accuracy(real,prediction):

    dist_accuracy = np.sqrt(((real[:,0]-prediction[:,0])**2)+((real[:,1]-prediction[:,1])**2))

    return dist_accuracy


def plot1():

    
    plt.cla()
    plt.clf()
    plt.subplots_adjust(left=0.1,
                    bottom=0.1, 
                    right=0.9, 
                    top=0.9, 
                    wspace=0.4, 
                    hspace=0.7)
    plt.subplot(2, 1, 1) # row 1, col 2 index 1
    plt.plot(Rv[:,0], Rv[:,1],label="Real Position",color="red")
    plt.plot(Rv[-1,0], Rv[-1,1], marker="*", markersize=5, markeredgecolor="red", markerfacecolor="red")
    plt.plot(Y[:,0], Y[:,1],label="Sensor Values",color="orange")
    plt.plot(Re[:,0], Re[:,1],label="Mean of Real Position",color="purple")
    plt.plot(Re[-1,0], Re[-1,1], marker="*", markersize=5, markeredgecolor="purple", markerfacecolor="purple")
    plt.plot(ReKalman[:,0], ReKalman[:,1],label="Mean of Kalman",color="blue")
    plt.plot(ReKalman[-1,0], ReKalman[-1,1], marker="*", markersize=5, markeredgecolor="blue", markerfacecolor="blue")
    #plt.plot(Z[:,0], Z[:,1],label="Mean Sensor")
    # naming the x axis
    plt.xlabel('X(m)')
    # naming the y axis
    plt.ylabel('Y(m)')
    plt.legend()
    # giving a title to my graph
    plt.title('Compare Graph')


    plt.subplot(2, 1, 2) # index 2
    plt.plot(tempo, Accuracy_kalman,label="Accuracy_Kalman",color="blue")
    plt.plot(tempo, Accuracy_sensor,label="Accuracy_sensor",color="orange")
    plt.title("Accuracy")
    plt.xlabel('Time')
    plt.ylabel('Error')
    
    plt.pause(0.000001)

def plot2():

    plt.plot(Rv[:,0], Rv[:,1],label="Real Position")
    plt.plot(Y[:,0], Y[:,1],label="Sensor Values")
    plt.plot(Re[:,0], Re[:,1],label="Mean of Real Position")
    plt.plot(ReKalman[:,0], ReKalman[:,1],label="Mean of Kalman")
    #plt.plot(Z[:,0], Z[:,1],label="Mean Sensor")
    # naming the x axis
    plt.xlabel('X(m)')
    # naming the y axis
    plt.ylabel('Y(m)')
    plt.legend()
    # giving a title to my graph
    plt.title('Compare Graph')
    
    plt.show()
 
    

#-------SISTEMA DO ROBO---------#
#Posicão Inicial do robô
Pv = np.array([[0, 0, 0]]) # x,y,teta
#Ruidos aleatorios da posicação verdadeira
sigma_x = 0.005
sigma_y = 0.005
sigma_teta = np.deg2rad(0.1)
sigma_v = 0.01
sigma_w = np.deg2rad(0.1)

# Matriz de covariâncias das Posições
Qk = np.matrix([[sigma_x**2, 0, 0], 
                [0, sigma_y**2, 0], 
                [0, 0, sigma_teta**2]])

#Matriz de covariâncias do controles
M = np.matrix([[sigma_v**2, 0], 
               [0, sigma_w**2]])

#posicao inicial da esperança do robo
Pe = np.array([[10, 10, 0]]) # x,y,teta
PeKalman = Pe
#Covariancia inicial entre a esperança e posicao real
Pk = Qk
PKalman = Pk

#Armazenando posicões inicias
Rv = Pv
Re = Pe
ReKalman = PeKalman



#Tempo de amostragem de simulação
dt = 0.1


#-------SENSOR---------#
#Matriz Controle do Sensor GPS
C = np.matrix([[1, 0, 0], 
               [0, 1, 0]])



#Ruidos aleatorios da sensor
sigma_z_x = 0.1
sigma_z_y = 0.1

#Matriz de covariâncias do do ruido sensor
R = np.matrix([[sigma_z_x**2, 0], 
               [0, sigma_z_y**2]])


#Leitura do Sensor inicial
y = np.dot(C,Pv.T) + np.dot(np.sqrt(R),np.random.normal(0,0.1,size=(2,1)))

#Armazenando Valor de Y
Y = y.T

#Esperança valor Inicial sensor

H = C
z = np.dot(H,Pe.T)

Z = z.T

#-------LEI DE CONTROLE ---------#

Raio = 2
v = 0.5
w = v/Raio

#Tempo máximo  10 Voltas

t=0.1
tmax = 10 *((2*math.pi*Raio)/v)

#Armazenando accuracy
Accuracy_kalman = np.array([[0]])
Accuracy_sensor = np.array([[0]])
tempo = np.array([[0]])
  
# ligando o modo interativo
plt.ion()

#--------LOOP-----------#
while t<=tmax:
    
    t = t + dt

    #Pegando valores das velocidades lineares e angulares
    v = 0.5     
    w = v/Raio  

    #Ruidos verdadeiros robo

    ksi_x = sigma_x * np.random.normal()
    ksi_y = sigma_y * np.random.normal()
    ksi_teta = sigma_teta * np.random.normal()
    ksi_v = sigma_v * np.random.normal()
    ksi_w = sigma_w * np.random.normal()

    #Posição verdadeira instante K
    Pv = np.array([[(Pv[0,0]+ksi_x)+((v+ksi_v)*math.cos(Pv[0,2]+ksi_teta)*dt),
                    (Pv[0,1]+ksi_y)+((v+ksi_v)*math.sin(Pv[0,2]+ksi_teta)*dt), 
                    (Pv[0,2]+ksi_teta)+((w+ksi_w)*dt)]])

    Rv = np.append(Rv, Pv, axis=0)

    #Leitura do Sensor
    y = np.dot(C,Pv.T) + np.dot(np.sqrt(R),np.random.normal(0,0.1,size=(2,1)))
    
    Y = np.append(Y, y.T, axis=0)

    #Posicão estimada pelo robô
    Pe = np.array([[(Pe[0,0])+(v)*math.cos(Pe[0,2])*dt,
                     (Pe[0,1])+(v)*math.sin(Pe[0,2])*dt, 
                     (Pe[0,2])+(w)*dt]])

    Re = np.append(Re, Pe, axis=0)

    PeKalman = np.array([[(PeKalman[0,0])+(v)*math.cos(PeKalman[0,2])*dt,
                     (PeKalman[0,1])+(v)*math.sin(PeKalman[0,2])*dt, 
                     (PeKalman[0,2])+(w)*dt]])

    #Calulo dos Jacobianos

    #Jacobiano do modelo
    F = np.matrix([[1, 0, v*(-1)*math.sin(Pe[0,2])*dt],
                   [0, 1, v*math.cos(Pe[0,2])*dt], 
                   [0, 0, 1]])
    Fkalman = np.matrix([[1, 0, v*(-1)*math.sin(PeKalman[0,2])*dt],
                   [0, 1, v*math.cos(PeKalman[0,2])*dt], 
                   [0, 0, 1]])
    #Jacobiano do controle
    G = np.matrix([[math.cos(Pe[0,2])*dt, 0],
                   [math.sin(Pe[0,2])*dt, 0], 
                   [0, dt]])
    Gkalman= np.matrix([[math.cos(PeKalman[0,2])*dt, 0],
                   [math.sin(PeKalman[0,2])*dt, 0], 
                   [0, dt]])
    #Caculo da Covariancia da Esperança e valor real
    Pk = np.dot(np.dot(F,Pk),F.T)+np.dot(np.dot(G,M),G.T)+Qk
    PKalman = np.dot(np.dot(Fkalman,PKalman),Fkalman.T)+np.dot(np.dot(Gkalman,M),Gkalman.T)+Qk
    #Esperança do Sensor
    z = np.dot(H,Pe.T)
    Z = np.append(Z, z.T, axis=0)
    zkalman = np.dot(H,PeKalman.T)
    #Filtro de Kalman Estendido
    
    #Ganho de Kalman
    K = np.dot(np.dot(PKalman,H.T),np.linalg.inv(np.dot(np.dot(H,PKalman),H.T)+R))
    
    #Correção Esperança do Sistema
    
    PeKalman = PeKalman + np.dot((y-zkalman).T,K.T)
    ReKalman = np.append(ReKalman, PeKalman, axis=0)

    #Correção da covariância
    PKalman = np.dot((np.eye(3))-np.dot(K,H),PKalman)

    #accuracy
    Accuracy_kalman = np.append(Accuracy_kalman, accuracy(Pv, PeKalman))


    Accuracy_sensor=  np.append(Accuracy_sensor, accuracy(Pv, y.T))

    tempo = np.append(tempo, t)


    #Plotar grafico
    plot1()
    


# desligando o modo interativo
plt.ioff()
#plot2()