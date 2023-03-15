#################################################### #################################################### 
#Importación de Librerias
import numpy as np
import math
from scipy.integrate import odeint
import matplotlib.pyplot as plt 
#################################################### 
#Parametros del Robot
Beta_b = 0.01 #Coeficiente de fricción viscosa del brazo
Beta_m = 0.04 #Coeficiente de fricción del eje del motor
lc = 0.5 #Distancia al centro de masa
l = 1 #Tamaño del brazo
m = 1 #Masa del brazo
g = 9.81 #Aceleración de la gravedad
Ib = (1/3)*m*pow(l,2) #Momento de inercia del brazo
Jm = 0.008 #Momento de inercia del motor
k = 50 #Constante torsional
#################################################### 
#gama = 1/Jm
#alfa4 = k/Jm
#alfa3 = Beta_b/Jm
#alfa2 = k/Ib
#alfa1 = m*g*lc/Ib
#alfa0 = Beta_b/Ib 
#################################################### 
#Ganancias del controlador
k1 = 10
k2 = 0.1
k3 = 2
k4 = 2
#################################################### 
#Vector de Tiempo de Simulación
start = 0
stop = 10
step =1e-3
t = np.arange(start,stop,step) 
#################################################### 
def f(x,t):
  #Referencia del sistema
  ref = math.sin(t)
  d1 = math.exp(-3*t)
  d2 = 5*math.exp(-3*t)
  e1=x[0]-ref
  e2=x[1]
  e3=x[2]
  e4=x[3]
  e = [e1,e2,e3,e4]

  w = -1*(k1*e[0]+k2*e[1]+k3*e[2]+k4*e[3])
  u = Beta_m*x[3]/Jm+x[2]*k/Jm-k*x[0]/Jm+w+d2/Jm
  
  dx_dt = [0,0,0,0]
  dx_dt[0] = x[1]
  dx_dt[1] = (1/Ib)*(-m*g*lc*math.sin(x[0])-k*x[0]) - (Beta_b/Ib*x[1]) + (k/Ib*x[2]) + d1/Ib
  dx_dt[2] = x[3]
  #dx_dt[3] = -Beta_m*x[3]/Jm-k*(x[2]-x[0])/Jm+u/Jm-k*d2/Jm
  dx_dt[3]=w
  return dx_dt

#################################################### 
#Solución de las ecuaciones diferenciales
Solucion = odeint(f, y0 =[0, 0,0,0], t=t) 
print('Solución', Solucion)
#Graficas
#Posición Angular del Robot 
plt.plot(t,Solucion[:,0], 'b', label='x1(t)') 
plt.xlabel('Tiempo (seg)') 
plt.ylabel('x1(t)')
plt.title('Angulo del Robot (rad)') 
plt.grid()
plt.ylim(-2,2)
#Graficando la función de seno
plt.plot(t,np.sin(t))
plt.show()
