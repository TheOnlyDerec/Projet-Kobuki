import matplotlib.pyplot as plt
from matplotlib import animation
import numpy as np

fig = plt.figure()

line, = plt.plot([], [], 'o')
line2, = plt.plot([],[],'x')
line3, = plt.plot([],[])
plt.xlim(-5, 5)
plt.ylim(-5, 5)

def init(): #Initialisation du plot
    line.set_data([], [])
    line2.set_data([],[])
    line3.set_xdata([1, 1, 2, 2, 1])
    line3.set_ydata([1, 2, 2, 1, 1])
    return [line, line2, line3]

X=np.linspace(0,1,10000)
Y=np.cos(X)
Y2=np.sin(X)

def animate(i): #Actualisation des données affichées pour la frame i
        x = [X[i]]
        y = [Y[i]]
        line.set_xdata([x])
        line.set_ydata([y])
        x2=[X[i]]
        y2=[Y2[i]]
        line2.set_xdata([x2])
        line2.set_ydata([y2])

        return [line, line2, line3]

#Option blit : on ne redessine que les parties qui ont changé
anim = animation.FuncAnimation(fig, animate, init_func=init,
                                       frames=10000, interval=0.1, blit=True)

plt.show()