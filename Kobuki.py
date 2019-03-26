import numpy as np
import math
import matplotlib.pyplot as plt
from matplotlib import animation

pas = 0.001
T = 10

class kobuki(object):
    def __init__(self, nom='Tortue', pos=np.array([0,0]), d=1, R=0.1):
        self.nom = nom  #Nom du robot
        self.pos = np.array([pos])  #Position initiale
        self.d = d  #Distance entre les roues
        self.R = R  #Rayon des roues
        self.orientation = np.array([0])

    def __str__(self):
        return self.nom+' le robot se trouve en '+str(self.pos[-1])+' et est orienté de '+str(self.orientation[-1])+' radians depuis l\'horizontale.'

    def mgd(self, Dg, Dd):
        """
        Evolution de la position et de l'orientation en fonction de l'evolution des angles des roues
        """
        dx = self.R*math.cos(self.orientation[-1])*(Dg+Dd)/2
        dy = self.R*math.sin(self.orientation[-1])*(Dg+Dd)/2
        dth = self.R*(Dd-Dg)/self.d
        return [dx, dy, dth]

    def mvt(self, Dg, Dd):
        [dx, dy, dth] = self.mgd(Dg, Dd)
        dpos = np.array([dx, dy])
        newpos = self.pos[-1] + dpos
        self.pos=np.vstack((self.pos, newpos))
        newOr = self.orientation[-1] + dth
        if newOr > math.pi: #Correction de l'angle
            newOr -= 2*math.pi
        if newOr < -math.pi:
            newOr += 2*math.pi
        self.orientation=np.append(self.orientation, newOr)
        return None

    def mvt2(self, Dg, Dd, dt): #Un pas de simulation en fonction des vitesses des deux roues
        [dx, dy, dth] = [x*dt for x in self.mgd(Dg, Dd)]
        dpos = np.array([dx, dy])
        newpos = self.pos[-1] + dpos
        self.pos=np.vstack((self.pos, newpos))
        newOr = self.orientation[-1] + dth
        if newOr > math.pi: #Correction de l'angle
            newOr -= 2*math.pi
        if newOr < -math.pi:
            newOr += 2*math.pi
        self.orientation=np.append(self.orientation, newOr)
        return None

    def goTo2(self, X, Y, dt):
        cX = self.pos[-1][0] #Position et orientation initiales
        cY = self.pos[-1][1]
        cO = self.orientation[-1]

        dX = X - cX
        dY = Y - cY

        dD = math.sqrt(dX*dX + dY*dY)
        dD0 = math.sqrt(dX*dX + dY*dY) #Normalisation des écarts

        while dD>dD0/100:
            cX = self.pos[-1][0]  # Position et orientation initiales
            cY = self.pos[-1][1]
            cO = self.orientation[-1]

            dX = X - cX
            dY = Y - cY

            dD = math.sqrt(dX * dX + dY * dY)

            dO = np.arctan2(dY, dX) - cO #Écart en angle
            if dO > math.pi: #Correction de l'écart
                dO -= 2*math.pi
            if dO < -math.pi:
                dO += 2*math.pi

            dd = dD/(self.R*math.pi)#*np.cos(dO)) #Diviser commande par D0/Vmax pour récupérer une vitesse
            dm = dO*self.d/self.R

            self.mvt2(-dm/2, dm/2, dt)
            self.mvt2(dd, dd, dt)

    def statPlot(self):
        """Plots all known previous positions"""
        X = [p[0] for p in self.pos]
        Y = [p[1] for p in self.pos]
        plt.plot(X,Y)
        plt.show()

    def goTo(self, X, Y):
        cX = self.pos[-1][0] #Position et orientation actuelles
        cY = self.pos[-1][1]
        cO = self.orientation[-1]

        dX = X - cX
        dY = Y - cY

        dD = math.sqrt(dX*dX + dY*dY) #Normalisation des écarts

        dO = np.arctan2(dY, dX) - cO #Écart en angle
        if dO > math.pi: #Correction de l'écart
            dO -= 2*math.pi
        if dO < -math.pi:
            dO += 2*math.pi

        dd = dD/self.R#*math.pi)#*np.cos(dO))
        dm = dO*self.d/self.R

        self.mvt(-dm/2, dm/2)
        self.mvt(dd, dd)


    def mci(self, x, y):
        """
        Instruit au robot de faire ce qu'il peut pour atteindre le point (x,y).
        """

        dX = x - self.pos[-1][0]
        dY = y - self.pos[-1][1]

        r = math.sqrt(dX*dX + dY*dY)
        theta = np.arctan2(dY, dX) - self.orientation[-1]

        if theta > math.pi: #Correction de l'écart
            theta -= 2*math.pi
        if theta < -math.pi:
            theta += 2*math.pi

        r = r/pas  #Vitesses
        theta = theta/pas

        G = 0.5*r/math.pi - theta*self.d
        D = 0.5*r/math.pi + theta*self.d

        if G > 2*math.pi and D > 2*math.pi:
            if G > D :
                d = G/D
                G = 2*math.pi
                D = G/d
            else :
                d = D/G
                D = 2 * math.pi
                G = D/d
        elif G > 2*math.pi :
            d = D/G
            G = 2*math.pi
            D = d*G
        elif D > 2*math.pi :
            d = G/D
            D = 2*math.pi
            G = d*D


        self.mvt2(G,D,pas)


    def movingPlot(self):
        line, = plt.plot([],[],'o')
        plt.xlim(-6,6)
        plt.ylim(-6,6)
        for i in range(len(self.pos)) :
            x = [self.pos[i][0]]
            y = [self.pos[i][1]]
            line.set_xdata(x)
            line.set_ydata(y)
            plt.pause(pas)
            plt.title(i)
        plt.show()

class essaim(object):
    def __init__(self, robots=np.array([]), mode=''):
        self.robots = robots
        self.mode = mode

    def anim8dPlot(self):
        line, = plt.plot([],[],'o')
        plt.xlim(-6,6)
        plt.ylim(-6,6)
        for i in range(len(self.robots[0].pos)) :
            x = [R.pos[i][0] for R in self.robots]
            y = [R.pos[i][1] for R in self.robots]
            line.set_xdata(x)
            line.set_ydata(y)
            plt.pause(pas)
            plt.title(i)
        plt.show()

    def anim9dPlot(self, T):
        fig = plt.figure()

        line, = plt.plot([], [], 'o')
        plt.xlim(-5, 5)
        plt.ylim(-5, 5)

        def init():
            line.set_data([], [])
            return line,

        def animate(i):
            x = [R.pos[i][0] for R in self.robots]
            y = [R.pos[i][1] for R in self.robots]
            line.set_xdata(x)
            line.set_ydata(y)
            return line,

        #Blit : on ne redessine que les parties qui ont changé
        anim = animation.FuncAnimation(fig, animate, init_func=init,
                                       frames=int(T/pas), interval=pas, blit=True)

        plt.show()

    def follow(self):
        """
        Les robots suivent le leader robot 0.
        Attention il faut actualiser la position du leader à chaque appel de cette fonction.
        """
        for R in self.robots:
            R.mci(self.robots[0].pos[-1][0],self.robots[0].pos[-1][1])


if __name__ == '__main__':
    A = kobuki()
    # t = np.linspace(0,2*math.pi,10)
    # for i in range(10):
    #     A.goTo(np.cos(t[i]),np.sin(t[i]))
    #     A.goTo(0,0)

    # x = -5 #Test mci primitif
    # y = 0
    #
    # dX = x - A.pos[-1][0]
    # dY = y - A.pos[-1][1]
    #
    # r = math.sqrt(dX * dX + dY * dY)
    #
    # while r>0.01:
    #     A.mci(x,y)
    #     dX = x - A.pos[-1][0]
    #     dY = y - A.pos[-1][1]
    #
    #     r = math.sqrt(dX * dX + dY * dY)
    #
    #
    # print(A)
    # A.statPlot()

    A = kobuki()
    B = kobuki()
    C = kobuki()
    D = kobuki()

    P = essaim(np.array([A,B,C,D]),'')

    for i in range(10000):
        P.robots[0].mci(5,0)
        P.robots[1].mci(-5,0)
        P.robots[2].mci(0,5)
        P.robots[3].mci(0,-5)

    P.anim9dPlot(10)