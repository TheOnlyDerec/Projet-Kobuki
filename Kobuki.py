import numpy as np
import math
import matplotlib.pyplot as plt
from matplotlib import animation
import random
from Controller import *

pas = 0.001 #Pas de simulation
T = 10 #Temps d'animation

class kobuki(object):
    def __init__(self, nom='Tortue', pos=np.array([0,0]), orientation=np.array([0]), d=1, R=0.1):
        self.nom = nom  #Nom du robot
        self.pos = np.array([pos])  #Position initiale
        self.d = d  #Distance entre les roues
        self.R = R  #Rayon des roues
        self.orientation = orientation #Oientation initiale

    def __str__(self):
        return self.nom+' le robot se trouve en '+str(self.pos[-1])+' et est orienté de '+str(self.orientation[-1])+' radians depuis l\'horizontale.'

    def MCD(self, Dg, Dd):
        """
        Vitesse en translation et rotation en fonction de la vitesse angulaire des roues.
        """
        dx = self.R*math.cos(self.orientation[-1])*(Dg+Dd)/2
        dy = self.R*math.sin(self.orientation[-1])*(Dg+Dd)/2
        dth = self.R*(Dd-Dg)/self.d
        return [dx, dy, dth]

    def mvt2(self, Dg, Dd, dt):
        """
        Effectue un pas de simulation d'une durée dt lors duquel les roues ont des vitesses Dg et Dd.
        """
        [dx, dy, dth] = [x*dt for x in self.MCD(Dg, Dd)]

        dpos = np.array([dx, dy])
        newpos = self.pos[-1] + dpos
        self.pos=np.vstack((self.pos, newpos)) #On utilise vstack pour ajouter une ligne à l'historique des positions.

        newOr = self.orientation[-1] + dth

        if newOr > math.pi: #Correction de l'angle
            newOr -= 2*math.pi
        if newOr < -math.pi:
            newOr += 2*math.pi

        self.orientation=np.append(self.orientation, newOr) #Ajout de ka nouvelle orientation à l'historique

    def goTo(self, X, Y, dt):
        """
        Le robot s'approche du point X,Y avec une tolérance d'un pourcent de la distance originelle.
        Cette fonction est surtout un reliquat du début du projet, on lui préfère maintenant d'autres modes de déplacement.
        """
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
        """
        Tracer statique dans le plan des positions du robot.
        """
        X = [p[0] for p in self.pos]
        Y = [p[1] for p in self.pos]
        plt.plot(X,Y)
        plt.show()

    def PtCible(self, x, y):
        """
        Instruit au robot de faire ce qu'il peut pour atteindre le point (x,y).
        """

        dX = x - self.pos[-1][0]
        dY = y - self.pos[-1][1]

        r = math.sqrt(dX*dX + dY*dY) #Calcul de la distance linéaire et angulaire entre le robot et la cible.
        theta = np.arctan2(dY, dX) - self.orientation[-1]

        if theta > math.pi: #Correction de l'écart angulaire.
            theta -= 2*math.pi
        if theta < -math.pi:
            theta += 2*math.pi

        r = r/pas  #Passage des distances aux vitesses.
        theta = theta/pas

        [G,D]=self.MCI(r,theta)

        self.mvt2(G,D,pas) #Finalement on simule

    def PtCiblePI(self, x, y):
        dX = x - self.pos[-1][0]
        dY = y - self.pos[-1][1]

        r = math.sqrt(dX * dX + dY * dY)  # Calcul de la distance linéaire et angulaire entre le robot et la cible.
        theta = np.arctan2(dY, dX)
        dtheta = theta - self.orientation[-1]

        if theta > math.pi:  # Correction de l'écart angulaire.
            theta -= 2 * math.pi
        if theta < -math.pi:
            theta += 2 * math.pi

        while dtheta > 0.01*np.pi or dtheta < -0.01*np.pi :
            V = ControllerPI(self.orientation-theta, 0, 15, 1000, 4000)
            self.mvt2(-V,V,pas)

            dX = x - self.pos[-1][0]
            dY = y - self.pos[-1][1]

            dtheta = theta - self.orientation[-1]

        R = np.array([])
        print(self.orientation[-1])

        for i in range(10000):
        #while r > 0.01 or r <-0.01 :
            dX = - self.pos[-1][0] + x
            dY = - self.pos[-1][1] + y

            R = np.append(R, -np.sqrt(dX*dX+dY*dY))

            r = R[-1]

            V = ControllerPI(R, 0, 15, 1000, 4000)

            print(V)

            self.mvt2(V, V, pas)

    def MCI(self, r, theta):
        """
        On donne une vitesse v et une vitesse angulaire vt. On retourne les vitesses des roues gauche et droite nécessaires pour les atteindre, dans les limites du possible.
        L'intérêt de cette fonction par rapport à PtCible est qu'on peut l'utiliser en conjonction avec des correcteurs.
        """
        G = r / self.R - 0.5 * theta * self.d / self.R  # MCI
        D = r / self.R + 0.5 * theta * self.d / self.R

        if G > 2 * math.pi and D > 2 * math.pi:  # On limite les vitesses des roues aux valeurs max. Ici on suppose un tour par seconde.
            if G > D:
                d = G / D
                G = 2 * math.pi
                D = G / d
            else:
                d = D / G
                D = 2 * math.pi
                G = D / d
        elif G > 2 * math.pi:
            d = D / G
            G = 2 * math.pi
            D = d * G
        elif D > 2 * math.pi:
            d = G / D
            D = 2 * math.pi
            G = d * D

        return [G,D]


    def movingPlot(self):
        """
        Tracer animé des positions du robot. Il sert surtour à illustrer une méthode alternative (moins efficace) d'effectuer un plot animé.
        """
        line, = plt.plot([],[],'o')
        plt.xlim(-6,6)
        plt.ylim(-6,6)

        for i in range(len(self.pos)): #Tracer de toutes les positions.
            x = [self.pos[i][0]]
            y = [self.pos[i][1]]
            line.set_xdata(x)
            line.set_ydata(y)
            plt.pause(pas)
            plt.title(i) #On donne comme titre à l'image le numéro de la frame.

        plt.show()

    def listen(self):
        """
        Permet la commande à distance d'un robot. Pendant num pas de simulation, il se dirige vers le point (x,y).
        """
        UDPSock = socket.socket(type=socket.SOCK_DGRAM)
        listen_addr = (socket.gethostname(), 12345)
        UDPSock.bind(listen_addr)
        while True:
            data, addr = UDPSock.recvfrom(1024)
            (num, x, y) = unpack('idd', data)
            for i in range(1,num):
                self.PtCible(x,y)

class essaim(object):
    def __init__(self, robots=np.array([]), mode=''):
        """
        On donne les robots et le mode (sous la forme d'une chaîne de char.). On peut imaginer plusieurs modes selon ce qu'on veut faire.
        """
        self.robots = robots
        self.mode = mode

    def animatedPlot(self, N): #Ajouter la possibilité de skip les premières frames de l'animation
        """
        Animation du mouvement des robots de l'essaim. On voit les N premières positions des robots.
        """
        fig = plt.figure()

        line, = plt.plot([], [], 'o') #On créée un plot qui sert d'arrière plan à l'animation
        plt.xlim(-5, 5)
        plt.ylim(-5, 5)

        def init(): #Initialisation du plot
            line.set_data([], [])
            return line,

        def animate(i): #Actualisation des données affichées pour la frame i
            x = [R.pos[i][0] for R in self.robots]
            y = [R.pos[i][1] for R in self.robots]
            line.set_xdata(x)
            line.set_ydata(y)
            return line,

        #Option blit : on ne redessine que les parties qui ont changé
        anim = animation.FuncAnimation(fig, animate, init_func=init,
                                       frames=N, interval=pas, blit=True)

        plt.show()

    def follow(self):
        """
        Les robots suivent le leader, robot 0.
        Attention il faut actualiser la position du leader à chaque appel de cette fonction.
        """
        for R in self.robots:
            if not R==self.robots[0]:
                R.PtCible(self.robots[0].pos[-1][0],self.robots[0].pos[-1][1])


class environnement(object):
    def __init__(self, obstacles=np.array([]), essaim=essaim(), objets=np.array([]), xl=(-5,5), yl=(-5,5)):
        """
        Les obstacles doivent être donnés en tant qu'éléments d'un array. Ils sont considérés rectangulaires,
                        leur coin supérieur gauche étant codé en premier et le coin inférieur droite en deuxième.
        Les objets sont des couples stockés dans un array, ils correspondent à des coordonnées.
        """
        self.obs=obstacles
        self.essaim=essaim
        self.obj=objets
        self.xl=xl
        self.yl=yl
        self.porte=np.array([False for R in self.essaim.robots])

    def envPlot(self, N):
        """
        Quasi identique à la méthode animatedPlot d la classe Kobuki. Permet cependant de voir les obstacles et les objets.
        Il faut ajouter la gestion des obstacles dans le tracé sous formes de listes.
        """
        fig = plt.figure()

        line, = plt.plot([], [], 'o') #On créée la ligne2D des robots
        line2, = plt.plot([], [], 'x') #Objets
        line3, = plt.plot([], []) #Obstacles

        plt.xlim(self.xl[0], self.xl[1])
        plt.ylim(self.yl[0], self.yl[1])



        def init(): #Initialisation du plot
            line.set_data([], [])
            line2.set_data([], [])
            line3.set_data([], [])
            return line,

        def animate(i): #Actualisation des données affichées pour la frame i
            x = [R.pos[i][0] for R in self.robots]
            y = [R.pos[i][1] for R in self.robots]
            line.set_xdata(x)
            line.set_ydata(y)
            xo = [O[0] for O in self.obj]
            yo = [O[1] for O in self.obj]
            line2.set_xdata(xo)
            line2.set_ydata(yo)
            return line, line2, line3

        #Option blit : on ne redessine que les parties qui ont changé
        anim = animation.FuncAnimation(fig, animate, init_func=init,
                                       frames=N, interval=pas, blit=True)

        plt.show()

    def actu(self):
        """
        Fonction qu'il faut théoriquement appeler à chaque appel. Elle sert à gérer les collisions et le transport d'objets.
        """


if __name__ == '__main__':

    A = kobuki()
    B = kobuki(pos=np.array([-1, -1]))
    C = kobuki(pos=np.array([-2, 0]))
    D = kobuki(pos=np.array([0, -2]))
    P = essaim(np.array([A,B,C,D]),'')


    #######
    # for i in range(10000):
    #     for R in P.robots:
    #         R.PtCible(random.uniform(-5,5), random.uniform(-5,5))
    # P.animatedPlot(10000)
    # print(len(P.robots[0].pos))
    ########

    ########
    ###Test de plusieurs fonctions. Les robots d'un essaim suivent leur leader pendant son mouvement.
    #
    # while np.sqrt((P.robots[1].pos[-1][0]-P.robots[0].pos[-1][0])**2+(P.robots[1].pos[-1][1]-P.robots[0].pos[-1][1])**2)>0.01:
    #     P.robots[0].PtCible(5,5)
    #     P.follow()
    #
    # P.animatedPlot(len(P.robots[0].pos)
    ########

    ########
    # P=essaim(np.array([A]),'')
    # vx=np.linspace(0,10,10000)
    # vy=np.sin(vx)
    # vr=np.sqrt(np.multiply(vx,vx)+np.multiply(vy,vy))
    # vt=np.arctan2(vy,vx)
    # for i in range(9999):
    #     [G,D]=P.robots[0].MCI(vr[i],vt[i])
    #     P.robots[0].mvt2(G,D,pas)
    # P.animatedPlot(10000)

    ########
    ###Test de la vitesse max des roues dans la méthode MCI
    # for i in range(10000):
    #     [G,D]=P.robots[0].MCI(100,0)
    #     P.robots[0].mvt2(G,D,pas)
    # print(P.robots[0].pos[-1][0])
    ########


    E = kobuki()
    F = essaim(np.array([E]),'')
    F.robots[0].PtCiblePI(5,5)
    F.animatedPlot(len(F.robots[0].pos))