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
        self.orientation = np.array([0]) #Oientation initiale

    def __str__(self):
        return self.nom+' le robot se trouve en '+str(self.pos[-1])+' et est orienté de '+str(self.orientation[-1])+' radians depuis l\'horizontale.'

    def mgd(self, Dg, Dd):
        """
        Vitesse en translation et rotation en fonction de la vitesse angulaire des roues.
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

    def mvt2(self, Dg, Dd, dt):
        """
        Effectue un pas de simulation d'une durée dt lors duquel les roues ont des vitesses Dg et Dd.
        """
        [dx, dy, dth] = [x*dt for x in self.mgd(Dg, Dd)]

        dpos = np.array([dx, dy])
        newpos = self.pos[-1] + dpos
        self.pos=np.vstack((self.pos, newpos)) #On utilise vstack pour ajouter une ligne à l'historique des positions.

        newOr = self.orientation[-1] + dth

        if newOr > math.pi: #Correction de l'angle
            newOr -= 2*math.pi
        if newOr < -math.pi:
            newOr += 2*math.pi

        self.orientation=np.append(self.orientation, newOr) #Ajout de ka nouvelle orientation à l'historique

    def goTo2(self, X, Y, dt):
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

        G = 0.5*r/math.pi - theta*self.d #MCI
        D = 0.5*r/math.pi + theta*self.d

        if G > 2*math.pi and D > 2*math.pi: #On limite les vitesses des roues aux valeurs max. Ici on suppost un tour par seconde.
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


        self.mvt2(G,D,pas) #Finalement on simule.

    def MCI(self, v, vt):
        """
        On donne une vitesse v et une vitesse angulaire vt. On retourne les vitesses des roues gauche et droite nécessaires pour les atteindre, dans les limites du possible.
        L'intérêt de cette fonction par rapport à PtCible est qu'on peut l'utiliser en conjonction avec des correcteurs.
        """
        G = 0.5 * r / math.pi - theta * self.d  # MCI
        D = 0.5 * r / math.pi + theta * self.d

        if G > 2 * math.pi and D > 2 * math.pi:  # On limite les vitesses des roues aux valeurs max. Ici on suppost un tour par seconde.
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

        for i in range(len(self.pos)): #Tracer de toutes les positons.
            x = [self.pos[i][0]]
            y = [self.pos[i][1]]
            line.set_xdata(x)
            line.set_ydata(y)
            plt.pause(pas)
            plt.title(i) #On donne comme titre à l'image le numéro de la frame.

        plt.show()

class essaim(object):
    def __init__(self, robots=np.array([]), mode=''):
        """
        On donne les robots et le mode (sous la forme d'une chaîne de char.). On peut imaginer plusieurs modes selon ce qu'on veut faire.
        """
        self.robots = robots
        self.mode = mode

    def anim9dPlot(self, N): #Ajouter la possibilité de skip les premières frames de l'animation
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
    B = kobuki(pos=np.array([-1,-1]))
    C = kobuki(pos=np.array([-2,0]))
    D = kobuki(pos=np.array([0,-2]))

    P = essaim(np.array([A,B,C,D]),'')

    while np.sqrt((P.robots[1].pos[-1][0]-P.robots[0].pos[-1][0])**2+(P.robots[1].pos[-1][1]-P.robots[0].pos[-1][1])**2)>0.01:
        P.robots[0].PtCible(5,5)
        P.follow()

    P.anim9dPlot(len(P.robots[0].pos))