import numpy as np
import math
import matplotlib.pyplot as plt

class kobuki(object):
    def __init__(self, nom='Tortue', pos=np.array([0,0]), d=1, R=0.1, vitesse=np.array([0,0])):
        self.nom = nom  #Nom du robot
        self.pos = np.array([pos])  #Position initiale
        self.d = d  #Distance entre les roues
        self.R = R  #Rayon des roues
        self.orientation = np.array([0])
        self.vitesse = vitesse

    def __str__(self):
        return self.nom+' le robot se trouve en '+str(self.pos[-1])+' et est orienté de '+str(self.orientation[-1])+' radians depuis l\'horizontale.'

    def mgd(self, Dg, Dd):
        """
        Returns evolution in Xwise, Ywise position and orientation for a change in wheel angle of Dg (left) and Dd (right).
        Dg and Dd are the changes in the angle between a wheel ray and the ground.
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

    def mvt2(self, Dg, Dd, dt): #Avec des vitesses
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


class essaim(object):
    def __init__(self, robots=np.array([]), leader=kobuki(), mode=''):
        self.robots = robots
        self.leader = leader
        self.mode = mode




if __name__ == '__main__':
    A = kobuki()
    print(A)
    # t = np.linspace(0,2*math.pi,10)
    # for i in range(10):
    #     A.goTo(np.cos(t[i]),np.sin(t[i]))
    #     A.goTo(0,0)
    for i in range(1000):
        A.mvt2(math.pi,2*math.pi,0.001)
    A.statPlot()
    print(A)