import numpy as np
import matplotlib.pyplot as plt
from Controller import *

pas = 0.001

class MoteurCC(object):
    def __init__(self, R=1, kc=0.01, f=0.01, J=0.01, ke=0.1, Um=1):
        """
        Vitesses = Omegas
        """
        self.R = 1000
        self.kc = kc
        self.f = f
        self.J = J
        self.ke = ke
        self.Um = Um
        self.K = kc*Um/(f+ke*kc)
        self.tau = R/(R*f+ke*kc)
        self.vitesses = np.array([0])
        self.couples = np.array([0])

    def step(self, dt=pas):
        """
        Un pas de simulation de dt secondes.
        """
        newVit = self.Um*self.kc*dt+self.R*self.J*self.vitesses[-1]
        newVit = newVit/(self.R*self.J + self.R*self.f*dt+self.ke*self.kc*dt)

        self.vitesses = np.append(self.vitesses, newVit)

        i = (self.Um - self.ke*self.vitesses[-1])/self.R
        np.append(self.couples, self.kc*i)


if __name__ == '__main__':
    ########
    ###Test moteur contrôllé
    # M = MoteurCC(Um=10)
    # for i in range(9999):
    #     C = ControllerPI(M.vitesses, 1, 0, 1000, 4000)
    #     M.Um = C
    #     M.step(pas)
    # plt.plot(np.linspace(0,10,10000),M.vitesses)
    # plt.show()
    ########

    M=MoteurCC()
    for i in range(10000):
        M.step(0.001)
        print(i)
    plt.plot(np.linspace(0,10,10001),M.vitesses)
    plt.show()

