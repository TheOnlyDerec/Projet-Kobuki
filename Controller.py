import numpy as np
import math

def ControllerP(positions, cible, kp):
    comm  = kp*(-positions[-1]+cible)
    return comm


def ControllerPI(positions, cible, kp, Te, ki):
    """
    :param Te: dt
    """
    comm = kp*(-positions[-1]+cible) + Te*(-np.ndarray.sum(positions-cible))/ki
    return comm


#if __name__ == '__main__':
