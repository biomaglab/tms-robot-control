import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize


# Rotation and Translation matrices to find pos on mTMS head
R = np.load('robot/resources/Rot.npy')
T = np.load('robot/resources/Tr.npy')
# Set tool origin and size bounds
# ..... based on size of mTMS

ORIG = np.array([0.0, 0.0, 0.06])
BOUNDS_X = (-0.08, 0.08)
BOUNDS_Y = (-0.08, 0.08)
BOUNDS_Z = (0, 0.06)

def _func(r, F, M, orig):
    '''
    Function to minimise objective function

    Returns: Norm of r x F - M (Np Array)  
    '''
    # Check if r is outside the box
    return np.linalg.norm(np.cross(r - orig, F) - M)


def find_r(F, M):
    '''
    Find point of application of force

    Returns: transformed point of application (list)
    '''

    # initial guess
    r0 = np.array([0.0, 0.0, 0.06])

    # find r that minimizes the objective function
    res = minimize(_func, r0, 
                   args=(F, M, ORIG), 
                   method='Nelder-Mead', 
                   bounds=(BOUNDS_X, BOUNDS_Y, BOUNDS_Z))
    
    r_min = res.x * 100  # multiply by 100 to get value in cm
    r_tran = R @ r_min + T

    # conditions to identify p.o.a
    if not (-8 <= r_tran[0] <= 8 and
            -8 <= r_tran[1] <= 8):
        r_tran[0], r_tran[1] = 0, 0

    else:
        # This checks that there is force in the z direction, such that the POA is only calculated when there is pressure
        if F[2] > 1:
            pass
        else:
            r_tran[0], r_tran[1] = 0, 0

    return [round(r_tran[i], 1) for i in range(0, len(r_tran))]

