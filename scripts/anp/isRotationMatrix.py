import numpy as np
from scipy.linalg import svd

def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6

def ToRotationMatrix(R):
    U, _, Vt = svd(R)
    return np.dot(U, Vt)