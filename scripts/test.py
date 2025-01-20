import numpy as np
Rz = np.array([
                    [0, 1, 0, 0],
                    [-1, 0, 0, 0],
                    [0, 0, 1, 0],
                    [0,  0, 0, 1]
                ])
print(np.linalg.inv(Rz))