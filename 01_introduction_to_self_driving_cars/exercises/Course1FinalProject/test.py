import numpy as np

pos = np.array([[-192.37132263183594, 68.01093292236328]])
waypoints=  np.array([[-181.37122534340753, 67.10779956443334]])

ct = np.sum(pos-waypoints, axis=1)
print(np.arctan(ct)*180/np.pi)