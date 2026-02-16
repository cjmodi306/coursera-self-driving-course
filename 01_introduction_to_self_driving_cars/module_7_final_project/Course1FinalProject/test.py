import numpy as np

#pos = np.array([[-192.37132263183594, 68.01093292236328]])
#waypoints=  np.array([[-181.37122534340753, 67.10779956443334]])

pos = np.array([[1,1]])
waypoints=  np.array([[-1, 1]])


#ct = (-waypoints[0,0]*pos[0,1] + waypoints[0,1]*pos[0,0]) / np.linalg.norm(waypoints)

ct = np.arctan2(pos[0,1] - waypoints[0,1], pos[0,0] - waypoints[0,0])
print(ct*180/np.pi)