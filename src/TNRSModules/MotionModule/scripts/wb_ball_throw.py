import json
import numpy as np
import matplotlib.pyplot as plt, numpy as np
from mpl_toolkits.mplot3d import Axes3D

if __name__ == '__main__':
  path = '/home/sensei/team-nust-robocup/logs/Robots/Sim/WBBallThrow/log1/WBBallThrow.json'
  json_root = json.loads(open(path).read())
  l_arm_position = []
  r_arm_position = []
  for position in json_root['eeTraj']['lArm']:
    l_arm_position.append(position)
  for position in json_root['eeTraj']['rArm']:
    r_arm_position.append(position)
  #print np.array(l_arm_position)
  l_arm_position = np.array(l_arm_position)
  r_arm_position = np.array(r_arm_position)
  
  fig = plt.figure()
  ax = plt.axes(projection='3d')

  ax.set_xlabel('x')
  ax.set_ylabel('y')
  ax.set_zlabel('z')

  ax.scatter(r_arm_position[:,0], r_arm_position[:,1], r_arm_position[:,2], zdir='z', c= 'blue')
  ax.scatter(l_arm_position[:,0], l_arm_position[:,1], l_arm_position[:,2], zdir='z', c= 'green')
  plt.show()
  
  
