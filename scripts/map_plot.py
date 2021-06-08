#!/usr/bin/env python

import rospy
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import animation
from geometry_msgs.msg import Point
import numpy as np

x = []
y = []
z = []
fig = plt.figure(figsize=(6,6))
ax = fig.add_subplot(111, projection='3d')
ax.scatter(x, y, z, c=z, cmap='viridis')

def callback_pointcloud(data):
    global x,y,z
    x.append(data.x)
    y.append(data.y)
    z.append(data.z)

def animate(i):
    ax.scatter(x, y, z, c=z, cmap='viridis')
    ax.set_xlim(np.min(x),np.max(x))
    ax.set_ylim(np.min(y),np.max(y))
    ax.set_zlim(60,180)

def map_plotter():
    rospy.init_node('map_plotter', anonymous=True)
    rospy.Subscriber("/pointCloud", Point, callback_pointcloud)
    ani = animation.FuncAnimation(fig, animate, interval=100)
    plt.show()		
		
if __name__ == '__main__':
    try:
        map_plotter()
    except rospy.ROSInterruptException:
        pass

