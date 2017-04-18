#'/home/aditya/catkin_ws/src/pico_flexx_pcl/caentroid_data_11april.csv'

import matplotlib.pyplot as plt
from matplotlib import style
import sys
import csv
import numpy as np
from numpy import genfromtxt

style.use('ggplot')

z = np.loadtxt('/home/aditya/catkin_ws/src/pico_flexx_pcl/caentroid_data_11april.csv',unpack=True,delimiter=',')

index = list(range(len(z)))
print len(index)
#print len(x)
#print len(y)
print len(z)
plt.plot(index,z)

plt.title('distance estimation')
plt.ylabel('distance axis')
plt.xlabel('time')

plt.show()
