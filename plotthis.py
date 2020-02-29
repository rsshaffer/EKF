import numpy as np
import matplotlib.pyplot as plt


#This python script plots the simulated sensor measurements in m.txt and the output of the EKF Filtered.txt


filename = "Filtered.txt"
filename2 = "m.txt"

data = np.loadtxt(filename, delimiter = ",")
sensor = np.loadtxt(filename2, delimiter = " ")




plt.plot(sensor[:,0],sensor[:,1],'ro', data[:,0],data[:,1],'g-*')
plt.show()