# Read the file and split by ';' and '\n'
list_ = open("calibration2.txt").read().split(';\n')

# Split each string by comma and convert each part into a list
final_list = [item.split(',') for item in list_]

x_values = []
y_values = []
for item in final_list:
    x_values.append(float(item[0]))
    y_values.append(float(item[1]))

import matplotlib.pyplot as plt
import numpy as np

xpoints = np.array(x_values)
ypoints = np.array(y_values)

plt.plot(xpoints, ypoints)
plt.show()