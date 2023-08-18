from math import cos
import numpy as np
T=100
x = [i for i in range(4 * T)]
y = [1.2 * 10 *(-cos(3.14 * 2 * (i % T) /T) + 1) /2 for i in x]
import matplotlib.pyplot as plt
plt.plot(x,y)
plt.show()