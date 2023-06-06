import math
import numpy as np
import matplotlib.pyplot as plt

o = 20
h = 20
print(math.asin(o/h))


x = [20, 12, 24, 32, 53, 64, 32, 21]
y = [23, 22, 64, 75, 21, 32, 53, 54]

model = np.polyfit(x, y, 1)
plt.scatter(x, y)

plt.show()
