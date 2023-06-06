# 0 : 8 inches sin-1 (0)
# facing my left and right
#
# left side:
# 1 :  sin-1 (3inches/8)
# 4: sin-1 (2.25/8)
# 6: sin-1 (1/8)
# 8: sin-1 (4/8)
# 9: sin-1(2.5/8)
# right side
# 2 :  sin-1 (2.75inches/8)
# 3: sin -1 (3.75/8)
# 5: sin-1(2/8)
# 7: sin-1(1.5/8)

import matplotlib.pyplot as plt
import math
import numpy as np

# # 0 : sin-1 (0)
# facing my left and right
#
# left side:
# 1: sin-1(4.5/11)
# 2: sin-1(4.5/8.5)
# 3: sin-1(3/10.5)
# 4: sin-1(3/11.5)
# 5: sin-1(1/11.75)
# 6: sin-1(0.75, 5.5)
# 7: sin-1 (3.5/ 11.25)
# 8: sin-1(2/7)
# 9: sin-1(4.25/12)
# right side


data_o = [0, 4.5, 4.5, 3, 3, 1, 0.75, 3.5, 2, 4.25]
data_h = [10, 11, 8.5, 10.5, 11.5, 11.75, 5.5, 11.25, 7, 12]
# px = [352.33, 544.02, 111.47, 34.55, 471.55,
#       166.49, 383.94, 194.25, 599.16, 501.59]
# py = [335.60, 361.13, 364.77, 366.16, 357.89,
#       362.11, 362.47, 360.42, 362.50, 368.76]
# blob_size = [47.91, 57.10, 54.02, 55.16,
#              50.43, 52.75, 52.84, 51.37, 54.82, 56.27]
px = [321.03, 606.36, 45.65, 152.20, 512.79,
      330.66, 261.07, 215.07, 513.18, 581.73]
py = [282.87, 286.60, 301.68, 289.68, 281.12,
      288.50, 339.89, 289.95, 317.66, 290.37]
blob_size = [40.74, 42.93, 51.65, 40.64,
             35.44, 31.89, 71.76, 33.07, 56.01, 34.53]

dist_list = []
radian_list = []
deg_list = []
PXYList = []
center_u = []
Res_blob = []
duck_loc = []

# converting pixels to inches:
# PPI resolution (640x480 pixels) => 1 inch = 100 pixels


# def getDistance(data_h, data_o):
#     # data_h => center of camera
#     # data_o => the side measured
#     dist = math.sqrt(data_h**2 - data_o**2)
#     dist_list.append(dist)


def pixels(x, y):
    # print(f"Pixel x: {x} & Pixel y: {y}")
    res_x = x/100.0
    res_y = y/100.0
    # print(f"res_x: {res_x} & res_y: {res_y}")
    PXYList.append((res_x, res_y))

# (u vs theta)


def center(x):
    res = abs(x - 320)
    center_u.append(res)
    # print(res)


def linApprox(x, size):
    # estimating
    # units in inches
    duck_size = 2
    # radians inside of radian_list
    duck_dist = 0
    # print()
    # print("pos:")
    for i in range(len(x)):
        # duck_dist = duck_size * size[i] / \
        #     (2 * math.tan(radian_list[i] / 2) * x[i])
        # dist_list
        pos = (x[i] * deg_list[i])-(((size[i]/100.0)+1.5)/2)
        # print(pos)
        duck_loc.append(pos)


def angleMeagured(o, h):
    # print(f"Opp: {o} & Hyp: {h}")
    # print("Radians: ")
    res_rad = math.asin(o/h)
    # print(res_rad)
    # print("Degrees:")
    res_deg = math.degrees(math.asin(o/h))
    # print(res_deg)
    # print()
    radian_list.append(res_rad)
    deg_list.append(res_deg)


getdegree = math.degrees(math.asin(3.5/8))
print(getdegree)

for i in range(len(px)):
    # print(i)
    angleMeagured(data_o[i], data_h[i])
    pixels(px[i], py[i])
    center(px[i])
    # getDistance(data_h[i], data_o[i])
    # pixelsToIn = blob_size[i]
    Res_blob.append(1/blob_size[i])
    linApprox(center_u, blob_size)


# print()
# print(PXYList)
# print()

print(center_u)
print(deg_list)
# print()
# print(center_u)
# print(radian_list)
# print(deg_list)
x = np.array(deg_list)
y = np.array(center_u)
slope, intercept = np.polyfit(x, y, 1)

yeq = f"{slope} * x + {intercept}"
print("equation 1: ", yeq)


# (u vs theta)
fig1 = plt.figure(1)
plt.scatter(x, y)
eq = (slope*x + intercept)
plt.plot(x, eq, color="red")
plt.title("U vs Theta")
plt.ylabel("Angle (theta)")
plt.xlabel("Pixels (u)")
plt.show()

fig2 = plt.figure(2)
# # 1/blob size vs dist

plt.scatter(data_h, Res_blob)
# print(dist_list)
# approx with reg line
x2 = np.array(data_h)
y2 = np.array(Res_blob)
slope2, intercept2 = np.polyfit(x2, y2, 1)
y2eq = f"{slope2} * x + {intercept2}"
print("equation 2: ", y2eq)
# eq2 = (slope*x2 + intercept2)
plt.plot(x2, slope2*x2 + intercept2, color="blue")
plt.grid(True)
plt.title("1/Sb vs d")
plt.ylabel("1/Sb")
plt.xlabel("Distance")
plt.show()


# function that receives u and S_b


def UlinApprox(u):
    # u vs theta
    # y = 0.0805x + 3.1146
    res = (0.0805 * u) + 3.1146
    print("U approx: ", res)
    return res


def SbLinApprox(s):
    # 1/sb vs dist
    # y = 0.0025x - 0.0002
    Sb = 1/s
    res = (0.0025 * Sb) - 0.0002
    print("Sb approx: ", res)
    return res
