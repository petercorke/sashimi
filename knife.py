import spatialmath as sm
import spatialmath.base as smb
import numpy as np
import matplotlib.pyplot as plt
from scipy import interpolate
import pickle

# first 2 points are on the top of the handle
# point 3 is on the blade closest to the handle
# last point is the tip of the blade

with open("perim.p", "rb") as f:
    pts = pickle.load(f)
handle = pts[:2]
knife = np.array(pts[2:]).T
scale = 300.0 / smb.norm(knife[:, -1] - knife[:, 0])
theta = np.arctan2(handle[1][1] - handle[0][1], handle[1][0] - handle[0][0])

knife = knife - np.array(handle[1]).reshape((2, 1))
knife = scale * smb.rot2(-theta) @ knife

x, y = knife

# plt.plot(x, y, "o")
# plt.gca().set_aspect("equal")
# plt.grid(True)
# plt.show(block=True)

f = interpolate.interp1d(x, y, kind="quadratic")
x = np.arange(min(x), max(x), 1)
y = f(x)
plt.plot(x, y)
plt.gca().set_aspect("equal")

for k in range(0, len(x), 10):
    dy = y[k + 1] - y[k]

    vx = (x[k + 1] - x[k], 0, y[k + 1] - y[k])
    vz = np.cross(vx, [0, 1, 0])
    vz = smb.unitvec(vz) * 5
    print(k, dy)

    plt.quiver(x[k], y[k], vz[0], vz[2])

plt.ylim(0, 80)
plt.show(block=True)
