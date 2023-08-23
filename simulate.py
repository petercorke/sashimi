import roboticstoolbox as rtb
import swift
from spatialmath import SE3
import spatialmath.base as smb
import numpy as np
from scipy import interpolate
import pickle

# ------------------------------------------------------------------------- #

# load the robot model

robot = rtb.models.URDF.Panda()
# print(robot)

# ------------------------------------------------------------------------- #

# load the knife model
# knife is defined in the xy plane, x is positive along the blade, y is positive distance
# back of blade to cutting edge

with open("perim.p", "rb") as f:
    pts = pickle.load(f)
handle = pts[:2]
knife = np.array(pts[2:]).T
scale = 300.0 / smb.norm(knife[:, -1] - knife[:, 0])
theta = np.arctan2(handle[1][1] - handle[0][1], handle[1][0] - handle[0][0])

knife = knife - np.array(handle[1]).reshape((2, 1))
knife = scale * smb.rot2(-theta) @ knife

x, y = knife

f = interpolate.interp1d(x, y, kind="quadratic")
x = np.arange(min(x), max(x), 1)
y = f(x)

# ------------------------------------------------------------------------- #

# compute end-effector poses so that knife slices past T_fish
#  * moving in the world -x direction
#  * moving downward from a height of z_max about T_fish to 0
#  * optinally keeping the knife tange parallel to the world x-axis

# pose of fish fillet, with its z-axis down, x same as world
T_fish = SE3.Trans(0.2, 0.5, 0.2) * SE3.Rx(np.pi)
z_max = 0.020  # thickness of the fillet

T_e = SE3.Empty()

for k in range(0, len(x), 10):  # for each point on the knife blade
    # approximate tangent, this is the x-axis of the knife frame
    vx = (x[k + 1] - x[k], 0, y[k + 1] - y[k])

    # compute the y- and z-axes of the knife frame
    vy = [0, 1, 0]
    vz = np.cross(vx, vy)
    vz = smb.unitvec(vz) * 5
    # print(k, dy)

    # the tangent computation is pretty noisy and this leads to considerable robot arm
    # motion.  For now just ignore making the blade tangent to the ground plane
    # T_knife = SE3.OA(vy, vz).inv()  # set orientation of knife frame
    T_knife = SE3()

    # set position of knife frame
    #  we make the knife move downward by making the blade less wide
    z = float(k) / len(x) * z_max
    T_knife.t = [x[k] / 1000.0, 0, y[k] / 1000.0 - z]

    # solve for T_e for this point on the knife blade: T_e * T_knife = T_fish
    T_e.append(T_knife.inv() * T_fish)

T_e.printline()

# solve inverse kimematics
sol = robot.ikine_LM(T_e)
# print(sol)

# ------------------------------------------------------------------------- #

# animate the robot motion, would be nice to add a knife to the Swift environment

env = swift.Swift()
env.launch(realtime=True)
env.add(robot)

for q in sol.q:
    robot.q = q
    env.step(0.1)
# env.hold()
