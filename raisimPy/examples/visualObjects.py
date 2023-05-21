import os
import numpy as np
import raisimpy as raisim
import math
import time

# create raisim world
rscDir = os.path.dirname(os.path.abspath(__file__)) + "/../../rsc/"
raisim.World.setLicenseFile(rscDir + "activation.raisim")
world = raisim.World()

# create and launch raisim server
server = raisim.RaisimServer(world)
server.launchServer(8080)

# ground
ground = world.addGround()

# primitives
visSphere = server.addVisualSphere("v_sphere", 1, 1, 1, 1, 1)
visBox = server.addVisualBox("v_box", 1, 1, 1, 1, 1, 1, 1)
visCylinder = server.addVisualCylinder("v_cylinder", 1, 1, 0, 1, 0, 1)
visCapsule = server.addVisualCapsule("v_capsule", 1, 0.5, 0, 0, 1, 1)
visMesh = server.addVisualMesh("v_mesh", rscDir + "monkey/monkey.obj")
varrow_x = server.addVisualArrow("v_arrow_x", 1, 2, 1, 0, 0, 1)
varrow_y = server.addVisualArrow("v_arrow_y", 1, 2, 0, 1, 0, 1)
varrow_z = server.addVisualArrow("v_arrow_z", 1, 2, 0, 0, 1, 1)

# positions and orientations
visSphere.setPosition(np.array([2, 0, 0]))
visCylinder.setPosition(np.array([0, 2, 0]))
visCapsule.setPosition(np.array([2, 2, 0]))
quat = np.array([0.6, 0.2, -0.6, 0.1])
quat = quat / np.linalg.norm(quat)
visMesh.setOrientation(quat)
visMesh.setPosition(np.array([2,-2,1]))
varrow_x.setPosition(np.array([0,0,4]))
varrow_y.setPosition(np.array([0,0,4]))
varrow_z.setPosition(np.array([0,0,4]))
xDir = np.array([0.70710678118, 0, 0.70710678118, 0])
yDir = np.array([0.70710678118, -0.70710678118, 0, 0])
varrow_x.setOrientation(xDir)
varrow_y.setOrientation(yDir)

# articulated system
laikago = server.addVisualArticulatedSystem("v_laikago", rscDir + "laikago/laikago.urdf")
laikago.setGeneralizedCoordinate(np.array([0, 0, 5.54, 1.0, 0.0, 0.0, 0.0, 0.03, 0.4, -0.8, -0.03, 0.4, -0.8, 0.03, -0.4, 0.8, -0.03, -0.4, 0.8]))
laikago.setColor(0.5, 0.0, 0.0, 0.5)

# polyline
lines = server.addVisualPolyLine("lines")
lines.setColor(0, 0, 1, 1)

for i in range(0, 100):
    lines.addPoint(np.array([math.sin(i * 0.1), math.cos(i * 0.1), i * 0.01]))

# visualmesh
vertex = np.array([0.5,0.5,0., 0.0,-1.,0., -0.5,0.5,0., 0.,0.,1.], dtype=np.float32)
color = np.array([255,0,0, 0,255,0, 0,0,255, 0,0,126], dtype=np.uint8)
index = np.array([0,3,1, 1,3,2, 2,3,0, 0,1,2], dtype=int)
dynamicMesh = server.addVisualMesh("dynamicMesh", vertex, color, index)
dynamicMesh.setPosition(np.array([-2,0,0]))

# visualization
counter = 0

for i in range(100000):
    counter = counter + 1
    visBox.setColor(1, 1, (counter % 255 + 1) / 256., 1)
    visSphere.setColor(1, (counter % 255 + 1) / 256., 1, 1)
    lines.setColor(1 - (counter % 255 + 1) / 256., 1, (counter % 255 + 1) / 256., 1)
    visBox.setBoxSize((counter % 255 + 1) / 256. + 0.01, 1, 1)
    vertex[0] = 0.5 * (2.+math.sin(counter*0.01))
    color[10] = counter/5%255
    dynamicMesh.updateMesh(vertex, color)
    world.integrate()
    time.sleep(world.getTimeStep())

server.killServer()
