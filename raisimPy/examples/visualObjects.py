import os
import numpy as np
import raisimpy as raisim
import math
import time

raisim.World.setLicenseFile(os.path.dirname(os.path.abspath(__file__)) + "/../../rsc/activation.raisim")
world = raisim.World()
ground = world.addGround()

# launch raisim server
server = raisim.RaisimServer(world)
server.launchServer(8080)

visSphere = server.addVisualSphere("v_sphere", 1, 1, 1, 1, 1)
visBox = server.addVisualBox("v_box", 1, 1, 1, 1, 1, 1, 1)
visCylinder = server.addVisualCylinder("v_cylinder", 1, 1, 0, 1, 0, 1)
visCapsule = server.addVisualCapsule("v_capsule", 1, 0.5, 0, 0, 1, 1)
laikago = server.addVisualArticulatedSystem("v_laikago", os.path.dirname(os.path.abspath(__file__)) + "/../../rsc/laikago/laikago.urdf")
laikago.setGeneralizedCoordinate(np.array([0, 0, 3.54, 1.0, 0.0, 0.0, 0.0, 0.03, 0.4, -0.8, -0.03, 0.4, -0.8, 0.03, -0.4, 0.8, -0.03, -0.4, 0.8]))
laikago.setColor(0.5, 0.0, 0.0, 0.5)

visSphere.setPosition(np.array([2, 0, 0]))
visCylinder.setPosition(np.array([0, 2, 0]))
visCapsule.setPosition(np.array([2, 2, 0]))

lines = server.addVisualPolyLine("lines")
lines.setColor(0, 0, 1, 1)

for i in range(0, 100):
    lines.addPoint(np.array([math.sin(i * 0.1), math.cos(i * 0.1), i * 0.01]))

counter = 0
server.startRecordingVideo("visualObjectDemo.mp4")

for i in range(1000):
    counter = counter + 1
    visBox.setColor(1, 1, (counter % 255 + 1) / 256., 1)
    visSphere.setColor(1, (counter % 255 + 1) / 256., 1, 1)
    lines.setColor(1 - (counter % 255 + 1) / 256., 1, (counter % 255 + 1) / 256., 1)
    visBox.setBoxSize((counter % 255 + 1) / 256. + 0.01, 1, 1)
    world.integrate()
    time.sleep(world.getTimeStep())

server.stopRecordingVideo()
server.killServer()
