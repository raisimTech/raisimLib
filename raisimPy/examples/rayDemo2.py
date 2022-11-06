import os
import numpy as np
import raisimpy as raisim
import time
import math


raisim.World.setLicenseFile(os.path.dirname(os.path.abspath(__file__)) + "/../../rsc/activation.raisim")
world = raisim.World() 
world.setTimeStep(0.001)

# create objects
terrainProperties = raisim.TerrainProperties()
terrainProperties.frequency = 0.2
terrainProperties.zScale = 2.0
terrainProperties.xSize = 70.0
terrainProperties.ySize = 70.0
terrainProperties.xSamples = 70
terrainProperties.ySamples = 70
terrainProperties.fractalOctaves = 3
terrainProperties.fractalLacunarity = 2.0
terrainProperties.fractalGain = 0.25

hm = world.addHeightMap(0.0, 0.0, terrainProperties)
hm.setAppearance("soil1")
robot = world.addArticulatedSystem(os.path.dirname(os.path.abspath(__file__)) + "/../../rsc/husky/husky.urdf")
robot.setName("smb")
robot.setGeneralizedCoordinate(np.array([0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0]))
robot.setGeneralizedVelocity(np.zeros(7))
robot.setJointDamping(np.array([0, 0, 0, 0, 0, 0, 1, 1, 1, 1]))

# launch raisim server
server = raisim.RaisimServer(world)
server.launchServer(8080)
scans = []
server.focusOn(robot)
scanSize1 = 8
scanSize2 = 25

# create visual boxes to visualize scan points
for i in range(scanSize1):
    for j in range(scanSize2):
        scans.append(server.addVisualBox("box" + str(i) + "/" + str(j), 0.3, 0.3, 0.3, 1, 0, 0))

for i in range(10000000000):
    server.integrateWorldThreadSafe()
    lidarPos = robot.getFramePosition("imu_joint")
    lidarOri = robot.getFrameOrientation("imu_joint")

    for i in range(scanSize1):
        for j in range(scanSize2):
            yaw = j * math.pi / scanSize2 * 0.6 - 0.3 * math.pi
            pitch = -(i * 0.3/scanSize1) + 0.2
            normInv = 1. / math.sqrt(pitch * pitch + 1)
            direction = np.mat([np.cos(yaw) * normInv, np.sin(yaw) * normInv, -pitch * normInv], dtype=np.float64)
            rayDirection = lidarOri.dot(direction.transpose())
            col = world.rayTest(lidarPos, rayDirection, 30)
            if col.size() > 0:
                scans[i * scanSize2 + j].setPosition(col.at(0).getPosition())
            else:
                scans[i * scanSize2 + j].setPosition(np.array([0, 0, 100]))

    robot.setGeneralizedForce(np.array([0, 0, 0, 0, 0, 0, -20, -20, -20, -20]))
    gc = robot.getGeneralizedCoordinate()

    if abs(gc[0]) > 35. or abs(gc[1]) > 35.:
        robot.setGeneralizedCoordinate(np.array([0, 0, 2, 1, 0, 0, 0, 0, 0, 0, 0]))

server.killServer()

