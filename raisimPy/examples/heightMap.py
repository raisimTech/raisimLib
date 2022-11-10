import os
import numpy as np
import raisimpy as raisim
import time

raisim.World.setLicenseFile(os.path.dirname(os.path.abspath(__file__)) + "/../../rsc/activation.raisim")
world = raisim.World()
world.setTimeStep(0.001)

# create objects
terrainProperties = raisim.TerrainProperties()
terrainProperties.frequency = 0.2
terrainProperties.zScale = 3.0
terrainProperties.xSize = 20.0
terrainProperties.ySize = 20.0
terrainProperties.xSamples = 50
terrainProperties.ySamples = 50
terrainProperties.fractalOctaves = 3
terrainProperties.fractalLacunarity = 2.0
terrainProperties.fractalGain = 0.25
hm = world.addHeightMap(0.0, 0.0, terrainProperties)

# robot
anymal_urdf_file = os.path.dirname(os.path.abspath(__file__)) + "/../../rsc/anymal/urdf/anymal.urdf"
anymal = world.addArticulatedSystem(anymal_urdf_file)

# ANYmal joint PD controller
anymal_nominal_joint_config = np.array([0, -1.5, 2.54, 1.0, 0.0, 0.0, 0.0, 0.03, 0.4, -0.8,
                                        -0.03, 0.4, -0.8, 0.03, -0.4, 0.8, -0.03, -0.4, 0.8])
anymal.setGeneralizedCoordinate(anymal_nominal_joint_config)
anymal.setPdGains(200*np.ones([18]), np.ones([18]))
anymal.setPdTarget(anymal_nominal_joint_config, np.zeros([18]))

# launch raisim server
server = raisim.RaisimServer(world)
server.launchServer(8080)

for i in range(500000):
    server.integrateWorldThreadSafe()
    time.sleep(0.0005)

server.killServer()
