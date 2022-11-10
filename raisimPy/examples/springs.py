import os
import numpy as np
import raisimpy as raisim
import time


world = raisim.World()
world.setTimeStep(0.001)

springed_cartpole_urdf_file = os.path.dirname(os.path.abspath(__file__)) + "/../../rsc/springDamper/cartpole.urdf"
server = raisim.RaisimServer(world)
ground = world.addGround()

springed_cartpole = world.addArticulatedSystem(springed_cartpole_urdf_file)
springed_cartpole.setName("springed_cartpole")
springed_cartpole.getSprings()[1].q_ref = np.array([2., 0, 0, 0]) # overwrite the spring mount position

for spring in springed_cartpole.getSprings():
    print(spring.q_ref)

server.launchServer(8080)

for i in range(500000):
    time.sleep(0.001)
    server.integrateWorldThreadSafe()

server.killServer()