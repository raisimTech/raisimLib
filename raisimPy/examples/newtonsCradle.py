import os
import numpy as np
import raisimpy as raisim
import math
import time


raisim.World.setLicenseFile(os.path.dirname(os.path.abspath(__file__)) + "/../../rsc/activation.raisim")
world = raisim.World()
ground = world.addGround()
world.setTimeStep(0.001)

world.setMaterialPairProp("steel", "steel", 0.1, 1.0, 0.0)

pin1 = world.addSphere(0.1, 0.8)
pin1.setAppearance("1,0,0,0.3")
pin1.setPosition(0.0, 0.0, 3.0)
pin1.setBodyType(raisim.BodyType.STATIC)

pin2 = world.addSphere(0.1, 0.8)
pin2.setAppearance("0,1,0,0.3")
pin2.setPosition(0.3, 0.0, 3.0)
pin2.setBodyType(raisim.BodyType.STATIC)

pin3 = world.addSphere(0.1, 0.8)
pin3.setAppearance("0,0,1,0.3")
pin3.setPosition(0.6, 0.0, 3.0)
pin3.setBodyType(raisim.BodyType.STATIC)

pin4 = world.addSphere(0.1, 0.8)
pin4.setAppearance("1,0,0,0.3")
pin4.setPosition(0.9, 0.0, 3.0)
pin4.setBodyType(raisim.BodyType.STATIC)

pin5 = world.addSphere(0.1, 0.8)
pin5.setPosition(0.9, 0.0, 6.0)
pin5.setBodyType(raisim.BodyType.STATIC)

pin6 = world.addSphere(0.1, 0.8)
pin6.setPosition(-3., 0.0, 7.0)
pin6.setBodyType(raisim.BodyType.STATIC)

pin7 = world.addSphere(0.1, 0.8)
pin7.setPosition(-4., 0.0, 7.0)
pin7.setBodyType(raisim.BodyType.STATIC)

anymalB_urdf_file = os.path.dirname(os.path.abspath(__file__)) + "/../../rsc/anymal/urdf/anymal.urdf"
anymalC_urdf_file = os.path.dirname(os.path.abspath(__file__)) + "/../../rsc/anymal_c/urdf/anymal.urdf"

anymalC = world.addArticulatedSystem(anymalC_urdf_file)
anymalB = world.addArticulatedSystem(anymalB_urdf_file)

jointNominalConfig = np.array([-3, 0, 4.54, 1.0, 0.0, 0.0, 0.0, 0.03, 0.4, -0.8, -0.03, 0.4, -0.8, 0.03, -0.4, 0.8, -0.03, -0.4, 0.8])
jointVelocityTarget = np.zeros([anymalC.getDOF()])
jointPgain = np.ones(anymalC.getDOF()) * 100.0
jointDgain = np.ones(anymalC.getDOF()) * 1.0

anymalC.setGeneralizedCoordinate(jointNominalConfig)
anymalC.setPdGains(jointPgain, jointDgain)
anymalC.setPdTarget(jointNominalConfig, jointVelocityTarget)
anymalC.setName("anymalC")

jointNominalConfig[0] = -4
anymalB.setGeneralizedCoordinate(jointNominalConfig)
anymalB.setPdGains(jointPgain, jointDgain)
anymalB.setPdTarget(jointNominalConfig, jointVelocityTarget)
anymalB.setName("anymalB")

ball1 = world.addSphere(0.1498, 0.8, "steel")
ball1.setPosition(0, 0.0, 1.0)

ball2 = world.addSphere(0.1499, 0.8, "steel")
ball2.setPosition(0.3, 0.0, 1.0)

ball3 = world.addSphere(0.1499, 0.8, "steel")
ball3.setPosition(0.6, 0.0, 1.0)

ball4 = world.addSphere(0.1499, 0.8, "steel")
ball4.setPosition(2.9, 0.0, 3.0)

box = world.addBox(.1, .1, .1, 1)
box.setPosition(0.9, 0.0, 4.2)

world.addStiffWire(pin1, 0, np.zeros(3), ball1, 0, np.zeros(3), 2.0)
world.addStiffWire(pin2, 0, np.zeros(3), ball2, 0, np.zeros(3), 2.0)
world.addStiffWire(pin3, 0, np.zeros(3), ball3, 0, np.zeros(3), 2.0)
world.addStiffWire(pin4, 0, np.zeros(3), ball4, 0, np.zeros(3), 2.0)

wire5 = world.addCompliantWire(pin5, 0, np.zeros(3), box, 0, np.zeros(3), 2.0, 200)
wire5.setStretchType(raisim.StretchType.BOTH)

wire6 = world.addCompliantWire(pin6, 0, np.zeros(3), anymalC, 0, np.zeros(3), 2.0, 1000)
wire6.setStretchType(raisim.StretchType.BOTH)

wire7 = world.addCustomWire(pin7, 0, np.zeros(3), anymalB, 0, np.zeros(3), 2.0)
wire7.setTension(310)

server = raisim.RaisimServer(world)
server.launchServer(8080)

for i in range(500000):
    time.sleep(0.001)
    server.integrateWorldThreadSafe()
    if i == 5000:
        world.removeObject(wire7)

server.killServer()
