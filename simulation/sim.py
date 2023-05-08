import pybullet as p
import time
import pybullet_data
import logging
import datetime

logging.basicConfig(filename="simulation_log_"+datetime.datetime.now().strftime("%d-%m-%Y_%H-%M-%S")+".log", level=logging.DEBUG, format='%(asctime)s [%(levelname)s] %(message)s')

def log_info(message, level=logging.INFO):
    if level == logging.DEBUG:
        logging.debug(message)
    elif level == logging.INFO:
        logging.info(message)
    elif level == logging.WARNING:
        logging.warning(message)
    elif level == logging.ERROR:
        logging.error(message)
    elif level == logging.CRITICAL:
        logging.critical(message)
    print(message)

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -10)

planeId = p.loadURDF("plane.urdf")

startPos = [0, 0, 0.5]
startOrientation = p.getQuaternionFromEuler([0, 0, 0])
robotId = p.loadURDF("./robot.urdf", startPos, startOrientation)

ground_friction = 1.0
p.changeDynamics(planeId, -1, lateralFriction=ground_friction)

wheel1_joint = 0
wheel2_joint = 1
wheel_friction = 0.1
p.changeDynamics(robotId, wheel1_joint, lateralFriction=wheel_friction,  spinningFriction=wheel_friction, rollingFriction=wheel_friction)
p.changeDynamics(robotId, wheel2_joint, lateralFriction=wheel_friction,  spinningFriction=wheel_friction, rollingFriction=wheel_friction)


for i in range(p.getNumJoints(robotId)):
    print(p.getJointInfo(robotId, i))

maxForce = 100
mode = p.VELOCITY_CONTROL
p.setJointMotorControl2(robotId, 0,
 	controlMode=mode, targetVelocity = 10, force=maxForce)

p.setJointMotorControl2(robotId, 1,
 	controlMode=mode, targetVelocity = 10, force=maxForce)


gain = 10
while True:
    p.stepSimulation()
    
    
    # Get the angular velocity of the robot's body
    _, body_angular_velocity = p.getBaseVelocity(robotId)
    
    # Get the current joint velocities
    _, wheel1_velocity, _, _ = p.getJointState(robotId, wheel1_joint)
    _, wheel2_velocity, _, _ = p.getJointState(robotId, wheel2_joint)

    # Adjust the wheel velocities to counteract the body's rotation
    wheel1_velocity -= gain * body_angular_velocity[1]
    wheel2_velocity -= gain * body_angular_velocity[1]

    # Set the new joint velocities
    p.setJointMotorControl2(robotId, wheel1_joint, p.VELOCITY_CONTROL, targetVelocity=wheel1_velocity)
    p.setJointMotorControl2(robotId, wheel2_joint, p.VELOCITY_CONTROL, targetVelocity=wheel2_velocity)
    
    log_info(f"Body angular velocity: {body_angular_velocity}, Wheel 1 velocity: {wheel1_velocity}, Wheel 2 velocity: {wheel2_velocity}")
    
    

    time.sleep(1.0 / 240.)

cubePos, cubeOrn = p.getBasePositionAndOrientation(robotId)
print(cubePos, cubeOrn)
p.disconnect()
