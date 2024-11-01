import pybullet as p
import pybullet_data
import pyrosim.pyrosim as ps
import numpy as np
import time 
import ctrnn
import matplotlib.pyplot as plt

physicsClient = p.connect(p.GUI)
p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.setGravity(0,0,-9.8)
planeId = p.loadURDF("plane.urdf")
#p.loadSDF("box.sdf")
robotId = p.loadURDF("body.urdf")

duration = 10000

#Neural Network Parameters
size = 20
duration = 100
stepsize = 0.01
RIGHT_MOTOR = 6 #Neuron controlling the right motor 0 [Low Frequency],2,4,6 (has a back and forth motion), 7 falls over, 
LEFT_MOTOR = 6 #Neuron controlling the left motor

simulation_time = np.arange(0.0,duration,stepsize)
num_steps = int(duration / stepsize)

ps.Prepare_To_Simulate(robotId)

nn = ctrnn.CTRNN(size)

#Initialize parameters of the CRNN
nn.load("ctrnn.npz")

outputs = np.zeros((len(simulation_time),size))


step = 0
positions = []
distances = []
cumulative_distance = 0
previous_position = np.array(p.getBasePositionAndOrientation(robotId)[0])   
for t in simulation_time:
    nn.step(stepsize)
    outputs[step] = nn.Outputs
    step += 1
    ps.Set_Motor_For_Joint(bodyIndex = robotId, 
                                jointName = b'Right_Torso',
                                controlMode = p.POSITION_CONTROL,
                                targetPosition = nn.Outputs[RIGHT_MOTOR], #Add some gain?
                                maxForce = 500)
    #print(nn.Outputs[RIGHT_MOTOR])
    ps.Set_Motor_For_Joint(bodyIndex = robotId, 
                                jointName = b'Left_Torso',
                                controlMode = p.POSITION_CONTROL,
                                targetPosition = nn.Outputs[LEFT_MOTOR],
                                maxForce = 500)
    current_position = np.array(p.getBasePositionAndOrientation(robotId)[0])
    positions.append(current_position)
   
    # Calculate the distance moved since the last position
    step_distance = np.linalg.norm(current_position - previous_position)
    cumulative_distance += step_distance
    distances.append(cumulative_distance)
   
    # Update previous position
    previous_position = current_position
# =============================================================================
#     if alternate < 10:
#         ps.Set_Motor_For_Joint(bodyIndex = robotId, 
#                                      jointName = b'Front_Left',
#                                      controlMode = p.POSITION_CONTROL,
#                                      targetPosition = nn.Outputs[LEFT_MOTOR],
#                                      maxForce = 500)
#         ps.Set_Motor_For_Joint(bodyIndex = robotId, 
#                                      jointName = b'Rear_Left',
#                                      controlMode = p.POSITION_CONTROL,
#                                      targetPosition = nn.Outputs[LEFT_MOTOR],
#                                      maxForce = 500)
#         alternate += 1
#     elif alternate < 19:
#         ps.Set_Motor_For_Joint(bodyIndex = robotId, 
#                                      jointName = b'Front_Right',
#                                      controlMode = p.POSITION_CONTROL,
#                                      targetPosition = nn.Outputs[LEFT_MOTOR],
#                                      maxForce = 500)
#     
#         ps.Set_Motor_For_Joint(bodyIndex = robotId, 
#                                      jointName = b'Rear_Right',
#                                      controlMode = p.POSITION_CONTROL,
#                                      targetPosition = nn.Outputs[LEFT_MOTOR],
#                                      maxForce = 500)
#         alternate += 1
#     else:
#         alternate = 0
# =============================================================================
    #print(nn.Outputs[LEFT_MOTOR])
    p.stepSimulation()
    time.sleep(1/500)

# =============================================================================
# for i in range(size):
#     plt.plot(simulation_time,outputs)
# plt.xlabel("Time")
# plt.ylabel("Output")
# plt.title("Neural activity")
# plt.show()
# =============================================================================

time_values = np.arange(num_steps) * stepsize

print("Distance Traveled: ")
print(distances[len(distances) - 1])

plt.figure(figsize=(10, 5))
plt.plot(time_values, distances, label="Distance Traveled")
plt.xlabel("Time (seconds)")
plt.ylabel("Distance (meters)")
plt.title("Distance Traveled Over Time")
plt.legend()
plt.grid()
plt.show()

p.disconnect()