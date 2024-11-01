import ctrnn
import matplotlib.pyplot as plt
import numpy as np

size = 20
duration = 100
stepsize = 0.01

time = np.arange(0.0,duration,stepsize)

nn = ctrnn.CTRNN(size)

nn.load("ctrnn2.npz")

nn.initializeState(np.zeros(size))

outputs = np.zeros((len(time),size))

# Run simulation
step = 0
for t in time:
    nn.step(stepsize)
    outputs[step] = nn.Outputs
    step += 1

# Plot activity
for i in range(size):
    #plt.plot(time,outputs)
    plt.plot(time, outputs.T[6], 'k')
plt.xlabel("Time")
plt.ylabel("Output")
plt.title("Neural activity")
plt.show()
