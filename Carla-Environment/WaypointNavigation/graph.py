import matplotlib.pyplot as plt
import numpy as np
import time

x = np.array([])
y = np.array([0,1,2,9,2,5,3])
y_arr = [0,1,2,5,9,4]

# You probably won't need this if you're embedding things in a tkinter plot...
plt.ion()

fig = plt.figure()
ax = fig.add_subplot(111)

line1, = ax.plot(y_arr) # Returns a tuple of line objects, thus the comma

for i in range(50):
    # line1.set_xdata(np.append(x,i))
    y_arr.append(i)
    ax.plot(y_arr) 
    # line1.set_ydata(i)
    fig.canvas.draw()
    fig.canvas.flush_events()

    time.sleep(0.5)