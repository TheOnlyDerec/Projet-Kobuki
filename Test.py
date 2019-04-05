import numpy as np
from matplotlib import pyplot as plt
from matplotlib import animation

# First set up the figure, the axis, and the plot element we want to animate
fig = plt.figure()
#ax = plt.axes(xlim=(0, 2), ylim=(-2, 2)) #Pas pigé l'utilité mais semble changer la vitesse
line, = plt.plot([],[],'o')
plt.xlim(-5,5)
plt.ylim(-5,5)

# initialization function: plot the background of each frame
def init():
    line.set_data([], [])
    return line,

x = np.linspace(0, 2, 2000)
y = np.sin(x)

# animation function.  This is called sequentially
def animate(i):
    #line.set_data(np.array([x[i]]), np.array([y[i]]))
    line.set_data([x[i]], [y[i]])
    return line,

# call the animator.  blit=True means only re-draw the parts that have changed.
anim = animation.FuncAnimation(fig, animate, init_func=init,
                               frames=2000, interval=20, blit=True)

plt.show()