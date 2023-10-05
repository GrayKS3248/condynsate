"""
This module provides the Animator class and all associated classes and 
functions that are used by it.
"""


###############################################################################
#DEPENDENCIES
###############################################################################
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import time
import multiprocessing


def start_ani(x, y):
    fig, ax = plt.subplots()
    animation.FuncAnimation(fig,
                            update,
                            frames=100,
                            interval=10,
                            blit=True)
    plt.show()


def update(i):
    x = np.linspace(0,100, 20)
    y = np.random.rand(20)
    p, = plt.plot(x, y)
    plt.tight_layout()
    return p,
    
if __name__ == "__main__":
    x = [0.]
    y = [0.]

    plt.plot(x,y)
    
    for i in range(100):
        plt.clf()
        x.append(x[-1]+0.01)
        y.append(np.random.rand())
        plt.plot(x, y)
        plt.pause(0.01)
        
    plt.show()
        
    
    
    
    