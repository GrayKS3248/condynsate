"""
This module provides the Animator class and all associated classes and 
functions that are used by it.
"""


###############################################################################
#DEPENDENCIES
###############################################################################
import matplotlib.pyplot as plt
import numpy as np


class Animator():
    """
    Animator manages the real time plotting of states.
    """
    def __init__(self):
        self.xs = []
        self.ys = []
        self.tails = []
        
        self.titles = []
        self.x_labels = []
        self.y_labels = []
        
        self.colors = []
        
        self.lock_xs = []
        self.lock_ys = []
        self.x_ranges = []
        self.y_ranges = []
    
    
    def add_plot(self,
                 title=None,
                 x_label=None,
                 y_label=None,
                 color=None,
                 lock_x_range=False,
                 lock_y_range=False,
                 tail=None):
        # Store the plot data
        self.xs.append([])
        self.ys.append([])
        self.tails.append(tail)
        
        # Store the plot labels
        self.titles.append(title)
        self.x_labels.append(x_label)
        self.y_labels.append(y_label)
        
        # Store the line parameters
        self.colors.append(color)
        
        # Store the range parameters
        self.lock_xs.append(lock_x_range)
        self.lock_ys.append(lock_y_range)
        self.x_ranges.append([None, None])
        self.y_ranges.append([None, None])
        
        # Return the index of the added plot
        plot_index = len(self.xs)-1
        return plot_index


    def set_plot_data(self,
                      plot_index,
                      x,
                      y):
        # Trim data to desired length
        tail = self.tails[plot_index]
        if tail != None and len(x) > tail:
            x = x[-tail:]
            y = y[-tail:]
            
        # Update the plot data
        self.xs[plot_index] = x
        self.ys[plot_index] = y
        
        # Update the plot x range data
        x_range = self.x_ranges[plot_index]
        if x_range[0] == None:
            min_x = np.min(x)
            max_x = np.max(x)
        else:
            min_x = min(np.min(x), x_range[0])
            max_x = max(np.max(x), x_range[1])
        self.x_ranges[plot_index] = (min_x, max_x)
        
        # Update the plot y range data
        y_range = self.y_ranges[plot_index]
        if y_range[0] == None:
            min_y = np.min(y)
            max_y = np.max(y)
        else:
            min_y = min(np.min(y), y_range[0])
            max_y = max(np.max(y), y_range[1])
        self.y_ranges[plot_index] = (min_y, max_y)


    def create_figure(self):
        # Determine the dimensions of the subplots
        num_plots = len(self.xs)
        n_cols = int(np.ceil(0.5*num_plots))
        if num_plots < 2:
            n_rows = 1
        else:
            n_rows = 2
        
        # Create the figure and the axes
        self.fig, self.axes = plt.subplots(n_rows, n_cols)
        if num_plots > 1:
            self.axes = self.axes.flatten()
        
        # Draw the axes
        for i in range(num_plots):
            # Retrieve data for given axis
            x = self.xs[i]
            y = self.ys[i]
            x_range = self.x_ranges[i]
            y_range = self.y_ranges[i]
            title = self.titles[i]
            x_label = self.x_labels[i]
            y_label = self.y_labels[i]
            color = self.colors[i]
            if num_plots > 1:
                axis = self.axes[i]
            else:
                axis = self.axes
                
            # Plot the data and set axis parameters
            axis.plot(x, y, color=color)
            axis.set_title(title)
            axis.set_xlabel(x_label)
            axis.set_ylabel(y_label)
            if self.lock_xs[i]:
                axis.set_xlim(x_range[0],
                              x_range[1])
            if self.lock_ys[i]:
                axis.set_ylim(y_range[0],
                              y_range[1])
        
        # Set the figure settings
        self.fig.tight_layout()
        
        
    def step(self,
             pause):
        num_plots = len(self.xs)
        for i in range(num_plots):
            # Retrieve the data for each axis
            x = self.xs[i]
            y = self.ys[i]
            x_range = self.x_ranges[i]
            y_range = self.y_ranges[i]
            title = self.titles[i]
            x_label = self.x_labels[i]
            y_label = self.y_labels[i]
            color = self.colors[i]
            if num_plots > 1:
                axis = self.axes[i]
            else:
                axis = self.axes
                
            # Clear and replot data
            axis.clear()
            axis.plot(x, y, color=color)
            axis.set_title(title)
            axis.set_xlabel(x_label)
            axis.set_ylabel(y_label)
            if self.lock_xs[i]:
                axis.set_xlim(x_range[0],
                              x_range[1])
            if self.lock_ys[i]:
                axis.set_ylim(y_range[0],
                              y_range[1])
            
        # Draw in real time according to the pause duration
        plt.pause(pause)


if __name__ == "__main__":
    x1 = []
    x2 = np.linspace(0,20,30)
    x3 = np.linspace(10,15,40)
    x4 = np.linspace(-5,5,50)
    y1 = []
    y2 = np.random.rand(30)
    y3 = np.random.rand(40)
    y4 = np.random.rand(50)
    
    a = Animator()
    
    i1 = a.add_plot(title="1",
                    x_label="$x_{1}$",
                    y_label="$x_{1}$",
                    color='r',
                    lock_x_range=False,
                    lock_y_range=False,
                    tail=20)
    i2 = a.add_plot(title="2",
                    x_label="$x_{2}$",
                    y_label="$y_{2}$",
                    color='g',
                    lock_x_range=True,
                    lock_y_range=False,
                    tail=20)
    i3 = a.add_plot(title="3",
                    x_label="$x_{3}$",
                    y_label="$y_{3}$",
                    color='b',
                    lock_x_range=False,
                    lock_y_range=True,
                    tail=20)
    i4 = a.add_plot(title="4",
                    x_label="$x_{4}$",
                    y_label="$y_{4}$",
                    color='k',
                    lock_x_range=True,
                    lock_y_range=True,
                    tail=20)

    a.create_figure()
    
    
    for i in range(100):
        x1 = np.append(x1, i*0.1)
        x2 = np.append(x2, 2.*x2[-1]-x2[-2])
        x3 = np.append(x3, 2.*x3[-1]-x3[-2])
        x4 = np.append(x4, 2.*x4[-1]-x4[-2])
        y1 = np.append(y1, np.random.rand())
        y2 = np.append(y2, np.random.rand())
        y3 = np.append(y3, np.random.rand())
        y4 = np.append(y4, np.random.rand())
        
        a.set_plot_data(i1, x=x1, y=y1)
        a.set_plot_data(i2, x=x2, y=y2)
        a.set_plot_data(i3, x=x3, y=y3)
        a.set_plot_data(i4, x=x4, y=y4)
        
        a.step(0.01)

    plt.show()