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
        
        self.titles = []
        self.x_labels = []
        self.y_labels = []
        
        self.colors = []
        
        self.lock_xs = []
        self.lock_ys = []
        self.x_ranges = []
        self.y_ranges = []
    
    
    def add_plot(self,
                 x=[],
                 y=[],
                 title=None,
                 x_label=None,
                 y_label=None,
                 color=None,
                 lock_x_range=False,
                 lock_y_range=False):
        # Store the plot data
        self.xs.append(x)
        self.ys.append(y)
        
        # Store the plot labels
        self.titles.append(title)
        self.x_labels.append(x_label)
        self.y_labels.append(y_label)
        
        # Store the line parameters
        self.colors.append(color)
        
        # Store the range parameters
        self.lock_xs.append(lock_x_range)
        min_x = np.min(x)
        max_x = np.max(x)
        self.x_ranges.append((min_x, max_x))
        self.lock_ys.append(lock_y_range)
        min_y = np.min(y)
        max_y = np.max(y)
        self.y_ranges.append((min_y, max_y))
        
        # Return the index of the added plot
        plot_index = len(self.xs)-1
        return plot_index


    def update_plot(self,
                    plot_index,
                    x=[],
                    y=[],
                    tail=None):
        # Trim data to desired length
        if tail != None and len(x) > tail:
            x = x[-tail:]
            y = y[-tail:]
            
        # Update the plot data
        self.xs[plot_index] = x
        self.ys[plot_index] = y


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
            
            # Handle range locking
            if self.lock_xs[i]:
                min_x = min(np.min(x), self.x_ranges[i][0])
                max_x = max(np.max(x), self.x_ranges[i][1])
                self.x_ranges[i] = (min_x, max_x)
                axis.set_xlim(left = min_x,
                              right = max_x)
            if self.lock_ys[i]:
                min_y = min(np.min(y), self.y_ranges[i][0])
                max_y = max(np.max(y), self.y_ranges[i][1])
                self.y_ranges[i] = (min_y, max_y)
                axis.set_ylim(bottom = min_y,
                              top = max_y)
        
        # Set the figure settings
        self.fig.tight_layout()
        
        
    def step(self,
             pause):
        num_plots = len(self.xs)
        for i in range(num_plots):
            # Retrieve the data for each axis
            x = self.xs[i]
            y = self.ys[i]
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
            
            # Handle range locking
            if self.lock_xs[i]:
                min_x = min(np.min(x), self.x_ranges[i][0])
                max_x = max(np.max(x), self.x_ranges[i][1])
                self.x_ranges[i] = (min_x, max_x)
                axis.set_xlim(left = min_x,
                              right = max_x)
            if self.lock_ys[i]:
                min_y = min(np.min(y), self.y_ranges[i][0])
                max_y = max(np.max(y), self.y_ranges[i][1])
                self.y_ranges[i] = (min_y, max_y)
                axis.set_ylim(bottom = min_y,
                              top = max_y)
            
        # Draw in real time according to the pause duration
        plt.pause(pause)


if __name__ == "__main__":
    x1 = np.linspace(0,10,20)
    x2 = np.linspace(0,20,30)
    x3 = np.linspace(10,15,40)
    x4 = np.linspace(-5,5,50)
    y1 = np.random.rand(20)
    y2 = np.random.rand(30)
    y3 = np.random.rand(40)
    y4 = np.random.rand(50)
    
    a = Animator()
    
    i1 = a.add_plot(x=x1,
                    y=y1,
                    title="1",
                    x_label="$x_{1}$",
                    y_label="$x_{1}$",
                    color='r',
                    lock_x_range=False,
                    lock_y_range=False)
    i2 = a.add_plot(x=x2,
                    y=y2,
                    title="2",
                    x_label="$x_{2}$",
                    y_label="$y_{2}$",
                    color='g',
                    lock_x_range=True,
                    lock_y_range=False)
    i3 = a.add_plot(x=x3,
                    y=y3,
                    title="3",
                    x_label="$x_{3}$",
                    y_label="$y_{3}$",
                    color='b',
                    lock_x_range=False,
                    lock_y_range=True)
    i4 = a.add_plot(x=x4,
                    y=y4,
                    title="4",
                    x_label="$x_{4}$",
                    y_label="$y_{4}$",
                    color='k',
                    lock_x_range=True,
                    lock_y_range=True)

    a.create_figure()
    
    
    for i in range(100):
        x1 = np.append(x1, 2.*x1[-1]-x1[-2])
        x2 = np.append(x2, 2.*x2[-1]-x2[-2])
        x3 = np.append(x3, 2.*x3[-1]-x3[-2])
        x4 = np.append(x4, 2.*x4[-1]-x4[-2])
        y1 = np.append(y1, np.random.rand())
        y2 = np.append(y2, np.random.rand())
        y3 = np.append(y3, np.random.rand())
        y4 = np.append(y4, np.random.rand())
        
        a.update_plot(i1, x=x1, y=y1, tail=20)
        a.update_plot(i2, x=x2, y=y2, tail=20)
        a.update_plot(i3, x=x3, y=y3, tail=20)
        a.update_plot(i4, x=x4, y=y4, tail=20)
        
        a.step(0.01)

    plt.show()