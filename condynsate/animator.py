"""
This module provides the Animator class and all associated classes and 
functions that are used by it.
"""


###############################################################################
#DEPENDENCIES
###############################################################################
import matplotlib.pyplot as plt
import numpy as np
import time


class Animator():
    """
    Animator manages the real time plotting of states.
    """
    def __init__(self, fr):
        # Plot data
        self.xs = []
        self.ys = []
        self.tails = []
        self.lines = []
        
        # Plot labels
        self.titles = []
        self.x_labels = []
        self.y_labels = []
        self.colors = []
        
        # Plot limit options
        self.x_data_ranges = []
        self.y_data_ranges = []
        self.fixed_x_plot_lims = []
        self.fixed_y_plot_lims = []
        self.x_plot_lims = []
        self.y_plot_lims = []
    
        # Boolean flag that indicates whether the figure is made
        self.figure_is_made = False
        
        # Frame rate data
        self.fr = fr
        self.last_step_time = time.time()
        self.n_plots = 0
    
    
    def add_plot(self,
                 title=None,
                 x_label=None,
                 y_label=None,
                 color=None,
                 tail=None,
                 x_lim=[None, None],
                 y_lim=[None, None]):
        # Store the plot data
        self.xs.append([])
        self.ys.append([])
        self.tails.append(tail)
        self.lines.append(None)
        
        # Store the plot labels
        self.titles.append(title)
        self.x_labels.append(x_label)
        self.y_labels.append(y_label)
        self.colors.append(color)
        
        # Store the limit options
        self.x_data_ranges.append([np.inf, -np.inf])
        self.y_data_ranges.append([np.inf, -np.inf])
        self.fixed_x_plot_lims.append(x_lim)
        self.fixed_y_plot_lims.append(y_lim)
        self.x_plot_lims.append([None, None])
        self.y_plot_lims.append([None, None])
        
        # Return the index of the added plot
        plot_index = len(self.xs)-1
        return plot_index


    def _trim_data(self,
                   x,
                   y,
                   tail):
        # Trim data to desired length
        if tail != None and len(x) > tail:
            x = x[-tail:]
            y = y[-tail:]
        return x, y


    def _get_ran_lims(self,
                      data,
                      data_range,
                      fixed_plot_lims):
        # Variables to hold the calulcated plot limits
        plot_limits = [0., 0.]
        plot_limits[0] = fixed_plot_lims[0]
        plot_limits[1] = fixed_plot_lims[1]
        data_min = np.min(data)#min(np.min(data), data_range[0])
        data_max = np.max(data)#max(np.max(data), data_range[1])
        data_min = 1.1*data_min - 0.1*data_max
        data_max = 1.1*data_max - 0.1*data_min
        new_data_range = [data_min, data_max]
        
        # If there is no lower hard plot limit, set to min of all data seen
        if plot_limits[0] == None:
            plot_limits[0] = data_min
        
        # If there is no upper hard plot limit, set to max of all data seen
        if plot_limits[1] == None:
            plot_limits[1] = data_max
            
        # If the calculated plot limits are the same, set the plot limits as
        # (None, None)
        if plot_limits[0] == plot_limits[1]:
            plot_limits = [None, None]
            
        # Return the calculated plot limits
        return new_data_range, plot_limits


    def set_plot_data(self,
                      plot_index,
                      x,
                      y):
        # Trim data to desired length
        tail = self.tails[plot_index]
        x, y = self._trim_data(x = x,
                               y = y,
                               tail = tail)
            
        # Update the plot data
        self.xs[plot_index] = x
        self.ys[plot_index] = y
        
        # Update the x data ranges and associated plot limits
        cur_x_range = self.x_data_ranges[plot_index]
        fix_x_lims = self.fixed_x_plot_lims[plot_index]
        new_x_range, x_lims = self._get_ran_lims(data = x,
                                                 data_range = cur_x_range,
                                                 fixed_plot_lims = fix_x_lims)
        self.x_data_ranges[plot_index] = new_x_range
        self.x_plot_lims[plot_index] = x_lims
        
        # Update the y data ranges and associated plot limits
        cur_y_range = self.y_data_ranges[plot_index]
        fix_y_lims = self.fixed_y_plot_lims[plot_index]
        new_y_range, y_lims = self._get_ran_lims(data = y,
                                                 data_range = cur_y_range,
                                                 fixed_plot_lims = fix_y_lims)
        self.y_data_ranges[plot_index] = new_y_range
        self.y_plot_lims[plot_index] = y_lims


    def _get_n_plots(self):
        # Get the length of all plot parameters
        lens = np.zeros(14)
        lens[0] = len(self.xs)
        lens[1] = len(self.ys)
        lens[2] = len(self.lines)
        lens[3] = len(self.tails)
        lens[4] = len(self.titles)
        lens[5] = len(self.x_labels)
        lens[6] = len(self.y_labels)
        lens[7] = len(self.colors)
        lens[8] = len(self.x_data_ranges)
        lens[9] = len(self.y_data_ranges)
        lens[10] = len(self.fixed_x_plot_lims)
        lens[11] = len(self.fixed_y_plot_lims)
        lens[12] = len(self.x_plot_lims)
        lens[13] = len(self.y_plot_lims)
        
        # Make sure each plot has a full set of parameters
        check_val = lens[0]
        for n in lens:
            if n != check_val:
                print("Something went wrong when getting number of plots.")
                return 0
            
        # If all plots have full set of parameters, return the number of plots
        n_plots = len(self.xs)
        return n_plots
            
    
    def _get_subplot_shape(self,
                          n_plots):
        # Get the number of columns needed
        n_cols = int(np.ceil(0.5*n_plots))
        
        # The number of rows should never exceed 2
        if n_plots == 0:
            n_rows = 0
        elif n_plots == 1:
            n_rows = 1
        else:
            n_rows = 2
            
         # Return the calculated dimension
        dim = (n_rows, n_cols)
        return dim
    
    
    def _create_subplot(self,
                        axis,
                        title,
                        x_label,
                        y_label,
                        x_plot_lim,
                        y_plot_lim):
            # Clear the axis
            axis.clear()
            
            # Set the labels
            axis.set_title(title)
            axis.set_xlabel(x_label)
            axis.set_ylabel(y_label)
            
            # Set the limits if there are any
            if x_plot_lim != [None, None]:
                axis.set_xlim(x_plot_lim[0],
                              x_plot_lim[1])
            if y_plot_lim != [None, None]:
                axis.set_ylim(y_plot_lim[0],
                              y_plot_lim[1])
    
    
    def create_figure(self):
        # Set the desired backend and matplotlib parameters for interactive
        # plotting
        plt.switch_backend("QtAgg")
        
        # Turn on interactive mode to run the GUI event loop
        plt.ion()
        
        # Determine the number of plots and only continue if it's greater than
        # 0.
        self.n_plots = self._get_n_plots()
        if self.n_plots == 0:
            plt.ioff()
            return
        
        # Calculate the subplot shape and create the figure
        (n_rows, n_cols) = self._get_subplot_shape(self.n_plots)
        self.fig, self.axes = plt.subplots(n_rows, n_cols)
        if self.n_plots > 1:
            self.axes = self.axes.flatten()

        # Create each subplot
        for i in range(self.n_plots):
            # Get the axis on which the subplot is drawn
            if self.n_plots > 1:
                axis = self.axes[i]
            else:
                axis = self.axes    
            
            # Retrieve the label data
            title = self.titles[i]
            x_label = self.x_labels[i]
            y_label = self.y_labels[i]
            
            # Retrieve the limit data
            x_plot_lim = self.x_plot_lims[i]
            y_plot_lim = self.y_plot_lims[i]
            
            # Create the subplot
            self._create_subplot(axis=axis,
                                 title=title,
                                 x_label=x_label,
                                 y_label=y_label,
                                 x_plot_lim=x_plot_lim,
                                 y_plot_lim=y_plot_lim)
            
        # Set the layout, draw on the figure, and flush the even buffer
        self.fig.tight_layout()
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events() 
        
        # Flag that the figure is done being made
        self.figure_is_made = True
        
        # Turn off interactive mode to suspend the GUI event loop
        plt.ioff()
       
       
    def _step_subplot(self,
                      x,
                      y,
                      line,
                      axis,
                      color,
                      x_plot_lim,
                      y_plot_lim):
        # Redraw the line data
        if line==None:
            line, = axis.plot(x, y, c=color)
        else:
            line.set_data(x, y)
            
        # Reset the axis limits
        axis.set_xlim(x_plot_lim[0],
                      x_plot_lim[1])
        axis.set_ylim(y_plot_lim[0],
                      y_plot_lim[1])
        
        # Return the line artist
        return line
        
        
    def step(self):
        # Turn on interactive mode to run the GUI event loop
        plt.ion()
        
        # Determine if step is to be taken based on set frame rate
        self.time_since_last_step = time.time() - self.last_step_time
        if self.time_since_last_step < (1. / self.fr):
            plt.ioff()
            return
        
        # Create the figure if it is not already made
        if not self.figure_is_made:
            self.create_figure()
        
        # Update each plot
        for i in range(self.n_plots):
            # Get the axis on which the subplot is drawn
            if self.n_plots > 1:
                axis = self.axes[i]
            else:
                axis = self.axes    
    
            # Retrieve the plot data
            x = self.xs[i]
            y = self.ys[i]
            
            # Retrieve the limit data
            x_plot_lim = self.x_plot_lims[i]
            y_plot_lim = self.y_plot_lims[i]
            
            # Retrieve the line artist for each axis
            line = self.lines[i]
            color = self.colors[i]
            
            # Update the plotted data
            if len(x)>1 and len(y)>1:
                new_line = self._step_subplot(x=x,
                                              y=y,
                                              line=line,
                                              axis=axis,
                                              color=color,
                                              x_plot_lim=x_plot_lim,
                                              y_plot_lim=y_plot_lim)
                if line==None:
                    self.lines[i] = new_line
            
        # Set the draw on the figure, and flush the even buffer
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events() 
        
        # Update time keeping
        self.last_step_time = time.time()
        
        # Turn off interactive mode to suspend the GUI event loop
        plt.ioff()


    def flush_events(self):
        plt.ion()
        self.fig.canvas.flush_events() 
        plt.ioff()


if __name__ == "__main__":
    x1 = []
    x2 = np.linspace(0,20,30)
    x3 = np.linspace(10,15,40)
    x4 = np.linspace(-5,5,50)
    y1 = []
    y2 = np.random.rand(30)
    y3 = np.random.rand(40)
    y4 = np.random.rand(50)
    
    a = Animator(fr=10.)
    
    i1 = a.add_plot(title="1",
                    x_label="$x_{1}$",
                    y_label="$x_{1}$",
                    color='r',
                    tail=20,
                    x_lim=[0,10],
                    y_lim=[0,1])
    i2 = a.add_plot(title="2",
                    x_label="$x_{2}$",
                    y_label="$y_{2}$",
                    color='g',
                    x_lim=[0,100])
    i3 = a.add_plot(title="3",
                    x_label="$x_{3}$",
                    y_label="$y_{3}$",
                    color='b',
                    y_lim=[0.25,0.75],
                    tail=20)
    i4 = a.add_plot(title="4",
                    x_label="$x_{4}$",
                    y_label="$y_{4}$",
                    color='k',
                    tail=50)
    
    a.create_figure()
    
    for i in range(100):
        print(str(i))
        x1 = np.append(x1, i*0.1)
        x2 = np.append(x2, 2.*x2[-1]-x2[-2])
        x3 = np.append(x3, 2.*x3[-1]-x3[-2])
        x4 = np.append(x4, 2.*x4[-1]-x4[-2])
        y1 = np.append(y1, np.random.rand())
        y2 = np.append(y2, i*np.random.rand())
        y3 = np.append(y3, np.random.rand())
        y4 = np.append(y4, np.random.rand())
        
        a.set_plot_data(i1, x=x1, y=y1)
        a.set_plot_data(i2, x=x2, y=y2)
        a.set_plot_data(i3, x=x3, y=y3)
        a.set_plot_data(i4, x=x4, y=y4)
        
        a.step()
        
        time.sleep(0.001)