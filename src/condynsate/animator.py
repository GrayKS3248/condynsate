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
        """
        Initializes a new instance of an animator

        Parameters
        ----------
        fr : float
            The frame rate (frames per second) at which the animated plots are
            updated.

        Returns
        -------
        None.

        """
        # Plot data
        self.xs = []
        self.ys = []
        self.all_xs = []
        self.all_ys = []
        self.tails = []
        self.lines = []
        
        # Plot labels
        self.titles = []
        self.x_labels = []
        self.y_labels = []
        
        # Line drawing parameters
        self.colors = []
        self.line_widths = []
        self.line_styles = []
        
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
        self.last_step_time = 0.0
        self.n_plots = 0
    
    
    def add_plot(self,
                 title=None,
                 x_label=None,
                 y_label=None,
                 color=None,
                 line_width=None,
                 line_style=None,
                 tail=None,
                 x_lim=[None, None],
                 y_lim=[None, None]):
        """
        Adds a plot to the animator. This function needs to be called to 
        define a plot before that plot's data can be set or updated

        Parameters
        ----------
        title : string, optional
            The title of the plot. Will be written above the plot when
            rendered. The default is None.
        x_label : string, optional
            The label to apply to the x axis. We be written under the plot when
            rendered. The default is None.
        y_label : string, optional
            The label to apply to the y axis. We be written to the left of the
            plot when rendered. The default is None.
        color : matplotlib color string, optional
            The color of the plot lines. The default is None.
        line_width : float, optional
            The weight of the line that is plotted. The default is None.
            When set to None, defaults to 1.0.
        line_style : matplotlib line style string, optional
            The style of the line that is plotted. The default is None. When 
            set the None, defaults to solid.
        tail : int, optional
            The number of points that are used to draw the line. Only the most 
            recent data points are kept. A value of None will plot all points
            in the plot data. The default is None.
        x_lim : [float, float], optional
            The limits to apply to the x axis of the plots. A value of None
            will apply automatically updating limits to that bound of the axis.
            The default is [None, None].
        y_lim : [float, float], optional
            The limits to apply to the y axis of the plots. A value of None
            will apply automatically updating limits to that bound of the axis.
            The default is [None, None].

        Returns
        -------
        plot_index : int
            A unique integer identifier that allows future plot interation.

        """
        # Store the plot data
        self.xs.append([])
        self.ys.append([])
        self.all_xs.append([])
        self.all_ys.append([])
        self.tails.append(tail)
        self.lines.append(None)
        
        # Store the plot labels
        self.titles.append(title)
        self.x_labels.append(x_label)
        self.y_labels.append(y_label)
        
        # Store line drawing parameters
        self.colors.append(color)
        self.line_widths.append(line_width)
        self.line_styles.append(line_style)
        
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

    
    def reset_plots(self):
        """
        Removes all previously plotted data from all plots.

        Returns
        -------
        None.

        """
        # Reset plot data
        for plot_index in range(len(self.xs)):
            self.xs[plot_index] = []
            self.ys[plot_index] = []
            self.x_data_ranges[plot_index] = [np.inf, -np.inf]
            self.y_data_ranges[plot_index] = [np.inf, -np.inf]
            self.x_plot_lims[plot_index] = [None, None]
            self.y_plot_lims[plot_index] = [None, None]
            
        
    def _trim_data(self,
                   plot_index,
                   x,
                   y,
                   tail):
        """
        Trims a plot dataset (x,y) to have length tail.

        Parameters
        ----------
        plot_index : int
            The plot's unique identifier.
        x : array-like, shape(n,)
            An array of the x data.
        y : array-like, shape(n,)
            An array of the y data.
        tail : int
            The number of points that are used to draw the line. Only the most 
            recent data points are kept. A value of None will keep all data
            points.

        Returns
        -------
        x : array-like, shape(tail,)
            An array of the trimmed x data.
        y : array-like, shape(tail,)
            An array of the trimmed y data.

        """       
        # Trim data to desired length
        if tail != None and len(x) > tail:
            x = x[-tail:]
            y = y[-tail:]
        return x, y


    def _get_ran_lims(self,
                      data,
                      data_range,
                      fixed_plot_lims):
        """
        Calculates the plot limits and the current range of data.

        Parameters
        ----------
        data : array-like
            The array of data over which the range is calculated.
        data_range : array-like, shape(2,)
            The previously calculated range of data.
        fixed_plot_lims : array-like, shape(2,)
            The fixed limits to be applied to the plot axis (if any).

        Returns
        -------
        new_data_range : array-like, shape(2,)
            The newly calculated range of data.
        plot_limits : array-like, shape(2,)
            The newly calculated limits to be applied to the plot axis.

        """
        # Handle the empty case
        if len(data) == 0:
            return [-np.inf, np.inf], [None, None]
        
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


    def add_plot_point(self,
                       plot_index,
                       x,
                       y):
        """
        Adds a single data point to the plot. Data point is appended to the end
        of all previously plotted data points.

        Parameters
        ----------
        plot_index : int
            The plot's unique identifier.
        x : float
            The x value of the data point added to the plot.
        y : float
            The y value of the data point added to the plot.

        Returns
        -------
        None.

        """
        # Collect the data point
        self.xs[plot_index].append(x)
        self.ys[plot_index].append(y)
        
        # Trim data to desired length
        tail = self.tails[plot_index]
        x, y = self._trim_data(plot_index = plot_index,
                               x = self.xs[plot_index],
                               y = self.ys[plot_index],
                               tail = tail)
            
        # Update the plot data with the trimmed values
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
        """
        Calculates how many plots have been set by the user.

        Returns
        -------
        n_plots : int
            The number of plots.

        """
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
        """
        Calculates the subplot dimensions for the figure given the 
        total number of plots.

        Parameters
        ----------
        n_plots : int
            The number of plots.

        Returns
        -------
        dim : tuple, shape(2)
            The dimensions of the subplots.

        """
        # Set the maximum number of rows
        n_rows_max = 3
        
        # The number of rows should never exceed 3
        n_rows = n_plots
        if n_rows > n_rows_max:
            n_rows = n_rows_max
            
        # Get the number of columns needed
        n_cols = int(np.ceil(n_plots / n_rows_max))
            
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
        """
        Creates a single subplot and sets axis artist setting. Subsequently 
        renders axis artis updates. Does not update GUI.

        Parameters
        ----------
        axis : matplotlib.axes
            A member of the matplotlib axis class.
        title : string, optional
            The title of the plot. Will be written above the plot when
            rendered. The default is None.
        x_label : string, optional
            The label to apply to the x axis. We be written under the plot when
            rendered. The default is None.
        y_label : string, optional
            The label to apply to the y axis. We be written to the left of the
            plot when rendered. The default is None.
        x_plot_lim : [float, float], optional
            The limits to apply to the x axis of the plots. A value of None
            will apply automatically updating limits to that bound of the axis.
            The default is [None, None].
        y_plot_lim : [float, float], optional
            The limits to apply to the y axis of the plots. A value of None
            will apply automatically updating limits to that bound of the axis.
            The default is [None, None].

        Returns
        -------
        None.

        """
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
    
    
    def _on_resize(self, event):
        """
        Sets actions to do to GUI event loop upon resize event.

        Parameters
        ----------
        event : matplotlib.backend_bases.ResizeEvent
            A member of the ResizeEvent class.

        Returns
        -------
        None.

        """
        plt.ion()
        self.fig.tight_layout()
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events() 
        plt.ioff()
    
    
    def create_figure(self):
        """
        Creates, renders, and draws on GUI a figure containing all specified
        subsplots.

        Returns
        -------
        None.

        """
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
            
        # Set the layout
        self.fig.tight_layout()
        
        self.fig.canvas.mpl_connect('resize_event', self._on_resize)
        
        # Manually cycle the GUI loop
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
                      line_width,
                      line_style,
                      x_plot_lim,
                      y_plot_lim):
        """
        Rerenders a single subplot. Does not update GUI.

        Parameters
        ----------
        x : array-like, shape(n,)
            An array of the new x data.
        y : array-like, shape(n,)
            An array of the new y data.
        line : matplotlib.lines.Line2D
            The line artist that belongs to the axis.
        axis : matplotlib.axes
            The axis artist that defines the subplot being updated.
        color : matplotlib color string, optional
            The color of the plot lines. The default is None.
        line_width : float, optional
            The weight of the line that is plotted. The default is None.
            When set to None, defaults to 1.0.
        line_style : matplotlib line style string, optional
            The style of the line that is plotted. The default is None. When 
            set the None, defaults to solid.
        x_plot_lim : [float, float], optional
            The limits to apply to the x axis of the plots. A value of None
            will apply automatically updating limits to that bound of the axis.
            The default is [None, None].
        y_plot_lim : [float, float], optional
            The limits to apply to the y axis of the plots. A value of None
            will apply automatically updating limits to that bound of the axis.
            The default is [None, None].

        Returns
        -------
        line : matplotlib.lines.Line2D
            The same line artist that belongs to the axis.

        """
        # Redraw the line data
        if line==None:
            line, = axis.plot(x, y, c=color, lw=line_width, ls=line_style)
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
        """
        Takes a single animator step. If called before redraw period (defined
        by frame rate), cycle the GUI event loop and return. If called
        during a redraw period, re-render plots and update GUI. Must be called
        regularly to keep GUI fresh. If not called regularly, GUI will become 
        unresponsive. If you wish to suspend the GUI but keep it responsive,
        call the flush_events() function in a loop.

        Returns
        -------
        None.

        """
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
            line_width = self.line_widths[i]
            line_style = self.line_styles[i]
                
            # Update the plotted data
            if len(x)==len(y):
                new_line = self._step_subplot(x=x,
                                              y=y,
                                              line=line,
                                              axis=axis,
                                              color=color,
                                              line_width=line_width,
                                              line_style=line_style,
                                              x_plot_lim=x_plot_lim,
                                              y_plot_lim=y_plot_lim)
                if line==None:
                    self.lines[i] = new_line
        
        # Manually cycle the GUI loop
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events() 
        
        # Update time keeping
        self.last_step_time = time.time()
        
        # Turn off interactive mode to suspend the GUI event loop
        plt.ioff()


    def flush_events(self):
        """
        Suspends the GUI keeping it responsive. Call regularly when not
        stepping.

        Returns
        -------
        None.

        """
        plt.ion()
        self.fig.canvas.flush_events() 
        plt.ioff()