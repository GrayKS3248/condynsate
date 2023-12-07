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


###############################################################################
#ANIMATOR CLASS
###############################################################################
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
        self.tails = []
        self.lines = []
        
        # Plot labels
        self.titles = []
        self.x_labels = []
        self.y_labels = []
        self.labels = []
        
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
    
    
    def add_subplot(self,
                    n_lines=1,
                    title=None,
                    x_label=None,
                    y_label=None,
                    colors=None,
                    line_widths=None,
                    line_styles=None,
                    labels=None,
                    tail=None,
                    x_lim=[None, None],
                    y_lim=[None, None]):
        """
        Adds a subplot to the animator. This function needs to be called to 
        define a subplot before data can be added to that plot.

        Parameters
        ----------
        n_lines : int, optional
            The number of lines to which data can be drawn on the subplot.
            The default is 1.
        title : string, optional
            The title of the plot. Will be written above the plot when
            rendered. The default is None.
        x_label : string, optional
            The label to apply to the x axis. We be written under the plot when
            rendered. The default is None.
        y_label : string, optional
            The label to apply to the y axis. We be written to the left of the
            plot when rendered. The default is None.
        colors : list of matplotlib color string, optional
            The colors of each subplot line. The default is None. When left 
            as default, all lines are plotted black. 
        line_widths : list of float, optional
            The weight of each line in a subplot. The default is None.
            When set to None, defaults to 1.0 for all lines.
        line_styles : list of matplotlib line style string, optional
            The style of each line in the subplot. The default is None. When 
            set the None, defaults to solid for all lines.
        labels : list of strings, optional
            The labels to apply to each line in the subplot. The default is 
            None. When left as none, no labels or legend will appear in the 
            subplot. Must be length n_lines if not None.
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
        subplot_index : int
            A unique integer identifier that allows future subplot interation.
        line_indices : tuple of ints
            The unique integer identifiers of each line on the subplot.

        """
        # Store the plot data
        self.xs.append([])
        self.ys.append([])
        self.lines.append([])
        line_indices = tuple(np.arange(n_lines).tolist())
        for i in range(n_lines):
            self.xs[-1].append([])
            self.ys[-1].append([])
            self.lines[-1].append(None)
        self.tails.append(tail)
        
        # Store the plot labels
        self.titles.append(title)
        self.x_labels.append(x_label)
        self.y_labels.append(y_label)
        self.labels.append(labels)
        
        # Store line drawing parameters
        self.colors.append(colors)
        self.line_widths.append(line_widths)
        self.line_styles.append(line_styles)
        
        # Store the limit options
        self.x_data_ranges.append([np.inf, -np.inf])
        self.y_data_ranges.append([np.inf, -np.inf])
        self.fixed_x_plot_lims.append(x_lim)
        self.fixed_y_plot_lims.append(y_lim)
        self.x_plot_lims.append([None, None])
        self.y_plot_lims.append([None, None])
        
        # Return the index of the added plot
        subplot_index = len(self.xs)-1
        return subplot_index, line_indices

    
    def reset_plots(self):
        """
        Removes all previously plotted data from all plots.

        Returns
        -------
        None.

        """
        # Reset plot data
        for plot_index in range(len(self.xs)):
            n_lines = len(self.xs[plot_index])
            self.xs[plot_index] = []
            self.ys[plot_index] = []
            for i in range(n_lines):
                self.xs[plot_index].append([])
                self.ys[plot_index].append([])
            self.x_data_ranges[plot_index] = [np.inf, -np.inf]
            self.y_data_ranges[plot_index] = [np.inf, -np.inf]
            self.x_plot_lims[plot_index] = [None, None]
            self.y_plot_lims[plot_index] = [None, None]
            
        
    def _trim_data(self,
                   x,
                   y,
                   tail):
        """
        Trims a line of a subplot to have length tail.

        Parameters
        ----------
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
        data : array of arrays
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
        if all([len(datum)==0 for datum in data]):
            return [-np.inf, np.inf], [None, None]
        
        # Variables to hold the calulcated plot limits
        plot_limits = [0., 0.]
        plot_limits[0] = fixed_plot_lims[0]
        plot_limits[1] = fixed_plot_lims[1]
        
        # Data range
        data_min = np.inf
        data_max = -np.inf
        for datum in data:
            if len(datum) > 0:
                curr_min = np.min(datum)
                curr_max = np.max(datum)
                if curr_min < data_min:
                    data_min = curr_min
                if curr_max > data_max:
                        data_max = curr_max
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


    def add_subplot_point(self,
                          subplot_index,
                          line_index,
                          x,
                          y):
        """
        Adds a single data point to the subplot. Data point is appended to the
        end of all previously plotted data points on the specified line.

        Parameters
        ----------
        subplot_index : int
            The subplot's unique identifier.
        line_index : int
            The subplot's line index to which the data is added.
        x : float
            The x value of the data point added to the plot.
        y : float
            The y value of the data point added to the plot.

        Returns
        -------
        None.

        """
        # Collect the data point
        self.xs[subplot_index][line_index].append(x)
        self.ys[subplot_index][line_index].append(y)
        
        # Trim data to desired length
        tail = self.tails[subplot_index]
        x, y = self._trim_data(x = self.xs[subplot_index][line_index],
                               y = self.ys[subplot_index][line_index],
                               tail = tail)
            
        # Update the plot data with the trimmed values
        self.xs[subplot_index][line_index] = x
        self.ys[subplot_index][line_index] = y
        
        # Update the x data ranges and associated plot limits
        cur_x_range = self.x_data_ranges[subplot_index]
        fix_x_lims = self.fixed_x_plot_lims[subplot_index]
        new_x_range, x_lims = self._get_ran_lims(data = self.xs[subplot_index],
                                                 data_range = cur_x_range,
                                                 fixed_plot_lims = fix_x_lims)
        self.x_data_ranges[subplot_index] = new_x_range
        self.x_plot_lims[subplot_index] = x_lims
        
        # Update the y data ranges and associated plot limits
        cur_y_range = self.y_data_ranges[subplot_index]
        fix_y_lims = self.fixed_y_plot_lims[subplot_index]
        new_y_range, y_lims = self._get_ran_lims(data = self.ys[subplot_index],
                                                 data_range = cur_y_range,
                                                 fixed_plot_lims = fix_y_lims)
        self.y_data_ranges[subplot_index] = new_y_range
        self.y_plot_lims[subplot_index] = y_lims


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
        title : string
            The title of the plot. Will be written above the plot when
            rendered.
        x_label : string
            The label to apply to the x axis. We be written under the plot when
            rendered.
        y_label : string
            The label to apply to the y axis. We be written to the left of the
            plot when rendered.
        x_plot_lim : [float, float]
            The limits to apply to the x axis of the plots. A value of None
            will apply automatically updating limits to that bound of the axis.
        y_plot_lim : [float, float]
            The limits to apply to the y axis of the plots. A value of None
            will apply automatically updating limits to that bound of the axis.

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
                      y_plot_lim,
                      head_marker):
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
        line_width : float
            The weight of the line that is plotted.
        line_style : matplotlib line style string,
            The style of the line that is plotted.
        x_plot_lim : [float, float]
            The limits to apply to the x axis of the plots. A value of None
            will apply automatically updating limits to that bound of the axis.
        y_plot_lim : [float, float]
            The limits to apply to the y axis of the plots. A value of None
            will apply automatically updating limits to that bound of the axis.
        head_marker : boolean
            A boolean flag that indicates whether a marker will be drawn at 
            the head of the line.

        Returns
        -------
        line : matplotlib.lines.Line2D
            The same line artist that belongs to the axis.

        """
        # Redraw the line data
        if line==None:
            if head_marker:
                line, = axis.plot(x,
                                  y,
                                  c=color,
                                  lw=line_width,
                                  ls=line_style,
                                  ms=2.5*line_width,
                                  marker='o')
            else:
                line, = axis.plot(x,
                                  y,
                                  c=color,
                                  lw=line_width,
                                  ls=line_style)
        else:
            if head_marker:
                line.set_markevery((len(x)-1, 1))
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
        for subplot in range(self.n_plots):
            # Get the axis on which the subplot is drawn
            if self.n_plots > 1:
                subplot_axis = self.axes[subplot]
            else:
                subplot_axis = self.axes    
    
            # Retrieve the plot data
            subplot_xs = self.xs[subplot]
            subplot_ys = self.ys[subplot]
            
            # Retrieve the limit data
            subplot_x_lim = self.x_plot_lims[subplot]
            subplot_y_lim = self.y_plot_lims[subplot]
            
            # Retrieve the line artist for each axis
            subplot_lines = self.lines[subplot]
            subplot_colors = self.colors[subplot]
            subplot_line_widths = self.line_widths[subplot]
            subplot_line_styles = self.line_styles[subplot]
            
            # Determine if a head marker will be drawn
            head_marker = self.tails[subplot] != None
            
            # Update the plotted data
            for line in range(len(subplot_xs)):
                # Get the x and y data of the line in the subplot
                line_x = subplot_xs[line]
                line_y = subplot_ys[line]
                
                # Get the line object being drawn to
                line_line = subplot_lines[line]
                
                # Get the line's color
                if subplot_colors == None:
                    line_color = "k"
                else:
                    line_color = subplot_colors[line]
                
                 # Get the line's line width
                if subplot_line_widths == None:
                    line_line_width = 1.0
                else:
                    line_line_width = subplot_line_widths[line]
                
                # Get the line's line style
                if subplot_line_styles == None:
                    line_line_style = "-"
                else:
                    line_line_style = subplot_line_styles[line]
                
                # Update the line
                new_line = self._step_subplot(x=line_x,
                                              y=line_y,
                                              line=line_line,
                                              axis=subplot_axis,
                                              color=line_color,
                                              line_width=line_line_width,
                                              line_style=line_line_style,
                                              x_plot_lim=subplot_x_lim,
                                              y_plot_lim=subplot_y_lim,
                                              head_marker=head_marker)
                
                # Store the line that was drawn if it is new
                if line_line==None:
                    self.lines[subplot][line] = new_line
                    if self.labels[subplot] != None:
                        subplot_axis.legend(self.labels[subplot],
                                            loc="upper right")
        
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
        