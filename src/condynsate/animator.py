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
import matplotlib as mpl
import copy


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
        self.types = []
        self.tails = []
        self.artists = []
        
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
    
    
    def _validate_inputs(self,
                         arg,
                         arg_str,
                         n_artists):
        """
        Make sure that the inputs of the add_subplot function are valid

        Parameters
        ----------
        arg : add_subplot argument
            Any of the arguments: colors, labels, line_widths, line_styles
        arg_str : string
            The string name of the argument.
        n_artists : TYPE
            The number of artists in the subplot.

        Raises
        ------
        Exception
            The argument is the wrong length.
        TypeError
            The argument is not a list.

        Returns
        -------
        None.

        """
        if arg==None:
            return
        right_length = False
        try: 
            right_length = len(arg)==n_artists
        except:
            raise TypeError(arg_str + " must be iterable.")
        if not right_length:
            raise Exception(arg_str + " must have length n_artists.")
    
    
    def add_subplot(self,
                    n_artists=1,
                    subplot_type='line',
                    title=None,
                    x_label=None,
                    y_label=None,
                    colors=None,
                    labels=None,
                    x_lim=[None, None],
                    y_lim=[None, None],
                    line_widths=None,
                    line_styles=None,
                    tail=None):
        """
        Adds a subplot to the animator. This function needs to be called to 
        define a subplot before data can be added to that plot.

        Parameters
        ----------
        n_artists : int, optional
            The number of artists that can draw on the subplot.
            The default is 1.
        subplot_type: either 'line' or 'bar', optional
            The type of plot. May either be 'line' or 'bar'. The default
            is 'line'.
        title : string, optional
            The title of the plot. Will be written above the plot when
            rendered. The default is None.
        x_label : string, optional
            The label to apply to the x axis. Will be written under the subplot
            when rendered. The default is None.
        y_label : string, optional
            The label to apply to the y axis. Will be written to the left of
            the subplot when rendered. The default is None.
        colors : list of matplotlib color string, optional
            The colors of each subplot artist. The default is None. When left 
            as default, all artists plot in black. Must be length n_artists
            if not None.
        labels : list of strings, optional
            The labels to apply to each artist in the subplot. The default is 
            None. When left as none, no labels or legend will appear in the 
            subplot. Must be length n_artists if not None.
        x_lim : [float, float], optional
            The limits to apply to the x axis of the subplot. A value of None
            will apply automatically updating limits to that bound of the axis.
            The default is [None, None].
        y_lim : [float, float], optional
            The limits to apply to the y axis of the subplot. A value of None
            will apply automatically updating limits to that bound of the axis.
            The default is [None, None].
        line_widths : list of float, optional
            The line weigth each artist uses. The default is None.
            When set to None, defaults to 1.0 for all lines.
            Must be length n_artists if not None.
        line_styles : list of matplotlib line style string, optional
            The line style each artist uses. The default is None. When 
            set the None, defaults to solid for all artists. Only used if
            subplot type is 'line'. Must be length n_artists if not None.
        tail : int, optional
            The number of points that are used to draw lines. Only the most 
            recent data points are kept. A value of None will plot all points
            in the plot data. The default is None. Only used if
            subplot type is 'line'.
            
        Raises
        ------
        Exception
            An argument is the wrong length.
        TypeError
            An argument is the wrong type.
        
        Returns
        -------
        subplot_index : int
            A unique integer identifier that allows future subplot interation.
        artist_inds : tuple of ints
            The unique integer identifiers of each line on the subplot.

        """
        # Ensure valid inputs
        self._validate_inputs(colors, 'colors', n_artists)
        self._validate_inputs(labels, 'labels', n_artists)
        self._validate_inputs(line_widths, 'line_widths', n_artists)
        self._validate_inputs(line_styles, 'line_styles', n_artists)
        
        # Store the plot data
        self.xs.append([])
        self.ys.append([])
        for i in range(n_artists):
            self.xs[-1].append([])
            self.ys[-1].append([])
            
        # Store types
        self.artists.append([])
        self.types.append(subplot_type)
        self.tails.append(tail)
        artist_inds = tuple(np.arange(n_artists).tolist())
        
        # Make a new slot for each artist
        for i in range(n_artists):
            self.artists[-1].append(None)
        
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
        return subplot_index, artist_inds

    
    def reset_plots(self):
        """
        Removes all previously plotted data from all plots.

        Returns
        -------
        None.

        """
        # Reset plot data
        for plot_index in range(len(self.xs)):
            n_artists = len(self.xs[plot_index])
            self.xs[plot_index] = []
            self.ys[plot_index] = []
            for i in range(n_artists):
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
                          artist_index,
                          x,
                          y):
        """
        Adds a single data point to the subplot. Data point is appended to the
        end of all previously plotted data points on the specified line.

        Parameters
        ----------
        subplot_index : int
            The subplot's unique identifier.
        artist_index : int
            The subplot's artist index to which the data is added.
        x : float
            The x value of the data point added to the plot.
        y : float
            The y value of the data point added to the plot.

        Returns
        -------
        None.

        """
        # Collect and trim data to desired length if line plot
        if self.types[subplot_index] == 'line':
            # Collect the data
            self.xs[subplot_index][artist_index].append(x)
            self.ys[subplot_index][artist_index].append(y)
            
            # Trim the data
            tail = self.tails[subplot_index]
            x, y = self._trim_data(x = 
                                   self.xs[subplot_index][artist_index],
                                   y = self.ys[subplot_index][artist_index],
                                   tail = tail)
            
            # Store the trimmed data
            self.xs[subplot_index][artist_index] = x
            self.ys[subplot_index][artist_index] = y
            
            # Update the x data ranges and associated plot limits
            cur_x_ran = self.x_data_ranges[subplot_index]
            fix_x = self.fixed_x_plot_lims[subplot_index]
            new_x_ran, x_lims = self._get_ran_lims(data=self.xs[subplot_index],
                                                   data_range=cur_x_ran,
                                                   fixed_plot_lims=fix_x)
            self.x_data_ranges[subplot_index] = new_x_ran
            self.x_plot_lims[subplot_index] = x_lims
        
            # Update the y data ranges and associated plot limits
            cur_y_ran = self.y_data_ranges[subplot_index]
            fix_y = self.fixed_y_plot_lims[subplot_index]
            new_y_ran, y_lims = self._get_ran_lims(data=self.ys[subplot_index],
                                                   data_range = cur_y_ran,
                                                   fixed_plot_lims = fix_y)
            self.y_data_ranges[subplot_index] = new_y_ran
            self.y_plot_lims[subplot_index] = y_lims
            
        # Keep the y value for bar chart
        elif self.types[subplot_index] == 'bar':
            self.ys[subplot_index][artist_index] = y
            
            # Update the y data ranges and plot limits
            cur_x_ran = self.x_data_ranges[subplot_index]
            fix_x = self.fixed_x_plot_lims[subplot_index]
            xs = [0]
            for i in range(len(self.xs[subplot_index])):
                x = self.xs[subplot_index][i]
                try:
                    for j in range(len(x)):
                        xx = x[j]
                        xs.append(xx)
                except:
                    xs.append(x)
            new_x_ran, x_lims = self._get_ran_lims(data=[xs],
                                                   data_range = cur_x_ran,
                                                   fixed_plot_lims = fix_x)
            self.x_data_ranges[subplot_index] = new_x_ran
            self.x_plot_lims[subplot_index] = x_lims


    def _get_n_plots(self):
        """
        Calculates how many plots have been set by the user.

        Returns
        -------
        n_plots : int
            The number of plots.

        """
        # Get the length of all plot parameters
        lens = np.zeros(18)
        lens[0] = len(self.xs)
        lens[1] = len(self.ys)
        lens[2] = len(self.types)
        lens[3] = len(self.tails)
        lens[4] = len(self.artists)
        lens[5] = len(self.titles)
        lens[6] = len(self.x_labels)
        lens[7] = len(self.y_labels)
        lens[8] = len(self.labels)
        lens[9] = len(self.colors)
        lens[10] = len(self.line_widths)
        lens[11] = len(self.line_styles)
        lens[12] = len(self.x_data_ranges)
        lens[13] = len(self.y_data_ranges)
        lens[14] = len(self.fixed_x_plot_lims)
        lens[15] = len(self.fixed_y_plot_lims)
        lens[16] = len(self.x_plot_lims)
        lens[17] = len(self.y_plot_lims)
        
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
        # Setup the rcparams to prevent unwanted keypresses
        mpl.rcParams["keymap.fullscreen"] = []
        mpl.rcParams["keymap.home"] = []
        mpl.rcParams["keymap.back"] = []
        mpl.rcParams["keymap.forward"] = []
        mpl.rcParams["keymap.pan"] = []
        mpl.rcParams["keymap.zoom"] = []
        mpl.rcParams["keymap.save"] = ['ctrl+s']
        mpl.rcParams["keymap.help"] = []
        mpl.rcParams["keymap.quit"] = []
        mpl.rcParams["keymap.quit_all"] = []
        mpl.rcParams["keymap.grid"] = []
        mpl.rcParams["keymap.grid_minor"] = []
        mpl.rcParams["keymap.yscale"] = []
        mpl.rcParams["keymap.xscale"] = []
        mpl.rcParams["keymap.copy"] = []
        
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
        
        # Tell what to do during resize
        self.fig.canvas.mpl_connect('resize_event', self._on_resize)
        
        # Manually cycle the GUI loop
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events() 
        
        # Flag that the figure is done being made
        self.figure_is_made = True
        
        # Turn off interactive mode to suspend the GUI event loop
        plt.ioff()
       
        
    def _step_line(self,
                   x,
                   y,
                   artist,
                   axis,
                   color,
                   line_width,
                   line_style,
                   x_plot_lim,
                   y_plot_lim,
                   head_marker):
        """
        Rerenders a single line subplot. Does not update GUI.

        Parameters
        ----------
        x : array-like, shape(n,)
            An array of the new x data.
        y : array-like, shape(n,)
            An array of the new y data.
        artist : matplotlib.lines.Lines2D or None
            The artist that belongs to the axis. None if it does not exist yet.
        axis : matplotlib.axes
            The axis artist that defines the subplot being updated.
        color : matplotlib color string
            The color of the plot lines.
        line_width : float
            The weight of the line that is plotted. Only if type is line.
        line_style : matplotlib line style string,
            The style of the line that is plotted. Only if type is line.
        x_plot_lim : [float, float]
            The limits to apply to the x axis of the plots. A value of None
            will apply automatically updating limits to that bound of the axis.
            Only if type is line.
        y_plot_lim : [float, float]
            The limits to apply to the y axis of the plots. A value of None
            will apply automatically updating limits to that bound of the axis.
        head_marker : boolean
            A boolean flag that indicates whether a marker will be drawn at 
            the head of the line. Only if type is line.

        Returns
        -------
        artist : matplotlib.lines.Line2D
            The same line artist that belongs to the axis.

        """
        # Create a new artist
        if artist==None:
            # Make the line artist add a marker to the head of the line
            if head_marker:
                artist, = axis.plot(x,
                                    y,
                                    c=color,
                                    lw=line_width,
                                    ls=line_style,
                                    ms=2.5*line_width,
                                    marker='o')
                
            # Do not make the line artist add a marker to the head of
            # line
            else:
                artist, = axis.plot(x,
                                    y,
                                    c=color,
                                    lw=line_width,
                                    ls=line_style)
            
        # Use a previous artist
        else:
            # If there is a head marker, adjust it
            if head_marker:
                artist.set_markevery((len(x)-1, 1))
            
            # Update the line artist's data
            artist.set_data(x, y)
            
        # Reset the axis limits
        axis.set_xlim(x_plot_lim[0],
                      x_plot_lim[1])
        axis.set_ylim(y_plot_lim[0],
                      y_plot_lim[1])
        
        # Return the line artist
        return artist
    
    
    def _step_bar(self,
                  labels,
                  heights,
                  artist,
                  axis,
                  colors,
                  line_widths,
                  x_plot_lim):
        """
        Rerenders a single bar type subplot. Does not update GUI.

        Parameters
        ----------
        labels : array-like, shape(n,)
            An array of the new bar name data.
        heights : array-like, shape(n,)
            An array of the new bar height data
        artist : matplotlib.container.BarContainer
            The artist that belongs to the axis. None if it does not exist yet.
        axis : matplotlib.axes
            The axis artist that defines the subplot being updated.
        colors : matplotlib color string
            The colors of the bars.
        line_widths : float
            The weight of the bar borders.
        x_plot_lim : [float, float]
            The limits to apply to the x axis of the plots. A value of None
            will apply automatically updating limits to that bound of the axis.

        Returns
        -------
        artist : matplotlib.container.BarContainer
            The same line artist that belongs to the axis.

        """
        # Makes sure we are not trying to plot empy data
        for i in range(len(heights)):
            h = heights[i]
            if not (isinstance(h,int) or isinstance(h,float)):
                heights[i] = 0.0
        
        # Create a new artist
        if artist==None:
            artist = axis.barh(labels,
                               heights,
                               align='center',
                               color=colors,
                               edgecolor='k',
                               linewidth=line_widths)
            
        # Use a previous artist
        else:
            for i in range(len(heights)):
                artist[i].set_width(heights[i])
            
        # Reset the y limits
        axis.set_xlim(x_plot_lim[0],
                      x_plot_lim[1])
        
        # Return the bar artist
        return artist
        
        
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
            subplot_type = self.types[subplot]
            
            # Retrieve the limit data
            subplot_x_lim = self.x_plot_lims[subplot]
            subplot_y_lim = self.y_plot_lims[subplot]
            
            # Retrieve the line artist for each axis
            subplot_artists = self.artists[subplot]
            subplot_colors = self.colors[subplot]
            subplot_line_widths = self.line_widths[subplot]
            subplot_line_styles = self.line_styles[subplot]
            
            # Determine if a head marker will be drawn
            head_marker = self.tails[subplot] != None
            
            # Update the plotted line data
            if subplot_type == 'line':
                
                # Go through each artist one at a time
                for artist_num in range(len(subplot_xs)):
                    # Get the x and y data of the artist
                    x = subplot_xs[artist_num]
                    y = subplot_ys[artist_num]
                    
                    # Get the artist
                    artist = subplot_artists[artist_num]
                    
                    # Get the artists's color
                    if subplot_colors == None:
                        color = "k"
                    else:
                        color = subplot_colors[artist_num]
                    
                     # Get the line's line width
                    if subplot_line_widths == None:
                        line_width = 1.0
                    else:
                        line_width = subplot_line_widths[artist_num]
                    
                    # Get the line's line style
                    if subplot_line_styles == None:
                        line_style = "-"
                    else:
                        line_style = subplot_line_styles[artist_num]
                    
                    # Update the line
                    new_artist = self._step_line(x=x,
                                                 y=y,
                                                 artist=artist,
                                                 axis=subplot_axis,
                                                 color=color,
                                                 line_width=line_width,
                                                 line_style=line_style,
                                                 x_plot_lim=subplot_x_lim,
                                                 y_plot_lim=subplot_y_lim,
                                                 head_marker=head_marker)
    
                    # Store the artist that was drawn if it is new
                    if artist==None:
                        self.artists[subplot][artist_num] = new_artist
                        
                        # Add a legend if needed
                        if self.labels[subplot] != None:
                            subplot_axis.legend(self.labels[subplot],
                                                loc="upper right")
        
            elif subplot_type == 'bar':
                # Default the bars labels
                if self.labels[subplot] == None:
                    labels = [None]*len(subplot_ys)
                else:
                    labels = self.labels[subplot]
                
                # Default the bars' colors
                if subplot_colors == None:
                    subplot_colors = 'k'
                
                 # Default the bars' line widths
                if subplot_line_widths == None:
                    subplot_line_widths = 0.0
                        
                # Step the bar chart
                new_artist = self._step_bar(labels=labels,
                                            heights=subplot_ys,
                                            artist=subplot_artists[0],
                                            axis=subplot_axis,
                                            colors=subplot_colors,
                                            line_widths=subplot_line_widths,
                                            x_plot_lim=subplot_x_lim)
                
                # Store the line that was drawn if it is new
                if subplot_artists[0]==None:
                    self.artists[subplot][0] = new_artist
        
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
        