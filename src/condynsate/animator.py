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
    
    
    ###########################################################################
    #STORE NEW SUBPLOT DATA STYLE INFORMATION
    ###########################################################################
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
                    x_lim=[None, None],
                    y_lim=[None, None],
                    colors=None,
                    labels=None,
                    line_widths=None,
                    line_styles=None,
                    tail=None):
        """
        Adds a subplot to the animator. This function needs to be called to 
        define a subplot before data can be added to that plot.

        Parameters
        ----------
        n_artists : int, optional
            The number of artists that draw on the subplot
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
        x_lim : [float, float], optional
            The limits to apply to the x axis of the subplot. A value of None
            will apply automatically updating limits to the corresponding
            bound of the axis. For example [None, 10.] will fix the upper
            bound to exactly 10, but the lower bound will freely change to
            show all data.The default is [None, None].
        y_lim : [float, float], optional
            The limits to apply to the y axis of the subplot. A value of None
            will apply automatically updating limits to the corresponding
            bound of the axis. For example [None, 10.] will fix the upper
            bound to exactly 10, but the lower bound will freely change to
            show all data.The default is [None, None].
        colors : list of matplotlib color string, optional
            A list of the color each artist draws in. Must have length
            n_artists. If n_artists = 1, has the form ['COLOR']. When None,
            all artists will default to drawing in black. The default is None.
        labels : list of strings, optional
            A list of the label applied to each artist. For line charts, 
            the labels are shown in a legend in the top right of the plot. For
            bar charts, the labels are shown on the y axis next to their 
            corresponging bars. Must have length n_artists. If n_artists = 1,
            has the form ['LABEL']. When None, no labels will be made for any
            aritsts. The default is None.
        line_widths : list of floats, optional
            The line weigth each artist uses. For line plots, this is the
            width of the plotted line, for bar charts, this is the width of 
            the border around each bar. Must be length n_artists. If
            n_artists = 1, has the form [LINE_WIDTH]. When set to None,
            defaults to 1.0 for all lines. The default is None.
        line_styles : list of matplotlib line style string, optional
            The line style each artist uses. For line plots, this is the
            style of the plotted line, for bar charts, this argument is not
            used and therefore ignored. Must be length n_artists. If
            n_artists = 1, has the form ['LINE_STYLE']. When set to None,
            defaults to 'solid' for all lines. The default is None.
        tail : int, optional
            Specifies how many data points are used to draw a line. Only the
            most recently added data points are kept. Any data points added
            more than tail data points ago are discarded and not plotted. Only
            valid for line plots, and applied to all artists in the plot. For 
            bar plots, this argument is ignored and not used. A value of None
            means that no data is ever discarded and all data points added to
            the animator will be drawn. The default is None.
            
        Raises
        ------
        Exception
            At least one of the arguments colors, labels, line_widths, or 
            line_styles do not have length n_artists.
        TypeError
            At least one of the arguments colors, labels, line_widths, or 
            line_styles is not a list.
        
        Returns
        -------
        subplot_index : int
            A integer identifier that is unique to the subplot created. 
            This allows future interaction with this subplot (adding data
            points, etc.).
        artist_inds : tuple of ints
            A tuple of integer identifiers that are unique to the artist
            created. This allows future interaction with these artists (adding
            data points, etc.).
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
        self.x_data_ranges.append([-np.inf, np.inf])
        self.y_data_ranges.append([-np.inf, np.inf])
        self.fixed_x_plot_lims.append(x_lim)
        self.fixed_y_plot_lims.append(y_lim)
        self.x_plot_lims.append([None, None])
        self.y_plot_lims.append([None, None])
        
        # Return the index of the added plot
        subplot_index = len(self.xs)-1
        return subplot_index, artist_inds


    ###########################################################################
    #RESET ALL SUBPLOTS
    ###########################################################################
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
            self.x_data_ranges[plot_index] = [-np.inf, np.inf]
            self.y_data_ranges[plot_index] = [-np.inf, np.inf]
            self.x_plot_lims[plot_index] = [None, None]
            self.y_plot_lims[plot_index] = [None, None]
            
        
    ###########################################################################
    #CREATE FIGURE
    ###########################################################################    
    def _disable_keystrokes(self):
        """
        Disables most keystrokes for the matplotlib QT widget to prevent
        unexpected interactions while simulations are occuring.

        Returns
        -------
        None.

        """
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
    
    
    def _get_lims(self,
                  data,
                  data_range,
                  fixed_lims):
        """
        Calculates the plot limits and the current range of data.

        Parameters
        ----------
        data : array of arrays
            The array of data over which the range is calculated.
        data_range : array-like, shape(2,)
            The previously calculated range of data.
        fixed_lims : array-like, shape(2,)
            The fixed limits to be applied to the plot axis (if any).

        Returns
        -------
        new_data_range : array-like, shape(2,)
            The newly calculated range of data.
        plot_limits : array-like, shape(2,)
            The newly calculated limits to be applied to the plot axis.

        """
        # Variables to hold the calulcated plot limits
        plot_limits = [0., 0.]
        plot_limits[0] = fixed_lims[0]
        plot_limits[1] = fixed_lims[1]
        
        # Handle the empty case
        if all([len(datum)==0 for datum in data]):
            return [-np.inf, np.inf], plot_limits
        
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


    def _update_limits(self,
                       subplot_index):
        """
        Updates the currently stored data ranges and plot limits for 
        both x and y axes for the subplot given.

        Parameters
        ----------
        subplot_index : int
            The subplot's unique identifier.

        Returns
        -------
        None.
        """
        
        # Collect the current relevant information for the x axis
        data_range = self.x_data_ranges[subplot_index]
        fixed_lims = self.fixed_x_plot_lims[subplot_index]
        data = self.xs[subplot_index]
        
        # Calcaulate the new data range and the new plot limits for the x axis
        data_range, plot_lims = self._get_lims(data=data,
                                               data_range=data_range,
                                               fixed_lims=fixed_lims)
        
        # Apply the new data range and new plot limits for the x axis
        self.x_data_ranges[subplot_index] = data_range
        self.x_plot_lims[subplot_index] = plot_lims
    
        # Collect the current relevant information for the y axis
        data_range = self.y_data_ranges[subplot_index]
        fixed_lims = self.fixed_y_plot_lims[subplot_index]
        data = self.ys[subplot_index]
        
        # Calcaulate the new data range and the new plot limits for the y axis
        data_range, plot_lims = self._get_lims(data=data,
                                               data_range=data_range,
                                               fixed_lims=fixed_lims)
        
        # Apply the new data range and new plot limits for the y axis
        self.y_data_ranges[subplot_index] = data_range
        self.y_plot_lims[subplot_index] = plot_lims
    
    
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
            
            
    def _get_artist_params(self,
                           subplot_index,
                           artist_index):
        """
        Gets style parameters for a given artist of a given subplot

        Parameters
        ----------
        subplot_index : int
            A subplot's unique identifier.
        artist_index : int
            A subplot's artist's unique identifier.

        Returns
        -------
        color : matplotlib color string
            The artist color.
        line_width : float
            The artist line width.
        line_style : matplotlib line style string
            The artist linestyle.
        label : string
            The artist label
        head_marker : bool
            A boolean flag that indicates whether the artist will apply a 
            marker to the end of a line plot. Only applies to line plots

        """
        # Get the color
        try:
            color = self.colors[subplot_index][artist_index]
        except:
             color = 'k'  
             
        # Get the line width
        try:
            line_width = self.line_widths[subplot_index][artist_index]
        except:
            line_width = 1.0
            
        # Get the line style
        try:
            line_style = self.line_styles[subplot_index][artist_index]
        except:
            line_style = "-"
        
        # Get the label
        try:
            label = self.labels[subplot_index][artist_index]
        except:
            label = None
        
        # Get the head marker boolean
        if self.types[subplot_index] == 'line':
            head_marker = (self.tails[subplot_index] != None)
        else:
            head_marker = False
        
        # Return parameters
        return color, line_width, line_style, label, head_marker
            
            
    def _make_line_artist(self,
                          axis,
                          color,
                          line_width,
                          line_style,
                          label,
                          head_marker):
        """
        Creates a line artist with the specified parameters. Makes a single
        point at the origin.

        Parameters
        ----------
        axis : matplotlib.axes
            The axis artist that defines the subplot being updated.
        color : matplotlib color string
            The color of the plot lines.
        line_width : float
            The weight of the line that is plotted. Only if type is line.
        line_style : matplotlib line style string,
            The style of the line that is plotted. Only if type is line.
        label : string
            The label to apply to the artist.
        head_marker : boolean
            A boolean flag that indicates whether a marker will be placed
            at the head of the dataset.

        Returns
        -------
        artist : matplotlib.lines.Line2D
            The same line artist that belongs to the axis.

        """
        # Make the artist by plotting a single point at the origin
        # Because a head marker is wanted, add a marker to this point
        if head_marker:
            artist, = axis.plot([0],
                                [0],
                                c=color,
                                lw=line_width,
                                ls=line_style,
                                label=label,
                                ms=2.5*line_width,
                                marker='o')
            return artist
        
        # If a head marker is not wanted, simply plot on point at 0,0
        artist, = axis.plot([0],
                            [0],
                            c=color,
                            lw=line_width,
                            ls=line_style,
                            label=label)
        return artist
        
        
    def _make_bar_artist(self,
                         axis,
                         colors,
                         line_widths,
                         labels):
        """
        Creates a bar artist with the specified parameters. Makes bar width 0.

        Parameters
        ----------
        axis : matplotlib.axes
            The axis artist that defines the subplot being updated.
        colors : list of matplotlib color string
            The colors of the bars.
        line_widths : list of float
            The weight of the lines of the bars.
        labels : list of string
            The labels to apply to each bar

        Returns
        -------
        artists : list of matplotlib.patches.Rectangle
            The bar artists that vary bar length.

        """        
        # Set all data to 0
        data = [0. for label in labels]
        
        # Make sure the labels aren't None
        if None in labels:
            for i in range(len(labels)):
                if labels[i] == None:
                    labels[i] = "UNAMED_"+str(i+1)

        # Make a bar chart
        container = axis.barh(labels,
                              data,
                              align='center',
                              color=colors,
                              edgecolor='k',
                              linewidth=line_widths)
        
        # Extract the rectangle artists from the container
        artists = [artist for artist in container]
        return artists
            
    
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
        self._disable_keystrokes()
        
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
        for subplot_index in range(self.n_plots):
            # Get the axis on which the subplot is drawn
            if self.n_plots > 1:
                axis = self.axes[subplot_index]
            else:
                axis = self.axes    
            
            # Retrieve the label data
            title = self.titles[subplot_index]
            x_label = self.x_labels[subplot_index]
            y_label = self.y_labels[subplot_index]
            plt_type = self.types[subplot_index]
            
            # Retrieve the limit data
            self._update_limits(subplot_index)
            x_plot_lim = self.x_plot_lims[subplot_index]
            y_plot_lim = self.y_plot_lims[subplot_index]
            
            # Create the subplot
            self._create_subplot(axis=axis,
                                 title=title,
                                 x_label=x_label,
                                 y_label=y_label,
                                 x_plot_lim=x_plot_lim,
                                 y_plot_lim=y_plot_lim)
            
            # Create line artists for subplot
            if plt_type == 'line':
                for artist_index in range(len(self.artists[subplot_index])):
                    # Retrieve the style parameters of the current artist
                    params = self._get_artist_params(subplot_index,
                                                     artist_index)
                                                           
                    # Make the line artist                                 
                    artist = self._make_line_artist(axis=axis,
                                                    color=params[0],
                                                    line_width=params[1],
                                                    line_style=params[2],
                                                    label=params[3],
                                                    head_marker=params[4])
                    
                    # Store the made artist
                    self.artists[subplot_index][artist_index] = artist
                    
                # Check if a legend should be added, and then add it if needed
                if self.labels[subplot_index] != None:
                    axis.legend(self.labels[subplot_index],
                                loc="upper right")
                    
            # Make bar artists for subplot
            elif plt_type == 'bar':
                # Get the artist params
                n_bars = len(self.artists[subplot_index])
                colors = []
                line_widths = []
                labels = []
                for bar in range(n_bars):
                    # Retrieve the style parameters of the current artist
                    params = self._get_artist_params(subplot_index, bar)
                    colors.append(params[0])
                    line_widths.append(params[1])
                    labels.append(params[3])
                
                # Make each artist

                artists = self._make_bar_artist(axis=axis,
                                                colors=colors,
                                                line_widths=line_widths,
                                                labels=labels)
                for artist_index in range(len(artists)):
                    artist = artists[artist_index]
                    self.artists[subplot_index][artist_index] = artist
            
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
       
        
    ###########################################################################
    #ADD DATA POINT TO SUBPLOT
    ###########################################################################
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
    
    
    def _add_line_point(self,
                        subplot_index,
                        artist_index,
                        x,
                        y):
        """
        Adds a single point to a line type subplot.

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
        # Add the data point
        self.xs[subplot_index][artist_index].append(x)
        self.ys[subplot_index][artist_index].append(y)
        
        # Trim the data
        tail = self.tails[subplot_index]
        x, y = self._trim_data(x = 
                               self.xs[subplot_index][artist_index],
                               y = self.ys[subplot_index][artist_index],
                               tail = tail)
        
        # Update the stored data to the trimmed version
        self.xs[subplot_index][artist_index] = x
        self.ys[subplot_index][artist_index] = y


    def _add_bar_point(self,
                       subplot_index,
                       artist_index,
                       width):
        """
        Adds a single point to a bar type subplot.

        Parameters
        ----------
        subplot_index : int
            The subplot's unique identifier.
        artist_index : int
            The subplot's artist index to which the data is added.
        width : float
            The updated width of the bar chart artist.

        Returns
        -------
        None.

        """
        # Store the data point
        self.xs[subplot_index][artist_index] = [0., width]
        
        
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
        # Add a data point to a line type subplot
        if self.types[subplot_index] == 'line':
            self._add_line_point(subplot_index=subplot_index,
                                 artist_index=artist_index,
                                 x=x,
                                 y=y)
            
        # Add a point to a bar type subplot
        if self.types[subplot_index] == 'bar':
            self._add_bar_point(subplot_index=subplot_index,
                                artist_index=artist_index,
                                width=x)
            
        # Update the x and y limits of the plot
        self._update_limits(subplot_index)
        

    ###########################################################################
    #STEP ANIMATOR FORWARD ONE TIME STEP
    ###########################################################################
    def _step_line(self,
                   x,
                   y,
                   artist,
                   axis,
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
            The artist that belongs to the axis.
        axis : matplotlib.axes
            The axis artist that defines the subplot being updated.
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
        None.

        """
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
    
    
    def _step_bar(self,
                  width,
                  artist,
                  axis,
                  x_plot_lim):
        """
        Rerenders a single bar in a subplot. Does not update GUI.

        Parameters
        ----------
        width : float
            The new bar width
        artist : matplotlib.patches.Rectangle
            The bar artist
        axis : matplotlib.axes
            The axis artist that defines the subplot being updated.
        x_plot_lim : [float, float]
            The limits to apply to the x axis of the plots. A value of None
            will apply automatically updating limits to that bound of the axis.

        Returns
        -------
        None.
        
        """
        # Makes sure we are not trying to plot empty data
        if width == []:
            width = [0.0]
        
        # Change the widths
        artist.set_width(width[-1])
            
        # Reset the x limits
        axis.set_xlim(x_plot_lim[0],
                      x_plot_lim[1])
        
        # Return the bar container
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
            
            # Determine if head markers are used for the line plots
            head_marker = (self.tails[subplot] != None)

            # Go through each artist one at a time
            for artist_num in range(len(subplot_xs)):
                # Get the x and y data of the artist
                x = subplot_xs[artist_num]
                y = subplot_ys[artist_num]
                
                # Get the artist
                artist = subplot_artists[artist_num]
                
                # Update the plotted line data
                if subplot_type == 'line':
                    self._step_line(x=x,
                                    y=y,
                                    artist=artist,
                                    axis=subplot_axis,
                                    x_plot_lim=subplot_x_lim,
                                    y_plot_lim=subplot_y_lim,
                                    head_marker=head_marker)
                    
                # Update the plotted bar data
                elif subplot_type == 'bar':
                    self._step_bar(width=x,
                                   artist=artist,
                                   axis=subplot_axis,
                                   x_plot_lim=subplot_x_lim)
        
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
        