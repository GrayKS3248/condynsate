"""
This module provides the Animator class and all associated classes and 
functions that are used by it.
"""


###############################################################################
#DEPENDENCIES
###############################################################################
from threading import (Thread, Lock)
import sys
from copy import copy
import warnings
import time
import matplotlib.pyplot as plt
import numpy as np
import cv2
from condynsate.misc import print_exception
FONT_SIZE = 7

###############################################################################
#ANIMATOR CLASS
###############################################################################
class Animator():
    """
    Animator manages the real time plotting in QT GUI.
    
    Parameters
    ----------
    fr : float, optional
        The frame rate (frames per second) at which the animated plots are
        updated. The default is 8.0.
        
    """
    def __init__(self, fr=8.0):
        """
        Constructor method.
        """
        # Plot data
        self.xs = []
        self.ys = []
        self.types = []
        self.tails = []
        self.artists = []
        self.n_plots = 0
        self.h_zero_line = []
        self.v_zero_line = []
        
        # Plot labels
        self.titles = []
        self.x_labels = []
        self.y_labels = []
        self.labels = []
        
        # Artist parameters
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
        self.window_name = 'condynsate: Animator'
        self.lock = Lock()
        self.figure_is_made = False
        self.done = True
        
        # Frame rate data
        self.fr = fr
        self.last_step_time = 0.0
    
    
    def __del__(self):
        self.terminate_animator()
    
    
    ###########################################################################
    #STORE NEW PLOT DATA STYLE INFORMATION
    ###########################################################################
    def _validate_inputs(self,
                         arg,
                         default,
                         arg_str,
                         n_artists):
        """
        Make sure that the inputs of the add_plot function are valid

        Parameters
        ----------
        arg : add_plot argument
            Any of the arguments: colors, labels, line_widths, line_styles
        default : TYPE
            The default value of arg
        arg_str : string
            The string name of the argument.
        n_artists : int
            The number of artists in the plot.

        Raises
        ------
        Exception
            The argument is the wrong length.

        Returns
        -------
        None.

        """
        arg_prime = copy(arg)
        if not isinstance(arg, list) and not isinstance(arg, tuple):
            if arg is None:
                arg = default
            arg = [arg,]*n_artists
        arg = list(arg)
        if len(arg) != n_artists:
            err = "Could not parse {}: {} to n_artists={}"
            raise TypeError(err.format(arg_str, arg_prime, n_artists))
        return arg
    
    
    def add_plot(self,
                 n_artists=1,
                 plot_type='line',
                 title=None,
                 x_label=None,
                 y_label=None,
                 x_lim=[None, None],
                 y_lim=[None, None],
                 h_zero_line=False,
                 v_zero_line=False,
                 color='black',
                 label=None,
                 line_width=1.5,
                 line_style='solid',
                 tail=-1):
        """
        Adds a plot to the animator. This function needs to be called to 
        define a plot before data can be added to that plot. If called after
        start_animator, this function will do nothing.

        Parameters
        ----------
        n_artists : int, optional
            The number of artists that draw on the plot
            The default is 1.
        plot_type: either 'line' or 'bar', optional
            The type of plot. May either be 'line' or 'bar'. The default
            is 'line'.
        title : string, optional
            The title of the plot. Will be written above the plot when
            rendered. The default is None.
        x_label : string, optional
            The label to apply to the x axis. Will be written under the plot
            when rendered. The default is None.
        y_label : string, optional
            The label to apply to the y axis. Will be written to the left of
            the plot when rendered. The default is None.
        x_lim : [float, float], optional
            The limits to apply to the x axis of the plot. A value of None
            will apply automatically updating limits to the corresponding
            bound of the axis. For example [None, 10.] will fix the upper
            bound to exactly 10, but the lower bound will freely change to
            show all data.The default is [None, None].
        y_lim : [float, float], optional
            The limits to apply to the y axis of the plot. A value of None
            will apply automatically updating limits to the corresponding
            bound of the axis. For example [None, 10.] will fix the upper
            bound to exactly 10, but the lower bound will freely change to
            show all data.The default is [None, None].
        h_zero_line : boolean, optional
            A boolean flag that indicates whether a horizontal line will be
            drawn on the y=0 line. The default is false
        v_zero_line : boolean, optional
            A boolean flag that indicates whether a vertical line will be
            drawn on the x=0 line. The default is false
        color : matplotlib color string or tuple of color strings, optional
            The color each artist draws in. When tuple, must have length 
            n_artists. The default is 'black'.
        label : string or tuple of strings, optional
            The label applied to each artist. For line charts, 
            the labels are shown in a legend in the top right of the plot. For
            bar charts, the labels are shown on the y axis next to their 
            corresponging bars. When tuple, must have length n_artists.
            When None, no labels are made. The default is None.
        line_width : float or tuple of floats, optional
            The line weigth each artist uses. For line plots, this is the
            width of the plotted line, for bar charts, this is the width of 
            the border around each bar. When tuple, must have length n_artists.
            The default is 1.5.
        line_style : line style string or tuple of ls strings, optional
            The line style each artist uses. For line plots, this is the
            style of the plotted line, for bar charts, this argument is not
            used and therefore ignored. When tuple, must have length n_artists.
            The default is 'solid'.
        tail : int or tuple of ints optional
            Specifies how many data points are used to draw a line. Only the
            most recently added data points are kept. Any data points added
            more than tail data points ago are discarded and not plotted. Only
            valid for line plots. When tuple, must have length n_artists. A 
            value less than or equal to 0 means that no data is ever discarded 
            and all data points added to the animator will be drawn. 
            The default is -1.
            
        Raises
        ------
        TypeError
            At least one of the arguments color, label, line_width, 
            line_style, or tail is not parasble to all artists.
        
        Returns
        -------
        plot_id : int
            A integer identifier that is unique to the plot created. 
            This allows future interaction with this plot (adding data
            points, etc.).
        artist_ids : tuple of ints, optional
            A tuple of integer identifiers that are unique to the artist
            created. This allows future interaction with these artists (adding
            data points, etc.). Is only returned when n_artists > 1.
        """
        # Ensure figure is not already open
        if self.figure_is_made:
            return None, None

        # Ensure valid inputs
        color=self._validate_inputs(color,'k','color',n_artists)
        label=self._validate_inputs(label,None,'label',n_artists)
        line_width=self._validate_inputs(line_width,1.5,'line_width',n_artists)
        line_style=self._validate_inputs(line_style,'-','line_style',n_artists)
        tail=self._validate_inputs(tail,-1,'tail',n_artists)
        
        # Store the plot data
        self.xs.append([])
        self.ys.append([])
        for i in range(n_artists):
            self.xs[-1].append([])
            self.ys[-1].append([])
            
        # Store types
        self.artists.append([])
        self.types.append(plot_type)
        
        # Make a new slot for each artist
        for i in range(n_artists):
            self.artists[-1].append(None)
        
        # Store the plot labels
        self.titles.append(title)
        self.x_labels.append(x_label)
        self.y_labels.append(y_label)
        self.h_zero_line.append(h_zero_line)
        self.v_zero_line.append(v_zero_line)
        
        # Store line drawing parameters
        self.colors.append(color)
        self.labels.append(label)
        self.line_widths.append(line_width)
        self.line_styles.append(line_style)
        self.tails.append(tail)
        
        # Store the limit options
        self.x_data_ranges.append([-np.inf, np.inf])
        self.y_data_ranges.append([-np.inf, np.inf])
        self.fixed_x_plot_lims.append(x_lim)
        self.fixed_y_plot_lims.append(y_lim)
        self.x_plot_lims.append([None, None])
        self.y_plot_lims.append([None, None])
        
        # Return the index of the added plot
        plot_id = len(self.xs)-1
        if n_artists <= 1:
            return plot_id
        artist_inds = tuple(np.arange(n_artists).tolist())
        return plot_id, artist_inds


    ###########################################################################
    #RESET ALL PLOTS
    ###########################################################################
    def reset_animator(self):
        """
        Removes all previously plotted data from all plots.

        Returns
        -------
        None.

        """
        # Reset plot data
        with self.lock:
            for plot_id in range(len(self.xs)):
                n_artists = len(self.xs[plot_id])
                self.xs[plot_id] = []
                self.ys[plot_id] = []
                for i in range(n_artists):
                    self.xs[plot_id].append([])
                    self.ys[plot_id].append([])
                self.x_data_ranges[plot_id] = [-np.inf, np.inf]
                self.y_data_ranges[plot_id] = [-np.inf, np.inf]
                self.x_plot_lims[plot_id] = [None, None]
                self.y_plot_lims[plot_id] = [None, None]
            
        
    ###########################################################################
    #CREATE FIGURE
    ###########################################################################       
    def _get_n_plots(self):
        """
        Calculates how many plots have been set by the user.

        Returns
        -------
        n_plots : int
            The number of plots.

        """
        # Get the length of all plot parameters
        lens = np.zeros(20)
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
        lens[18] = len(self.h_zero_line)
        lens[19] = len(self.v_zero_line)
        
        # Make sure each plot has a full set of parameters
        check_val = lens[0]
        for n in lens:
            if n != check_val:
                print("Something went wrong when getting number of plots.")
                return 0
            
        # If all plots have full set of parameters, return the number of plots
        n_plots = len(self.xs)
        return n_plots
    
    
    def _get_plot_shape(self,
                          n_plots):
        """
        Calculates the plot dimensions for the figure given the 
        total number of plots.

        Parameters
        ----------
        n_plots : int
            The number of plots.

        Returns
        -------
        dim : tuple, shape(2)
            The dimensions of the plots.

        """
        # Set the maximum number of rows
        n_rows_max = 2
        
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
                       plot_id):
        """
        Updates the currently stored data ranges and plot limits for 
        both x and y axes for the plot given.

        Parameters
        ----------
        plot_id : int
            The plot's unique identifier.

        Returns
        -------
        None.
        """
        with self.lock:
            # Collect the current relevant information for the x axis
            data_range = self.x_data_ranges[plot_id]
            fixed_lims = self.fixed_x_plot_lims[plot_id]
            data = self.xs[plot_id]
            
            # Calcaulate the new data range and the new plot limits for the x
            data_range, plot_lims = self._get_lims(data=data,
                                                   data_range=data_range,
                                                   fixed_lims=fixed_lims)
            
            # Apply the new data range and new plot limits for the x axis
            self.x_data_ranges[plot_id] = data_range
            self.x_plot_lims[plot_id] = plot_lims
        
            # Collect the current relevant information for the y axis
            data_range = self.y_data_ranges[plot_id]
            fixed_lims = self.fixed_y_plot_lims[plot_id]
            data = self.ys[plot_id]
            
            # Calcaulate the new data range and the new plot limits for the y
            data_range, plot_lims = self._get_lims(data=data,
                                                   data_range=data_range,
                                                   fixed_lims=fixed_lims)
            
            # Apply the new data range and new plot limits for the y axis
            self.y_data_ranges[plot_id] = data_range
            self.y_plot_lims[plot_id] = plot_lims
    
    
    def _create_plot(self,
                     axis,
                     title,
                     x_label,
                     y_label,
                     x_plot_lim,
                     y_plot_lim,
                     h_zero_line,
                     v_zero_line):
        """
        Creates a single plot and sets axis artist setting. Subsequently 
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
        h_zero_line : boolean
            A boolean flag that indicates whether a horizontal line will be
            drawn on the y=0 line.
        v_zero_line : boolean
            A boolean flag that indicates whether a vertical line will be
            drawn on the x=0 line.
            
        Returns
        -------
        None.

        """
        # Clear the axis
        axis.clear()
        
        # Set the labels
        axis.set_title(title, fontsize=FONT_SIZE+1)
        axis.set_xlabel(x_label, fontsize=FONT_SIZE)
        axis.set_ylabel(y_label, fontsize=FONT_SIZE)
        
        # Tickmark size
        axis.tick_params(axis='both', which='major', labelsize=FONT_SIZE)
        axis.tick_params(axis='both', which='minor', labelsize=FONT_SIZE)
        
        # Set the limits if there are any
        if x_plot_lim != [None, None]:
            axis.set_xlim(x_plot_lim[0],
                          x_plot_lim[1])
        if y_plot_lim != [None, None]:
            axis.set_ylim(y_plot_lim[0],
                          y_plot_lim[1])
        
        # Add the zero lines
        if h_zero_line:
            axis.axhline(y=0, xmin=0, xmax=1, alpha=0.75, lw=0.75, c='k')
        if v_zero_line:
            axis.axvline(x=0, ymin=0, ymax=1, alpha=0.75, lw=0.75, c='k')
            
            
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
            The axis artist that defines the plot being updated.
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
            The axis artist that defines the plot being updated.
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
    
    
    def _show(self):
        """
        Renders and shows the current figure.

        Returns
        -------
        None.

        """
        if not self.figure_is_made:
            return
        with self.lock:
            self.fig.canvas.draw()
            buf = self.fig.canvas.buffer_rgba()
            img = np.asarray(buf)
            cv2.imshow(self.window_name, cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
            
    
    def _start(self):
        cv2.namedWindow(self.window_name, cv2.WINDOW_AUTOSIZE) 
        time.sleep(0.1)
        try:
            while True:
                self._step()
                self._show()
                plt.close()
                cv2.waitKey(int(1000.*(1.0/self.fr)))
                with self.lock:
                    if self.done:
                        cv2.destroyAllWindows()
                        break
        except Exception as e:
            print_exception(e, self._start)
            self.done = True
            cv2.destroyAllWindows()
            warn = "While running, the animator crashed. Terminating..."
            warnings.warn(warn, RuntimeWarning)
            sys.stderr.flush()

        
    def start_animator(self):
        """
        Builds and shows the animator GUI. Starts the GUI thread that runs
        until the terminate_animator function is called. Once start_animator
        is called, no more plots can be added.

        Returns
        -------
        None.

        """       
        # If figure is already made, do nothing
        if self.figure_is_made:
            return
        
        # Determine the number of plots and only continue if it's greater than
        # 0.
        self.n_plots = self._get_n_plots()
        if self.n_plots == 0:
            return
        
        # Calculate the plot shape and create the figure
        (n_rows, n_cols) = self._get_plot_shape(self.n_plots)
        fig_res = 300 * n_rows
        fig_dpi = 150
        fig_AR = 1.6*(n_cols/n_rows)
        fig_size = (fig_AR*fig_res/fig_dpi, fig_res/fig_dpi)
        self.fig = plt.figure(figsize=fig_size, dpi=fig_dpi, frameon=True, 
                              facecolor="w")
        self.axes = []
        for i in range(self.n_plots):
                self.axes.append(self.fig.add_subplot(n_rows, n_cols, i+1))

        # Create each plot
        for plot_id in range(self.n_plots):
            # Get the axis on which the plot is drawn
            axis = self.axes[plot_id] 
            
            # Retrieve the label data
            title = self.titles[plot_id]
            x_label = self.x_labels[plot_id]
            y_label = self.y_labels[plot_id]
            plt_type = self.types[plot_id]
            h_zero_line = self.h_zero_line[plot_id]
            v_zero_line = self.v_zero_line[plot_id]
            
            # Retrieve the limit data
            self._update_limits(plot_id)
            x_plot_lim = self.x_plot_lims[plot_id]
            y_plot_lim = self.y_plot_lims[plot_id]
            
            # Create the plot
            self._create_plot(axis=axis,
                              title=title,
                              x_label=x_label,
                              y_label=y_label,
                              x_plot_lim=x_plot_lim,
                              y_plot_lim=y_plot_lim,
                              h_zero_line=h_zero_line,
                              v_zero_line=v_zero_line)
            
            # Create line artists for plot
            if plt_type == 'line':
                for artist_id in range(len(self.artists[plot_id])):
                    # Retrieve the style parameters of the current artist
                    c = self.colors[plot_id][artist_id]
                    lw = self.line_widths[plot_id][artist_id]
                    ls = self.line_styles[plot_id][artist_id]      
                    label = self.labels[plot_id][artist_id]             
                    head_marker = self.tails[plot_id][artist_id] > 0
                    
                    # Make the line artist                                 
                    artist = self._make_line_artist(axis=axis,
                                                    color=c,
                                                    line_width=lw,
                                                    line_style=ls,
                                                    label=label,
                                                    head_marker=head_marker)
                    
                    # Store the made artist
                    self.artists[plot_id][artist_id] = artist
                    
                # Check if a legend should be added, and then add it if needed
                if any([not x is None for x in self.labels[plot_id]]):
                    axis.legend(self.artists[plot_id],
                                self.labels[plot_id],
                                loc="upper right", 
                                fontsize=FONT_SIZE-1)
                    
            # Make bar artists for plot
            elif plt_type == 'bar':
                # Get the artist params
                cs = []
                lws = []
                labels = []
                for artist_id in range(len(self.artists[plot_id])):
                    # Retrieve the style parameters of the current artist
                    cs.append(self.colors[plot_id][artist_id])
                    lws.append(self.line_widths[plot_id][artist_id])
                    labels.append(self.labels[plot_id][artist_id])       
                
                # Make each artist
                artists = self._make_bar_artist(axis=axis,
                                                colors=cs,
                                                line_widths=lws,
                                                labels=labels)
                for artist_id in range(len(artists)):
                    artist = artists[artist_id]
                    self.artists[plot_id][artist_id] = artist
            
        # Set the layout
        self.fig.tight_layout()
        
        # Flag that the figure is done being made
        self.figure_is_made = True
        self.done = False
        
        # Start the GUI thread
        self.thread = Thread(target=self._start)
        self.thread.start()
       
        
    ###########################################################################
    #ADD DATA POINT TO PLOT
    ###########################################################################
    def _trim_data(self,
                   x,
                   y,
                   tail):
        """
        Trims a line of a plot to have length tail.

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
        if tail > 0 and len(x) > tail:
            x = x[-tail:]
            y = y[-tail:]
        return x, y
    
    
    def _add_line_point(self,
                        plot_id,
                        artist_id,
                        x,
                        y):
        """
        Adds a single point to a line type plot.

        Parameters
        ----------
        plot_id : int
            The plot's unique identifier.
        artist_id : int
            The plot's artist index to which the data is added.
        x : float
            The x value of the data point added to the plot.
        y : float
            The y value of the data point added to the plot.

        Returns
        -------
        None.

        """
        with self.lock:
            # Add the data point
            self.xs[plot_id][artist_id].append(x)
            self.ys[plot_id][artist_id].append(y)
            
            # Trim the data
            tail = self.tails[plot_id][artist_id]
            x, y = self._trim_data(x = 
                                   self.xs[plot_id][artist_id],
                                   y = self.ys[plot_id][artist_id],
                                   tail = tail)
            
            # Update the stored data to the trimmed version
            self.xs[plot_id][artist_id] = x
            self.ys[plot_id][artist_id] = y
            
        
    def set_bar_value(self, 
                      plot_id,
                      artist_id,
                      value,):
        """
        Sets the value of one of the bars in a bar plot.

        Parameters
        ----------
        plot_id : int
            The plot's unique identifier.
        artist_id : int
            The plot's artist index whose value is being set.
        value : float
            The value to which the bar is being set.

        Returns
        -------
        None.

        """
        # Add a point to a bar type plot
        if self.types[plot_id] == 'bar':
            with self.lock:
                self.xs[plot_id][artist_id] = [0., value]
            
        # Update the x and y limits of the plot
        self._update_limits(plot_id)
    
        
    def add_line_datum(self,
                       plot_id,
                       x,
                       y,
                       artist_id = 0):
        """
        Adds a single data point to the plot. Data point is appended to the
        end of all previously plotted data points on the specified line.

        Parameters
        ----------
        plot_id : int
            The plot's unique identifier.
        x : float
            The x value of the data point added to the plot.
        y : float
            The y value of the data point added to the plot.
        artist_id : int, optional
            The plot's artist index to which the data is added. If plot
            only has a single artist, is ignored. The default value is 0.
            
        Returns
        -------
        None.

        """
        # Add a data point to a line type plot
        if self.types[plot_id] == 'line':
            self._add_line_point(plot_id=plot_id,
                                 artist_id=artist_id,
                                 x=x,
                                 y=y)
            
        # Update the x and y limits of the plot
        self._update_limits(plot_id)
        

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
        Rerenders a single line plot. Does not update GUI.

        Parameters
        ----------
        x : array-like, shape(n,)
            An array of the new x data.
        y : array-like, shape(n,)
            An array of the new y data.
        artist : matplotlib.lines.Lines2D or None
            The artist that belongs to the axis.
        axis : matplotlib.axes
            The axis artist that defines the plot being updated.
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
        with self.lock:
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
        Rerenders a single bar in a plot. Does not update GUI.

        Parameters
        ----------
        width : float
            The new bar width
        artist : matplotlib.patches.Rectangle
            The bar artist
        axis : matplotlib.axes
            The axis artist that defines the plot being updated.
        x_plot_lim : [float, float]
            The limits to apply to the x axis of the plots. A value of None
            will apply automatically updating limits to that bound of the axis.

        Returns
        -------
        None.
        
        """
        with self.lock:
            # Makes sure we are not trying to plot empty data
            if width == []:
                width = [0.0]
            
            # Change the widths
            artist.set_width(width[-1])
                
            # Reset the x limits
            axis.set_xlim(x_plot_lim[0],
                          x_plot_lim[1])
        
        
    def _step(self):
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
        # Create the figure if it is not already made
        if not self.figure_is_made:
            self.start_animator()
        
        # Update each plot
        for plot in range(self.n_plots):
            # Get the axis on which the plot is drawn
            plot_axis = self.axes[plot]
    
            # Retrieve the plot data
            plot_xs = self.xs[plot]
            plot_ys = self.ys[plot]
            plot_type = self.types[plot]
            
            # Retrieve the limit data
            plot_x_lim = self.x_plot_lims[plot]
            plot_y_lim = self.y_plot_lims[plot]
            
            # Retrieve the line artist for each axis
            plot_artists = self.artists[plot]
            
            # Determine if head markers are used for the line plots
            head_marker = (self.tails[plot] != None)

            # Go through each artist one at a time
            for artist_id in range(len(plot_xs)):
                # Get the x and y data of the artist
                x = plot_xs[artist_id]
                y = plot_ys[artist_id]
                
                # Get the artist
                artist = plot_artists[artist_id]
                
                # Update the plotted line data
                if plot_type == 'line':
                    self._step_line(x=x,
                                    y=y,
                                    artist=artist,
                                    axis=plot_axis,
                                    x_plot_lim=plot_x_lim,
                                    y_plot_lim=plot_y_lim,
                                    head_marker=head_marker)
                    
                # Update the plotted bar data
                elif plot_type == 'bar':
                    self._step_bar(width=x,
                                   artist=artist,
                                   axis=plot_axis,
                                   x_plot_lim=plot_x_lim)
                    
    def terminate_animator(self):
        """
        Terminates the animator window.
        
        Returns
        -------
        None.

        """
        if self.figure_is_made and not self.done:
            with self.lock:
                self.done = True
            self.thread.join()