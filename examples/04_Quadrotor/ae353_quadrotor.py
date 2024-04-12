"""
This modules provides a backend for the ae353 quadrotor example
"""

###############################################################################
#DEPENDENCIES
###############################################################################
from condynsate.simulator import Simulator
from pathlib import Path
import numpy as np
import os


###############################################################################
#POSITION IN COURSE FUNCTIONS AND CLASS
###############################################################################
def get_gate_cen(gate, sim, scale):
    # Get the stl that was used to build the gate
    stl_pth = sim._get_urdf_vis_dat(gate)[0][0]
    stl_name = os.path.basename(stl_pth)
    
    # Extract gate height data based on gate size
    gate_height = 0.0
    if stl_name == 'gate_short.stl':
        gate_height=0.5
    elif stl_name == 'gate_med.stl':
        gate_height=1.5
    elif stl_name == 'gate_tall.stl':
        gate_height=2.0
        
    # Get the position of the base of the gate
    state = sim.get_base_state(urdf_obj=gate,
                               body_coords=False)
    pos = state['position']
    
    # Get the center of the gate
    cen = np.array([pos[0], pos[1], gate_height*scale])
    return cen
    
    
def get_gate_dirn(gate, sim):
    # Extract the state data
    state = sim.get_base_state(urdf_obj=gate,
                               body_coords=False)
    
    # Apply the yaw rotation to the base pointing direction
    yaw = state['yaw']
    c = np.cos(yaw)
    s = np.sin(yaw)
    Rz = np.array([[c, -s, 0.],
                   [s,  c, 0.],
                   [0., 0., 1.]])
    base = np.array([0., 1., 0.])
    dirn = Rz @ base
    
    # Return the calculated direction
    return dirn

    
class Course_Tracker():
    def __init__(self, quadrotor, team_name, sim, gate_data):
        # Track relevant data
        self.quadrotor = quadrotor
        self.team_name = team_name
        self.sim  = sim
        self.gate_data = gate_data
        
        # Get the state of the quadrotor in world coords
        world_state = self.sim.get_base_state(urdf_obj=self.quadrotor,
                                              body_coords=False)
        position = world_state['position']
                
        # Set the current gate and side
        self.num_laps = 0
        self.curr_gate = 0
        self.start_side = self.get_gate_side(position)
        
        # Get the time tracking variables
        self.lap_start_time = None
        self.lap_finish_time = None
        self.gate_times = []
        
        
    def reset(self):
        # Get the state of the quadrotor in world coords
        world_state = self.sim.get_base_state(urdf_obj=self.quadrotor,
                                              body_coords=False)
        position = world_state['position']
        
        # Reset current gate and current gate side
        self.num_laps = 0
        self.curr_gate = 0
        self.start_side  = self.get_gate_side(position)

        # Get the time tracking variables
        self.lap_start_time = None
        self.lap_finish_time = None
        self.gate_times = []
        

    def near_gate(self, position):
        # Extract gate data
        gate_cen = self.gate_data['cens'][self.curr_gate]
        gate_radius = self.gate_data['radii'][self.curr_gate]
        
        # Get the distance from the quadrotor to the gate center
        delta_to_gate = gate_cen - position
        dist = np.linalg.norm(delta_to_gate)
        
        # Return if the distance from the quadrotor to gate center is less than
        # the radius of the gate
        return dist < gate_radius
    
    
    def get_gate_side(self, position):
        # Extract gate data
        gate_cen = self.gate_data['cens'][self.curr_gate]
        gate_dirn = self.gate_data['dirns'][self.curr_gate]
        
        # Get the dot product of the gate direction and direction from
        # quadrotor to gate center
        delta_to_gate = gate_cen - position
        dotp = np.dot(delta_to_gate, gate_dirn)
        
        # Get the side of the gate the quadrotor is on. 1 is before the gate,
        # -1 is after the gate.
        side = int(dotp / abs(dotp))
        return side
    
    
    def passed_gate(self, position):
        # Determine the current side of the gate and whether or not the
        # quadrotor is near the gate
        in_gate_radius = self.near_gate(position)
        curr_side = self.get_gate_side(position)
        
        # If we are near the gate and we change side of the gate, we passed 
        # through it
        if curr_side != self.start_side and in_gate_radius:
            passed = True
        else:
            passed = False
            
        # Return results
        return passed
    
    
    def step(self, position, verbose):
        # Set the lap start time
        if self.lap_start_time == None:
            self.lap_start_time = self.sim.time
            
        # If the current gate is passed...
        if self.passed_gate(position):
            # Get elapsed time
            elapsed = self.sim.time - self.lap_start_time
            elapsed = np.round(elapsed, 3)
            self.gate_times.append(elapsed)
            
            # Print to screen...
            quad_num = self.quadrotor.urdf_id
            if verbose:
                print("{}:{} PASSED GATE {} IN {}s".format(self.team_name,
                                                             quad_num,
                                                             self.curr_gate+1,
                                                             elapsed))
            
            # update the current gate and lap number...
            self.curr_gate = self.curr_gate + 1
            if self.curr_gate >= len(self.gate_data['cens']):
                self.curr_gate = 0
                self.num_laps = self.num_laps + 1
                self.lap_finish_time = elapsed
                if verbose:
                    print("{}:{} COMPLETED LAP IN {}s".format(self.team_name,
                                                                quad_num,
                                                                elapsed))
                self.lap_start_time = self.sim.time
                
            # and update the side on which we start.
            self.start_side = self.get_gate_side(position)
            
        # Return the current gate's position direction and if it's the last one
        cen = self.gate_data['cens'][self.curr_gate]
        dirn = self.gate_data['dirns'][self.curr_gate]
        is_last = (self.curr_gate == len(self.gate_data['cens']) - 1)
        return cen, dirn, is_last, self.num_laps
            

###############################################################################
#OTHER PUBLIC FUNCTIONS
###############################################################################
def get_gravity(gravity_str, verbose):
        gravities = {"mercury" : 3.708,
                     "venus" : 8.897,
                     "earth" : 9.81,
                     "moon" : 1.628,
                     "mars" : 3.698,
                     "jupiter" : 23.151,
                     "saturn" : 8.985,
                     "uranus" : 8.721,
                     "neptune" : 10.987,
                     "pluto" : 0.696}
        lower_gravity_str = gravity_str.lower()
        if lower_gravity_str in gravities:
            gravity = gravities[lower_gravity_str]
        else:
            if verbose:
                string = "{} not recognized setting. New gravity is {}."
                print(string.format(gravity_str, 9.81))
            gravity = 9.81
        return gravity


###############################################################################
#SIMULATION CLASS
###############################################################################
class Quadrotor_Sim():
    def __init__(self,
                 team_list,
                 n_quadrotors,
                 planet="Earth",
                 use_keyboard=True,
                 visualization=True,
                 visualization_fr=45.,
                 animation=True,
                 animation_fr=10.,
                 verbose=False):
        """
        Initializes an instance of the simulation class.

        Parameters
        ----------
        n_quadrotors : int
            The number of quadrotor controllers provided to the simulator.
        team_list : list of strings, shape(n_quadrotors,)
            A list of the team names associated with each of the n_quadrotors
            controllers. The teams names to select from include: "the ORB",
            "Team Kachow", "Team Steam Tunnels", "Team Flying Mambas",
            "Team Spare Chang-e", and "the Flying Illini". The string ""
            results in an empty team flag.
        use_keyboard : bool, optional
            A boolean flag that indicates whether the simulation will allow
            the use of keyboard interactivity. The default is True.
        visualization : bool, optional
            A boolean flag that indicates whether the simulation will be 
            visualized in meshcat. The default is True.
        visualization_fr : float, optional
            The frame rate (frames per second) at which the visualizer is
            updated. The default is 45..
        animation : bool, optional
            A boolean flag that indicates whether animated plots are created
            in real time. The default is True.
        animation_fr : float, optional
            The frame rate (frames per second) at which the animated plots are
            updated. The default is 10..
        verbose : Bool, optional
            A boolean flag that indicates whether warnings are printing during
            function execution. The default is False.
            
        Returns
        -------
        None.

        """
        # Check all inputs
        if not isinstance(use_keyboard, bool):
            if verbose:
                string = "use_keyboard must be type {}."
                print(string.format(bool))
            return None
        if not isinstance(visualization, bool):
            if verbose:
                string = "visualization must be type {}."
                print(string.format(bool))
            return None
        if not isinstance(animation, bool):
            if verbose:
                string = "animation must be type {}."
                print(string.format(bool))
            return None
        if not isinstance(visualization_fr, float):
            if verbose:
                string = "visualization_fr must be type {}."
                print(string.format(float))
            return None
        if not isinstance(animation_fr, float):
            if verbose:
                string = "animation_fr must be type {}."
                print(string.format(float))
            return None
        if not isinstance(n_quadrotors, int):
            if verbose:
                string = "n_quadrotors must be type {}."
                print(string.format(int))
            return None
        if not isinstance(team_list, list):
            if verbose:
                string = "team_list must be type {}."
                print(string.format(list))
            return None
        if n_quadrotors > 11:
            if verbose:
                string = "n_quadrotors cannot be greater than 11."
                print(string)
            return None
        if not len(team_list) == n_quadrotors:
            if verbose:
                string = "team_list must be length {}."
                print(string.format(n_quadrotors))
            return None
        valid_team_names = ["the ORB", "Team Kachow", "Team Steam Tunnels", 
                            "Team Flying Mambas", "Team Spare Chang-e",
                            "the Flying Illini", ""]
        for team_name in team_list:
            if team_name not in valid_team_names:
                if verbose:
                    string = "{} is not a valid team name."
                    print(string.format(team_name))
                return None
        
        # Set the visualization and animation options
        self.visualization = visualization
        self.animation = animation

        # Get the gravity value
        grav = get_gravity(planet, verbose)

        # Initialize and instance of the simulator
        self.sim = Simulator(keyboard=use_keyboard,
                             visualization=visualization,
                             visualization_fr=visualization_fr,
                             animation=animation,
                             animation_fr=animation_fr,
                             gravity=[0., 0., -grav])
        
        # Get the path to the current directory
        path = (Path(__file__).parents[0]).absolute().as_posix()
        
        # Get the initial positions of the quadrotors
        self.n_quadrotors = n_quadrotors
        delta = 2.0
        n_sides = int(np.ceil(np.sqrt(float(self.n_quadrotors))))
        sides = np.linspace(0.0, (n_sides-1)*delta, n_sides)
        y, x = np.meshgrid(sides, sides)
        x = x.flatten()
        y = y.flatten()
        x = x - np.mean(x)
        y = y - np.mean(y)
        positions = []
        for i in range(self.n_quadrotors):
            position = [x[i], y[i], 0.0]
            positions.append(position)
        
        # Load the quadrotor(s)
        self.quad_objs = []
        self.team_list = team_list
        for i in range(self.n_quadrotors):
            if self.team_list[i] == "":
                urdf_path = path + "/quadcopter_vis/flagless_quadcopter.urdf"
                quad_obj = self.sim.load_urdf(urdf_path=urdf_path,
                                              position=positions[i],
                                              fixed=False,
                                              update_vis=True)
            else:
                urdf_path = path + "/quadcopter_vis/quadcopter.urdf"
                team_path = path+"/quadcopter_vis/"+self.team_list[i]+".jpg"
                quad_obj = self.sim.load_urdf(urdf_path=urdf_path,
                                              tex_path=team_path,
                                              position=positions[i],
                                              fixed=False,
                                              update_vis=True)
            self.quad_objs.append(quad_obj)
            
        # Load the ground
        urdf_path = path + "/quadcopter_vis/plane.urdf"
        tex_path = path + "/quadcopter_vis/carpet.png"
        _ = self.sim.load_urdf(urdf_path=urdf_path,
                               tex_path=tex_path,
                               position=[0., 0., -0.051],
                               fixed=True,
                               update_vis=False)

        # Define the course
        gate_scale = 5.
        pos_scale = 0.6
        p1 = pos_scale*np.array([20., 0., 0.])
        p2 = pos_scale*np.array([35., 13., 0.])
        p3 = pos_scale*np.array([40., 30., 0.])
        p4 = pos_scale*np.array([20., 45., 0.])
        p5 = pos_scale*np.array([0., 30., 0.])
        p6 = pos_scale*np.array([0., -16., 0.])
        p7 = pos_scale*np.array([-6., -22., 0.])
        y1 = -np.pi/2.
        y2 = -np.pi/6.
        y3 = 0.
        y4 = np.pi/2.
        y5 = np.pi
        y6 = np.pi
        y7 = np.pi/2.

        # Load the course
        urdf_path = path + "/quadcopter_vis/gate_short.urdf"
        gate_1 = self.sim.load_urdf(urdf_path=urdf_path,
                                    position=p1,
                                    yaw=y1,
                                    fixed=True,
                                    update_vis=False)
        urdf_path = path + "/quadcopter_vis/gate_short.urdf"
        gate_2 = self.sim.load_urdf(urdf_path=urdf_path,
                                    position=p2,
                                    yaw=y2,
                                    fixed=True,
                                    update_vis=False)
        urdf_path = path + "/quadcopter_vis/gate_med.urdf"
        gate_3 = self.sim.load_urdf(urdf_path=urdf_path,
                                    position=p3,
                                    yaw=y3,
                                    fixed=True,
                                    update_vis=False)
        urdf_path = path + "/quadcopter_vis/gate_tall.urdf"
        gate_4 = self.sim.load_urdf(urdf_path=urdf_path,
                                    position=p4,
                                    yaw=y4,
                                    fixed=True,
                                    update_vis=False)
        urdf_path = path + "/quadcopter_vis/gate_tall.urdf"
        gate_5 = self.sim.load_urdf(urdf_path=urdf_path,
                                    position=p5,
                                    yaw=y5,
                                    fixed=True,
                                    update_vis=False)
        urdf_path = path + "/quadcopter_vis/gate_tall.urdf"
        gate_6 = self.sim.load_urdf(urdf_path=urdf_path,
                                    position=p6,
                                    yaw=y6,
                                    fixed=True,
                                    update_vis=False)
        urdf_path = path + "/quadcopter_vis/gate_med.urdf"
        gate_7 = self.sim.load_urdf(urdf_path=urdf_path,
                                    position=p7,
                                    yaw=y7,
                                    fixed=True,
                                    update_vis=False)
        
        # Compile the gate data
        gates = [gate_1, gate_2, gate_3, gate_4, gate_5, gate_6, gate_7]
        self.gate_data = {"gates" : gates,
                          "cens" : [],
                          "dirns" : [],
                          "radii" : []}
        for gate in self.gate_data['gates']:
            cen = get_gate_cen(gate, self.sim, gate_scale)
            dirn = get_gate_dirn(gate, self.sim)
            self.gate_data['cens'].append(cen)
            self.gate_data['dirns'].append(dirn)
            self.gate_data['radii'].append(0.5*gate_scale)
        
        # Make a course tracker(s)
        self.trackers = []
        for quad_obj, team_name in zip(self.quad_objs, self.team_list):
            tracker = Course_Tracker(quad_obj,
                                     team_name,
                                     self.sim,
                                     self.gate_data)
            self.trackers.append(tracker)
        
        # Only set the background lighting if the visualizer is being used
        if self.visualization:
            self.sim.set_background(top_color=[43, 50, 140],
                                    bot_color=[125, 149, 219])
            self.sim.set_posx_pt_light(on=True,
                                       intensity=0.5,
                                       distance=10.)
            self.sim.set_negx_pt_light(on=True,
                                       intensity=0.5,
                                       distance=10.)
            self.sim.set_ambient_light(on=True,
                                       intensity=0.65)
        
        # If there is animation, add subplots
        if self.animation:
            # Get the colors, labels, etc. for each quadrotor
            possible_colors = ['k', 'r', 'g', 'b', 'm', 'c', 'tab:orange',
                               'tab:purple', 'tab:brown', 'tab:pink',
                               'tab:gray']
            colors = [possible_colors[i%11] for i in range(self.n_quadrotors)]
            labels = [i for i in range(self.n_quadrotors)]
            line_widths = [1.0 for i in range(self.n_quadrotors)]
            line_styles = ["-" for i in range(self.n_quadrotors)]
            
            # Make plot for state and input
            self.p1, self.a1 = self.sim.add_subplot(
                                    n_artists=self.n_quadrotors,
                                    subplot_type='line',
                                    title="Ground Track",
                                    x_label="x position [m]",
                                    y_label="y position [m]",
                                    colors=colors,
                                    labels=labels,
                                    line_widths=line_widths,
                                    line_styles=line_styles,
                                    h_zero_line=True,
                                    v_zero_line=True)
            
            # Open the animator GUI
            self.sim.open_animator_gui()


    def run(self,
            controllers,
            sensor_noise=0.01,
            collect_data = True,
            max_time = None,
            verbose=False):
        """
        Runs a complete simulation

        Parameters
        ----------
        controllers : list of Controller class
            A list of length n_quadrotors where each element is a member of 
            a custom class that, at a minimum, must provide the functions
            controller.run() and controller.reset()
        sensor_noise : float, optional
            The noise applied to the mocap sensor, i.e., the standard deviation
            of the components of each mocap marker tracker measurement.
            The default value is 0.01.
        collect_data : bool, optional
            A boolean flag that indicates whether or not simulation data is
            collected
        max_time : Float, optional
            The total amount of time the simulation is allowed to run. When
            set to None, the simulator will run until the terminal command is 
            called. If the keyboard is disabled, users are not allowed to 
            set max time as None. The default value is None. 
        verbose : Bool, optional
            A boolean flag that indicates whether warnings are printing during
            function execution. The default is False.

        Returns
        -------
        data : Dictionary of Lists
            data["position"] : List of arrays, shape(n,3)
                The [x, y, z] position of the quadrotor at each time stamp.
            data["orientation"] : List of arrays, shape(n,3)
                The [roll, pitch, yaw] orientation of the quadrotor at each
                time stamp.
            data["velocity"] : List of arrays, shape(n,3)
                The [vx, vy, vz] velocity of the quadrotor in body fixed 
                coordinates at each time stamp.
            data["angular velocity"] : List of arrays, shape(n,3)
                The [wx, wy, wz] angular velocity of the quadrotor in body 
                fixed coordinates at each time stamp.
            data["mocap 1"] : List of arrays, shape(n,3)
                The true (not noisy) position of the mocap 1 marker (+x) in 
                meters at each time stamp.
            data["mocap 2"] : List of arrays, shape(n,3)
                The true (not noisy) position of the mocap 2 marker (+y) in 
                meters at each time stamp.
            data["mocap 3"] : List of arrays, shape(n,3)
                The true (not noisy) position of the mocap 3 marker (-x) in 
                meters at each time stamp.
            data["mocap 4"] : List of arrays, shape(n,3)
                The true (not noisy) position of the mocap 4 marker (-y) in 
                meters at each time stamp.
            data["inputs"] : list of arrays, shape(n,4):
                The inputs applied to the quadrotor. [tau_x, tau_y, tau_z, f_z]
            data["time"] : List of Floats, shape(n,)
                A list of the time stamps in seconds.
        track_stats : List of Dictionaries
            ...TODO

        """
        # Check all inputs
        if not isinstance(controllers, list):
            if verbose:
                string = "controllers must be type {}."
                print(string.format(list))
            return None
        if not len(controllers) == self.n_quadrotors:
            if verbose:
                string = "controllers must be length {}."
                print(string.format(self.n_quadrotors))
            return None
        if not isinstance(sensor_noise, float):
            if verbose:
                string = "sensor_noise must be type {}."
                print(string.format(float))
            return None
        if (not isinstance(max_time, float)) and (not max_time == None):
            if verbose:    
                string = "max_time must be type {} or {}."
                print(string.format(None, float))
            return None
        
        # Reset the simulator and tracker(s)
        self.sim.reset()
        for tracker in self.trackers:
            tracker.reset()

        # Reset the controller(s)
        self.controllers = controllers
        for controller in self.controllers:
            controller.reset()
        
        # Create a dictionary to hold the simulation data
        self.collect_data = collect_data
        if self.collect_data:
            self.data = []
            for i in range(self.n_quadrotors):
                datum = {"position" : [],
                         "orientation" : [],
                         "velocity" : [],
                         "angular velocity" : [],
                         "mocap 1" : [],
                         "mocap 2" : [],
                         "mocap 3" : [],
                         "mocap 4" : [],
                         "inputs" : [],
                         "time" : []}
                self.data.append(datum)

        # Await run command if visualization or animation is on
        if self.visualization or self.animation:
            self.sim.await_keypress(key='enter')

        # Run the simulation loop
        self.race_done = False
        while (not self.sim.is_done) and (not self.race_done):
            # Get the state of each quadrotor before looping through all other
            # quadrotors and controller. This allows us to send the position
            # of the other quadrotors to each controller.
            positions = []
            orientations = []
            velocities = []
            ang_velocities = []
            for i in range(self.n_quadrotors):
                quad_obj = self.quad_objs[i]
                
                # Get the state of the quadrotor in world coords
                world_state = self.sim.get_base_state(urdf_obj=quad_obj,
                                                      body_coords=False)
                position = world_state['position']
                orientation = np.array([world_state['roll'],
                                        world_state['pitch'],
                                        world_state['yaw']])
                
                # Get the state of the quadrotor in body coords
                body_state = self.sim.get_base_state(urdf_obj=quad_obj,
                                                     body_coords=True)
                vel = body_state['velocity']
                ang_vel = body_state['angular velocity']
                
                # Store all state data
                positions.append(position)
                orientations.append(orientation)
                velocities.append(vel)
                ang_velocities.append(ang_vel)
            
            # Go through each quadrotor
            for i in range(self.n_quadrotors):          
                ###############################################################
                # SENSORS
                # Get the state of the quadrotor(s) in world coords
                quad_obj = self.quad_objs[i]
                position = positions[i]
                orientation = orientations[i]
                vel = velocities[i]
                ang_vel = ang_velocities[i]
                
                # Calculate the position of the mocap markers
                mocap_1 = position + np.array([0.25, 0., 0.046875])
                mocap_2 = position + np.array([0., 0.25, 0.046875])
                mocap_3 = position + np.array([-0.25, 0., 0.046875])
                mocap_4 = position + np.array([0., -0.25, 0.046875])
                
                # Apply noise to mocap markers
                mocap_1_noisy = np.random.normal(loc=mocap_1,
                                                 scale=sensor_noise)
                mocap_2_noisy = np.random.normal(loc=mocap_2, 
                                                 scale=sensor_noise)
                mocap_3_noisy = np.random.normal(loc=mocap_3, 
                                                 scale=sensor_noise)
                mocap_4_noisy = np.random.normal(loc=mocap_4, 
                                                 scale=sensor_noise)

                ###############################################################
                # COURSE DETECTION
                # Determine where the quadrotor(s) is(are)
                tracker = self.trackers[i]
                cen, dirn, is_last, n_laps = tracker.step(position, verbose)
                if n_laps > 0:
                    self.race_done = True
                
                ###############################################################
                # CONTROLLER
                # Get the inputs as calculated by the controller(s)
                # Extract all the data for each controller
                controller = self.controllers[i]

                # Get the positions of the other quadrotors
                pos_others = []
                for j in range(self.n_quadrotors):
                    if j != i:
                        # check for nan
                        if (positions[j] == positions[j]).all():  
                            pos_others.append(positions[j])
                
                # Get the inputs for controller i
                inputs = controller.run(dt = self.sim.dt,
                                        mocap_1 = mocap_1_noisy,
                                        mocap_2 = mocap_2_noisy,
                                        mocap_3 = mocap_3_noisy,
                                        mocap_4 = mocap_4_noisy,
                                        next_gate = cen,
                                        dir_gate = dirn,
                                        is_last_gate = is_last,
                                        pos_others = pos_others)
            
                # Extract the inputs
                tau_x = inputs[0]
                tau_y = inputs[1]
                tau_z = inputs[2]
                f_z = inputs[3]
                
                # Limit the inputs
                tau_x = np.clip(tau_x, -0.03, 0.03)
                tau_y = np.clip(tau_y, -0.03, 0.03)
                tau_z = np.clip(tau_z, -0.03, 0.03)
                f_z = np.clip(f_z, 0.0, 14.72)
                
                # Make sure valid
                vx=isinstance(tau_x,(int,float)) and not isinstance(tau_x,bool)
                vy=isinstance(tau_y,(int,float)) and not isinstance(tau_y,bool)
                vz=isinstance(tau_z,(int,float)) and not isinstance(tau_z,bool)
                vfz=isinstance(f_z,(int,float)) and not isinstance(f_z,bool)
                if not vx or (tau_x!=tau_x):
                    tau_x = 0.0
                    if verbose:
                        string = "tau_x must be type {} or {}."
                        print(string.format(int, float))
                if not vy or (tau_y!=tau_y):
                    tau_y = 0.0
                    if verbose:
                        string = "tau_y must be type {} or {}."
                        print(string.format(int, float))
                if not vz or (tau_z!=tau_z):
                    tau_z = 0.0
                    if verbose:
                        string = "tau_z must be type {} or {}."
                        print(string.format(int, float))
                if not vfz or (f_z!=f_z):
                    f_z = 0.0
                    if verbose:
                        string = "f_z must be type {} or {}."
                        print(string.format(int, float))
            
                ###############################################################
                # ACTUATOR
                # Apply the inputs
                show_arrow = self.n_quadrotors==1 and self.visualization
                self.sim.apply_force_to_com(urdf_obj=quad_obj,
                                            force=[0,0,f_z],
                                            body_coords=True,
                                            show_arrow=show_arrow,
                                            arrow_scale=0.05,
                                            arrow_offset=0.0)
                self.sim.apply_external_torque(urdf_obj=quad_obj,
                                               torque=[tau_x, tau_y, tau_z],
                                               body_coords=True,
                                               show_arrow=show_arrow,
                                               arrow_scale=10.0,
                                               arrow_offset=0.)
            
                ###############################################################
                # SIMULATION DATA
                # Append data to history lists
                if self.collect_data:
                    self.data[i]["position"].append(position)
                    self.data[i]["orientation"].append(orientation)
                    self.data[i]["velocity"].append(vel)
                    self.data[i]["angular velocity"].append(ang_vel)
                    self.data[i]["mocap 1"].append(mocap_1)
                    self.data[i]["mocap 2"].append(mocap_2)
                    self.data[i]["mocap 3"].append(mocap_3)
                    self.data[i]["mocap 4"].append(mocap_4)
                    self.data[i]["inputs"].append([tau_x, tau_y, tau_z, f_z])
                    self.data[i]["time"].append(self.sim.time)
            
                ###############################################################
                # UPDATE THE PLOTS
                # This is how we add data points to the animator
                if self.animation:
                    self.sim.add_subplot_point(subplot_index=self.p1,
                                               artist_index=self.a1[i],
                                               x=position[0],
                                               y=position[1])
            
            ###################################################################
            # STEP THE SIMULATION
            # Step the sim
            val = self.sim.step(real_time=self.visualization or self.animation,
                                update_vis=self.visualization,
                                update_ani=self.animation,
                                max_time=max_time)
            
            # Handle resetting the controller and simulation data
            if val == 3:
                
                # Reset the controller
                for controller in self.controllers:
                    controller.reset()
                
                # Reset gate tracking
                for tracker in self.trackers:
                    tracker.reset()
                        
                # Reset the race
                self.race_done = False
                
                # Reset the history
                if self.collect_data:
                    self.data = []
                    for i in range(self.n_quadrotors):
                        datum = {"position" : [],
                                 "orientation" : [],
                                 "velocity" : [],
                                 "angular velocity" : [],
                                 "mocap 1" : [],
                                 "mocap 2" : [],
                                 "mocap 3" : [],
                                 "mocap 4" : [],
                                 "inputs" : [],
                                 "time" : []}
                        self.data.append(datum)
       
        # When the simulation is done running, return the data
        if self.collect_data:
            # Get the track status of all the quadrotors
            track_stats = []
            for tracker in self.trackers:
                track_stats.append({"completed_lap" : (tracker.num_laps>0),
                                    "gate_times" : tracker.gate_times,
                                    "lap_time" : tracker.lap_finish_time})
            
            # Return the tack statuses and the simulation data
            return (track_stats, self.data)
        else:
            return (None, None)
