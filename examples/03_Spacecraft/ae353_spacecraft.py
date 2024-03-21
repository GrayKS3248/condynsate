"""
This modules provides a backend for the ae353 spacecraft example
"""

###############################################################################
#DEPENDENCIES
###############################################################################
from condynsate.simulator import Simulator
from pathlib import Path
import numpy as np


###############################################################################
#GLOBAL FUNCTIONS
###############################################################################
def get_star_coords():
    """
    Return the equitorial celestial cooridinates of the constellation.

    Returns
    -------
    star_coords : dictionary of lists
        A dictionary whose keys are the right ascension, "alpha", and 
        declination, "delta" of each star in the constellation.

    """
    eqi_coords = np.array([[-0.06, -0.09],
                           [ 0.00, -0.09],
                           [ 0.06, -0.09],
                           [ 0.00, -0.03],
                           [ 0.00,  0.03],
                           [-0.06,  0.09],
                           [ 0.00,  0.09],
                           [ 0.06,  0.09]])
    star_coords = {'alpha' : eqi_coords[:,0],
                   'delta' : eqi_coords[:,1]}
    return star_coords


###############################################################################
#SIMULATION CLASS
###############################################################################
class Spacecraft_Sim():
    def __init__(self,
                 use_keyboard=True,
                 visualization=True,
                 visualization_fr=20.,
                 animation=True,
                 animation_fr=10.,
                 n_stars = 20):
        """
        Initializes an instance of the simulation class.

        Parameters
        ----------
        use_keyboard : bool, optional
            A boolean flag that indicates whether the simulation will allow
            the use of keyboard interactivity. The default is True.
        visualization : bool, optional
            A boolean flag that indicates whether the simulation will be 
            visualized in meshcat. The default is True.
        visualization_fr : float, optional
            The frame rate (frames per second) at which the visualizer is
            updated. The default is 20..
        animation : bool, optional
            A boolean flag that indicates whether animated plots are created
            in real time. The default is True.
        animation_fr : float, optional
            The frame rate (frames per second) at which the animated plots are
            updated. The default is 10..
        n_stars : int, optional
            The number of random stars to add to the background. Purely
            visual. Has no impact on the simulation. The default is 20.
        Returns
        -------
        None.

        """
        # Set the visualization and animation options
        self.visualization = visualization
        self.animation = animation

        # Initialize and instance of the simulator
        self.sim = Simulator(keyboard=use_keyboard,
                             visualization=visualization,
                             visualization_fr=visualization_fr,
                             animation=animation,
                             animation_fr=animation_fr,
                             gravity=[0., 0., 0.])
        
        # Get the path to the current directory
        path = (Path(__file__).parents[0]).absolute().as_posix()
        
        # Load the spacecraft
        craft_path = path + "/spacecraft_vis/spacecraft.urdf"
        self.craft_obj = self.sim.load_urdf(urdf_path=craft_path,
                                            fixed=False,
                                            update_vis=True)
        
        # Generate star cartesian coordinates based on their depth and
        # equatorial astronomical coords (right ascension, declination)
        star_depth = int(np.round((n_stars/6.) + (110./3.)))
        if star_depth < 40.:
            star_depth = 40.
        if star_depth > 70.:
            star_depth = 70.
        star_coords = get_star_coords()
        self.P_star_inW = []
        for alpha, delta in zip(star_coords['alpha'], star_coords['delta']):
            s_a = np.sin(alpha)
            c_a = np.cos(alpha)
            s_d = np.sin(delta)
            c_d = np.cos(delta)
            cart_coord = np.array([c_a*c_d, s_a*c_d, s_d]) * star_depth
            self.P_star_inW.append(cart_coord)
        # Load the constellation
        constellation_path = path + "/spacecraft_vis/star.urdf"
        debris_path = path + "/spacecraft_vis/shootingstar.urdf"
        for pos in self.P_star_inW:
            _ = self.sim.load_urdf(urdf_path=constellation_path,
                                   position=pos,
                                   fixed=True,
                                   update_vis=False)
        
        # Load random stars
        self.n_debris = 15
        n = n_stars + self.n_debris
        rand_depths = star_depth + (np.random.rand(n)*0.2-0.1)*star_depth
        positions = np.random.randn(n,3)
        positions = positions / np.linalg.norm(positions,axis=1).reshape(n,1)
        positions = positions * rand_depths.reshape(n,1)
        self.debris_objs = []
        for i in range(len(positions)):
            # Get the current random position
            pos = positions[i]
            
            # Load self.n_debris debris objects
            if i < self.n_debris:
                debris_obj = self.sim.load_urdf(urdf_path=debris_path,
                                                position=pos,
                                                fixed=False,
                                                update_vis=True)
                self.debris_objs.append(debris_obj)  
                
            # Load the rest of the randomly placed stars
            else:
                _ = self.sim.load_urdf(urdf_path=constellation_path,
                                       position=pos,
                                       fixed=True,
                                       update_vis=False)
        
        # Calulate the frequency to toss debris
        frequency = 30.0 / self.n_debris
        self.debris_prob = 1.0 / (100.*frequency)
        
        # Only set the background lighting if the visualizer is being used
        if self.visualization:
            self.sim.set_background(top_color=[10,10,10],
                                    bot_color=[20, 20, 40])
            self.sim.set_spotlight(on=False)
            self.sim.set_fill_light(on=False)
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
            # Make plot for state and input
            self.p1, self.a1 = self.sim.add_subplot(
                                    n_artists=3,
                                    subplot_type='line',
                                    title="Euler Angles vs. Time",
                                    x_label="Time [s]",
                                    y_label="Angle [Rad]",
                                    colors=['g', 'r', 'b'],
                                    labels=['Roll', 'Pitch', 'Yaw'],
                                    line_widths=[2.5, 2.5, 2.5],
                                    line_styles=["-", "-", "-"],
                                    h_zero_line=True)
            
            # Open the animator GUI
            self.sim.open_animator_gui()


    def run(self,
            controller,
            initial_orientation = [0., 0., 0.],
            initial_ang_vel = [0., 0., 0.],
            sensor_noise = 0.1,
            debris = False,
            max_time = None):
        """
        Runs a complete simulation

        Parameters
        ----------
        controller : class
            A custom class that, at a minimum, must provide the functions
            controller.run() and controller.reset()
        initial_orientation : Array, shape(3,), optional
            The initial orientation to apply to the spacecraft. The initial
            orientation is applied that the beginning of each simulation and
            whenever the simulation is reset. The three values in the
            initial_orientation array are roll, pitch, and yaw. Each value is
            in radians. Roll is defined as the rotation of the craft about the
            x-axis of the world coordinate system. Pitch is defined as the
            rotation of the craft about the y-axis of the world coordinate
            system. Yaw is defined as the rotation of the craft about the
            z-axis of the world coordinate system. The default value is
            [0., 0., 0.].
        initial_ang_vel : Array, shape(3,), optional
            The initial angular velocity to apply to the spacecraft. The
            initial angular velocity is applied that the beginning of each
            simulation and whenever the simulation is reset. The three values
            in the initial_ang_vel array are wx, wy, and wz. Each value is in
            radians per second. wx is defined as the rotation rate of the craft
            about the body-fixed x-axis. wy is defined as the rotation rate of
            the craft about the body-fixed y-axis. wz is defined as the
            rotation rate of the craft about the body-fixed z-axis. The default
            value is [0., 0., 0.].
        sensor_noise : Float, optional
            The noise applied to the star tracker, i.e., the standard deviation
            of the and components of each star tracker measurement. The default
            value is 0.1.
        debris : bool, optional
            A boolean flag that indicates whether or not random space debris
            will be spawned and strike the spacecraft.
        max_time : Float, optional
            The total amount of time the simulation is allowed to run. When
            set to None, the simulator will run until the terminal command is 
            called. If the keyboard is disabled, users are not allowed to 
            set max time as None. The default value is None. 

        Returns
        -------
        data : Dictionary of Lists
            data["roll"] : List of Floats
                The roll of the craft in rad at every time stamp in the
                simulation. Roll is defined as the rotation of the craft
                about the x-axis of the world coordinate system.
            data["pitch"] : List of Floats
                The pitch of the craft in rad at every time stamp in the
                simulation. Pitch is defined as the rotation of the craft
                about the y-axis of the world coordinate system.
            data["yaw"] : List of Floats
                The yaw of the craft in rad at every time stamp in the
                simulation. Yaw is defined as the rotation of the craft
                about the z-axis of the world coordinate system.
            data["wx"] : List of Floats
                The angular velocity of the spacecraft about the body-fixed
                x axis at each time stamp in the simulation. Given in rad/sec.
            data["wy"] : List of Floats
                The angular velocity of the spacecraft about the body-fixed
                y axis at each time stamp in the simulation. Given in rad/sec.
            data["wz"] : List of Floats
                The angular velocity of the spacecraft about the body-fixed
                z axis at each time stamp in the simulation. Given in rad/sec.
            data["q stars"] : List of 2D Coords
                A list of the 2D coords of each star in the star-tracker image
                at each time stamp in the simulation. nan means that the star
                is not in the star tracker image.
            data["torques"] : List of 4 Tuples of Floats
                A list of the torques applied to each wheel of the spacecraft
                in Newton-meters at each time stamp during the simulation. The
                wheel order is 1, 2, 3, 4.
            data["wheel speeds"] : List of 4 Tuples of Floats
                A list of the wheel speeds of each wheel of the spacecraft
                in rad / sec at each time stamp during the simulation. The
                wheel order is 1, 2, 3, 4.
            data["time"] : List of Floats
                A list of the time stamps in seconds.

        """
        # Reset the simulator
        self.sim.reset()
        
        # Set the intial orientation and angular vel
        self.sim.set_base_state(urdf_obj=self.craft_obj,
                                roll=initial_orientation[0],
                                pitch=initial_orientation[1],
                                yaw=initial_orientation[2],
                                ang_velocity=initial_ang_vel,
                                body_coords=True,
                                initial_cond=True)
        
        # Reset the debris counter
        self.in_motion = np.zeros(self.n_debris)
        self.curr_debris = 0
        
        # Reset the controller
        self.controller = controller
        self.controller.reset()
        
        # Create a dictionary to hold the simulation data
        self.data = {"roll" : [],
                     "pitch" : [],
                     "yaw" : [],
                     "wx" : [],
                     "wy" : [],
                     "wz" : [],
                     "q stars" : [],
                     "torques" : [],
                     "wheel speeds" : [],
                     "time" : []}

        # Await run command if visualization or animation is on
        if self.visualization or self.animation:
            self.sim.await_keypress(key='enter')
        
        # Run the simulation loop
        self.limits_exceeded = False
        while(not self.sim.is_done and not self.limits_exceeded):
            ##################################################################
            # SENSOR
            # Get the position orientation of the craft in world coords
            world_state = self.sim.get_base_state(urdf_obj=self.craft_obj,
                                                  body_coords=False)
            R_ofW_inC = world_state['R of world in body']
            roll = world_state['roll']
            pitch = world_state['pitch']
            yaw = world_state['yaw']
            
            # Convert the world coordinates of the stars to craft coordinates
            # then convert the craft coordinates to scope coordinates
            q_stars = []
            for P_inW in self.P_star_inW:
                P_star_inC = R_ofW_inC@P_inW
                q_y = (P_star_inC[1] / P_star_inC[0]) * 2.625
                q_z = (P_star_inC[2] / P_star_inC[0]) * 2.625
                q_star = np.array([q_y, q_z])
                q_star_norm = q_y*q_y + q_z*q_z
                if q_star_norm < 1.:
                    q_stars.append(q_star)
                else:
                    q_stars.append(np.array([np.nan, np.nan]))
                    self.limits_exceeded = True
                    print("Star(s) out of view")
            q_stars = np.array(q_stars)
            
            # Add sensor noise
            q_stars_noisy = np.zeros(q_stars.shape)
            for i in range(len(q_stars)):
                for j in range(len(q_stars[i])):
                    mean = q_stars[i][j]
                    if mean is np.nan:
                        q_stars_noisy[i][j] = np.nan
                    else:
                        noisy = np.random.normal(loc=mean, scale=sensor_noise)
                        q_stars_noisy[i][j] = noisy
            
            # Get the angular rates of the craft in the craft's coord system
            body_state = self.sim.get_base_state(urdf_obj=self.craft_obj,
                                                 body_coords=True)
            wx = body_state['angular velocity'][0]
            wy = body_state['angular velocity'][1]
            wz = body_state['angular velocity'][2]
            
            # Retrieve the wheel speeds
            whl1_state = self.sim.get_joint_state(urdf_obj=self.craft_obj,
                                                  joint_name='bus_to_wheel_1')
            whl2_state = self.sim.get_joint_state(urdf_obj=self.craft_obj,
                                                  joint_name='bus_to_wheel_2')
            whl3_state = self.sim.get_joint_state(urdf_obj=self.craft_obj,
                                                  joint_name='bus_to_wheel_3')
            whl4_state = self.sim.get_joint_state(urdf_obj=self.craft_obj,
                                                  joint_name='bus_to_wheel_4')
            whl1_speed = whl1_state['velocity']
            whl2_speed = whl2_state['velocity']
            whl3_speed = whl3_state['velocity']
            whl4_speed = whl4_state['velocity']
            wheel_speeds = (whl1_speed, whl2_speed, whl3_speed, whl4_speed)
            
            ###################################################################
            # CONTROLLER
            # Get the torque as calculated by the controller
            a = self.sim.is_pressed("a")
            q = self.sim.is_pressed("q")
            s = self.sim.is_pressed("s")
            w = self.sim.is_pressed("w")
            d = self.sim.is_pressed("d")
            e = self.sim.is_pressed("e")
            f = self.sim.is_pressed("f")
            r = self.sim.is_pressed("r")
            inputs = self.controller.run(a=a,
                                         q=q,
                                         s=s,
                                         w=w,
                                         d=d,
                                         e=e,
                                         f=f,
                                         r=r,
                                         q_stars=q_stars_noisy,
                                         dt=self.sim.dt)
            tau_1 = inputs[0]
            tau_2 = inputs[1]
            tau_3 = inputs[2]
            tau_4 = inputs[3]
            
            # Ensure torques are numbers
            if (tau_1 != tau_1):
                tau_1 = 0.0
            if (tau_2 != tau_2):
                tau_2 = 0.0
            if (tau_3 != tau_3):
                tau_3 = 0.0
            if (tau_4 != tau_4):
                tau_4 = 0.0
                
            # Limit the torques
            if tau_1 > 1.0:
                tau_1 = 1.0
            if tau_2 > 1.0:
                tau_2 = 1.0
            if tau_3 > 1.0:
                tau_3 = 1.0
            if tau_4 > 1.0:
                tau_4 = 1.0
            if tau_1 < -1.0:
                tau_1 = -1.0
            if tau_2 < -1.0:
                tau_2 = -1.0
            if tau_3 < -1.0:
                tau_3 = -1.0
            if tau_4 < -1.0:
                tau_4 = -1.0
                
            # Turn off torque if wheel speed too high
            if whl1_speed>=50. and tau_1>0.:
                tau_1 = 0.
                self.limits_exceeded = True
                print("Wheel 1 speed > 50 rad/s")
            if whl2_speed>=50. and tau_2>0.:
                tau_2 = 0.
                self.limits_exceeded = True
                print("Wheel 2 speed > 50 rad/s")
            if whl3_speed>=50. and tau_3>0.:
                tau_3 = 0.
                self.limits_exceeded = True
                print("Wheel 3 speed > 50 rad/s")
            if whl4_speed>=50. and tau_4>0.:
                tau_4 = 0.
                self.limits_exceeded = True
                print("Wheel 4 speed > 50 rad/s")
            if whl1_speed<=-50. and tau_1<0.:
                tau_1 = 0.
                self.limits_exceeded = True
                print("Wheel 1 speed < -50 rad/s")
            if whl2_speed<=-50. and tau_2<0.:
                tau_2 = 0.
                self.limits_exceeded = True
                print("Wheel 2 speed < -50 rad/s")
            if whl3_speed<=-50. and tau_3<0.:
                tau_3 = 0.
                self.limits_exceeded = True
                print("Wheel 3 speed < -50 rad/s")
            if whl4_speed<=-50. and tau_4<0.:
                tau_4 = 0.
                self.limits_exceeded = True
                print("Wheel 4 speed < -50 rad/s")
                
            # Assemble torques
            torque = (tau_1, tau_2, tau_3, tau_4)
            
            # ###################################################################
            # SIMULATION DATA
            # Append data to history lists
            self.data["roll"].append(roll)
            self.data["pitch"].append(pitch)
            self.data["yaw"].append(yaw)
            self.data["wx"].append(wx)
            self.data["wy"].append(wy)
            self.data["wz"].append(wz)
            self.data["q stars"].append(q_stars)
            self.data["torques"].append(torque)
            self.data["wheel speeds"].append(wheel_speeds)
            self.data["time"].append(self.sim.time)
            
            # ###################################################################
            # # ACTUATOR
            
            # # Apply one quater of the controller calculated torque to
            # # each of the four the wheels.
            self.sim.set_joint_torque(urdf_obj=self.craft_obj,
                                      joint_name='bus_to_wheel_1',
                                      torque=tau_1,
                                      show_arrow=True,
                                      arrow_scale=2.,
                                      arrow_offset=0.1)
            self.sim.set_joint_torque(urdf_obj=self.craft_obj,
                                      joint_name='bus_to_wheel_2',
                                      torque=tau_2,
                                      show_arrow=True,
                                      arrow_scale=2.,
                                      arrow_offset=0.1)
            self.sim.set_joint_torque(urdf_obj=self.craft_obj,
                                      joint_name='bus_to_wheel_3',
                                      torque=tau_3,
                                      show_arrow=True,
                                      arrow_scale=2.,
                                      arrow_offset=0.1)
            self.sim.set_joint_torque(urdf_obj=self.craft_obj,
                                      joint_name='bus_to_wheel_4',
                                      torque=tau_4,
                                      show_arrow=True,
                                      arrow_scale=2.,
                                      arrow_offset=0.1)
            
            # Ensure the spacecraft is at [0,0,0]
            self.sim.set_base_state(urdf_obj=self.craft_obj,
                                    position=[0., 0., 0.],
                                    body_coords=False,
                                    initial_cond=False)
           
            ###################################################################
            # UPDATE THE PLOTS
            # This is how we add data points to the animator
            if self.animation:
                self.sim.add_subplot_point(subplot_index=self.p1,
                                           artist_index=self.a1[0],
                                           x=self.sim.time,
                                           y=roll)
                self.sim.add_subplot_point(subplot_index=self.p1,
                                           artist_index=self.a1[1],
                                           x=self.sim.time,
                                           y=pitch)
                self.sim.add_subplot_point(subplot_index=self.p1,
                                           artist_index=self.a1[2],
                                           x=self.sim.time,
                                           y=yaw)
                
            ###################################################################
            # DEBRIS MOVEMENT
            # Test whether to start debris movement
            debris_is_left = self.curr_debris<self.n_debris
            should_throw = np.random.rand()<self.debris_prob
            if debris and debris_is_left and should_throw:
                # Get the position of the debris to toss
                obj = self.debris_objs[self.curr_debris]
                debris_state = self.sim.get_base_state(urdf_obj=obj,
                                                       body_coords=False)
                debris_pos = debris_state['position']
                
                # Set the direction in which to move the debris
                debris_vel_dirn = -debris_pos / np.linalg.norm(debris_pos)
                self.sim.set_base_state(urdf_obj=obj,
                                        velocity=50.*debris_vel_dirn,
                                        body_coords=False,
                                        initial_cond=False)
                
                # Iterate the debris counter
                self.in_motion[self.curr_debris] = 1
                self.curr_debris = self.curr_debris + 1
            
            # Remove any debris that has hit the craft
            if debris:
                for i in range(self.curr_debris):
                    # Check if debris is in motion
                    if not self.in_motion[i]:
                        continue
                    
                    # See if the debris is in the craft
                    obj = self.debris_objs[i]
                    debris_state = self.sim.get_base_state(urdf_obj=obj,
                                                           body_coords=False)
                    debris_pos = debris_state['position']
                    debris_norm = np.linalg.norm(debris_pos)
                    
                    # If the debris is in the craft, move it out of view and
                    # impart a random rotational impulse to the craft
                    if debris_norm < 2.0:
                        # Remove the debris that struck
                        self.sim.set_base_state(urdf_obj=obj,
                                                position=[200., 200., 200.],
                                                velocity=[0., 0., 0.],
                                                body_coords=False,
                                                initial_cond=False)
                        self.in_motion[i] = 0
                        
                        # Check to see if no more debris is left
                        if all(self.in_motion==0) and not debris_is_left:
                            self.limits_exceeded = True
                            print("No debris left")
                        
                        # Get a random impulse
                        x_impulse = 0.25*(np.random.rand()-0.5)
                        y_impulse = 0.25*(np.random.rand()-0.5)
                        z_impulse = 0.25*(np.random.rand()-0.5)
                        wx_impact = wx + x_impulse
                        wy_impact = wy + y_impulse
                        wz_impact = wz + z_impulse
                        ang_vel_impact = np.array([wx_impact,
                                                   wy_impact,
                                                   wz_impact])
                        
                        # Set the angular vel after impulse
                        self.sim.set_base_state(urdf_obj=self.craft_obj,
                                                ang_velocity=ang_vel_impact,
                                                body_coords=True,
                                                initial_cond=False)
            
            ###################################################################
            # STEP THE SIMULATION
            # Step the sim
            val = self.sim.step(real_time=True,
                                update_vis=self.visualization,
                                update_ani=self.animation,
                                max_time=max_time)
            
            # Handle resetting the controller and simulation data
            if val == 3:
                # Reset the debris counter
                self.curr_debris = 0
                self.in_motion = np.zeros(self.n_debris)
                
                # Reset the controller
                self.controller.reset()
                
                # Reset the terminate condition
                self.limits_exceeded = False
                
                # Reset the history
                self.data = {"roll" : [],
                             "pitch" : [],
                             "yaw" : [],
                             "wx" : [],
                             "wy" : [],
                             "wz" : [],
                             "q stars" : [],
                             "torques" : [],
                             "wheel speeds" : [],
                             "time" : []}
       
        # When the simulation is done running, return the data
        return self.data
            