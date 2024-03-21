"""
This modules provides a backend for the ae353 segbot example
"""

###############################################################################
#DEPENDENCIES
###############################################################################
from condynsate.simulator import Simulator
from pathlib import Path
import numpy as np


###############################################################################
#SIMULATION CLASS
###############################################################################
class Segbot_sim():
    def __init__(self,
                 use_keyboard=True,
                 visualization=True,
                 visualization_fr=20.,
                 animation=True,
                 animation_fr=10.,
                 bumpy=False):
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
        bumpy: bool, optional
            A boolean flag that indicates whether the station will be built
            with bumps on the driving surface or not.
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
                             animation_fr=animation_fr)
        
        # Get the path to the current directory
        path = (Path(__file__).parents[0]).absolute().as_posix()
        
        # Load the station
        if bumpy:
            station = path + "/segbot_vis/station_bumpy.urdf"
        else:
            station = path + "/segbot_vis/station.urdf"
        self.station_obj = self.sim.load_urdf(urdf_path=station,
                                        position=[0, 2.5, 20],
                                        roll=np.pi/2.,
                                        pitch=np.pi/2.,
                                        fixed=True,
                                        update_vis=True)
    
        # Load the segbot
        segbot = path + "/segbot_vis/segbot.urdf"
        self.segbot_obj = self.sim.load_urdf(urdf_path=segbot,
                                        position=[0, 0, 0.527],
                                        fixed=False,
                                        update_vis=True)
        
        # Load the target marker
        target_path = path + "/segbot_vis/target_arrow.urdf"
        self.target_obj = self.sim.load_urdf(urdf_path=target_path,
                                             position=[-2.0, 0., 0.7],
                                             roll=np.pi,
                                             fixed=True,
                                             update_vis=True)
        
        # Only set the background lighting if the visualizer is being used
        if visualization:
            # Set background and lighting properties
            self.sim.set_background(top_color=[30, 30, 30],
                                    bot_color=[40, 40, 60])
            self.sim.set_posx_pt_light(on=True,
                                       intensity=0.5,
                                      distance=10.)
            self.sim.set_negx_pt_light(on=True,
                                       intensity=0.5,
                                      distance=10.)
            self.sim.set_ambient_light(on=True,
                                       intensity=0.65)
        
        # If there is animation, add subplots
        if animation:
            # Make plot for state and input
            self.p1, self.a1 = self.sim.add_subplot(
                                n_artists=2,
                                subplot_type='line',
                                title="Angles vs. Time",
                                x_label="Time [Seconds]",
                                y_label="Angles [Rad]",
                                colors=["m", "c"],
                                line_widths=[2.5, 2.5],
                                line_styles=["-", "-"],
                                labels=["Pitch", "Yaw"])
            self.p2, self.a2 = self.sim.add_subplot(
                                n_artists=2,
                                subplot_type='line',
                                title="Lateral Tracking",
                                x_label="Time [Seconds]",
                                y_label="Position [Meters]",
                                colors=["r", "b"],
                                line_widths=[2.5, 2.5],
                                line_styles=["-", ":"],
                                labels=["Pos", "Tag"])
            
            # Open the animator GUI
            self.sim.open_animator_gui()


    def run(self,
            controller,
            max_time = None,
            initial_e_lat = 0.0,
            station_velocity = -0.1):
        """
        Runs a complete simulation

        Parameters
        ----------
        controller : class
            A custom class that, at a minimum, must provide the functions
            controller.run() and controller.reset()
        max_time : Float, optional
            The total amount of time the simulation is allowed to run. When
            set to None, the simulator will run until the terminal command is 
            called. If the keyboard is disabled, users are not allowed to 
            set max time as None. The default value is None. 
        initial_e_lat : Float, optional
            The initial lateral error in meters. Changes the target lateral
            error, does not change the lateral position of the segbot.
            The default value is 0.0.
        station_velocity : Float, optional
            The angular velocity of the station in radians per second.
            Remains constant throughout simulation. The default value is 
            -0.1.

        Returns
        -------
        data : Dictionary of Lists
            data["lat_pos"] : List of Floats
                A list of the lateral position of the segbot in meters at 
                each time stamp during the simulation.
            data["tag_lat"] : List of Floats
                A list of the taget lateral position of the segbot in meters
                at each time stamp during the simulation.
            data["e_lat"] : List of Floats
                A list of the lateral position error of the segbot in meters at
                each time stamp during the simulation.
            data["v"] : List of Floats
                A list of the forward velocity of the segbot in meters/second
                at each time stamp during the simulation.
            data["phi"] : List of Floats
                A list of the heading angle of the chassis in radians at each
                time stamp during the simulation.
            data["phidot"] : List of Floats
                A list of the heading  angular rate of the chassis in 
                adians/second at each time stamp during the simulation.
            data["theta"] : List of Floats
                A list of the pitch angle of the chassis in radians at each
                time stamp during the simulation.
            data["thetadot"] : List of Floats
                A list of the pitch angular rate of the chassis in
                radians/second at each time stamp during the simulation.
            data["tau_l"] : List of Floats
                A list of the torque applied to the left wheel of the segbot in
                Newton-meters at each time stamp during the simulation.
            data["tau_r"] : List of Floats
                A list of the torque applied to the right wheel of the segbot
                in Newton-meters at each time stamp during the simulation.
            data["time"] : List of Floats
                A list of the time stamps in seconds.

        """
        # Set the initial values
        self.sim.set_joint_velocity(urdf_obj=self.station_obj,
                                joint_name='space_to_ring',
                                velocity=-station_velocity,
                                initial_cond=True,
                                physics=False)
        self.initial_e_lat = initial_e_lat
        tag_lat = self.initial_e_lat
        if tag_lat > 2.:
            tag_lat = 2.
        elif tag_lat < -2.:
            tag_lat = -2.
            
        # Reset the controller
        controller.reset()
        
        # Create a dictionary to hold the simulation data
        data = {"lat_pos" : [],
                "tag_lat" : [],
                "e_lat" : [],
                "v" : [],
                "phi" : [],
                "phidot" : [],
                "theta" : [],
                "thetadot" : [],
                "tau_l" : [],
                "tau_r" : [],
                "time" : []}

        # Await run command
        self.sim.await_keypress(key='enter')
            
        # Run the simulation loop
        while(not self.sim.is_done):
            ##################################################################
            # SENSOR
            # Use a sensor to collect the pendulum angle and rate
            body_state = self.sim.get_base_state(urdf_obj=self.segbot_obj,
                                                 body_coords=True)
            world_state = self.sim.get_base_state(urdf_obj=self.segbot_obj,
                                                  body_coords=False)
            lat_pos = -world_state['position'][1]
            v = -body_state['velocity'][0]
            phi = -world_state['yaw']
            phidot = -world_state['angular velocity'][2]
            theta = -body_state['pitch']
            thetadot = -body_state['angular velocity'][1]
            
            ###################################################################
            # CONTROLLER
            # Get the torque as calculated by the controller
            sw = self.sim.is_pressed("shift+w")
            w = self.sim.is_pressed("w")
            ss = self.sim.is_pressed("shift+s")
            s = self.sim.is_pressed("s")
            sd = self.sim.is_pressed("shift+d")
            d = self.sim.is_pressed("d")
            sa = self.sim.is_pressed("shift+a")
            a = self.sim.is_pressed("a")
            inputs = controller.run(e_lat=tag_lat-lat_pos,
                                    v=v,
                                    phi=phi,
                                    phidot=phidot,
                                    theta=theta,
                                    thetadot=thetadot,
                                    time=self.sim.time,
                                    sw=sw,
                                    w=w,
                                    ss=ss,
                                    s=s,
                                    sd=sd,
                                    d=d,
                                    sa=sa,
                                    a=a)
            tau_l = -inputs[0]
            tau_r = -inputs[1]
            if tau_l > 1.0:
                tau_l = 1.0
            if tau_r > 1.0:
                tau_r = 1.0
            if tau_l < -1.0:
                tau_l = -1.0
            if tau_r < -1.0:
                tau_r = -1.0
                
            # ###################################################################
            # SIMULATION DATA
            # Append data to history lists
            data["lat_pos"].append(lat_pos)
            data["tag_lat"].append(tag_lat)
            data["e_lat"].append(tag_lat-lat_pos)
            data["v"].append(v)
            data["phi"].append(phi)
            data["phidot"].append(phidot)
            data["theta"].append(theta)
            data["thetadot"].append(thetadot)
            data["tau_l"].append(tau_l)
            data["tau_r"].append(tau_r)
            data["time"].append(self.sim.time)
            
            # ###################################################################
            # # ACTUATOR
            # # Apply one quater of the controller calculated torque to
            # # each of the four the wheels.
            self.sim.set_joint_torque(urdf_obj=self.segbot_obj,
                                      joint_name="chassis_to_left_wheel",
                                      torque=tau_l,
                                      show_arrow=True,
                                      arrow_scale=0.5,
                                      arrow_offset=0.051)
            self.sim.set_joint_torque(urdf_obj=self.segbot_obj,
                                      joint_name="chassis_to_right_wheel",
                                      torque=tau_r,
                                      show_arrow=True,
                                      arrow_scale=0.5,
                                      arrow_offset=-0.051)
           
            ###################################################################
            # UPDATE THE PLOTS
            # This is how we add data points to the animator
            if self.animation:
                self.sim.add_subplot_point(subplot_index=self.p1,
                                           artist_index=self.a1[0],
                                           x=self.sim.time,
                                           y=theta)
                self.sim.add_subplot_point(subplot_index=self.p1,
                                           artist_index=self.a1[1],
                                           x=self.sim.time,
                                           y=phi)
                self.sim.add_subplot_point(subplot_index=self.p2,
                                           artist_index=self.a2[0],
                                           x=self.sim.time,
                                           y=lat_pos)
                self.sim.add_subplot_point(subplot_index=self.p2,
                                           artist_index=self.a2[1],
                                           x=self.sim.time,
                                           y=tag_lat)
            
            ###################################################################
            # UPDATE THE TARGET
            # Update the target lateral position
            tag_lat = self.sim.iterate_val(curr_val=tag_lat,
                                          down_key='l',
                                          up_key='j',
                                          iter_val=0.03,
                                          min_val=-2.,
                                          max_val=2.)
            
            # Adjust the target arrow so that it is always 
            # pointing in the target angle direction
            i = self.target_obj.urdf_id
            p = [-2.0, -tag_lat, 0.125*np.sin(self.sim.time*3) + 0.7]
            o = self.sim.engine.getQuaternionFromEuler([np.pi, 0, 0])
            self.sim.engine.resetBasePositionAndOrientation(bodyUniqueId=i,
                                                            posObj=p,
                                                            ornObj=o)
            
            ###################################################################
            # STEP THE SIMULATION
            # Step the sim
            val = self.sim.step(real_time=True,
                                update_vis=self.visualization,
                                update_ani=self.animation,
                                max_time=max_time)
            
            # Handle resetting the controller and simulation data
            if val == 3:
                # Reset the controller
                controller.reset()
                
                # Set the station speed
                self.sim.set_joint_velocity(urdf_obj=self.station_obj,
                        joint_name='space_to_ring',
                        velocity=-station_velocity,
                        initial_cond=True,
                        physics=False)
                
                # Reset the target lateral position
                tag_lat = self.initial_e_lat
                if tag_lat > 2.:
                    tag_lat = 2.
                elif tag_lat < -2.:
                    tag_lat = -2.
                
                # Reset the history
                data = {"lat_pos" : [],
                        "tag_lat" : [],
                        "e_lat" : [],
                        "v" : [],
                        "phi" : [],
                        "phidot" : [],
                        "theta" : [],
                        "thetadot" : [],
                        "tau_l" : [],
                        "tau_r" : [],
                        "time" : []}
                
        # When the simulation is done running, return the data
        return data
            