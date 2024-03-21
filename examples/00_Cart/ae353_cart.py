"""
This modules provides a backend for the ae353 cart example
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
class Cart_sim():
    def __init__(self,
                 use_keyboard=True,
                 visualization=True,
                 visualization_fr=20.,
                 animation=True,
                 animation_fr=10.):
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
        
        # Load the ground
        plane_path = path + "/cart_vis/plane.urdf"
        self.ground_obj = self.sim.load_urdf(urdf_path=plane_path,
                                        position=[0., 0., 0.],
                                        wxyz_quaternion=[1., 0., 0., 0],
                                        fixed=True,
                                        update_vis=False)

        # Load the walls
        concrete_path = path + "/cart_vis/concrete.png"
        self.left_wall_obj = self.sim.load_urdf(urdf_path=plane_path,
                                           tex_path=concrete_path,
                                           position=[0., -5., 0.],
                                           roll=-np.pi/2.,
                                           fixed=True,
                                           update_vis=False)
        self.right_wall_obj = self.sim.load_urdf(urdf_path=plane_path,
                                            tex_path=concrete_path,
                                            position=[0., 5., 0.],
                                            roll=np.pi/2.,
                                            fixed=True,
                                            update_vis=False)
        
        # Load the cart
        cart_path = path + "/cart_vis/cart.urdf"
        self.cart_obj = self.sim.load_urdf(urdf_path=cart_path,
                                           position=[0., 0., 0.25],
                                           yaw=np.pi/2,
                                           fixed=False,
                                           update_vis=True)

        # If there is no animation, do not add subplots
        if not animation:
            return
        
        # Make plot for state and input
        self.p1, self.a1 = self.sim.add_subplot(n_artists=2,
                                                subplot_type='line',
                                                title="Angles vs Time",
                                                x_label="Time [Seconds]",
                                                y_label="Angles [Deg]",
                                                colors=["m", "c"],
                                                line_widths=[2.5, 2.5],
                                                line_styles=["-", "-"],
                                                labels=["Pendulum",
                                                        "Wheel"],
                                                h_zero_line=True)
        self.p2, self.a2 = self.sim.add_subplot(n_artists=1,
                                                subplot_type='line',
                                                title="Torque vs Time",
                                                x_label="Time [Seconds]",
                                                y_label="Torque [Nm]",
                                                y_lim=[-0.80, 0.80],
                                                colors=["k"],
                                                line_widths=[2.5],
                                                line_styles=["-"],
                                                h_zero_line=True)
        
        # Open the animator GUI
        self.sim.open_animator_gui()


    def run(self,
            controller,
            max_time = None,
            initial_pendulum_angle = 0.0,
            initial_wheel_angle = 0.0,
            initial_pendulum_velocity = 0.0,
            initial_wheel_velocity = 0.0):
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
        initial_pendulum_angle : Float, optional
            The initial angle of the pendulum in radians. This is set
            when the simulation starts and when the simulation is
            reset. The default value is 0.0.
        initial_wheel_angle : Float, optional
            The initial angle of the wheels in radians. This is set
            when the simulation starts and when the simulation is
            reset. The default value is 0.0.
        initial_pendulum_velocity : Float, optional
            The initial velocity of the pendulum in radians/second.
            This is set when the simulation starts and when the
            simulation is reset. The default value is 0.0.
        initial_wheel_velocity : Float, optional
            The initial velocity of the wheels in radians/second.
            This is set when the simulation starts and when the
            simulation is reset. The default value is 0.0.
            
        Returns
        -------
        data : Dictionary of Lists
            data["pendulum_angle"] : List of Floats
                A list of the pendulum angle in radians at each time stamp
                during the simulation.
            data["wheel_angle"] : List of Floats
                A list of the wheel angle in radians at each time stamp during
                the simulation.
            data["pendulum_velocity"] : List of Floats
                A list of the pendulum velocity in radians/secondat each time
                stamp during the simulation.
            data["wheel_velocity"] : List of Floats
                A list of the wheel velocity in radians/second at each time
                stamp during the simulation.
            data["torque"] : List of Floats
                A list of the applied torque in Newton-meters at each time
                stamp during the simulation.
            data["time"] : List of Floats
                A list of the time stamps in seconds.

        """    
        # Set the initial values
        self.sim.set_joint_position(urdf_obj=self.cart_obj,
                                joint_name='chassis_to_arm',
                                position=initial_pendulum_angle,
                                initial_cond=True,
                                physics=False)
        self.sim.set_joint_velocity(urdf_obj=self.cart_obj,
                                joint_name='chassis_to_arm',
                                velocity=initial_pendulum_velocity,
                                initial_cond=True,
                                physics=False)
        wheels = ['chassis_to_wheel_1',
                  'chassis_to_wheel_2',
                  'chassis_to_wheel_3',
                  'chassis_to_wheel_4']
        for wheel_name in wheels:
            self.sim.set_joint_position(urdf_obj=self.cart_obj,
                                    joint_name=wheel_name,
                                    position=initial_wheel_angle,
                                    initial_cond=True,
                                    physics=False)
            self.sim.set_joint_velocity(urdf_obj=self.cart_obj,
                                    joint_name=wheel_name,
                                    velocity=initial_wheel_velocity,
                                    initial_cond=True,
                                    physics=False)
        
        
        # Reset the controller
        controller.reset()
        
        # Create a lists to hold the simulation data
        time_history = []
        pendulum_angle_history =[]
        pendulum_velocity_history = []
        wheel_angle_history =[]
        wheel_velocity_history = []
        torque_history = []

        # Await run command
        self.sim.await_keypress(key='enter')
            
        # Run the simulation loop
        while(not self.sim.is_done):
            ##################################################################
            # SENSOR
            # Use a sensor to collect the pendulum angle and rate
            pendulum_state = self.sim.get_joint_state(urdf_obj=self.cart_obj,
                                                  joint_name='chassis_to_arm')
            pendulum_angle = pendulum_state['position']
            pendulum_rate = pendulum_state['velocity']
            
            # Use a sensor to collect the angles and rates of each wheel
            cart_state = self.sim.get_base_state(urdf_obj=self.cart_obj,
                                                 body_coords=False)
            
            # Calculate the average wheel angle and velocity
            wheel_angle = -cart_state['position'][1]/0.125
            wheel_rate = -cart_state['velocity'][1]/0.125
            
            ###################################################################
            # CONTROLLER
            # Get the torque as calculated by the controller
            sd = self.sim.is_pressed("shift+d")
            sa = self.sim.is_pressed("shift+a")
            d = self.sim.is_pressed("d")
            a = self.sim.is_pressed("a")
            inputs = controller.run(pendulum_angle=pendulum_angle,
                                    wheel_angle=wheel_angle,
                                    pendulum_velocity=pendulum_rate,
                                    wheel_velocity=wheel_rate,
                                    time=self.sim.time,
                                    sd=sd,
                                    sa=sa,
                                    d=d,
                                    a=a)
            torque = inputs[0]
            if torque > 0.75:
                torque = 0.75
            elif torque < -0.75:
                torque = -0.75
            
            ###################################################################
            # SIMULATION DATA
            # Append data to history lists
            time_history.append(self.sim.time)
            pendulum_angle_history.append(pendulum_angle)
            pendulum_velocity_history.append(pendulum_rate)
            wheel_angle_history.append(wheel_angle)
            wheel_velocity_history.append(wheel_rate)
            torque_history.append(torque)
            
            ###################################################################
            # ACTUATOR
            # Apply one quater of the controller calculated torque to
            # each of the four the wheels.
            for wheel_name in wheels:
                self.sim.set_joint_torque(urdf_obj=self.cart_obj,
                                          joint_name=wheel_name,
                                          torque=torque,
                                          show_arrow=True,
                                          arrow_scale=0.25,
                                          arrow_offset=0.025)
            
            ###################################################################
            # UPDATE THE PLOTS
            # This is how we add data points to the animator
            # Plot the pendulum angle, wheel angle, and torque
            if self.animation:
                self.sim.add_subplot_point(subplot_index=self.p1,
                                           artist_index=self.a1[0],
                                           x=self.sim.time,
                                           y=180.*pendulum_angle/np.pi)
                self.sim.add_subplot_point(subplot_index=self.p1,
                                           artist_index=self.a1[1],
                                           x=self.sim.time,
                                           y=180.*wheel_angle/np.pi)
                self.sim.add_subplot_point(subplot_index=self.p2,
                                           artist_index=self.a2[0],
                                           x=self.sim.time,
                                           y=torque)
            
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
                controller.reset()
                
                # Reset the history
                time_history = []
                pendulum_angle_history =[]
                pendulum_velocity_history = []
                wheel_angle_history =[]
                wheel_velocity_history = []
                torque_history = []
                
                
        # When the simulation is done running, gather all simulation data 
        # into a dictionary and return it
        data = {'time' : time_history,
                'pendulum_angle' : pendulum_angle_history,
                'wheel_angle' : wheel_angle_history,
                'pendulum_velocity' : pendulum_velocity_history,
                'wheel_velocity' : wheel_velocity_history,
                'torque' : torque_history}
        return data
            