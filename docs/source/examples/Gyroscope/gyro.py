"""
This modules provides a backend for the a 2 axis gyroscope example
"""

###############################################################################
#DEPENDENCIES
###############################################################################
from condynsate import Simulator
from condynsate import __assets__ as assets
import numpy as np


###############################################################################
#SIMULATION CLASS
###############################################################################
class Gyro_sim():
    def __init__(self,
                 use_keyboard=True,
                 visualization_fr=40.):
        """
        Initializes an instance of the simulation class.

        Parameters
        ----------
        keyboard : bool, optional
            A boolean flag that indicates whether the simulation will allow
            the use of keyboard interactivity. The default is True.
        visualization_fr : float, optional
            The frame rate (frames per second) at which the visualizer is
            updated. The default is 40..

        Returns
        -------
        None.

        """
        # Initialize and instance of the simulator
        self.sim = Simulator(keyboard=use_keyboard,
                             visualization=True,
                             visualization_fr=visualization_fr,
                             animation=False,
                             dt=0.005)

        # Load the ground
        self.plane_obj = self.sim.load_urdf(urdf_path=assets['plane_big'],
                                           fixed=True,
                                           update_vis=False)
        
        # Load the gyro
        self.gyro_obj = self.sim.load_urdf(urdf_path=assets['gyro'],
                                           fixed=True,
                                           update_vis=True,
                                           position=[0.,0.,0.05])


    def run(self,
            wheel_vel=10.,):
        # Reset the simulator
        self.sim.reset()
        
        # Set the initial conditions
        wheel_mx = 20.0
        wheel_mn = -20.0
        wheel_vel = np.clip(wheel_vel, wheel_mn, wheel_mx)
        self.sim.set_joint_velocity(urdf_obj=self.gyro_obj,
                                    joint_name='r1_to_core',
                                    velocity=wheel_vel,
                                    initial_cond=True,
                                    physics=False,
                                    color=True,
                                    min_vel=wheel_mn,
                                    max_vel=wheel_mx)
        # Set joint damping
        self.sim.set_joint_damping(urdf_obj=self.gyro_obj,
                                   joint_name='base_to_outer',
                                   damping=0.02)
        self.sim.set_joint_damping(urdf_obj=self.gyro_obj,
                                   joint_name='outer_to_r2',
                                   damping=0.02)
        self.sim.set_joint_damping(urdf_obj=self.gyro_obj,
                                   joint_name='r2_to_r1',
                                   damping=0.02)
        
        # Data collection 
        data = {"time" : [],
                "outer" : [],
                "r2" : [],
                "r1" : [],
                "tauO" : [],
                "tau2" : [],
                "tau1" : [],}
        
        # Await run command
        self.sim.await_keypress(key='enter')
            
        # Run the simulation loop
        while(not self.sim.is_done):
            # Determine what torques to apply based on key presses
            tauO = 0
            tau2 = 0
            tau1 = 0
            if self.sim.is_pressed('q'): tauO -= 0.1
            if self.sim.is_pressed('e'): tauO += 0.1
            if self.sim.is_pressed('w'): tau2 -= 0.1
            if self.sim.is_pressed('s'): tau2 += 0.1
            if self.sim.is_pressed('a'): tau1 -= 0.1
            if self.sim.is_pressed('d'): tau1 += 0.1
            
            # Apply torques based on key presses
            self.sim.set_joint_torque(urdf_obj=self.gyro_obj,
                                      joint_name="base_to_outer",
                                      torque=tauO,
                                      show_arrow=True,
                                      arrow_offset=-1.3,
                                      arrow_scale=8.)
            self.sim.set_joint_torque(urdf_obj=self.gyro_obj,
                                      joint_name="outer_to_r2",
                                      torque=tau2,
                                      show_arrow=True,
                                      arrow_offset=0.9,
                                      arrow_scale=8.)
            self.sim.set_joint_torque(urdf_obj=self.gyro_obj,
                                      joint_name="r2_to_r1",
                                      torque=tau1,
                                      show_arrow=True,
                                      arrow_offset=-0.65,
                                      arrow_scale=8.)
            
            # Collect states and add to data
            data["time"].append(self.sim.time)
            data["outer"].append(self.sim.get_joint_state(self.gyro_obj,
                                                       "base_to_outer"))
            data["r2"].append(self.sim.get_joint_state(self.gyro_obj,
                                                       "outer_to_r2"))
            data["r1"].append(self.sim.get_joint_state(self.gyro_obj,
                                                       "r2_to_r1"))
            data["tauO"].append(tauO)
            data["tau2"].append(tau2)
            data["tau1"].append(tau1)

            # Iterate the wheel speed
            wheel_vel = self.sim.iterate_val(wheel_vel, 
                                             'f', 
                                             'r',
                                             iter_val=0.2,
                                             min_val=wheel_mn,
                                             max_val=wheel_mx)
            self.sim.set_joint_velocity(urdf_obj=self.gyro_obj,
                                        joint_name='r1_to_core',
                                        velocity=wheel_vel,
                                        initial_cond=False,
                                        physics=False,
                                        color=True,
                                        min_vel=wheel_mn,
                                        max_vel=wheel_mx)

            # Step the sim
            val = self.sim.step(real_time=True,
                                update_vis=True,
                                update_ani=False,
                                max_time=None)
            
            # Handle resetting the controller and simulation data
            if val == 3:
                wheel_vel = self.sim.get_joint_state(self.gyro_obj,
                                                     'r1_to_core')['velocity']
                data = {"time" : [],
                        "outer" : [],
                        "r2" : [],
                        "r1" : [],
                        "tauO" : [],
                        "tau2" : [],
                        "tau1" : [],}
        
        # Return the collected data
        return data
    