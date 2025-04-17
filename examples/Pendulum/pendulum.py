"""
This modules provides a backend for a pendulum example
"""

###############################################################################
#DEPENDENCIES
###############################################################################
from condynsate.simulator import Simulator as conSim
from condynsate import __assets__ as assets


###############################################################################
#SIMULATION CLASS
###############################################################################
class Pendulum_sim():
    def __init__(self,
                 use_keyboard=True,
                 visualization=True,
                 visualization_fr=40.):
        
        # Create a instance of condynsate.simulator.
        self.visualization = visualization
        self.s = conSim(keyboard=use_keyboard,
                        visualization=visualization,
                        visualization_fr=visualization_fr,
                        animation=False)
        
        # Load the pendulum
        self.pendulum = self.s.load_urdf(urdf_path=assets['pendulum'],
                                         position=[0., 0., 0.],
                                         yaw=1.5708,
                                         pitch=3.1416,
                                         wxyz_quaternion=[1.,0.,0.,0],
                                         fixed=True,
                                         update_vis=True)


    def run(self, 
            starting_angle=0.0, 
            damping=0.0,
            max_time=None):
        # Create an empty dictionary of data to return to users at the 
        # end of the simulation
        data = {'angle' : [],
                'angle_vel' : [],
                'time' : [],}
        
        # Set the initial conditions
        self.s.set_joint_position(urdf_obj=self.pendulum,
                                    joint_name='chassis_to_arm',
                                    position=starting_angle,
                                    initial_cond=True,
                                    physics=False)
        self.s.set_joint_damping(urdf_obj=self.pendulum,
                                 joint_name='chassis_to_arm',
                                 damping=damping)
        
        # Reset the simulator. It is best practice to do this before every 
        # simulation loop.
        self.s.reset()

        # Await run command.
        self.s.await_keypress(key='enter')
        
        # Run the simulation loop until the is_done boolean flag is flipped to
        # True
        while(not self.s.is_done):    
            # Add the simulation data at the current time
            state = self.s.get_joint_state(urdf_obj=self.pendulum,
                                             joint_name='chassis_to_arm')
            data['time'].append(self.s.time)
            data['angle'].append(state['position'])
            data['angle_vel'].append(state['velocity'])
            
            # Take a single physics step of dt seconds. This will
            # automatically update the physics and the visualizer, and attempts
            # to run in real time.
            val = self.s.step(real_time=self.visualization,
                              max_time=max_time)
            
            # Handle resetting the data collection if the simulation is reset
            # by the user pressing the backspace key. 
            if val == 3:
                data = {'angle' : [],
                        'angle_vel' : [],
                        'time' : [],}
        
        # At the end of the simulation, return the data
        return data
    