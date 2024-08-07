from condynsate.simulator import Simulator as conSim
from pathlib import Path

class Project():
    def __init__(self):
        '''
        Initializes an instance of the simulation class.

        Parameters
        ----------
        None.

        Returns
        -------
        None.

        '''
        # Create a instance of condynsate.simulator
        self.s = conSim(animation=False)
        
        # Get the path to the current directory
        path = (Path(__file__).parents[0]).absolute().as_posix()
        
        # Get the absolute paths that point to the plane and sphere URDF files
        plane_path = path + "/vis/plane.urdf"
        pendulum_path = path + "/vis/pendulum.urdf"
        
        # Load the ground
        self.ground = self.s.load_urdf(urdf_path=plane_path,
                                       position=[0., 0., 0.],
                                       wxyz_quaternion=[1., 0., 0., 0],
                                       fixed=True,
                                       update_vis=False)

        # Load the sphere so that its bottom plane is sitting on the ground
        self.pendulum = self.s.load_urdf(urdf_path=pendulum_path,
                                         position=[0., 0., 0.125],
                                         yaw=1.570796327,
                                         wxyz_quaternion=[1., 0., 0., 0],
                                         fixed=False,
                                         update_vis=True)
        
        # Set the initial angle of the pendulum arm
        self.s.set_joint_position(urdf_obj=self.pendulum,
                                  joint_name='chassis_to_arm',
                                  position=10.0*3.141592654/180.,
                                  initial_cond=True,
                                  physics=False)

    def run(self, max_time):
        '''
        Runs a complete simulation.

        Parameters
        ----------
        max_time : Float
            The total amount of time the simulation is allowed to run. When
            set to None, the simulator will run until the 'esc' key is 
            pressed.
            
        Returns
        -------
        None.

        '''
        # Reset the simulator. It is best practice to do this before every 
        # simulation loop.
        self.s.reset()

        # Await run command.
        self.s.await_keypress(key='enter')
        
        # Run the simulation loop until the is_done boolean flag is flipped to
        # True
        while(not self.s.is_done):
            # Use a sensor to collect the angles and rates of each wheel
            state = self.s.get_joint_state(urdf_obj=self.pendulum,
                                            joint_name='chassis_to_arm')
            
            # Extract angle of the chassis_to_arm joint
            angle = 180. * state['position'] / 3.141592654
            
            # If the angle is greater than some threshold angle, apply a torque
            # that will return it towards 0.0. If the angle is less than some 
            # threshold angle, apply a torque that will also return it towards
            # 0.0. Otherwise, do not apply a torque
            if angle > 5.:
                self.s.set_joint_torque(urdf_obj=self.pendulum,
                                        joint_name='chassis_to_arm',
                                        torque=-10,
                                        show_arrow=True,
                                        arrow_scale=0.05,
                                        arrow_offset=0.05)
            elif angle < -5.:
                self.s.set_joint_torque(urdf_obj=self.pendulum,
                                        joint_name='chassis_to_arm',
                                        torque=10,
                                        show_arrow=True,
                                        arrow_scale=0.05,
                                        arrow_offset=0.05)
            else:
                self.s.set_joint_torque(urdf_obj=self.pendulum,
                                        joint_name='chassis_to_arm',
                                        torque=0.0,
                                        show_arrow=True)
            
            
            # Take a single physics step of 0.01 seconds. This will
            # automatically update the physics and the visualizer, and attempts
            # to run in real time. Additionally, it will apply the torques set
            # using the set_joint_torque() function
            self.s.step(max_time=max_time)

if __name__ == "__main__":
    # Create an instance of the Project class. 
    proj = Project()
    
    # Run the simulation.
    proj.run(max_time = 20.)
    