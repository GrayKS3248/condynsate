'''
This py script provides a tutorial and template for creating the backend of a 
project using condynsate that applies torques to specific joints of a URDF
object. In this tutorial, we will cover how to torques to continuous joints
of a URDF object and how to measure the position and velocity of that joint.
'''


'''
To begin, as in tutorial 02, we will import all of the required dependencies.
'''
from condynsate.simulator import Simulator as conSim
from pathlib import Path


'''
After importing our dependencies, we will again create a project class. This
class provides the same functions as before ---the initialization function and
the run function. 
'''
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
        
        '''
        After loading our pendulum, we want to set its initial position so that
        it is at a slight angle. This is done using the 
        condynsate.simulator.set_joint_position() function. It has 5 arguments:
        urdf_obj : URDF_Obj
            A URDF_Obj that contains that joint whose position is being set.
        joint_name : string
            The name of the joint whose position is set. The joint name is
            specified in the .urdf file.
        position : float, optional
            The position in rad to be applied to the joint.
            The default is 0..
        physics : bool, optional
            A boolean flag that indicates whether physics based poisiton
            controller will be used to change joint position or whether
            the joint position will be reset immediately to the target
            position with zero end velocity. The default is False. 
        initial_cond : bool, optional
            A boolean flag that indicates whether the position set is an 
            initial condition of the system. If it is an initial condition,
            when the simulation is reset using backspace, the joint position
            will be set again.
        
        Note that because this is an intial condition, we set the initial_cond
        flag to true.
        '''
        # Set the initial angle of the pendulum arm
        self.s.set_joint_position(urdf_obj=self.pendulum,
                                  joint_name='chassis_to_arm',
                                  position=10.0*3.141592654/180.,
                                  initial_cond=True,
                                  physics=False)


    '''
    Now that we have created a simple initialization function, we move on to 
    the run function. This function will run a physics simulation using
    condynsate. Essentially, we will do three things:
        1) Reset the simulation environment to the initial conditions.
        2) Wait for a user to press enter.
        3) Run a simulation that applies torques based on the angle of a 
           specific joint
    '''
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
            # Take a single physics step of 0.01 seconds. This will
            # automatically update the physics and the visualizer, and attempts
            # to run in real time.
            self.s.step(max_time=max_time)


'''
This main loop is an example of how to use the Project class we just created to
run a complete simulation of time 20 seconds.
'''
if __name__ == "__main__":
    # Create an instance of the Project class. 
    proj = Project()
    
    # Run the simulation.
    proj.run(max_time = 20.)
    
    
'''
This tutorial is now complete. For an added challenge, think of how you would 
modify the Project.run() to implement a PD controller. 
'''