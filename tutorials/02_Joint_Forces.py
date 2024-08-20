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
        
        # Get the absolute path that points to the URDF file
        pendulum_path = path + "/vis/pendulum.urdf"

        '''
        Note here that we have set fixed to false. This means that the physics
        of the base of the object will not be update. That is, the object's
        base will be fixed in space. We can still apply external and internal
        forces to all of the joints, though, and thier physics will be updated.
        '''
        # Load the pendulum so that its bottom plane is sitting on the ground
        self.pendulum = self.s.load_urdf(urdf_path=pendulum_path,
                                         position=[0., 0., 0.05],
                                         yaw=1.570796327,
                                         wxyz_quaternion=[1., 0., 0., 0],
                                         fixed=True,
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
            '''
            First we want to measure the state (position, and velocity) of the 
            pendulum joint. For a given URDF object with joints that has
            already been loaded into the physics environment, we can measure
            this by using the condynsate.simulator.get_joint_state() function.
            There are two arguments to this function:
                1) urdf_obj : URDF_Obj
                   A URDF_Obj whose joint state is being measured.
                2) joint_name : string
                   The name of the joint whose state is measured. The joint
                   name is specified in the .urdf file.
            
            state, which is returned by condynsate.simulator.get_joint_state()
            has the following form:
            state : dictionary with the following keys
                'position' : float
                    The position value of this joint.
                'velocity' : float
                    The velocity value of this joint.
                'reaction force' : list shape(3,)
                    These are the joint reaction forces. Only read if a torque
                    sensor is enabled for this joint.
                'reaction torque' : list shape(3,)
                    These are the joint reaction torques. Only read if a torque
                    sensor is enabled for this joint.
                'applied torque' : float
                    This is the motor torque applied during the last
                    stepSimulation. Note that this only applies in
                    VELOCITY_CONTROL and POSITION_CONTROL. If you use
                    TORQUE_CONTROL then the applied joint motor torque is
                    exactly what you provide, so there is no need to report it
                    separately.
            '''
            # Use a sensor to collect the state of the pendulum
            state = self.s.get_joint_state(urdf_obj=self.pendulum,
                                            joint_name='chassis_to_arm')
            
            
            '''
            Suppose we wanted to apply a returning torque whenever the pendulum
            angle is above some threshold. To do this, we would need to first
            measure the angle of the pendulum. We can do this by extracting the
            position (angle for a continuous joint) from its state.
            '''
            # Extract angle of the "chassis_to_arm" (pendulum) joint
            angle = 180. * state['position'] / 3.141592654
            
            
            '''
            Now we simply write an if statement that applies a returning torque
            to the joint if its angle is greater than 5.0 degrees. To apply a
            torque about a joint, we use the 
            condynsate.simulator.set_joint_torque() function. This function
            has six arguments:
                urdf_obj : URDF_Obj
                    A URDF_Obj that contains that joint whose torque is being
                    set.
                joint_name : string
                    The name of the joint whose torque is set. The joint name
                    is specified in the .urdf file
                torque : float, optional
                    The torque in NM to be applied to the joint. The default is
                    0..
                show_arrow : bool, optional
                    A boolean flag that indicates whether an arrow will be
                    rendered on the link to visualize the applied torque. The
                    default is False.
                arrow_scale : float, optional
                    The scaling factor that determines the size of the arrow.
                    The default is 0.1.
                arrow_offset : float, optional
                    The amount to offset the drawn arrow along the joint axis.
                    The default is 0.0.
                    
            In this case, we want to draw the torque arrow so we set show_arrow
            to True and adjust arrow_scale and arrow_offset until the size and
            position of the arrow look correct, respectively. 
            '''
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
            
            
            # Take a single physics step of dt seconds. This will
            # automatically update the physics and the visualizer, and attempts
            # to run in real time. Additionally, it will apply the torques set
            # using the set_joint_torque() function
            self.s.step(max_time=max_time)


'''
This main loop is an example of how to use the Project class we just created to
run a complete simulation of time 5 seconds.
'''
if __name__ == "__main__":
    # Create an instance of the Project class. 
    proj = Project()
    
    # Run the simulation.
    proj.run(max_time = 5.)
    
    
'''
This tutorial is now complete. For an added challenge, think of how you would 
modify the Project.run() to implement a PD controller. 
'''
