'''
This py script provides a tutorial and template for creating the backend of a 
project using condynsate that applies forces to the center of mass of an
unjointed URDF object. In this tutorial, we will cover how to apply forces and 
torques to the center of mass of an object and how to measure the position, 
orientation, velocity, and rotational velocity of the center of mass of that
object.
'''


'''
To begin, as in tutorial 01, we will import all of the required dependencies.
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
        sphere_path = path + "/vis/sphere.urdf"
        
        # Load the ground
        self.ground = self.s.load_urdf(urdf_path=plane_path,
                                       position=[0., 0., 0],
                                       wxyz_quaternion=[1., 0., 0., 0],
                                       fixed=True,
                                       update_vis=False)

        # Load the sphere so that its bottom plane is sitting on the ground
        self.sphere = self.s.load_urdf(urdf_path=sphere_path,
                                       position=[0., 0., 0.5],
                                       wxyz_quaternion=[1., 0., 0., 0],
                                       fixed=False,
                                       update_vis=True)


    '''
    Now that we have created a simple initialization function, we move on to 
    the run function. This function will run a physics simulation using
    condynsate. Essentially, we will do three things:
        1) Reset the simulation environment to the initial conditions.
        2) Wait for a user to press enter.
        3) Run a simulation that applies forces and torques based on the 
           state of the center of mass of the cube.
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
            First we want to measure the state (position, orientation,
            velocity, and angular vel) about the center of mass of the sphere.
            For a given URDF object that has already been loaded into the 
            physics environment, we can measure this by using the 
            condynsate.simulator.get_base_state() function. There are two 
            arguments to this function:
                1) urdf_obj: the unique URDF object ID that is returned when
                   condynsate.simulator.load_urdf() is called
                2) body_coords: A boolean flag that indicates whether the
                   velocity and angular velocity is given in world coords
                   (False) or body coords (True). World coords are defined 
                   around the axes defined in the URDF file.
            In this case, we want the world coords, so we set body_coords to
            False
            
            state, which is returned by condynsate.simulator.get_base_state()
            has the following form:
            state : a dictionary with the following keys:
                'position' : array-like, shape (3,)
                    The (x,y,z) world coordinates of the base of the urdf.
                'roll' : float
                    The Euler angle roll of the base of the urdf
                    that define the body's orientation in the world. Rotation
                    of the body about the world's x-axis.
                'pitch' : float
                    The Euler angle pitch of the base of the urdf
                    that define the body's orientation in the world. Rotation
                    of the body about the world's y-axis.
                'yaw' : float
                    The Euler angle yaw of the base of the urdf
                    that define the body's orientation in the world. Rotation
                    of the body about the world's z-axis.
                'R of world in body' : array-like, shape(3,3):
                    The rotation matrix that takes vectors in world coordinates 
                    to body coordinates. For example, let V_inB be a 3vector
                    written in body coordinates. Let V_inW be a 3vector 
                    written in world coordinates. Then:
                    V_inB = R_ofWorld_inBody @ V_inW
                'velocity' : array-like, shape (3,)
                    The linear velocity of the base of the urdf in either world 
                    coords or body coords. Ordered as either
                    (vx_inW, vy_inW, vz_inW) or (vx_inB, vy_inB, vz_inB).
                'angular velocity' : array-like, shape (3,)
                    The angular velocity of the base of the urdf in either
                    world coords or body coords. Ordered as either
                    (wx_inW, wy_inW, wz_inW), or (wx_inB, wy_inB, wz_inB). When
                    written in world coordinates, exactly equal to the roll
                    rate, the pitch rate, and the yaw rate.
            '''
            # Use a sensor to collect the angles and rates of each wheel
            state = self.s.get_base_state(urdf_obj=self.sphere,
                                          body_coords=False)
            
            
            '''
            Suppose we wanted to apply an upward force to the center of mass of
            the sphere if it is less than 1.0 meters above the ground. To do
            this, we would need to first measure its height off of the ground.
            We can do this by extracting the height from its state.
            '''
            # Extract the height of the center of mass of the sphere
            position = state['position']
            height = position[2]
            
            
            '''
            Now we simply write an if statement that applies a force to the
            center of mass if the height is less than 1.0 and applies 0 force
            if the height is greater than 1.0. To apply a force, we use the 
            condynsate.simulator.apply_force_to_com() function. This function
            has six arguments:
                urdf_obj : URDF_Obj
                    A URDF_Obj to which the force is applied.
                force : array-like, shape (3,)
                    The force vector in either world or body coordinates to
                    apply to the body.
                body_coords : bool, optional
                    A boolean flag that indicates whether force is given in
                    body coords (True) or world coords (False). The default is
                    False.
                show_arrow : bool, optional
                    A boolean flag that indicates whether an arrow will be
                    rendered on the CoM to visualize the applied force. The
                    default is False.
                arrow_scale : float, optional
                    The scaling factor that determines the size of the arrow.
                    The default is 0.4.
                arrow_offset : float, optional
                    The amount by which the drawn force arrow will be offset
                    from the center of mass along the direction of the applied
                    force. The default is 0.0.
                    
            In this case, we want to draw the force arrow so we set show_arrow
            to True and adjust arrow_scale and arrow_offset until the size and
            position of the arrow look correct, respectively. 
            '''
            # Apply an upward force if the height is less than 1.0
            # meters. Because the sphere has a radius of 0.5m, we set the 
            # arrow offset to 0.5 meters.
            if height <= 1.0:
                self.s.apply_force_to_com(urdf_obj=self.sphere,
                                          force=[0.,0.,20.],
                                          body_coords=False,
                                          show_arrow=True,
                                          arrow_scale=0.05,
                                          arrow_offset=0.5) 
            else:
                self.s.apply_force_to_com(urdf_obj=self.sphere,
                                          force=[0.,0.,0.])
              
                
            '''
            Suppose we also wanted to apply a torque about the center of mass
            of the sphere near the top of its trajectory. To do this we would
            need to measure not only its height, but also its upward speed.
            We can do this by extracting the upward speed from its state.
            '''
            # Extract the upward speed of the center of mass of the sphere
            velocity = state['velocity']
            upward_speed = abs(velocity[2])
            
            
            '''
            Now 1we simply write an if statement that applies a torque to the
            center of mass if the height is more than 1.0 and its upward speed
            is less than 2.0 in magnitude. Otherwise it applies 0 torque. To
            apply a torque about the center of mass of a URDF object, we use
            the  condynsate.simulator.apply_external_torque() function. This
            has five arguments:
                urdf_obj : URDF_Obj
                    A URDF_Obj to which the torque is applied.
                torque : array-like, shape(3,)
                    The torque vector in world coordinates to apply to the
                    body.
                body_coords : bool, optional
                    A boolean flag that indicates whether torque is given in
                    body coords (True) or world coords (False). The default is
                    False.
                show_arrow : bool, optional
                    A boolean flag that indicates whether an arrow will be
                    rendered on the com to visualize the applied torque. The
                    default is False.
                arrow_scale : float, optional
                    The scaling factor that determines the size of the arrow.
                    The default is 0.1.
                arrow_offset : float, optional
                    The amount by which the drawn torque arrow will be offset
                    from the center of mass along the direction of the applied
                    torque. The default is 0.0.
                    
            In this case, we want to draw the torque arrow so we set show_arrow
            to True and adjust arrow_scale and arrow_offset until the size and
            position of the arrow look correct, respectively. 
            '''
            if height > 1.0 and upward_speed < 2.0:
                self.s.apply_external_torque(urdf_obj=self.sphere,
                                             torque=[0.,0.,0.1],
                                             body_coords=False,
                                             show_arrow=True,
                                             arrow_scale=8.0,
                                             arrow_offset=0.5) 
            else:
                self.s.apply_external_torque(urdf_obj=self.sphere,
                                             torque=[0.,0.,0.])
            
            
            # Take a single physics step of dt seconds. This will
            # automatically update the physics and the visualizer, and attempts
            # to run in real time. Additionally, it will apply the forces and
            # torques set using the apply_force_to_com() and 
            # apply_external_torque() functions
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
modify the Project.run() function so that the force applied is porportional to 
the magnitude of the velocity of the sphere.
'''
    