'''
This py script provides a tutorial and template for creating the backend of a 
new project using condynsate. In this tutorial, we will cover how to import 
physics objects in the condynsate physics environment by importing a single
cube 1 meter above a solid ground plane. We will then test that this cube 
behaves as expected before moving on to tutorial 2. This project will not cover
how to create your own URDF files. Instead, we recommend reviewing 
https://wiki.ros.org/urdf.
'''


'''
To begin, we will import all of the required dependencies. In general, for 
projects that will simulate the physics of a set of urdf objects, the only 
module needed from the condynsate project is condynsate.simulator. pathlib.Path
is also used in this tutorial
'''
from condynsate.simulator import Simulator as conSim
from pathlib import Path


'''
After importing our dependencies, we will create a project class. This class 
will provide exactly two functions ---the initialization function and the run 
function. 

The purpose of the Project class initialization function is to create a new 
instance of the project. Specifically, we want to do at least two things in
the initialization function:
    1) Create an instance of a condynsate.simulator class. This class will 
       handle visualizing in real time and simulating the physics of whichever 
       urdf files you load. 
    2) Load your urdf files into the condynsate simulator.
    
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
        
        
        '''
        STEP 1: Initialize an instance of the condynsate.simulator class. 
                Note that for this project, we do not want to use animation. 
                This means that we do not want to plot things in real time. 
                To avoid condynsate from assuming that we want to do this and
                running some things in the background that take compute power, 
                we simply set the animation flag in the initialization function
                to be false.
        '''
        # Create a instance of condynsate.simulator
        self.s = conSim(animation=False)
        
        
        '''
        STEP 2: Load our URDF files. Here we have already included the required
                plane and cube URDF files so you do not need to worry about 
                creating your own. They are stored in a folder in called vis.
                To access these files, we use pathlib to get the absolute path
                pointing to them and then tell our instance of the condynsate
                simulator to load the URDF file at that path location.
        '''
        # Get the path to the current directory
        path = (Path(__file__).parents[0]).absolute().as_posix()
        
        # Get the absolute paths that point to the plane and cube URDF files
        plane_path = path + "/vis/plane.urdf"
        cube_path = path + "/vis/cube.urdf"
        
        
        '''
        Load the ground URDF file into the simulator as a fixed object.
        
        The urdf_path requires a posix style, absolute path STRING pointing
        to the URDF file that will be loaded into the simulation
        
        position and wxyz_quaternion define the position and orientation 
        in which the URDF object will be placed. These positions and
        orientations are defined around the parent axes of the URDF object.
        By setting position = [0, 0, 0] and wxyz_quaternion = [1, 0, 0, 0],
        we place the plane at (0, 0, 0) oriented in the XY plane.
        
        By setting fixed = True, we are telling the simulator NOT to update 
        the physics of this object. It will still have collision, though. 
        
        By setting update_vis = False, we are telling the simulator NOT to 
        send updates to the visualizer for this object. This flag is usually
        set to False when fixed is set to True to reduce the amount of compute
        power required by the visualization
        '''
        # Load the ground
        self.ground = self.s.load_urdf(urdf_path=plane_path,
                                       position=[0., 0., 0.],
                                       wxyz_quaternion=[1., 0., 0., 0],
                                       fixed=True,
                                       update_vis=False)
        
        
        '''
        Load the cube URDF file into the simulator as a physics object.
        
        By setting position = [0, 0, 1] and wxyz_quaternion = [1, 0, 0, 0],
        we place the cube at (0, 0, 1) in the orientation defined by the URDF
        parent axes.
        
        By setting fixed = False, we are telling the simulator to update 
        the physics of the cube.
        
        By setting update_vis = True, we are telling the simulator to 
        send updates to the visualizer for this object. This is usually set to 
        true when an object is not fixed and will change its position and/or
        orientation.
        '''
        # Load the cube
        self.cube_1 = self.s.load_urdf(urdf_path=cube_path,
                                       position=[0., 0., 1.],
                                       wxyz_quaternion=[1., 0., 0., 0],
                                       fixed=False,
                                       update_vis=True)


    '''
    Now that we have created a simple initialization function, we move on to 
    the run function. This function will run a physics simulation using
    condynsate. Essentially, we will do three things:
        1) Reset the simulation environment to the initial conditions. Whereas
           this will not do anything for this particular example, it is best 
           practice to always call condynsate.simulator.reset() before 
           running a simulation loop.
        2) Wait for a user to press enter. This is acheived with the 
           condynsate.simulator.await_keypress(key='enter') function call.
        3) Run a simulation to completion. In this case, completion is defined
           as the simulator reaching max_time or the user pressing the 'esc' 
           key. We will describe how to do this using a while loop and 
           condynsate.simulator.step() below.
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


        '''
        This function call makes it so that the simulator will fully suspend
        until the user presses the enter key. Note that this is not a necessary
        function to call. If you want the simulation to run as soon as
        Project.run() is called, do not include this function call.
        '''
        # Await run command.
        self.s.await_keypress(key='enter')
            
        
        '''
        Now we create the simulation loop. condynsate.simulator cannot run an
        entire simulation by itself. Instead, it can take one time step of 0.01
        seconds. Therefore, the way we run an entire simulation is to place the 
        condynsate.simulator.step() function inside a while loop that executes 
        until the boolean flag condynsate.simulator.is_done is True. Note that
        this flag will automatically be set to true when max_time is reached or
        the user presses the esc key. Note that if the max_time argument to 
        condynsate.simulator.step() is None, the is_done boolean flag will not
        be set to true until the esc key is pressed. Further, if the max_time 
        argument to  condynsate.simulator.step() is None AND keyboard 
        interactivity is disabled, the is_done boolean flag will be set to true
        at 10.0 seconds.
        '''
        # Run the simulation loop until the is_done boolean flag is flipped to
        # True
        while(not self.s.is_done):
            
            # Take a single physics step of 0.01 seconds. This will
            # automatically update the physics and the visualizer, and attempts
            # to run in real time.
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
modify the Project.initialization function so that two cubes, one above the 
other, are loaded but only the top one has physics applied to it.
'''
    