'''
In this tutorial we will cover how to extract state data of a particular link 
of a URDF object, as well as how to extract the mass of a given link. We will
also cover a general approach for storing this data during the simulation and
then creating a plot of the simulation data after the simulation has
terminated.
'''

'''
To being, as always, we import our dependencies
'''
from condynsate.simulator import Simulator as conSim
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt

'''
Then we, again, create the Project class. Note that this time we include two
additional functions compared to the previous tutorials. These are
Project.add_data_point() and Project.plot_data(). We will cover the specifics
of these functions as we get to them but generally speaking, add_data_point
is what we use to collect and store simulation data and plot_data is what we 
use to generate a plot of those data.
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
        Note here that, differently that the previous tutorials, we have 
        asked the simulator to use a smaller time step. Doing this will 
        increase the simulation accuracy at the expense of real-time 
        execution time. Everything else in the __init__ function has been 
        covered in previous tutorials.
        '''
        # Create a instance of condynsate.simulator. In this case, animation is
        # set to False and the fixed time step is reduced from default of 0.01
        # to improve simulation accuracy
        self.s = conSim(animation=False,
                        dt=0.0075)
        
        # Get the path to the current directory
        path = (Path(__file__).parents[0]).absolute().as_posix()
        
        # Get the absolute paths that point to the plane and sphere URDF files
        pendulum_path = path + "/vis/pendulum.urdf"

        # Load the sphere so that its bottom plane is sitting on the ground
        self.pendulum = self.s.load_urdf(urdf_path=pendulum_path,
                                         position=[0., 0., 0.],
                                         yaw=1.570796327,
                                         wxyz_quaternion=[1.,0.,0.,0],
                                         fixed=True,
                                         update_vis=True)

        # Set the initial angle of the pendulum arm
        self.s.set_joint_velocity(urdf_obj=self.pendulum,
                                  joint_name='chassis_to_arm',
                                  velocity=90*0.017453292519943295,
                                  initial_cond=True,
                                  physics=False)

    '''
    Moving on to the run() function, most things have already been covered in
    previous tutorials. What is new is the data dictionary, and collecting
    the returned value passed by condynsate.simulator.step(). We will cover
    these now.
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
        
        '''
        During this simulation, we want to collect state and time data so that
        we can use it to create a plot or pass it to a user that runs this 
        simulation. Here we choose to use a dictionary of lists as the data
        structure in which we store the simulation data in which we are 
        interested. Whereas this is not the only method to do this, it is a 
        method that is convienent for readability. This dictionary, with the 
        keys of the data that we want to collect, is created below.
        '''
        # Create an empty dictionary of data to return to users at the 
        # end of the simulation
        data = {'angle' : [],
                'angle_vel' : [],
                'KE' : [],
                'PE' : [],
                'time' : [],}
        
        # Reset the simulator. It is best practice to do this before every 
        # simulation loop.
        self.s.reset()

        # Await run command.
        self.s.await_keypress(key='enter')
        
        # Run the simulation loop until the is_done boolean flag is flipped to
        # True
        while(not self.s.is_done):    
            
            '''
            To add a data point for the current time step to the data
            structure, we call the Project.add_data_point() function. We will 
            cover what this function does in more detail below when we define 
            it.
            '''
            # Add the simulation data at the current time
            data = self.add_data_point(data)
            
            '''
            Note that, compared to the previous tutorials, this time when we 
            call condynsate.simulator.step() we store its return value. 
            condynsate.simulator.step() gives a different return value based
            on different situations. These return values are as follows:
                0: A normal step was taken
                1: No step was taken because the simulation was paused but now
                   it is no longer paused (spacebar was pressed).
                2: No step was taken because the simulation was paused.
                3: No step was taken because the simulation recieved a reset
                   command. The simulation reset itself, the visualizer, and 
                   the animator to the intial state. (backspace was pressed)
                4: A normal step was taken and the simulation is now paused.
                   (spacebar was pressed)
                5: A normal step was taken and the maximum time has been 
                   reached. The simulation will now end.
                -1: No step was taken because the simulation recieved a 
                    terminate command from the user (esc was pressed)
                -2: Something went wrong.
              
            We want to keep track of these because if the simulation is reset
            we will need to empty out all of the simulation data that we have
            already collected, and if the simulation is paused, we will need to 
            make sure we aren't collecting paused data.
            '''
            # Take a single physics step of dt seconds. This will
            # automatically update the physics and the visualizer, and attempts
            # to run in real time.
            val = self.s.step(max_time=max_time)
            
            '''
            Here we make sure we arent't collecting data from when the
            simulation is paused
            '''
            # Handle removing the last data point if the simulation is pasued
            # by the user pressing the spacebar key. 
            if val == 1 or val == 2:
                for key in data:
                    data[key].pop()
            
            '''
            And here we are resetting the data if we reset the simulation.
            '''
            # Handle resetting the data collection if the simulation is reset
            # by the user pressing the backspace key. 
            if val == 3:
                data = {'angle' : [],
                        'angle_vel' : [],
                        'KE' : [],
                        'PE' : [],
                        'time' : [],}
        
        '''
        Once the simulation loop is complete, we collected the terminal state
        and then plot all the data we collected. Finally, we return the data
        we collected so that a user may do whatever post processing they want
        to it.
        '''
        # Collect the very last data point 
        data = self.add_data_point(data)
        
        # At the end of the simulation, plot and return the data
        self.plot_data(data)
        return data
    
    '''
    This function is what we use to collect data points during the simulation.
    Essentially what it does is collect joint and link state data, calculate
    energies, and then append these data to our data structure. 
    '''
    def add_data_point(self, data):
        '''
        Adds a new data point to the data dictionary

        Parameters
        ----------
        data : Dictionary of lists
            The dictionary to which we are adding a new data point.

        Returns
        -------
        data : Dictionary of lists
            The dictionary with the added data point.

        '''
        
        '''
        We have already discussed how to collected joint state data in previous
        tutorials.
        '''
        # Use a sensor to collect the state of the joint
        state = self.s.get_joint_state(urdf_obj=self.pendulum,
                                       joint_name='chassis_to_arm')

        # Extract angle and angular velo of the joints
        angle = state['position'] * 57.29577951308232
        angle_vel = state['velocity'] * 57.29577951308232
        
        '''
        To collect the state of a specific link in a URDF object we call
        condynsate.simulator.get_link_state(). This works for either the 
        base link or any children links and takes the following arguments:
            urdf_obj : URDF_Obj
                A URDF_Obj whose state is being measured.
            link_name : string
                The name of the link whose state is measured. The link name is
                specified in the .urdf file.
            body_coords : bool
                A boolean flag that indicates whether the passed velocities are
                in world coords or body coords.
        
        It then returns:
            state : a dictionary with the following keys:
                'position' : array-like, shape (3,)
                    The (x,y,z) world coordinates of the link.
                'roll' : float
                    The Euler angle roll of the link
                    that defines the link's orientation in the world. Rotation
                    of the link about the world's x-axis.
                'pitch' : float
                    The Euler angle pitch of the link
                    that defines the link's orientation in the world. Rotation
                    of the link about the world's y-axis.
                'yaw' : float
                    The Euler angle yaw of the link
                    that defines the link's orientation in the world. Rotation
                    of the link about the world's z-axis.
                'R of world in link' : array-like, shape(3,3):
                    The rotation matrix that takes vectors in world coordinates 
                    to link coordinates. For example, let V_inL be a 3vector
                    written in link coordinates. Let V_inW be a 3vector 
                    written in world coordinates. Then:
                    V_inL = R_ofWorld_inLink @ V_inW
                'velocity' : array-like, shape (3,)
                    The (x,y,z) linear velocity in world coordinates of the
                    link.
                'angular velocity' : array-like, shape (3,)
                    The (x,y,z) angular velocity in world coordinates of the
                    link.
        '''
        # Retrieve the state of the mass at the end of the rod
        state = self.s.get_link_state(urdf_obj=self.pendulum,
                                      link_name='mass',
                                      body_coords=True)
        
        '''
        To collect the mass of a specific link in a URDF object we call
        condynsate.simulator.get_link_state(). This works for either the 
        base link or any children links and takes the following arguments:
            urdf_obj : URDF_Obj
                A URDF_Obj that contains that link whose mass is measured.
            link_name : string
                The name of the link whose mass is measured. The link name is
                specified in the .urdf file.
                
        It then returns:
            mass : float
                The mass of the link in Kg. If link is not found, returns none.
        '''
        # Get the mass of the mass
        mass = self.s.get_link_mass(urdf_obj=self.pendulum,
                                    link_name='mass')
            
        '''
        Finally, we use the collected link and joint data to add a single data
        point to our data structure. This is done by simply appending the 
        calculated data to the end of its respective list.
        '''
        # Calculate the energies of the mass
        height = state['position'][2]
        vel = state['velocity']
        KE = 0.5*mass*vel.T@vel
        PE = mass*9.81*height
        
        # Append all of the data calculated at this time step to the list
        data['angle'].append(angle)
        data['angle_vel'].append(angle_vel)
        data['KE'].append(KE)
        data['PE'].append(PE)
        data['time'].append(self.s.time) # This is how we get the current 
                                         # simulation time
        
        # Return the new data list
        return data
    
    '''
    The specifics of this function are outside the scope of a condynsate
    tutorial.
    '''
    def plot_data(self, data):
        '''
        Generates a pretty plot of the simulation data. The plot is saved as 
        a png named 04_image to the folder in which this tutorial is located

        Parameters
        ----------
        data : Dictionary of lists
            The dictionary that contains the data we are plotting

        Returns
        -------
        None.

        '''
        # Make the plot and subplots
        fig, (ax1, ax2) = plt.subplots(nrows=2, ncols=1)
        
        # Plot the phase space
        ax1.plot(data['angle'], data['angle_vel'], c='k', lw=2.5)
        ax1.set_xlabel('Angle [Deg]')
        ax1.set_ylabel('Angle Rate [Deg / Sec]')
        
        # Plot the energy
        ax2.plot(data['time'], data['KE'], label='KE', c='m', lw=2.5)
        ax2.plot(data['time'], data['PE'], label='PE', c='c', lw=2.5)
        total_E = np.array(data['KE']) + np.array(data['PE'])
        ax2.plot(data['time'], total_E, label='Total', c='k', lw=2.5, ls=':')
        ax2.legend(fancybox=True, shadow=True)
        ax2.set_xlabel('Time [Sec]')
        ax2.set_ylabel('Energy [J]')
        
        # Figure settings
        fig.tight_layout()
        
        # Save and close
        fig.savefig("./04_image.png", dpi=240)
        plt.close()
        
'''
This main loop is an example of how to use the Project class we just created to
run a complete simulation of time 5 seconds.
'''
if __name__ == "__main__":
    # Create an instance of the Project class. 
    proj = Project()
    
    # Run the simulation.
    data = proj.run(max_time = 5.0)
    
'''
In this tutorial, you learned how to extract link state and mass data as well
as a standard approach for storing simulation data during the simulation. You 
also learned how to read the return values of condynsate.simulator.step() and
what each of the return values mean. You then updated how data was collected 
based on this behavior. For an added challenge, try creating a similar project
but using the included double pendulum model. 
'''
    