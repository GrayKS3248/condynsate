from condynsate.simulator import Simulator as conSim
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt

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
        # Use a sensor to collect the state of the joint
        state = self.s.get_joint_state(urdf_obj=self.pendulum,
                                       joint_name='chassis_to_arm')

        # Extract angle and angular velo of the joints
        angle = state['position'] * 57.29577951308232
        angle_vel = state['velocity'] * 57.29577951308232
        
        # Retrieve the state of the mass at the end of the rod
        state = self.s.get_link_state(urdf_obj=self.pendulum,
                                      link_name='mass',
                                      body_coords=True)
        
        # Get the mass of the mass
        mass = self.s.get_link_mass(urdf_obj=self.pendulum,
                                    link_name='mass')
            
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
        data['time'].append(self.s.time)
        
        # Return the new data list
        return data

    def plot_data(self, data):
        '''
        Generates a pretty plot of the simulation data

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
            # Add the simulation data at the current time
            data = self.add_data_point(data)
            
            # Take a single physics step of dt seconds. This will
            # automatically update the physics and the visualizer, and attempts
            # to run in real time.
            val = self.s.step(max_time=max_time)
            
            # Handle removing the last data point if the simulation is pasued
            # by the user pressing the spacebar key. 
            if val == 2:
                for key in data:
                    data[key].pop()
            
            # Handle resetting the data collection if the simulation is reset
            # by the user pressing the backspace key. 
            if val == 3:
                data = {'angle' : [],
                        'angle_vel' : [],
                        'KE' : [],
                        'PE' : [],
                        'time' : [],}
        
        # Collect the very last data point 
        data = self.add_data_point(data)
        
        # At the end of the simulation, plot and return the data
        self.plot_data(data)
        return data
 
if __name__ == "__main__":
    # Create an instance of the Project class. 
    proj = Project()
    
    # Run the simulation.
    data = proj.run(max_time = 5.0)
    
