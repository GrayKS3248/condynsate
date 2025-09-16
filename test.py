from condynsate import Simulator as conSim
from condynsate import __assets__ as assets
import numpy as np
from time import time

def read_states(simulator, cart):
    # Define which joints to read
    joint_names = ('chassis_to_wheel_1',
                   'chassis_to_wheel_2',
                   'chassis_to_wheel_3',
                   'chassis_to_wheel_4',
                   'chassis_to_arm',)

    # Loop over each joint and read the state extracting angle and angular vel.
    angles = []
    angular_vels = []
    for joint_name in joint_names:
        state = simulator.get_joint_state(urdf_obj = cart,
                                          joint_name = joint_name)
        angles.append(state['position'])
        angular_vels.append(state['velocity'])
    
    # For the wheels, return the mean angle and angular vel.
    mean_wheel_angle = np.mean(angles[:-1])
    mean_wheel_ang_vel = np.mean(angular_vels[:-1])

    # Additionally, calculate the wheel deviation from mean.
    wheel_deviations = []
    for wheel_angle in angles[:-1]:
        wheel_deviations.append(wheel_angle - mean_wheel_angle)
    
    return (mean_wheel_angle, mean_wheel_ang_vel), (angles[-1], angular_vels[-1]), wheel_deviations

def apply_torque(simulator, cart, mean_wheel_state, pendulum_state):
    # Calculate the torque via magic
    K = np.array([[ -2. ,  -0.1, -10. ,  -0.1]])
    m_e = np.zeros((4,1))
    n_e = np.zeros((1,1))
    m = np.array([[pendulum_state[1]],
                  [mean_wheel_state[1]],
                  [pendulum_state[0]],
                  [mean_wheel_state[0]]])
    torque = (-K@(m - m_e) + n_e).flatten()[0]
    torque = np.clip(torque, -0.75, 0.75)
    
    # Define which joints we apply torque to
    joint_names = ('chassis_to_wheel_1',
                   'chassis_to_wheel_2',
                   'chassis_to_wheel_3',
                   'chassis_to_wheel_4',)

    # Apply the torque we calculated
    for joint_name in joint_names:
        simulator.set_joint_torque(urdf_obj = cart,
                                   joint_name = joint_name,
                                   torque = torque,
                                   show_arrow = True, # Define arrow params to visualize the torque
                                   arrow_scale = 0.25,
                                   arrow_offset = 0.025)

    return torque

def add_torque_point(simulator, plot_id, torque):
    simulator.add_line_datum(plot_id = plot_id,
                             x = simulator.time,
                             y = torque)

def add_state_points(simulator, plot, plot_artists, mean_wheel_state, pendulum_state):
    simulator.add_line_datum(plot_id = plot,
                             artist_id = plot_artists[0], # Draw to the first artist (whose label is "Pendulum")
                             x = simulator.time,
                             y = pendulum_state[0])
    simulator.add_line_datum(plot_id = plot,
                             artist_id = plot_artists[1], # Draw to the second artist (whose label is "Wheel")
                             x = simulator.time,           
                             y = mean_wheel_state[0])
    
def set_deviation_values(simulator, plot, plot_artists, wheel_deviations):
    for plot_artist, wheel_deviation in zip(plot_artists, wheel_deviations):
        simulator.set_bar_value(plot_id = plot,
                                artist_id = plot_artist,
                                value = wheel_deviation)

if __name__ == "__main__":
    simulator = conSim(keyboard = False,
                       visualization = True,
                       visualization_fr = 24.0,
                       animation = True,
                       animation_fr = 8.0)
    
    # Load the ground
    ground = simulator.load_urdf(urdf_path = assets['plane_big'],
                                 fixed = True,
                                 update_vis = False)
    
    # Load the walls
    left_wall = simulator.load_urdf(urdf_path = assets['plane_big'],
                                    tex_path = assets['concrete_img'], # Choose a default texture for appearance
                                    position = [0., -5., 0.],
                                    roll = -1.5707,
                                    fixed = True,
                                    update_vis = False)
    right_wall = simulator.load_urdf(urdf_path = assets['plane_big'],
                                     tex_path = assets['concrete_img'],  # Choose a default texture for appearance
                                     position = [0., 5., 0.],
                                     roll = 1.5707,
                                     fixed = True,
                                     update_vis = False)
    
    cart = simulator.load_urdf(urdf_path = assets['cart'],
                               position = [0., 0., 0.25],
                               yaw = 1.5707,
                               fixed = False,
                               update_vis = True)
    
    # Set joint damping
    joint_names = ('chassis_to_wheel_1',
                   'chassis_to_wheel_2',
                   'chassis_to_wheel_3',
                   'chassis_to_wheel_4',
                   'chassis_to_arm',)
    for joint_name in joint_names:
        simulator.set_joint_damping(urdf_obj = cart,
                                   joint_name = joint_name,
                                   damping = 0.01)
        
    initial_angle = 0.275
    simulator.set_joint_position(urdf_obj = cart,
                                 joint_name = 'chassis_to_arm',
                                 position = initial_angle, # For a rotational joint, this value is in radians
                                 initial_cond = True,
                                 physics = False)
    
    plot_1 = simulator.add_plot(plot_type = 'line',
                                title = "Torque vs Time",
                                x_label = "Time [Seconds]",
                                y_label = "Torque [Nm]",
                                line_width = 2.5,
                                y_lim = [-0.80, 0.80],
                                h_zero_line = True)
    
    plot_2, plot_2_artists = simulator.add_plot(n_artists = 2,
                                                plot_type = 'line',
                                                title = "Angles vs Time",
                                                x_label = "Time [Seconds]",
                                                y_label = "Angles [Rad]",
                                                color = ["m", "c"], # One color for each artist
                                                line_width = 2.5,   # Single value to give all artists same line_width
                                                line_style = "-",   # Single value to give all artists same line_style
                                                label = ["Pendulum", "Wheel"], # One label for each artist
                                                h_zero_line = True)
    
    plot_3, plot_3_artists = simulator.add_plot(n_artists = 4,
                                                plot_type = 'bar',
                                                x_lim = [-0.4, 0.4],
                                                title = "Wheel Deviation from Mean",
                                                x_label = "Deviation [Rad]",
                                                y_label = "Wheel Number",
                                                color = "b",
                                                label = ["W1", "W2", "W3", "W4"],
                                                v_zero_line = True)
    
    strt = time()
    simulator.start_animator()
    
    # Reset before running a simulation loop
    simulator.reset()
        
    # Run the simulation loop
    while(not simulator.is_done):
        # Step the sim
        ret_code = simulator.step(real_time = True,
                                  max_time = 10.0)
    
        # If successful step was taken
        if ret_code > 0:
            # Read the states and apply the torque
            mean_wheel_state, pendulum_state, wheel_deviations = read_states(simulator, cart)
            torque = apply_torque(simulator, cart, mean_wheel_state, pendulum_state)
    
            # Add the new data points to the animator
            add_torque_point(simulator, plot_1, torque)
            add_state_points(simulator, plot_2, plot_2_artists, mean_wheel_state, pendulum_state)
            set_deviation_values(simulator, plot_3, plot_3_artists, wheel_deviations)
            
    simulator.terminate_animator()
    print(time() - strt)
        