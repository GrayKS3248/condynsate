###############################################################################
#DEPENDENCIES
###############################################################################
import condynsate
import numpy as np
import os
import control
from sympy import Symbol, Matrix, Function, Derivative
from sympy import diff, sin, cos, solve


###############################################################################
#DERIVE THE EQUATIONS OF MOTION
###############################################################################
# Constants of the system
mc = 0.1
Ic = np.diag([0.0022, 0.0022, 0.0042])
mr = 0.01
Ic = np.diag([0.00036, 0.00036, 0.00070])
l = 1.0
h = 0.0875
g = 9.81
kf = 0.0032


# Time is a symbol (variable)
t = Symbol('t')

# The generalized coordinates and the input torque are both functions of time.
# This means that they are initialized as Functions.
x = Function('x')
y = Function('y')
z = Function('z')
roll = Function('roll')
pitch = Function('pitch')
yaw = Function('yaw')
phi1 = Function('phi1')
phi2 = Function('phi2')
phi3 = Function('phi3')
phi4 = Function('phi4')
t1 = Function('t1')
t2 = Function('t2')
t3 = Function('t3')
t4 = Function('t4')

# Get the energies of the core
pos = Matrix([ x(t), y(t), z(t) ])
vel = diff(pos, t)
ori = Matrix([ roll(t), pitch(t), yaw(t) ])
ang_vel = diff(ori, t)
core_KE = 0.5 * mc * (vel.T @ vel)[0,0]
core_RE = 0.5 * (ang_vel.T @ Ic @ ang_vel)[0,0]
core_PE = mc * g * z(t)

# Get the energies of the rotors
pos_1 = Matrix([ x(t),   y(t)+l, z(t)+h ])
pos_2 = Matrix([ x(t)+l, y(t),   z(t)+h ])
pos_3 = Matrix([ x(t),   y(t)-l, z(t)+h ])
pos_4 = Matrix([ x(t)-l, y(t),   z(t)+h ])
vel_1 = diff(pos_1, t)
vel_2 = diff(pos_2, t)
vel_3 = diff(pos_3, t)
vel_4 = diff(pos_4, t)


###############################################################################
#BUILD A CONTROLLER
###############################################################################
def manual_controller(**kwargs):
    # Get the simulator
    sim = kwargs['sim']
    
    # Set the torques to min
    torques = {'torque1' : 0.,
               'torque2' : 0.,
               'torque3' : 0.,
               'torque4' : 0.}
    
    # Listen for keyboard presses:
    if sim.is_pressed('a'):
        torques['torque1'] = 1.
    if sim.is_pressed('s'):
        torques['torque2'] = 1.
    if sim.is_pressed('d'):
        torques['torque3'] = 1.
    if sim.is_pressed('f'):
        torques['torque4'] = 1.

    # Return the manually set torques
    return torques


###############################################################################
#POSITION IN COURSE TRACKING
###############################################################################
class course:
    def __init__(self, sim, quadrotor, *args):
        # Set simultion env and quadrotor object
        self.sim = sim
        self.quadrotor = quadrotor
        
        # Collect gate data
        self.gates = [gate for gate in args]
        self.gate_data = self.get_gate_data()
        
        # Variables used to track progress through course
        self.active_gate = 0
        self.curr_side = self.get_quadrotor_side()
        self.curr_in_channel = self.quadrotor_in_channel()
        
        # Choose the colors for the gates
        self.active_col = [0, 0, 255]
        self.active_outofline_col = [130, 0, 0]
        self.inactive_col = [245, 130, 30]
    
    
    def get_gate_data(self, gate_scale=8.):
        # Go through each gate and get position, roll, and bounding box
        gate_data = []
        for gate in self.gates:
            
            # Extract gate height data
            stl_pth = self.sim._get_urdf_vis_dat(gate)[0][0]
            stl_name = os.path.basename(stl_pth)
            if stl_name == 'gate_short.stl':
                gate_height=0.5
            elif stl_name == 'gate_med.stl':
                gate_height=1.5
            elif stl_name == 'gate_tall.stl':
                gate_height=2.0
            else:
                return -1
        
            # Get the position and orientation of the gate
            state = self.sim.get_base_state(gate)
            pos = state['position']
            roll = np.pi * state['roll'] / 180.
            
            # Calculate the bounding box of the pass through zone
            bbox_1 = np.array([-abs(np.cos(roll)*0.375),
                               -abs(np.sin(roll)*0.375),
                               -0.375+gate_height]) * gate_scale + pos
            bbox_2 = np.array([abs(np.cos(roll)*0.375),
                               abs(np.sin(roll)*0.375),
                               0.375+gate_height]) * gate_scale + pos
            
            # Calculate the center of the gate
            cen = 0.5*(bbox_1 + bbox_2)
            
            # Calculate the direction the gate is pointing in
            norm = np.array([np.sin(roll),
                             np.cos(roll),
                             0.0])
            
            # Package and return the gate data
            gate_datum = {'bbox' : {'BL' : bbox_1,
                                    'TR' : bbox_2},
                          'center' : cen,
                          'norm' : norm}
            gate_data.append(gate_datum)
            
        # Return complete gate data collection
        return gate_data
    
    
    def get_quadrotor_com(self):
        # Get the quadrotor's center of mass
        quad_state = self.sim.get_base_state(self.quadrotor)
        quad_com = quad_state['position']
        return quad_com
    
    
    def get_quadrotor_side(self):
        # Retrieve data from active gate
        active_gate_data = self.gate_data[self.active_gate]
        gate_center = active_gate_data['center']
        gate_norm = active_gate_data['norm']
        
        # Get the quadrotor's center of mass
        quad_com = self.get_quadrotor_com()
        
        # Calculate relative direction of gate center to quadrotor center
        gate_to_quad = quad_com - gate_center
        gate_to_quad_dirn = gate_to_quad / np.linalg.norm(gate_to_quad)
        
        # Determine which side the quadrotor is on
        # Negative is behind, positive is ahead, 0 is perfectly in gate plane
        gate_side = np.sign(np.dot(gate_to_quad_dirn, gate_norm))
        
        # Return which side of the gate the quadrotor is on
        return gate_side
    
    
    def quadrotor_in_channel(self):
        # Retrieve data from active gate
        active_gate_data = self.gate_data[self.active_gate]
        gate_center = active_gate_data['center']
        gate_norm = active_gate_data['norm']
        bl = active_gate_data['bbox']['BL']
        tr = active_gate_data['bbox']['TR']
        
        # Get the quadrotor's center of mass
        quad_com = self.get_quadrotor_com()
        
        # Project the quadrotor position onto the gate's plane
        gate_to_quad = quad_com - gate_center
        quad_on_gate_norm = np.dot(gate_to_quad, gate_norm) * gate_norm
        quad_on_gate_plane = gate_to_quad - quad_on_gate_norm + gate_center
        
        # Check if the projected quadrotor position is within the bbox of the
        # gate. If it is, the quadrotor is 'in channel;
        in_x = quad_on_gate_plane[0]>=bl[0] and quad_on_gate_plane[0]<=tr[0]
        in_y = quad_on_gate_plane[1]>=bl[1] and quad_on_gate_plane[1]<=tr[1]
        in_z = quad_on_gate_plane[2]>=bl[2] and quad_on_gate_plane[2]<=tr[2]
        in_channel = in_x and in_y and in_z
        
        # Return whether the quadrotor is in the gate channel
        return in_channel
    
    
    def check_if_pass(self):
        # Check to see if the quadrotor passed through the active gate plane
        next_side = self.get_quadrotor_side()
        passed_gate_plane = (self.curr_side < 0 and next_side >= 0)
        
        # Check to see if the quadrotor stayed in the gate channel
        next_in_channel = self.quadrotor_in_channel()
        stayed_in_channel = self.curr_in_channel and next_in_channel
        
        # Check gate pass condition
        if passed_gate_plane and stayed_in_channel:
            # Iterate the active gate
            self.active_gate = self.active_gate + 1
            if self.active_gate >= len(self.gates):
                self.active_gate = 0
                
            # Set the current gate side and channel value
            self.curr_side = self.get_quadrotor_side()
            self.curr_in_channel = self.quadrotor_in_channel()
        
        # If the quadrotor did not pass
        else:
            self.curr_side = next_side
            self.curr_in_channel = next_in_channel
            
        # Color the gates
        self.color_gates()
    

    def color_gates(self):
        # Select color 
        for i in range(len(self.gates)):
            color = self.inactive_col
            if i == self.active_gate:
                if self.curr_in_channel:
                    color = self.active_col
                else: 
                    color = self.active_outofline_col
                
            # Apply the selected color
            gate = self.gates[i]
            self.sim.set_link_color(urdf_obj = gate,
                                    link_name = 'gate',
                                    color=color)


###############################################################################
#BUILD THE SIMULATOR ENVIRONMENT
###############################################################################
# Create an instance of the simulator with visualization
sim = condynsate.Simulator(visualization=True,
                           animation=True,
                           animation_fr=5.)

# Load the ground
ground_obj = sim.load_urdf(urdf_path='./quadcopter_vis/plane.urdf',
                           position=[0., 0., -0.051],
                           fixed=True,
                           update_vis=False)

# Load the quadrotor
quad_obj = sim.load_urdf(urdf_path='./quadcopter_vis/quadcopter.urdf',
                         tex_path='./quadcopter_vis/directions.png',
                         fixed=False,
                         update_vis=True)

# Load the course
gate_1 = sim.load_urdf(urdf_path='./quadcopter_vis/gate_short.urdf',
                        position=[0., 20., 0.],
                        fixed=True,
                        update_vis=False)
gate_2 = sim.load_urdf(urdf_path='./quadcopter_vis/gate_short.urdf',
                        position=[-13., 35., 0.],
                        yaw=np.pi/3,
                        fixed=True,
                        update_vis=False)
gate_3 = sim.load_urdf(urdf_path='./quadcopter_vis/gate_med.urdf',
                        position=[-30., 35., 0.],
                        yaw=np.pi/2,
                        fixed=True,
                        update_vis=False)
gate_4 = sim.load_urdf(urdf_path='./quadcopter_vis/gate_tall.urdf',
                        position=[-16., 0., 0.],
                        yaw=3*np.pi/2,
                        fixed=True,
                        update_vis=False)
gate_5 = sim.load_urdf(urdf_path='./quadcopter_vis/gate_tall.urdf',
                        position=[16., 0., 0.],
                        yaw=3*np.pi/2,
                        fixed=True,
                        update_vis=False)
gate_6 = sim.load_urdf(urdf_path='./quadcopter_vis/gate_med.urdf',
                        position=[10., -20., 0.],
                        yaw=2*np.pi/3,
                        fixed=True,
                        update_vis=False)
gate_7 = sim.load_urdf(urdf_path='./quadcopter_vis/gate_short.urdf',
                        position=[0., -40., 0.],
                        fixed=True,
                        update_vis=False)

# Create an object to track position in course
course = course(sim, quad_obj, 
                gate_1, gate_2, gate_3, gate_4, gate_5, gate_6, gate_7)

# Apply damping to rotors
sim.set_joint_damping(urdf_obj=quad_obj,
                      joint_name='spar1_to_rotor1',
                      damping=0.003)
sim.set_joint_damping(urdf_obj=quad_obj,
                      joint_name='spar2_to_rotor2',
                      damping=0.003)
sim.set_joint_damping(urdf_obj=quad_obj,
                      joint_name='spar3_to_rotor3',
                      damping=0.003)
sim.set_joint_damping(urdf_obj=quad_obj,
                      joint_name='spar4_to_rotor4',
                      damping=0.003)

# Make plot for ground track
plot1, artists1 = sim.add_subplot(n_artists=3,
                                  subplot_type='line',
                                  title="Body Lin Vel",
                                  x_label="Time [s]",
                                  y_label="Lin Vel [m/s]",
                                  x_lim=[0.0, None],
                                  y_lim=[-20, 20],
                                  colors=["r", "g", "b"],
                                  line_widths=[2.5, 2.5, 2.5],
                                  line_styles=["-", "-", "-"],
                                  labels=['vx', 'vy', 'vz'])
plot2, artists2 = sim.add_subplot(n_artists=3,
                                  subplot_type='line',
                                  title="Body Position",
                                  x_label="Time [s]",
                                  y_label="Pos [m]",
                                  x_lim=[0.0, None],
                                  y_lim=[-10, 10],
                                  colors=["r", "g", "b"],
                                  line_widths=[2.5, 2.5, 2.5],
                                  line_styles=["-", "-", "-"],
                                  labels=['x', 'y', 'z'])
plot3, artists3 = sim.add_subplot(n_artists=3,
                                  subplot_type='line',
                                  title="Body Ang Vel",
                                  x_label="Time [s]",
                                  y_label="Ang Vel [rad/s]",
                                  x_lim=[0.0, None],
                                  y_lim=[-10, 10],
                                  colors=["r", "g", "b"],
                                  line_widths=[2.5, 2.5, 2.5],
                                  line_styles=["-", "-", "-"],
                                  labels=['wx', 'wy', 'wz'])
plot4, artists4 = sim.add_subplot(n_artists=3,
                                  subplot_type='line',
                                  title="Body Orientation",
                                  x_label="Time [s]",
                                  y_label="Ori [rad]",
                                  x_lim=[0.0, None],
                                  y_lim=[-5, 5],
                                  colors=["r", "g", "b"],
                                  line_widths=[2.5, 2.5, 2.5],
                                  line_styles=["-", "-", "-"],
                                  labels=['roll', 'pitch', 'yaw'])
plot5, artists5 = sim.add_subplot(n_artists=4,
                                  subplot_type='line',
                                  title="Rotor Speed",
                                  x_label="Time [s]",
                                  y_label="Speed [rad/s]",
                                  x_lim=[0.0, None],
                                  y_lim=[0, 350],
                                  colors=["r", "g", "b", "m"],
                                  line_widths=[2.5, 2.5, 2.5, 2.5],
                                  line_styles=["-", "-", "-", "-"],
                                  labels=['r1', 'r2', 'r3', 'r4'])

###############################################################################
#SIMULATION LOOP
###############################################################################
# Run the simulation
sim.open_animator_gui()
sim.await_keypress(key="enter")
while(not sim.is_done):
    ###########################################################################
    # SENSOR
    # Use a sensor to collect the rotor speeds
    rotor1 = sim.get_joint_state(urdf_obj=quad_obj,
                                 joint_name="spar1_to_rotor1")
    rotor2 = sim.get_joint_state(urdf_obj=quad_obj,
                                 joint_name="spar2_to_rotor2")
    rotor3 = sim.get_joint_state(urdf_obj=quad_obj,
                                 joint_name="spar3_to_rotor3")
    rotor4 = sim.get_joint_state(urdf_obj=quad_obj,
                                 joint_name="spar4_to_rotor4")
    rotor1_vel = rotor1['velocity']
    rotor2_vel = rotor2['velocity']
    rotor3_vel = rotor3['velocity']
    rotor4_vel = rotor4['velocity']
    
    # Use a sensor to get the position and velocity of the quadrotor's CoM
    state = sim.get_base_state(quad_obj,
                               body_coords=True)
    com_pos = state['position']
    com_vel = state['velocity']
    ang_vel = state['angular velocity']
    roll = state['roll']
    pitch = state['pitch']
    yaw = state['yaw']
    
    ###########################################################################
    # CONTROLLER
    torques = manual_controller(sim=sim)
    torque1 = torques['torque1']
    torque2 = torques['torque2']
    torque3 = torques['torque3']
    torque4 = torques['torque4']
    
    ###########################################################################
    # ACTUATOR
    # Apply torque to each rotor
    sim.set_joint_torque(urdf_obj=quad_obj,
                        joint_name='spar1_to_rotor1',
                        torque=torque1,
                        show_arrow=True,
                        arrow_offset=-0.1)
    sim.set_joint_torque(urdf_obj=quad_obj,
                        joint_name='spar2_to_rotor2',
                        torque=torque2,
                        show_arrow=True,
                        arrow_offset=0.1)
    sim.set_joint_torque(urdf_obj=quad_obj,
                        joint_name='spar3_to_rotor3',
                        torque=torque3,
                        show_arrow=True,
                        arrow_offset=-0.1)
    sim.set_joint_torque(urdf_obj=quad_obj,
                        joint_name='spar4_to_rotor4',
                        torque=torque4,
                        show_arrow=True,
                        arrow_offset=0.1)
    
    ###########################################################################
    # UPDATE THE PLOTS
    # Get the momentum and the derivative of the momentum
    sim.add_subplot_point(subplot_index=plot1,
                          artist_index=artists1[0],
                          x=sim.time,
                          y=com_vel[0])
    sim.add_subplot_point(subplot_index=plot1,
                          artist_index=artists1[1],
                          x=sim.time,
                          y=com_vel[1])
    sim.add_subplot_point(subplot_index=plot1,
                          artist_index=artists1[2],
                          x=sim.time,
                          y=com_vel[2])
    sim.add_subplot_point(subplot_index=plot2,
                          artist_index=artists2[0],
                          x=sim.time,
                          y=com_pos[0])
    sim.add_subplot_point(subplot_index=plot2,
                          artist_index=artists2[1],
                          x=sim.time,
                          y=com_pos[1])
    sim.add_subplot_point(subplot_index=plot2,
                          artist_index=artists2[2],
                          x=sim.time,
                          y=com_pos[2])
    sim.add_subplot_point(subplot_index=plot3,
                          artist_index=artists3[0],
                          x=sim.time,
                          y=ang_vel[0])
    sim.add_subplot_point(subplot_index=plot3,
                          artist_index=artists3[1],
                          x=sim.time,
                          y=ang_vel[1])
    sim.add_subplot_point(subplot_index=plot3,
                          artist_index=artists3[2],
                          x=sim.time,
                          y=ang_vel[2])
    sim.add_subplot_point(subplot_index=plot4,
                          artist_index=artists4[0],
                          x=sim.time,
                          y=roll)
    sim.add_subplot_point(subplot_index=plot4,
                          artist_index=artists4[1],
                          x=sim.time,
                          y=pitch)
    sim.add_subplot_point(subplot_index=plot4,
                          artist_index=artists4[2],
                          x=sim.time,
                          y=yaw)
    sim.add_subplot_point(subplot_index=plot5,
                          artist_index=artists5[0],
                          x=sim.time,
                          y=rotor1_vel)
    sim.add_subplot_point(subplot_index=plot5,
                          artist_index=artists5[1],
                          x=sim.time,
                          y=rotor2_vel)
    sim.add_subplot_point(subplot_index=plot5,
                          artist_index=artists5[2],
                          x=sim.time,
                          y=rotor3_vel)
    sim.add_subplot_point(subplot_index=plot5,
                          artist_index=artists5[3],
                          x=sim.time,
                          y=rotor4_vel)
    
    
    ###########################################################################
    # ROTOR PHYSICS
    # Set rotor force based on velocity
    sim.apply_force_to_link(urdf_obj=quad_obj,
                            link_name='rotor1',
                            force=[0., 0., 0.0032*rotor1_vel],
                            show_arrow=True,
                            arrow_scale=0.40)
    sim.apply_force_to_link(urdf_obj=quad_obj,
                            link_name='rotor2',
                            force=[0., 0., 0.0032*rotor2_vel],
                            show_arrow=True,
                            arrow_scale=0.40)
    sim.apply_force_to_link(urdf_obj=quad_obj,
                            link_name='rotor3',
                            force=[0., 0., 0.0032*rotor3_vel],
                            show_arrow=True,
                            arrow_scale=0.40)
    sim.apply_force_to_link(urdf_obj=quad_obj,
                            link_name='rotor4',
                            force=[0., 0., 0.0032*rotor4_vel],
                            show_arrow=True,
                            arrow_scale=0.40)
    
    ###########################################################################
    # STEP THE SIMULATION
    course.check_if_pass()
    sim.step(real_time=True,
              update_vis=True,
              update_ani=True)
            