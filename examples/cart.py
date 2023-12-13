###############################################################################
#DEPENDENCIES
###############################################################################
import numpy as np
import condynsate
import control
from sympy import Symbol, Matrix, Function, Derivative
from sympy import diff, simplify, sin, cos, solve


###############################################################################
#BUILD A CONTROLLER
###############################################################################
# Parameters
mp = 1.0
mc = 6.0
mw = 1.0
l = 1.0
r = 0.25
g = 9.81

# Equilibrium
omega_theta_e = 0.0
omega_phi_e = 0.0
theta_e = 0.0
phi_e = 0.0
tau_e = 0.0

# State symbols
t = Symbol('t')
omega_theta = Symbol('omega_theta')
omega_phi = Symbol('omega_phi')
theta = Symbol('theta')
phi = Symbol('phi')
tau = Symbol('tau')

# Functions of time
theta_t = Function('theta_t')
phi_t = Function('phi_t')
tau_t = Function('tau_t')

# Energy of particle
p_pos = Matrix([-phi_t(t)*r-sin(theta_t(t))*l,
                0.0,
                cos(theta_t(t))*l])
p_vel = diff(p_pos,t)

p_KE = 0.5 * mp * (p_vel.T @ p_vel)[0,0]
p_PE = mp*g*l*cos(theta_t(t))

# Energy of cart
c_pos = Matrix([-phi_t(t)*r,
                0.0,
                0.0])
c_vel = diff(c_pos,t)
c_KE = 0.5 * mc * (c_vel.T @ c_vel)[0,0]

# Energy of wheels
w_pos = Matrix([-phi_t(t)*r,
                0.0,
                0.0])
w_vel = diff(w_pos,t)
w_KE = 4.0 * (0.5 * mw * (w_vel.T @ w_vel)[0,0])
I_w = 0.5*mw*r**2
w_RE = 4.0*(I_w*diff(phi_t(t),t)**2)

# Lagrangian
L = (p_KE + c_KE + w_KE + w_RE - p_PE)

# Lagrange's equations
eq1 = diff(diff(L, Derivative(theta_t(t), t)), t) - diff(L, theta_t(t))
eq2 = diff(diff(L, Derivative(phi_t(t), t)), t) - diff(L, phi_t(t)) - tau_t(t)
eq1 = simplify(eq1)
eq2 = simplify(eq2)
soln = solve([eq1, eq2],
             Derivative(theta_t(t), (t, 2)),
             Derivative(phi_t(t), (t, 2)))

# Place in standard form
d_omega_theta = soln[Derivative(theta_t(t), (t, 2))]
d_omega_theta = d_omega_theta.replace(Derivative(theta_t(t), t), omega_theta)
d_omega_theta = d_omega_theta.replace(Derivative(phi_t(t), t), omega_phi)
d_omega_theta = d_omega_theta.replace(theta_t(t), theta)
d_omega_theta = d_omega_theta.replace(phi_t(t), phi)
d_omega_theta = d_omega_theta.replace(tau_t(t), tau)
d_omega_phi = soln[Derivative(phi_t(t), (t, 2))]
d_omega_phi = d_omega_phi.replace(Derivative(theta_t(t), t), omega_theta)
d_omega_phi = d_omega_phi.replace(Derivative(phi_t(t), t), omega_phi)
d_omega_phi = d_omega_phi.replace(theta_t(t), theta)
d_omega_phi = d_omega_phi.replace(phi_t(t), phi)
d_omega_phi = d_omega_phi.replace(tau_t(t), tau)
d_omega_theta = simplify(d_omega_theta)
d_omega_phi = simplify(d_omega_phi)

# Make the system
sys = Matrix([d_omega_theta,
              d_omega_phi,
              omega_theta,
              omega_phi])
x = Matrix([omega_theta,
            omega_phi,
            theta,
            phi])
u = Matrix([tau])

# Get the A state matrix
A = sys.jacobian(x)
A = A.replace(omega_theta, omega_theta_e)
A = A.replace(omega_phi, omega_phi_e)
A = A.replace(theta, theta_e)
A = A.replace(phi, phi_e)
A = A.replace(tau, tau_e)
A = np.array(A).astype(float)

# Get the B state matrix
B = sys.jacobian(u)
B = B.replace(omega_theta, omega_theta_e)
B = B.replace(omega_phi, omega_phi_e)
B = B.replace(theta, theta_e)
B = B.replace(phi, phi_e)
B = B.replace(tau, tau_e)
B = np.array(B).astype(float)

# Define our optimal control matrices
Q = np.eye(4)
Q[0,0] = 1./((np.pi/0.1)**2)
Q[1,1] = 1./((np.pi/0.1)**2)
Q[2,2] = 1./((np.pi/0.1)**2)
Q[3,3] = 1./((np.pi/20.)**2)
R = np.eye(1)
R[0,0] = 1./((0.05)**2)

# Get the control gains
K, X, E = control.lqr(A, B, Q, R)

# Make the controller function
def controller(**kwargs):
    try:
        # Retrieve the arguments
        omega_theta = kwargs['arm_velocity']
        omega_phi = kwargs['wheel_velocity']
        theta = kwargs['arm_angle']
        phi = kwargs['wheel_angle']
        omega_theta_e = kwargs['arm_velocity_e']
        omega_phi_e = kwargs['wheel_velocity_e']
        theta_e = kwargs['arm_angle_e']
        phi_e = kwargs['wheel_angle_e']
        torque_e = kwargs['torque_e']
        
        # Make the state vector
        x = np.array([[omega_theta - omega_theta_e],
                      [omega_phi - omega_phi_e],
                      [theta - theta_e],
                      [phi - phi_e]])
        
        # Apply the control gains
        torque = (-K@x)[0,0] + torque_e
        return torque
        
    # If we can't get the torque, just return 0.0
    except:
        return 0.0
    
    
###############################################################################
#MAIN LOOP
###############################################################################
# Create an instance of the simulator with visualization
sim = condynsate.Simulator(visualization=True,
                           animation=True,
                           animation_fr=15.)

# Load all urdf objects
ground_obj = sim.load_urdf(urdf_path='./cart_vis/plane.urdf',
                            position=[0., 0., 0.],
                            wxyz_quaternion=[1., 0., 0., 0],
                            fixed=True,
                            update_vis=False)
left_wall_obj = sim.load_urdf(urdf_path='./cart_vis/plane.urdf',
                              tex_path='./cart_vis/concrete.png',
                              position=[0., -5., 0.],
                              roll=-np.pi/2.,
                              fixed=True,
                              update_vis=False)
right_wall_obj = sim.load_urdf(urdf_path='./cart_vis/plane.urdf',
                               tex_path='./cart_vis/concrete.png',
                               position=[0., 5., 0.],
                               roll=np.pi/2.,
                               fixed=True,
                               update_vis=False)
cart_obj = sim.load_urdf(urdf_path='./cart_vis/cart.urdf',
                            position=[0., 0., 0.25],
                            yaw=np.pi/2,
                            fixed=False,
                            update_vis=True)

# Make plot for states
plot1, artists1 = sim.add_subplot(n_artists=3,
                                  subplot_type='line',
                                  title="States vs Time",
                                  x_label="Time [Seconds]",
                                  y_label="Angles [Rad]",
                                  colors=["m", "c", "k"],
                                  line_widths=[2.5, 2.5, 2.5],
                                  line_styles=["-", "-", ":"],
                                  labels=["Pendulum", "Wheel", "Target Wheel"])
plot2, artists2 = sim.add_subplot(n_artists=1,
                                  subplot_type='line',
                                  title="Torque vs Time",
                                  x_label="Time [Seconds]",
                                  y_label="Torque [Nm]",
                                  colors=["r"],
                                  line_widths=[2.5],
                                  line_styles=["-"])

# Give the pendulum an initial angle of some non-zero value
sim.set_joint_position(urdf_obj=cart_obj,
                        joint_name='chassis_to_arm',
                        position=.1,
                        physics=False)

# Run the simulation
sim.open_animator_gui()
sim.await_keypress(key="enter")
target_wheel_angle = 0.0
while(not sim.is_done):
    
    # Get the states
    arm_state = sim.get_joint_state(urdf_obj=cart_obj,
                                    joint_name='chassis_to_arm')
    wheel1_state = sim.get_joint_state(urdf_obj=cart_obj,
                                       joint_name='chassis_to_wheel_1')
    wheel2_state = sim.get_joint_state(urdf_obj=cart_obj,
                                       joint_name='chassis_to_wheel_2')
    wheel3_state = sim.get_joint_state(urdf_obj=cart_obj,
                                       joint_name='chassis_to_wheel_3')
    wheel4_state = sim.get_joint_state(urdf_obj=cart_obj,
                                       joint_name='chassis_to_wheel_4')
    
    # Calculate the average wheel speed
    wheel_ang = 0.25*(wheel1_state['position']+
                      wheel2_state['position']+
                      wheel3_state['position']+
                      wheel4_state['position'])
    wheel_vel = 0.25*(wheel1_state['velocity']+
                      wheel2_state['velocity']+
                      wheel3_state['velocity']+
                      wheel4_state['velocity'])
    
    # Apply the controller
    torque = controller(arm_velocity = arm_state['velocity'],
                        wheel_velocity = wheel_vel,
                        arm_angle = arm_state['position'],
                        wheel_angle = wheel_ang,
                        arm_velocity_e = 0.0,
                        wheel_velocity_e = 0.0,
                        arm_angle_e = 0.0,
                        wheel_angle_e = target_wheel_angle,
                        torque_e = 0.0)
    
    # Limit the torque
    if torque > 2.0:
        torque = 2.0
    if torque < -2.0:
        torque = -2.0
    
    # Set wheel torques
    sim.set_joint_torque(urdf_obj=cart_obj,
                          joint_name='chassis_to_wheel_1',
                          torque=0.25*torque,
                          show_arrow=True,
                          arrow_scale=0.5,
                          arrow_offset=0.025)
    sim.set_joint_torque(urdf_obj=cart_obj,
                          joint_name='chassis_to_wheel_2',
                          torque=0.25*torque,
                          show_arrow=True,
                          arrow_scale=0.5,
                          arrow_offset=0.025)
    sim.set_joint_torque(urdf_obj=cart_obj,
                          joint_name='chassis_to_wheel_3',
                          torque=0.25*torque,
                          show_arrow=True,
                          arrow_scale=0.5,
                          arrow_offset=-0.025)
    sim.set_joint_torque(urdf_obj=cart_obj,
                          joint_name='chassis_to_wheel_4',
                          torque=0.25*torque,
                          show_arrow=True,
                          arrow_scale=0.5,
                          arrow_offset=-0.025)
    
    # Plot the state
    sim.add_subplot_point(subplot_index=plot1,
                          artist_index=artists1[0],
                          x=sim.time,
                          y=arm_state['position'])
    sim.add_subplot_point(subplot_index=plot1,
                          artist_index=artists1[1],
                          x=sim.time,
                          y=wheel_ang)
    sim.add_subplot_point(subplot_index=plot1,
                          artist_index=artists1[2],
                          x=sim.time,
                          y=target_wheel_angle)
    sim.add_subplot_point(subplot_index=plot2,
                          artist_index=artists2[0],
                          x=sim.time,
                          y=0.25*torque)
    
    # Update the target wheel angular position based on keyboard input
    target_wheel_angle = sim.iterate_val(curr_val=target_wheel_angle,
                                         iter_val=0.10,
                                         up_key='w',
                                         down_key='s',
                                         max_val=20.,
                                         min_val=-20.)
    
    # Step the sim
    sim.step(real_time=True,
              update_vis=True,
              update_ani=True)
            