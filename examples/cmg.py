###############################################################################
#DEPENDENCIES
###############################################################################
import numpy as np
import condynsate
import control
from sympy import Symbol, Matrix, Function, Derivative
from sympy import diff, sin, cos, solve


###############################################################################
#DERIVE THE EQUATIONS OF MOTION
###############################################################################
# Constants of the system
mp = 0.5
ixx = 0.03
izz = 0.04
l = 1.5
g = 9.81
omega = 100.0

# Time is a symbol (variable)
t = Symbol('t')

# The generalized coordinates and the input torque are both functions of time.
# This means that they are initialized as Functions.
theta = Function('theta')
phi = Function('phi')
tau = Function('tau')

# Get the kinetic and potential energy of the mass
pos = Matrix([-l*sin(theta(t)),
              0.0,
              l*cos(theta(t))])
vel = diff(pos,t)
mass_KE = 0.5 * mp * (vel.T @ vel)[0,0]
mass_PE = mp*g*l*cos(theta(t))

# Get the rotational energy of the wheel
wheel_rate = Matrix([Derivative(phi(t), t),
                     Derivative(theta(t), t)*cos(phi(t)),
                     omega - Derivative(theta(t), t)*sin(phi(t))])
I = Matrix([[ixx, 0.0, 0.0],
            [0.0, ixx, 0.0],
            [0.0, 0.0, izz]])
wheel_RE = (0.5 * (wheel_rate.T @ I @ wheel_rate))[0, 0]

# Get the lagrangian
L = (mass_KE + wheel_RE) - mass_PE

# Get the equations of motion
eq1 = diff(diff(L, Derivative(theta(t), t)), t) - diff(L, theta(t))
eq2 = diff(diff(L, Derivative(phi(t), t)), t) - diff(L, phi(t)) - tau(t)

# Make the new functions for change of variables
omega_theta = Function('omega_theta')
omega_phi = Function('omega_phi')

# Make the change of variables
eq1 = eq1.subs({Derivative(theta(t), (t, 2)) : Derivative(omega_theta(t), t), 
                Derivative(phi(t), (t, 2))   : Derivative(omega_phi(t), t),
                Derivative(theta(t), t)      : omega_theta(t),
                Derivative(phi(t), t)        : omega_phi(t)})
eq2 = eq2.subs({Derivative(theta(t), (t, 2)) : Derivative(omega_theta(t), t), 
                Derivative(phi(t), (t, 2))   : Derivative(omega_phi(t), t),
                Derivative(theta(t), t)      : omega_theta(t),
                Derivative(phi(t), t)        : omega_phi(t)})

# Solve the equations for Derivative(omega_theta(t), t) and 
# Derivative(omega_phi(t), t)
soln = solve([eq1, eq2],
              Derivative(omega_theta(t), t),
              Derivative(omega_phi(t), t))

# Build the equations of motion vector
sys = Matrix([soln[Derivative(omega_theta(t), t)],
              soln[Derivative(omega_phi(t), t)],
              omega_theta(t),
              omega_phi(t)])


###############################################################################
#BUILD A CONTROLLER
###############################################################################
# Choose an equilibrium point
omega_theta_e = 0.0
omega_phi_e = 0.0
theta_e = 0.0
phi_e = 0.0
tau_e = 0.0

# Take the state jacobian (all the partial derivatives of state variables)
# of the equations of motion vector
x = Matrix([omega_theta(t),
            omega_phi(t),
            theta(t),
            phi(t)])
A_jac = sys.jacobian(x)

# Evaluate the state jacobian at the equilibrium point
A = A_jac.subs({omega_theta(t) : omega_theta_e, 
                omega_phi(t)   : omega_phi_e,
                theta(t)       : theta_e,
                phi(t)         : phi_e,
                tau(t)         : tau_e})

# Take the input jacobian (all the partial derivatives of input variables)
# of the equations of motion vector
u = Matrix([tau(t)])
B_jac = sys.jacobian(u)

# Evaluate the input jacobian at the equilibrium point
B = B_jac.subs({omega_theta(t) : omega_theta_e, 
                omega_phi(t)   : omega_phi_e,
                theta(t)       : theta_e,
                phi(t)         : phi_e,
                tau(t)         : tau_e})

# Define our state and input weights:
Q = np.eye(4)
Q[0,0] = 1.
Q[1,1] = 2.
Q[2,2] = 5.
Q[3,3] = 10.
R = np.eye(1)
R[0,0] = 10.

# Get the control gains
K, X, E = control.lqr(A, B, Q, R)

# Make the controller
def controller(**kwargs):
    # Gather the states measured via sensors
    omega_theta = kwargs['frame_rate']
    omega_phi = kwargs['cage_rate']
    theta = kwargs['frame_angle']
    phi = kwargs['cage_angle']

    # Gather the equilibrium state values
    omega_theta_e = kwargs['equil_frame_rate']
    omega_phi_e = kwargs['equil_cage_rate']
    theta_e = kwargs['equil_frame_angle']
    phi_e = kwargs['equil_cage_angle']
    tau_e = kwargs['equil_torque']

    # Gather the gains
    K = kwargs['gains']

    # Build the state vector
    x = np.array([omega_theta - omega_theta_e,
                  omega_phi - omega_phi_e,
                  theta - theta_e,
                  phi - phi_e])

    # Apply the gains using the formula u = -Kx
    u = -K@x

    # Convert the input vector to real inputs
    tau = u[0] + tau_e

    # Limit the torque to between -0.5 and 0.5 Nm
    if tau > 1.0:
        tau = 1.0
    elif tau < -1.0:
        tau = -1.0

    # Return the controller calculated torque
    return tau

# Make a manual controller
def manual_controller(**kwargs):
    # Get the simulator
    sim = kwargs['sim']
    
    # Set the torque to min
    torque = 0.0
    
    # Listen for keyboard presses:
    if sim.is_pressed('d'):
        torque = torque + 0.125
    if sim.is_pressed('shift+d'):
        torque = torque + 0.5
    if sim.is_pressed('a'):
        torque = torque - 0.125
    if sim.is_pressed('shift+a'):
        torque = torque - 0.5

    # Return the manually set torque
    return torque


###############################################################################
#BUILD THE SIMULATOR ENVIRONMENT
###############################################################################
# Create an instance of the simulator with visualization
sim = condynsate.Simulator(visualization=True,
                           animation=True,
                           animation_fr=15.)

# # Load urdf objects
cmg_obj = sim.load_urdf(urdf_path='./cmg_vis/cmg.urdf',
                        roll=np.pi/2,
                        yaw=3*np.pi/4.,
                        pitch=np.pi,
                        fixed=True,
                        update_vis=True)

# Set the wheel rate and pendulum damping
sim.set_joint_velocity(urdf_obj=cmg_obj,
                        joint_name='cage_to_wheel',
                        velocity = omega,
                        initial_cond=True)
sim.set_joint_damping(urdf_obj=cmg_obj,
                      joint_name='wall_to_frame_axle',
                      damping=0.1)

# Make plot for phase space
plot1, artists1 = sim.add_subplot(n_artists=2,
                                  subplot_type='line',
                                  title="State",
                                  x_label="Time [s]",
                                  y_label="Angles [Deg]",
                                  colors=["m", "c"],
                                  line_widths=[2.5, 2.5],
                                  line_styles=["-", "-"],
                                  labels=['Pendulum', 'Cage'],
                                  y_lim=[-90.,90],
                                  h_zero_line=True)
plot2, artists2 = sim.add_subplot(n_artists=1,
                                  subplot_type='line',
                                  title="Input",
                                  x_label="Time [s]",
                                  y_label="Torque [Nm]",
                                  colors=["k"],
                                  line_widths=[2.5],
                                  line_styles=["-"],
                                  y_lim=[-1.,1.],
                                  h_zero_line=True)

# Set initial frame angle and rate
initial_frame_angle = np.pi/12.0
initial_frame_rate = 0.0
sim.set_joint_position(urdf_obj=cmg_obj,
                        joint_name='wall_to_frame_axle',
                        position = initial_frame_angle,
                        initial_cond=True)
sim.set_joint_velocity(urdf_obj=cmg_obj,
                        joint_name='wall_to_frame_axle',
                        velocity = initial_frame_rate,
                        initial_cond=True)
sim.set_joint_position(urdf_obj=cmg_obj,
                        joint_name='frame_to_cage_axle',
                        position = np.pi/12.,
                        initial_cond=True)


###############################################################################
#SIMULATION LOOP
###############################################################################
# Run the simulation
sim.open_animator_gui()
sim.await_keypress(key="enter")
while(not sim.is_done):    
    ###########################################################################
    # SENSOR
    # Get the frame angle and rate
    frame_state = sim.get_joint_state(urdf_obj=cmg_obj,
                                joint_name="wall_to_frame_axle")
    frame_angle = frame_state['position']
    frame_rate = frame_state['velocity']
    
    # Get the cage angle and rate
    cage_state = sim.get_joint_state(urdf_obj=cmg_obj,
                                joint_name="frame_to_cage_axle")
    cage_angle = cage_state['position']
    cage_rate = cage_state['velocity']
    
    ###########################################################################
    # CONTROLLER
    torque =  controller(frame_angle=frame_angle,
                          frame_rate=frame_rate,
                          cage_angle=cage_angle,
                          cage_rate=cage_rate,
                          equil_frame_angle=theta_e,
                          equil_frame_rate=omega_theta_e,
                          equil_cage_angle=phi_e,
                          equil_cage_rate=omega_phi_e,
                          equil_torque=tau_e,
                          gains=K)
    # torque =  manual_controller(sim=sim)

    ###########################################################################
    # ACTUATOR
    # Set the CMG wheel torque
    sim.set_joint_torque(urdf_obj=cmg_obj,
                          joint_name="frame_to_cage_axle",
                          torque=torque,
                          show_arrow=True,
                          arrow_scale=1.)
    
    ###########################################################################
    # UPDATE THE PLOT
    sim.add_subplot_point(subplot_index=plot1,
                          artist_index=artists1[0],
                          x=sim.time,
                          y=frame_angle*180/np.pi)
    sim.add_subplot_point(subplot_index=plot1,
                          artist_index=artists1[1],
                          x=sim.time,
                          y=cage_angle*180/np.pi)
    sim.add_subplot_point(subplot_index=plot2,
                          artist_index=artists2[0],
                          x=sim.time,
                          y=torque)
    
    ###########################################################################
    # STEP THE SIMULATION
    sim.step(real_time=True,
              update_vis=True,
              update_ani=True)
            