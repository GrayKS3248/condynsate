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
mp = 1.0
mc = 6.0
mw = 1.0
l = 1.0
r = 0.25
g = 9.81

# Time is a symbol (variable)
t = Symbol('t')

# The generalized coordinates and the input torque are both functions of time.
# This means that they are initialized as Functions.
theta = Function('theta')
phi = Function('phi')
tau = Function('tau')

# Get the kinetic and potential energy of the mass
pos = Matrix([-r*phi(t) - l*sin( theta(t) ),
              0.0,
              l*cos( theta(t) )])
vel = diff(pos,t)
mass_KE = 0.5 * mp * (vel.T @ vel)[0,0]
mass_PE = mp*g*l*cos(theta(t))

# Get the kinetic energy of the cart
pos = Matrix([-phi(t)*r,
              0.0,
              0.0])
vel = diff(pos,t)
cart_KE = 0.5 * mc * (vel.T @ vel)[0,0]

# Get the kinetic and rotational energy of all four wheels
pos = Matrix([-phi(t)*r,
              0.0,
              0.0])
vel = diff(pos,t)
wheel_KE = 4.0 * (0.5 * mw * (vel.T @ vel)[0,0])
I = 0.5*mw*r**2
wheel_RE = 4.0*(I*diff(phi(t),t)**2)

# Get the lagrangian
L = (mass_KE + cart_KE + wheel_KE + wheel_RE) - mass_PE

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
Q[1,1] = 1.
Q[2,2] = 1.
Q[3,3] = 1.
R = np.eye(1)
R[0,0] = 10.

# Get the control gains
K, X, E = control.lqr(A, B, Q, R)

# Make the controller
def controller(**kwargs):
    # Gather the states measured via sensors
    omega_theta = kwargs['pendulum_rate']
    omega_phi = kwargs['wheel_rate']
    theta = kwargs['pendulum_angle']
    phi = kwargs['wheel_angle']

    # Gather the equilibrium state values
    omega_theta_e = kwargs['equil_pendulum_rate']
    omega_phi_e = kwargs['equil_wheel_rate']
    theta_e = kwargs['equil_pendulum_angle']
    phi_e = kwargs['equil_wheel_angle']
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

    # Limit the torque to between -5 and 5 Nm
    if tau > 5.0:
        tau = 5.0
    elif tau < -5.0:
        tau = -5.0

    # Return the controller calculated torque
    return tau

# Make a manual controller
def manual_controller(**kwargs):
    # Get the simulator object so we can use it to detect key presses
    sim = kwargs['sim']

    # Detect key presses and set torque accordingly 
    if sim.is_pressed("shift+d"):
        torque = 5.0
    elif sim.is_pressed("shift+a"):
        torque = -5.0
    elif sim.is_pressed("d"):
        torque = 1.0
    elif sim.is_pressed("a"):
        torque = -1.0
    else:
        torque = 0.0

    # Return the manually set torque
    return torque
    

###############################################################################
#BUILD THE SIMULATOR ENVIRONMENT
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
plot1, artists1 = sim.add_subplot(n_artists=2,
                                  subplot_type='line',
                                  title="Angles vs Time",
                                  x_label="Time [Seconds]",
                                  y_label="Angles [Deg / Rad]",
                                  colors=["m", "c"],
                                  line_widths=[2.5, 2.5],
                                  line_styles=["-", "-"],
                                  labels=["Pendulum [Deg]", "Wheel [Rad]"])
plot2, artists2 = sim.add_subplot(n_artists=1,
                                  subplot_type='line',
                                  title="Torque vs Time",
                                  x_label="Time [Seconds]",
                                  y_label="Torque [Nm]",
                                  y_lim=[-5.,5.],
                                  colors=["r"],
                                  line_widths=[2.5],
                                  line_styles=["-"])

# Give the pendulum an initial angle and speed
initial_angle = 0.25
initial_speed = 0.0

# Set the initial angle
sim.set_joint_position(urdf_obj=cart_obj,
                        joint_name='chassis_to_arm',
                        position=initial_angle,
                        initial_cond=True)

# Set the initial velocity
sim.set_joint_velocity(urdf_obj=cart_obj,
                        joint_name='chassis_to_arm',
                        velocity=initial_speed,
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
    # Use a sensor to collect the pendulum angle and rate
    pendulum_state = sim.get_joint_state(urdf_obj=cart_obj,
                                    joint_name='chassis_to_arm')
    pendulum_angle = pendulum_state['position']
    pendulum_rate = pendulum_state['velocity']
    
    # Use a sensor to collect the angles and rates of each wheel
    wheel1_state = sim.get_joint_state(urdf_obj=cart_obj,
                                       joint_name='chassis_to_wheel_1')
    wheel2_state = sim.get_joint_state(urdf_obj=cart_obj,
                                       joint_name='chassis_to_wheel_2')
    wheel3_state = sim.get_joint_state(urdf_obj=cart_obj,
                                       joint_name='chassis_to_wheel_3')
    wheel4_state = sim.get_joint_state(urdf_obj=cart_obj,
                                       joint_name='chassis_to_wheel_4')
    
    # Calculate the average wheel angle and velocity
    wheel_angle = 0.25*(wheel1_state['position']+
                       wheel2_state['position']+
                       wheel3_state['position']+
                       wheel4_state['position'])
    wheel_rate = 0.25*(wheel1_state['velocity']+
                       wheel2_state['velocity']+
                       wheel3_state['velocity']+
                       wheel4_state['velocity'])
    
    ###########################################################################
    # CONTROLLER
    # This is the section where you apply your controller.
    torque = controller(pendulum_angle=pendulum_angle,
                        pendulum_rate=pendulum_rate,
                        wheel_angle=wheel_angle,
                        wheel_rate=wheel_rate,
                        equil_pendulum_angle=0.0,
                        equil_pendulum_rate=0.0,
                        equil_wheel_angle=0.0,
                        equil_wheel_rate=0.0,
                        equil_torque=0.0,
                        gains=K)

    # We can also apply the manual controller if we want
    #torque = manual_controller(sim=sim)
    
    ###########################################################################
    # ACTUATOR
    # Apply one quater of the controller calculated torque to
    # each of the four the wheels.
    sim.set_joint_torque(urdf_obj=cart_obj,
                          joint_name='chassis_to_wheel_1',
                          torque=0.25*torque,
                          show_arrow=True,
                          arrow_scale=0.25,
                          arrow_offset=0.025)
    sim.set_joint_torque(urdf_obj=cart_obj,
                          joint_name='chassis_to_wheel_2',
                          torque=0.25*torque,
                          show_arrow=True,
                          arrow_scale=0.25,
                          arrow_offset=0.025)
    sim.set_joint_torque(urdf_obj=cart_obj,
                          joint_name='chassis_to_wheel_3',
                          torque=0.25*torque,
                          show_arrow=True,
                          arrow_scale=0.25,
                          arrow_offset=-0.025)
    sim.set_joint_torque(urdf_obj=cart_obj,
                          joint_name='chassis_to_wheel_4',
                          torque=0.25*torque,
                          show_arrow=True,
                          arrow_scale=0.25,
                          arrow_offset=-0.025)
    
    ###########################################################################
    # UPDATE THE PLOTS
    # This is how we add data points to the animator
    # Plot the pendulum angle, wheel angle, and torque
    sim.add_subplot_point(subplot_index=plot1,
                          artist_index=artists1[0],
                          x=sim.time,
                          y=180.*pendulum_angle/np.pi)
    sim.add_subplot_point(subplot_index=plot1,
                          artist_index=artists1[1],
                          x=sim.time,
                          y=wheel_angle)
    sim.add_subplot_point(subplot_index=plot2,
                          artist_index=artists2[0],
                          x=sim.time,
                          y=torque)
    
    ###########################################################################
    # STEP THE SIMULATION
    sim.step(real_time=True,
              update_vis=True,
              update_ani=True)
            