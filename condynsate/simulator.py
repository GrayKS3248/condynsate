"""
This module provides the Simulator class and all associated classes and 
functions that are used by it.
"""


###############################################################################
#DEPENDENCIES
###############################################################################
import numpy as np
import time
import pybullet
from pybullet_utils import bullet_client as bc
from .visualizer import Visualizer
from .animator import Animator
from .utils import format_path, format_RGB, wxyz_to_xyzw, xyzw_to_wxyz


###############################################################################
#URDF OBJECT CLASS
###############################################################################
class URDF_Obj:
    """
    URDF_Obj encapsulates a urdf id, a joint map, and a link map.
    """
    def __init__(self,
                 urdf_id,
                 joint_map,
                 link_map,
                 update_vis):
        """
        Initialize an instance of the URDF_Obj class. This class is used to
        store information relating to a urdf described by a .urdf file.

        Parameters
        ----------
        urdf_id : int
            The unique integer ID of the loaded urdf in the simulation
            engine.
        joint_map : dictionary
            A dictionary that maps all urdf joint names to joint
            indices.
        link_map : dictionary
            A dictionary that maps all urdf link names to joint indices.
        update_vis : bool
            A boolean flag that indicates whether this urdf will be updated
            by the visualizer each time step.
            
        Returns
        -------
        None.

        """
        self.urdf_id = urdf_id
        self.joint_map = joint_map
        self.link_map = link_map     
        self.update_vis = update_vis


###############################################################################
#SIMULATOR CLASS
###############################################################################
class Simulator:
    """
    Simulator manages the PyBullet based simulation of dynamic objects and 
    handles automatic visualization
    """
    def __init__(self,
                 visualization=True,
                 animation=True,
                 gravity=[0., 0., -9.81]):
        """
        Initializes an instance of the Simulator class.

        Parameters
        ----------
        visualization : bool, optional
            A boolean flag that indicates whether the simulation will be 
            visualized in meshcat.
        visualization : bool, optional
            A boolean flag that indicates whether plots will be animated
            in real time.
        gravity : array-like, shape (3,) optional
            The gravity vectory in m/s^2. The default is [0., 0., -9.81].

        Returns
        -------
        None.

        """
        # Connect to pybullet
        self.engine = bc.BulletClient(connection_mode=pybullet.DIRECT)
        
        # Configure gravity
        self.set_gravity(gravity)
        
        # Configure physics engine parameters
        self.time = 0.0
        self.times = [self.time]
        self.dt = 0.01
        self.last_step_time = time.time()
        self.engine.setPhysicsEngineParameter(
            fixedTimeStep=self.dt,
            numSubSteps=4,
            restitutionVelocityThreshold=0.05,
            enableFileCaching=0)
        
        # Keep track of all urdfs loaded to simulator
        self.urdf_objs = []
        
        # Create a visualizer
        if visualization:
            self.vis = Visualizer(grid_vis=False,axes_vis=False)
        else:
            self.vis=None
            
        # Create an animator
        if animation:
            self.ani = Animator()
        else:
            self.ani=None
        
    
    def set_gravity(self,
                    gravity):
        """
        Sets the acceleration due to gravity vector.

        Parameters
        ----------
        gravity : array-like, shape (3,)
            The acceleration due to gravity vector in m/s^2.

        Returns
        -------
        None.

        """
        # Configure gravity
        self.engine.setGravity(gravity[0],
                               gravity[1],
                               gravity[2])
    
        
    def load_urdf(self,
                  urdf_path,
                  tex_path='./examples/cmg_vis/check.png',
                  position = [0., 0., 0.],
                  wxyz_quaternion = [1., 0., 0., 0.],
                  roll=None,
                  pitch=None,
                  yaw=None,
                  fixed=False,
                  update_vis=True):
        """
        Loads a URDF to the simulation engine. All joint's
        velocity control is disabled (this allows the joint to move freely),
        angular and linear damping is set to 0 (eliminates air resistance), and 
        joint dampling is set to 0 (eliminates joint friction).

        Parameters
        ----------
        urdf_path : string
            The path to the .urdf file that describes the urdf to be
            loaded into the simulation.
        tex_path : string, optional
            The path pointing towards a texture file. This texture is applied
            only to static .obj objects.
            The default is './examples/cmg_vis/check.png'.
        position : array-like, shape (3,) optional
            The initial position of the urdf. The default is [0., 0., 0.].
        wxyz_quaternion : array-like, shape (4,) optional
            A wxyz quaternion that describes the intial orientation of the
            urdf. When roll, pitch, and yaw all have None type, the
            quaternion is used. If any roll, pitch, or yaw have non None type,
            the quaternion is ignored. The default is [1., 0., 0., 0.].
        roll : float, optional
            The initial roll angle of the urdf. The default is None.
        pitch : float, optional
            The initial pitch angle of the urdf. The default is None.
        yaw : float, optional
            The initial yaw angle of the urdf. The default is None.
        fixed : bool, optional
            A boolean flag that indicates whether the base joint of the
            loaded urdf is fixed. The default is False.
        update_vis : bool, optional
            A boolean flag that indicates whether this urdf will be updated
            by the visualizer each time step. The default is True.
            
        Returns
        -------
        urdf_obj : URDF_Obj
            A URDF_Obj that describes the urdf that was loaded into
            the simulation.
            
        """
        # Get the properly formatted string of the urdf path
        urdf_path = format_path(urdf_path)
        
        # Get the initial position of the urdf object in world coordinates
        position = np.array(position)
        
        # If no euler angles are specified, use the quaternion to set the
        # initial orientation of the urdf object
        if roll==None and pitch==None and yaw==None: 
            orientation = wxyz_to_xyzw(wxyz_quaternion)
        
        # If any euler angles are specified, use the euler angles to set the
        # initial orientation of the urdf object
        # Any unspecified euler angles are set to 0.0
        else:
            if roll==None:
                roll=0.0
            if pitch==None:
                pitch=0.0
            if yaw==None:
                yaw=0.0
            euler_angles = [roll, pitch,  yaw]
            orientation = self.engine.getQuaternionFromEuler(euler_angles)
        
        # Use implicit cylinder for collision and physics calculation
        # Specifies to the engine to use the inertia from the urdf file
        f1 = self.engine.URDF_USE_IMPLICIT_CYLINDER
        f2 = self.engine.URDF_USE_INERTIA_FROM_FILE
        
        # Load the urdf object
        urdf_id = self.engine.loadURDF(urdf_path,
                                       flags=(f1 | f2),
                                       basePosition=position,
                                       baseOrientation=orientation,
                                       useFixedBase=fixed)
        
        # Get the joint and link maps for the urdf object
        joint_map, link_map = self.make_joint_and_link_maps(urdf_id)
        
        # Create urdf_obj and adjust the default state of its joints
        urdf_obj = URDF_Obj(urdf_id, joint_map, link_map, update_vis)
        for joint_name in joint_map:
            
            # Set the joint's friction parameters to model metal to metal
            # friction
            self.set_joint_friction_params(urdf_obj=urdf_obj,
                                           joint_name=joint_name,
                                           lateral_friction=1.0,
                                           spinning_friction=0.0,
                                           rolling_friction=0.0)
            
            # Set the joint's contact parameters to model stiff metal contact
            self.set_joint_contact_params(urdf_obj=urdf_obj,
                                          joint_name=joint_name,
                                          restitution=0.5,
                                          contact_damping=-1.0,
                                          contact_stiffness=-1.0)
            
            # Set the linear and angular damping to 0 (eliminate drag)
            self.set_joint_lin_ang_damp(urdf_obj,
                                        joint_name=joint_name,
                                        linear_damping=0.,
                                        angular_damping=0.)
            
            # Set damping of joints to 0 (eliminate joint friction)
            self.set_joint_damping(urdf_obj,
                                   joint_name=joint_name,
                                   damping=0.)

            # Disable velocity control if the joint is not a base joint
            if joint_map[joint_name]!=-1:
                self.disable_joint_vel_con(urdf_obj,
                                           joint_name=joint_name)

        # Add urdf objects to the visualizer if visualization is occuring
        if isinstance(self.vis, Visualizer):
            self.add_urdf_to_visualizer(urdf_obj=urdf_obj,
                                        tex_path=tex_path)

        # Return the URDF_Obj
        self.urdf_objs.append(urdf_obj)
        return urdf_obj
    
    
    def make_joint_and_link_maps(self,
                                 urdf_id):
        """
        Creates a joint map and a link map for a urdf.

        Parameters
        ----------
        urdf_id : int
            The unique integer ID of a loaded urdf in the simulation
            engine.

        Returns
        -------
        joint_map : dictionary
            A dictionary that maps all urdf joint names to joint id
        link_map : dictionary
            A dictionary that maps all urdf link names to joint id

        """
        # A dictionary that maps joint names to joint id
        # A dictionary that maps link names to joint id
        joint_map = {}
        link_map = {}

        # Check if the urdf object has a base link
        data = self.engine.getVisualShapeData(urdf_id)
        if data[0][1] == -1:
            joint_map['base'] = -1
            link_map['base'] = -1
        
        # Go through all joints in the urdf object
        num_joints = self.engine.getNumJoints(urdf_id)
        for joint_id in range(num_joints):

            # Lookup joint info
            joint_info = self.engine.getJointInfo(urdf_id, joint_id)
            joint_name = joint_info[1].decode('UTF-8')
            link_name = joint_info[12].decode('UTF-8')
                
            # Add joint and link data
            joint_map[joint_name] = joint_id
            link_map[link_name] = joint_id
            
        # Return the maps
        return joint_map, link_map
    
    
    def disable_joint_vel_con(self,
                              urdf_obj,
                              joint_name):
        """
        Disable velocity control mode for a joint. In pybullet, all joints are
        initialized with velocity control mode engaged and a target velocity of
        0. To simulate freely moving joints, velocity control is turned off.

        Parameters
        ----------
        urdf_obj : URDF_Obj
            A URDF_Obj that contains that joint for which velocity control is
            disabled.
        joint_name : string
            The name of the joint whose velocity control is disabled. The
            joint name is specified in the .urdf file.

        Returns
        -------
        None.

        """
        # Gather information from urdf_obj
        urdf_id = urdf_obj.urdf_id
        joint_map = urdf_obj.joint_map
        
        # Set the joint velocity
        if joint_name in joint_map:
            joint_id = [joint_map[joint_name]]
            mode = self.engine.VELOCITY_CONTROL
            self.engine.setJointMotorControlArray(urdf_id,
                                                  joint_id,
                                                  mode,
                                                  forces=[0])
    
    
    def set_joint_lin_ang_damp(self,
                               urdf_obj,
                               joint_name,
                               linear_damping=0.,
                               angular_damping=0.):
        """
        Allows user to set the linear and angular damping of a joint. Linear
        and angular damping is a way to model drag. It is typically
        reccomended that the user set these values to 0 for all joints.

        Parameters
        ----------
        urdf_obj : URDF_Obj
            A URDF_Obj that contains that joint whose linear and angular
            damping is being set.
        joint_name : string
            The name of the joint whose linear and angular damping is set. The
            joint name is specified in the .urdf file.
        linear_damping : float, optional
            The value of linear damping to apply. The default is 0..
        angular_damping : float, optional
            The value of angular damping to apply. The default is 0..

        Returns
        -------
        None.

        """
        # Gather information from urdf_obj
        urdf_id = urdf_obj.urdf_id
        joint_map = urdf_obj.joint_map
        
        # Set the joint linear and angular damping
        if joint_name in joint_map:
            joint_id = joint_map[joint_name]
            self.engine.changeDynamics(urdf_id,
                                       joint_id,
                                       linearDamping=linear_damping,
                                       angularDamping=angular_damping)
    
    
    def set_joint_damping(self,
                          urdf_obj,
                          joint_name,
                          damping=0.):
        """
        Sets the damping of a joint in a urdf. The damping of a joint
        defines the energy loss incurred during movement of the joint. It is
        a way to model joint friction.

        Parameters
        ----------
        urdf_obj : URDF_Obj
            A URDF_Obj that contains that joint whose damping is being set.
        joint_name : string
            The name of the joint whose damping is set. The joint name is
            specified in the .urdf file.
        damping : float, optional
            The value of damping to apply. The default is 0..

        Returns
        -------
        None.

        """
        # Gather information from urdf_obj
        urdf_id = urdf_obj.urdf_id
        joint_map = urdf_obj.joint_map
        
        # Set the joint damping
        if joint_name in joint_map:
            joint_id = joint_map[joint_name]
            self.engine.changeDynamics(urdf_id, joint_id, jointDamping=damping)
    
    
    def set_joint_friction_params(self,
                                  urdf_obj,
                                  joint_name,
                                  lateral_friction=0.0,
                                  spinning_friction=0.0,
                                  rolling_friction=0.0):
        """
        Sets a joint's friction parameters. These parameters determine the
        friction characteristics between 2 joints.

        Parameters
        ----------
        urdf_obj : URDF_Obj
            A URDF_Obj that contains that joint whose friction is being set.
        joint_name : string
            The name of the joint whose friction is set. The joint name is
            specified in the .urdf file.
        lateral_friction : float, optional
            The lateral friction applied to the joint. The default is 0.0.
        spinning_friction : float, optional
            The spinning friction applied to the joint. The default is 0.0.
        rolling_friction : float, optional
            The rolling friction applied to the joint. The default is 0.0.

        Returns
        -------
        None.

        """
        # Gather information from urdf_obj
        urdf_id = urdf_obj.urdf_id
        joint_map = urdf_obj.joint_map
        
        # Set the joint friction parameters
        if joint_name in joint_map:
            joint_id = joint_map[joint_name]
            self.engine.changeDynamics(urdf_id,
                                       joint_id,
                                       lateralFriction=lateral_friction, 
                                       spinningFriction=spinning_friction, 
                                       rollingFriction=rolling_friction)
    
    
    def set_joint_contact_params(self,
                                 urdf_obj,
                                 joint_name,
                                 restitution=0.0,
                                 contact_damping=0.0,
                                 contact_stiffness=0.0):
        """
        Sets a joint's contact parameters. These parameters determine the
        energy transfer between two joints in contact.

        Parameters
        ----------
        urdf_obj : URDF_Obj
            A URDF_Obj that contains that joint whose contact params are
            being set.
        joint_name : string
            The name of the joint whose contact params are set.
            The joint name is specified in the .urdf file.
        restitution : float, optional
            The restitution applied to the joint. The default is 0.0.
        contact_damping : float, optional
            The contact damping friction applied to the joint.
            The default is 0.0.
        contact_stiffness : float, optional
            The contact stiffness applied to the joint. The default is 0.0.

        Returns
        -------
        None.

        """
        # Gather information from urdf_obj
        urdf_id = urdf_obj.urdf_id
        joint_map = urdf_obj.joint_map
        
        # Set the joint contact parameters
        if joint_name in joint_map:
            joint_id = joint_map[joint_name]
            self.engine.changeDynamics(urdf_id,
                                       joint_id,
                                       restitution=restitution, 
                                       contactDamping=contact_damping, 
                                       contactStiffness=contact_stiffness)
        
        
    def set_joint_position(self,
                           urdf_obj,
                           joint_name,
                           position=0.):
        """
        Sets the position of a joint of a urdf .

        Parameters
        ----------
        urdf_obj : URDF_Obj
            A URDF_Obj that contains that joint whose position is being set.
        joint_name : string
            The name of the joint whose position is set. The joint name is
            specified in the .urdf file.
        position : float, optional
            The position in rad to be applied to the joint.
            The default is 0..

        Returns
        -------
        None.

        """
        # Gather information from urdf_obj
        urdf_id = urdf_obj.urdf_id
        joint_map = urdf_obj.joint_map
        
        # Set the joint velocity
        if joint_name in joint_map:
            joint_id = [joint_map[joint_name]]
            mode = self.engine.POSITION_CONTROL
            position = [position]
            self.engine.setJointMotorControlArray(urdf_id,
                                                  joint_id,
                                                  mode,
                                                  forces=[1000.],
                                                  targetPositions=position)
    
    
    def set_joint_velocity(self,
                          urdf_obj,
                          joint_name,
                          velocity=0.):
        """
        Sets the velocity of a joint of a urdf.

        Parameters
        ----------
        urdf_obj : URDF_Obj
            A URDF_Obj that contains that joint whose velocity is being set.
        joint_name : string
            The name of the joint whose velocity is set. The joint name is
            specified in the .urdf file.
        velocity : float, optional
            The velocity in rad/s to be applied to the joint.
            The default is 0..

        Returns
        -------
        None.

        """
        # Gather information from urdf_obj
        urdf_id = urdf_obj.urdf_id
        joint_map = urdf_obj.joint_map
        
        # Set the joint velocity
        if joint_name in joint_map:
            joint_id = [joint_map[joint_name]]
            mode = self.engine.VELOCITY_CONTROL
            velocity = [velocity]
            self.engine.setJointMotorControlArray(urdf_id,
                                                  joint_id,
                                                  mode,
                                                  forces=[1000.],
                                                  targetVelocities=velocity)
    
    
    def set_joint_torque(self,
                         urdf_obj,
                         joint_name,
                         torque=0.):
        """
        Sets the torque of a joint of a urdf.

        Parameters
        ----------
        urdf_obj : URDF_Obj
            A URDF_Obj that contains that joint whose torque is being set.
        joint_name : string
            The name of the joint whose torque is set. The joint name is
            specified in the .urdf file
        torque : float, optional
            The torque in NM to be applied to the joint. The default is 0..

        Returns
        -------
        None.

        """
        # Gather information from urdf_obj
        urdf_id = urdf_obj.urdf_id
        joint_map = urdf_obj.joint_map
        
        # Set the joint torque
        if joint_name in joint_map:
            joint_id = [joint_map[joint_name]]
            mode = self.engine.TORQUE_CONTROL
            torque = [torque]
            self.engine.setJointMotorControlArray(urdf_id,
                                                  joint_id,
                                                  mode,
                                                  forces=torque)
            
            
    def reset_joint(self,
                    urdf_obj,
                    joint_name,
                    position=0.,
                    velocity=0.):
        """
        Resets a joint to a desired position and velocity.
    
        Parameters
        ----------
        urdf_obj : URDF_Obj
            A URDF_Obj that contains that joint whose torque is being set.
        joint_name : string
            The name of the joint whose torque is set. The joint name is
            specified in the .urdf file.
        position : float, optional
            The position to which the joint is reset. The default is 0..
        velocity : float, optional
            The velocity to which the joint is reset. The default is 0..
    
        Returns
        -------
        None.
    
        """
        # Gather information from urdf_obj
        urdf_id = urdf_obj.urdf_id
        joint_map = urdf_obj.joint_map
        
        # Set the joint torque
        if joint_name in joint_map:
            joint_id = joint_map[joint_name]
            self.engine.resetJointState(urdf_id,
                                        joint_id,
                                        position,
                                        velocity)

            
    def set_link_color(self,
                          urdf_obj,
                          link_name,
                          color=[91, 155, 213],
                          transparent = False,
                          opacity = 1.0):
        """
        Allows the user to change the color, transparency, and opacity
        of an existing urdf in the simulation. The position and orientation
        are not altered.

        Parameters
        ----------
        urdf_obj : URDF_Obj
            A URDF_Obj that contains that link whose color is being updated.
        link_name : string
            The name of the link whose color is being updated. The link name is
            specified in the .urdf file unless the link is the base link of the
            urdf, then its name is always 'base'.
        color : array-like, size (3,), optional
            The 0-255 RGB color of the link.
            The default is [91, 155, 213].
        transparent : boolean, optional
            A boolean that indicates if the link is transparent.
            The default is False.
        opacity : float, optional
            The opacity of the link. Can take float values between 0.0 and 1.0.
            The default is 1.0.

        Returns
        -------
        None.

        """
        # If there is no visualizer, do not attempt to update it
        if not isinstance(self.vis, Visualizer):
            return    
        
        # If the link name doesn't exist, don't attempt to update it
        if not (link_name in urdf_obj.link_map):
            return
    
        # Get name and id data from urdf_obj
        urdf_id = urdf_obj.urdf_id
        urdf_name = str(urdf_id)
        link_id = urdf_obj.link_map[link_name]
        
        # Get current visual data for the requested link
        vis_data = self.engine.getVisualShapeData(urdf_id)
        stl_path = ""
        for vis_datum in vis_data:
            if vis_datum[1] == link_id:
                stl_path = vis_datum[4]
            
        # Format stl path
        stl_path = format_path(stl_path.decode('UTF-8'))
        
        # Ensure color is in proper format
        color = format_RGB(color,
                            range_to_255=False)
        
        # Set the requested color
        self.vis.set_link_color(urdf_name = urdf_name,
                                   link_name = link_name,
                                   stl_path = stl_path, 
                                   color = color,
                                   transparent = transparent,
                                   opacity = opacity)
        
        
    def set_link_mass(self,
                      urdf_obj,
                      link_name,
                      mass=0.):
        """
        Sets the mass of a link in a urdf.

        Parameters
        ----------
        urdf_obj : URDF_Obj
            A URDF_Obj that contains that link whose mass is being set.
        link_name : string
            The name of the link whose mass is set. The link name is
            specified in the .urdf file unless the link is the base link of the
            urdf, then its name is always 'base'.
        mass : float, optional
            The mass to set in kg. The default is 0..

        Returns
        -------
        None.

        """
        # Gather information from urdf_obj
        urdf_id = urdf_obj.urdf_id
        link_map = urdf_obj.link_map
        
        # Set the link mass
        if link_name in link_map:
            joint_id = link_map[link_name]
            self.engine.changeDynamics(urdf_id, joint_id, mass=mass)
        
        
    def apply_force_to_link(self,
                            urdf_obj,
                            link_name,
                            force=[0., 0., 0.]):
        """
        Applies an external force the to center of a specified link of a urdf.

        Parameters
        ----------
        urdf_obj : URDF_Obj
            A URDF_Obj that contains that link to which the force is applied.
        link_name : string
            The name of the link to which the force is applied.
            The link name is specified in the .urdf file unless the link is
            the base link of the urdf, then its name is always 'base'.
        force : array-like, shape (3,), optional
            The force vector in body coordinates to apply to the link.
            The default is [0., 0., 0.].

        Returns
        -------
        None.

        """
        # Gather information from urdf_obj
        urdf_id = urdf_obj.urdf_id
        link_map = urdf_obj.link_map
        
        # Set the link mass
        if link_name in link_map:
            joint_id = link_map[link_name]
        else:
            return
        
        # Apply the external force
        self.engine.applyExternalForce(urdf_id,
                                       joint_id,
                                       force,
                                       [0., 0., 0.],
                                       self.engine.LINK_FRAME)
           
        
    def get_base_state(self,
                       urdf_obj,
                       body_coords=False):
        """
        Gets the rigid body states (position, orientation, linear velocity, and
        angular velocitiy) of the base of a given urdf. 

        Parameters
        ----------
        urdf_obj : URDF_Obj
            A URDF_Obj whose state is being measured.
        body_coords : bool, optional
            A boolean flag that indicates whether the velocity and angular 
            velocity is given in world coords (False) or body coords (True).
            The default is False.

        Returns
        -------
        pos : array-like, shape (3,)
            The (x,y,z) world coordinates of the base of the urdf.
        rpy : array-like, shape (3,)
            The Euler angles (roll, pitch, yaw) of the base of the urdf
            that define the body's orientation in the world.
        vel : array-like, shape (3,)
            The linear velocity of the base of the urdf in either world 
            coords or body coords.
        ang_vel : array-like, shape (3,)
            The angular velocity of the base of the urdf in either world 
            coords or body coords.

        """
        # Get object id
        urdf_id = urdf_obj.urdf_id
        
        # Retrieve pos, rpy, and vel (in world coordinates) data
        pos, xyzw_ori = self.engine.getBasePositionAndOrientation(urdf_id)
        rpy = self.engine.getEulerFromQuaternion(xyzw_ori)
        vel_world, ang_vel_world = self.engine.getBaseVelocity(urdf_id)
        
        # Format the base state data
        pos = np.array(pos)
        rpy = np.array(rpy)
        vel_world = np.array(vel_world)
        ang_vel_world = np.array(ang_vel_world)
        
        if body_coords:
            # Get the rotation matrix of the body in the world
            R_body_to_world = self.engine.getMatrixFromQuaternion(xyzw_ori)
            R_body_to_world = np.array(R_body_to_world)
            R_body_to_world = np.reshape(R_body_to_world, (3,3))
            
            # Get the body velocities in body coordinates
            R_world_to_body = R_body_to_world.T
            vel_body = R_world_to_body @ vel_world
            ang_vel_body =  R_world_to_body @ ang_vel_world
            
            return pos, rpy, vel_body, ang_vel_body
        
        return pos, rpy, vel_world, ang_vel_world
    
    
    def get_joint_state(self,
                        urdf_obj,
                        joint_name):
        """
        Gets the state of a joint (angle and velocity for continuous joints).

        Parameters
        ----------
        urdf_obj : URDF_Obj
            A URDF_Obj whose joint state is being measured.
        joint_name : string
            The name of the joint whose state is measured. The joint name is
            specified in the .urdf file.

        Returns
        -------
        ang : float
            The angle, in rads, of the joint.
        vel : float
            The angular velocity, in rads/s, of the joint.

        """
        # Get object id and joint id
        urdf_id = urdf_obj.urdf_id
        joint_map = urdf_obj.joint_map
        
        # Get the joint id
        if joint_name in joint_map:
            joint_id = [joint_map[joint_name]]
        else:
            return
        
        # Retrieve the joint states
        states = self.engine.getJointStates(urdf_id, joint_id)
        ang = states[0][0]
        vel = states[0][1]
        return ang, vel
    
    
    def add_urdf_to_visualizer(self,
                               urdf_obj,
                               tex_path='./cmg_vis/check.png'):
        """
        Adds urdfs to the visualizer. URDFs describe systems assembles from
        .stl and .obj links.

        Parameters
        ----------
        vis : Visualizer
            The visualizer to which the urdf is added.
        urdf_obj : URDF_Obj
            A URDF_Obj that will be added to the visualizer.
        tex_path : string, optional
            The path pointing towards a texture file. This texture is applied
            to all .obj links in the urdf.

        Returns
        -------
        None.

        """
        # If there is no visualizer, do not attempt to update it
        if not isinstance(self.vis, Visualizer):
            return
        
        # Extract the visual data from the urdf object in the simulator
        paths,names,scales,colors,poss,oris = self.get_urdf_vis_dat(urdf_obj)
        
        # Make the URDF name and format the texture path
        urdf_name = str(urdf_obj.urdf_id)
        tex_path = format_path(tex_path)
        
        # Loop through all the links
        for i in range(len(paths)):
            
            # If the current link is defined by an .obj file
            if paths[i][-4:] == ".obj":
                self.vis.add_obj(urdf_name=urdf_name,
                                 link_name=names[i],
                                 obj_path=paths[i],
                                 tex_path=tex_path,
                                 scale=scales[i],
                                 translate=poss[i],
                                 wxyz_quaternion=oris[i])
                
            # If the current link is defined by an .stl file
            elif paths[i][-4:] == ".stl":
                link_name = names[i]
                rgb = format_RGB(colors[i][0:3],
                                  range_to_255=True)
                opacity = colors[i][3]
                transparent = opacity != 1.0
                self.vis.add_stl(urdf_name=urdf_name,
                                 link_name=link_name,
                                 stl_path=paths[i],
                                 color=rgb,
                                 transparent=transparent,
                                 opacity=opacity,
                                 scale=scales[i],
                                 translate=poss[i],
                                 wxyz_quaternion=oris[i])

    
    def update_urdf_visual(self,
                           urdf_obj):
        """
        Updates the positions of dynamic links in the visualizer.

        Parameters
        ----------
        urdf_obj : URDF_Obj, optional
            A URDF_Obj whose links are being updated.

        Returns
        -------
        None.

        """
        # If there is no visualizer, do not attempt to update it
        if not isinstance(self.vis, Visualizer):
            return
        
        # Collect the visual data and urdf name
        paths,names,scales,colors,poss,oris = self.get_urdf_vis_dat(urdf_obj)
        urdf_name = str(urdf_obj.urdf_id)
        
        # Go through all links in urdf object and update their position
        for i in range(len(paths)):
            link_name = names[i]
            self.vis.apply_transform(urdf_name=urdf_name,
                                     link_name=link_name,
                                     scale=scales[i],
                                     translate=poss[i],
                                     wxyz_quaternion=oris[i])
    
    
    def get_urdf_vis_dat(self,
                         urdf_obj):
        """
        Extracts all relevant visual data from a urdf loaded into the
        simulator.

        Parameters
        ----------
        urdf_obj : URDF_Obj
            A URDF_Obj whose visual data is being extracted.

        Returns
        -------
        paths : list of strings
            A list containing the paths to the files containing urdf or link
            geometries.
        link_names : list of strings
            A list containing the name of all links in the urdf.
        scales : list of lists (3,)
            A list containing the scale data for the urdf or all links
            in the urdf .
        colors : list of lists (4,)
            A list containing the RGBA data for the urdf or all links
            in the urdf.
        positions : list of lists (3,)
            A list containing the position data for the urdf of all
            links in the urdf.
        orientations : list of lists (4,)
            A list containing the wxyz quaternion(s) for the urdf or all
            links in the urdf.

        """
        # Create placeholders for all visual data collected
        paths = []
        link_names = []
        scales = []
        colors = []
        positions = []
        orientations = []
    
        # Determine the urdf id of the object
        urdf_id = urdf_obj.urdf_id
        
        # Get the visual data of the urdf object
        vis_data = self.engine.getVisualShapeData(urdf_id)
        for vis_datum in vis_data:
            path = vis_datum[4].decode('UTF-8')
            path = format_path(path)
            paths.append(path)
            scale = list(vis_datum[3])
            scales.append(scale)
            color = list(vis_datum[7])
            colors.append(color)
            
            # Extract link id
            link_id = vis_datum[1]
            
            # Link id of -1 implies that the current link is the base
            # of a robot
            if link_id == -1:
                link_names.append('base')
                pos_ori = self.engine.getBasePositionAndOrientation(urdf_id)
                position = list(pos_ori[0])
                positions.append(position)
                orientation = list(xyzw_to_wxyz(pos_ori[1]))
                orientations.append(orientation)
                
            # If the link id is not -1, then the current link is not the base.
            # Therefore, position and orientation data is extracted from the
            # joint state and link state
            else:
                # Collect link name
                joint_data = self.engine.getJointInfo(urdf_id, link_id)
                link_name = joint_data[12].decode('UTF-8')
                link_names.append(link_name)
                
                # Extract link positions and orientations
                link_state = self.engine.getLinkState(urdf_id, link_id)
                position = list(link_state[4])
                positions.append(position)
                orientation = list(xyzw_to_wxyz(link_state[5]))
                orientations.append(orientation)
                
        return paths, link_names, scales, colors, positions, orientations
    
    
    def transform_camera(self,
                         scale = [1., 1., 1.],
                         translate = [0., 0., 0.],
                         wxyz_quaternion = [1., 0., 0., 0.],
                         roll=None,
                         pitch=None,
                         yaw=None):
        """
        Transforms the position, orientation, and scale of the Visualizer scene
        camera.

        Parameters
        ----------
        scale : array-like, size (3,), optional
            The scaling of the camera view about the camera point along the
            three axes. The default is [1., 1., 1.].
        translate : array-like, size (3,), optional
            The translation of the camera point along the three axes.
            The default is [0., 0., 0.].
        wxyz_quaternion : array-like, shape (4,) optional
            A wxyz quaternion that describes the intial orientation of camera
            about the camera point. When roll, pitch, and yaw all have None
            type, the quaternion is used. If any roll, pitch, or yaw have non
            None type, the quaternion is ignored.
            The default is [1., 0., 0., 0.].
        roll : float, optional
            The roll of the camera about the camera point.
            The default is None.
        pitch : float, optional
            The pitch of the camera about the camera point.
            The default is None.
        yaw : float, optional
            The yaw of the camera about the camera point.
            The default is None.

        Returns
        -------
        None.

        """
        # If there is no visualizer, do not attempt to update it
        if not isinstance(self.vis, Visualizer):
            return
        
        # Apply the camera transform
        else:
            self.vis.transform_camera(scale = scale,
                                      translate = translate,
                                      wxyz_quaternion = wxyz_quaternion,
                                      roll=roll,
                                      pitch=pitch,
                                      yaw=yaw)
        
        
    def set_background(self,
                       top_color = None,
                       bot_color = None):
        """
        Set the top and bottom colors of the background of the scene.
    
        Parameters
        ----------
        top_color : array-like, shape (3,), optional
            The 0-255 color to apply to the top of the background.
            The default is None. If top_color is set to None, top_color
            is not altered.
        bot_color : array-like, shape (3,), optional
            The 0-255 color to apply to the bottom of the background.
            The default is None. If bot_color is set to None, bot_color
            is not altered.
    
        Returns
        -------
        None.
    
        """
        # If there is no visualizer, do not attempt to update it
        if not isinstance(self.vis, Visualizer):
            return
        
        # Apply the background colors
        else:
            self.vis.set_background(top_color = top_color,
                                    bot_color = bot_color)
     
        
    def set_spotlight(self,
                      on = False,
                      intensity = 1.0,
                      distance = 100.):
        """
        Sets the properties of the spotlight in the scene.

        Parameters
        ----------
        on : bool, optional
            A boolean flag that indicates whether the spotlight is on.
            The default is False.
        intensity : float (0. to 20.), optional
            The brightness of the spotlight. The default is 1.0.
        distance : float (0. to 100.), optional
            The distance from the origin of the spotlight. The default is 100..

        Returns
        -------
        None.

        """
        # If there is no visualizer, do not attempt to update it
        if not isinstance(self.vis, Visualizer):
            return
        
        # Apply the background colors
        else:
            self.vis.set_spotlight(on = on,
                                   intensity = intensity,
                                   distance = distance)
    
    
    def set_posx_pt_light(self,
                          on = False,
                          intensity = 1.0,
                          distance = 100.):
        """
        Sets the properties of the point light on the positive x axis
        in the scene.

        Parameters
        ----------
        on : bool, optional
            A boolean flag that indicates whether the light is on.
            The default is False.
        intensity : float (0. to 20.), optional
            The brightness of the light. The default is 1.0.
        distance : float (0. to 100.), optional
            The distance from the origin of the light. The default is 100..

        Returns
        -------
        None.

        """
        # If there is no visualizer, do not attempt to update it
        if not isinstance(self.vis, Visualizer):
            return
        
        # Apply the background colors
        else:
            self.vis.set_posx_pt_light(on = on,
                                       intensity = intensity,
                                       distance = distance)
            
            
    def set_negx_pt_light(self,
                          on = False,
                          intensity = 1.0,
                          distance = 100.):
        """
        Sets the properties of the point light on the negative x axis
        in the scene.

        Parameters
        ----------
        on : bool, optional
            A boolean flag that indicates whether the light is on.
            The default is False.
        intensity : float (0. to 20.), optional
            The brightness of the light. The default is 1.0.
        distance : float (0. to 100.), optional
            The distance from the origin of the light. The default is 100..

        Returns
        -------
        None.

        """
        # If there is no visualizer, do not attempt to update it
        if not isinstance(self.vis, Visualizer):
            return
        
        # Apply the background colors
        else:
            self.vis.set_negx_pt_light(on = on,
                                       intensity = intensity,
                                       distance = distance)
            
            
    def set_ambient_light(self,
                          on = False,
                          intensity = 1.0):
        """
        Sets the properties of the ambient light of the scene.

        Parameters
        ----------
        on : bool, optional
            A boolean flag that indicates whether the light is on.
            The default is False.
        intensity : float (0. to 20.), optional
            The brightness of the light. The default is 1.0.

        Returns
        -------
        None.

        """
        # If there is no visualizer, do not attempt to update it
        if not isinstance(self.vis, Visualizer):
            return
        
        # Apply the background colors
        else:
            self.vis.set_ambient_light(on = on,
                                       intensity = intensity)
            
            
    def set_fill_light(self,
                          on = False,
                          intensity = 1.0):
        """
        Sets the properties of the fill light in the scene.

        Parameters
        ----------
        on : bool, optional
            A boolean flag that indicates whether the light is on.
            The default is False.
        intensity : float (0. to 20.), optional
            The brightness of the light. The default is 1.0.

        Returns
        -------
        None.

        """
        # If there is no visualizer, do not attempt to update it
        if not isinstance(self.vis, Visualizer):
            return
        
        # Apply the background colors
        else:
            self.vis.set_fill_light(on = on,
                                       intensity = intensity)
            
            
    def add_plot_to_animator(self,
                             title=None,
                             x_label=None,
                             y_label=None,
                             color=None,
                             lock_x_range=False,
                             lock_y_range=False,
                             tail=None):
        
        # If there is no animator, do not attempt to add a plot to it
        if not isinstance(self.ani, Animator):
            return
        
        # Add the plot data to the plot
        plot_index = self.ani.add_plot(title=title,
                                       x_label=x_label,
                                       y_label=y_label,
                                       color=color,
                                       lock_x_range=lock_x_range,
                                       lock_y_range=lock_y_range,
                                       tail=tail)
        
        # Return the plot index
        return plot_index
        
        
    def set_plot_data(self,
                      plot_index,
                      x,
                      y):
        # If there is no animator, do not attempt to update it
        if not isinstance(self.ani, Animator):
            return
        
        # Update the plot
        self.ani.set_plot_data(plot_index,
                               x,
                               y)
        
            
    def step(self,
             real_time=True,
             update_vis=True):
        # Pause before step if running in real time
        if real_time:
            time_since_last_step = time.time() - self.last_step_time
            time_to_wait = self.dt - time_since_last_step
            if time_to_wait > 0.:
                time.sleep(time_to_wait)
            
        # Step the physics engine
        self.engine.stepSimulation()
        
        # Update the time
        self.last_step_time = time.time()
        self.time = self.time + self.dt
        self.times.append(self.time)
        
        # Update the visualizer if it exists
        if update_vis and isinstance(self.vis, Visualizer):
            for urdf_obj in self.urdf_objs:
                if urdf_obj.update_vis:
                    self.update_urdf_visual(urdf_obj)