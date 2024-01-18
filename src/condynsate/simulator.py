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
from condynsate.visualizer import Visualizer
from condynsate.animator import Animator
from condynsate.utils import format_path,format_RGB,wxyz_to_xyzw,xyzw_to_wxyz
from condynsate.utils import xyzw_quat_mult, get_rot_from_2_vecs
from condynsate.keyboard import Keys
from matplotlib import colormaps as cmaps
from pathlib import Path


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
        self.initial_conds = {}


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
                 visualization_fr = 30.,
                 animation=True,
                 animation_fr = 10.,
                 gravity=[0., 0., -9.81]):
        """
        Initializes an instance of the Simulator class.

        Parameters
        ----------
        visualization : bool, optional
            A boolean flag that indicates whether the simulation will be 
            visualized in meshcat. The default is True.
        visualization_fr : float, optional
            The frame rate (frames per second) at which the visualizer is
            updated. The default is 30..
        animation : bool, optional
            A boolean flag that indicates whether animated plots are created
            in real time. The default is True.
        animation_fr : float, optional
            The frame rate (frames per second) at which the animated plots are
            updated. The default is 10..
        gravity : array-like, shape (3,) optional
            The gravity vectory in m/s^2. The default is [0., 0., -9.81].

        Returns
        -------
        None.

        """
        # Note that the simulator is running
        self.paused = False
        
        # Connect to pybullet
        self.engine = bc.BulletClient(connection_mode=pybullet.DIRECT)
        
        # Configure gravity
        self.set_gravity(gravity)
        
        # Configure physics engine parameters
        self.time = 0.0
        self.dt = 0.01
        self.last_step_time = time.time()
        self.last_vis_time = time.time()
        self.visualization_fr = visualization_fr
        self.engine.setPhysicsEngineParameter(
            fixedTimeStep=self.dt,
            numSubSteps=4,
            restitutionVelocityThreshold=0.05,
            enableFileCaching=0)
        
        # Keep track of all urdfs loaded to simulator
        self.urdf_objs = []
        
        # Keep track of all the arrows loaded into the visualizer
        self.lin_arr_map = {}
        self.ccw_arr_map = {}
        
        # Create a visualizer
        if visualization:
            self.vis = Visualizer(grid_vis=False,axes_vis=False)
        else:
            self.vis=None
            
        # Create an animator
        if animation:
            self.ani = Animator(fr=animation_fr)
        else:
            self.ani=None
            
        # Start the keyboard listener
        self.keys = Keys()
        
        # Keep track if the simulation is done or not
        self.is_done = False
        
    
    ###########################################################################
    #PHYSICS ENGINE PROPERTY SETTERS
    ###########################################################################
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
        # Do nothing if paused
        if self.paused:
            return
        
        # Configure gravity
        self.engine.setGravity(gravity[0],
                               gravity[1],
                               gravity[2])
    
        
    ###########################################################################
    #PHYSICS URDF LOADING
    ###########################################################################
    def load_urdf(self,
                  urdf_path,
                  tex_path=None,
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
            The default is None which loads './examples/cmg_vis/check.png'.
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
            by the Visualizer each time step. The default is True.
            
        Returns
        -------
        urdf_obj : URDF_Obj
            A URDF_Obj that describes the urdf that was loaded into
            the simulation.
            
        """
        # Do nothing if paused
        if self.paused:
            return
        
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
        joint_map, link_map = self._make_joint_and_link_maps(urdf_id)
        
        # Create urdf_obj and adjust the default state of its joints
        urdf_obj = URDF_Obj(urdf_id,
                            joint_map,
                            link_map,
                            update_vis)
        
        # Make an initial condition map for the base
        initial_conds = {'position' : position,
                         'orientation' : orientation}
        
        # Record the initial conditions of each joint
        for key in joint_map:
            joint_id = joint_map[key]
            if joint_id == -1:
                continue
            state = self.get_joint_state(urdf_obj,key)
            joint_initial_cond = {'position' : state['position'],
                                  'velocity' : state['velocity']}
            initial_conds[joint_id] = joint_initial_cond
            
        # Send the initial conditions to the urdf object
        urdf_obj.initial_conds = initial_conds
        
        # Go through each joint and set friction and contact params
        for joint_name in joint_map:
            
            # Set the joint's friction parameters to very sticky
            self.set_joint_friction_params(urdf_obj=urdf_obj,
                                           joint_name=joint_name,
                                           lateral_friction=100.0,
                                           spinning_friction=0.0,
                                           rolling_friction=0.0)
            
            # Set the joint's contact parameters to model stiff metal contact
            self.set_joint_contact_params(urdf_obj=urdf_obj,
                                          joint_name=joint_name,
                                          restitution=0.0,
                                          contact_damping=-1.0,
                                          contact_stiffness=-1.0)
            
            # Set the linear and angular damping to 0 (eliminate drag)
            self.set_joint_lin_ang_damp(urdf_obj=urdf_obj,
                                        joint_name=joint_name,
                                        linear_damping=0.,
                                        angular_damping=0.)
            
            # Set damping of joints to 0 (eliminate joint friction)
            self.set_joint_damping(urdf_obj=urdf_obj,
                                   joint_name=joint_name,
                                   damping=0.)

            # If not a base joint
            if joint_map[joint_name]!=-1:
                # Disable velocity control
                self._disable_joint_vel_con(urdf_obj=urdf_obj,
                                            joint_name=joint_name)

                # Increase the maximum joint speed to 1000
                self._set_max_joint_vel(urdf_obj=urdf_obj,
                                        joint_name=joint_name,
                                        max_vel=1000.)

                # Enable the force and torque sensor
                self.set_joint_force_sensor(urdf_obj=urdf_obj,
                                            joint_name=joint_name,
                                            enable_sensor=True)

        # Add urdf objects to the visualizer if visualization is occuring
        if isinstance(self.vis, Visualizer):
            if tex_path == None:
                condynsate_path = Path(__file__).parents[0]
                condynsate_path = condynsate_path.absolute().as_posix()
                tex_path = condynsate_path + "/__assets__/check.png"
            self.add_urdf_to_visualizer(urdf_obj=urdf_obj,
                                        tex_path=tex_path)

        # Return the URDF_Obj
        self.urdf_objs.append(urdf_obj)
        return urdf_obj
    
    
    def _make_joint_and_link_maps(self,
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
            
            base_link_name = self.engine.getBodyInfo(urdf_id)[0]
            base_link_name = base_link_name.decode('UTF-8')
            link_map[base_link_name] = -1
        
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
    
    
    def _disable_joint_vel_con(self,
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
            
            
    def _set_max_joint_vel(self,
                            urdf_obj,
                            joint_name,
                            max_vel=1000.):
        # Gather information from urdf_obj
        urdf_id = urdf_obj.urdf_id
        joint_map = urdf_obj.joint_map
        
        # Set the max joint velocity
        if joint_name in joint_map:
            joint_id = joint_map[joint_name]
            self.engine.changeDynamics(urdf_id,
                                       joint_id,
                                       maxJointVelocity=max_vel)
            
    ###########################################################################
    #JOINT SETTERS
    ###########################################################################
    def set_joint_force_sensor(self,
                               urdf_obj,
                               joint_name,
                               enable_sensor):
        """
        Enables reaction force, moment, and applied torque to be calculated
        for a joint.

        Parameters
        ----------
        urdf_obj : URDF_Obj
            A URDF_Obj that contains that joint for which the sensor is set.
        joint_name : string
            The name of the joint whose sensor is set.
        enable_sensor : bool
            A boolean flag that indicates whether to enable or disable the
            force sensor.

        Returns
        -------
        None.

        """
        # Do nothing if paused
        if self.paused:
            return
        
        # Gather information from urdf_obj
        urdf_id = urdf_obj.urdf_id
        joint_map = urdf_obj.joint_map
        
        # Set the joint velocity
        if joint_name in joint_map:
            joint_id = joint_map[joint_name]
            self.engine.enableJointForceTorqueSensor(urdf_id,
                                                     joint_id,
                                                     enable_sensor)
    
    
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
        # Do nothing if paused
        if self.paused:
            return
        
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
        # Do nothing if paused
        if self.paused:
            return
        
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
        # Do nothing if paused
        if self.paused:
            return
        
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
        # Do nothing if paused
        if self.paused:
            return
        
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
                           position=0.,
                           physics=False,
                           initial_cond=False,
                           color=False,
                           min_pos=None,
                           max_pos=None):
        """
        Sets the position of a joint of a urdf.

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
        physics : bool, optional
            A boolean flag that indicates whether physics based poisiton
            controller will be used to change joint position or whether
            the joint position will be reset immediately to the target
            position with zero end velocity. The default is False. 
        initial_cond : bool, optional
            A boolean flag that indicates whether the position set is an 
            initial condition of the system. If it is an initial condition,
            when the simulation is reset using backspace, the joint position
            will be set again.
        color : bool, optional
            A boolean flag that indicates whether to color the joint based on
            its position. The default is False.
        min_pos : float
            The minimum possible position. Used only for coloring. Value is 
            ignored if color is False. The default is None. Must be set to 
            a float value for coloring to be applied.
        max_pos : float
            The maximum possible position. Used only for coloring. Value is 
            ignored if color is False. The default is None. Must be set to 
            a float value for coloring to be applied.

        Returns
        -------
        None.

        """
        # Do nothing if paused
        if self.paused:
            return
        
        # Gather information from urdf_obj
        urdf_id = urdf_obj.urdf_id
        joint_map = urdf_obj.joint_map
        
        # Set the joint velocity
        if joint_name in joint_map:
            joint_id = [joint_map[joint_name]]
            
            # Ensure that we don't try to change a base joint
            if joint_id[0] < 0:
                return
            
            # Set the position
            if physics and not initial_cond:
                mode = self.engine.POSITION_CONTROL
                position = [position]
                self.engine.setJointMotorControlArray(urdf_id,
                                                      joint_id,
                                                      mode,
                                                      forces=[100.],
                                                      targetPositions=position)
            # Reset joint position
            else:
                self.engine.resetJointState(bodyUniqueId=urdf_id,
                                            jointIndex=joint_id[0],
                                            targetValue=position,
                                            targetVelocity=0.0)
                
            # Update the initial conditions
            if initial_cond:
                urdf_obj.initial_conds[joint_id[0]]['position'] = position
           
            # Color the link based on the position
            if color and min_pos!=None and max_pos!=None:
                self.set_color_from_pos(urdf_obj=urdf_obj,
                                        joint_name=joint_name,
                                        min_pos=min_pos, 
                                        max_pos=max_pos)
                
    
    def set_joint_velocity(self,
                          urdf_obj,
                          joint_name,
                          velocity=0.,
                          physics=False,
                          initial_cond=False,
                          color=False,
                          min_vel=-100.,
                          max_vel=100.):
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
        physics : bool, optional
            A boolean flag that indicates whether physics based velocity
            controller will be used to change joint velocity or whether
            the joint velocity will be reset immediately to the target
            velocity. The default is False. 
        initial_cond : bool, optional
            A boolean flag that indicates whether the velocity set is an 
            initial condition of the system. If it is an initial condition,
            when the simulation is reset using backspace, the joint velocity
            will be set again.
        color : bool, optional
            A boolean flag that indicates whether to color the joint based on
            its velocity. The default is False.
        min_vel : float
            The minimum possible velocity. Used only for coloring. Value is 
            ignored if color is False. The default is -100..
        max_vel : float
            The maximum possible velocity. Used only for coloring. Value is 
            ignored if color is False. The default is 100..
            
        Returns
        -------
        None.

        """
        # Do nothing if paused
        if self.paused:
            return
        
        # Gather information from urdf_obj
        urdf_id = urdf_obj.urdf_id
        joint_map = urdf_obj.joint_map
        
        # Set the joint velocity
        if joint_name in joint_map:
            joint_id = [joint_map[joint_name]]
            
            # Ensure that we don't try to change a base joint
            if joint_id[0] < 0:
                return
            
            # Set the velocity with physics
            if physics and not initial_cond:
                mode = self.engine.VELOCITY_CONTROL
                target = [velocity]
                self.engine.setJointMotorControlArray(urdf_id,
                                                      joint_id,
                                                      mode,
                                                      forces=[1000.],
                                                      targetVelocities=target)
                
            # Set the velocity without physics
            else:
                state = self.get_joint_state(urdf_obj=urdf_obj,
                                             joint_name=joint_name)
                curr_pos = state['position']
                self.engine.resetJointState(bodyUniqueId=urdf_id,
                                            jointIndex=joint_id[0],
                                            targetValue=curr_pos,
                                            targetVelocity=velocity)
                
            if initial_cond:
                urdf_obj.initial_conds[joint_id[0]]['velocity'] = velocity
                
            # Color the link based on the velocity
            if color:
                self.set_color_from_vel(urdf_obj=urdf_obj,
                                        joint_name=joint_name,
                                        min_vel=min_vel, 
                                        max_vel=max_vel)
    
    
    def set_joint_torque(self,
                         urdf_obj,
                         joint_name,
                         torque=0.,
                         show_arrow=False,
                         arrow_scale=0.1,
                         arrow_offset=0.0,
                         color=False,
                         min_torque=-1.,
                         max_torque=1.):
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
        show_arrow : bool, optional
            A boolean flag that indicates whether an arrow will be rendered
            on the link to visualize the applied torque. The default is False.
        arrow_scale : float, optional
            The scaling factor that determines the size of the arrow. The
            default is 0.1.
        arrow_offset : float, optional
            The amount to offset the drawn arrow along the joint axis.
            The default is 0.0.
        color : bool, optional
            A boolean flag that indicates whether the child link will be
            colored based on the applied torque. The default is False.
        min_torque : float, optional
            The minimum value of torque that can be applied. Used for link
            coloring. Does nothing if color is not enabled. The default is -1..
        max_torque : float, optional
            The maximum value of torque that can be applied. Used for link
            coloring. Does nothing if color is not enabled. The default is 1..
            
        Returns
        -------
        None.

        """
        # Do nothing if paused
        if self.paused:
            return
        
        # Gather information from urdf_obj
        urdf_id = urdf_obj.urdf_id
        joint_map = urdf_obj.joint_map
        
        # Get the joint_id that defines the joint
        if joint_name in joint_map:
            joint_id = joint_map[joint_name]
        else:
            return
        
        # Ensure that we don't try to change a base joint
        if joint_id < 0:
            return
        
        # If the arrow isn't meant to be visualized, hide it
        vis_exists = isinstance(self.vis, Visualizer)
        arr_exists = joint_name in self.ccw_arr_map
        if (not show_arrow) and vis_exists and arr_exists:
            arrow_name = str(self.ccw_arr_map[joint_name])
            path = Path(__file__).parents[0]
            path = path.absolute().as_posix()
            path = path + "/__assets__/arrow_ccw.stl"
            self.vis.set_link_color(urdf_name = "Torque Arrows",
                                    link_name = arrow_name,
                                    stl_path=path, 
                                    color = [0, 0, 0],
                                    transparent = True,
                                    e = 0.0)
        
        # Handle torque arrow visualization
        if show_arrow and isinstance(self.vis, Visualizer):
            # Get the orientation, in body coordinates, of the arrow based
            # on direction of torque
            axis = self.get_joint_axis(urdf_obj=urdf_obj,
                                       joint_name=joint_name)
            if torque>=0.:
                arrow_xyzw_in_body = get_rot_from_2_vecs([0,0,1], axis)
            else:
                arrow_xyzw_in_body = get_rot_from_2_vecs([0,0,-1], axis)
                
            # Get the child link of the joint to which torque is applied
            joint_index = list(urdf_obj.link_map.values()).index(joint_id)
            link_name = list(urdf_obj.link_map.keys())[joint_index]
                
            # Get the link state
            state = self.get_link_state(urdf_obj=urdf_obj,
                                        link_name=link_name)
            body_xyzw_in_world = state['orientation']
            body_xyzw_in_world = np.array(body_xyzw_in_world)
            
            # Get the arrow offset
            body_wxyz_axis = np.insert(np.array(axis), 0, 0.)
            wxyz_R = xyzw_to_wxyz(body_xyzw_in_world)
            wxyz_R_prime = np.array([wxyz_R[0],
                                     -wxyz_R[1],
                                     -wxyz_R[2],
                                     -wxyz_R[3]])
            r1 = xyzw_quat_mult(wxyz_R,body_wxyz_axis)
            world_wxyz_axis = -xyzw_quat_mult(r1, wxyz_R_prime)
            world_axis = np.array([world_wxyz_axis[1],
                                   world_wxyz_axis[2],
                                   world_wxyz_axis[3]])
            pos = state['position']
            pos = np.array(pos) + arrow_offset*np.array(world_axis)
            pos = tuple(pos.tolist())
            
            # Combine the two rotations
            xyzw_ori = xyzw_quat_mult(arrow_xyzw_in_body, body_xyzw_in_world)
            wxyz_ori = xyzw_to_wxyz(xyzw_ori)
                
            # Get the scale of the arrow based on the magnitude of the torque
            scale = arrow_scale*abs(torque)*np.array([1., 1., 0.1])
            scale = scale.tolist()
            
            # If the arrow already exists, only update its position and ori
            if link_name in self.ccw_arr_map:
                arrow_name = str(self.ccw_arr_map[link_name])
                path = Path(__file__).parents[0]
                path = path.absolute().as_posix()
                path = path + "/__assets__/arrow_ccw.stl"
                self.vis.set_link_color(urdf_name = "Torque Arrows",
                                        link_name = arrow_name,
                                        stl_path=path, 
                                        color = [0, 0, 0],
                                        transparent = False,
                                        opacity = 1.0)
                self.vis.apply_transform(urdf_name="Torque Arrows",
                                         link_name=arrow_name,
                                         scale=scale,
                                         translate=pos,
                                         wxyz_quaternion=wxyz_ori)
            
            # If the arrow is not already created, add it to the visualizer
            else:
                # Add the arrow to the linear arrow map
                self.ccw_arr_map[link_name] = len(self.ccw_arr_map)
                arrow_name = str(self.ccw_arr_map[link_name])
                
                # Add an arrow to the visualizer
                path = Path(__file__).parents[0]
                path = path.absolute().as_posix()
                path = path + "/__assets__/arrow_ccw.stl"
                self.vis.add_stl(urdf_name="Torque Arrows",
                                 link_name=arrow_name,
                                 stl_path=path,
                                 color = [0, 0, 0],
                                 transparent=False,
                                 opacity = 1.0,
                                 scale=scale,
                                 translate=pos,
                                 wxyz_quaternion=wxyz_ori)
        
        # Set the link color based on applied torque if option is selected.
        if color and isinstance(self.vis, Visualizer):
            self.set_color_from_torque(urdf_obj=urdf_obj,
                                       joint_name=joint_name,
                                       torque=torque,
                                       min_torque=min_torque,
                                       max_torque=max_torque)
        
        
        # Set the joint torque
        if joint_name in joint_map:
            joint_id = [joint_map[joint_name]]
            mode = self.engine.TORQUE_CONTROL
            torque = [torque]
            self.engine.setJointMotorControlArray(urdf_id,
                                                  joint_id,
                                                  mode,
                                                  forces=torque)
            
    ###########################################################################
    #JOINT GETTERS
    ###########################################################################
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
        state : dictionary with the following keys
            'position' : float
                The position value of this joint.
            'velocity' : float
                The velocity value of this joint.
            'reaction force' : list shape(3,)
                These are the joint reaction forces. Only read if a torque
                sensor is enabled for this joint.
            'reaction torque' : list shape(3,)
                These are the joint reaction torques. Only read if a torque
                sensor is enabled for this joint.
            'applied torque' : float
                This is the motor torque applied during the last
                stepSimulation. Note that this only applies in VELOCITY_CONTROL
                and POSITION_CONTROL. If you use TORQUE_CONTROL then the
                applied joint motor torque is exactly what you provide, so
                there is no need to report it separately.
            
        """
        # Get object id and joint id
        urdf_id = urdf_obj.urdf_id
        joint_map = urdf_obj.joint_map
        
        # Get the joint id
        if joint_name in joint_map:
            joint_id = [joint_map[joint_name]]
        else:
            return
        
        # Don't attempt to get the base state
        if joint_id == -1:
            return
        
        # Retrieve the joint states
        states = self.engine.getJointStates(urdf_id, joint_id)
        states = states[0]
        pos = states[0]
        vel = states[1]
        rxn = states[2]
        rxn_force = [rxn[0], rxn[1], rxn[2]]
        rxn_torque = [rxn[3], rxn[4], rxn[5]]
        applied_torque = states[3]
        
        # Make the state dictionary
        state = {'position' : pos,
                 'velocity' : vel,
                 'reaction force' : rxn_force,
                 'reaction torque' : rxn_torque,
                 'applied torque' : applied_torque}
        return state
    
    
    def get_joint_axis(self,
                        urdf_obj,
                        joint_name):
        """
        Get the joint axis in local frame.

        Parameters
        ----------
        urdf_obj : URDF_Obj
            A URDF_Obj that contains that joint whose axis is found.
        joint_name : string
            The name of the joint axis is returned.

        Returns
        -------
        axis : array-like, shape(3,)
            The joint axis in local frame.

        """
        # Gather information from urdf_obj
        urdf_id = urdf_obj.urdf_id
        joint_map = urdf_obj.joint_map
        
        # Set the joint velocity
        if joint_name in joint_map:
            joint_id = joint_map[joint_name]
            info = self.engine.getJointInfo(urdf_id,
                                            joint_id)
            axis = info[13]
            axis=np.array(axis).tolist()
            return axis
            
            
    ###########################################################################
    #LINK SETTERS
    ###########################################################################
    def set_link_mass(self,
                      urdf_obj,
                      link_name,
                      mass=0.,
                      color=False,
                      min_mass=None,
                      max_mass=None):
        """
        Sets the mass of a link in a urdf.

        Parameters
        ----------
        urdf_obj : URDF_Obj
            A URDF_Obj that contains that link whose mass is being set.
        link_name : string
            The name of the link whose mass is set. The link name is
            specified in the .urdf file.
        mass : float, optional
            The mass to set in kg. The default is 0..
        color : bool, optional
            A boolean flag that indicates whether to color the joint based on
            its velocity. The default is False.
        min_mass : float
            The minimum possible mass. Used only for coloring. Value is 
            ignored if color is False. The default is None. Must be set to 
            a float value for coloring to be applied.
        max_mass : float
            The maximum possible mass. Used only for coloring. Value is 
            ignored if color is False. The default is None. Must be set to 
            a float value for coloring to be applied.
            
        Returns
        -------
        None.

        """
        # Do nothing if paused
        if self.paused:
            return
        
        # Gather information from urdf_obj
        urdf_id = urdf_obj.urdf_id
        link_map = urdf_obj.link_map
        
        # Set the link mass
        if link_name in link_map:
            joint_id = link_map[link_name]
            self.engine.changeDynamics(urdf_id, joint_id, mass=mass)
            
            # Color the link based on the position
            if color and min_mass!=None and max_mass!=None:
                self.set_color_from_mass(urdf_obj=urdf_obj,
                                         link_name=link_name,
                                         min_mass=min_mass, 
                                         max_mass=max_mass)
                
                    
    ###########################################################################
    #LINK GETTERS
    ###########################################################################
    def get_link_state(self,
                       urdf_obj,
                       link_name):
        """
        Gets the rigid body position and orientation of a link in world
        cooridinates.

        Parameters
        ----------
        urdf_obj : URDF_Obj
            A URDF_Obj that contrains the link whose state is being measured.
        link_name : string
            The name of the link to measure.

        Returns
        -------
        state : dictionary with the following keys
            'position' : array-like, shape (3,)
                Cartesian position of center of mass.
            'orientation' : array-like, shape(4,)
                Cartesian orientation of center of mass, in quaternion
                [x,y,z,w]

        """
        # Get object id and link map
        urdf_id = urdf_obj.urdf_id
        link_map = urdf_obj.link_map
        
        # Get the link id
        if link_name in link_map:
            link_id = [link_map[link_name]]
        else:
            return
        
        # Retrieve the link states
        link_states = self.engine.getLinkStates(urdf_id, link_id)
        link_states = link_states[0]
        pos_in_world = link_states[0]
        ori_in_world = link_states[1]
        
        #  Make the dictionary
        state = {'position' : pos_in_world,
                 'orientation' : ori_in_world}
        return state
    
    
    def get_link_mass(self,
                      urdf_obj,
                      link_name):
        """
        Gets the current mass of a link.

        Parameters
        ----------
        urdf_obj : URDF_Obj
            A URDF_Obj that contains that link whose mass is measured.
        link_name : string
            The name of the link whose mass is measured. The link name is
            specified in the .urdf file.

        Returns
        -------
        mass : float
            The mass of the link in Kg. If link is not found, returns none.

        """
        # Gather information from urdf_obj
        urdf_id = urdf_obj.urdf_id
        link_map = urdf_obj.link_map
        
        # Ensure the link exists
        if not (link_name in link_map):
           return None
            
        # Get the mass
        link_id = link_map[link_name]
        info = self.engine.getDynamicsInfo(urdf_id,link_id)
        mass = info[0]
        return mass


    ###########################################################################
    #BODY GETTERS
    ###########################################################################
    def get_base_state(self,
                       urdf_obj,
                       body_coords=False):
        """
        Gets the rigid body states (position, orientation, linear velocity, and
        angular velocitiy) of the base link of a given urdf. 

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
        state : a dictionary with the following keys:
            'position' : array-like, shape (3,)
                The (x,y,z) world coordinates of the base of the urdf.
            'roll' : float
                The Euler angle roll of the base of the urdf
                that define the body's orientation in the world.
            'pitch' : float
                The Euler angle pitch of the base of the urdf
                that define the body's orientation in the world.
            'yaw' : float
                The Euler angle yaw of the base of the urdf
                that define the body's orientation in the world.
            'velocity' : array-like, shape (3,)
                The linear velocity of the base of the urdf in either world 
                coords or body coords.
            'angular velocity' : array-like, shape (3,)
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
            R_world_to_body = self.engine.getMatrixFromQuaternion(xyzw_ori)
            R_world_to_body = np.array(R_world_to_body)
            R_world_to_body = np.reshape(R_world_to_body, (3,3))
            
            # Get the body velocities in body coordinates
            vel_body = R_world_to_body @ vel_world
            ang_vel_body =  R_world_to_body @ ang_vel_world
            
            # Make the dictionary
            state = {'position' : pos,
                     'roll' : rpy[0],
                     'pitch' : rpy[1],
                     'yaw' : rpy[2],
                     'velocity' : vel_body,
                     'angular velocity' : ang_vel_body}
            return state
        
        # Make the dictionary
        state = {'position' : pos,
                 'roll' : rpy[0],
                 'pitch' : rpy[1],
                 'yaw' : rpy[2],
                 'velocity' : vel_world,
                 'angular velocity' : ang_vel_world}
        return state
    
    
    def get_center_of_mass(self,
                           urdf_obj):
        """
        Get the center of mass of a body .

        Parameters
        ----------
        urdf_obj : URDF_Obj
            A URDF_Obj whose center of mass is calculated.

        Returns
        -------
        com : array-like, shape(3,)
            The cartesian coordinates of the center of mass of the body in
            world coordinates.

        """
        # Gather information from urdf_obj
        urdf_id = urdf_obj.urdf_id
        link_map = urdf_obj.link_map
        
        # Go through each link and update body com
        weighted_pos = np.array([0., 0., 0.])
        total_mass = 0.
        for link_name in link_map:
            link_id = link_map[link_name]
            
            # Get the mass of each link
            mass = self.engine.getDynamicsInfo(urdf_id,link_id)[0]
            
            # Get the center of mass of each link
            if link_id==-1:
                pos = self.engine.getBasePositionAndOrientation(urdf_id)[0]
            else:
                pos = self.engine.getLinkState(urdf_id, link_id)[0]
            pos = np.array(pos)
            
            # Update the center of mass parametersd
            weighted_pos = weighted_pos + mass*pos
            total_mass = total_mass + mass
            
        # Calculate the com
        if total_mass > 0.:
            com = weighted_pos / total_mass
        else:
            com = np.array([0., 0., 0.])
        return com
    
    
    ###########################################################################
    #EXTERNAL FORCE AND TORQUE APPLICATION
    ###########################################################################   
    def apply_force_to_link(self,
                            urdf_obj,
                            link_name,
                            force,
                            show_arrow=False,
                            arrow_scale=0.4,
                            arrow_offset=0.0):
        """
        Applies an external force the to center of a specified link of a urdf.

        Parameters
        ----------
        urdf_obj : URDF_Obj
            A URDF_Obj that contains that link to which the force is applied.
        link_name : string
            The name of the link to which the force is applied.
            The link name is specified in the .urdf file.
        force : array-like, shape (3,)
            The force vector in body coordinates to apply to the link.
        show_arrow : bool, optional
            A boolean flag that indicates whether an arrow will be rendered
            on the link to visualize the applied force. The default is False.
        arrow_scale : float, optional
            The scaling factor that determines the size of the arrow. The
            default is 0.4.
        arrow_offset : float, optional
            The amount by which the drawn force arrow will be offset from the
            center of mass along the direction of the applied force. The 
            default is 0.0.
            
        Returns
        -------
        None.

        """
        # Do nothing if paused
        if self.paused:
            return
        
        # Gather information from urdf_obj
        urdf_id = urdf_obj.urdf_id
        link_map = urdf_obj.link_map
        
        # Get the joint_id that defines the link
        if link_name in link_map:
            joint_id = link_map[link_name]
        else:
            return
        
        # Draw the force arrow
        self._apply_force_arrow(urdf_obj=urdf_obj,
                                link_name=link_name,
                                force=force,
                                show_arrow=show_arrow,
                                arrow_scale=arrow_scale,
                                arrow_offset=arrow_offset)
        
        # Apply the external force
        self.engine.applyExternalForce(urdf_id,
                                       joint_id,
                                       force,
                                       [0., 0., 0.],
                                       flags=self.engine.LINK_FRAME)
                
        
    def apply_force_to_com(self,
                           urdf_obj,
                           force,
                           show_arrow=False,
                           arrow_scale=0.4,
                           arrow_offset=0.0):
        """
        Applies an external force to the center of mass of the body.

        Parameters
        ----------
        urdf_obj : URDF_Obj
            A URDF_Obj to which the force is applied.
        force : array-like, shape (3,)
            The force vector in world coordinates to apply to the body.
        show_arrow : bool, optional
            A boolean flag that indicates whether an arrow will be rendered
            on the com to visualize the applied force. The default is False.
        arrow_scale : float, optional
            The scaling factor that determines the size of the arrow. The
            default is 0.4.
        arrow_offset : float, optional
            The amount by which the drawn force arrow will be offset from the
            center of mass along the direction of the applied force. The 
            default is 0.0.

        Returns
        -------
        None.

        """
        # Do nothing if paused
        if self.paused:
            return
        
        # Gather information from urdf_obj
        urdf_id = urdf_obj.urdf_id
        link_map = urdf_obj.link_map
        
        # Get the highest link in the body tree
        highest_link_id = min(link_map.values())
        
        # Get the center of mass of the body in world cooridnates
        com = self.get_center_of_mass(urdf_obj)
        if np.linalg.norm(force) != 0:
            force_dirn = np.array(force) / np.linalg.norm(force)
        else:
            force_dirn = np.array([0., 0., 0.])
        pos = com + arrow_offset*force_dirn
        pos = tuple(pos.tolist())
        
        # If the arrow isn't meant to be visualized, hide it
        vis_exists = isinstance(self.vis, Visualizer)
        arr_exists = 'COM' in self.lin_arr_map
        if (not show_arrow) and vis_exists and arr_exists:
            arrow_name = str(self.lin_arr_map['COM'])
            path = Path(__file__).parents[0]
            path = path.absolute().as_posix()
            path = path + "/__assets__/arrow_lin.stl"
            self.vis.set_link_color(urdf_name = "Force Arrows",
                                    link_name = arrow_name,
                                    stl_path=path, 
                                    color = [0, 0, 0],
                                    transparent = True,
                                    opacity = 0.0)
        
        # Handle force arrow visualization
        if show_arrow and isinstance(self.vis, Visualizer):
            # Get the orientation of the force arrow
            xyzw_ori = get_rot_from_2_vecs([0,0,1], force)
            wxyz_ori = xyzw_to_wxyz(xyzw_ori)
            
            # Get the scale of the arrow based on the magnitude of the force
            scale = arrow_scale*np.linalg.norm(force)*np.array([1., 1., 1.])
            scale=scale.tolist()
            
            # If the arrow already exists, only update its position and ori
            if 'COM' in self.lin_arr_map:
                arrow_name = str(self.lin_arr_map['COM'])
                path = Path(__file__).parents[0]
                path = path.absolute().as_posix()
                path = path + "/__assets__/arrow_lin.stl"
                self.vis.set_link_color(urdf_name = "Force Arrows",
                                        link_name = arrow_name,
                                        stl_path=path, 
                                        color = [0, 0, 0],
                                        transparent = False,
                                        opacity = 1.0)
                self.vis.apply_transform(urdf_name="Force Arrows",
                                         link_name=arrow_name,
                                         scale=scale,
                                         translate=pos,
                                         wxyz_quaternion=wxyz_ori)
            
            # If the arrow is not already created, add it to the visualizer
            else:
                # Add the arrow to the linear arrow map
                self.lin_arr_map['COM'] = len(self.lin_arr_map)
                arrow_name = str(self.lin_arr_map['COM'])
                path = Path(__file__).parents[0]
                path = path.absolute().as_posix()
                path = path + "/__assets__/arrow_lin.stl"
                
                # Add an arrow to the visualizer
                self.vis.add_stl(urdf_name="Force Arrows",
                                 link_name=arrow_name,
                                 stl_path=path,
                                 color = [0, 0, 0],
                                 transparent=False,
                                 opacity = 1.0,
                                 scale=scale,
                                 translate=pos,
                                 wxyz_quaternion=wxyz_ori)
        
        # Apply a force to the highest link at the center of mass of the body
        self.engine.applyExternalForce(urdf_id,
                                       highest_link_id,
                                       force,
                                       com,
                                       flags=self.engine.WORLD_FRAME)
        
        
    def apply_external_torque(self,
                              urdf_obj,
                              torque,
                              show_arrow=False,
                              arrow_scale=0.1,
                              arrow_offset=0.0):
        """
        Applies an external torque to the center of mass of the body.

        Parameters
        ----------
        urdf_obj : URDF_Obj
            A URDF_Obj to which the torque is applied.
        torque : array-like, shape(3,)
            The torque vector in world coordinates to apply to the body.
        show_arrow : bool, optional
            A boolean flag that indicates whether an arrow will be rendered
            on the com to visualize the applied torque. The default is False.
        arrow_scale : float, optional
            The scaling factor that determines the size of the arrow. The
            default is 0.1.
        arrow_offset : float, optional
            The amount by which the drawn torque arrow will be offset from the
            center of mass along the direction of the applied torque. The 
            default is 0.0.

        Returns
        -------
        None.

        """
        # Do nothing if paused
        if self.paused:
            return
        
        # Gather information from urdf_obj
        urdf_id = urdf_obj.urdf_id
        link_map = urdf_obj.link_map
        
        # Get the highest link in the body tree
        highest_link_id = min(link_map.values())
        
        # Get the center of mass of the body in world cooridnates
        com = self.get_center_of_mass(urdf_obj)
        torque_dirn = np.array(torque) / np.linalg.norm(torque)
        pos = com + arrow_offset*torque_dirn
        pos = tuple(pos.tolist())
        
        # If the arrow isn't meant to be visualized, hide it
        vis_exists = isinstance(self.vis, Visualizer)
        arr_exists = 'COM' in self.ccw_arr_map
        if (not show_arrow) and vis_exists and arr_exists:
            arrow_name = str(self.lin_arr_map['COM'])    
            path = Path(__file__).parents[0]
            path = path.absolute().as_posix()
            path = path + "/__assets__/arrow_ccw.stl"
            self.vis.set_link_color(urdf_name = "Torque Arrows",
                                    link_name = arrow_name,
                                    stl_path=path, 
                                    color = [0, 0, 0],
                                    transparent = True,
                                    opacity = 0.0)
        
        # Handle force arrow visualization
        if show_arrow and isinstance(self.vis, Visualizer):
            # Get the orientation of the force arrow
            xyzw_ori = get_rot_from_2_vecs([0,0,1], torque)
            wxyz_ori = xyzw_to_wxyz(xyzw_ori)
            
            # Get the scale of the arrow based on the magnitude of the force
            scale = arrow_scale*np.linalg.norm(torque)*np.array([1., 1., 0.1])
            scale=scale.tolist()
            
            # If the arrow already exists, only update its position and ori
            if 'COM' in self.ccw_arr_map:
                arrow_name = str(self.ccw_arr_map['COM'])
                path = Path(__file__).parents[0]
                path = path.absolute().as_posix()
                path = path + "/__assets__/arrow_ccw.stl"
                self.vis.set_link_color(urdf_name = "Torque Arrows",
                                        link_name = arrow_name,
                                        stl_path=path, 
                                        color = [0, 0, 0],
                                        transparent = False,
                                        opacity = 1.0)
                self.vis.apply_transform(urdf_name="Torque Arrows",
                                         link_name=arrow_name,
                                         scale=scale,
                                         translate=pos,
                                         wxyz_quaternion=wxyz_ori)
            
            # If the arrow is not already created, add it to the visualizer
            else:
                # Add the arrow to the linear arrow map
                self.ccw_arr_map['COM'] = len(self.ccw_arr_map)
                arrow_name = str(self.ccw_arr_map['COM'])
                path = Path(__file__).parents[0]
                path = path.absolute().as_posix()
                path = path + "/__assets__/arrow_ccw.stl"
                
                # Add an arrow to the visualizer
                self.vis.add_stl(urdf_name="Torque Arrows",
                                 link_name=arrow_name,
                                 stl_path=path,
                                 color = [0, 0, 0],
                                 transparent=False,
                                 opacity = 1.0,
                                 scale=scale,
                                 translate=pos,
                                 wxyz_quaternion=wxyz_ori)
        
        # Apply a force to the highest link at the center of mass of the body
        self.engine.applyExternalTorque(urdf_id,
                                        highest_link_id,
                                        torque,
                                        flags=self.engine.WORLD_FRAME)
        
        
    def _apply_force_arrow(self,
                           urdf_obj,
                           link_name,
                           force,
                           show_arrow,
                           arrow_scale,
                           arrow_offset):
        """
        Draws a body coordinate force arrow based on force applied to a link.

        Parameters
        ----------
        urdf_obj : URDF_Obj
            A URDF_Obj that contains that link to which the force is applied.
        link_name : string
            The name of the link to which the force is applied.
            The link name is specified in the .urdf file.
        force : array-like, shape (3,)
            The force vector in body coordinates to apply to the link.
        show_arrow : bool
            A boolean flag that indicates whether an arrow will be rendered
            on the link to visualize the applied force
        arrow_scale : float
            The scaling factor that determines the size of the arrow.
        arrow_offset : float
            The amount by which the drawn force arrow will be offset from the
            center of mass along the direction of the applied force.
            
        Returns
        -------
        None.

        """
        # If the arrow isn't meant to be visualized, hide it
        vis_exists = isinstance(self.vis, Visualizer)
        arr_exists = link_name in self.lin_arr_map
        if (not show_arrow) and vis_exists and arr_exists:
            arrow_name = str(self.lin_arr_map[link_name])
            path = Path(__file__).parents[0]
            path = path.absolute().as_posix()
            path = path + "/__assets__/arrow_lin.stl"
            self.vis.set_link_color(urdf_name = "Force Arrows",
                                    link_name = arrow_name,
                                    stl_path=path, 
                                    color = [0, 0, 0],
                                    transparent = True,
                                    opacity = 0.0)
        
        # Handle force arrow visualization
        if show_arrow and isinstance(self.vis, Visualizer):
            
            # Get the orientation, in body coordinates, of the arrow based
            # on direction of force
            arrow_xyzw_in_body = get_rot_from_2_vecs([0,0,1], force)
            
            # Get the link state
            state = self.get_link_state(urdf_obj=urdf_obj,
                                        link_name=link_name)
            pos = state['position']
            if np.linalg.norm(force) != 0:
                force_dirn = np.array(force) / np.linalg.norm(force)
            else:
                force_dirn = np.array([0., 0., 0.])
            pos = np.array(pos) + arrow_offset*force_dirn
            pos = tuple(pos.tolist())
            body_xyzw_in_world = state['orientation']
            body_xyzw_in_world = np.array(body_xyzw_in_world)
            
            # Combine the two rotations
            xyzw_ori = xyzw_quat_mult(arrow_xyzw_in_body, body_xyzw_in_world)
            wxyz_ori = xyzw_to_wxyz(xyzw_ori)
            
            # Get the scale of the arrow based on the magnitude of the force
            scale = arrow_scale*np.linalg.norm(force)*np.array([1., 1., 1.])
            scale=scale.tolist()
            
            # If the arrow already exists, only update its position and ori
            if link_name in self.lin_arr_map:
                arrow_name = str(self.lin_arr_map[link_name])
                path = Path(__file__).parents[0]
                path = path.absolute().as_posix()
                path = path + "/__assets__/arrow_lin.stl"
                self.vis.set_link_color(urdf_name = "Force Arrows",
                                        link_name = arrow_name,
                                        stl_path=path, 
                                        color = [0, 0, 0],
                                        transparent = False,
                                        opacity = 1.0)
                self.vis.apply_transform(urdf_name="Force Arrows",
                                         link_name=arrow_name,
                                         scale=scale,
                                         translate=pos,
                                         wxyz_quaternion=wxyz_ori)
            
            # If the arrow is not already created, add it to the visualizer
            else:
                # Add the arrow to the linear arrow map
                self.lin_arr_map[link_name] = len(self.lin_arr_map)
                arrow_name = str(self.lin_arr_map[link_name])
                path = Path(__file__).parents[0]
                path = path.absolute().as_posix()
                path = path + "/__assets__/arrow_lin.stl"
                
                # Add an arrow to the visualizer
                self.vis.add_stl(urdf_name="Force Arrows",
                                 link_name=arrow_name,
                                 stl_path=path,
                                 color = [0, 0, 0],
                                 transparent=False,
                                 opacity = 1.0,
                                 scale=scale,
                                 translate=pos,
                                 wxyz_quaternion=wxyz_ori)
                
                
    ###########################################################################
    #URDF VISUALIZATION MANIPULATION
    ###########################################################################
    def add_urdf_to_visualizer(self,
                               urdf_obj,
                               tex_path=None):
        """
        Adds urdfs to the Visualizer. URDFs describe systems assembles from
        .stl and .obj links.

        Parameters
        ----------
        vis : Visualizer
            The Visualizer to which the urdf is added.
        urdf_obj : URDF_Obj
            A URDF_Obj that will be added to the Visualizer.
        tex_path : string, optional
            The path pointing towards a texture file. This texture is applied
            to all .obj links in the urdf.

        Returns
        -------
        None.

        """
        # Do nothing if paused
        if self.paused:
            return
        
        # If there is no visualizer, do not attempt to update it
        if not isinstance(self.vis, Visualizer):
            return
        
        # Extract the visual data from the urdf object in the simulator
        paths,names,scales,colors,poss,oris = self._get_urdf_vis_dat(urdf_obj)
        
        # Make the URDF name and format the texture path
        urdf_name = str(urdf_obj.urdf_id)
        if tex_path == None:
            condynsate_path = Path(__file__).parents[0]
            condynsate_path = condynsate_path.absolute().as_posix()
            tex_path = condynsate_path + "/__assets__/check.png"
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

    
    def _update_urdf_visual(self,
                            urdf_obj):
        """
        Updates the positions of dynamic links in the Visualizer.

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
        paths,names,scales,colors,poss,oris = self._get_urdf_vis_dat(urdf_obj)
        urdf_name = str(urdf_obj.urdf_id)
        
        # Go through all links in urdf object and update their position
        for i in range(len(paths)):
            link_name = names[i]
            self.vis.apply_transform(urdf_name=urdf_name,
                                     link_name=link_name,
                                     scale=scales[i],
                                     translate=poss[i],
                                     wxyz_quaternion=oris[i])
    
    
    def _get_urdf_vis_dat(self,
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
                base_link_name = self.engine.getBodyInfo(urdf_id)[0]
                base_link_name = base_link_name.decode('UTF-8')
                link_names.append(base_link_name)
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
    
    
    ###########################################################################
    #VISUALIZATION COLOR MANIPULATION
    ###########################################################################
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
            specified in the .urdf file.
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
        # Do nothing if paused
        if self.paused:
            return
        
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


    def set_color_from_pos(self,
                           urdf_obj,
                           joint_name,
                           min_pos,
                           max_pos):
        """
        Sets the color of the child link of a specified joint based on the 
        position of the joint.

        Parameters
        ----------
        urdf_obj : URDF_Obj
            A URDF_Obj that contains that joint whose position is measured.
        joint_name : string
            The name of the joint whose position is used to set the link color.
            The joint name is specified in the .urdf file.
        min_pos : float
            The minimum possible position of the given joint.
        max_pos : float
            The maximum possible position of the given joint.

        Returns
        -------
        None.

        """
        # Do nothing if paused
        if self.paused:
            return
        
        # Gather information from urdf_obj
        joint_map = urdf_obj.joint_map
    
        # If the joint is invalid, do nothing
        if (joint_name in joint_map):
            joint_id = joint_map[joint_name]
        else:
            return
        
        # If there is no visualizer, do not color
        if not isinstance(self.vis, Visualizer):
            return    
        
        # Get the joint position
        state = self.get_joint_state(urdf_obj=urdf_obj,
                                     joint_name=joint_name)
        pos = state['position']
        
        # Calculate the position saturation and get the associated color
        sat = np.clip((pos - min_pos) / (max_pos - min_pos), 0.0, 1.0)
        col = cmaps['coolwarm'](round(255*sat))[0:3]
        col = format_RGB(col,
                         range_to_255=True)
        
        # Get the child link of the joint from which pos is measured
        joint_index = list(urdf_obj.link_map.values()).index(joint_id)
        link_name = list(urdf_obj.link_map.keys())[joint_index]
        
        # Set link color
        self.set_link_color(urdf_obj=urdf_obj,
                            link_name=link_name,
                            color=col)
        
        
    def set_color_from_vel(self,
                           urdf_obj,
                           joint_name,
                           min_vel=-100.,
                           max_vel=100.):
        """
        Sets the color of the child link of a specified joint based on the 
        velocity of the joint.

        Parameters
        ----------
        urdf_obj : URDF_Obj
            A URDF_Obj that contains that joint whose velocity is measured.
        joint_name : string
            The name of the joint whose velocity is used to set the link color.
            The joint name is specified in the .urdf file.
        min_vel : float, optional
            The minimum possible velocity of the given joint. The default is
            -100.. Unless otherwise set, PyBullet does not allow joint
            velocities to exceed a magnitude of 100.
        max_vel : float, optional
            The maximum possible velocity of the given joint. The default is
            100.. Unless otherwise set, PyBullet does not allow joint
            velocities to exceed a magnitude of 100.

        Returns
        -------
        None.

        """
        # Do nothing if paused
        if self.paused:
            return
        
        # Gather information from urdf_obj
        joint_map = urdf_obj.joint_map
    
        # If the joint is invalid, do nothing
        if (joint_name in joint_map):
            joint_id = joint_map[joint_name]
        else:
            return
        
        # If there is no visualizer, do not color
        if not isinstance(self.vis, Visualizer):
            return    
        
        # Get the joint velocity
        state = self.get_joint_state(urdf_obj=urdf_obj,
                                     joint_name=joint_name)
        vel = state['velocity']
        
        # Calculate the velocity saturation and get the associated color
        sat = np.clip((vel - min_vel) / (max_vel - min_vel), 0.0, 1.0)
        col = cmaps['coolwarm'](round(255*sat))[0:3]
        col = format_RGB(col,
                         range_to_255=True)
        
        # Get the child link of the joint from which vel is measured
        joint_index = list(urdf_obj.link_map.values()).index(joint_id)
        link_name = list(urdf_obj.link_map.keys())[joint_index]
        
        # Set link color
        self.set_link_color(urdf_obj=urdf_obj,
                            link_name=link_name,
                            color=col)
    
    
    def set_color_from_torque(self,
                              urdf_obj,
                              joint_name,
                              torque,
                              min_torque=-1.,
                              max_torque=1.):
        """
        Sets the color of the child link of a specified joint based on the 
        torque applied to the joint.

        Parameters
        ----------
        urdf_obj : URDF_Obj
            A URDF_Obj that contains that joint whose torque is measured.
        joint_name : string
            The name of the joint whose torque is used to set the link color.
            The joint name is specified in the .urdf file.
        torque : float
            The torque applied to the joint.
        min_torque : float, optional
            The minimum possible torque to apply to the joint. The default is
            -1..
        max_torque : float, optional
            The maximum possible torque to apply to the joint. The default is
            
            1..

        Returns
        -------
        None.

        """
        # Do nothing if paused
        if self.paused:
            return
        
        # Gather information from urdf_obj
        joint_map = urdf_obj.joint_map
    
        # If the joint is invalid, do nothing
        if (joint_name in joint_map):
            joint_id = joint_map[joint_name]
        else:
            return
        
        # If there is no visualizer, do not color
        if not isinstance(self.vis, Visualizer):
            return    
        
        # Calculate the torque saturation and get the associated color
        sat = (torque - min_torque) / (max_torque - min_torque)
        sat = np.clip(sat, 0.0, 1.0)
        col = cmaps['coolwarm'](round(255*sat))[0:3]
        col = format_RGB(col,
                         range_to_255=True)
        
        # Get the child link of the joint from which torque is measured
        joint_index = list(urdf_obj.link_map.values()).index(joint_id)
        link_name = list(urdf_obj.link_map.keys())[joint_index]
        
        # Set link color
        self.set_link_color(urdf_obj=urdf_obj,
                            link_name=link_name,
                            color=col)
        
        
    def set_color_from_mass(self,
                            urdf_obj,
                            link_name,
                            min_mass,
                            max_mass):
        """
        Sets the color of a link based on its mass.

        Parameters
        ----------
        urdf_obj : URDF_Obj
            A URDF_Obj that contains that joint whose torque is measured.
        link_name : string
            The name of the link whose mass is used to set the link color.
            The link name is specified in the .urdf file.
        min_mass : float, optional
            The minimum possible mass of the link.
        max_mass : float, optional
            The maximum possible mass of the link.

        Returns
        -------
        None.

        """
        # Do nothing if paused
        if self.paused:
            return
        
        # Gather information from urdf_obj
        link_map = urdf_obj.link_map
    
        # If the link is invalid, do nothing
        if not (link_name in link_map):
            return
        
        # If there is no visualizer, do not color
        if not isinstance(self.vis, Visualizer):
            return    
        
        # Get the mass
        mass = self.get_link_mass(urdf_obj=urdf_obj,
                                  link_name=link_name)
        
        # Calculate the mass saturation and get the associated color
        sat = (mass - min_mass) / (max_mass - min_mass)
        sat = np.clip(sat, 0.0, 1.0)
        col = cmaps['binary'](round(255*sat))[0:3]
        col = format_RGB(col,
                         range_to_255=True)
        
        # Set link color
        self.set_link_color(urdf_obj=urdf_obj,
                            link_name=link_name,
                            color=col)
        
        
    ###########################################################################
    #VISUALIZATION SCENE MANIPULATION
    ###########################################################################
    def transform_camera(self,
                         scale = [1., 1., 1.],
                         translate = [0., 0., 0.],
                         wxyz_quaternion = [1., 0., 0., 0.],
                         roll=None,
                         pitch=None,
                         yaw=None):
        """
        Transforms the position, orientation, and scale of the Visualizer's
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
        # Do nothing if paused
        if self.paused:
            return
        
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
        Set the top and bottom colors of the background of the Visualizer.
    
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
        # Do nothing if paused
        if self.paused:
            return
        
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
        Sets the properties of the spotlight in the Visualizer.

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
        # Do nothing if paused
        if self.paused:
            return
        
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
        in the Visualizer.

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
        # Do nothing if paused
        if self.paused:
            return
        
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
        in the Visualizer.

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
        # Do nothing if paused
        if self.paused:
            return
        
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
        Sets the properties of the ambient light of the Visualizer.

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
        # Do nothing if paused
        if self.paused:
            return
        
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
        Sets the properties of the fill light in the Visualizer.

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
        # Do nothing if paused
        if self.paused:
            return
        
        # If there is no visualizer, do not attempt to update it
        if not isinstance(self.vis, Visualizer):
            return
        
        # Apply the background colors
        else:
            self.vis.set_fill_light(on = on,
                                       intensity = intensity)
        
        
    ###########################################################################
    #ANIMATOR MANIPULATION
    ###########################################################################
    def add_subplot(self,
                    n_artists=1,
                    subplot_type='line',
                    title=None,
                    x_label=None,
                    y_label=None,
                    x_lim=[None, None],
                    y_lim=[None, None],
                    h_zero_line=False,
                    v_zero_line=False,
                    colors=None,
                    labels=None,
                    line_widths=None,
                    line_styles=None,
                    tail=None):
        """
        Adds a subplot to the animator. This function needs to be called to 
        define a subplot before data can be added to that plot.

        Parameters
        ----------
        n_artists : int, optional
            The number of artists that draw on the subplot
            The default is 1.
        subplot_type: either 'line' or 'bar', optional
            The type of plot. May either be 'line' or 'bar'. The default
            is 'line'.
        title : string, optional
            The title of the plot. Will be written above the plot when
            rendered. The default is None.
        x_label : string, optional
            The label to apply to the x axis. Will be written under the subplot
            when rendered. The default is None.
        y_label : string, optional
            The label to apply to the y axis. Will be written to the left of
            the subplot when rendered. The default is None.
        x_lim : [float, float], optional
            The limits to apply to the x axis of the subplot. A value of None
            will apply automatically updating limits to the corresponding
            bound of the axis. For example [None, 10.] will fix the upper
            bound to exactly 10, but the lower bound will freely change to
            show all data.The default is [None, None].
        y_lim : [float, float], optional
            The limits to apply to the y axis of the subplot. A value of None
            will apply automatically updating limits to the corresponding
            bound of the axis. For example [None, 10.] will fix the upper
            bound to exactly 10, but the lower bound will freely change to
            show all data.The default is [None, None].
        h_zero_line : boolean, optional
            A boolean flag that indicates whether a horizontal line will be
            drawn on the y=0 line. The default is false
        v_zero_line : boolean, optional
            A boolean flag that indicates whether a vertical line will be
            drawn on the x=0 line. The default is false
        colors : list of matplotlib color string, optional
            A list of the color each artist draws in. Must have length
            n_artists. If n_artists = 1, has the form ['COLOR']. When None,
            all artists will default to drawing in black. The default is None.
        labels : list of strings, optional
            A list of the label applied to each artist. For line charts, 
            the labels are shown in a legend in the top right of the plot. For
            bar charts, the labels are shown on the y axis next to their 
            corresponging bars. Must have length n_artists. If n_artists = 1,
            has the form ['LABEL']. When None, no labels will be made for any
            aritsts. The default is None.
        line_widths : list of floats, optional
            The line weigth each artist uses. For line plots, this is the
            width of the plotted line, for bar charts, this is the width of 
            the border around each bar. Must be length n_artists. If
            n_artists = 1, has the form [LINE_WIDTH]. When set to None,
            defaults to 1.0 for all lines. The default is None.
        line_styles : list of matplotlib line style string, optional
            The line style each artist uses. For line plots, this is the
            style of the plotted line, for bar charts, this argument is not
            used and therefore ignored. Must be length n_artists. If
            n_artists = 1, has the form ['LINE_STYLE']. When set to None,
            defaults to 'solid' for all lines. The default is None.
        tail : int, optional
            Specifies how many data points are used to draw a line. Only the
            most recently added data points are kept. Any data points added
            more than tail data points ago are discarded and not plotted. Only
            valid for line plots, and applied to all artists in the plot. For 
            bar plots, this argument is ignored and not used. A value of None
            means that no data is ever discarded and all data points added to
            the animator will be drawn. The default is None.
            
        Raises
        ------
        Exception
            At least one of the arguments colors, labels, line_widths, or 
            line_styles do not have length n_artists.
        TypeError
            At least one of the arguments colors, labels, line_widths, or 
            line_styles is not a list.
        
        Returns
        -------
        subplot_index : int
            A integer identifier that is unique to the subplot created. 
            This allows future interaction with this subplot (adding data
            points, etc.).
        artist_inds : tuple of ints
            A tuple of integer identifiers that are unique to the artist
            created. This allows future interaction with these artists (adding
            data points, etc.).
        """
        # If there is no animator, do not attempt to add a plot to it
        if not isinstance(self.ani, Animator):
            return
        
        # Add the plot data to the plot
        v1, v2 = self.ani.add_subplot(n_artists=n_artists,
                                      subplot_type=subplot_type,
                                      title=title,
                                      x_label=x_label,
                                      y_label=y_label,
                                      colors=colors,
                                      labels=labels,
                                      x_lim=x_lim,
                                      y_lim=y_lim,
                                      h_zero_line=h_zero_line,
                                      v_zero_line=v_zero_line,
                                      line_widths=line_widths,
                                      line_styles=line_styles,
                                      tail=tail)
        subplot_index = v1
        artist_inds = v2
        
        # Return the plot index
        return subplot_index, artist_inds
        
        
    def add_subplot_point(self,
                          subplot_index,
                          artist_index,
                          x=None,
                          y=None):
        """
        Adds a single data point to the plot. Data point is appended to the end
        of all previously plotted data points.

        Parameters
        ----------
        plot_index : int
            The subplot's unique identifier.
        artist_index : int
            The subplot's artist index to which the data is added.
        x : float, optional
            The x value of the data point added to the plot. The default
            value is None.
        y : float, optional
            The y value of the data point added to the plot. The default
            value is None.

        Returns
        -------
        None.

        """
        # Do nothing if paused
        if self.paused:
            return
        
        # If there is no animator, do not attempt to update it
        if not isinstance(self.ani, Animator):
            return
        
        # Update the plot
        self.ani.add_subplot_point(subplot_index=subplot_index,
                                   artist_index=artist_index,
                                   x=x,
                                   y=y)
        
        
    def reset_plots(self):
        """
        Removes all previously plotted data from all plots.

        Returns
        -------
        None.

        """
        # Do nothing if paused
        if self.paused:
            return
        
        # If there is no animator, do not attempt to update it
        if not isinstance(self.ani, Animator):
            return
        
        self.ani.reset_plots()
        
        
    def open_animator_gui(self):
        """
        Opens the Animator GUI with the specified plots. After the Animator
        is open, no more plots can be added; however, the plot data can still
        be set.

        Returns
        -------
        None.

        """
        # Open the animator figure window if it exists
        if isinstance(self.ani, Animator):
            self.ani.create_figure()
        
        
    ###########################################################################
    #SIMULATION EXECUTION
    ###########################################################################
    def is_pressed(self,
                   key):
        """
        Wrapper for the keyboard.Keys.is_pressed() function.
        Returns a boolean flag to indicate whether a desired key is pressed.
        The key may be alpha numeric or some special keys.

        Parameters
        ----------
        key : string
            The key to be detected. May be alpha numeric ("a", "A", "1", "!",
            "`", etc.) or some special keys. The special keys are as follows:
            "space", "enter", "backspace", "tab", "shift", "alt",
            "ctrl", and "esc". The following modifiers can also be used:
            "shift+", "alt+", and "ctrl+". Modifiers are added with the
            following format: "shift+a", "ctrl+a", "alt+a", "shift+ctrl+alt+a",
            etc. If "esc" is pressed, the keyboard listener will automatically
            stop and cannot be restarted.

        Returns
        -------
        bool
            A boolean flag to indicate whether the desired key is pressed.

        """
        return self.keys.is_pressed(key)
        
        
    def await_keypress(self,
                       key = "enter"):
        """
        Waits until a specified keystroke is recieved.
        When an Animator GUI is open, this function must be called to
        keep the GUI responsive. If a GUI is not present, this function is
        optional. 

        Parameters
        ----------
        key : string, optional
            The key string identifier. The default is "enter".

        Returns
        -------
        None.

        """
        # Update the visualizer if it exists
        if isinstance(self.vis, Visualizer):
            for urdf_obj in self.urdf_objs:
                if urdf_obj.update_vis:
                    self._update_urdf_visual(urdf_obj)
                    
        # Tell user what to do
        print("PRESS "+key.upper()+" TO START SIMULATION.")
        print("PRESS ESC TO QUIT.")
        print("PRESS SPACE TO PAUSE/RESUME SIMULATION.")
        print("PRESS BACKSPACE TO RESET SIMULATION.")
        while not self.is_pressed("enter"):
            # Ensure so the GUI remains interactive if simulation is suspended
            if isinstance(self.ani, Animator):
                self.ani.flush_events()
                
            # Termination condition
            if self.is_pressed("esc"):
                self.is_done = True
                print("QUITTING...")
                break
            
            # Note that the simulation is started
            if self.is_pressed("enter"):
                print("CONTINUING...")
      
      
    def iterate_val(self,
                    curr_val,
                    down_key,
                    up_key,
                    iter_val=0.0333,
                    min_val=-np.inf,
                    max_val=np.inf):
        """
        Iterates a current value by some amount every time this function is 
        called according to which keys are pressed. If no keys are pressed,
        no iteration occurs. If the down key is pressed, decrement the value.
        If the up key is pressed, increment the value. If both the up and 
        down keys are pressed, do nothing. The iterated value is clipped 
        between [min_val and max_val].

        Parameters
        ----------
        curr_val : numeric value
            The current value which will be incremented or decremented.
        down_key : string
            A Keyboard string that specifies what must be depressed for 
            decrementation to occur.
        up_key : string
            A Keyboard string that specifies what must be depressed for 
            incrementation to occur.
        iter_val : numeric value, optional
            The value by which the current value is either incremented
            or decremented. The default is 0.0333.
        min_val : numeric value, optional
            The minimum possible value that can be returned.
            The default is -np.inf.
        max_val : numeric value, optional
            The maximum possible value that can be returned.
            The default is np.inf.

        Returns
        -------
        new_val : numeric value
            The iterated value.

        """
        # Do nothing if the simulation is paused
        if self.paused:
            return curr_val

        # Do nothing if both up and down keys are pressed
        if self.is_pressed(down_key) and self.is_pressed(up_key):
            return curr_val

        # Set the new val to the current value
        new_val = curr_val

        # Listen to see if the down key is pressed
        if self.is_pressed(up_key):
            new_val = curr_val + iter_val
            if new_val > max_val:
                new_val = max_val
    
        # Listen to see if the up key is pressed
        elif self.is_pressed(down_key):
            new_val = curr_val - iter_val
            if new_val < min_val:
                new_val = min_val
    
        # Return the updated target value
        return new_val
      
        
    def reset(self):
        """
        Reset the physics state to the initial conditions.
        Reset the visualizer to the initial conditions.
        Reset the animator to the initial conditions

        Returns
        -------
        None.

        """
        # Do nothing if paused
        if self.paused:
            return
        
        # Note the simulation is no longer done and reset the time
        self.is_done = False
        self.time = 0.
        self.last_step_time = time.time()
        
        # Reset each urdf
        for urdf_obj in self.urdf_objs:
            
            # Reset the base position and orientation of all urdfs
            i = urdf_obj.urdf_id
            p = urdf_obj.initial_conds['position']
            o = urdf_obj.initial_conds['orientation']
            self.engine.resetBasePositionAndOrientation(bodyUniqueId=i,
                                                        posObj=p,
                                                        ornObj=o)
            
            # Reset each joint
            for joint_name in urdf_obj.joint_map.keys():
                
                # Ensure we aren't setting a base joint
                joint_id = urdf_obj.joint_map[joint_name]
                if joint_id!=-1:
                    
                    # Reset joint state to the initial conditions
                    pos = urdf_obj.initial_conds[joint_id]['position']
                    vel = urdf_obj.initial_conds[joint_id]['velocity']
                    self.engine.resetJointState(bodyUniqueId=urdf_obj.urdf_id,
                                                jointIndex=joint_id,
                                                targetValue=pos,
                                                targetVelocity=vel)
                    
                    # Set the joint torque to 0
                    self.set_joint_torque(urdf_obj,
                                          joint_name,
                                          torque=0.,
                                          show_arrow=False,
                                          color=False,)
        
        # Reset the plots
        self.reset_plots()
        
        
    def step(self,
             real_time=True,
             update_vis=True,
             update_ani=True,
             max_time=None):
        """
        Takes a single step of the simulation. In this step, the physics
        engine, the Visualizer (3D visualization of urdfs), and the Animator
        (2D animation of plots) are all updated.

        Parameters
        ----------
        real_time : bool, optional
            A boolean flag that indicates whether or not the step is taken in
            real time. If True, step() is suspended until the time since the
            last step is equal to 0.01 seconds (the fixed time step of the
            physics engine). If False, step() is run as quickly as possible.
            The default is True.
        update_vis : bool, optional
            A boolean flag that indicates whether the Visualizer is updated.
            The default is True.
        update_ani : bool, optional
            A boolean flag that indicates whether the Animator is updated.
            The default is True.
        max_time : float or None, optional
            The maximum amount of simulated seconds the simulator is allowed to
            run. If None, the simulation will run until "ESC" is pressed.

        Returns
        -------
        None.

        """
        # Collect keyboard IO for termination
        if self.is_pressed("esc"):
            self.is_done = True
            print("QUITTING...")
            return -1 # Return end code
        
        # Suspend if paused or resume if space is pressed
        if self.paused:
            time.sleep(0.05)
            if isinstance(self.ani, Animator):
                self.ani.flush_events()
            if self.is_pressed("space"):
                self.paused = False
                print("RESUME")
                time.sleep(0.2)
                return 1 # Return end pause code
            return 2 # Return paused code
            
        # Reset upon request
        if self.is_pressed("backspace"):
            self.reset()
            time.sleep(0.2)
            print("RESET")
            return 3 # Return reset code
        
        # IF NOT PAUSED, NOT ENDING, AND NOT RESETTING
        # Step the physics engine
        curr_step_time = time.time()
        self.engine.stepSimulation()
        self.time = self.time + self.dt
        
        # Update the visualizer if it exists
        time_since_last_vis = curr_step_time - self.last_vis_time
        vis_time_okay =  time_since_last_vis >= (1. / self.visualization_fr)
        if vis_time_okay and update_vis and isinstance(self.vis, Visualizer):
            for urdf_obj in self.urdf_objs:
                if urdf_obj.update_vis:
                    self._update_urdf_visual(urdf_obj)
                    
        # Update the animator if it exists
        if update_ani and isinstance(self.ani, Animator):
            self.ani.step()
        
        # Pause upon request
        if self.is_pressed("space"):
            self.paused = True
            if isinstance(self.ani, Animator):
                self.ani.flush_events()
            print("PAUSED")
            time.sleep(0.2)
            return 4 # Return start pause code
        
        # Calculate suspend time if running in real time
        if real_time:
            time_since_last_step = curr_step_time - self.last_step_time
            time_to_wait = self.dt - time_since_last_step
            if time_to_wait > 0:
                time.sleep(time_to_wait)
        self.last_step_time = curr_step_time
    
        # Check for max time
        if max_time != None:
            if self.time > max_time:
                self.is_done = True
                return 5 # Return time out code
    
        return 0 # Return normal code
            