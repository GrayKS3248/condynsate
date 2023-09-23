# -*- coding: utf-8 -*-
"""
Copyright (c) 2023; Grayson Schaer

Permission is hereby granted, free of charge, to any person obtaining
a copy of this software and associated documentation files 
(the “Software”), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge,
publish, distribute, sublicense, and/or sell copies of the Software,
and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included
in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, 
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES 
OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. 
IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, 
TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH 
THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
"""

###############################################################################
"""DEPENDENCIES"""
###############################################################################
import numpy as np
import pybullet
from pybullet_utils import bullet_client as bc
import time
import json
import importlib
import meshcat
import meshcat.geometry as g
import meshcat.transformations as t
from pathlib import Path
import keyboard


###############################################################################
"""PUBLIC FUNCTIONS"""
###############################################################################
def _xyzw_to_wxyz(xyzw_quaternion):
    """
    Converts a JPL quaternion (xyzw) to a Hamilton quaternion (wxyz)

    Parameters
    ----------
    xyzw_quaternion : array-like, size (4,)
        A JPL quaternion to be converted.

    Returns
    -------
    wxyz_quaternion : array-like, size (4,)
        The Hamilton representation of the input JPL quaterion

    """
    wxyz_quaternion = np.roll(xyzw_quaternion, 1)
    return wxyz_quaternion


def _wxyz_to_xyzw(wxyz_quaternion):
    """
    Converts a Hamilton quaternion (wxyz) to a JPL quaternion (xyzw)

    Parameters
    ----------
    wxyz_quaternion : array-like, size (4,)
        A Hamilton quaternion to be converted.

    Returns
    -------
    xyzw_quaternion : array-like, size (4,)
        The JPL representation of the input Hamilton quaterion.

    """
    xyzw_quaternion = np.roll(wxyz_quaternion, -1)
    return xyzw_quaternion


def _format_path(unformatted_path):
    """
    Converts an unformatted relative path to a formatted Windows path string

    Parameters
    ----------
    unformatted_path : string
        The unformatted relative path.

    Returns
    -------
    formatted_path : string
        The formatted Windows path string.

    """
    formatted_path = str(Path(unformatted_path))
    return formatted_path
    

###############################################################################
"""VISUALIZER CLASS"""
###############################################################################
class Visualizer():
    def __init__(self,
                 grid_vis=True,
                 axes_vis=True):
        """
        Initializes a new instance of a visualizer

        Parameters
        ----------
        grid_vis : bool, optional
            The boolean value to which the visibility of the XY grid is set. 
            The default is True.
        axes_vis : bool, optional
            The boolean value to which the visibility of the axes is set.
            The default is True.

        Returns
        -------
        None.

        """
        # Open a new instance of a meshcat visualizer
        self.vis = meshcat.Visualizer().open()
        
        # Delete all instances from the visualizer
        self.vis.delete()
        
        # Set the visibility of grid and axes
        self.set_grid(grid_vis)
        self.set_axes(axes_vis)


    def add_object(self,
                   obj_name='/Object',
                   obj_path='./urdf/plane.obj', 
                   tex_path='./urdf/check.png',
                   scale=[1., 1., 1.],
                   translate=[0., 0., 0.],
                   wxyz_quaternion=[1., 0., 0., 0.]):
        """
        Adds a textured .obj object to the visualization. Objects are meant to
        be stationary and their position cannot be updated after they are
        placed in the visualization.

        Parameters
        ----------
        obj_name : string, optional
            The name to be assigned to the .obj object in the visualization.
            The default is '/Object'.
        obj_path : string, optional
            Relative path pointing to the .obj file that defines the object.
            The default is './urdf/plane.obj'.
        tex_path : string, optional
            Relative path pointing to the .png file that provides the object
            texture. The default is './urdf/check.png'.
        scale : array-like, size (3,), optional
            The initial scaling along the three axes applied to the object.
            The default is [1., 1., 1.].
        translate : array-like, size (3,), optional
            The initial translation along the three axes applied to the object.
            The default is [0., 0., 0.].
        wxyz_quaternion : array-like, size (4,), optional
            The wxyz quaternion that defines the initial rotation applied to
            the object. The default is [1., 0., 0., 0.].

        Returns
        -------
        None.

        """
        # Get geometry of object from the .obj file at obj_path
        obj_path = _format_path(obj_path)
        obj_geometry = g.ObjMeshGeometry.from_file(obj_path)
        
        # Get the texture of object from the .png file at tex_path
        tex_path = _format_path(tex_path)
        meshcat_png = g.PngImage.from_file(tex_path)
        im_tex = g.ImageTexture(image=meshcat_png,
                                wrap=[1, 1],
                                repeat=[1, 1])
        obj_texture = g.MeshPhongMaterial(map = im_tex)
        
        # Add the textured object to the visualizer
        self.vis[obj_name].set_object(obj_geometry, obj_texture)
        
        # Transform the object to its orientation and position
        transform = self.get_transform(scale, translate, wxyz_quaternion)
        self.vis[obj_name].set_transform(transform)
        
        
    def add_link(self,
                 urdf_name = '/URDF',
                 link_name = '/Link',
                 stl_path = './urdf/cmg_inner.stl',
                 color = [91, 155, 213],
                 transparent=False,
                 opacity = 1.0,
                 scale=[1., 1., 1.],
                 translate=[0., 0., 0.],
                 wxyz_quaternion=[1., 0., 0., 0.]):
        """
        Adds a link of a urdf object to the visualizer. Links are not fixed and
        are meant to have their position and orientation updated.

        Parameters
        ----------
        urdf_name : string, optional
            The name of the urdf object to which the link is being added.
            The default is '/URDF'. URDF objects define robots or assemblies.
        link_name : string, optional
            The name of the link. The default is '/Link'.
        stl_path : string, optional
            The relative path pointing to the .stl description of the link that
            is being added. The default is './urdf/cmg_inner.stl'.
        color : array-like, size (3,), optional
            The 0-255 RGB color of the link. The default is [91, 155, 213].
        transparent : boolean, optional
            A boolean that indicates if the link is transparent.
            The default is False.
        opacity : float, optional
            The opacity of the link. Can take float values between 0.0 and 1.0.
            The default is 1.0.
        scale : array-like, size (3,), optional
            The initial scaling along the three axes applied to the link.
            The default is [1., 1., 1.].
        translate : array-like, size (3,), optional
            The initial translation along the three axes applied to the link.
            The default is [0., 0., 0.].
        wxyz_quaternion : array-like, size (4,), optional
            The wxyz quaternion that defines the initial rotation applied to
            the link. The default is [1., 0., 0., 0.].

        Returns
        -------
        transform : array-like, size (4,4)
            The initial 4x4 3D affine transformation matrix applied to the 
            link.

        """
        # Set the parts's geometry
        stl_path = _format_path(stl_path)
        link_geometry = g.StlMeshGeometry.from_file(stl_path)
        
        # Set the parts's color
        color_int = color[0]*256**2 + color[1]*256 + color[2]
        
        # Set the parts's material
        link_mat = g.MeshPhongMaterial(color=color_int,
                                       transparent=False,
                                       opacity=opacity)
        
        # Add the part to the visualizer
        self.vis[urdf_name][link_name].set_object(link_geometry, link_mat)
        
        # Transform the part
        transform = self.apply_transform(urdf_name=urdf_name,
                                         link_name=link_name,
                                         scale=scale,
                                         translate=translate,
                                         wxyz_quaternion=wxyz_quaternion)
        
        # Return the initial transformation
        return transform
        

    def apply_transform(self,
                        urdf_name='/URDF',
                        link_name='/Link',
                        scale=[1., 1., 1.],
                        translate=[0., 0., 0.],
                        wxyz_quaternion=[1., 0., 0., 0.]):
        """
        Applies a 3D affine transformation inclunding scaling, translating,
        and rotating to a specified link.

        Parameters
        ----------
        urdf_name : string, optional
            The name of the urdf object being transformed.
            The default is '/URDF'.
        link_name : string, optional
            The name of the link being transformed. The default is '/Link'.
        scale : array-like, size (3,), optional
            The scaling along the three axes. The default is [1., 1., 1.].
        translate : array-like, size (3,), optional
            The translation along the three axes. The default is [0., 0., 0.].
        wxyz_quaternion : array-like, size (4,), optional
            The wxyz quaternion that defines the rotation. 
            The default is [1., 0., 0., 0.].

        Returns
        -------
        transform : array-like, size (4,4)
            The 4x4 3D affine transformation matrix applied to the link.

        """
        
        # Calculate and apply the transform
        transform = self.get_transform(scale, translate, wxyz_quaternion)
        self.vis[urdf_name][link_name].set_transform(transform)
            
        # Return the transform
        return transform


    def get_transform(self,
                      scale=[1., 1., 1.],
                      translate=[0., 0., 0.],
                      wxyz_quaternion=[1., 0., 0., 0.]):
        """
        Calculates the spatial transformation matrix that defines a 3D affine
        transformation inclunding scaling, translating, and rotating.

        Parameters
        ----------
        scale : array-like, size (3,), optional
            The scaling along the three axes. The default is [1., 1., 1.].
        translate : array-like, size (3,), optional
            The translation along the three axes. The default is [0., 0., 0.].
        wxyz_quaternion : array-like, size (4,), optional
            The wxyz quaternion that defines the rotation. 
            The default is [1., 0., 0., 0.].

        Returns
        -------
        transform : array-like, size (4,4)
            The resultant 4x4 3D affine transformation matrix.

        """
        # Get the scaling matrix based on the scale vector
        # Get the translation matrix based on the translation vector
        # Get the rotation matrix based on the wxyz quaternion
        scale_matrix = np.diag(np.concatenate((scale, [1.0])))
        translate_matrix = t.translation_matrix(translate)
        rotation_matrix = t.quaternion_matrix(wxyz_quaternion)
        
        # Calculate and return the total transformation matrix
        transform = scale_matrix @ translate_matrix @ rotation_matrix
        return transform
        
        
    def set_grid(self, visible=True):
        """
        Controls the visibility state of the XY grid in the visualizer.

        Parameters
        ----------
        visible : bool, optional
            The boolean value to which the visibility of the XY grid is set.
            The default is True.

        Returns
        -------
        None.

        """
        self.vis["/Grid"].set_property("visible", visible)
        
        
    def set_axes(self, visible=True):
        """
        Controls the visibility state of the axes in the visualizer.

        Parameters
        ----------
        visible : bool, optional
            The boolean value to which the visibility of the axes is set.
            The default is True.

        Returns
        -------
        None.

        """
        self.vis["/Axes"].set_property("visible", visible)
        

###############################################################################
"""URDF OBJECT CLASS"""
###############################################################################
class URDF_Obj:
    def __init__(self,
                 urdf_id=0,
                 joint_map={},
                 link_map={}):
        """
        Initialize an instance of the URDF_Obj class. This class is used to
        store information relating to a urdf object described by a .urdf file.

        Parameters
        ----------
        urdf_id : int, optional
            The unique integer ID of the loaded urdf object in the simulation
            engine. The default is 0. 
        joint_map : dictionary, optional
            A dictionary that maps all urdf object joint names to joint
            indices. The default is {}.
        link_map : dictionary, optional
            A dictionary that maps all urdf object link names to joint indices.
            The default is {}.

        Returns
        -------
        None.

        """
        self.urdf_id = urdf_id
        self.joint_map = joint_map
        self.link_map = link_map        


###############################################################################
"""SIMULATOR CLASS"""
###############################################################################
class Simulator:
    def __init__(self,
                 gravity=[0., 0., -9.81]):
        """
        Initializes an instance of the Simulator class.

        Parameters
        ----------
        gravity : array-like, shape (3,) optional
            The gravity vectory in m/s^2. The default is [0., 0., -9.81].

        Returns
        -------
        None.

        """
        self.engine = bc.BulletClient(
            connection_mode=pybullet.GUI,
            options=f'--width={640} --height={480}',
        )
        self.engine.configureDebugVisualizer(
            self.engine.COV_ENABLE_GUI, 0,
            lightPosition=[10., 10., 10.],
        )
        self.engine.resetDebugVisualizerCamera(5., 140, -40, (0., 0.5, -1.))
        
        # Connect to pybullet
        #self.engine = bc.BulletClient(connection_mode=pybullet.DIRECT)
        
        # Configure gravity
        self.gravity = gravity
        self.engine.setGravity(self.gravity[0],
                               self.gravity[1],
                               self.gravity[2])
        
        # Configure physics engine parameters
        self.dt = 0.01
        self.engine.setPhysicsEngineParameter(
            fixedTimeStep=self.dt,
            numSubSteps=4,
            restitutionVelocityThreshold=0.05,
            enableFileCaching=0)
        
        
    def load_urdf(self,
                  urdf_path='./urdf/plane.urdf',
                  position = [0., 0., 0.],
                  wxyz_quaternion = [1., 0., 0., 0.],
                  roll=None,
                  pitch=None,
                  yaw=None):
        """
        Loads a URDF object to the simulation engine. All joint's
        velocity control is disabled (this allows the joint to move freely),
        angular and linear damping is set to 0 (eliminates air resistance), and 
        joint dampling is set to 0 (eliminates joint friction).

        Parameters
        ----------
        urdf_path : string, optional
            The path to the .urdf file that describes the urdf object to be
            loaded into the simulation. The default is './urdf/plane.urdf'.
        position : array-like, shape (3,) optional
            The initial position of the urdf object.
            The default is [0., 0., 0.].
        wxyz_quaternion : array-like, shape (4,) optional
            A wxyz quaternion that describes the intial orientation of the urdf
            object. When roll, pitch, and yaw all have None type, the
            quaternion is used. If any roll, pitch, or yaw have non None type,
            the quaternion is ignored. The default is [1., 0., 0., 0.].
        roll : float, optional
            The initial roll angle of the urdf object. The default is None.
        pitch : float, optional
            The initial pitch angle of the urdf object. The default is None.
        yaw : float, optional
            The initial yaw angle of the urdf object. The default is None.

        Returns
        -------
        urdf_obj : URDF_Obj
            A URDF_Obj that describes the .urdf object that was loaded into
            the simulation.
            
        """
        # Get the properly formatted string of the urdf path
        urdf_path = _format_path(urdf_path)
        
        # Get the initial position of the urdf object in world coordinates
        position = np.array(position)
        
        # If no euler angles are specified, use the quaternion to set the
        # initial orientation of the urdf object
        if roll==None and pitch==None and yaw==None: 
            orientation = _wxyz_to_xyzw(wxyz_quaternion)
        
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
                                       useFixedBase=1)
        
        # Get the joint and link maps for the urdf object
        joint_map, link_map = self.make_joint_and_link_maps(urdf_id)
        
        # Create urdf_obj and adjust the default state of its joints
        urdf_obj = URDF_Obj(urdf_id, joint_map, link_map)
        for joint_name in joint_map:
            # Disable velocity control ()
            self.disable_velocity_control(urdf_obj,
                                          joint_name=joint_name)
            
            # Set the linear and angular damping to 0 (eliminate drag)
            self.set_linear_angular_damping(urdf_obj,
                                            joint_name=joint_name,
                                            linear_damping=0.,
                                            angular_damping=0.)
            
            # Set damping of joints to 0 (eliminate friction)
            self.set_joint_damping(urdf_obj,
                                   joint_name=joint_name,
                                   damping=0.)

        # Return the URDF_Obj
        return urdf_obj
    
    
    def make_joint_and_link_maps(self,
                                 urdf_id=0):
        """
        Creates a joint map and a link map for a urdf object.

        Parameters
        ----------
        urdf_id : int, optional
            The unique integer ID of a loaded urdf object in the simulation
            engine. The default is 0.

        Returns
        -------
        joint_map : dictionary
            A dictionary that maps all urdf object joint names to joint id
        link_map : dictionary
            A dictionary that maps all urdf object link names to joint id

        """
        # A dictionary that maps joint names to joint id
        # A dictionary that maps link names to joint id
        joint_map = {}
        link_map = {}
        
        # Go through all joints in the urdf object
        num_joints = self.engine.getNumJoints(urdf_id)
        for joint_id in range(num_joints):
            joint_info = self.engine.getJointInfo(urdf_id, joint_id)
            joint_name = joint_info[1].decode('UTF-8')
            link_name = joint_info[12].decode('UTF-8')
            joint_map[joint_name] = joint_id
            link_map[link_name] = joint_id
            
        # Return the maps
        return joint_map, link_map
    
    
    def disable_velocity_control(self,
                                 urdf_obj=URDF_Obj(),
                                 joint_name=""):
        """
        Disable velocity control mode for a joint. In pybullet, all joints are
        initialized with velocity control mode engaged and a target velocity of
        0. To simulate freely moving joints, velocity control is turned off.

        Parameters
        ----------
        urdf_obj : URDF_Obj, optional
            A URDF_Obj that contains that joint for which velocity control is
            disabled. The default is URDF_Obj().
        joint_name : string, optional
            The name of the joint whose velocity control is disabled. The
            joint name is specified in the .urdf file. The default is "".

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
    
    
    def set_linear_angular_damping(self,
                                   urdf_obj=URDF_Obj(),
                                   joint_name="",
                                   linear_damping=0.,
                                   angular_damping=0.):
        """
        Allows user to set the linear and angular damping of a joint. Linear
        and angular damping is a way to model drag. It is typically
        reccomended that the user set these values to 0 for all joints.

        Parameters
        ----------
        urdf_obj : URDF_Obj, optional
            A URDF_Obj that contains that joint whose linear and angular
            damping is being set. The default is URDF_Obj().
        joint_name : string, optional
            The name of the joint whose linear and angular damping is set. The
            joint name is specified in the .urdf file. The default is "".
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
                          urdf_obj=URDF_Obj(),
                          joint_name="",
                          damping=0.):
        """
        Sets the damping of a joint in a urdf object. The damping of a joint
        defines the energy loss incurred during movement of the joint. It is
        a way to model joint friction.

        Parameters
        ----------
        urdf_obj : URDF_Obj, optional
            A URDF_Obj that contains that joint whose damping is being set.
            The default is URDF_Obj().
        joint_name : string, optional
            The name of the joint whose damping is set. The joint name is
            specified in the .urdf file. The default is "".
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
    
    
    def set_link_mass(self,
                      urdf_obj=URDF_Obj(),
                      link_name="",
                      mass=0.):
        """
        Sets the mass of a link in a urdf object.

        Parameters
        ----------
        urdf_obj : URDF_Obj, optional
            A URDF_Obj that contains that link whose mass is being set.
            The default is URDF_Obj().
        link_name : string, optional
            The name of the link whose mass is set. The link name is
            specified in the .urdf file. The default is "".
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
    
        
    def set_joint_position(self,
                          urdf_obj=URDF_Obj(),
                          joint_name="",
                          position=0.):
        """
        Sets the position of a joint of a urdf object.

        Parameters
        ----------
        urdf_obj : URDF_Obj, optional
            A URDF_Obj that contains that joint whose position is being set.
            The default is URDF_Obj().
        joint_name : string, optional
            The name of the joint whose position is set. The joint name is
            specified in the .urdf file. The default is "".
        position : float, optional
            The position to be applied to the joint. The default is 0..

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
                                                  targetPositions=position)
    
    
    def set_joint_velocity(self,
                          urdf_obj=URDF_Obj(),
                          joint_name="",
                          velocity=0.,
                          verbose=False):
        """
        Sets the velocity of a joint of a urdf object.

        Parameters
        ----------
        urdf_obj : URDF_Obj, optional
            A URDF_Obj that contains that joint whose velocity is being set.
            The default is URDF_Obj().
        joint_name : string, optional
            The name of the joint whose velocity is set. The joint name is
            specified in the .urdf file. The default is "".
        velocity : float, optional
            The velocity to be applied to the joint. The default is 0..
        verbose : bool, optional
            Debug tool. When set to True, velocity is printed after being set.
            The default is False.

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
                                                  targetVelocities=velocity)
    
    
    def set_joint_torque(self,
                         urdf_obj=URDF_Obj(),
                         joint_name="",
                         torque=0.):
        """
        Sets the torque of a joint of a urdf object.

        Parameters
        ----------
        urdf_obj : URDF_Obj, optional
            A URDF_Obj that contains that joint whose torque is being set.
            The default is URDF_Obj().
        joint_name : string, optional
            The name of the joint whose torque is set. The joint name is
            specified in the .urdf file. The default is "".
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
                    urdf_obj=URDF_Obj(),
                    joint_name="",
                    position=0.,
                    velocity=0.):
        """
        Resets a joint to a desired position and velocity.
    
        Parameters
        ----------
        urdf_obj : URDF_Obj, optional
            A URDF_Obj that contains that joint whose torque is being set.
            The default is URDF_Obj().
        joint_name : string, optional
            The name of the joint whose torque is set. The joint name is
            specified in the .urdf file. The default is "".
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
            
    
    def add_urdf_to_visualizer(self,
                               viwer,
                               urdf_obj=URDF_Obj(),
                               tex_path='./urdf/check.png'):
        """
        Adds static .obj objects and dynamic .stl links to the visualizer
        based on their urdf description.

        Parameters
        ----------
        viewer : Visualizer
            The visualizer to which the object is added.
        urdf_obj : URDF_Obj, optional
            A URDF_Obj that will be added to the visualizer.
            The default is URDF_Obj().
        tex_path : string, optional
            The path pointing towards a texture file. This texture is applied
            only to static .obj objects. The default is './urdf/check.png'.

        Returns
        -------
        None.

        """
        # Extract the visual data from the urdf object in the simulator
        paths,names,scales,colors,poss,oris = self.urdf_visual_data(urdf_obj)
        
        # Make the URDF name and format the texture path
        urdf_name = "/"+str(urdf_obj.urdf_id)
        tex_path = _format_path(tex_path)
        
        # Loop through all the links
        for i in range(len(paths)):
            
            # If the current object is a static .obj object, add a static
            # object to the visualizer
            if paths[i][-4:] == ".obj":
                viewer.add_object(obj_name=urdf_name,
                                  obj_path=paths[i],
                                  tex_path=tex_path,
                                  scale=scales[i],
                                  translate=poss[i],
                                  wxyz_quaternion=oris[i])
                
            # If the current object is a link, add a .stl link to the
            # visualizer
            elif paths[i][-4:] == ".stl":
                link_name = "/" + names[i]
                rgb = colors[i][0:3]
                opacity = colors[i][3]
                transparent = opacity != 1.0
                viewer.add_link(urdf_name=urdf_name,
                                link_name=link_name,
                                stl_path=paths[i],
                                color=rgb,
                                transparent=transparent,
                                opacity=opacity,
                                scale=scales[i],
                                translate=poss[i],
                                wxyz_quaternion=oris[i])

    
    def update_urdf_visual(self,
                           viewer,
                           urdf_obj=URDF_Obj()):
        """
        Updates the positions of dynamic links in the visualizer.

        Parameters
        ----------
        viewer : Visualizer
            The visualizer which is being updated.
        urdf_obj : URDF_Obj, optional
            A URDF_Obj whose links are being updated.
            The default is URDF_Obj().

        Returns
        -------
        None.

        """
        paths,names,scales,colors,poss,oris = self.urdf_visual_data(urdf_obj)
        
        urdf_name = "/"+str(urdf_obj.urdf_id)
        
        for i in range(len(paths)):
            if paths[i][-4:]==".stl":
                link_name = "/" + names[i]
                viewer.apply_transform(urdf_name=urdf_name,
                                       link_name=link_name,
                                       scale=scales[i],
                                       translate=poss[i],
                                       wxyz_quaternion=oris[i])
    
    
    def urdf_visual_data(self,
                        urdf_obj=URDF_Obj()):
        """
        Extracts all relevant visual data from a urdf object loaded into the
        simulator.

        Parameters
        ----------
        urdf_obj : URDF_Obj, optional
            A URDF_Obj whose visual data is being extracted.
            The default is URDF_Obj().

        Returns
        -------
        paths : list of strings
            A list containing the paths to the files containing urdf or link
            geometries.
        link_names : list of strings
            A list containing the name of all links in the urdf object.
        scales : list of lists (3,)
            A list containing the scale data for the urdf object or all links
            in the urdf object.
        colors : list of lists (4,)
            A list containing the RGBA data for the urdf object or all links
            in the urdf object.
        positions : list of lists (3,)
            A list containing the position data for the urdf object of all
            links in the urdf object.
        orientations : list of lists (4,)
            A list containing the wxyz quaternion(s) for the urdf object or all
            links in the urdf object.

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
            path = _format_path(path)
            paths.append(path)
            scale = list(vis_datum[3])
            scales.append(scale)
            color = list(vis_datum[7])
            colors.append(color)
            
            # If the path points to a .obj file, then the object for which we
            # are collecting data is a static object and we gather no link data
            if path[-4:] == ".obj":
                pos_ori_data = self.engine.getBasePositionAndOrientation(urdf_id)
                position = list(pos_ori_data[0])
                positions.append(position)
                orientation = list(_xyzw_to_wxyz(pos_ori_data[1]))
                orientations.append(orientation)
            
            # If the path points to a .stl file, then the object for which we
            # are collecting data is a dynamic link and we gather link data
            elif path[-4:] == ".stl":
                # Extract link names
                link_id = vis_datum[1]
                joint_data = self.engine.getJointInfo(urdf_id, link_id)
                link_name = joint_data[12].decode('UTF-8')
                link_names.append(link_name)
                
                # Extract link positions and orientations
                link_state = self.engine.getLinkState(urdf_id, link_id)
                position = list(link_state[4])
                positions.append(position)
                orientation = list(_xyzw_to_wxyz(link_state[5]))
                orientations.append(orientation)
                
        return paths, link_names, scales, colors, positions, orientations


###############################################################################
"""MAIN LOOP"""
###############################################################################
if __name__ == "__main__":
    sim = Simulator()
    
    # Load all urdf objects
    ground_obj = sim.load_urdf(urdf_path='./urdf/plane.urdf',
                                position=[0., 0., -3.],
                                wxyz_quaternion=[1., 0., 0., 0.])
    wall_obj = sim.load_urdf(urdf_path='./urdf/plane.urdf',
                              position=[0., 0., 0.],
                              roll=0.5*np.pi,
                              pitch=0.,
                              yaw=np.pi)
    cmg_obj = sim.load_urdf(urdf_path='./urdf/cmg.urdf',
                            position=[0., 1.1, 0.],
                            roll=0.,
                            pitch=0.,
                            yaw=0.)
    
    # Set link mass
    sim.set_link_mass(urdf_obj=cmg_obj,
                      link_name="mass",
                      mass=1.0)
    
    # Set joint damping
    sim.set_joint_damping(urdf_obj=cmg_obj,
                          joint_name="world_to_outer",
                          damping=0.1)
    sim.set_joint_damping(urdf_obj=cmg_obj,
                          joint_name="outer_to_inner",
                          damping=0.01)
    
    # Reset joints to 0 position and velocity
    sim.reset_joint(urdf_obj=cmg_obj,
                    joint_name="world_to_outer",
                    position=0.,
                    velocity=0.)
    sim.reset_joint(urdf_obj=cmg_obj,
                    joint_name="outer_to_inner",
                    position=0.,
                    velocity=0.)
    sim.reset_joint(urdf_obj=cmg_obj,
                    joint_name="inner_to_wheel",
                    position=0.,
                    velocity=100.)
    
    # Add urdf objects to the visualizer
    viewer = Visualizer(grid_vis=False,axes_vis=False)
    sim.add_urdf_to_visualizer(viewer,
                               urdf_obj=ground_obj,
                               tex_path='./urdf/check.png')
    sim.add_urdf_to_visualizer(viewer,
                               urdf_obj=wall_obj,
                               tex_path='./urdf/concrete.png')
    sim.add_urdf_to_visualizer(viewer,
                               urdf_obj=cmg_obj)
    
    TIME = 0
    active_mass = 1.0
    while(True):
        start_time = time.time()
        sim.engine.stepSimulation()
        
        if TIME % 0.1 < sim.dt:
            if keyboard.is_pressed("shift+d"):
                active_mass = active_mass + 0.1
            elif keyboard.is_pressed("d"):
                active_mass = active_mass + 0.01
        
            if keyboard.is_pressed("shift+a"):
                active_mass = active_mass - 0.1
            elif keyboard.is_pressed("a"):
                active_mass = active_mass - 0.1
            
            if active_mass < 0.:
                active_mass = 0.
            print(active_mass)
            sim.set_link_mass(urdf_obj=cmg_obj,
                              link_name="mass",
                              mass=active_mass)
        
        sim.update_urdf_visual(viewer, cmg_obj)
        
        TIME = TIME + sim.dt
        time_to_wait = sim.dt + start_time - time.time()
        if time_to_wait > 0.:
            time.sleep(time_to_wait)
            
        
    # viewer = Visualizer(grid_vis=False,
    #                     axes_vis=False)
    # viewer.add_object(obj_name='/Wall', 
    #                   tex_path='./urdf/concrete.png',
    #                   translate=[0, -1.1, 0],
    #                   wxyz_quaternion=[0.70710678, 0.70710678, 0.0, 0.0])
    # viewer.add_object(obj_name='/Ground',
    #                   tex_path='./urdf/check.png',
    #                   translate=[0,0,-4])
    # viewer.add_link(urdf_name="/CMG",
    #                 link_name="/Inner",
    #                 stl_path='./urdf/cmg_inner.stl')
    # viewer.add_link(urdf_name="/CMG",
    #                 link_name="/Mass",
    #                 stl_path='./urdf/cmg_mass.stl')
    # viewer.add_link(urdf_name="/CMG",
    #                 link_name="/Outer",
    #                 stl_path='./urdf/cmg_outer.stl')
    # viewer.add_link(urdf_name="/CMG",
    #                 link_name="/Spar",
    #                 stl_path='./urdf/cmg_spar.stl')
    # viewer.add_link(urdf_name="/CMG",
    #                 link_name="/Wheel",
    #                 stl_path='./urdf/cmg_wheel.stl')