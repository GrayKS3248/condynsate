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
        Adds a textured .obj object to the visualization.

        Parameters
        ----------
        obj_name : string, optional
            The name to be assigned to the object in the visualization.
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
        transform : array-like, size (4,4)
            The initial 4x4 3D affine transformation matrix applied to the 
            object.

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
        
        # Transform the object
        transform = self.apply_transform(obj_name=obj_name,
                                         scale=scale,
                                         translate=translate,
                                         wxyz_quaternion=wxyz_quaternion)
        
        # Return the initial transformation
        return transform
        
        
    def add_part(self,
                 obj_name = '/Object',
                 part_name = '/Part',
                 stl_path = './urdf/cmg_inner.stl',
                 color = [91, 155, 213],
                 transparent=False,
                 opacity = 1.0,
                 scale=[1., 1., 1.],
                 translate=[0., 0., 0.],
                 wxyz_quaternion=[1., 0., 0., 0.]):
        """
        Adds a part of an object to the visualizer.

        Parameters
        ----------
        obj_name : string, optional
            The name of the object to which the part is being added.
            The default is '/Body'.
        part_name : string, optional
            The name of the part. The default is '/Object'.
        stl_path : string, optional
            The relative path pointing to the .stl description of the part that
            is being added. The default is './urdf/cmg_inner.stl'.
        color : array-like, size (3,), optional
            The 0-255 RGB color of the part. The default is [91, 155, 213].
        transparent : boolean, optional
            A boolean that indicates if the part is transparent.
            The default is False.
        opacity : float, optional
            The opacity of the part. Can take float values between 0.0 and 1.0.
            The default is 1.0.
        scale : array-like, size (3,), optional
            The initial scaling along the three axes applied to the part.
            The default is [1., 1., 1.].
        translate : array-like, size (3,), optional
            The initial translation along the three axes applied to the part.
            The default is [0., 0., 0.].
        wxyz_quaternion : array-like, size (4,), optional
            The wxyz quaternion that defines the initial rotation applied to
            the part. The default is [1., 0., 0., 0.].

        Returns
        -------
        transform : array-like, size (4,4)
            The initial 4x4 3D affine transformation matrix applied to the 
            part.

        """
        # Set the parts's geometry
        stl_path = _format_path(stl_path)
        part_geometry = g.StlMeshGeometry.from_file(stl_path)
        
        # Set the parts's color
        color_int = color[0]*256**2 + color[1]*256 + color[2]
        
        # Set the parts's material
        part_mat = g.MeshPhongMaterial(color=color_int,
                                       transparent=False,
                                       opacity=opacity)
        
        # Add the part to the visualizer
        self.vis[obj_name][part_name].set_object(part_geometry, part_mat)
        
        # Transform the part
        transform = self.apply_transform(obj_name=obj_name,
                                         part_name=part_name,
                                         scale=scale,
                                         translate=translate,
                                         wxyz_quaternion=wxyz_quaternion)
        
        # Return the initial transformation
        return transform
        

    def apply_transform(self,
                        obj_name='/Object',
                        part_name=None,
                        scale=[1., 1., 1.],
                        translate=[0., 0., 0.],
                        wxyz_quaternion=[1., 0., 0., 0.]):
        """
        Applies a 3D affine transformation inclunding scaling, translating,
        and rotating to a specified object or part.

        Parameters
        ----------
        obj_name : string, optional
            The name of the object being transformed. The default is '/Object'.
        part_name : string, optional
            The name of the part being transformed. If an object is being
            transformed instead, leave part_name as None. The default is None.
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
            The 4x4 3D affine transformation matrix applied to the object or
            part.

        """
        
        # Calculate the transform
        transform = self.get_transform(scale, translate, wxyz_quaternion)
        
        # Apply the transform to either an object or a part
        if isinstance(part_name, str):
            self.vis[obj_name][part_name].set_transform(transform)
        else:
            self.vis[obj_name].set_transform(transform)
            
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
        Loads a URDF object to the simulation engine.

        Parameters
        ----------
        urdf_path : string, optional
            DESCRIPTION. The default is './urdf/plane.urdf'.
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
        urdf_id : int
            The unique integer ID of the loaded urdf object in the simulation
            engine.
        joint_map : dictionary
            A dictionary that maps all urdf object joint names to joint indices
        link_map : dictionary
            A dictionary that maps all urdf object link names to joint indices

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
        
        # Return the ID of the loaded urdf object
        return urdf_id, joint_map, link_map
    
    
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
    
    
    def set_joint_damping(self,
                          urdf_id=0,
                          joint_map={},
                          joint_name="",
                          damping=0.0):
        """
        Sets the damping for a joint of a urdf object.

        Parameters
        ----------
        urdf_id : int, optional
            The unique integer ID of a loaded urdf object in the simulation
            engine. The default is 0.
        joint_map : dict, optional
            A dictionary that maps all urdf object joint names to joint id.
            The default is {}.
        joint_name : string, optional
            The name of the joint whose damping is set. The joint name is
            specified in the .urdf file. The default is "".
        damping : float, optional
            The value of damping to apply. The default is 0.0.

        Returns
        -------
        None.

        """
        if joint_name in joint_map:
            joint_id = joint_map[joint_name]
            self.engine.changeDynamics(urdf_id, joint_id, jointDamping=damping)


###############################################################################
"""MAIN LOOP"""
###############################################################################
if __name__ == "__main__":
    sim = Simulator()
    ground_id, _, _ = sim.load_urdf(urdf_path='./urdf/plane.urdf',
                                    position=[0., 0., -3.],
                                    wxyz_quaternion=[1., 0., 0., 0.])
    wall_id, _, _ = sim.load_urdf(urdf_path='./urdf/plane.urdf',
                                  position=[0., 0., 0.],
                                  roll=0.5*np.pi,
                                  pitch=0.,
                                  yaw=np.pi)
    cmg_id, cmg_joints, cmg_links = sim.load_urdf(urdf_path='./urdf/cmg.urdf',
                                                  position=[0., 1.1, 0.],
                                                  roll=0.,
                                                  pitch=0.,
                                                  yaw=0.)
    
    # viewer = Visualizer(grid_vis=False,
    #                     axes_vis=False)
    # viewer.add_object(obj_name='/Wall', 
    #                   tex_path='./urdf/concrete.png',
    #                   translate=[0, -1.1, 0],
    #                   wxyz_quaternion=[0.70710678, 0.70710678, 0.0, 0.0])
    # viewer.add_object(obj_name='/Ground',
    #                   tex_path='./urdf/check.png',
    #                   translate=[0,0,-4])
    # viewer.add_part(obj_name="/CMG",
    #                 part_name="/Inner",
    #                 stl_path='./urdf/cmg_inner.stl')
    # viewer.add_part(obj_name="/CMG",
    #                 part_name="/Mass",
    #                 stl_path='./urdf/cmg_mass.stl')
    # viewer.add_part(obj_name="/CMG",
    #                 part_name="/Outer",
    #                 stl_path='./urdf/cmg_outer.stl')
    # viewer.add_part(obj_name="/CMG",
    #                 part_name="/Spar",
    #                 stl_path='./urdf/cmg_spar.stl')
    # viewer.add_part(obj_name="/CMG",
    #                 part_name="/Wheel",
    #                 stl_path='./urdf/cmg_wheel.stl')