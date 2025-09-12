"""
This module provides the Visualizer class.
"""


###############################################################################
#DEPENDENCIES
###############################################################################
import numpy as np
import meshcat
import meshcat.geometry as geo
from condynsate.misc import (format_path, 
                             wxyz_from_euler)


###############################################################################
#VISUALIZER CLASS
###############################################################################
class Visualizer():
    """
    Visualizer manages the meshcat based visulation.
    """
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
        self.scene = meshcat.Visualizer().open()
        
        # Delete all instances from the visualizer
        self.scene.delete()
        
        # Set the visibility of grid and axes
        self.set_grid(grid_vis)
        self.set_axes(axes_vis)
        
        # Set the default lights
        self.set_spotlight(on=False)
        self.set_posx_pt_light(on=False)
        self.set_negx_pt_light(on=True,
                               intensity=0.5,
                               distance=10.)
        self.set_fill_light(on=True,
                            intensity=0.35)
        self.set_ambient_light(on=True,
                               intensity=0.65)          


    def add_obj(self,
                urdf_name,
                link_name,
                obj_path, 
                tex_path,
                scale=[1., 1., 1.],
                translate=[0., 0., 0.],
                wxyz_quaternion=[1., 0., 0., 0.]):
        """
        Adds a textured .obj to the visualization.

        Parameters
        ----------
        urdf_name : string
            The name of the urdf to which the .obj is being added as a link.
            URDF objects define robots or assemblies.
        link_name : string
            The name of the link.
        tex_path : string
            Relative path pointing to the .png file that provides the
            texture.
        scale : array-like, size (3,), optional
            The initial scaling along the three axes applied to the .obj.
            The default is [1., 1., 1.].
        translate : array-like, size (3,), optional
            The initial translation along the three axes applied to the .obj.
            The default is [0., 0., 0.].
        wxyz_quaternion : array-like, size (4,), optional
            The wxyz quaternion that defines the initial rotation applied to
            the .obj. The default is [1., 0., 0., 0.].

        Returns
        -------
        obj_geometry : meshcat.geometry.ObjMeshGeometry
            The object mesh.
        obj_texture : meshcat.geometry.MeshPhongMaterial
            The object texture.
            
        """
        # Get geometry of object from the .obj file at obj_path
        obj_path = format_path(obj_path)
        obj_geometry = geo.ObjMeshGeometry.from_file(obj_path)
        
        # Get the texture of object from the .png file at tex_path
        tex_path = format_path(tex_path)
        meshcat_png = geo.PngImage.from_file(tex_path)
        im_tex = geo.ImageTexture(image=meshcat_png,
                                  wrap=[1, 1],
                                  repeat=[1, 1])
        obj_texture = geo.MeshPhongMaterial(map = im_tex)
        
        # Calculate the transform
        transform = self._get_transform(scale, translate, wxyz_quaternion)

        # Add and transform the object to its orientation and position
        self.scene[urdf_name][link_name].set_object(obj_geometry, obj_texture)
        self.scene[urdf_name][link_name].set_transform(transform)
        
        # Return the geometry and texture
        return obj_geometry, obj_texture
        
        
    def add_stl(self,
                urdf_name,
                link_name,
                stl_path,
                color = [91, 155, 213],
                transparent=False,
                opacity = 1.0,
                scale=[1., 1., 1.],
                translate=[0., 0., 0.],
                wxyz_quaternion=[1., 0., 0., 0.]):
        """
        Adds a colored .stl to the visualization.

        Parameters
        ----------
        urdf_name : string
            The name of the urdf to which the .stl is being added as a link.
            URDF objects define robots or assemblies.
        link_name : string
            The name of the link.
        stl_path : string
            The relative path pointing to the .stl description of the link that
            is being added.
        color : array-like, size (3,), optional
            The 0-255 RGB color of the .stl. The default is [91, 155, 213].
        transparent : boolean, optional
            A boolean that indicates if the .stl is transparent.
            The default is False.
        opacity : float, optional
            The opacity of the .stl. Can take float values between 0.0 and 1.0.
            The default is 1.0.
        scale : array-like, size (3,), optional
            The initial scaling along the three axes applied to the .stl.
            The default is [1., 1., 1.].
        translate : array-like, size (3,), optional
            The initial translation along the three axes applied to the .stl.
            The default is [0., 0., 0.].
        wxyz_quaternion : array-like, size (4,), optional
            The wxyz quaternion that defines the initial rotation applied to
            the .stl. The default is [1., 0., 0., 0.].

        Returns
        -------
        link_geometry : meshcat.geometry.StlMeshGeometry
            The link mesh.
        link_mat : meshcat.geometry.MeshPhongMaterial
            The link material.

        """
        # Set the parts's geometry
        stl_path = format_path(stl_path)
        link_geometry = geo.StlMeshGeometry.from_file(stl_path)
        
        # Set the parts's color
        color_int = color[0]*256**2 + color[1]*256 + color[2]
        
        # Set the parts's material
        link_mat = geo.MeshPhongMaterial(color=color_int,
                                         transparent=False,
                                         opacity=opacity,
                                         reflectivity=0.3)
        
        # Calculate the transform
        transform = self._get_transform(scale, translate, wxyz_quaternion)

        # Add and transform the link to its orientation and position
        self.scene[urdf_name][link_name].set_object(link_geometry, link_mat)
        self.scene[urdf_name][link_name].set_transform(transform)
        
        # Return the geometry and material
        return link_geometry, link_mat
    
    
    def set_link_color(self,
                       urdf_name,
                       link_name,
                       link_geometry,
                       color = [91, 155, 213],
                       transparent = False,
                       opacity = 1.0):
        """
        Set a link color by deleting it and then adding another copy of it.

        Parameters
        ----------
        urdf_name : string
            The name of the urdf that contains the link being refreshed.
            URDF objects define robots or assemblies.
        link_name : string
            The name of the link.
        link_geometry : meshcat.geometry.StlMeshGeometry
            The link mesh.
        color : array-like, size (3,), optional
            The 0-255 RGB color of the link. The default is [91, 155, 213].
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
        # Set the parts's color
        color_int = color[0]*256**2 + color[1]*256 + color[2]
        link_mat = geo.MeshPhongMaterial(color=color_int,
                                         transparent=transparent,
                                         opacity=opacity,
                                         reflectivity=0.3)
        
        # Update the part's geometry and color
        self.scene[urdf_name][link_name].set_object(link_geometry, link_mat)


    def _get_transform(self,
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
        # Extract rotation data
        w = wxyz_quaternion[0]
        x = wxyz_quaternion[1]
        y = wxyz_quaternion[2]
        z = wxyz_quaternion[3]
        
        # Perform calculations used to transform quaternion to rotation matrix
        xx = 2.*x*x
        xy = 2.*x*y
        xz = 2.*x*z
        yy = 2.*y*y
        yz = 2.*y*z
        zz = 2.*z*z
        wx = 2.*w*x
        wy = 2.*w*y
        wz = 2.*w*z
        
        # Extract scale data
        s1 = scale[0]
        s2 = scale[1]
        s3 = scale[2]
        
        # Extract translate data
        t1 = translate[0]
        t2 = translate[1]
        t3 = translate[2]
        
        # Directly build the transform matrix
        H = np.array([[s1*(1.-yy-zz), s2*(xy-wz),    s3*(xz+wy),    t1],
                      [s1*(xy+wz),    s2*(1.-xx-zz), s3*(yz-wx),    t2],
                      [s1*(xz-wy),    s2*(yz+wx),    s3*(1.-xx-yy), t3],
                      [0.,            0.,            0.,            1.]])
        
        # Return the translation matrix
        return H


    def apply_transform(self,
                        urdf_name,
                        link_name,
                        scale=[1., 1., 1.],
                        translate=[0., 0., 0.],
                        wxyz_quaternion=[1., 0., 0., 0.]):
        """
        Applies a 3D affine transformation inclunding scaling, translating,
        and rotating to a specified link.

        Parameters
        ----------
        urdf_name : string
            The name of the urdf being transformed.
        link_name : string
            The name of the link being transformed.
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
        transform = self._get_transform(scale, translate, wxyz_quaternion)
        self.scene[urdf_name][link_name].set_transform(transform)
            
        # Return the transform
        return transform
        
        
    def set_grid(self,
                 visible=True):
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
        self.scene["/Grid"].set_property("visible", visible)
        
        
    def set_axes(self,
                 visible=True):
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
        self.scene["/Axes"].set_property("visible", visible)
        
    
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
        # Convert 0-255 top color to 0-1 formatted
        if top_color != None:
            top_color = np.array(top_color)
            top_color = top_color / 255.0
            top_color = top_color.astype(float)
            top_color = top_color.tolist()
            
            # Apply selected color to top of background
            self.scene["/Background"].set_property('top_color', top_color)
        
        # Convert 0-255 bottom color to 0-1 formatted
        if bot_color != None:
            bot_color = np.array(bot_color)
            bot_color = bot_color / 255.0
            bot_color = bot_color.astype(float)
            bot_color = bot_color.tolist()
            
            # Apply selected color to bottom of background
            self.scene["/Background"].set_property('bottom_color', bot_color)


    def set_spotlight(self,
                   on = False,
                   intensity = 1.0,
                   radius = 0.0,
                   distance = 100.):
        """
        Sets the properties of the spotlight in the scene.

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
        # Ensure all values are in the correct range
        intensity = np.clip(intensity, 0., 20.)
        distance = np.clip(distance, 0., 100.)
        
        # Set the spotlight properties
        path = "/Lights/SpotLight/<object>"
        self.scene["/Lights/SpotLight"].set_property('visible', on)
        self.scene[path].set_property('visible', on)   
        self.scene[path].set_property('intensity', intensity)           
        self.scene[path].set_property('distance', distance)       


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
        # Ensure all values are in the correct range
        intensity = np.clip(intensity, 0., 20.)
        distance = np.clip(distance, 0., 100.)
        
        # Set the spotlight properties
        path = "/Lights/PointLightPositiveX/<object>"
        self.scene["/Lights/PointLightPositiveX"].set_property('visible', on)
        self.scene[path].set_property('visible', on)   
        self.scene[path].set_property('intensity', intensity)              
        self.scene[path].set_property('distance', distance)  


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
        # Ensure all values are in the correct range
        intensity = np.clip(intensity, 0., 20.)
        distance = np.clip(distance, 0., 100.)
        
        # Set the spotlight properties
        path = "/Lights/PointLightNegativeX/<object>"
        self.scene["/Lights/PointLightNegativeX"].set_property('visible', on)
        self.scene[path].set_property('visible', on)   
        self.scene[path].set_property('intensity', intensity)              
        self.scene[path].set_property('distance', distance) 
        

    def set_ambient_light(self,
                          on = False,
                          intensity = 1.0):
        """
        Sets the properties ambient light of the scene.

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
        # Ensure all values are in the correct range
        intensity = np.clip(intensity, 0., 20.)
        
        # Set the spotlight properties
        path = "/Lights/AmbientLight/<object>"
        self.scene["/Lights/AmbientLight"].set_property('visible', on)
        self.scene[path].set_property('visible', on)   
        self.scene[path].set_property('intensity', intensity)              


    def set_fill_light(self,
                       on = False,
                       intensity = 1.0):
        """
        Sets the properties fill light of the scene.

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
        # Ensure all values are in the correct range
        intensity = np.clip(intensity, 0., 20.)
        
        # Set the spotlight properties
        path = "/Lights/FillLight/<object>"
        self.scene["/Lights/FillLight"].set_property('visible', on)
        self.scene[path].set_property('visible', on)   
        self.scene[path].set_property('intensity', intensity)     


    def transform_camera(self,
                         scale = [1., 1., 1.],
                         translate = [0., 0., 0.],
                         wxyz_quaternion = [1., 0., 0., 0.],
                         roll=None,
                         pitch=None,
                         yaw=None):
        """
        Transforms the position, orientation, and scale of the camera
        in the scene.

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
        # If any euler angles are specified, use the euler angles to set the
        # initial orientation of the urdf object
        # Any unspecified euler angles are set to 0.0
        if roll!=None or pitch!=None or yaw!=None:
            if roll==None:
                roll=0.0
            if pitch==None:
                pitch=0.0
            if yaw==None:
                yaw=0.0
            wxyz_quaternion = wxyz_from_euler(roll, pitch, yaw)
        
        # Calculate and apply the transform
        transform = self._get_transform(scale=scale,
                                       translate=translate,
                                       wxyz_quaternion=wxyz_quaternion)
        self.scene["/Cameras/default"].set_transform(transform)
        