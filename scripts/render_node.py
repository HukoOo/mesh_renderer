#!/usr/bin/env python
import cv2
import rospy
from geometry_msgs.msg import Pose
import os
import sys
import yaml
import open3d
import open3d.visualization.gui as gui
import open3d.visualization.rendering as rendering
import numpy as np
from tf.transformations import quaternion_matrix

# current file path
SCRIPT_PATH = os.path.dirname(os.path.abspath(__file__))
CONFIG_PATH = os.path.join(SCRIPT_PATH, '../config')
MESH_PATH = os.path.join(SCRIPT_PATH, '../meshes')

class MeshRenderNode():
    def __init__(self):
        self.color_info = self.load_camera_info(os.path.join(CONFIG_PATH, 'color_camera.yaml'))
        self.depth_info = self.load_camera_info(os.path.join(CONFIG_PATH, 'depth_camera.yaml'))
        self.color_img = self.load_image(os.path.join(CONFIG_PATH, 'color_camera.png'))
        self.depth_img = self.load_image(os.path.join(CONFIG_PATH, 'depth_camera.png'))

    def load_camera_info(self, yaml_path):
        camera_info = yaml.load(open(yaml_path, 'r'), Loader=yaml.FullLoader)
        return camera_info

    def load_image(self, img_path):
        img = cv2.imread(img_path)
        return img
    
    def render_image(self, name, pose):
        # create render scene
        image_width = self.color_info['width']
        image_height = self.color_info['height']
        renderer = rendering.OffscreenRenderer(image_width, image_height)
        renderer.scene.scene.enable_sun_light(True)
        renderer.scene.show_axes(False)
        renderer.scene.set_background([0., 0., 0., 0.])  # RGBA

        # load mesh
        yellow = rendering.MaterialRecord()
        yellow.base_color = [1.0, 0.75, 0.0, 1.0]
        yellow.shader = "defaultLit"
        renderer.scene.clear_geometry()
        mesh = open3d.io.read_triangle_mesh(os.path.join(MESH_PATH, 'conveyor.obj'))
        renderer.scene.add_geometry("conveyor", mesh, yellow)

        # setup camera intrinsic values
        fx = self.color_info['K'][0]
        fy = self.color_info['K'][4]
        cx = self.color_info['K'][2]
        cy = self.color_info['K'][5]
        intrinsic = open3d.camera.PinholeCameraIntrinsic(image_width, image_height, fx, fy, cx, cy)
        cam_param = open3d.camera.PinholeCameraParameters()
        cam_param.intrinsic = intrinsic

        # ROS pose to 4x4 transformation matrix
        RT = np.eye(4)
        r_mat = quaternion_matrix([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
        t_vec = np.array([pose.position.x, pose.position.y, pose.position.z])
        RT[:3, :3] = r_mat[:3, :3]
        RT[:3, 3] = t_vec
        
        cam_param.extrinsic = RT

        renderer.setup_camera(cam_param.intrinsic, cam_param.extrinsic)

        render_image = cv2.cvtColor(np.asarray(renderer.render_to_image()), cv2.COLOR_BGR2RGB)

        overlay = cv2.addWeighted(self.color_img, 0.8, render_image, 0.8, 0)
        cv2.imshow('render', overlay)
        cv2.waitKey(0)


if __name__ == '__main__':
    rospy.init_node('mesh_render_node')

    pose = Pose()
    pose.position.x = -0.22152750194072723
    pose.position.y = -0.0831286832690239
    pose.position.z = 0.4721667766571045
    pose.orientation.x = -0.05238669738173485
    pose.orientation.y = 0.9983741044998169
    pose.orientation.z = -0.009188877418637276
    pose.orientation.w = -0.012252114713191986

    mesh_render_node = MeshRenderNode()
    mesh_render_node.render_image('conveyor', pose)

