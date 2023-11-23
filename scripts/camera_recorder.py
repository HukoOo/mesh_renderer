#!/usr/bin/env python
import os
import sys
import cv2
import rospy
import yaml
from sensor_msgs.msg import Image, CameraInfo
import message_filters
from cv_bridge import CvBridge

# current file path
SCRIPT_PATH = os.path.dirname(os.path.abspath(__file__))
CONFIG_PATH = os.path.join(SCRIPT_PATH, '../config')

class CameraRecorder():
    def __init__(self):
        self.bridge = CvBridge()
        self.file_saved = False

        # subscribers
        rospy.Subscriber('/camera/rgb/image_raw', Image, self.rgb_img_cb)
        rospy.Subscriber('/camera/depth/image_raw', Image, self.depth_img_cb)
        rospy.Subscriber('/camera/rgb/camera_info', CameraInfo, self.rgb_info_cb)
        rospy.Subscriber('/camera/depth/camera_info', CameraInfo, self.depth_info_cb)

        rgb_img_sub = message_filters.Subscriber('/rgb/image_raw', Image)
        rgb_info_sub = message_filters.Subscriber('/rgb/camera_info', CameraInfo)
        depth_img_sub = message_filters.Subscriber('/depth_to_rgb/image_raw', Image)
        depth_info_sub = message_filters.Subscriber('/depth_to_rgb/camera_info', CameraInfo)

        ts = message_filters.TimeSynchronizer([rgb_img_sub, depth_img_sub, rgb_info_sub, depth_info_sub], 10)
        ts.registerCallback(self.camera_sync_cb)

    def rgb_img_cb(self, data):
        pass
    def rgb_info_cb(self, data):
        pass
    def depth_img_cb(self, data):
        pass
    def depth_info_cb(self, data):
        pass
    def camera_sync_cb(self, rgb_data, depth_data, rgb_info, depth_info):
        # convert images to cv2
        rgb_img = self.bridge.imgmsg_to_cv2(rgb_data, desired_encoding='passthrough')
        depth_img = self.bridge.imgmsg_to_cv2(depth_data, desired_encoding='passthrough')

        # show images
        # cv2.imshow('rgb', rgb_img)
        # cv2.imshow('depth', depth_img)
        # cv2.waitKey(1)

        # save camera info
        rgb_info_dict = {
            'K': rgb_info.K,
            'D': rgb_info.D,
            'R': rgb_info.R,
            'P': rgb_info.P,
            'width': rgb_info.width,
            'height': rgb_info.height,
            'distortion_model': rgb_info.distortion_model,
            'frame_id': rgb_info.header.frame_id,
            'binning_x': rgb_info.binning_x,
            'binning_y': rgb_info.binning_y,
            'roi': {'x_offset': rgb_info.roi.x_offset,
                    'y_offset': rgb_info.roi.y_offset,
                    'height': rgb_info.roi.height,
                    'width': rgb_info.roi.width,
                    'do_rectify': rgb_info.roi.do_rectify}
        }
        depth_info_dict = {
            'K': depth_info.K,
            'D': depth_info.D,
            'R': depth_info.R,
            'P': depth_info.P,
            'width': depth_info.width,
            'height': depth_info.height,
            'distortion_model': depth_info.distortion_model,
            'frame_id': depth_info.header.frame_id,
            'binning_x': depth_info.binning_x,
            'binning_y': depth_info.binning_y,
            'roi': {'x_offset': depth_info.roi.x_offset,
                    'y_offset': depth_info.roi.y_offset,
                    'height': depth_info.roi.height,
                    'width': depth_info.roi.width,
                    'do_rectify': depth_info.roi.do_rectify}
        }
        with open(os.path.join(CONFIG_PATH, 'color_camera.yaml'), 'w') as f:
            yaml.dump(rgb_info_dict, f)
            f.close()
        with open(os.path.join(CONFIG_PATH, 'depth_camera.yaml'), 'w') as f:
            yaml.dump(depth_info_dict, f)
            f.close()

        # save images
        cv2.imwrite(os.path.join(CONFIG_PATH, 'color_camera.png'), rgb_img)
        cv2.imwrite(os.path.join(CONFIG_PATH, 'depth_camera.png'), depth_img)
        rospy.loginfo('Images saved.')

        # load yaml
        with open(os.path.join(CONFIG_PATH, 'color_camera.yaml'), 'r') as f:
            ci = yaml.load(f, Loader=yaml.FullLoader)
            print(ci)
            f.close()
        self.file_saved = True

    def run(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown() and not self.file_saved:
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('camera_recorder', anonymous=True)

    recorder = CameraRecorder()
    recorder.run()


