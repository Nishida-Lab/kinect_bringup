#!/usr/bin/env python
import numpy as np
import struct
import rospy
import rospkg
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import PointCloud2
from cv_bridge import CvBridge, CvBridgeError
import cv2
import sensor_msgs.point_cloud2 as pc2
import tf2_ros
import tf2_geometry_msgs
import geometry_msgs
from visualization_msgs.msg import Marker

class Get3dPointNode:

    def __init__(self):
        rospy.init_node('get3d', anonymous=True)
        self.bridge = CvBridge()
        self.point2d_sub = rospy.Subscriber("kinect_head/qhd/points",PointCloud2, self.callback, queue_size=1, buff_size=2**24)
        # self.point3d_pub = rospy.Publisher('projected_3d_point', Float32MultiArray, queue_size=1)
        self.marker_pub = rospy.Publisher("transformed_point", Marker, queue_size = 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listner = tf2_ros.TransformListener(self.tf_buffer)

    def pixelTo3DPoint(self, cloud, u, v):
        width = cloud.width
        height = cloud.height
        point_step = cloud.point_step
        row_step = cloud.row_step

        array_pos = v*row_step + u*point_step

        bytesX = [ord(x) for x in cloud.data[array_pos:array_pos+4]]
        bytesY = [ord(x) for x in cloud.data[array_pos+4: array_pos+8]]
        bytesZ = [ord(x) for x in cloud.data[array_pos+8:array_pos+12]]

        byte_format=struct.pack('4B', *bytesX)
        X = struct.unpack('f', byte_format)[0]

        byte_format=struct.pack('4B', *bytesY)
        Y = struct.unpack('f', byte_format)[0]

        byte_format=struct.pack('4B', *bytesZ)
        Z = struct.unpack('f', byte_format)[0]

        return [X, Y, Z]

    def transform_point_to_world_frame(self, point):

        pose_stamped = geometry_msgs.msg.PoseStamped()
        pose_stamped.header.frame_id = "kinect_head_rgb_optical_frame"
        pose_stamped.pose.orientation.w = 1.0    # Neutral orientation
        pose_stamped.pose.position.x = point[0]
        pose_stamped.pose.position.y = point[1]
        pose_stamped.pose.position.z = point[2]

        transform = self.tf_buffer.lookup_transform("world",
                                                    pose_stamped.header.frame_id,
                                                    rospy.Time(0),
                                                    rospy.Duration(1.0))        
        pose_transformed = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)

        return pose_transformed


    def generate_marker(self, point):
        marker_data = Marker()
        marker_data.header.frame_id = point.header.frame_id
        marker_data.header.stamp = rospy.Time.now()

        marker_data.ns = "basic_shapes"
        marker_data.id = 0

        marker_data.action = Marker.ADD

        marker_data.pose.position = point.pose.position
        marker_data.pose.orientation = point.pose.orientation

        marker_data.color.r = 1.0
        marker_data.color.g = 0.7
        marker_data.color.b = 0.0
        marker_data.color.a = 1.0

        marker_data.scale.x = 0.05
        marker_data.scale.y = 0.05
        marker_data.scale.z = 0.05

        marker_data.type = 2

        return marker_data


    def callback(self, pointcloud):
        try:
            rospy.loginfo("PointCloud subscribed!!")

            point_from_original_frame = self.pixelTo3DPoint(pointcloud, 480,270)
            # point_from_original_frame = self.pixelTo3DPoint(pointcloud, 440,270)            

            point_from_world_frame = self.transform_point_to_world_frame(point_from_original_frame)

            # print "on kinect frame: x= "+str(point_from_original_frame[0])+", y= "+str(point_from_original_frame[1])+", z= "+str(point_from_original_frame[2])
            # print "on world frame: x= "+str(point_from_world_frame.pose.position.x)+", y= "+str(point_from_world_frame.pose.position.y)+", z= "+str(point_from_world_frame.pose.position.z)

            marker_data = self.generate_marker(point_from_world_frame)
            self.marker_pub.publish(marker_data)

        except CvBridgeError as e:
            print(e)

def main():
    Get3dPointNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
         print("Shutting down")

if __name__ == '__main__':
    main()
