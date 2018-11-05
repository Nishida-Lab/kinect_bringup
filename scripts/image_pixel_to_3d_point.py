#!/usr/bin/env python
import numpy as np
import struct
import rospy
import rospkg
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import PointCloud2
import tf2_ros
import tf2_geometry_msgs
import geometry_msgs
from visualization_msgs.msg import Marker


class ImagePixelTo3DPointNode:

    def __init__(self):
        rospy.init_node('ImagePixelTo3DPointNode', anonymous=True)
        self.pointcloud_sub = rospy.Subscriber("kinect_head/qhd/points",PointCloud2, self.pointcloud_callback, queue_size=1, buff_size=2**24)
        self.point2d_sub = rospy.Subscriber("image_pixel",Int32MultiArray, self.pixel_callback, queue_size=1, buff_size=2**24)
        self.marker_pub = rospy.Publisher("transferred_image_pixel", Marker, queue_size = 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listner = tf2_ros.TransformListener(self.tf_buffer)
        rospy.loginfo("Initialized ImagePixelTo3DPointNode.")

    def pixelTo3DPoint(self, u, v, pointcloud):
        point_step = pointcloud.point_step
        row_step = pointcloud.row_step

        array_pos = v*row_step + u*point_step

        bytesX = [ord(x) for x in pointcloud.data[array_pos:array_pos+4]]
        bytesY = [ord(x) for x in pointcloud.data[array_pos+4: array_pos+8]]
        bytesZ = [ord(x) for x in pointcloud.data[array_pos+8:array_pos+12]]

        byte_format=struct.pack('4B', *bytesX)
        X = struct.unpack('f', byte_format)[0]

        byte_format=struct.pack('4B', *bytesY)
        Y = struct.unpack('f', byte_format)[0]

        byte_format=struct.pack('4B', *bytesZ)
        Z = struct.unpack('f', byte_format)[0]

        return X, Y, Z

    def transform_3D_point_to_world_frame(self, xyz_point_from_sensor, sensor_frame_id):

        pose_stamped = geometry_msgs.msg.PoseStamped()
        pose_stamped.header.frame_id = sensor_frame_id
        pose_stamped.pose.orientation.w = 1.0
        pose_stamped.pose.position.x = xyz_point_from_sensor[0]
        pose_stamped.pose.position.y = xyz_point_from_sensor[1]
        pose_stamped.pose.position.z = xyz_point_from_sensor[2]

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

        marker_data.scale.x = 0.06
        marker_data.scale.y = 0.06
        marker_data.scale.z = 0.06

        marker_data.type = 2

        return marker_data

    def pointcloud_callback(self, data):
        self.pointcloud = data

    def pixel_callback(self, data):
        u = data.data[0] #480
        v = data.data[1] #270
        # rospy.loginfo("Subscribed 2D point.")
        sensor_frame_id = self.pointcloud.header.frame_id
        xyz_point_from_sensor = self.pixelTo3DPoint(u,v,self.pointcloud)
        xyz_point_from_world = self.transform_3D_point_to_world_frame(xyz_point_from_sensor, sensor_frame_id)
        marker_data = self.generate_marker(xyz_point_from_world)
        self.marker_pub.publish(marker_data)


def main():
    ImagePixelTo3DPointNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting Down.")


if __name__ == '__main__':
    main()
