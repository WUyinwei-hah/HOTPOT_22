import sys

from geometry_msgs.msg import TransformStamped

import rclpy
from rclpy.node import Node

from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

import tf_transformations


class StaticFramePublisher(Node):
   """
   Broadcast transforms that never change.

   This example publishes transforms from `world` to a static turtle frame.
   The transforms are only published once at startup, and are constant for all
   time.
   """

   def __init__(self, transformation):
      super().__init__('static_turtle_tf2_broadcaster')

      self._tf_publisher = StaticTransformBroadcaster(self)

      # Publish static transforms once at startup
      self.make_transforms(transformation)

   def make_transforms(self, transformation):
   
      static_transformStamped_imu = TransformStamped()
      static_transformStamped_imu.header.stamp = 0
      static_transformStamped_imu.header.frame_id = 'base_link'
      static_transformStamped_imu.child_frame_id = 'red_standard_robot1/chassis/chassis_imu'
      static_transformStamped_imu.transform.translation.x = 0.
      static_transformStamped_imu.transform.translation.y = 0.
      static_transformStamped_imu.transform.translation.z = 0.
      quat = tf_transformations.quaternion_from_euler(
            float(0), float(0), float(0))
      static_transformStamped_imu.transform.rotation.x = quat[0]
      static_transformStamped_imu.transform.rotation.y = quat[1]
      static_transformStamped_imu.transform.rotation.z = quat[2]
      static_transformStamped_imu.transform.rotation.w = quat[3]

      
      
      
      static_transformStamped_lidar = TransformStamped()
      static_transformStamped_lidar.header.stamp = 0
      static_transformStamped_lidar.header.frame_id = 'base_link'
      static_transformStamped_lidar.child_frame_id = 'red_standard_robot1/front_rplidar_a2/front_rplidar_a2'
      static_transformStamped_lidar.transform.translation.x = 0.15
      static_transformStamped_lidar.transform.translation.y = 0.
      static_transformStamped_lidar.transform.translation.z = -0.03
      quat = tf_transformations.quaternion_from_euler(
            float(3.14), float(0), float(0))
      static_transformStamped_lidar.transform.rotation.x = quat[0]
      static_transformStamped_lidar.transform.rotation.y = quat[1]
      static_transformStamped_lidar.transform.rotation.z = quat[2]
      static_transformStamped_lidar.transform.rotation.w = quat[3]

      self._tf_publisher.sendTransform([static_transformStamped_imu, static_transformStamped_lidar])


def main():
   logger = rclpy.logging.get_logger('logger')

   # pass parameters and initialize node
   rclpy.init()
   node = StaticFramePublisher(sys.argv)
   try:
      rclpy.spin(node)
   except KeyboardInterrupt:
      pass

   rclpy.shutdown()
