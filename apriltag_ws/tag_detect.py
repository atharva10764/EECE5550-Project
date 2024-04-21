#!/usr/bin/env python3

import rospy
import numpy as np
from scipy.spatial.transform import Rotation as R
from apriltag_ros.msg import AprilTagDetectionArray
import tf2_ros

class AprilTagTracker:
    def _init_(self):
        self.node_name = 'tag_tracking_node'
        self.robot_name = f'tags_{np.random.randint(1000, 10000) * np.random.randint(50, 500)}'
        self.file_name = f'{self.robot_name}.txt'
        self.transform_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10))
        self.transform_listener = tf2_ros.TransformListener(self.transform_buffer)
        self.transform_camera_origin = None
        self.found_tags = {}
        self.transform_origin = 'map'
        self.transform_camera = 'base_scan'
        self.frequency = 1  # Hz

    def run(self):
        rospy.init_node(self.node_name)
        rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.get_tag_detection, queue_size=10)
        rospy.Timer(rospy.Duration(1 / self.frequency), self.update_transform_and_log)
        rospy.spin()

    def get_transform(self, A, B):
        try:
            pose = self.transform_buffer.lookup_transform(A, B, rospy.Time(0), rospy.Duration(4))
            t = [pose.transform.translation.x, pose.transform.translation.y, pose.transform.translation.z]
            q = [pose.transform.rotation.x, pose.transform.rotation.y, pose.transform.rotation.z, pose.transform.rotation.w]
            r = R.from_quat([q[3], q[0], q[1], q[2]]).as_matrix()
            return np.vstack((np.hstack((r, np.array(t).reshape(3, 1))), [0, 0, 0, 1]))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"Error getting transform from {A} to {B}: {e}")
            return None

    def get_tag_detection(self, msg):
        if not msg.detections:
            return
        for detection in msg.detections:
            tag_id = detection.id[0]
            pose = detection.pose.pose.pose
            t = [pose.position.x, pose.position.y, pose.position.z]
            q = [pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z]
            r = R.from_quat([q[0], q[1], q[2], q[3]]).as_matrix()
            T_AC = np.vstack((np.hstack((r, np.array(t).reshape(3, 1))), [0, 0, 0, 1]))

            if self.transform_camera_origin is None:
                rospy.logerr("Camera origin transform not yet initialized.")
                return

            T_AO = np.dot(T_AC, self.transform_camera_origin)
            if tag_id in self.found_tags:
                rospy.loginfo(f"Updating position for tag {tag_id}")
                self.found_tags[tag_id] = 0.9 * self.found_tags[tag_id] + 0.1 * T_AO
            else:
                rospy.loginfo(f"New tag detected: {tag_id}")
                self.found_tags[tag_id] = T_AO

    def update_transform_and_log(self, event):
        self.transform_camera_origin = self.get_transform(self.transform_camera, self.transform_origin)
        if self.found_tags:
            with open(self.file_name, 'w') as file:
                for tag_id, transform in self.found_tags.items():
                    file.write(f"Tag {tag_id}: {transform}\n")

if _name_ == '_main_':
    try:
        tracker = AprilTagTracker()
        tracker.run()
    except rospy.ROSInterruptException:
        pass