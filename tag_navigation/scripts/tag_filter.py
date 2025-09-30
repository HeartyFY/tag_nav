#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from apriltag_ros.msg import AprilTagDetectionArray
from collections import deque

class TagPoseFilter:
    def __init__(self):
        rospy.init_node('tag_pose_filter', anonymous=True)
        
        self.window_size = rospy.get_param('~window_size', 10)
        self.target_tag_id = rospy.get_param('~target_tag_id', 0)
        self.pose_history = deque(maxlen=self.window_size)
        # 发布过滤后的位姿
        self.filtered_pub = rospy.Publisher('/filtered_tag_pose', PoseStamped, queue_size=10)
        # 订阅原始检测
        self.tag_sub = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.tag_callback)
        rospy.loginfo("Tag位姿滤波节点已启动")
    
    def tag_callback(self, msg):
        for detection in msg.detections:
            if detection.id[0] == self.target_tag_id:
                # 添加到历史记录
                self.pose_history.append(detection.pose)
                
                if len(self.pose_history) >= self.window_size:
                    # 应用滤波
                    filtered_pose = self.apply_filter()
                    
                    # 发布滤波后的位姿
                    filtered_pose.header.stamp = rospy.Time.now()
                    filtered_pose.header.frame_id = detection.pose.header.frame_id
                    self.filtered_pub.publish(filtered_pose)
    
    def apply_filter(self):
        # 简单移动平均滤波
        positions = []
        orientations = []
        
        for pose in self.pose_history:
            p = pose.pose.position
            o = pose.pose.orientation
            positions.append([p.x, p.y, p.z])
            orientations.append([o.x, o.y, o.z, o.w])

        # 计算平均位置和方向
        avg_position = np.mean(positions, axis=0)
        avg_orientation = np.mean(orientations, axis=0)
        # 归一化四元数
        norm = np.linalg.norm(avg_orientation)
        avg_orientation = avg_orientation / norm
        
        # 创建新的PoseStamped
        filtered_pose = PoseStamped()
        filtered_pose.pose.position.x = avg_position[0]
        filtered_pose.pose.position.y = avg_position[1]
        filtered_pose.pose.position.z = avg_position[2]
        filtered_pose.pose.orientation.x = avg_orientation[0]
        filtered_pose.pose.orientation.y = avg_orientation[1]
        filtered_pose.pose.orientation.z = avg_orientation[2]
        filtered_pose.pose.orientation.w = avg_orientation[3]
        
        return filtered_pose

if __name__ == '__main__':
    try:
        filter_node = TagPoseFilter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass