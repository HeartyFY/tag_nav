#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import PoseStamped, Pose, Quaternion
from apriltag_ros.msg import AprilTagDetectionArray, AprilTagDetection
from tf2_geometry_msgs import do_transform_point, do_transform_pose
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import tf2_ros
import geometry_msgs.msg
from actionlib_msgs.msg import GoalStatusArray, GoalStatus

class AprilTagFollower:
    def __init__(self):
        rospy.init_node('apriltag_follower', anonymous=True)
        
        # 参数设置
        self.target_tag_id = rospy.get_param('~target_tag_id', 75)  # 要跟随的标签ID
        self.desired_distance = rospy.get_param('~desired_distance', 0.01)  # 期望距离（米）
        self.desired_angle = rospy.get_param('~desired_angle', 0.0)  # 期望角度（弧度）
        self.control_frequency = rospy.get_param('~control_frequency', 2.0)  # 控制频率（Hz）
        self.camera_frame_id = rospy.get_param('~camera_frame_id', 'usb_cam')  # 相机坐标系
        
        # 状态变量
        self.current_goal_status = None
        self.is_goal_reached = False
        
        # 发布器 - 发布目标位置到move_base
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        
        # 订阅器
        rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.tag_detection_callback)
        rospy.Subscriber('/move_base/status', GoalStatusArray, self.goal_status_callback)  # 订阅导航状态
        
        # TF2相关
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # 存储当前检测到的标签信息
        self.current_tag = None
        self.last_detection_time = rospy.Time(0)
        self.tag_position = None 
        
        # 控制定时器
        self.control_timer = rospy.Timer(rospy.Duration(1.0/self.control_frequency), self.control_callback)

        rospy.loginfo("AprilTag跟随节点已启动，寻找ID为 {} 的标签".format(self.target_tag_id))
    
    def goal_status_callback(self, msg):
        """处理导航状态信息"""
        if len(msg.status_list) > 0:
            self.current_goal_status = msg.status_list[-1].status
            # 检查是否到达目标
            if self.current_goal_status == GoalStatus.SUCCEEDED:
                self.is_goal_reached = True
                rospy.loginfo("已到达目标点，停止发布新目标")
            elif self.current_goal_status in [GoalStatus.ACTIVE, GoalStatus.PENDING]:
                self.is_goal_reached = False
        else:
            self.current_goal_status = None
            self.is_goal_reached = False
    
    def tag_detection_callback(self, msg):
        """处理AprilTag检测结果"""
        if len(msg.detections) == 0:
            return
        
        # 寻找目标标签
        for detection in msg.detections:
            if detection.id[0] == self.target_tag_id:
                self.current_tag = detection
                self.last_detection_time = rospy.Time.now()
                self.tag_position = detection.pose.pose.pose.position
                rospy.logdebug("检测到目标标签 ID: {}".format(self.target_tag_id))
                break
    
    def get_tag_pose_in_map(self, detection):
        """将标签从相机坐标系转换到地图坐标系"""
        try:
            # 创建PoseStamped消息
            tag_pose_camera = PoseStamped()
            tag_pose_camera.header = detection.pose.header
            tag_pose_camera.pose = detection.pose.pose.pose
            
            # 尝试直接获取从相机到地图的变换
            transform = self.tf_buffer.lookup_transform('map', 
                                                      tag_pose_camera.header.frame_id, 
                                                      rospy.Time(0),
                                                      rospy.Duration(1.0))
            
            # 变换位姿到地图坐标系
            tag_pose_map = do_transform_pose(tag_pose_camera, transform)
            return tag_pose_map.pose
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("TF转换失败: {}".format(str(e)))
            return None
    
    def get_camera_pose_in_map(self, camera_frame_id):
        """将相机位姿从相机坐标系转换到地图坐标系"""
        try:
            # 创建PoseStamped消息
            camera_pose_self = PoseStamped()
            camera_pose_self.header.stamp = rospy.Time.now()
            camera_pose_self.header.frame_id = camera_frame_id
            camera_pose_self.pose.position.x = 0.0
            camera_pose_self.pose.position.y = 0.0
            camera_pose_self.pose.position.z = 0.0
            camera_pose_self.pose.orientation.x = 0.0
            camera_pose_self.pose.orientation.y = 0.0
            camera_pose_self.pose.orientation.z = 0.0
            camera_pose_self.pose.orientation.w = 1.0
            
            # 尝试获取从相机坐标系到地图坐标系的变换
            transform = self.tf_buffer.lookup_transform('map', 
                                                    camera_frame_id, 
                                                    rospy.Time(0),
                                                    rospy.Duration(1.0))
            
            # 变换位姿到地图坐标系
            camera_pose_map = do_transform_pose(camera_pose_self, transform)
            return camera_pose_map.pose
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("相机TF转换失败: {}".format(str(e)))
            return None
    
    def control_callback(self, event):
        """计算并发布目标位置"""
        # 如果已经到达目标点，则停止发布新目标
        if self.is_goal_reached:
            return
            
        if self.current_tag is None:
            rospy.logdebug_throttle(5.0, "未检测到目标标签")
            return
        
        # 检查检测是否过时
        if (rospy.Time.now() - self.last_detection_time).to_sec() > 1.0:
            rospy.logdebug("检测信息已过时")
            self.current_tag = None
            return
        
        try:
            # 获取标签在地图坐标系中的位姿
            tag_pose_map = self.get_tag_pose_in_map(self.current_tag)
            cam_pose_map = self.get_camera_pose_in_map(self.camera_frame_id)
            if tag_pose_map is None:
                rospy.logwarn("无法获取标签在地图坐标系中的位姿")
                return
            if cam_pose_map is None:
                rospy.logwarn("无法获取相机在地图坐标系中的位姿")
                return
            print(cam_pose_map)
            goal_pose = PoseStamped()
            goal_pose.header.stamp = rospy.Time.now()
            goal_pose.header.frame_id = "map"
            
            # 获取标签的方向（欧拉角）
            orientation_tag = tag_pose_map.orientation
            (roll, pitch, yaw) = euler_from_quaternion([orientation_tag.x, orientation_tag.y, orientation_tag.z, orientation_tag.w])
            orientation_cam = cam_pose_map.orientation
            (roll_cam, pitch_cam, yaw_cam) = euler_from_quaternion([orientation_cam.x, orientation_cam.y, orientation_cam.z, orientation_cam.w])
            adjusted_pitch_cam = abs(pitch_cam)
            print(roll_cam, pitch_cam, yaw_cam)

            # 目标位置 - 在标签前方desired_distance处
            goal_pose.pose.position.x = cam_pose_map.position.x + self.tag_position.x - self.desired_distance * math.cos(adjusted_pitch_cam)
            goal_pose.pose.position.y = cam_pose_map.position.y + self.tag_position.z - self.desired_distance * math.sin(adjusted_pitch_cam)
            goal_pose.pose.position.z = 0.0

            # 设置朝向 - 面向标签
            angle_to_tag = math.atan2(
                cam_pose_map.position.y - goal_pose.pose.position.y,
                cam_pose_map.position.x - goal_pose.pose.position.x
            )
            print(angle_to_tag)

            q = quaternion_from_euler(0, 0, angle_to_tag)
            goal_pose.pose.orientation = Quaternion(*q)
            
            # 发布目标位置
            self.goal_pub.publish(goal_pose)
            rospy.loginfo("发布目标位置: x={:.2f}, y={:.2f}, theta={:.2f}".format(
                goal_pose.pose.position.x, 
                goal_pose.pose.position.y, 
                angle_to_tag
            ))

        except Exception as e:
            rospy.logerr("计算目标位置时出错: {}".format(str(e)))
    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        follower = AprilTagFollower()
        follower.run()
    except rospy.ROSInterruptException:
        pass
