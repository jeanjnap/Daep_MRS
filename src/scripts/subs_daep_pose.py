#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped

from mrs_msgs.srv import PathSrv, PathSrvRequest
from mrs_msgs.msg import Reference
from rpl_exploration.msg import Goal
from tf.transformations import euler_from_quaternion


global goal_x, goal_y, goal_z, goal_yaw
global pose_x, pose_y, pose_z, pose_yaw

global curr_pose_x, curr_pose_y, curr_pose_z
global curr_goal_x, curr_goal_y, curr_goal_z

goal_x = 0
goal_y = 0
goal_z = 0
goal_yaw = 0

pose_x = 0
pose_y = 0
pose_z = 0
pose_yaw = 0

curr_pose_x, curr_pose_y, curr_pose_z = -1, -1, -1


def send_mrs_trajectory(
        pos_x = [0.0], 
        pos_y = [0.0], 
        pos_z = [0.0], 
        yaw = [0.0], 
        max_speed = 8.0,
        drone_number = 1
    ):
    topic = '/uav1/trajectory_generation/path'
    rospy.wait_for_service(topic)

    try:
        path_service = rospy.ServiceProxy(topic, PathSrv)

        request = PathSrvRequest()
        request.path.header.stamp = rospy.Time.now()
        request.path.fly_now = True
        request.path.stop_at_waypoints = False
        request.path.loop = False

        request.path.override_constraints = True
        request.path.override_max_velocity_horizontal = max_speed
        request.path.override_max_acceleration_horizontal = 4.0
        request.path.override_max_jerk_horizontal = 60.0
        request.path.override_max_velocity_vertical = 4.0
        request.path.override_max_acceleration_vertical = 2.0
        request.path.override_max_jerk_vertical = 60.0
        
        request.path.use_heading = True
        request.path.relax_heading = False

        # Modify to follow the next point
        if len(yaw) == 1 and yaw[0] == 0:
            yaw = [0] * len(pos_x) 

        for px, py, pz, yw in zip(pos_x, pos_y, pos_z, yaw):
            wp = Reference()
            wp.position.x = px
            wp.position.y = py
            wp.position.z = pz
            wp.heading = yw
            request.path.points.append(wp)

        response = path_service(request)

        if response.success:
            rospy.loginfo('Path request successful: %s', response.message)
        else:
            rospy.logwarn('Path request failed: %s', response.message)

    except rospy.ServiceException as e:
        rospy.logerr('Service call failed: %s', str(e))


# def goal_callback(data):
#     global goal_x, goal_y, goal_z, goal_yaw
#     global curr_goal_x, curr_goal_y, curr_goal_z

#     arr_x = [data.x]
#     arr_y = [data.y]
#     arr_z = [data.z]
#     arr_yaw = [data.yaw]

#     curr_goal_x, curr_goal_y, curr_goal_z = data.x, data.y, data.z
#     if curr_goal_x == curr_pose_x and curr_goal_y == curr_pose_y and curr_goal_z == curr_pose_z:
#         if goal_x != arr_x[0] or goal_y != arr_y[0] or goal_z != arr_z[0] or goal_yaw != arr_yaw[0]:
#             send_mrs_trajectory(arr_x, arr_y, arr_z, arr_yaw)
#             goal_x = arr_x[0]
#             goal_y = arr_y[0]
#             goal_z = arr_z[0]
#             goal_yaw = arr_yaw[0]


# def pose_callback(data):
#     global pose_x, pose_y, pose_z, pose_yaw
#     global curr_pose_x, curr_pose_y, curr_pose_z

#     (_, _, current_yaw) = euler_from_quaternion([data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w])
#     arr_x = [data.pose.position.x]
#     arr_y = [data.pose.position.y]
#     arr_z = [data.pose.position.z]
#     arr_yaw = [current_yaw]

#     curr_goal_x, curr_goal_y, curr_goal_z = arr_x[0], arr_y[0], arr_z[0]
#     if pose_x != arr_x[0] or pose_y != arr_y[0] or pose_z != arr_z[0] or pose_yaw != arr_yaw[0]:
#         send_mrs_trajectory(arr_x, arr_y, arr_z, arr_yaw)
#         pose_x = arr_x[0]
#         pose_y = arr_y[0]
#         pose_z = arr_z[0]
#         pose_yaw = arr_yaw[0]


# def pose_subscriber():
#     rospy.init_node('pose_subscriber_daep', anonymous=True)

#     rospy.Subscriber('/mrs/goal', PoseStamped, pose_callback)
#     rospy.Subscriber("/aeplanner/goal", Goal, goal_callback)

#     rospy.spin()


# if __name__ == '__main__':
#     try:
#         pose_subscriber()
#     except rospy.ROSInterruptException:
#         pass


import rospy
from geometry_msgs.msg import PoseStamped
from rpl_exploration.msg import Goal  
from tf.transformations import euler_from_quaternion

class PoseSubscriber:
    def __init__(self):
        self.goal_x, self.goal_y, self.goal_z, self.goal_yaw = 0.0, 0.0, 0.0, 0.0
        self.curr_goal_x, self.curr_goal_y, self.curr_goal_z = 0.0, 0.0, 0.0

        self.pose_x, self.pose_y, self.pose_z, self.pose_yaw = 0.0, 0.0, 0.0, 0.0
        self.curr_pose_x, self.curr_pose_y, self.curr_pose_z = -1.0, -1.0, -1.0

        rospy.init_node('pose_subscriber_daep', anonymous=True)

        rospy.Subscriber('/mrs/goal', PoseStamped, self.pose_callback)
        rospy.Subscriber("/aeplanner/goal", Goal, self.goal_callback)

        rospy.spin()


    def goal_callback(self, data):
        print('GOAL CALLBACK')
        arr_x = [data.x]
        arr_y = [data.y]
        arr_z = [data.z]
        arr_yaw = [data.yaw]

        self.curr_goal_x, self.curr_goal_y, self.curr_goal_z = data.x, data.y, data.z
        if (self.goal_x != arr_x[0] or
            self.goal_y != arr_y[0] or
            self.goal_z != arr_z[0] or
            self.goal_yaw != arr_yaw[0]):

            send_mrs_trajectory(arr_x, arr_y, arr_z, arr_yaw)
            self.goal_x = arr_x[0]
            self.goal_y = arr_y[0]
            self.goal_z = arr_z[0]
            self.goal_yaw = arr_yaw[0]


    def pose_callback(self, data):
        print('POSE CALLBACK')
        (_, _, current_yaw) = euler_from_quaternion([data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w])
        arr_x = [data.pose.position.x]
        arr_y = [data.pose.position.y]
        arr_z = [data.pose.position.z]
        arr_yaw = [current_yaw]

        self.curr_goal_x, self.curr_goal_y, self.curr_goal_z = arr_x[0], arr_y[0], arr_z[0]
        if (self.curr_goal_x == self.curr_pose_x and
            self.curr_goal_y == self.curr_pose_y and
            self.curr_goal_z == self.curr_pose_z):

            if (self.pose_x != arr_x[0] or
                self.pose_y != arr_y[0] or
                self.pose_z != arr_z[0] or
                self.pose_yaw != arr_yaw[0]):

                send_mrs_trajectory(arr_x, arr_y, arr_z, arr_yaw)
                self.pose_x = arr_x[0]
                self.pose_y = arr_y[0]
                self.pose_z = arr_z[0]
                self.pose_yaw = arr_yaw[0]


if __name__ == '__main__':
    try:
        pose_subscriber = PoseSubscriber()
    except rospy.ROSInterruptException:
        pass
