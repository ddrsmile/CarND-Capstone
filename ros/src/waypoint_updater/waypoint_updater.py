#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

from math import cos, sin, sqrt
from tf.transformations import euler_from_quaternion

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 50 # Number of waypoints we will publish. You can change this number
MPH_TO_MPS = 0.44704  # simple conversion constant


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        self.sub_cur_pose = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1)
        self.sub_cur_vel  = rospy.Subscriber('/current_velocity', TwistStamped, self.velocity_cb, queue_size=1)
        self.sub_base_wp  = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb, queue_size=1)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.time = None
        self.pose = None
        self.lin_vel = 0.0
        self.ang_vel = 0.0
        self.base_wp = None
        self.desired_vel = 20.0

        rospy.spin()

    def pose_cb(self, msg):
        """
        Processes the reception of the /current_pose message. This function is also responsible for generating the
        next set of waypoints required for the /final_waypoints and publish the message.
        :param msg: A PoseStamped object
        :return: None
        """
        self.time = msg.header.time
        self.pose = msg.pose

        if self.base_wp is not None:
            index = self.__get_closest_waypoint()
            lookahead_waypoints = self.__get_next_waypoints(index, LOOKAHEAD_WPS)

            for waypoint in lookahead_waypoints:
                waypoint.twist.twist.linear.x = self.desired_vel * MPH_TO_MPS
            
            lane = self.__make_lane(msg.header.frame_id, lookahead_waypoints)
            self.pub_final_waypoints.publish(lane)

    def waypoints_cb(self, msg):
        # copy the waypoints
        self.base_wp = msg.waypoints

        # Unsubscribe, since this is no longer requried
        self.sub_base_wp.unregister()

    def velocity_cb(self, msg):
        self.lin_vel = msg.twist.linear.x
        self.ang_vel = msg.twist.angular.z

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def __get_closest_waypoint(self):
        best_gap = float('inf')
        best_index = 0
        my_position = self.pose.position

        for i, waypoint in enumerate(self.base_wp):
            other_position = waypoint.pose.pose.position
            gap = self.__distance(my_position, other_position)

            if gap < best_gap:
                best_index, best_gap = i, gap
        
        is_behide = self.__is_waypoint_behind(self.base_wp[best_index])
        best_index = best_index + 1 if is_behide else best_index
        return best_index
    
    def __is_waypoint_behind(self, waypoint):
        _, _, yaw = euler_from_quaternion([
            self.pose.orientation.x,
            self.pose.orientation.y,
            self.pose.orientation.z,
            self.pose.orientation.w,
        ])

        origin_x = self.pose.position.x
        origin_y = self.pose.position.y

        shifted_x = waypoint.pose.pose.position.x - origin_x
        shifted_y = waypoint.pose.pose.position.y - origin_y

        x = (shift_x * cos(0.0 - yaw)) - (shift_y * sin(0.0 - yaw))

        return x <= 0.0
    
    def __get_next_waypoints(self, i, n):
        m = min(len(self.base_wp), i + n)
        return self.base_wp[i:m]
    
    @classmethod
    def __make_lane(frame_id, waypoints):
        lane = Lane()
        lane.header.frame_id = frame_id
        lane.header.stamp = rospy.Time.now()
        lane.waypoints = waypoints
        return lane

    @classmethod
    def __distance(a, b):
        return sqrt(((a.x - b.x) ** 2) + ((a.y - b.y) ** 2))

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
