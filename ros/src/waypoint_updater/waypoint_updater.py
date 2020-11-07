#!/usr/bin/env python

# ROS module imports
import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
#Non-ROS import
from scipy.spatial import KDTree
import numpy as np
import math

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

"""
Parts of this coding are from previous udacity projects and there is referred material to Udacity courses
"""

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number
DEBUGGER = False
STOPPING_DISTANCE = 10
SPEED_MPH = 20
DECELARATION_LIMIT = 0.45


class WaypointUpdater(object):
    def __init__(self):
        rospy.logerr("In Waypoint UPDATER")
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        #rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
 

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        #rospy.Subscriber('/traffic_waypoint', Lane, self.traffic_cb)
        #rospy.Subscriber('/obstacle_waypoint', Lane, self.obstacle_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.pose = None
        self.base_waypoints = None
        self.waypoints_2d = None
        self.waypoint_tree = None
        self.red_trafficlight_waypoint = None

        rospy.spin()
        
    def loop(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            if self.pose and self.base_waypoints:
                closest_waypoint_index = self.get_closest_waypoint_index()
                self.publish_waypoints(closest_waypoint_index)
            rate.sleep()
            

    def get_closest_waypoint_index(self):
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        closest_index = self.waypoint_tree.query([x,y], 1)[1]
        closest_coord = self.waypoints_2d[closest_index]
        previous_coord = self.waypoints_2d[closest_index-1]
        
        closest_vector = np.array(closest_coord)
        prev_vector = np.array(previous_coord)
        position_vector = np.array([x, y])
        
        value = np.dot(closest_vector - prev_vector, position_vector - closest_vector)
        
        #Waypoint behind current car position
        if value > 0:
            closest_index = (closest_index + 1) % len(self.waypoints_2d)
        return closest_index
        
    
    def pose_cb(self, msg):
        # TODO: Implement
        self.pose = msg
        #if self.base_waypoints is not None:
        #    self.publish_waypoints()
            

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        #if waypoints is not None:
        self.base_waypoints = waypoints
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)
            
    def traffic_cb(self, msg):  # course
        # TODO: Callback for /traffic_waypoint message. Implement
        self.stopline_wp_inx = msg.data
        rospy.loginfo("Traffic light is : " + str(msg.data))
        if self.stopline_wp_inx > -1:
            self.publish()

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

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

    def nearest_waypoint(self, pose, waypoints):
        nearest_waypoint = 0
        closest_len = 100000
        
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2)
        for wp_index, waypoint in enumerate(self.base_waypoints):
            dist = dl(pose.position, waypoint.pose.pose.position)
            if (dist < closest_len):
                nearest_waypoint = wp_index
                closest_len = dist
                
        return nearest_waypoint
       
    def next_waypoint(self, pose, waypoints):
        nearest_waypoint = self.nearest_waypoint(pose, waypoints)
        wp_x = waypoints[nearest_waypoint].pose.pose.position.x
        wp_y = waypoints[nearest_waypoint].pose.pose.position.y
        
        heading = math.atan2((wp_y-pose.position.y), (wp_x-pose.position.x))
        quarternion = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        _,_,yaw = tf.transformations.euler_from_quarternion(quarternion)
        angle = abs(yaw-heading)
        
        if angle > (math.pi/4):
            nearest_waypoint += 1
            
        return nearest_waypoint

    def generate_lane(self):
        lane = Lane()
        
        closest_wp_index = self.get_closest_waypoints_index()
        farthest_wp_index = closest_wp_index + LOOKAHEAD_WPS
        base_waypoints = self.base_lane.waypoints[closest_wp_index:farthest_wp_index]
        
        if self.stopline_wp_index == -1 or (self.stopline_wp_index >= farthest_wp_index):
            lane.waypoints = base_waypoints
        else:
            lane.waypoints = self.decelerate_waypoints(base_waypoints, closest_wp_index)
        
        return lane
    
    def decelerate_waypoints(self, waypoints, closest_index):
        temp_waypoints = []
        print("Waypoints : ", waypoints)
        print("Closest WP index : ", closest_index)
        for i, wp in enumerate(waypoints):
            p = Waypoint()
            p.pose = wp.pose
            
            stop_index = max(self.stopline_wp_index - closest_index - 2, 0)
            distance = self.distance(waypoints, i, stop_index)
            velocity = math.sqrt(2* MAX_DECEL * dist)
            # Can also implement a S curve implementation here
            if velocity < 1.0:
                velocity = 0.
            
            p.twist.twist.linear.x = min(velocity, wp.twist.twist.linear.x)
            temp_waypoints.append(p)

        return temp_waypoints
            
        
    def publish_waypoints(self, closest_wp_index):
        lane = Lane()
        lane.header = self.base_waypoints.header
        lane.waypoints = self.based_waypoints.waypoints[closest_wp_index:closest_wp_index + LOOKAHEAD_WPS]
        self.final_waypoints_pub.publish(lane)       

if __name__ == '__main__':
    try:
        rospy.logerr("In Waypoint updater")
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
