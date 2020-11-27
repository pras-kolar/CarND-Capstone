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

LOOKAHEAD_WPS = 30 # Number of waypoints we will publish. You can change this number
DEBUGGER = False
STOPPING_DISTANCE = 10
SPEED_MPH = 10
DECELARATION_LIMIT = 0.45
TRAFFIC_STOP_DIST = 2
SMOOTH_DECEL = 1/LOOKAHEAD_WPS


class WaypointUpdater(object):
    def __init__(self):
        rospy.logerr("In Waypoint UPDATER")
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)


        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        
        # Presently obstacles are not handled ; it can be done at a later time
        #rospy.Subscriber('/obstacle_waypoint', Lane, self.obstacle_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        # Pose, waypoints, deceleration and related variables below
        self.pose = None
        self.base_lane = None
        self.waypoints_2d = None
        self.waypoint_tree = None
        self.red_trafficlight_waypoint = -1
        self.decelerate_count = 0.

        self.loop()
        
    def loop(self):
        """
        Function    : Common highlevel looper that calls required codes
        
        Input Args  : None
        Output      : None
        """
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            if self.pose and self.base_lane:
                self.publish_waypoints()
            rate.sleep()
            

    def get_closest_waypoint_idx(self):
        """
        Function    : Calculate the index of the closest waypoint
        
        Input Args  : None
        Output      : Index of closest waypoint
        """
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        if self.waypoint_tree is None:
            return 
        # Calculate the index of the closest waypoint, calculate the related coordinate
        closest_index = self.waypoint_tree.query([x,y],1)[1]
        closest_coord = self.waypoints_2d[closest_index]
        
        # Calculate the index of the previous waypoint of the closest waypoint
        previous_coord = self.waypoints_2d[closest_index-1]

        # Create a vector using the closest coordinates
        closest_vector = np.array(closest_coord)
        prev_vector = np.array(previous_coord)
        position_vector = np.array([x, y])
        
        value = np.dot(closest_vector - prev_vector, position_vector - closest_vector)

        #Waypoint behind current car position
        if value > 0:
            closest_index = (closest_index + 1) % len(self.waypoints_2d)
        return closest_index
        
    #pose callback that populates the values in the pose class variable    
    def pose_cb(self, msg):
        """
        Function    : Function that copies the topic information to the class pose 
        
        Input Args  : topic message
        Output      : None
        """        
        # TODO: Implement
        self.pose = msg
       
    def waypoints_cb(self, waypoints):
        """
        Function    : waypoint callback that populates 
        the values in the waypoint class variable 
        
        Input Args  : waypoints
        Output      : None
        """        
        # TODO: Implement
        self.base_lane = waypoints
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)
        rospy.logerr("In waypoints cb")         

    
    def traffic_cb(self, msg):  # course
        """
        Function    : traffic light callback that performs traffic 
        and related data processing
        
        Input Args  : traffic data
        Output      : None
        """                
        rospy.logerr("In traffic cb")
        # TODO: Callback for /traffic_waypoint message. Implement
        self.red_trafficlight_waypoint = msg.data
        rospy.loginfo("Traffic light is : " + str(msg.data))

    #Obstacke callback that will be implemented later
    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        """
        Function    : traffic light callback that performs traffic 
        and related data processing
        
        Input Args  : Waypoints
        Output      : Velocity component
        """                        
        rospy.logerr("In GET waypoints VELOCITY")
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        """
        Function    : Sets velocity for the waypoint in question
        
        Input Args  : Waypoints waypoint and velocity components
        Output      : None
        """
        rospy.logerr("In SET waypoints VELOCITY")
        waypoints[waypoint].twist.twist.linear.x = velocity

    #Calculate the distance between 2 waypoints
    def distance(self, waypoints, wp1, wp2):
        """
        Function    : Calculates distance between 2 waypoints 
        
        Input Args  : 2 waypoints
        Output      : distance component
        """
        #rospy.logerr("DISTANCE")
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def generate_lane(self):
        """
        Function    : Code to generate lane related waypoints
        
        Input Args  : None
        Output      : Lane components
        """
        #rospy.logerr("In GENERATE LANE ")
        lane = Lane()
        closest_wp_index = self.get_closest_waypoint_idx()

        farthest_wp_index = closest_wp_index + LOOKAHEAD_WPS
        base_waypoints = self.base_lane.waypoints[closest_wp_index:farthest_wp_index]
        #Continue driving if not red light and if away from farthest waypoint
        if (self.red_trafficlight_waypoint == -1) or (self.red_trafficlight_waypoint >= farthest_wp_index):
            lane.waypoints = base_waypoints
        else:
            # Decelerate the car if red
            lane.waypoints = self.decelerate_waypoints(base_waypoints, closest_wp_index)
        
        return lane

    
    def decelerate_waypoints(self, waypoints, closest_index):
        """
        Function    : Decelerate the car if red light and stop close to it
        
        Input Args  : Waypoints and closest Red light index
        Output      : temporary deceleration waypoints
        """
        rospy.logerr("In Decelerate waypoints")
        temp_waypoints = []
        print("Waypoints : ", waypoints)
        print("Closest WP index : ", closest_index)
        for i, wp in enumerate(waypoints):
            p = Waypoint()
            p.pose = wp.pose
            
            stop_index = max(self.red_trafficlight_waypoint - closest_index - 3, 0)
            distance = self.distance(waypoints, i, stop_index)
            velocity = math.sqrt(2* DECELARATION_LIMIT * distance) #+ (i * SMOOTH_DECEL)  #SMOOTH_DECEL needed for smoother slowing
            # Can also implement a S curve implementation here
            if velocity < 1.0:
                velocity = 0.
            
            p.twist.twist.linear.x = min(velocity, wp.twist.twist.linear.x)
            temp_waypoints.append(p)
            
        self.decelerate_count += 1
        if (self.decelerate_count % 100) == 0.0:
            wp_size = len(waypoints) -1
            velocity_start = temp_waypoints[0].twist.twist.linear.x
            velocity_end = temp_waypoints[wp_size].twist.twist.linear.x
            # Logging the messages for information and documentation

        return temp_waypoints
    
    def publish_waypoints(self):
        """
        Function    : Simple publisher to Publish all the related waypoints; calculated and then passed here
        
        Input Args  : None
        Output      : None
        """        
        final_wp = self.generate_lane()
        self.final_waypoints_pub.publish(final_wp)
        
if __name__ == '__main__':
    try:
        rospy.logerr("In Waypoint updater")
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
