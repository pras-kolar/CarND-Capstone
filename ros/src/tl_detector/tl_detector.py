#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
from scipy.spatial import KDTree

STATE_COUNT_THRESHOLD = 1
IMG_COUNT_THRESHOLD = 1
#traffic_light_state = ['Red', 'Yellow', 'Green', 'Unknown', 'Unknown']
traffic_light_state = ['Red', 'Yellow', 'Green', 'Unknown']

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.waypoints_2d = None
        self.waypoint_tree = None
        self.use_camera = True
        self.camera_image = None
        self.has_image = False
        self.lights = []
        self.img_count = 0

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        if self.config['is_site']:
            rospy.loginfo("Carla is being used")
        else:
            rospy.loginfo("Simulation is being used")            

        if self.use_camera:
            rospy.loginfo("Camera is being currently used")
        else:
            rospy.loginfo("Camera is not being used now")

        sub_pose = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub_lane = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light 
        in 3D map space and helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        
        sub_traffic = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        if self.config['is_site']:
            sub_image  = rospy.Subscriber('/image_raw', Image, self.image_cb)
        else:
            sub_image = rospy.Subscriber('/image_color', Image, self.image_cb)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)
        
        self.bridge = CvBridge()
        self.light_classifier = TLClassifier(self.config['is_site'])
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0
        
        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)

    # traffic callback from the subscriber
    # when the system receives trafic light image(s) they are processed
    # if a traffic light is found in the image, it gets the
    # closest waypoint 
    def traffic_cb(self, msg):
        self.lights = msg.lights
        if not self.use_camera and self.waypoints:
            light_wp, state = self.process_traffic_lights()
            '''
            Publish upcoming red lights at camera frequency.
            Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
            of times till we start using it. Otherwise the previous stable state is
            used.
            '''
            if self.state != state:
                self.state_count = 0
                self.state = state
            elif self.state_count >= STATE_COUNT_THRESHOLD:
                self.last_state = self.state
                light_wp = light_wp if state == TrafficLight.RED else -1
                self.last_wp = light_wp
                self.upcoming_red_light_pub.publish(Int32(light_wp))
            else:
                self.upcoming_red_light_pub.publish(Int32(self.last_wp))
            self.state_count += 1
        
    def image_cb(self, msg):
        """
        This function identifies red lights in the input camera images
        imput variables: image from the camera on the car
        output : Publishes the closest waypoint index to the red light's stopping line to /traffic_waypoint
        There is no format return       
        """
        if self.use_camera and self.waypoints:
            if self.img_count >= IMG_COUNT_THRESHOLD:
                self.img_count = 0
                self.has_image = True
                self.camera_image = msg
                light_wp, state = self.process_traffic_lights()
                '''
                Publish upcoming red lights at camera frequency.
                Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
                of times till we start using it. Otherwise the previous stable state is
                used.
                '''
                if self.state != state:
                    self.state_count = 0
                    self.state = state
                elif self.state_count >= STATE_COUNT_THRESHOLD:
                    self.last_state = self.state
                    light_wp = light_wp if state == TrafficLight.RED else -1
                    #rospy.loginfo("Publishing new waypoint")
                    self.upcoming_red_light_pub.publish(Int32(light_wp))
                    self.last_wp = light_wp
                else:
                    rospy.loginfo("Publishing old waypoints")
                    self.upcoming_red_light_pub.publish(Int32(self.last_wp))
                self.state_count += 1
            else:
                self.img_count += 1

    def get_closest_waypoint(self, x, y):
        """
        This function identifies the closest path waypoint of the car's given position
        Reference : https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        imput variables: position for waypoint matching
        Output Returns: index of the closest waypoint
        """
        #TODO implement
        closest_wp_idx = None
        if self.waypoint_tree:
            closest_wp_idx = self.waypoint_tree.query([x, y], 1)[1]
        return closest_wp_idx 

    def get_light_state(self, light):
        """
        This function obtains the color of the traffic light
        imput variables: TrafficLight : which gives the light to classify
        Output Returns: ID of traffic light color as given in styx_msgs/TrafficLight
        """
        if(not self.has_image):
            self.prev_light_loc = None
            rospy.loginfo("Traffic light not present in image")
            return light.state

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")
        return self.light_classifier.get_classification(cv_image, self.config['is_site'])

    def process_traffic_lights(self):
        """
        This function finds closest visible traffic light and determines its location and color
        imput variables: None
        Output Returns: index of waypoint closest to the upcoming stop line for a traffic light (-1 if none exists)
        ID of traffic light color
        """
        closest_light = None
        line_wp_idx = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        # default code uses pose directly.  But I wanted to try using the x,y coordinates for better performance
        if(self.pose):
            car_wp_idx = self.get_closest_waypoint(self.pose.pose.position.x, self.pose.pose.position.y)
        else:
            car_wp_idx = 0

        #TODO find the closest visible traffic light (if one exists)
        # Find the number of waypoints
        wp_len = len(self.waypoints.waypoints)
        for i, light in enumerate(self.lights):
            # Get stop line waypoint index
            line = stop_line_positions[i]
            interim_wp_idx = self.get_closest_waypoint(line[0], line[1])
            # Find closest stop line waypoint index
            dist_wp = interim_wp_idx - car_wp_idx
            if dist_wp >= 0 and dist_wp < wp_len:
                wp_len = dist_wp
                closest_light = light
                line_wp_idx = interim_wp_idx

        if closest_light:
            if wp_len < 180:
                state = self.get_light_state(closest_light)
            else:
                state = TrafficLight.UNKNOWN
            return line_wp_idx, state
  
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
    rospy.logerr("tl_detector done")