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
import time

STATE_COUNT_THRESHOLD = 3
TEST_MODE_ENABLED = False
SAVE_IMAGES = False
IMAGE_SAMPLING_RATE = 2
IMAGE_SAVING_RATE = 2
LOOKAHEAD_WPS = 100

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.base_waypoints = None
        self.waypoints_2d = None
        self.waypoint_tree = None
        self.camera_image = None
        self.lights = []

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=10)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
#        self.light_classifier = TLClassifier(rospy.get_param('~model_file'))
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0
        self.class_count = 0
        self.process_count = 0
        self.img_count = 0
        self.prev_class = 0
        self.prev_class_count = 0


        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.base_waypoints = waypoints
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in
                                 waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.img_count +=1
        #rospy.logwarn("Image count: {0}".format(self.img_count))
        if (self.img_count % IMAGE_SAMPLING_RATE == 0):
            
            self.has_image = True
            self.camera_image = msg
            light_wp, state = self.process_traffic_lights()
            #car_wp_idx = self.get_closest_waypoint(self.pose.pose.position.x, self.pose.pose.position.y)
            #if light_wp <= (car_wp_idx + LOOKAHEAD_WPS):
                #rospy.logwarn("DETECTED A CLOSE TF: line_wp_idx={}, state={}".format(light_wp, self.to_string(state)))
                
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
                self.last_wp  = light_wp if state == TrafficLight.RED else -1
                #self.last_wp = light_wp
                #self.upcoming_red_light_pub.publish(Int32(self.last_wp))
            #else:
                #self.upcoming_red_light_pub.publish(Int32(self.last_wp))
            self.state_count += 1
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))

    def get_closest_waypoint(self, x, y):
        """
        Identifies the closest path waypoint to the given position
        https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        :param x: x coord position to match a waypoint to
        :param y: y coord position to match a waypoint to

        :return: index of the closest waypoint in self.waypoints

        """
        closest_idx = self.waypoint_tree.query([x, y], 1)[1]
        return closest_idx

    def to_string(self, state):
        out = "unknown"
        if state == TrafficLight.GREEN:
            out = "green"
        elif state == TrafficLight.YELLOW:
            out = "yellow"
        elif state == TrafficLight.RED:
            out = "red"
        return out


    def get_light_state(self, light):
        """
        Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        # For test mode, just return the light state
        if TEST_MODE_ENABLED:
            classification = light.state
#        else:

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

            # Get classification
        if not TEST_MODE_ENABLED:
            classification = self.light_classifier.get_classification(cv_image)

            # Save image
        if SAVE_IMAGES and (self.process_count % IMAGE_SAVING_RATE == 0):
            if (self.prev_class == classification and self.prev_class_count < 1) or self.prev_class_count == 0 :
             self.prev_class = classification
             self.prev_class_count += 1
             save_file = "../../../imgs/dataset/{}-{:.0f}.jpeg".format(self.to_string(classification), (time.time() * 100))
             cv2.imwrite(save_file, cv_image)
             rospy.logwarn("Saving image for : {0}".format(self.to_string(classification)))
            elif (not (self.prev_class == classification)):
             self.prev_class_count -= 1
             save_file = "../../../imgs/dataset/{}-{:.0f}.jpeg".format(self.to_string(classification), (time.time() * 100))
             cv2.imwrite(save_file, cv_image)
             rospy.logwarn("Saving image for : {0}".format(self.to_string(classification)))
             
             
#        rospy.logwarn("TrafficLight classification: {0}".format(classification))
        return classification
        
        

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        closest_light = None
        line_wp_idx = -1
        state = TrafficLight.UNKNOWN
        
        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if self.pose and self.waypoint_tree:
            car_wp_idx = self.get_closest_waypoint(self.pose.pose.position.x, self.pose.pose.position.y)
            
            diff = len(self.base_waypoints.waypoints)
            for i, light in enumerate(self.lights):
                line = stop_line_positions[i]
                temp_wp_idx = self.get_closest_waypoint(line[0], line[1])
                d = temp_wp_idx - car_wp_idx
                if 0 <= d < diff:
                    diff = d
                    closest_light = light
                    line_wp_idx = temp_wp_idx

        if closest_light and line_wp_idx <= (car_wp_idx + LOOKAHEAD_WPS):
            self.process_count += 1
            state = self.get_light_state(closest_light)
            #rospy.logwarn("DETECTED A CLOSE TF: line_wp_idx={}, state={}".format(line_wp_idx, self.to_string(state)))
             #if (self.process_count % IMAGE_SAMPLING_RATE) == 0:
             #    rospy.logwarn("DETECT: line_wp_idx={}, state={}".format(line_wp_idx, self.to_string(state)))
        return line_wp_idx, state
        #self.waypoints = None
        #return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')

