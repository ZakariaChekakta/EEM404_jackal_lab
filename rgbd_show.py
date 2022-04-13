#!/usr/bin/env python

# Author: Masoud Sotoodeh Bahraini
# City University of London
# sotoodeh.bahraini@city.ac.uk

# import roslib
import rospy
import sys
import cv2
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
import message_filters


class kinect_Demo():
    def __init__(self):
        self.node_name = "ucity_kinect"
        # Initialize the ros node
        rospy.init_node(self.node_name, anonymous=True)
        # What we do during shutdown
        rospy.on_shutdown(self.cleanup)
        # Create the cv_bridge object
        self.bridge = CvBridge()
        # subscribe to stereo images and set the appropriate callbacks
        rospy.loginfo("Create subscribers for each topic")
        self.rgb_image = message_filters.Subscriber(
            "/kinect2/qhd/image_color_rect", Image)
        self.depth_image = message_filters.Subscriber(
            "/kinect2/qhd/image_depth_rect", Image)
        rospy.loginfo(
            "subscribed to:  /camera/color/image_raw  and  /camera/depth/image_raw")
        rospy.loginfo(
            "Create sync filter. Use exact or approximate as appropriate.")
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_image, self.depth_image], queue_size=3, slop=0.1)
        rospy.loginfo("Registering callback")
        self.ts.registerCallback(self.image_callback)
        # Callback executed when the timer timeout
        rospy.Timer(rospy.Duration(0.03), self.show_img_cb)
        rospy.loginfo("Waiting for image topics to show ...")

    # Here is the callback to visualize the actual stereo images and processed stereo image:

    def show_img_cb(self, event=None):
        try:
            # initialize the windows
            cv2.namedWindow("RGB_Image", cv2.WINDOW_NORMAL)
            cv2.moveWindow("RGB_Image", 25, 75)
            cv2.namedWindow("Depth_Image", cv2.WINDOW_NORMAL)
            cv2.moveWindow("Depth_Image", 750, 75)
            # Left and Right images
            cv2.imshow("RGB_Image", self.cv_rgb_image)
            cv2.imshow("Depth_Image", self.cv_depth_image)
            cv2.waitKey(3)
        except:
            pass

    def image_callback(self, msg_rgb_image, msg_depth_image):
        # Use cv_bridge() to convert the ROS image to OpenCV format
        # convert images ROS -> OpenCV
        try:
            self.cv_rgb_image = self.bridge.imgmsg_to_cv2(
                msg_rgb_image, "rgb8")
        except CvBridgeError as e:
            rospy.logwarn('error converting rgb image: %s' % e)
            return
        try:
            self.cv_depth_image = self.bridge.imgmsg_to_cv2(msg_depth_image)
        except CvBridgeError as e:
            rospy.logwarn('error converting depth image: %s' % e)
            return

    # The following function will close the image window when the node shuts down:
    def cleanup(self):
        print("Shutting down vision node.")
        cv2.destroyAllWindows()


def main(args):
    try:
        kinect_Demo()
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down vision node.")
        cv2.DestroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
