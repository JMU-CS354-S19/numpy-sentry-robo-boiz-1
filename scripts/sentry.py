#!/usr/bin/env python

""" 
SentryBot lets us know if an intruder walks past.

Author: 
Version:
"""

import rospy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np


class SentryNode(object):
    """Monitor a vertical scan through the depth map and create an
    audible signal if the change exceeds a threshold.

    Subscribes:
         /camera/depth_registered/image
       
    Publishes:
        /mobile_base/commands/sound

    """

    def __init__(self):
        """ Set up the Sentry node. """
        rospy.init_node('sentry')
        self.cv_bridge = CvBridge()
        rospy.Subscriber('/camera/depth_registered/image',
                         Image, self.depth_callback, queue_size=1)
        self.c = None
        self.p = None
        self.avg = 1
        self.alpha = .5
        self.threshold = .5
        rospy.spin()

    def depth_callback(self, depth_msg):
        """ Handle depth callbacks. """

        # Convert the depth message to a numpy array
        depth = self.cv_bridge.imgmsg_to_cv2(depth_msg)

        # YOUR CODE HERE.
        # HELPER METHODS ARE GOOD.
        x, y = depth.shape
        
        if self.c is not None:
            self.p = self.c
            
        self.c = depth[:, x/2] # extract central column
        
        self.c = self.c[~np.isnan(self.c)]
        
        if self.p is not None and self.c is not None:            
            diff_arr = self.c - self.p
            np.absolute(diff_arr)
            d = np.sum(diff_arr)
        
            #d = 10
            self.avg = self.avg * self.alpha + d * (1-self.alpha)
        
            if d/self.avg > self.threshold:
                rospy.loginfo("reached")
        #rospy.loginfo(depth[:, x/2])

if __name__ == "__main__":
    SentryNode()
