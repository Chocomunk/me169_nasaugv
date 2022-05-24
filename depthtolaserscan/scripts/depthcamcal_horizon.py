#!/usr/bin/env python3
#
#   depthcamcal_horizon.py
#
#   Depth Camera Calibration: Horizon.  This draws the horizon line on
#   the depth image, given the camera's pitch angle.
#
#   Node:       /depthtocamcal_horizon
#
#   Arguments:  pitch           Optional.  Else from TF.    
#
#   Params:     ~ground_frame   Frame of the ground (robot base)
#
#   Subscribe:  /camera/depth/camera_info       sensor_msgs/CameraInfo
#               /camera/depth/image_rect_raw    sensor_msgs/Image
#
#   Publish:    /camera/depth/image_overlaid    sensor_msgs/Image
#

# Imports
import math
import numpy as np
import sys
import rospy

from depthcamcal_utilities import *

from sensor_msgs.msg import Image


#
#   Calibrator Object
#
class Calibrator:
    # Initialization
    def __init__(self):
        # Grab the camera's intrinsic parameters
        ((self.Nu, self.cu, self.fu),
         (self.Nv, self.cv, self.fv)) = grabCameraInfo()

        # Get the command line arguments.
        argv = rospy.myargv(argv=sys.argv)

        # Determine the pitch angle.
        if (len(argv) > 1):
            # Use the command line if available.
            pitch = float(argv[1])
        else:
            # No command line value available.
            rospy.loginfo("No pitch angle specified...")
            pitch = grabCameraPitch()

        # Convert this into pixel coordinate.
        self.ucenter  = int(round(self.cu))
        self.vcenter  = int(round(self.cv))
        self.vhorizon = int(round(self.cv + self.fv * math.tan(pitch)))

        # Report.
        rospy.loginfo("Using pitch angle of %6.3fdeg (%4.3frad)" %
                      (math.degrees(pitch), pitch))
        rospy.loginfo("Corresponding to image pixel row %d" % self.vhorizon)

        # Create a publisher for the overlaid depth images.
        self.pub = rospy.Publisher(OVERLAY_TOPIC, Image, queue_size=10)
        
        # Create a subscriber to listen to the depth images.  Using a
        # queue size of one means only the most recent message is stored
        # for the next subscriber callback.  Skip backed-up messages.
        _ = rospy.Subscriber(IMAGE_TOPIC, Image, self.cb_depthimage,
                             queue_size=1)

    ######################################################################
    # Depth Image Callback
    def cb_depthimage(self, imagemsg):
        # Report (for debugging).
        #rospy.loginfo("Image #%4d at %10d secs" %
        #              (imagemsg.header.seq, imagemsg.header.stamp.secs))

        # Check the image width/height.
        if (imagemsg.width != self.Nu) or (imagemsg.height != self.Nv):
            rospy.logerr("Depth image has changed size (%dx%d) vs (%dx%d)!" %
                         (imagemsg.width, imagemsg.height, self.Nu, self.Nv))
            return

        # Check the data encoding.
        encoding = '16UC1'
        if (imagemsg.encoding != encoding):
            rospy.logerr("Depth image encoding '%s' different from '%s'!" %
                         (imagemsg.encoding, encoding))
            return

        # Extract the depth image information (distance in mm as uint16).
        # And reshape into a (NuxNv) image.
        (Nu,Nv) = (imagemsg.width,imagemsg.height)
        depth   = np.frombuffer(imagemsg.data, np.uint16).reshape(Nv,Nu)

        # Overlay the horizon line.
        overlay = np.copy(depth)
        udash   = [i for i in range(self.Nu) if (i%(2*DASH_LEN)<DASH_LEN)]
        overlay[self.vhorizon, udash] = 10000

        # Publish the new depth image.
        imagemsg.data = overlay.tobytes()
        self.pub.publish(imagemsg)


#
#   Main Code
#
if __name__ == "__main__":
    # Initialize the ROS node.
    rospy.init_node('depthtocamcal_horizon')

    # Report.
    print("")
    print("Usage: depthcamcal_horizon [pitch]")
    print("")
    print(" [pitch]    Optional pitch angle, else use URDF parameters")
    print("")
    print("Draw the horizon line on the depth image (image_overlaid).")
    print("IF YOU DO NOT SEE THE LINE, ENLARGE THE IMAGE DISPLAY.")
    print("")
    print("Calibration process:")
    print("  1. Place object in front of camera, at same height")
    print("     (so that it indicates the horizon")
    print("  2. Adjust pitch (restarting this code) until overlay matches")
    print("  3. Possibly fix the pitch in the URDF - WHICH APPEARS TWICE")
    print("")

    # Instantiate the depth camera calibration object
    _ = Calibrator()

    # Report and spin (waiting while callbacks do their work).
    rospy.loginfo("Spinning...")
    rospy.spin()
    rospy.loginfo("Stopped.")
