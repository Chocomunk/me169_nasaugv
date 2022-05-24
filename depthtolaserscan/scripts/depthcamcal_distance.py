#!/usr/bin/env python3
#
#   depthcamcal_distance.py
#
#   Depth Camera Calibration: Distance.  This determines the distance
#   to the wall.
#
#   Node:       /depthtocamcal_distance
#
#   Arguments:  pitch           Optional.  Else from TF.    
#               hmin            Optional.  Else from ROS parameter.
#               hmax            Optional.  Else from ROS parameter.
#
#   Params:     ~ground_frame   Frame of the ground (robot base)
#
#               ~min_height     Min relative height of wall for calibration
#               ~max_height     Max relative height of wall for calibration
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
#   Constants
#
IMAGES = 15             # Images over which to average


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
            self.pitch = float(argv[1])
        else:
            # No command line value available.
            rospy.loginfo("No pitch angle specified...")
            self.pitch = grabCameraPitch()

        # Determine the min/max relative height.
        if (len(argv) > 3):
            # Use the command line if available.
            self.hmin = float(argv[2])
            self.hmax = float(argv[3])
        else:
            # No command line value available.
            self.hmin = rospy.get_param('~min_height', MIN_HEIGHT)
            self.hmax = rospy.get_param('~max_height', MAX_HEIGHT)

        # Convert this into pixel coordinate.
        self.ucenter  = int(round(self.cu))
        self.vcenter  = int(round(self.cv))
        self.vhorizon = int(round(self.cv + self.fv * math.tan(self.pitch)))

        # Initialize the distance numbers.
        self.index = 0
        self.N  = np.zeros(IMAGES)
        self.R  = np.zeros(IMAGES)
        self.R2 = np.zeros(IMAGES)

        # Report.
        rospy.loginfo("Using pitch angle of %6.3fdeg (%4.3frad)" %
                      (math.degrees(self.pitch), self.pitch))
        rospy.loginfo("Corresponding to image pixel row %d" % self.vhorizon)
        rospy.loginfo("Scanning data %5.3fm to %5.3fm above camera height" %
                      (self.hmin, self.hmax))

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

        # Extract the center column.
        centerline = depth[:, self.ucenter]

        # Precompute the sin/cos of pitch
        (sp, cp) = (math.sin(self.pitch), math.cos(self.pitch))

        # Run through all pixels
        (N, R, R2)   = (0, 0.0, 0.0)
        (vmin, vmax) = (Nv, 0)
        for v in range(Nv):
            #  Place the pixel in space (r forward, z up)
            d    = float(centerline[v]) * 0.001
            vbar = (float(v) - self.cv) / self.fv
            r    = d * (cp + sp * vbar)
            h    = d * (sp - cp * vbar)

            # Should we use the point to fit?
            if (d > 0.0) and (h < self.hmax) and (h > self.hmin):
                N  += 1
                R  += r
                R2 += r**2
                vmin = min(v,   vmin)
                vmax = max(v+1, vmax)

        # Update the index and add the info.
        self.index          = (self.index + 1) % IMAGES
        self.N[self.index]  = N
        self.R[self.index]  = R
        self.R2[self.index] = R2

        # Report.
        if self.index == 0:
            # Combine all image data.
            N  = np.sum(self.N)
            R  = np.sum(self.R)
            R2 = np.sum(self.R2)

            # Compute the mean and standard deviation.
            if   N < 1:  (r, s) = (0.0, math.inf)
            elif N < 2:  (r, s) = (R,   math.inf)
            else:
                r = R/float(N)
                s = math.sqrt((R2 - float(N)*r**2)/float(N-1))

            # Report.
            print("Pitch %5.3fdeg: r = %6.3fm std %5.3fm for %d samples" %
                  (math.degrees(self.pitch), r, s, N))
        
        # Overlay the horizon and vertical line.
        overlay = np.copy(depth)
        udash   = [i for i in range(self.Nu) if (i%(2*DASH_LEN)<DASH_LEN)]
        overlay[self.vhorizon, udash]    = 10000
        overlay[vmin:vmax, self.ucenter] = 10000

        # Publish the new depth image.
        imagemsg.data = overlay.tobytes()
        self.pub.publish(imagemsg)


#
#   Main Code
#
if __name__ == "__main__":
    # Initialize the ROS node.
    rospy.init_node('depthtocamcal_distance')

    # Report.
    print("")
    print("Usage: depthcamcal_distance [pitch] [hmin] [hmax]")
    print("")
    print(" [pitch]    Optional pitch angle, else use URDF parameters")
    print(" [hmin]     Optional minimum relative height to use on wall")
    print(" [hmax]     Optional maximum relative height to use on wall")
    print("")
    print("Check the distance to the wall in the vertical center line.")
    print("")
    print("Calibration process:")
    print("  1. Place robot facing a vertical wall")
    print("  2. Compare the actual to reported distance")
    print("")

    # Instantiate the depth camera calibration object
    _ = Calibrator()

    # Report and spin (waiting while callbacks do their work).
    rospy.loginfo("Spinning...")
    rospy.spin()
    rospy.loginfo("Stopped.")
