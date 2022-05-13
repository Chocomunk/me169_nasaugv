#!/usr/bin/env python3
#
#   depthcamcal.py
#
#   Calibrate the depth camera.  This assume the depth camera is
#   looking at a wall.  It reports the camera mounting angle and
#   distance to wall.  It assumes the camera is mounted horizontally!
#
#   Node:       /depthtocamcal
#
#   Params:     ~min_height         Min height of wall points for calibration
#               ~max_height         Max height of wall points for calibration
#
#   Subscribe:  /camera/depth/...
#                 .../camera_info       sensor_msgs/CameraInfo
#                 .../image_rect_raw    sensor_msgs/Image
#
#   Publish:    nothing
#
import math
import numpy as np
import rospy

from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg   import CameraInfo
from sensor_msgs.msg   import Image


#
#   Constants
#
TIMEOUT = 5.0           # Timeout for the initialization

INFO_TOPIC  = '/camera/depth/camera_info'
IMAGE_TOPIC = '/camera/depth/image_rect_raw'

MIN_HEIGHT = 0.100      # Min/Max vertical height relative to camera
MAX_HEIGHT = 1.500

PITCH_START      = math.radians(20.0)
PITCH_RANGE      = math.radians(30.0)
PITCH_RESOLUTION = math.radians(.01)


#
#   Mathematics (Coordinates)
#
#
#   Before we derive the algorithm, we define two coordinate frames in
#   the same location, but with different axes.  The Image frame is
#   horizontal, but pitched up.  The Laser frame is aligned with the
#   ground (Z vertical).
#
#     Image Frame:  X image right horizontal, Y image down, Z into image
#     Laser Frame:  X forward horizontal, Y left horizontal, Z vertical up
#
#   A point in space can be decribed in image frame (xi, yi, zi) or
#   laser frame (xl, yl, zl) and converted with:
#
#     xl = cos(pitch) zi + sin(pitch) yi  = cp * zi + sp * yi
#     yl = - xi                           = -xi
#     zl = sin(pitch) zi - cos(pitch) yi  = sp * zi - cp * yi
#
#     xi = yl                             = yl
#     yi = sin(pitch) xl - cos(pitch) zl  = sp * xl - cp * zl
#     zi = cos(pitch) xl + sin(pitch) zl  = cp * xl + sp * zl
#
#   The point (given in image frame) can also be mapped to the image
#   coordinates (u,v) and depth (d):
#
#     u = cu + fu * (xi/zi)
#     v = cv + fv * (yi/zi)
#     d = zi
#
#   where cu/cv are the center of the image, and fu/fv are the focal
#   lengths expressed in pixel size.  This again can be inverted as:
#
#     xi = d * (u-cu)/fu = d * ubar    ubar = (u-cu)/fu
#     yi = d * (v-cv)/fv = d * vbar    vbar = (v-cv)/fv
#     zi = d
#
#   And then the image coordinates can be directly mapped to the
#   laser frame (vertical up):
#
#     xl = d * (cp + sp * vbar)
#     yl = d * (         -ubar)
#     zl = d * (sp - cp * vbar)
#
#   so that the horizon (zl=0) is located on image row:
#
#     vbar_horizon =  sp/cp
#


#
#   Calibrator Object
#
class DepthCamCalibrator:
    # Initialization
    def __init__(self):
        # Grab the ROS parameters and camera image info.
        self.grabROSParameters()
        self.grabCameraInfo()

        # Create a subscriber to listen to the depth images.  Using a
        # queue size of one means only the most recent message is stored
        # for the next subscriber callback.  Skip backed-up messages.
        _ = rospy.Subscriber(IMAGE_TOPIC, Image, self.callback_depthimage,
                             queue_size=1, buff_size=20*self.Nu*self.Nv*2)
        self.pub = rospy.Publisher("/depth2", Image, queue_size=10)

    ######################################################################
    # Setup

    # Grab the ROS parameters.
    def grabROSParameters(self):
        rospy.loginfo("Grabbing the ROS parameters...")

        # Grab the private parameters.
        self.hmin = rospy.get_param('~min_height', MIN_HEIGHT)
        self.hmax = rospy.get_param('~max_height', MAX_HEIGHT)

        # Report.
        rospy.loginfo("Using data %5.3fm to %5.3fm above ground" %
                      (self.hmin, self.hmax))

    # Grab the depth camera image info.
    def grabCameraInfo(self):
        rospy.loginfo("Grabbing the depth camera info...")

        # Grab a single message of the camera_info topic.
        infomsg = rospy.wait_for_message(INFO_TOPIC, CameraInfo, TIMEOUT)

        # Note the width and height.
        self.Nu = infomsg.width
        self.Nv = infomsg.height

        # Pull the image resolution and center numbers from the projection
        # matrix.  note this maps an (x,y,z) point to (u=column,v=row) as:
        #    u = cu + fu * (x/z)
        #    v = cv + fv * (y/z)
        (self.fu, self.cu) = (infomsg.P[0], infomsg.P[2])
        (self.fv, self.cv) = (infomsg.P[5], infomsg.P[6])

        # Determine the field of  view angles.
        FOVu = math.atan(float(self.Nu)/self.fu/2.0) * 2.0
        FOVv = math.atan(float(self.Nv)/self.fv/2.0) * 2.0

        # Report.
        rospy.loginfo("Pixel: u = %7.3f + %7.3f * (x/z)" % (self.cu, self.fu))
        rospy.loginfo("       v = %7.3f + %7.3f * (y/z)" % (self.cv, self.fv))
        rospy.loginfo("Image %dx%d = %.0fx%.0f deg FOV" %
                      (self.Nu, self.Nv,
                       math.degrees(FOVu), math.degrees(FOVv)))


    ######################################################################
    # Depth Image Callback
    def callback_depthimage(self, imagemsg):
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
        # Reshape into a (NuxNv) image.  And extract the center column.
        depth = np.frombuffer(imagemsg.data, np.uint16)
        depth = depth.reshape(self.Nv, self.Nu)
        depth2 = np.copy(depth)
        depth = depth[:,int(round(self.cu))]

        #print(repr(depth))

        # Define a best fit function for distance, assuming pitch.
        def fit(pitch):
            # Precompute the sin/cos of pitch
            (sp, cp) = (math.sin(pitch), math.cos(pitch))

            # Run through all pixels
            R  = 0.0
            R2 = 0.0
            N  = 0
            for v in range(self.Nv):
                #  Place the pixel in space (r forward, z up)
                d    = float(depth[v]) * 0.001
                vbar = (float(v) - self.cv) / self.fv
                r    = d * (cp + sp * vbar)
                h    = d * (sp - cp * vbar)

                # Should we use the point to fit?
                if (d > 0.0) and (h < self.hmax) and (h > self.hmin):
                    R  += r
                    R2 += r**2
                    N  += 1

            # Compute the mean and standard deviation.
            if   N < 1:  (r, s) = (0.0, math.inf)
            elif N < 2:  (r, s) = (R,   math.inf)
            else:
                r = R/float(N)
                s = math.sqrt((R2 - float(N)*r**2)/float(N-1))

            # Report to debug and return.
            #print("Test pitch %5.3fdeg: r = %6.3fm std %5.3fm for %d samples" %
            #      (math.degrees(pitch), r, s, N))
            return(pitch, N, r, s)

        # Find the lowest standard deviation fit.
        below = fit(PITCH_START - PITCH_RANGE)
        mid   = fit(PITCH_START)
        above = fit(PITCH_START + PITCH_RANGE)

        # Should we try to re-estimate the pitch angle.
        if False:
            while (above[0]-below[0] > PITCH_RESOLUTION):
                if (above[0]-mid[0] > mid[0]-below[0]):
                    new = fit(0.5*(above[0]+mid[0]))
                    if new[3] > mid[3]:  (below, mid, above) = (below, mid, new)
                    else:                (below, mid, above) = (mid, new, above)
                else:
                    new = fit(0.5*(below[0]+mid[0]))
                    if new[3] > mid[3]:  (below, mid, above) = (new, mid, above)
                    else:                (below, mid, above) = (below, new, mid)

        vbar = np.tan(mid[0])
        v = vbar * self.fv + self.cv
        depth2[int(np.round(v)), :] = 10000

        imagemsg.data = depth2.flatten().tobytes()
        self.pub.publish(imagemsg)

        # Report.
        (pitch, N, r, s) = mid
        print("Pitch %5.3fdeg: r = %6.3fm with std %5.3fm over %d samples" %
              (math.degrees(pitch), r, s, N))


#
#   Main Code
#
if __name__ == "__main__":
    # Initialize the ROS node.
    rospy.init_node('depthtocamcal')

    # Instantiate the depth camera calibration object
    camcal = DepthCamCalibrator()

    # Report and spin (waiting while callbacks do their work).
    rospy.loginfo("Depth Camera Calibration spinning...")
    rospy.spin()
    rospy.loginfo("Depth Camera Calibration stopped.")
