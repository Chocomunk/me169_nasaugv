#!/usr/bin/env python3
#
#   depthcamcal_utilities.py
#
#   Depth Camera Calibration Utilities.  This grabs the camera
#   intrinsic parameters (camera info) and camera pitch angle.
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
import tf2_ros

from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg   import CameraInfo
from sensor_msgs.msg   import Image


#
#   Constants
#
TIMEOUT = 5.0           # Timeout for the initialization

INFO_TOPIC    = '/camera/depth/camera_info'
IMAGE_TOPIC   = '/camera/depth/image_rect_raw'
OVERLAY_TOPIC = '/camera/depth/image_overlaid'

GROUND_FRAME  = 'base'
IMAGE_FRAME   = 'camera_depth_optical_frame'

DASH_LEN      = 10

MIN_HEIGHT    = 0.100           # Min/Max vertical height relative to camera
MAX_HEIGHT    = 1.500

PITCH_DEFAULT = math.radians(17.0)


######################################################################
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
#   so that the horizon (zl=0) and center of image are located at:
#
#     vbar_horizon =  sp/cp   ->   v_horizon = int(round(cv + fv * sp/cp))
#     vbar_center  =  0       ->   v_center  = int(round(cv))
#     ubar_center  =  0       ->   u_center  = int(round(cu))
#

######################################################################
#
#   Grab the depth camera intrinsic parameters
#
def grabCameraInfo():
    # Report.
    rospy.loginfo("Grabbing the depth camera info...")

    # Grab a single message of the camera_info topic.
    try:
        infomsg = rospy.wait_for_message(INFO_TOPIC, CameraInfo, TIMEOUT)
    except Exception as ex:
        rospy.logerr("Unable to get depth camera info!")
        raise ex

    # Note the width and height.
    Nu = infomsg.width
    Nv = infomsg.height

    # Pull the image resolution and center numbers from the projection
    # matrix.  note this maps an (x,y,z) point to (u=column,v=row) as:
    #    u = cu + fu * (x/z)
    #    v = cv + fv * (y/z)
    (fu, cu) = (infomsg.P[0], infomsg.P[2])
    (fv, cv) = (infomsg.P[5], infomsg.P[6])

    # Determine the field of  view angles.
    FOVu = math.atan(float(Nu)/fu/2.0) * 2.0
    FOVv = math.atan(float(Nv)/fv/2.0) * 2.0

    # Report.
    rospy.loginfo("Pixels: u = %7.3f + %7.3f * (x/z)" % (cu, fu))
    rospy.loginfo("        v = %7.3f + %7.3f * (y/z)" % (cv, fv))
    rospy.loginfo("Image is %dx%d = %.0fx%.0f deg FOV" %
                  (Nu, Nv, math.degrees(FOVu), math.degrees(FOVv)))

    # Return the info.
    return ((Nu, cu, fu), (Nv, cv, fv))


######################################################################
#
#   Grab the Camera Pitch angle (upward)
#
def grabCameraPitch():
    # Report.
    rospy.loginfo("Grabbing the URDF depth camera pitch angle...")

    # Create a TF buffer and listener.
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    # Determine the BASE and IMAGE frames.
    groundframe = rospy.get_param('~ground_frame', GROUND_FRAME)
    rospy.loginfo("Starting from ground in frame '%s'" % (groundframe))
    try:
        imagemsg   = rospy.wait_for_message(IMAGE_TOPIC, Image, TIMEOUT)
        imageframe = imagemsg.header.frame_id
        rospy.loginfo("Image reports in frame '%s'" % (imageframe))
    except Exception as ex:
        imageframe = IMAGE_FRAME
        rospy.loginfo("Defaulting to image frame '%s'" % (imageframe))

    # Try to grab the BASE to IMAGE transform.  Use rospy.Time(0) to
    # grab the latest transform, as opposed to the transform at a
    # particular time.  Use a timeout, in case no transforms appear.
    try:
        # Grab the transform as a TransformStamped message.
        msg = tfBuffer.lookup_transform(groundframe, imageframe, rospy.Time(0),
                                        rospy.Duration(TIMEOUT))
    except Exception as ex:
        rospy.logwarn("Unable to get the '%s' to '%s' transform!"
                      % (groundframe, imageframe))
        rospy.logwarn("(have you started the robot_state_publisher?)")
        rospy.logwarn("Using DEFAULT PITCH ANGLE")
        return (PITCH_DEFAULT)

    # Remove the listener (don't need any more information).
    listener.unregister()

    # Extract the rotation and the vertical components of the X/Y/Z
    # axes, knowing the ground frame's Z axis is vertical.
    r = msg.transform.rotation
    xvertical = 2*(r.x*r.z - r.w*r.y)   # X = Image right
    yvertical = 2*(r.y*r.z + r.w*r.x)   # Y = Image down
    zvertical = 1-2*(r.x**2 + r.y**2)   # Z = Image forward

    # Check the camera (Image-right) being horizontal.
    if (abs(xvertical) > 1e-9):
        errstr = "The depth camera is not mounted horizontally!"
        rospy.logerr(errstr)
        raise rospy.ROSException(errstr)

    # Grab the pitch angle.
    spitch =  zvertical       # Vertical component of Z axis
    cpitch = -yvertical       # Vertical component of -Y axis
    pitch  = math.atan2(spitch, cpitch)

    # Also grab the height (currently unused).
    hcam = msg.transform.translation.z

    # Report.
    rospy.loginfo("The depth camera is located %.2fmm above ground"
                  % (hcam * 1000.0))
    rospy.loginfo("The depth camera is pitched up %6.3fdeg (%4.3frad)"
                  % (math.degrees(pitch), pitch))

    # Return the data.
    return (pitch)
