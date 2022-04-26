#!/usr/bin/env python3
#
#   odometry.py
#
#   Odometry node.  This
#   (a) converts both a body velocity command to wheel velocity commands.
#   (b) estimates the body velocity and pose from the wheel motions
#       and the gyroscope.
#
#   Node:       /odometry
#   Publish:    /odom                   geometry_msgs/TransJointState
#               TF odom -> base         geometry_msgs/TransformStamped
#               /wheel_command          sensor_msgs/JointState
#   Subscribe:  /vel_cmd                geometry_msgs/Twist
#               /wheel_state            sensor_msgs/JointState
#
import math
import rospy
import tf2_ros

from geometry_msgs.msg import Point, Quaternion, Twist
from geometry_msgs.msg import TransformStamped, Vector3
from nav_msgs.msg      import Odometry
from sensor_msgs.msg   import JointState


#
#   Constants
#
R = .03257   # Wheel radius (m)
d = .062     # Halfwidth between wheels (m)


#
#   Odometry Object
#
class OdometryObj:
    # Initialize.
    def __init__(self):
        # Set the initial pose to zero.
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Store the last position
        self.lastT = rospy.Time.now()
        self.lastlpsi = 0.0
        self.lastrpsi = 0.0
        self.lasttheta = 0.0

        # Create a publisher to send wheel commands.
        self.pub_wcmd = rospy.Publisher('/wheel_command', JointState,
                                        queue_size=3)

        # Create a publisher to send odometry information.
        self.pub_odom = rospy.Publisher('/odom', Odometry, queue_size=10)

        # Create a TF2 transform broadcaster.
        self.brd_tf = tf2_ros.TransformBroadcaster()

        # Create a subscriber to listen to twist commands.
        rospy.Subscriber('/vel_cmd', Twist, self.cb_vel_cmd)

        # Create a subscriber to listen to wheel state.
        rospy.Subscriber('/wheel_state', JointState, self.cb_wheel_state)


    # Velocity Command Message Callback
    def cb_vel_cmd(self, msg: Twist):
        # Grab the forward and spin (velocity) commands.
        v = msg.linear.x
        w = msg.angular.z

        # CONVERT THE BODY VELOCITY COMMANDS TO L/R WHEEL COMMANDS
        lpsi_dot = (v-d*w)/R
        rpsi_dot = (v+d*w)/R

        # Create the wheel command msg and publish.  Note the incoming
        # message does not have a time stamp, so generate one here.
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name         = ['leftwheel', 'rightwheel']
        msg.velocity     = [lpsi_dot,    rpsi_dot    ]
        self.pub_wcmd.publish(msg)


    # Wheel State Message Callback
    def cb_wheel_state(self, msg):
        # Grab the timestamp, wheel and gyro position/velocities.
        
        timestamp = msg.header.stamp
        
        li = msg.name.index('leftwheel')  # Left index
        ri = msg.name.index('rightwheel') # Right index
        gi = msg.name.index('gyro')       # Gyro index
        
        lpsi = msg.position[li]
        rpsi = msg.position[ri]
        gtheta = msg.position[gi]
        
        lpsidot = msg.velocity[li]
        rpsidot = msg.velocity[ri]
        gthetadot = msg.velocity[gi]

        # Calculate change in position and velocity
        dtheta = gtheta - self.lasttheta
        hdtheta = dtheta / 2.

        dlpsi = lpsi - self.lastlpsi
        drpsi = rpsi - self.lastrpsi

        # Use gyro to correct wheel-slipping on the encoders
        #   Slipping wheel generally faster, correct on the slow one
        if abs(lpsidot) < abs(rpsidot):
            rpsidot = gthetadot * 2 * d / R + lpsidot
            drpsi = dtheta * 2 * d / R + dlpsi
        else:
            lpsidot = -gthetadot * 2 * d / R + rpsidot
            dlpsi = -dtheta * 2 * d / R + drpsi
        
        dp = (R/2.)*(dlpsi + drpsi)
        vx = (R/2.)*(lpsidot + rpsidot)
        wz = gthetadot
        
        # Update previous values with actual encoder reading (we care more about
        # difference in reading than the true reading value, so don't use the 
        # corrected dlpsi/drpsi)
        self.lastlpsi = lpsi
        self.lastrpsi = rpsi
        self.lasttheta = gtheta
        self.lastT = timestamp

        # Update the pose.
        # TODO: If pose varies wildy, try fixing hdtheta to 0 when close to 0
        subtle = 1 if hdtheta == 0 else math.sin(hdtheta)/hdtheta
        
        self.x    += dp * math.cos(self.theta + hdtheta)*subtle
        self.y    += dp * math.sin(self.theta + hdtheta)*subtle
        self.theta += dtheta

        # Convert to a ROS Point, Quaternion, Twist (lin&ang veloocity).
        p = Point(self.x, self.y, 0.0)
        q = Quaternion(0.0, 0.0, math.sin(self.theta/2), math.cos(self.theta/2))
        t = Twist(Vector3(vx, 0.0, 0.0), Vector3(0.0, 0.0, wz))

        # Create the odometry msg and publish (reuse the time stamp).
        msg = Odometry()
        msg.header.stamp            = timestamp
        msg.header.frame_id         = 'odom'
        msg.child_frame_id          = 'base'
        msg.pose.pose.position      = p
        msg.pose.pose.orientation   = q
        msg.twist.twist             = t
        self.pub_odom.publish(msg)

        # Create the transform msg and broadcast (reuse the time stamp).
        msg = TransformStamped()
        msg.header.stamp            = timestamp
        msg.header.frame_id         = 'odom'
        msg.child_frame_id          = 'base'
        msg.transform.translation   = p
        msg.transform.rotation      = q
        self.brd_tf.sendTransform(msg)


#
#   Main Code
#
if __name__ == "__main__":
    # Initialize the ROS node.
    rospy.init_node('odometry')

    # Instantiate the Odometry object
    odometry = OdometryObj()

    # Report and spin (waiting while callbacks do their work).
    rospy.loginfo("Odometry spinning...")
    rospy.spin()
    rospy.loginfo("Odometry stopped.")
