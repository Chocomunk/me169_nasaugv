#!/usr/bin/env python3
#
#   wheelcontrol.py
#
#   This is a skelenton for the implementation of the Wheel-Control node.  Beyond
#   the basic, it should
#     - stops if no commands are received after 0.25sec
#     - filter the commands to avoid spikes (high torque/current loads)
#     - filters the actual velocity
#     - adds feedback to perfectly achieve the velocity commands
#
#   Node:       /wheelcontrol
#   Publish:    /wheel_state            sensor_msgs/JointState
#               /wheel_desired          sensor_msgs/JointState
#   Subscribe:  /wheel_command          sensor_msgs/JointState
#
#   Other Inputs:   Encoder Channels (GPIO)
#   Other Outputs:  Motor Driver Commands (via I2C)
#
import sys
import time
import math
import rospy
import smbus

from gyro import Gyro
from encoder import Encoder
from motor.driver_i2c import Driver

from sensor_msgs.msg import JointState


class WheelControlObj:
    """ 
    Keeps track of drivers and state for a wheelcontrol node.
    
    Must be created AFTER the node is initialized
    """

    # Physical constants
    GR = 2. * math.pi / 720     # 720 counts / full rotation
    plim = 3 * math.pi / 4      # p-error limit
    cumplim = math.pi / 3       # cumulative-p-error limit

    # Filter Constants. Default: 1
    ACTLAM = 1. / 0.05      # T = 0.05s
    DESLAM = 1. / 0.1       # T = 0.1s
    CORLAM = 1. / 0.35      # T = 0.25s

    # PID constants
    P = 50
    I = 50
    PWM_I = 0.5

    def __init__(self, dt, publish_desired, publish_actual,
                enc_chLA=24, enc_chLB=25, enc_chRA=23, enc_chRB=22,
                drv_chL=0, drv_chR=1, drv_revL=1, drv_revR = 1):
        """ Initializes a wheel controller """
        # Inititlize the low level.
        self.i2cbus = smbus.SMBus(1)
        self.encoder = Encoder(enc_chLA, enc_chLB, enc_chRA, enc_chRB)
        self.driver  = Driver(self.i2cbus, drv_chL, drv_chR, drv_revL, drv_revR)
        self.gyro = Gyro(self.i2cbus)

        # Default values for params
        if self.ACTLAM < 0:
            self.ACTLAM = 1. / dt
        if self.DESLAM < 0:
            self.DESLAM = 1. / dt
        self.ACTLAM = max(0, min(self.ACTLAM, 1. / dt))
        self.DESLAM = max(0, min(self.DESLAM, 1. / dt))

        # Store publishers
        self.publish_des = publish_desired
        self.publish_act = publish_actual

        # ---------- Initialize state variables ----------
        self.cmdvel = [0, 0]
        self.cmdtime = rospy.Time.now()

        # Actual state values
        self.lpos = 0
        self.rpos = 0
        self.lvel = 0
        self.rvel = 0
        self.gyro_thetaz = 0

        # Desired state values
        self.deslpos = 0
        self.desrpos = 0
        self.deslvel = 0
        self.desrvel = 0

        # PID error state values
        self.cumlperr = 0
        self.cumrperr = 0

    def shutdown(self):
        """ Clean up the low level. """
        self.driver.shutdown()
        self.encoder.shutdown()
        self.gyro.shutdown()

    def pwm(self, vel):
        """ Map velocity into PWM """
        # (bottom) pwm = 9.6 v - 22.4
        # (top)    pwm = 9.6 v + 22.4
        k = 22.4 * math.copysign(1, vel)
        return 9.6 * vel + k

    def callback_command(self, msg: JointState):
        """
        Command Callback Function

        Save the command and the time received.
        """
        # TODO: Check the message?

        # Note the current time (to timeout the command).
        now = rospy.Time.now()

        # Save...
        self.cmdvel  = msg.velocity
        self.cmdtime = now


    def callback_timer(self, event):
        """ Timer Callback Function """
        # Note the current time to compute dt and populate the ROS messages.
        now = rospy.Time.now()

        # ---------- Process the commands ---------- 
        if (now - self.cmdtime).to_sec() > 0.25:
            lvelcmd, rvelcmd = 0, 0
        else:
            lvelcmd, rvelcmd = self.cmdvel

        # Filter desired velocity
        self.deslvel = self.deslvel + self.DESLAM * dt * (lvelcmd - self.deslvel)
        self.desrvel = self.desrvel + self.DESLAM * dt * (rvelcmd - self.desrvel)

        self.deslpos += self.deslvel * dt
        self.desrpos += self.desrvel * dt

        # ---------- Process actual state ---------- 
        # Process the encoders, convert to wheel angles!
        lastlpos = self.lpos     # Save previous values
        lastrpos = self.rpos

        self.lpos = self.GR * self.encoder.leftencoder()
        self.rpos = self.GR * self.encoder.rightencoder()

        # Compute AND filter actual velocity
        self.lvel = (1 - self.ACTLAM * dt) * self.lvel + self.ACTLAM * (self.lpos - lastlpos)
        self.rvel = (1 - self.ACTLAM * dt) * self.rvel + self.ACTLAM * (self.rpos - lastrpos)

        # Read gyro and integrate heading
        # TODO: Handle saturated
        omega_z, _ = self.gyro.read()
        self.gyro_thetaz += omega_z * dt

        # ---------- Compute PWM ---------- 
        # Add corrective velocity to desired
        corrlvel = self.deslvel
        corrrvel = self.desrvel

        # PID update
        lperr = min(self.plim, max(self.deslpos - self.lpos, -self.plim))
        rperr = min(self.plim, max(self.desrpos - self.rpos, -self.plim))

        self.deslpos = lperr + self.lpos
        self.desrpos = rperr + self.rpos

        self.cumlperr += min(self.cumplim, max(lperr * dt, -self.cumplim))
        self.cumrperr += min(self.cumplim, max(rperr * dt, -self.cumplim))

        # Generate motor commands (convert wheel speed to PWM with PID)
        pwml = self.pwm(corrlvel + self.CORLAM * lperr + self.PWM_I * self.cumlperr)
        pwmr = self.pwm(corrrvel + self.CORLAM * rperr + self.PWM_I * self.cumrperr)

        # Send wheel commands.
        self.driver.left(pwml)
        self.driver.right(-pwmr)  # Might need to be negative

        # ---------- Publish States ---------- 
        # Publish the actual wheel state
        msg = JointState()
        msg.header.stamp = now
        msg.name         = ['leftwheel', 'rightwheel', 'gyro']
        msg.position     = [self.lpos, self.rpos, self.gyro_thetaz]
        msg.velocity     = [self.lvel, self.rvel, omega_z]
        msg.effort       = [0.0, 0.0, 0.0]
        self.publish_act.publish(msg)

        # Publish the desired wheel state
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name         = ['leftwheel', 'rightwheel']
        msg.position     = [self.deslpos, self.desrpos]
        msg.velocity     = [self.deslvel, self.desrvel]
        msg.effort       = [pwml, pwmr]
        self.publish_des.publish(msg)


#
#   Main Code
#
if __name__ == "__main__":
    # Initialize the ROS node.
    rospy.init_node('wheelcontrol')

    # Define durations
    duration = rospy.Duration(1. / 100)       # 100 Hz
    dt       = duration.to_sec()

    # Create a publisher to send the wheel desired and actual (state).
    pubdes = rospy.Publisher('/wheel_desired', JointState, queue_size=10)
    pubact = rospy.Publisher('/wheel_state',   JointState, queue_size=10)

    # Initialize state and drivers
    wheelcontrol = WheelControlObj(dt, pubdes, pubact)

    # Create a subscriber to listen to wheel commands.
    sub = rospy.Subscriber("/wheel_command", JointState, wheelcontrol.callback_command)

    # Create the timer.
    timer = rospy.Timer(duration, wheelcontrol.callback_timer)

    # Spin while the callbacks are doing all the work.
    rospy.loginfo("Running with dt = %.3f sec..." % dt)
    rospy.spin()
    rospy.loginfo("Stopping...")

    # Stop the timer (if not already done).
    timer.shutdown()

    # Shutdown drivers
    wheelcontrol.shutdown()
