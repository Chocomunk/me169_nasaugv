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
import math
import sys
import time
import rospy

from encoder import Encoder
from driver import Driver

from sensor_msgs.msg import JointState


# ----- Constants -----
GR = 2. * math.pi / 720     # 720 counts / full rotation
DT = 1. / 100       # 100 Hz
ACTLAM = 1. / 0.05  # T = 0.05s,     Default: -1
DESLAM = 1. / 0.1   # T = 0.1s
CORLAM = 1. / 0.35   # T = 0.25s

#    PID constants
P = 50
I = 50
PWM_I = 0.5

# Wind-up limits
plim = 3 * math.pi / 4
cumplim = math.pi / 3

#   Encoder channels
chLA = 24
chLB = 25
chRA = 23
chRB = 22

#    Driver constants
chL = 0
chR = 1
reserveL = 1
reverseR = 1


# --------- Initialize global vars ----------
cmdvel = [0, 0]
cmdtime = 0

# Actual state values
lpos = 0
rpos = 0
lvel = 0
rvel = 0

# Desired state values
deslpos = 0
desrpos = 0
deslvel = 0
desrvel = 0

# PID error state values
cumlperr = 0
cumrperr = 0


# (bottom) pwm = 9.6 v - 22.4
# (top)    pwm = 9.6 v + 22.4
def pwm(vel):
    k = 22.4 * math.copysign(1, vel)
    return 9.6 * vel + k


#
#   Command Callback Function
#
#   Save the command and the time received.
#
def callback_command(msg):
    global cmdvel, cmdtime

    # TODO: Check the message?

    # Note the current time (to timeout the command).
    now = rospy.Time.now()

    # Save...
    cmdvel  = msg.velocity
    cmdtime = now


#
#   Timer Callback Function
#
def callback_timer(event):
    global lpos, rpos, lvel, rvel
    global deslpos, desrpos, deslvel, desrvel
    global lverr, rverr, cumlperr, cumrperr

    # Note the current time to compute dt and populate the ROS messages.
    now = rospy.Time.now()

    # Process the commands.
    if (now - cmdtime).to_sec() > 0.25:
        lvelcmd, rvelcmd = 0, 0
    else:
        lvelcmd, rvelcmd = cmdvel

    deslvel = deslvel + DESLAM * dt * (lvelcmd - deslvel)            # filtering
    desrvel = desrvel + DESLAM * dt * (rvelcmd - desrvel)

    deslpos += deslvel * dt
    desrpos += desrvel * dt

    # Process the encoders, convert to wheel angles!
    lastlpos = lpos     # Save previous values
    lastrpos = rpos

    lpos = GR * encoder.leftencoder()
    rpos = GR * encoder.rightencoder()

    lvel = (1 - ACTLAM * dt) * lvel + ACTLAM * (lpos - lastlpos)      # filtering
    rvel = (1 - ACTLAM * dt) * rvel + ACTLAM * (rpos - lastrpos)

    # Add corrective velocity to desired
    corrlvel = deslvel
    corrrvel = desrvel

    # PID update
    lperr = min(plim, max(deslpos - lpos, -plim))
    rperr = min(plim, max(desrpos - rpos, -plim))

    deslpos = lperr + lpos
    desrpos = rperr + rpos

    cumlperr += min(cumplim, max(lperr * dt, -cumplim))
    cumrperr += min(cumplim, max(rperr * dt, -cumplim))

    # Generate motor commands (convert wheel speed to PWM with PID)
    pwml = pwm(corrlvel + CORLAM * lperr + PWM_I * cumlperr)
    pwmr = pwm(corrrvel + CORLAM * rperr + PWM_I * cumrperr)

    # Send wheel commands.
    driver.left(pwml)
    driver.right(-pwmr)  # Might need to be negative

    # Publish the actual wheel state
    msg = JointState()
    msg.header.stamp = now
    msg.name         = ['leftwheel', 'rightwheel']
    msg.position     = [lpos, rpos]
    msg.velocity     = [lvel, rvel]
    msg.effort       = [0.0, 0.0]
    pubact.publish(msg)

    # Publish the desired wheel state
    msg = JointState()
    msg.header.stamp = rospy.Time.now()
    msg.name         = ['leftwheel', 'rightwheel']
    msg.position     = [deslpos, desrpos]
    msg.velocity     = [deslvel, desrvel]
    msg.effort       = [pwml, pwmr]
    pubdes.publish(msg)


#
#   Main Code
#
if __name__ == "__main__":
    # Initialize the ROS node.
    rospy.init_node('wheelcontrol')
    cmdtime = rospy.Time.now()

    # Inititlize the low level.
    encoder = Encoder(chLA, chLB, chRA, chRB)
    driver  = Driver(chL, chR, reserveL, reverseR)

    # Create a publisher to send the wheel desired and actual (state).
    pubdes = rospy.Publisher('/wheel_desired', JointState, queue_size=10)
    pubact = rospy.Publisher('/wheel_state',   JointState, queue_size=10)

    # Create a subscriber to listen to wheel commands.
    sub = rospy.Subscriber("/wheel_command", JointState, callback_command)

    # Create the timer.
    duration = rospy.Duration(DT);
    dt       = duration.to_sec()
    timer    = rospy.Timer(duration, callback_timer)

    # Default values for params
    if ACTLAM < 0:
        ACTLAM = 1. / dt
    if DESLAM < 0:
        DESLAM = 1. / dt
    ACTLAM = max(0, min(ACTLAM, 1. / dt))
    DESLAM = max(0, min(DESLAM, 1. / dt))

    print(ACTLAM)
    print(DESLAM)

    # Spin while the callbacks are doing all the work.
    rospy.loginfo("Running with dt = %.3f sec..." % dt)
    rospy.spin()
    rospy.loginfo("Stopping...")

    # Stop the timer (if not already done).
    timer.shutdown()

    # Clean up the low level.
    driver.shutdown()
    encoder.shutdown()
