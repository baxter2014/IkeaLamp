#!/usr/bin/env python

# Copyright (c) 2013-2014, Rethink Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
Baxter RSDK Joint Position Waypoints Example
"""
import argparse
import sys
import rospy
from std_msgs.msg import String
import baxter_interface
import time

rospy.init_node('secondweek')

#grab the left arm
Llimb = baxter_interface.Limb('left')
#set left limb to 'comfortable' position to get started with
Langles = {'left_w0': 2.0793109556030274, 'left_w1': -1.4522963092712404, 'left_w2': -3.0533887547973633, 'left_e0': -0.17832526638793947, 'left_e1': 2.1414371774414063, 'left_s0': 0.22472818516845705, 'left_s1': -0.683388440222168}
Llimb.move_to_joint_positions(Langles)
Lnav = (baxter_interface.Navigator('right'))


#grab the right arm
Rlimb = baxter_interface.Limb('right')
Rangles = {'right_s0': -0.12923788123168947, 'right_s1': -0.34361169609375003, 'right_w0': -1.1830826813049318, 'right_w1': 1.1347622865417482, 'right_w2': 2.5966459757263185, 'right_e0': 1.1581554935302736, 'right_e1': 1.3974564961669922}
Rlimb.move_to_joint_positions(Rangles)
Rnav = (baxter_interface.Navigator('right'))



#this should import the grippers left and right
right_gripper = baxter_interface.Gripper('right')
left_gripper = baxter_interface.Gripper('left')
#force calibration of left gripper 
left_gripper.calibrate()
#initialize the left gripper as closed
left_gripper.close()
Lclosed = True

class Waypoints(object):
    def __init__(self, limb, speed, accuracy):

        self.lastRotate = time.time()
        # Create baxter_interface limb instance
        self._arm = limb
        self._limb = baxter_interface.Limb(self._arm)

        # Parameters which will describe joint position moves
        self._speed = speed
        self._accuracy = accuracy

        # Recorded waypoints
        self._waypoints = list()


        self._suctionPoints = list()
        self._sucking = False
        self._right_gripper = baxter_interface.Gripper('right')

        # Recording state
        self._is_recording = False

        # Verify robot is enabled
        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable()
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()

        # Create Navigator I/O
        self._navigator_io = baxter_interface.Navigator(self._arm)
        self._close_io = baxter_interface.DigitalIO('%s_upper_button' % (self._arm,)) # 'dash' btn
        self._open_io = baxter_interface.DigitalIO('%s_lower_button' % (self._arm,)) # 'circle' btn

    def _record_waypoint(self, value):
        """
        Stores joint position waypoints

        Navigator 'OK/Wheel' button callback
        """
        if value:
            print("Waypoint Recorded")
            print self._limb.joint_angles()
            self._waypoints.append(self._limb.joint_angles())
            self._suctionPoints.append(self._sucking)

    def _toggle_suction(self, value):
        if value and self._sucking:
            self._sucking = False
            self._record_waypoint(True)
            self._right_gripper.open()
            print('hey, I opened the gripper')
        elif value and not self._sucking:
            self._sucking= True
            self._record_waypoint(True)
            self._right_gripper.close()
            print('hey, I closed the gripper')

    def _stop_recording(self, value):
        """
        Sets is_recording to false

        Navigator 'Rethink' button callback
        """
        # On navigator Rethink button press, stop recording
        if value:
            self._is_recording = False

    def record(self):
        """
        Records joint position waypoints upon each Navigator 'OK/Wheel' button
        press.
        """
        rospy.loginfo("Waypoint Recording Started")
        print("Press Navigator 'OK/Wheel' button to record a new joint "
        "joint position waypoint.")
        print("Press Navigator 'Rethink' button when finished recording "
              "waypoints to begin playback")
        self._waypoints = list()
        self._suctionPoints = list()
        # Connect Navigator I/O signals
        # Navigator scroll wheel button press

        #self._navigator_io.button0_changed.connect(self._record_waypoint)
        self._open_io.state_changed.connect(self._record_waypoint)
        self._close_io.state_changed.connect(self._toggle_suction)
        # Navigator Rethink button press
        self._navigator_io.button2_changed.connect(self._stop_recording)

        # Set recording flag
        self._is_recording = True

        # Loop until waypoints are done being recorded ('Rethink' Button Press)
        while self._is_recording and not rospy.is_shutdown():
            rospy.sleep(1.0)

        # We are now done with the navigator I/O signals, disconnecting them
        self._open_io.state_changed.disconnect(self._record_waypoint)
        self._close_io.state_changed.disconnect(self._toggle_suction)
        self._navigator_io.button2_changed.disconnect(self._stop_recording)

    def playback(self):
        """
        Loops playback of recorded joint position waypoints until program is
        exited
        """
        rospy.sleep(1.0)

        rospy.loginfo("Waypoint Playback Started")
        print("  Press Ctrl-C to stop...")

        # Set joint position speed ratio for execution
        self._limb.set_joint_position_speed(self._speed)

        # Loop one time, enumerating so that we can add in the suction waypoints
        for i, waypoint in enumerate(self._waypoints):
            if rospy.is_shutdown():
                break
            self._limb.move_to_joint_positions(waypoint, timeout=20.0)
            if not self._suctionPoints[i]:
                self._right_gripper.open()
            elif self._suctionPoints[i]:
                self._right_gripper.close()

        # Set joint position speed back to default
        self._limb.set_joint_position_speed(0.3)

    def clean_shutdown(self):
        print("\nExiting example...")
        if not self._init_state:
            print("Disabling robot...")
            self._rs.disable()
        return True

def setButtonHandlers(arm):
# inputs
    if arm == 'left':

        _close_io = baxter_interface.DigitalIO('%s_upper_button' % (arm,)) # 'dash' btn
        #_close_io.state_changed.connect(toggleLeftGripper)

        _open_io = baxter_interface.DigitalIO('%s_lower_button' % (arm,)) # 'circle' btn
        #_open_io.state_changed.connect(rotateLeftGripper)
        
        Lnav = (baxter_interface.Navigator('left'))
        #Lnav.button0_changed.connect(scrollWheel)
        #Lnav.wheel_changed.connect(wheel_moved)

    elif arm == 'right':
        pass
        #Rnav = (baxter_interface.Navigator('right'))

        #press 'ok' button to record
        #Rnav.button0_changed.connect(pickFlower)
        #press 'Rethink' button to record everything
        #Rnav.button2_changed.connect(recordNewFlowerPick)

    else:
        print 'uhm something went wrong here'



########################################## Left arm stuff

def rotateLeftGripper(rotationAng = -0.2):
    print("rotating the cuff")
    Langles = Llimb.joint_angles()

    if Langles['left_w2'] >= 3:
        Langles['left_w2'] = -2.9
        Llimb.move_to_joint_positions(Langles)
    elif Langles['left_w2'] <= -3:
        Langles['left_w2'] = 2.9
        Llimb.move_to_joint_positions(Langles)
    else:
        Langles['left_w2'] = Langles['left_w2'] + rotationAng
        Llimb.move_to_joint_positions(Langles)

def wheel_moved(v):
    if v == -1:
        rotateLeftGripper(0.2)
    else:
        rotateLeftGripper()
    print (v)

def toggleLeftGripper(value):
    global Lclosed
    if value and Lclosed:
        print 'I got a gripper change'
        left_gripper.open()
        print 'should be open'
        Lclosed = False
    elif value:
        print 'I got a gripper change'
        left_gripper.close()
        print 'should be closed'
        Lclosed = True

################################### main
def callback(data):
    print data.data
    if data.data == 'toggleLeftGripper':
        toggleLeftGripper(True)
    elif data.data == 'rotateLeftGripper':
        rotateLeftGripper()
    elif data.data == 'rotateLeftGripperR':
        rotateLeftGripper(0.2)
    elif data.data == 'recordNewPickup':
        waypoints.record()
    elif data.data == 'playBackPickup':
        print 'playingback'
        waypoints.playback()
    else:
        print 'whaaaaa mistake!'



def main():

    print 'wo online'

    rospy.Subscriber('Lamp',String, callback, queue_size = 1)

    waypoints = Waypoints('right', 0.4, 1)
    global waypoints
    # Register clean shutdown
    rospy.on_shutdown(waypoints.clean_shutdown)

    # set button handlers and be cool
    setButtonHandlers('left')
    setButtonHandlers('right')
   
    rospy.spin()
    return 0

if __name__ == '__main__':
    main()


