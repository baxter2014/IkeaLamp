#!/usr/bin/env python

import cv2
import rospy
import baxter_interface
import cv
import cv_bridge
from sensor_msgs.msg import (
    Image,
)
from std_msgs.msg import String
from BaxGUI import Screen
import time
class Manager:

    def __init__(self, Baxter = True):
        self.Baxter = Baxter

        self.last = time.time()

        if Baxter:
            print 'hello '
            rospy.init_node('BaxGUI')

            self.pubLamp = rospy.Publisher('Lamp', String)
            self.pubImage = rospy.Publisher('/robot/xdisplay', Image, latch=True)

            self.Head = baxter_interface.Head()
            self.headLocation = 0
            self.Head.set_pan(self.headLocation)

            self.Lnav = (baxter_interface.Navigator('left'))
            self.Lnav.button0_changed.connect(self.scrollPush)
            self.Lnav.wheel_changed.connect(self.scrolled)
            self.Lnav.button1_changed.connect(self.backPush)
            self.Lnav.button2_changed.connect(self.rethinkPush)

            self.Rnav = (baxter_interface.Navigator('right'))
            self.Rnav.button0_changed.connect(self.scrollPush)
            #self.Rnav.wheel_changed.connect()
            self.Rnav.button1_changed.connect(self.backPush)
            self.Rnav.button2_changed.connect(self.rethinkPush)

            self._close_io = baxter_interface.DigitalIO('%s_upper_button' % ('right',))
            self._close_io.state_changed.connect(self.dashRightPush)

            self.temp = baxter_interface.DigitalIO('%s_upper_button' % ('left',))
            self.temp.state_changed.connect(self.dashLeftPush)


        self.currentScreen = 'page2'
        self.screenList = {}

        home = Screen('home','/home/argun/BaxGUI Images/', 'bg.png')
        home.addButton(10, 100, '', 'homeButton', ['screen', 'home'], 49, None)
        home.addButton(10, 100, '', 'scrollUp', ['scrollup', 'options'], 2490368, None)
        home.addButton(10, 150, '', 'scrollDown', ['scrolldown', 'options'], 2621440, None)
        home.addButton(10, 200, '', 'select', ['select', 'options'], 13, None)
        home.addOptionList(100, 100, 100, 30, 'options', ['lamp 1',
                         'lamp 2',
                         'lamp 3',
                         'lamp 4'],
           [['screen', 'page1'],
            ['screen', 'page2'],
            ['screen', 'page3'],
            ['screen', 'page4']],
           2, 13,
           'imagewindow',
           ['1.png',
            '2.png',
            '3.png',
            '4.png'])
        home.addImage(500, 500, 'home.png', 'imagewindow')
        self.addScreen(home)

        page1 = Screen('page1','/home/argun/BaxGUI Images/', 'bg.png')
        page1.addButton(10, 100, '', 'testButton', ['screen', 'home'], 49, None)
        page1.addButton(500, 10, 'rotate head: wheel', 'headRotateLeft', ['head', 'left'], 2490368, imagePath = None)
        page1.addButton(500, 10, 'rotate head: wheel', 'headRotateRight', ['head', 'right'], 2621440, imagePath = None)
        self.addScreen(page1)

        page2 = Screen('page2','/home/argun/BaxGUI Images/', 'bg.png')
        page2.addButton(10, 100, '', 'testButton', ['screen', 'home'], 49, None)
        page2.addButton(10, 100, '', 'rotateGripper', ['baxter', 'rotateLeftGripper'], 2490368, None)
        page2.addButton(10, 100, '', 'rotateGripperR', ['baxter', 'rotateLeftGripperR'], 2621440, None)
        page2.addButton(10, 100, '', 'recordNewPickup', ['baxter', 'recordNewPickup'], 13, None)
        page2.addButton(10, 100, '', 'playBackPickup', ['baxter', 'playBackPickup'], 1113864, None)
        page2.addButton(10, 100, '', 'toggleGripper', ['baxter', 'toggleLeftGripper'], 2, None)
        self.addScreen(page2)

        page3 = Screen('page3','/home/argun/BaxGUI Images/', 'bg.png')
        page3.addButton(10, 100, '', 'testButton', ['screen', 'home'], 49, None)
        self.addScreen(page3)

        page4 = Screen('page4','/home/argun/BaxGUI Images/', 'bg.png')
        page4.addButton(10, 100, '', 'testButton', ['screen', 'home'], 49, None)
        self.addScreen(page4)

    def addScreen(self, newScreen):
        self.screenList[newScreen.getName()] = newScreen
        print 'added new screen: ' + newScreen.getName()

    def send_image(self, image):
        msg = cv_bridge.CvBridge().cv_to_imgmsg(image, encoding="bgr8")
        self.pubImage.publish(msg)

    def scrollPush(self, value):
        print 'scrollPush' + str(value)
        if value:
            self.onKey(13)

    def scrolled(self, value):
        print 'scrolled: ' + str(value)
        if value == -1:
            self.onKey(2490368)
        else:
            self.onKey(2621440)

    def backPush(self, value):
        print 'backPush'
        if value:
            self.onKey(49)

    def rethinkPush(self, value):
        print 'rethinkPush'
        if value:
            self.onKey(1113864)

    def dashRightPush(self, value):
        print 'dashRightPush'
        if value:
            self.onKey(1)

    def dashLeftPush(self, value):
        print 'dashLeftPush'
        if value:
            self.onKey(2)

    def onKey(self, key):
        nextAction = self.screenList[self.currentScreen].onKeyPress(key)
        self.processAction(nextAction)
        print 'nextAction: ' + str(nextAction)

    def processAction(self, action):
        try:
            if action[0] == 'screen':
                self.currentScreen = action[1]
                print self.currentScreen
            elif action[0] == 'scrollup':
                self.screenList[self.currentScreen].objectList[action[1]].selectUp()
                path = self.screenList[self.currentScreen].objectList[action[1]].getImagePath()
                print path
                window = self.screenList[self.currentScreen].objectList[action[1]].getWindow()
                self.screenList[self.currentScreen].objectList[window].changeImage(path)
            elif action[0] == 'scrolldown':
                self.screenList[self.currentScreen].objectList[action[1]].selectDown()
                path = self.screenList[self.currentScreen].objectList[action[1]].getImagePath()
                window = self.screenList[self.currentScreen].objectList[action[1]].getWindow()
                self.screenList[self.currentScreen].objectList[window].changeImage(path)
            elif action[0] == 'select':
                self.currentScreen = self.screenList[self.currentScreen].objectList[action[1]].getAction()[1]
            elif action[0] == 'head' and self.Baxter:
                if action[1] ==  'left':
                    self.headLocation += 0.01
                    if self.headLocation > 1.6:
                        self.headLocation = 1.6
                else:
                    self.headLocation -= 0.01
                    if self.headLocation < -1.6:
                        self.headLocation = -1.6
                self.Head.set_pan(self.headLocation)
            elif action[0] == 'baxter' and self.Baxter and False:
                if action[1] == 'rotateLeftGripper' and time.time() - self.last < 1:
                    pass
                elif action[1] == 'rotateLeftGripperR' and time.time() - self.last  < 1:
                    pass
                else:
                    self.last = time.time()
                    self.pubLamp.publish(action[1])
            elif action[0] == 'baxter' and self.Baxter:
                self.pubLamp.publish(action[1])
        except:
            pass
        #print('failed action: ' + str(nextAction))
        image = self.screenList[self.currentScreen].render()
        self.render(image)

    def render(self, image):
        if self.Baxter:
            baxImage = cv.fromarray(image)
            self.send_image(baxImage)
        cv2.imshow('BaxGUI', image)
