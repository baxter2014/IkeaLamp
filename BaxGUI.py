#!/usr/bin/env python

import cv2
from argparse import Action

global font
font = cv2.FONT_HERSHEY_SIMPLEX

def readImage(imagePath):
    try:
        return cv2.imread(imagePath)
    except:
        return cv2.imread('/home/argun/BaxGUI Images/' + 'ImageNotFound.png')

def inlayImage(x, y, background, inlay):
    dx = inlay.shape[0]
    dy = inlay.shape[1]
    x1 = max(x, 0)
    y1 = max(y, 0)
    x2 = min(x + dx, background.shape[0])
    y2 = min(y + dy, background.shape[1])

    background[x1:x2, y1:y2, :] = inlay


class Screen:

    def __init__(self, name, root, bgImagePath = 'bg.png', defaultFont = font):
        self.imageRoot = root
        self.font = defaultFont
        self.name = name
        self.root = root
        self.bgChange(bgImagePath)
        self.objectList = {}
        #binds a key to an action
        self.actionList= {}

    def getName(self):
        return self.name

    def bgChange(self, imagePath):
        self.bgImage = readImage(self.root + imagePath)

    def addButton(self, x, y, text, name, action, key, imagePath = None, buttonFont = font, fontScale = 1, color = (0, 0, 0)):
        if imagePath == None:
            temp = None
        else:
            temp = self.imageRoot + imagePath
        self.objectList[name] = Button(x, y, text, name, action, temp, buttonFont, fontScale, color)
        self.actionList[key] = action

    def addOptionList(self, x, y, dx, dy, name, textList, actionList, showNumberButtons, key, window, imagePaths, fontScale = 1, textColor = (0,0,0), boxColor = (255,168,0), selectColor = (255,0,0), listFont = font):
        for i in range(0, len(imagePaths)):
            imagePaths[i] = self.root + imagePaths[i]
        self.objectList[name] = OptionList(x, y, dx, dy, name, textList, actionList, showNumberButtons, key, window, imagePaths, fontScale, textColor, boxColor, selectColor, listFont)

    def addImage(self, x, y, imagePath, name):
        self.objectList[name] = ImageWindow(x, y, self.imageRoot + imagePath, name)

    def onKeyPress(self, key):
        if self.actionList.has_key(key):
            return self.actionList[key]

    def onAction(self, action):
        pass

    def render(self):
        image = self.bgImage.copy()
        for name in self.objectList:
            self.objectList[name].render(image)
        cv2.putText(image, self.getName(), (30,30), self.font, 1, (100,100,100))
        return image

class Button:

    def __init__(self, x, y, text, name, action, imagePath, buttonFont, fontScale, color):
        self.x = x
        self.y = y
        self.text = text
        self.name = name
        self.action = action
        self.buttonFont = buttonFont
        self.fontScale = fontScale
        self.color = color

        self.setImage(imagePath)

    def setText(self, text):
        self.text = text

    def getText(self):
        return self.text

    def getName(self):
        return self.name

    def getAction(self):
        return self.action

    def setAction(self, action):
        self.action = action

    def setColor(self, color):
        self.color = color

    def setFont(self, font):
        self.buttonFont = font

    def setImage(self, imagePath):
        if not imagePath == None:
            self.image = readImage(imagePath)
            self.hasImage = True
        else:
            self.hasImage = False

    def render(self, image):
        if self.hasImage:
            inlayImage(self.x, self.y, image, self.image)
        cv2.putText(image, self.text, (self.x, self.y), self.buttonFont, self.fontScale, self.color)

class OptionList:

    def __init__(self, x, y, dx, dy, name, textList, actionList, showNumberButtons, key, window, imagePaths, fontScale = 1, textColor = (0,0,0), boxColor = (255,168,0), selectColor = (255,0,0), listFont = font):

        self.selected = 0
        self.x = x
        self.y = y
        self.dx = dx
        self.dy = dy
        self.name = name
        self.textList = textList
        self.actionList = actionList
        self.actionCount = len(actionList)
        self.showNumberButtons = showNumberButtons
        self.key = key
        self.window = window
        self.imagePaths = imagePaths
        self.fontScale = fontScale
        self.textColor = textColor
        self.boxColor = boxColor
        self.selectColor = selectColor
        self.listFont = listFont

    def selectDown(self):
        self.selected = min(self.selected + 1, self.actionCount - 1)

    def selectUp(self):
        self.selected = max(self.selected - 1, 0)

    def getAction(self, index = None):
        if index == None:
            index = self.selected
        return self.actionList[index]

    def getImagePath(self, index = None):
        if index == None:
            index = self.selected
        return self.imagePaths[index]

    def getWindow(self):
        return self.window

    def render(self, image):
        count = 0
        color = self.boxColor

        for i in range(self.selected - self.showNumberButtons, self.selected + self.showNumberButtons + 1):
            if i > -1 and i < self.actionCount:
                x = self.x
                y = self.y + count*(self.dy + 10)

                if count == self.showNumberButtons:
                    color = self.selectColor

                cv2.rectangle(image, (x, y), (x + self.dx, y + self.dy), color)
                cv2.putText(image, self.textList[i], (x, y + self.dy), self.listFont, self.fontScale, self.textColor)

                color = self.boxColor
            count += 1

class ImageWindow:

    def __init__(self, x, y, imagePath, name):
        self.x = x
        self.y = y
        self.changeImage(imagePath)
        self.name = name

    def changeImage(self, imagePath):
        self.image = readImage(imagePath)

    def render(self, image):
        inlayImage(self.x, self.y, image, self.image)
