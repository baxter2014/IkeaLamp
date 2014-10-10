#!/usr/bin/env python

from manager import Manager
import cv2
import time

man = Manager(True)

key = 0

key = cv2.waitKey()
print key
man.onKey(key)


while not key == 27:
    key = cv2.waitKey()
    #escape
    if key == 1048603:
        key = 27
    #up
    elif key == 1113938:
        key = 2490368
    #down
    elif key == 1113940:
        key = 2621440
    #enter
    elif key == 1048586:
        key = 13
    #1
    elif key == 1048625:
        key = 49
    print key
    start = time.time()
    man.onKey(key)
    print time.time() - start
