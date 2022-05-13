#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState

import os, sys
import numpy as np
from py_console import console, bgColor, textColor
from pynput.keyboard import Key, Listener

SELECTED_JOINT = 0
JOINTS = {
    ' waist    ': 0,
    ' shoulder ': 0,
    ' elbow    ': 0,
    ' wrist    ': 0,
}
LIMITS = {
    'low': -90,
    'high': 90,
    'step': 10,
}

def updateScreen():
    # Restart screen
    os.system('clear')
    # Show start message
    console.log('-'*65)
    console.log(f" Use {console.highlight('UP')} and {console.highlight('DOWN')} arrows to move between joints")
    console.log(f" Use {console.highlight('LEFT')} and {console.highlight('RIGHT')} arrows to change the value of selected joint")
    console.log('-'*65)
    # Highlight selected joint
    for i in range(len(JOINTS)):
        joint = list(JOINTS.keys())[i]
        bg = bgColor.GREEN if i == SELECTED_JOINT else ''
        txt = textColor.GREEN if i == SELECTED_JOINT else textColor.WHITE
        name = console.highlight(joint, bgColor=bg, textColor=textColor.WHITE)
        value = console.highlight(str(JOINTS[joint]), bgColor='', textColor=txt)
        console.log(f"\t{name}\t{value}")
    console.log('-'*65)

def updateSelectedJoint(key):
    global SELECTED_JOINT
    if key == Key.down: SELECTED_JOINT = SELECTED_JOINT + 1 if SELECTED_JOINT < len(JOINTS) - 1 else 0
    if key == Key.up: SELECTED_JOINT = SELECTED_JOINT - 1 if SELECTED_JOINT > 0 else len(JOINTS) - 1  

def updateSelectedValue(key):
    global SELECTED_JOINT
    joint = list(JOINTS.keys())[SELECTED_JOINT]
    if key == Key.right: JOINTS[joint] = JOINTS[joint] + LIMITS['step'] if JOINTS[joint] < LIMITS['high'] else LIMITS['high']
    if key == Key.left: JOINTS[joint] = JOINTS[joint] - LIMITS['step'] if JOINTS[joint] > LIMITS['low'] else LIMITS['low']

def publishMessage():
    # Define and fill message
    state = JointState()
    state.header.stamp = rospy.Time.now()
    state.name = list(map(lambda s: s.replace(' ', ''), JOINTS.keys()))
    state.position = list(map(lambda p: p*np.pi/180, JOINTS.values()))
    # Create publisher and publish message
    pub = rospy.Publisher('/joint_states', JointState, queue_size=0)
    pub.publish(state)

def keyPressed(key):
    if key == Key.esc: sys.exit()
    updateSelectedJoint(key)
    updateSelectedValue(key)
    updateScreen()
    publishMessage()

def keyReleased(key):
    pass

if __name__ == '__main__':
    rospy.init_node('px_keyop')
    console.setShowTimeDefault(False)
    updateScreen()
    with Listener(on_press=keyPressed, on_release=keyReleased) as listener:
        listener.join()