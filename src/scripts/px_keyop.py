#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState

import os, sys
import numpy as np
from py_console import console, bgColor, textColor
from pynput.keyboard import Key, Listener

SELECTED_JOINT = 0
POSITIONS = {
    '1': {
        'waist': 0,
        'shoulder': 0,
        'elbow': 0,
        'wrist': 0,
        'gripper': 0,
    },
    '2': {
        'waist': -20,
        'shoulder': 20,
        'elbow': -20,
        'wrist': 20,
        'gripper': 0,
    },
    '3': {
        'waist': 30,
        'shoulder': -30,
        'elbow': 30,
        'wrist': -30,
        'gripper': 0,
    },
    '4': {
        'waist': -90,
        'shoulder': 15,
        'elbow': -55,
        'wrist': 17,
        'gripper': 0,
    },
    '5': {
        'waist': -90,
        'shoulder': 45,
        'elbow': -55,
        'wrist': 45,
        'gripper': 10,
    },
}
JOINTS = {
    ' waist    ': POSITIONS['1']['waist'],
    ' shoulder ': POSITIONS['1']['shoulder'],
    ' elbow    ': POSITIONS['1']['elbow'],
    ' wrist    ': POSITIONS['1']['wrist'],
}
LIMITS = {
    'low': -90,
    'high': 90,
    'step': 2,
}

def updateScreen(key=None):
    # Restart screen
    os.system('clear')
    # Show start message
    console.log('-'*65)
    console.log(f" Press {console.highlight('UP')} and {console.highlight('DOWN')} arrows to move between joints")
    console.log(f" Press {console.highlight('LEFT')} and {console.highlight('RIGHT')} arrows to change the value of selected joint")
    console.log(f" Press {console.highlight('1 2 3 4 5')} keys to go to predefined positions")
    console.log('-'*65)
    console.warn(f"Pressed key: {key}" if key else "Press a key!")
    console.log('-'*65)
    # Highlight selected joint
    for i in range(len(JOINTS)):
        joint = list(JOINTS.keys())[i]
        bg = bgColor.GREEN if i == SELECTED_JOINT else ''
        txt = textColor.GREEN if i == SELECTED_JOINT else textColor.WHITE
        name = console.highlight(joint, bgColor=bg, textColor=textColor.WHITE)
        value = console.highlight(str(JOINTS[joint]), bgColor='', textColor=txt)
        console.log(f"\t{name}\t{value} deg")
    console.log('-'*65)

def predefinedPositions(key):
    try:
        currentKey = key.char
    except AttributeError:
        currentKey = key
    if currentKey in POSITIONS.keys():
        for i in range(len(JOINTS)):
            joint = list(POSITIONS[currentKey].keys())[i]
            JOINTS[list(JOINTS.keys())[i]] = POSITIONS[currentKey][joint]

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
    # Seek events
    if key == Key.esc: sys.exit()
    predefinedPositions(key)
    updateSelectedJoint(key)
    updateSelectedValue(key)
    # Run events
    updateScreen(key)
    publishMessage()

def keyReleased(key):
    pass

if __name__ == '__main__':
    # Run node
    node_name = 'px_keyop'
    rospy.init_node(node_name)
    rospy.loginfo(f'>> STATUS: Node \"{node_name}\" initialized.')
    rospy.sleep(1)
    # Disable timestamp for logger
    console.setShowTimeDefault(False)
    # Print first screen
    updateScreen()
    # Publish HOME position
    # TODO how to reach home automatically
    # Start listener
    with Listener(on_press=keyPressed, on_release=keyReleased) as listener:
        listener.join()