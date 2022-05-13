#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from dynamixel_workbench_msgs.srv import DynamixelCommand

import numpy as np

LIMIT = 150*np.pi/180

# Start at home
lastPos = (None, None, None, None)

def jointCommand(command, id_num, addr_name, value, time=0):
    rospy.wait_for_service('dynamixel_workbench/dynamixel_command')
    try:        
        dynamixel_command = rospy.ServiceProxy(
            '/dynamixel_workbench/dynamixel_command', DynamixelCommand)
        result = dynamixel_command(command,id_num,addr_name,value)
        rospy.sleep(time)
        return result.comm_result
    except rospy.ServiceException as exc:
        print(str(exc))

def callback(data):
    rospy.loginfo(data.position)
    rawPosition = list(map(lambda p: int(511*((p/LIMIT) + 1)), data.position))

    print(rawPosition)

    #  Parse JointState (rad) to DynamixelCommand
    for i in range(len(lastPos)):
        if lastPos[i] != rawPosition[i]:
            jointCommand('', i+1, 'Goal_Position', rawPosition[i])
    
def listener():
    # Run node
    node_name = 'joint_states_translator'
    rospy.init_node(node_name)
    rospy.loginfo(f'>> STATUS: Node \"{node_name}\" initialized.')
    # Subscribe to topic
    rospy.Subscriber("/keyop_joint_states", JointState, callback)
    # Set torque limits
    jointCommand('', 1, 'Torque_Limit', 600, 0)
    jointCommand('', 2, 'Torque_Limit', 500, 0)
    jointCommand('', 3, 'Torque_Limit', 400, 0)
    jointCommand('', 4, 'Torque_Limit', 400, 0)
    # Hold rosnode
    rospy.spin()

if __name__ == '__main__':
    try:
        while not rospy.is_shutdown():
            listener()
    except rospy.ROSInterruptException:
        pass