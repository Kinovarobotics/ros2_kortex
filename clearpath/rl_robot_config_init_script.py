#! /usr/bin/env python3

###
# KINOVA (R) KORTEX (TM)
#
# Copyright (c) 2018 Kinova inc. All rights reserved.
#
# This software may be modified and distributed
# under the terms of the BSD 3-Clause license.
#
# Refer to the LICENSE file for details.
#
###

import sys
import os
import time
import threading

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
from kortex_api.autogen.messages import Base_pb2, BaseCyclic_pb2, Common_pb2
from kortex_api.autogen.client_stubs.DeviceConfigClientRpc import DeviceConfigClient
from kortex_api.autogen.client_stubs.DeviceManagerClientRpc import DeviceManagerClient
# Maximum allowed waiting time during actions (in seconds)
TIMEOUT_DURATION = 20

def disable_joint_safety(device_config, device_manager):
    safety_handle = Common_pb2.SafetyHandle()
    '''
    safety_handle.identifier = 4 # Disabling joint high limit
    safety_enable = device_config.GetSafetyEnable(safety_handle)
    safety_enable.enable = False
    # Extraction of the joints IDs
    all_devices_info = device_manager.ReadAllDevices()
    for dev in all_devices_info.device_handle:
        if dev.device_type == 3 or dev.device_type == 4: #If the device is big or medium actuator (the only actuator types used in Gen3)
            device_config.SetSafetyEnable(safety_enable,dev.device_identifier)
            print("Joint high limit safety activation: ", device_config.GetSafetyEnable(safety_handle, dev.device_identifier).enable)
    '''
    safety_handle.identifier = 8 # Disabling joint low limit
    safety_enable = device_config.GetSafetyEnable(safety_handle)
    safety_enable.enable = False
    # Extraction of the joints IDs
    all_devices_info = device_manager.ReadAllDevices()
    for dev in all_devices_info.device_handle:
        if dev.device_type == 3 or dev.device_type == 4: #If the device is big or medium actuator (the only actuator types used in Gen3)
            device_config.SetSafetyEnable(safety_enable,dev.device_identifier)
            print("Joint low limit safety activation: ", device_config.GetSafetyEnable(safety_handle, dev.device_identifier).enable)
    return True

# Create closure to set an event after an END or an ABORT
def check_for_end_or_abort(e):
    """Return a closure checking for END or ABORT notifications

    Arguments:
    e -- event to signal when the action is completed
        (will be set when an END or ABORT occurs)
    """
    def check(notification, e = e):
        print("EVENT : " + \
              Base_pb2.ActionEvent.Name(notification.action_event))
        if notification.action_event == Base_pb2.ACTION_END \
        or notification.action_event == Base_pb2.ACTION_ABORT:
            e.set()
    return check
 
def example_move_to_initial_position(base):
    # Make sure the arm is in Single Level Servoing mode
    base_servo_mode = Base_pb2.ServoingModeInformation()
    base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
    base.SetServoingMode(base_servo_mode)
    
    # Move arm to ready position
    print("Moving the arm to a safe position")
    action_type = Base_pb2.RequestedActionType()
    action_type.action_type = Base_pb2.REACH_JOINT_ANGLES
    action_list = base.ReadAllActions(action_type)
    action_handle = None
    for action in action_list.action_list:
        if action.name == "Gen3_Husky_Initial_Position":
            action_handle = action.handle

    if action_handle == None:
        print("Can't reach safe position. Exiting")
        return False

    e = threading.Event()
    notification_handle = base.OnNotificationActionTopic(
        check_for_end_or_abort(e),
        Base_pb2.NotificationOptions()
    )

    base.ExecuteActionFromReference(action_handle)
    finished = e.wait(TIMEOUT_DURATION)
    base.Unsubscribe(notification_handle)

    if finished:
        print("Safe position reached")
    else:
        print("Timeout on action notification wait")
    return finished

def main():
    
    # Import the utilities helper module
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
    import utilities

    # Parse arguments
    args = utilities.parseConnectionArguments()
    
    # Create connection to the device and get the router
    with utilities.DeviceConnection.createTcpConnection(args) as router:

        # Create required services
        base = BaseClient(router)
        base_cyclic = BaseCyclicClient(router)
        device_conf = DeviceConfigClient(router)
        device_manager = DeviceManagerClient(router)

        # Example core
        success = False

        success &= example_move_to_initial_position(base)
        success &= disable_joint_safety(device_conf, device_manager)


        return 0 if success else 1

if __name__ == "__main__":
    exit(main())
