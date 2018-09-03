#!/usr/bin/env python

"""
    relax_all_servos.py - Version 0.1 2012-03-24
    
    Relax all servos by disabling the torque and setting the speed
    and torque limit to a moderate values.
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2012 Patrick Goebel.  All rights reserved.
    Modified by Tekin Mericli

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
"""
import roslib
roslib.load_manifest('mobi_head')
import rospy, time
from dynamixel_controllers.srv import TorqueEnable, SetTorqueLimit, SetSpeed

class Relax():
    def __init__(self):
        print '++++++++++++++++ RELAX +++++++++++++++'

        rospy.init_node('relax_all_servos')

        namespace = rospy.get_namespace()

        namespace = 'dynamixel_controller'

        # The joints parameter needs to be set by the servo controller        
        self.joints = rospy.get_param(namespace + '/dynamixels', '')
        #self.joints = rospy.get_param(namespace, '')
            
        print 'RELAX_SERVOS: namespace: ', namespace, ' self.joints: ', self.joints
            
        default_dynamixel_speed = rospy.get_param('~default_dynamixel_speed', 0.5)
        default_dynamixel_torque = rospy.get_param('~default_dynamixel_torque', 0.0)

        speed_services = list()   
        torque_services = list()
        set_torque_limit_services = list()

        print 'RELAX_SERVOS: self.joints: ', self.joints
            
        for controller in sorted(self.joints):
            controller = controller.lower().replace('_joint', '_controller')
            #torque_service = '/' + controller + '/torque_enable'
            torque_enable_service = '/' + namespace + '/' + controller + '/torque_enable'
            
            #rospy.loginfo('torque_service: ', torque_enable_service)
            #print 'controller: ', controller
            print 'torque_enable_service: ', torque_enable_service
            
            rospy.wait_for_service(torque_enable_service)  
            torque_services.append(rospy.ServiceProxy(torque_enable_service, TorqueEnable))

            print 'torque_service became available'
            
            set_torque_limit_service = '/' + namespace + '/' + controller + '/set_torque_limit'
            rospy.wait_for_service(set_torque_limit_service)  
            set_torque_limit_services.append(rospy.ServiceProxy(set_torque_limit_service, SetTorqueLimit))

            print 'torque_limit_service became available'
            
            speed_service = '/' + namespace + '/' + controller + '/set_speed'
            rospy.wait_for_service(speed_service)  
            speed_services.append(rospy.ServiceProxy(speed_service, SetSpeed))

            print 'speed_service became available'

        # Set the default speed to something small
        for set_speed in speed_services:
            try:
                set_speed(default_dynamixel_speed)
            except:
                pass
            
        # Set the torque limit to a moderate value
        for set_torque_limit in set_torque_limit_services:
            try:
                set_torque_limit(default_dynamixel_torque)
            except:
                pass

        # Relax all servos to give them a rest.
        for torque_enable in torque_services:
            try:
                torque_enable(True)
                print '***************** TORQUE ENABLE *****************'
            except:
                pass
        
if __name__=='__main__':
    Relax()