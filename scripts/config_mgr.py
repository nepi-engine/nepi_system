#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#

# The config_mgr node serves as a bridge between the ROS param server and the filesystem.
# It provides the rest of the Numurus/ROS node set the ability to save and restore config.
# files with rudimentary coordination to reduce overloading file system during times of
# heavy updates.

import rospy
import rosparam

import os
import errno

from nepi_edge_sdk_base import nepi_ros
from nepi_edge_sdk_base import nepi_msg 

from std_msgs.msg import String, Empty
from nepi_ros_interfaces.srv import FileReset



CFG_PATH = '/opt/nepi/ros/etc'
CFG_SUFFIX = '.yaml'
FACTORY_SUFFIX = '.num_factory'

USER_CFG_PATH = '/mnt/nepi_storage/user_cfg'
USER_SUFFIX = '.user'

pending_nodes = {}

# Files outside the normal NEPI-ROS cfg. scheme
SYS_CFGS_TO_PRESERVE = {
    'sys_env.bash' : '/opt/nepi/sys_env.bash', # Serial number, ROS launch file, external ROS MASTER etc.
    'authorized_keys' : '/opt/nepi/config/home/nepi/ssh/authorized_keys', # NEPI Device SSH public keys
    'hostname' : '/opt/nepi/config/etc/hostname', # NEPI Device hostname
    'sshd_config' : '/opt/nepi/config/etc/ssh/sshd_config', # SSH Server Config
    'chrony.conf' : '/opt/nepi/config/etc/chrony/chrony.conf.user', # NTP/Chrony Config
    'iptables.rules' : '/opt/nepi/config/etc/iptables/s2x_iptables.rules', # Route and forwarding rules; e.g., for dual-interface devices
    'sensor_if_static_ip' : '/opt/nepi/config/etc/network/interfaces.d/s2x_sensor_if_static_ip', # Static IP address for secondary/sensor ethernet interface
    'ip_aliases' : '/opt/nepi/config/etc/network/interfaces.d/nepi_user_ip_aliases', # IP alias addresses for primary ethernet interface
    'static_ip' : '/opt/nepi/config/etc/network/interfaces.d/nepi_static_ip', # Principal static IP address for primary ethernet interface
    'fstab' : '/opt/nepi/config/etc/fstab', # Filesystem mounting rules; e.g., nepi_storage on SD vs SSD
    'smb.conf' : '/opt/nepi/config/etc/samba/smb.conf', # Samba configuration
    'nepi_connect_cfg' : '/opt/nepi/nepi_link/nepi-bot/devinfo/', # NEPI Connect Device NUID, SSH keys, etc.
    'nepi_connect_bot_cfg.json' : '/opt/nepi/nepi_link/nepi-bot/cfg/bot/config.json' # NEPI Connect device (nepi-bot) config file
}

class config_mgr(object):
    #######################
    ### Node Initialization
    DEFAULT_NODE_NAME = "config_mgr" # Can be overwitten by luanch command
    def __init__(self):
        #### APP NODE INIT SETUP ####
        nepi_ros.init_node(name= self.DEFAULT_NODE_NAME)
        self.node_name = nepi_ros.get_node_name()
        self.base_namespace = nepi_ros.get_base_namespace()
        nepi_msg.createMsgPublishers(self)
        nepi_msg.publishMsgInfo(self,"Starting Initialization Processes")
        ##############################

        rospy.Subscriber('save_config', Empty, self.save_non_ros_cfgs) # Global one only
        rospy.Subscriber('store_params', String, self.store_params)
        rospy.Subscriber('full_user_restore', Empty, self.restore_user_cfgs_mgr)

        rospy.Service('factory_reset', FileReset, self.factory_reset)
        rospy.Service('user_reset', FileReset, self.user_reset)

        # Restore user configurations
        self.restore_user_cfgs()

        #########################################################
        ## Initiation Complete
        nepi_msg.publishMsgInfo(self,"Initialization Complete")
        # Spin forever (until object is detected)
        nepi_ros.spin()
        #########################################################


    # Moving symlinks is typically faster and more robust than copying files, so to reduce the
    # chance of filesystem corruption in the event of e.g., power failure, we use a symlink-based config
    # file scheme.
    def symlink_force(self,target, link_name):
        link_dirname = os.path.dirname(link_name)
        if not os.path.exists(link_dirname):
            nepi_msg.publishMsgWarn(self,"Skipping symlink for " + link_name + " because path does not exist... missing factory config?")
            return FalsepublishJointStateAndStatus
        return True

    def separate_node_name_in_msg(self,qualified_node_name):
        return qualified_node_name.split("/")[-1]

    def update_from_file(self,file_pathname, namespace):
        try:
            paramlist = rosparam.load_file(file_pathname, namespace, verbose=True)
            for params, ns in paramlist:
                rosparam.upload_params(ns, params, verbose=True)
        except:
            nepi_msg.publishMsg(self,"Unable to load factory parameters from file " + file_pathname)
            return [False]

        return [True]

    def get_cfg_pathname(self,qualified_node_name):
        node_name = self.separate_node_name_in_msg(qualified_node_name)
        subfolder_name = "/"
        if nepi_ros.find_topic(qualified_node_name + "/idx").find("idx") != -1:
            subfolder_name = "/drivers/"
        elif nepi_ros.find_topic(qualified_node_name + "/lsx").find("lsx") != -1:
            subfolder_name = "/drivers/"
        elif nepi_ros.find_topic(qualified_node_name + "/ptx").find("ptx") != -1:
            subfolder_name = "/drivers/"
        elif nepi_ros.find_topic(qualified_node_name + "/rbx").find("rbx") != -1:
            subfolder_name = "/drivers/"
        elif nepi_ros.find_topic(qualified_node_name + "/npx").find("npx") != -1:
            subfolder_name = "/drivers/"
        #nepi_msg.publishMsgInfo(self,node_name)
        cfg_pathname = CFG_PATH + subfolder_name + node_name  + '/' + node_name + CFG_SUFFIX
        #nepi_msg.publishMsgInfo(self,cfg_pathname)
        return cfg_pathname

    def get_user_cfg_pathname(self,qualified_node_name):
        node_name = self.separate_node_name_in_msg(qualified_node_name)
        user_cfg_dirname = os.path.join(USER_CFG_PATH, 'ros')
        
        # Ensure the path we report actually exists
        if not os.path.isdir(user_cfg_dirname):
            os.makedirs(user_cfg_dirname)

        user_cfg_pathname = os.path.join(user_cfg_dirname, node_name + CFG_SUFFIX + USER_SUFFIX)
        return user_cfg_pathname

    def user_reset(self,req):
        qualified_node_name = req.node_name
        cfg_pathname = self.get_cfg_pathname(qualified_node_name)

        # Now update the param server
        return self.update_from_file(cfg_pathname, qualified_node_name)

    def factory_reset(self,req):
        qualified_node_name = req.node_name
        cfg_pathname = self.get_cfg_pathname(qualified_node_name)
        factory_cfg_pathname = cfg_pathname + FACTORY_SUFFIX

        # First, move the symlink
        if False == self.symlink_force(factory_cfg_pathname, cfg_pathname):
            return [False] # Error logged upstream

        # Now update the param server
        return self.update_from_file(cfg_pathname, qualified_node_name)

    def store_params(self,msg):
        qualified_node_name = msg.data
        user_cfg_pathname = self.get_user_cfg_pathname(qualified_node_name)
        
        # First, write to the user file
        rosparam.dump_params(user_cfg_pathname, qualified_node_name)

        # Now, ensure the link points to the correct file
        cfg_pathname = self.get_cfg_pathname(qualified_node_name)
        self.symlink_force(user_cfg_pathname, cfg_pathname) # Error logged upstream

    def save_non_ros_cfgs(self,msg):
        target_dir = os.path.join(USER_CFG_PATH, 'sys')
        if not os.path.exists(target_dir):
            os.makedirs(target_dir)

        for cfg in SYS_CFGS_TO_PRESERVE:
            source = SYS_CFGS_TO_PRESERVE[cfg]
            target = os.path.join(USER_CFG_PATH, 'sys', cfg)
            os.system('cp -rf ' + source + ' ' + target)

    def restore_user_cfgs_mgr(self,msg):
        self.restore_user_cfgs()

    def restore_user_cfgs(self):
        # First handle the NEPI-ROS user configs.
        for root, dirs, files in os.walk(CFG_PATH):
            for name in files:
                full_name = os.path.join(root, name)
                if full_name.endswith(CFG_SUFFIX) and os.path.islink(full_name):
                    user_cfg_name = os.path.join(USER_CFG_PATH, 'ros', name + USER_SUFFIX)
                    if os.path.exists(user_cfg_name): # Restrict to those with present user configs
                        link_name = os.path.join(root, name.replace(FACTORY_SUFFIX, ''))
                        nepi_msg.publishMsgInfo(self,"Updating " + link_name + " to user config")
                        self.symlink_force(user_cfg_name, link_name)
                    else:
                    	nepi_msg.publishMsgInfo(self,"User config file does not exist at " + user_cfg_name)

        # Now handle non-ROS user system configs.        
        for name in SYS_CFGS_TO_PRESERVE:
            full_name = os.path.join(USER_CFG_PATH, 'sys', name)
            if os.path.exists(full_name):
                if os.path.isdir(full_name):
                    full_name = os.path.join(full_name,'*') # Wildcard avoids copying source folder into target folder as a subdirectory
                target = SYS_CFGS_TO_PRESERVE[name]
                nepi_msg.publishMsgInfo(self,"Upettings_capabilities_query:1672] call_service InvalidServiceException: Service /nedating " + target + " from user config")
                os.system('cp -rf ' + full_name + ' ' + target)
                os.system('chown -R nepi:nepi ' + target)



if __name__ == '__main__':
    config_mgr()
