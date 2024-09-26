#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#


import os
#NEPI_BASE_NAMESPACE = '/nepi/s2x/'
#os.environ["ROS_NAMESPACE"] = NEPI_BASE_NAMESPACE[0:-1]
import rospy
import glob
import rosnode
import sys
import subprocess
import time
import warnings

from nepi_edge_sdk_base import nepi_ros
from nepi_edge_sdk_base import nepi_apps

from std_msgs.msg import Empty, String, Int32, Bool, Header
from nepi_ros_interfaces.msg import AppsStatus, AppStatus, UpdateState, UpdateOrder
from nepi_ros_interfaces.srv import SystemStorageFolderQuery, SystemStorageFolderQueryResponse

from nepi_edge_sdk_base.save_cfg_if import SaveCfgIF


APPS_FALLBACK_FOLDER = '/opt/nepi/ros/share/nepi_apps'
APPS_INSTALL_FALLBACK_FOLDER = '/mnt/nepi_storage/nepi_src/nepi_apps'

#########################################

#########################################
# Node Class
#########################################

class NepiAppsMgr(object):

  APP_UPDATE_CHECK_INTERVAL = 5
  save_cfg_if = None
  apps_folder = ''
  apps_files = []
  apps_ordered_list = []
  apps_active_list = []
  apps_install_folder = ''
  apps_install_files = []
  apps_install_list = []
  apps_active_dict = dict()
  status_apps_msg = None

  #################################################################
  DEFAULT_NODE_NAME = "apps_mgr"
  def publishMsg(self,msg,level = None):
    nepi_ros.printMsg(self.node_name, msg, level)
    if (self.msg_pub.get_num_connections() > 0):
      self.msg_pub.publish(str(msg))
    if (self.msg_pub_sys.get_num_connections() > 0):
      self.msg_pub_sys.publish(self.node_name + " " + str(msg))
  #######################
  ### Node Initialization
  def __init__(self):
    rospy.init_node(name=self.DEFAULT_NODE_NAME) # Node name could be overridden via remapping
    rospy.loginfo("Starting " + self.DEFAULT_NODE_NAME)
    self.node_name = rospy.get_name().split('/')[-1]
    # Create a node msg publisher
    self.msg_pub = rospy.Publisher("~messages", String, queue_size=1)
    self.msg_pub_sys = rospy.Publisher("messages", String, queue_size=1)
    time.sleep(1)
    #################################################################
    self.base_namespace = nepi_ros.get_base_namespace()
    # Get app folder paths
    self.apps_folder = APPS_FALLBACK_FOLDER
    #self.publishMsg("App folder set to " + self.apps_folder)
    self.apps_files = nepi_apps.getAppLaunchFilesList(self.apps_folder)
    #self.publishMsg("App folder files " + str(self.apps_files))
    # Get Install Apps Folder

    NEPI_BASE_NAMESPACE = nepi_ros.get_base_namespace()
    get_folder_name_service = NEPI_BASE_NAMESPACE + 'system_storage_folder_query'
    self.publishMsg("Waiting for system storage folder query service " + get_folder_name_service)
    rospy.wait_for_service(get_folder_name_service)
    self.publishMsg("Calling system storage folder query service " + get_folder_name_service)
    try:
        self.publishMsg("Getting storage folder query service " + get_folder_name_service)
        folder_query_service = rospy.ServiceProxy(get_folder_name_service, SystemStorageFolderQuery)
        response = folder_query_service('nepi_src/nepi_apps')
        self.publishMsg("Got storage folder path" + response.folder_path)
        time.sleep(1)
        self.save_data_root_directory = response.folder_path
    except Exception as e:
        self.save_data_root_directory = APPS_INSTALL_FALLBACK_FOLDER
        rospy.logwarn(self.node_name + "Failed to obtain system data folder, falling back to: " + APPS_INSTALL_FALLBACK_FOLDER + " " + str(e))
    if os.path.exists(self.save_data_root_directory):
        self.apps_install_folder = self.save_data_root_directory
    else:
        self.save_data_root_directory = ""

    self.publishMsg("App folder set to " + self.apps_install_folder)
    self.apps_install_files = nepi_apps.getAppPackagesList(self.apps_install_folder)
    self.publishMsg("App install packages folder files " + str(self.apps_install_files))

    # Setup Node Status Publisher
    self.apps_status_pub = rospy.Publisher("~status", AppsStatus, queue_size=1, latch=True)
    self.app_status_pub = rospy.Publisher("~status_app", AppStatus, queue_size=1, latch=True)

    # Setup message publisher and init param server
    self.publishMsg("Starting Initialization Processes")


    self.initParamServerValues(do_updates = False)

    self.save_cfg_if = SaveCfgIF(updateParamsCallback=self.initParamServerValues, 
                                 paramsModifiedCallback=self.updateFromParamServer)
        
 
    ## Mgr ROS Setup 
    mgr_reset_sub = rospy.Subscriber('~factory_reset', Empty, self.resetMgrCb, queue_size = 10)
    mgr_reset_sub = rospy.Subscriber('~refresh_apps', Empty, self.refreshCb, queue_size = 10)


    # Apps Management Scubscirbers
    rospy.Subscriber('~select_app', String, self.selectAppCb)
    rospy.Subscriber('~update_state', UpdateState, self.updateStateCb)
    rospy.Subscriber('~update_order', UpdateOrder, self.updateOrderCb)

    #rospy.Subscriber('~install_app_pkg', String, self.installAppPkgCb)
    #rospy.Subscriber('~backup_on_remeove', Bool, self.enableBackupCb)
    #rospy.Subscriber('~remove_app', String, self.removeAppCb)


    # Setup a app folder timed check
    rospy.Timer(rospy.Duration(1), self.checkAndUpdateCb, oneshot=True)
    time.sleep(1)
    ## Publish Status
    self.publish_status()

    #########################################################
    ## Initiation Complete
    self.publishMsg("Initialization Complete")
    #Set up node shutdown
    rospy.on_shutdown(self.cleanup_actions)
    # Spin forever (until object is detected)
    rospy.spin()
    #########################################################

        
  # ln = sys._getframe().f_lineno ; 
  def printND(self):
    level = 'WARN'
    apps_dict = rospy.get_param("~apps_dict",self.init_apps_dict)
    self.publishMsg('',level)
    self.publishMsg('*******************')
    self.publishMsg('Printing Apps Dictionary')
    for app_name in apps_dict.keys():
      app_dict = apps_dict[app_name]
      self.publishMsg('',level)
      self.publishMsg(app_name)
      self.publishMsg(str(app_dict))

  def publish_status(self):
    self.publish_apps_status()
    self.publish_app_status()


  def publish_apps_status(self):
    self.last_status_apps_msg = self.status_apps_msg
    self.status_apps_msg = self.getAppsStatusMsg()
    if not rospy.is_shutdown():
      self.apps_status_pub.publish(self.status_apps_msg)
    self.save_cfg_if.saveConfig(do_param_updates = False) # Save config after initialization for appt time

  def getAppsStatusMsg(self):
    apps_dict = rospy.get_param("~apps_dict",self.init_apps_dict)
    self.apps_ordered_list = nepi_apps.getAppsOrderedList(apps_dict)
    self.apps_active_list = nepi_apps.getAppsActiveOrderedList(apps_dict)
    status_apps_msg = AppsStatus()
    status_apps_msg.apps_path = self.apps_folder
    status_apps_msg.apps_ordered_list = str(self.apps_ordered_list)
    status_apps_msg.apps_active_list = str(self.apps_active_list)
    status_apps_msg.apps_install_path = self.apps_install_folder
    status_apps_msg.apps_install_list = str(self.apps_install_files)
    status_apps_msg.apps_rui_lists = str(nepi_apps.getAppsRuiActiveLists(apps_dict))
    status_apps_msg.app_backup_path = self.apps_install_folder
    status_apps_msg.backup_removed_apps = rospy.get_param("~backup_enabled",self.init_backup_enabled)
    status_apps_msg.selected_app = self.selected_app
    return status_apps_msg

  
  def publish_app_status(self):
    self.status_app_msg = self.getAppStatusMsg()
    if not rospy.is_shutdown():
      self.app_status_pub.publish(self.status_app_msg)


  def getAppStatusMsg(self):
    apps_dict = rospy.get_param("~apps_dict",self.init_apps_dict)
    app_name = self.selected_app
    status_app_msg = AppStatus()
    status_app_msg.name = app_name
    if app_name in apps_dict.keys() and app_name != 'NONE':
      app = apps_dict[app_name]
      status_app_msg.active_state  = app['active']
      status_app_msg.order  = app['order']
      status_app_msg.description = app['description']
      status_app_msg.msg_str = app['msg']
    return status_app_msg

  
  def checkAndUpdateCb(self,_):
    ###############################
    ## First update Database
    apps_dict = rospy.get_param("~apps_dict",self.init_apps_dict)
    apps_files = nepi_apps.getAppLaunchFilesList(self.apps_folder)
    self.apps_install_files = nepi_apps.getAppPackagesList(self.apps_install_folder)
    need_update = self.apps_files != apps_files
    if need_update:
      self.publishMsg("Need to Update App Database")
      try:
        apps_dict = rospy.get_param("~apps_dict")
        apps_dict = nepi_apps.updateAppsDict(self.apps_folder,apps_dict)
      except:
        apps_dict = nepi_apps.getAppsDict(self.apps_folder)
        apps_dict = nepi_apps.setFactoryAppOrder(apps_dict)
        apps_dict = nepi_apps.activateAllApps(apps_dict)
    #self.printND()
    rospy.set_param("~apps_dict",apps_dict)
    self.apps_files = apps_files
    self.apps_ordered_list = nepi_apps.getAppsOrderedList(apps_dict)
    self.apps_active_list = nepi_apps.getAppsActiveOrderedList(apps_dict)
    ## process active app processes


    ###############################
    # Get list of active nodes
    warnings.filterwarnings('ignore', '.*unclosed.*', ) 
    node_namespace_list = nepi_ros.get_node_list()
    node_list = []
    for i in range(len(node_namespace_list)):
      node_list.append(node_namespace_list[i].split("/")[-1])

    ################################    
    ## Check and purge disabled app proccess that might be running
    # First check on running nodes
    purge_list = []
    for app_name in self.apps_ordered_list:
      if app_name not in self.apps_active_list and app_name in self.apps_active_dict.keys():
          node_name = self.apps_active_dict[app_name]['node_name']
          if node_name in node_list:
            sub_process = self.apps_active_dict[app_name]['subprocess']
            success = nepi_apps.killAppNode(node_name,sub_process)
            if success:
              purge_list.append(app_name)
    # purge from active discovery dict
    for app_name in purge_list:
      if app_name in self.apps_active_dict.keys():
        del self.apps_active_dict[app_name]


    ###############################
    #rospy.logwarn("DRV_MGR: App active dict: " + str(self.apps_active_dict.keys()))
    #self.publishMsg(node_list,level = "WARN")
    for app_name in self.apps_ordered_list:
      if app_name in self.apps_active_list and app_name in apps_dict.keys():
        app_dict = apps_dict[app_name]
        #self.publishMsg(app_dict,level = "WARN")
        app_pkg_name = app_dict['pkg_name']
        app_file_name = app_dict['app_file']
        app_path_name = app_dict['app_path']
        app_node_name = app_dict['node_name']
        app_file_path = app_path_name + '/' + app_file_name
        #rospy.logwarn("DRV_MGR: Checking for running node: " + app_node_name)
        # Check Auto-Node processes
        #rospy.logwarn("DRV_MGR: Checking for Node: " + app_node_name)
        #rospy.logwarn("DRV_MGR: Node List: " + str(node_list))
        if app_node_name not in node_list: #node_list:
          #Try and launch node
          self.publishMsg("")
          self.publishMsg("Launching application node: " + app_node_name)
          [success, msg, sub_process] = nepi_apps.launchAppNode(app_pkg_name, app_file_name, app_node_name)
          if success:
            apps_dict[app_name]['msg'] = "Discovery process lanched"
            self.apps_active_dict[app_name]=dict()
            self.apps_active_dict[app_name]['node_name'] = app_node_name
            self.apps_active_dict[app_name]['subprocess'] = sub_process
          else:
            apps_dict[app_name]['msg'] = msg
        else:
          apps_dict[app_name]['msg'] = "App process running"
        time.sleep(1)

    # Publish Status
    self.publish_status()
    # And now that we are finished, start a timer for the appt runDiscovery()
    nepi_ros.sleep(self.APP_UPDATE_CHECK_INTERVAL,100)
    rospy.Timer(rospy.Duration(1), self.checkAndUpdateCb, oneshot=True)
   
  

  ###################
  ## Apps Mgr Callbacks

  def selectAppCb(self,msg):
    self.publishMsg(msg)
    app_name = msg.data
    apps_dict = rospy.get_param("~apps_dict",self.init_apps_dict)
    if app_name in apps_dict.keys() or app_name == "NONE":
      self.selected_app = app_name
    self.publish_status()

  def updateStateCb(self,msg):
    self.publishMsg(msg)
    app_name = msg.name
    new_active_state = msg.active_state
    apps_dict = rospy.get_param("~apps_dict",self.init_apps_dict)
    if app_name in apps_dict.keys():
      app = apps_dict[app_name]
      active_state = app['active']
    if new_active_state != active_state:
      if new_active_state == True:
        apps_dict = nepi_apps.activateApp(app_name,apps_dict)
      else:
        apps_dict = nepi_apps.disableApp(app_name,apps_dict)
    rospy.set_param("~apps_dict",apps_dict)
    self.publish_status()


  def updateOrderCb(self,msg):
    self.publishMsg(msg)
    app_name = msg.name
    move_cmd = msg.move_cmd
    moveFunction = self.getOrderUpdateFunction(move_cmd)
    apps_dict = rospy.get_param("~apps_dict",self.init_apps_dict)
    if app_name in apps_dict.keys():
      apps_dict = moveFunction(app_name,apps_dict)
    rospy.set_param("~apps_dict",apps_dict)
    self.publish_status()


  def getOrderUpdateFunction(self,move_cmd):
    if move_cmd == 'top':
      updateFunction = nepi_apps.moveAppTop
    elif move_cmd == 'bottom':
      updateFunction = nepi_apps.moveAppBottom
    elif move_cmd == 'up':
      updateFunction = nepi_apps.moveAppUp
    elif move_cmd == 'down':
      updateFunction = nepi_apps.moveAppDown
    else:
      updateFunction = self.moveAppNone
    return updateFunction

  def moveAppNone(self,app_name,apps_dict):
    return apps_dict



  def installAppPkgCb(self,msg):
    self.publishMsg(msg)
    pkg_name = msg.data
    apps_dict = rospy.get_param("~apps_dict",self.init_apps_dict)
    if pkg_name in self.apps_install_files:
     [success,apps_dict]  = nepi_apps.installAppPkg(pkg_name,apps_dict,self.apps_install_folder,self.apps_folder)
    rospy.set_param("~apps_dict",apps_dict)
    self.publish_status()

  def removeAppCb(self,msg):
    self.publishMsg(msg)
    app_name = msg.data
    apps_dict = rospy.get_param("~apps_dict",self.init_apps_dict)
    backup_folder = None
    backup_enabled = rospy.get_param("~backup_enabled",self.init_backup_enabled)
    if backup_enabled:
      backup_folder = self.apps_install_folder
    if app_name in apps_dict:
      [success,apps_dict] = nepi_apps.removeApp(app_name,apps_dict,backup_folder)
    rospy.set_param("~apps_dict",apps_dict)
    self.publish_status()


  def enableBackupCb(self,msg):
    self.publishMsg(msg)
    backup_enabled = msg.data
    rospy.set_param("~backup_enabled",backup_enabled)
    self.publish_status()



   #######################
  ### Mgr Config Functions

  def resetMgrCb(self,msg):
    self.resetMgr()

  def resetMgr(self):
    # reset apps dict
    rospy.set_param("~backup_enabled",True)
    self.apps_files = nepi_apps.getAppLaunchFilesList(self.apps_folder)
    self.apps_install_files = nepi_apps.getAppPackagesList(self.apps_install_folder)
    apps_dict = nepi_apps.getAppsgetAppsDict(self.apps_folder)
    apps_dict = nepi_apps.setFactoryAppOrder(apps_dict)
    apps_dict = activateAllApps(apps_dict)
    rospy.set_param("~apps_dict",apps_dict)
    self.publish_status()


  def refreshCb(self,msg):
    self.refresh()

  def refresh(self):
    # refresh apps dict
    self.apps_files = nepi_apps.getAppLaunchFilesList(self.apps_folder)
    self.apps_install_files = nepi_apps.getAppPackagesList(self.apps_install_folder)
    apps_dict = rospy.get_param("~apps_dict",self.init_apps_dict)
    apps_dict = nepi_apps.refreshAppsDict(self.apps_folder,apps_dict)
    rospy.set_param("~apps_dict",apps_dict)
    self.publish_status()

  def saveConfigCb(self, msg):  # Just update Class init values. Saving done by Config IF system
    pass # Left empty for sim, Should update from param server

  def setCurrentAsDefault(self):
    rospy.set_param("~backup_enabled", True)
    self.initParamServerValues(do_updates = False)

  def updateFromParamServer(self):
    #rospy.logwarn("DRV_MGR: Debugging: param_dict = " + str(param_dict))
    #Run any functions that need updating on value change
    # Don't need to run any additional functionsinstallAppPkgCb
    pass
    
  def initParamServerValues(self,do_updates = True):
      self.selected_app = 'NONE'
      self.publishMsg("Setting init values to param values")
      self.init_backup_enabled = rospy.get_param("~backup_enabled", True)
      self.apps_files = nepi_apps.getAppLaunchFilesList(self.apps_folder)
      self.apps_install_files = nepi_apps.getAppPackagesList(self.apps_install_folder)
      #self.publishMsg(str(rospy.get_param("~apps_dict")))
      try:
        apps_dict = rospy.get_param("~apps_dict")
        apps_dict = nepi_apps.updateAppsDict(self.apps_folder,apps_dict)
        self.publishMsg("Got apps_dict values from param server")
      except:
        apps_dict = nepi_apps.getAppsDict(self.apps_folder)
        apps_dict = nepi_apps.setFactoryAppOrder(apps_dict)
        apps_dict = nepi_apps.activateAllApps(apps_dict)
        self.publishMsg("Got apps_dict values from reading in file data")
      self.init_apps_dict = apps_dict
      rospy.set_param("~apps_dict",apps_dict)
      #self.printND()
      self.resetParamServer(do_updates)
      #self.printND()

  def resetParamServer(self,do_updates = True):
      rospy.set_param("~backup_enabled",self.init_backup_enabled)
      rospy.set_param("~apps_dict",self.init_apps_dict)
      if do_updates:
          self.updateFromParamServer()
          self.publish_status()

  #######################
  # Misc Utility Function

  def getAvailableDevPaths(self):
    dev_path_list = glob.glob('/dev/*')
    return dev_path_list


  #######################
  # Node Cleanup Function
  
  def cleanup_actions(self):
    self.publishMsg("Shutting down: Executing script cleanup actions")


#########################################
# Main
#########################################
if __name__ == '__main__':
  NepiAppsMgr()







