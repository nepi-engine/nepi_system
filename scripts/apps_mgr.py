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
#self.base_namespace = '/nepi/s2x/'
#os.environ["ROS_NAMESPACE"] = self.base_namespace[0:-1]
import rospy
import glob
import rosnode
import rosparam
import sys
import subprocess
import time
import warnings

from nepi_edge_sdk_base import nepi_ros
from nepi_edge_sdk_base import nepi_msg
from nepi_edge_sdk_base import nepi_apps

from std_msgs.msg import Empty, String, Int32, Bool, Header
from nepi_ros_interfaces.msg import AppsStatus, AppStatus, UpdateState, UpdateOrder
from nepi_ros_interfaces.srv import SystemStorageFolderQuery, SystemStorageFolderQueryResponse
from nepi_ros_interfaces.srv import AppStatusQuery, AppStatusQueryResponse

from nepi_edge_sdk_base.save_cfg_if import SaveCfgIF

APPS_FALLBACK_FOLDER = '/opt/nepi/ros/share/nepi_apps'
APPS_INSTALL_FALLBACK_FOLDER = '/mnt/nepi_storage/nepi_src/nepi_apps'
APPS_CONFIG_FALLBACK_FOLDER = '/opt/nepi/ros/etc'

#########################################

#########################################
# Node Class
#########################################

class NepiAppsMgr(object):


  UPDATE_CHECK_INTERVAL = 5
  PUBLISH_STATUS_INTERVAL = 1
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

  #######################
  ### Node Initialization
  DEFAULT_NODE_NAME = "apps_mgr" # Can be overwitten by luanch command
  def __init__(self):
    #### APP NODE INIT SETUP ####
    nepi_ros.init_node(name= self.DEFAULT_NODE_NAME)
    self.node_name = nepi_ros.get_node_name()
    self.base_namespace = nepi_ros.get_base_namespace()
    nepi_msg.createMsgPublishers(self)
    nepi_msg.publishMsgInfo(self,"Starting Initialization Processes")
    ##############################

    # Get app folder paths
    self.apps_folder = APPS_FALLBACK_FOLDER
    #nepi_msg.publishMsgInfo(self,"App folder set to " + self.apps_folder)
    self.apps_files = nepi_apps.getAppInfoFilesList(self.apps_folder)
    #nepi_msg.publishMsgInfo(self,"App folder files " + str(self.apps_files))
    # Initialize apps_mgr param server
    self.save_cfg_if = SaveCfgIF(updateParamsCallback=self.initParamServerValues, 
                                 paramsModifiedCallback=self.updateFromParamServer)
    self.save_cfg_if.userReset()
    time.sleep(3)
    apps_dict = nepi_ros.get_param(self,"~apps_dict",dict())
    app_dict = dict()
    #nepi_msg.publishMsgWarn(self,"Got init apps dict: " + str(apps_dict))
    for app_name in apps_dict:
      app_dict[app_name] = apps_dict[app_name]['active']
    nepi_msg.publishMsgInfo(self,"Got init app dict active list: " + str(app_dict))
    self.initParamServerValues(do_updates = True)

    self.init_active_list = nepi_ros.get_param(self,"~active_list",[])
    nepi_ros.set_param(self,"~active_list",self.init_active_list)

    # setup folders
    get_folder_name_service = self.base_namespace + 'system_storage_folder_query'
    nepi_msg.publishMsgInfo(self,"Waiting for system storage folder query service " + get_folder_name_service)
    rospy.wait_for_service(get_folder_name_service)
    nepi_msg.publishMsgInfo(self,"Calling system storage folder query service " + get_folder_name_service)
    try:
        nepi_msg.publishMsgInfo(self,"Getting storage folder query service " + get_folder_name_service)
        folder_query_service = rospy.ServiceProxy(get_folder_name_service, SystemStorageFolderQuery)
        response = folder_query_service('nepi_src/nepi_apps')
        nepi_msg.publishMsgInfo(self,"Got storage folder path" + response.folder_path)
        time.sleep(1)
        self.save_data_root_directory = response.folder_path
    except Exception as e:
        self.save_data_root_directory = APPS_INSTALL_FALLBACK_FOLDER
        nepi_msg.publishMsgWarn(self,self.node_name + "Failed to obtain system data folder, falling back to: " + APPS_INSTALL_FALLBACK_FOLDER + " " + str(e))
    if os.path.exists(self.save_data_root_directory):
        self.apps_install_folder = self.save_data_root_directory
    else:
        self.save_data_root_directory = ""

    nepi_msg.publishMsgInfo(self,"App folder set to " + self.apps_install_folder)
    self.apps_install_files = nepi_apps.getAppPackagesList(self.apps_install_folder)
    nepi_msg.publishMsgInfo(self,"App install packages folder files " + str(self.apps_install_files))

    # Setup Node Status Publisher
    self.apps_status_pub = rospy.Publisher("~status", AppsStatus, queue_size=1, latch=True)
    self.app_status_pub = rospy.Publisher("~status_app", AppStatus, queue_size=1, latch=True)

    time.sleep(1)
    rospy.Timer(rospy.Duration(0.5), self.statusPublishCb)

    # Setup message publisher and init param server
    nepi_msg.publishMsgInfo(self,"Starting Initialization Processes")


 
    ## Mgr ROS Setup 
    mgr_reset_sub = rospy.Subscriber('~factory_reset', Empty, self.resetMgrCb, queue_size = 10)
    mgr_reset_sub = rospy.Subscriber('~refresh_apps', Empty, self.refreshCb, queue_size = 10)


    # Apps Management Scubscirbers
    rospy.Subscriber('~enable_all_apps', Empty, self.enableAllCb, queue_size = 10)
    rospy.Subscriber('~disable_all_apps', Empty, self.disableAllCb, queue_size = 10)
    rospy.Subscriber('~select_app', String, self.selectAppCb)
    rospy.Subscriber('~update_state', UpdateState, self.updateStateCb)
    rospy.Subscriber('~update_order', UpdateOrder, self.updateOrderCb)

    #rospy.Subscriber('~install_app_pkg', String, self.installAppPkgCb)
    #rospy.Subscriber('~backup_on_remeove', Bool, self.enableBackupCb)
    #rospy.Subscriber('~remove_app', String, self.removeAppCb)

    # Start capabilities services
    rospy.Service('~app_status_query', AppStatusQuery, self.appStatusService)


    # Setup a app folder timed check
    nepi_ros.timer(nepi_ros.duration(1), self.checkAndUpdateCb, oneshot=True)
    nepi_ros.timer(nepi_ros.duration(self.PUBLISH_STATUS_INTERVAL), self.publishStatusCb, oneshot=True)
    time.sleep(1)
    ## Publish Status
    self.publish_status()

    #########################################################
    ## Initiation Complete
    nepi_msg.publishMsgInfo(self,"Initialization Complete")
    #Set up node shutdown
    nepi_ros.on_shutdown(self.cleanup_actions)
    # Spin forever (until object is detected)
    nepi_ros.spin()
    #########################################################


   #######################
  ### Mgr Config Functions

  def resetMgrCb(self,msg):
    self.resetMgr()

  def resetMgr(self):
    # reset apps dict
    nepi_ros.set_param(self,"~backup_enabled",True)
    self.apps_files = nepi_apps.getAppInfoFilesList(self.apps_folder)
    self.apps_install_files = nepi_apps.getAppPackagesList(self.apps_install_folder)
    apps_dict = nepi_apps.getAppsgetAppsDict(self.apps_folder)
    apps_dict = nepi_apps.setFactoryAppOrder(apps_dict)
    apps_dict = activateAllApps(apps_dict)
    nepi_ros.set_param(self,"~apps_dict",apps_dict)
    self.resetParamServer()
    self.publish_status()


  def refreshCb(self,msg):
    self.refresh()

  def refresh(self):
    # refresh apps dict
    self.apps_files = nepi_apps.getAppInfoFilesList(self.apps_folder)
    self.apps_install_files = nepi_apps.getAppPackagesList(self.apps_install_folder)
    apps_dict = nepi_ros.get_param(self,"~apps_dict",self.init_apps_dict)
    apps_dict = nepi_apps.refreshAppsDict(self.apps_folder,apps_dict)
    nepi_ros.set_param(self,"~apps_dict",apps_dict)
    self.publish_status()

  def saveConfigCb(self, msg):  # Just update Class init values. Saving done by Config IF system
    pass # Left empty for sim, Should update from param server

  def setCurrentAsDefault(self):
    nepi_ros.set_param(self,"~backup_enabled", True)
    self.initParamServerValues(do_updates = False)
    self.publish_status()

  def updateFromParamServer(self):
    #Run any functions that need updating on value change
    # Don't need to run any additional functionsinstallAppPkgCb
    pass
    
  def initParamServerValues(self,do_updates = True):
      self.selected_app = 'NONE'
      nepi_msg.publishMsgInfo(self,"Setting init values to param values")
      self.init_backup_enabled = nepi_ros.get_param(self,"~backup_enabled", True)
      self.apps_files = nepi_apps.getAppInfoFilesList(self.apps_folder)
      self.apps_install_files = nepi_apps.getAppPackagesList(self.apps_install_folder)
      none_dict = dict(NoneApps = "None")
      apps_dict = nepi_ros.get_param(self,"~apps_dict",none_dict)
      if 'NoneApps' not in apps_dict.keys():
        try:
          apps_dict = nepi_ros.get_param(self,"~apps_dict",apps_dict)
          apps_dict = nepi_apps.updateAppsDict(self.apps_folder,apps_dict)
        except:
          nepi_msg.publishMsgWarn(self,"Got bad dict from param server server so reseting")
          apps_dict = none_dict
      if 'NoneApps' in apps_dict.keys():
        apps_dict = nepi_apps.getAppsDict(self.apps_folder)
        apps_dict = nepi_apps.setFactoryAppOrder(apps_dict)
        active_list = nepi_ros.get_param(self,"~active_list",[])
        if "ALL" in active_list:
          apps_dict = nepi_apps.activateAllApps(apps_dict)
        else:
          apps_dict = nepi_apps.initAppsActiveOrder(active_list,apps_dict)
      #self.printND()
      self.init_apps_dict = apps_dict
      self.apps_ordered_list = nepi_apps.getAppsOrderedList(apps_dict)
      self.apps_active_list = nepi_apps.getAppsActiveOrderedList(apps_dict)
      self.init_active_list = self.apps_active_list

      self.resetParamServer(do_updates)
      #self.printND()

  def resetParamServer(self,do_updates = True):
      nepi_ros.set_param(self,"~apps_dict",self.init_apps_dict)
      nepi_ros.set_param(self,"~active_list",self.init_active_list)
      nepi_ros.set_param(self,"~backup_enabled",self.init_backup_enabled)
      nepi_ros.set_param(self,"~apps_dict",self.init_apps_dict)
      if do_updates:
          self.updateFromParamServer()

  def publishStatusCb(self,timer):
    self.publish_status()


  def checkAndUpdateCb(self,_):
    ###############################
    ## First update Database
    
    apps_files = nepi_apps.getAppInfoFilesList(self.apps_folder)
    self.apps_install_files = nepi_apps.getAppPackagesList(self.apps_install_folder)
    need_update = self.apps_files != apps_files
    if need_update:
      nepi_msg.publishMsgInfo(self,"Need to Update App Database")
      self.initParamServerValues()
    apps_dict = nepi_ros.get_param(self,"~apps_dict",self.init_apps_dict)
    ## process active app processes


    ###############################
    # Get list of active nodes
    warnings.filterwarnings('ignore', '.*unclosed.*', ) 
    node_namespace_list = nepi_ros.get_node_list()
    node_list = []
    for i in range(len(node_namespace_list)):
      node_list.append(node_namespace_list[i].split("/")[-1])
    #nepi_msg.publishMsgInfo(self,str(node_list),level = "WARN")
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


    ################################    
    ## Process Apps
    for app_name in self.apps_ordered_list:
      if app_name in self.apps_active_list and app_name in apps_dict.keys():
        app_dict = apps_dict[app_name]
        #nepi_msg.publishMsgInfo(self,app_dict,level = "WARN")
        app_pkg_name = app_dict['APP_DICT']['pkg_name']
        app_file_name = app_dict['APP_DICT']['app_file']
        app_config_file_name = app_dict['APP_DICT']['config_file']
        app_path_name = app_dict['APP_DICT']['app_path']
        app_node_name = app_dict['APP_DICT']['node_name']
        app_file_path = app_path_name + '/' + app_file_name
        if app_name not in self.apps_active_dict.keys():
          #Try and initialize app param values
          config_file_path = APPS_CONFIG_FALLBACK_FOLDER + "/" + app_config_file_name.split(".")[0] + "/" + app_config_file_name
          params_namespace = os.path.join(self.base_namespace, app_node_name)
          apt_dict_pn = os.path.join(params_namespace, 'app_dict')
          nepi_ros.set_param(self,apt_dict_pn,app_dict)
          if os.path.exists(config_file_path):
            try:
              nepi_ros.load_params_from_file(config_file_path, params_namespace)
            except Exception as e:
              nepi_msg.publishMsgWarn(self,"Could not get params from config file: " + config_file_path + " " + str(e))
          else:
            nepi_msg.publishMsgWarn(self,"Could not find config file at: " + config_file_path + " starting with factory settings")
          #Try and launch node
          nepi_msg.publishMsgInfo(self,"")
          nepi_msg.publishMsgInfo(self,"Launching application node: " + app_node_name)
          [success, msg, sub_process] = nepi_apps.launchAppNode(app_pkg_name, app_file_name, app_node_name)
          if success:
            apps_dict[app_name]['msg'] = "Discovery process lanched"
            self.apps_active_dict[app_name]=dict()
            self.apps_active_dict[app_name]['node_name'] = app_node_name
            self.apps_active_dict[app_name]['subprocess'] = sub_process

    # Publish Status
    self.publish_status()
    # And now that we are finished, start a timer for the appt runDiscovery()
    nepi_ros.sleep(self.UPDATE_CHECK_INTERVAL,100)
    nepi_ros.timer(nepi_ros.duration(1), self.checkAndUpdateCb, oneshot=True)
   
  



  def appStatusService(self,request):
    app_name = request.name
    response = self.getAppStatusServiceMsg(app_name)
    return response


  def getAppStatusServiceMsg(self):
    apps_dict = nepi_ros.get_param(self,"~apps_dict",self.init_apps_dict)
    app_name = self.selected_app
    status_app_msg = AppStatusQueryResponse()
    status_app_msg.name = app_name
    if app_name in apps_dict.keys() and app_name != 'NONE':
      app = apps_dict[app_name]
      try:
        status_app_msg.pkg_name = app['APP_DICT']['pkg_name']
        status_app_msg.pkg_name = app['APP_DICT']['group_name']
        status_app_msg.description = app['APP_DICT']['description']
        status_app_msg.node_name = app['APP_DICT']['node_name']
        status_app_msg.app_file = app['APP_DICT']['app_file']
        status_app_msg.app_path = app['APP_DICT']['app_path']   
        status_app_msg.rui_files_list = app['RUI_DICT']['rui_files']
        status_app_msg.rui_main_file = app['RUI_DICT']['rui_main_file']
        status_app_msg.rui_main_class = app['RUI_DICT']['rui_main_class']  
        status_app_msg.rui_menu_name = app['RUI_DICT']['rui_menu_name']
        status_app_msg.active_state  = app['active']
        status_app_msg.order  = app['order']
        status_app_msg.msg_str = app['msg']
      except Exception as e:
        nepi_msg.publishMsgInfo(self,"Failed to create app status message: " + str(e))
    return status_app_msg
        
  # ln = sys._getframe().f_lineno ; 
  def printND(self):
    apps_dict = nepi_ros.get_param(self,"~apps_dict",self.init_apps_dict)
    nepi_msg.publishMsgInfo(self,'')
    nepi_msg.publishMsgInfo(self,'*******************')
    nepi_msg.publishMsgInfo(self,'Printing Apps Dictionary')
    for app_name in apps_dict.keys():
      app_dict = apps_dict[app_name]
      nepi_msg.publishMsgInfo(self,'')
      nepi_msg.publishMsgInfo(self,app_name)
      nepi_msg.publishMsgInfo(self,str(app_dict))



  def statusPublishCb(self,timer):
      self.publish_status()


  def publish_status(self):
    self.publish_apps_status()
    self.publish_app_status()

  def publish_apps_status(self):
    self.last_status_apps_msg = self.status_apps_msg
    self.status_apps_msg = self.getAppsStatusMsg()
    if not nepi_ros.is_shutdown():
      self.apps_status_pub.publish(self.status_apps_msg)
    self.save_cfg_if.saveConfig(do_param_updates = False) # Save config

  def getAppsStatusMsg(self):
    apps_dict = nepi_ros.get_param(self,"~apps_dict",self.init_apps_dict)
    self.apps_ordered_list = nepi_apps.getAppsOrderedList(apps_dict)
    self.apps_group_list = nepi_apps.getAppsGroupList(apps_dict)
    self.apps_active_list = nepi_apps.getAppsActiveOrderedList(apps_dict)
    status_apps_msg = AppsStatus()
    status_apps_msg.apps_path = self.apps_folder
    status_apps_msg.apps_ordered_list = self.apps_ordered_list
    apps_group_list = []
    for app_name in self.apps_ordered_list:
      apps_group_list.append(apps_dict[app_name]['APP_DICT']['group_name'])
    status_apps_msg.apps_group_list = apps_group_list
    status_apps_msg.apps_active_list = self.apps_active_list
    status_apps_msg.apps_install_path = self.apps_install_folder
    status_apps_msg.apps_install_list = self.apps_install_files
    status_apps_msg.apps_rui_list = nepi_apps.getAppsRuiActiveList(apps_dict)
    status_apps_msg.app_backup_path = self.apps_install_folder
    status_apps_msg.backup_removed_apps = nepi_ros.get_param(self,"~backup_enabled",self.init_backup_enabled)
    status_apps_msg.selected_app = self.selected_app
    return status_apps_msg

  
  def publish_app_status(self):
    self.status_app_msg = self.getAppStatusMsg()
    if not nepi_ros.is_shutdown():
      self.app_status_pub.publish(self.status_app_msg)


  def getAppStatusMsg(self):
    apps_dict = nepi_ros.get_param(self,"~apps_dict",self.init_apps_dict)
    app_name = self.selected_app
    status_app_msg = AppStatus()
    status_app_msg.name = app_name
    if app_name in apps_dict.keys() and app_name != 'NONE':
      app = apps_dict[app_name]
      try:
        status_app_msg.pkg_name = app['APP_DICT']['pkg_name']
        status_app_msg.pkg_name = app['APP_DICT']['group_name']
        status_app_msg.description = app['APP_DICT']['description']
        status_app_msg.node_name = app['APP_DICT']['node_name']
        status_app_msg.app_file = app['APP_DICT']['app_file']
        status_app_msg.app_path = app['APP_DICT']['app_path']   
        status_app_msg.rui_files_list = app['RUI_DICT']['rui_files']
        status_app_msg.rui_main_file = app['RUI_DICT']['rui_main_file']
        status_app_msg.rui_main_class = app['RUI_DICT']['rui_main_class']  
        status_app_msg.rui_menu_name = app['RUI_DICT']['rui_menu_name']
        status_app_msg.active_state  = app['active']
        status_app_msg.order  = app['order']
        status_app_msg.msg_str = app['msg']
      except Exception as e:
        nepi_msg.publishMsgInfo(self,"Failed to create app status message: " + str(e))

    return status_app_msg

  

  ###################
  ## Apps Mgr Callbacks

  def enableAllCb(self,msg):
    apps_dict = nepi_ros.get_param(self,"~apps_dict",self.init_apps_dict)
    apps_dict = nepi_apps.activateAllApps(apps_dict)
    nepi_ros.set_param(self,"~apps_dict",apps_dict)
    self.publish_status()

  def disableAllCb(self,msg):
    apps_dict = nepi_ros.get_param(self,"~apps_dict",self.init_apps_dict)
    apps_dict = nepi_apps.disableAllApps(apps_dict)
    nepi_ros.set_param(self,"~apps_dict",apps_dict)
    self.publish_status()


  def selectAppCb(self,msg):
    nepi_msg.publishMsgInfo(self,str(msg))
    app_name = msg.data
    apps_dict = nepi_ros.get_param(self,"~apps_dict",self.init_apps_dict)
    if app_name in apps_dict.keys() or app_name == "NONE":
      self.selected_app = app_name
    self.publish_status()

  def updateStateCb(self,msg):
    nepi_msg.publishMsgInfo(self,str(msg))
    app_name = msg.name
    new_active_state = msg.active_state
    apps_dict = nepi_ros.get_param(self,"~apps_dict",self.init_apps_dict)
    if app_name in apps_dict.keys():
      app = apps_dict[app_name]
      active_state = app['active']
    if new_active_state != active_state:
      if new_active_state == True:
        apps_dict = nepi_apps.activateApp(app_name,apps_dict)
      else:
        apps_dict = nepi_apps.disableApp(app_name,apps_dict)
    nepi_ros.set_param(self,"~apps_dict",apps_dict)
    self.publish_status()


  def updateOrderCb(self,msg):
    nepi_msg.publishMsgInfo(self,str(msg))
    app_name = msg.name
    move_cmd = msg.move_cmd
    moveFunction = self.getOrderUpdateFunction(move_cmd)
    apps_dict = nepi_ros.get_param(self,"~apps_dict",self.init_apps_dict)
    if app_name in apps_dict.keys():
      apps_dict = moveFunction(app_name,apps_dict)
    nepi_ros.set_param(self,"~apps_dict",apps_dict)
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
    nepi_msg.publishMsgInfo(self,str(msg))
    pkg_name = msg.data
    apps_dict = nepi_ros.get_param(self,"~apps_dict",self.init_apps_dict)
    if pkg_name in self.apps_install_files:
     [success,apps_dict]  = nepi_apps.installAppPkg(pkg_name,apps_dict,self.apps_install_folder,self.apps_folder)
    nepi_ros.set_param(self,"~apps_dict",apps_dict)
    self.publish_status()

  def removeAppCb(self,msg):
    nepi_msg.publishMsgInfo(self,str(msg))
    app_name = msg.data
    apps_dict = nepi_ros.get_param(self,"~apps_dict",self.init_apps_dict)
    backup_folder = None
    backup_enabled = nepi_ros.get_param(self,"~backup_enabled",self.init_backup_enabled)
    if backup_enabled:
      backup_folder = self.apps_install_folder
    if app_name in apps_dict:
      [success,apps_dict] = nepi_apps.removeApp(app_name,apps_dict,backup_folder)
    nepi_ros.set_param(self,"~apps_dict",apps_dict)
    self.publish_status()


  def enableBackupCb(self,msg):
    nepi_msg.publishMsgInfo(self,str(msg))
    backup_enabled = msg.data
    nepi_ros.set_param(self,"~backup_enabled",backup_enabled)
    self.publish_status()




  #######################
  # Misc Utility Function

  def getAvailableDevPaths(self):
    dev_path_list = glob.glob('/dev/*')
    return dev_path_list


  #######################
  # Node Cleanup Function
  
  def cleanup_actions(self):
    nepi_msg.publishMsgInfo(self,"Shutting down: Executing script cleanup actions")


#########################################
# Main
#########################################
if __name__ == '__main__':
  NepiAppsMgr()







