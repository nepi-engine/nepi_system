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
from nepi_edge_sdk_base import nepi_nex

from std_msgs.msg import Empty, String, Int32, Bool, Header
from nepi_ros_interfaces.msg import DriversStatus, DriverStatus, UpdateState, UpdateOrder, UpdateOption #, DriverUpdateMsg
from nepi_ros_interfaces.srv import SystemStorageFolderQuery, SystemStorageFolderQueryResponse

from nepi_edge_sdk_base.save_cfg_if import SaveCfgIF


DRIVERS_FALLBACK_FOLDER = '/opt/nepi/ros/lib/nepi_drivers'
DRIVERS_INSTALL_FALLBACK_FOLDER = '/mnt/nepi_storage/nepi_src/nepi_drivers'

#########################################

#########################################
# Node Class
#########################################

class NepiDriversMgr(object):

  DRIVER_UPDATE_CHECK_INTERVAL = 5
  save_cfg_if = None
  drivers_folder = ''
  drivers_files = []
  drivers_ordered_list = []
  drivers_active_list = []
  drivers_install_folder = ''
  drivers_install_files = []
  drivers_install_list = []

  status_drivers_msg = DriversStatus()
  last_status_drivers_msg = DriversStatus()

  discovery_active_dict = dict()

  selected_driver = "NONE"
  active_paths_list = [] 
  imported_classes_dict = dict()

  base_namespace = ""

  init_backup_enabled = True

  #################################################################
  DEFAULT_NODE_NAME = "drivers_mgr"
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
    # Get driver folder paths
    self.drivers_folder = DRIVERS_FALLBACK_FOLDER
    #self.publishMsg("Driver folder set to " + self.drivers_folder)
    self.drivers_files = nepi_nex.getDriverFilesList(self.drivers_folder)
    #self.publishMsg("Driver folder files " + str(self.drivers_files))
    # Get Install Drivers Folder

    NEPI_BASE_NAMESPACE = nepi_ros.get_base_namespace()
    get_folder_name_service = NEPI_BASE_NAMESPACE + 'system_storage_folder_query'
    self.publishMsg("Waiting for system storage folder query service " + get_folder_name_service)
    rospy.wait_for_service(get_folder_name_service)
    self.publishMsg("Calling system storage folder query service " + get_folder_name_service)
    try:
        self.publishMsg("Getting storage folder query service " + get_folder_name_service)
        folder_query_service = rospy.ServiceProxy(get_folder_name_service, SystemStorageFolderQuery)
        response = folder_query_service('nepi_src/nepi_drivers')
        self.publishMsg("Got storage folder path" + response.folder_path)
        time.sleep(1)
        self.save_data_root_directory = response.folder_path
    except Exception as e:
        self.save_data_root_directory = DRIVERS_INSTALL_FALLBACK_FOLDER
        rospy.logwarn(self.node_name + "Failed to obtain system data folder, falling back to: " + DRIVERS_INSTALL_FALLBACK_FOLDER + " " + str(e))
    if os.path.exists(self.save_data_root_directory):
        self.drivers_install_folder = self.save_data_root_directory
    else:
        self.save_data_root_directory = ""

    self.publishMsg("Driver folder set to " + self.drivers_install_folder)
    self.drivers_install_files = nepi_nex.getDriverPackagesList(self.drivers_install_folder)
    self.publishMsg("Driver install packages folder files " + str(self.drivers_install_files))

    # Setup Node Status Publisher
    self.drivers_status_pub = rospy.Publisher("~status", DriversStatus, queue_size=1, latch=True)
    self.driver_status_pub = rospy.Publisher("~status_driver", DriverStatus, queue_size=1, latch=True)

    # Setup message publisher and init param server
    self.publishMsg("Starting Initialization Processes")


    self.initParamServerValues(do_updates = False)

    self.save_cfg_if = SaveCfgIF(updateParamsCallback=self.initParamServerValues, 
                                 paramsModifiedCallback=self.updateFromParamServer)
        
 
    ## Mgr ROS Setup 
    mgr_reset_sub = rospy.Subscriber('~factory_reset', Empty, self.resetMgrCb, queue_size = 10)
    mgr_reset_sub = rospy.Subscriber('~refresh_drivers', Empty, self.refreshCb, queue_size = 10)


    # Drivers Management Scubscirbers
    rospy.Subscriber('~select_driver', String, self.selectDriverCb)
    rospy.Subscriber('~update_state', UpdateState, self.updateStateCb)
    rospy.Subscriber('~update_order', UpdateOrder, self.updateOrderCb)
    rospy.Subscriber('~update_option_1', UpdateOption, self.updateOption1Cb)
    rospy.Subscriber('~update_option_2', UpdateOption, self.updateOption2Cb)
    #rospy.Subscriber('~select_driver_status', DriverUpdateMsg, self.updateMsgCb)

    rospy.Subscriber('~install_driver_pkg', String, self.installDriverPkgCb)
    rospy.Subscriber('~backup_on_remeove', Bool, self.enableBackupCb)
    rospy.Subscriber('~remove_driver', String, self.removeDriverCb)


    # Setup a driver folder timed check
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
    nex_database = rospy.get_param("~nex_database",self.init_nex_database)
    self.publishMsg('',level)
    self.publishMsg('*******************')
    self.publishMsg('Printing Nex Driver Dictionary')
    for nex_name in nex_database.keys():
      nex_dict = nex_database[nex_name]
      self.publishMsg('',level)
      self.publishMsg(nex_name)
      self.publishMsg(str(nex_dict))

  def publish_status(self):
    self.publish_drivers_status()
    self.publish_driver_status()


  def publish_drivers_status(self):
    self.last_status_drivers_msg = self.status_drivers_msg
    self.status_drivers_msg = self.getDriversStatusMsg()
    if not rospy.is_shutdown():
      self.drivers_status_pub.publish(self.status_drivers_msg)
    self.save_cfg_if.saveConfig(do_param_updates = False) # Save config after initialization for next time

  def getDriversStatusMsg(self):
    nex_database = rospy.get_param("~nex_database",self.init_nex_database)
    self.drivers_ordered_list = nepi_nex.getDriversOrderedList(nex_database)
    self.drivers_active_list = nepi_nex.getDriversActiveOrderedList(nex_database)
    status_drivers_msg = DriversStatus()
    status_drivers_msg.drivers_path = self.drivers_folder
    status_drivers_msg.drivers_ordered_list = str(self.drivers_ordered_list)
    status_drivers_msg.drivers_active_list = str(self.drivers_active_list)
    status_drivers_msg.drivers_install_path = self.drivers_install_folder
    status_drivers_msg.drivers_install_list = str(self.drivers_install_files)
    status_drivers_msg.driver_backup_path = self.drivers_install_folder
    status_drivers_msg.backup_removed_drivers = rospy.get_param("~backup_enabled",self.init_backup_enabled)
    status_drivers_msg.selected_driver = self.selected_driver
    return status_drivers_msg

  
  def publish_driver_status(self):
    self.status_driver_msg = self.getDriverStatusMsg()
    if not rospy.is_shutdown():
      self.driver_status_pub.publish(self.status_driver_msg)


  def getDriverStatusMsg(self):
    nex_database = rospy.get_param("~nex_database",self.init_nex_database)
    driver_name = self.selected_driver
    status_driver_msg = DriverStatus()
    status_driver_msg.name = driver_name
    if driver_name in nex_database.keys() and driver_name != 'NONE':
      driver = nex_database[driver_name]
      status_driver_msg.group = driver['group']
      status_driver_msg.group_id  = driver['group_id']
      status_driver_msg.interfaces  = str(driver['driver_interfaces'])
      status_driver_msg.options_1_name  = str(driver['driver_options_1_name'])
      status_driver_msg.options_1  = str(driver['driver_options_1'])
      status_driver_msg.set_option_1  = driver['driver_set_option_1']
      status_driver_msg.options_2_name  = str(driver['driver_options_2_name'])
      status_driver_msg.options_2  = str(driver['driver_options_2'])
      status_driver_msg.set_option_2  = driver['driver_set_option_2']
      status_driver_msg.discovery = driver['discovery_method']
      status_driver_msg.other_users_list  = str(driver['users'])
      status_driver_msg.active_state  = driver['active']
      status_driver_msg.order  = driver['order']
      status_driver_msg.description = driver['description']
      status_driver_msg.msg_str = driver['msg']
    return status_driver_msg

  
  def checkAndUpdateCb(self,_):
    ###############################
    ## First update Database
    nex_database = rospy.get_param("~nex_database",self.init_nex_database)
    drivers_files = nepi_nex.getDriverFilesList(self.drivers_folder)
    self.drivers_install_files = nepi_nex.getDriverPackagesList(self.drivers_install_folder)
    need_update = self.drivers_files != drivers_files
    if need_update:
      self.publishMsg("Need to Update Nex Database")
      try:
        nex_database = rospy.get_param("~nex_database")
        nex_database = nepi_nex.updateDriversDict(self.drivers_folder,nex_database)
      except:
        nex_database = nepi_nex.getDriversDict(self.drivers_folder)
        nex_database = nepi_nex.setFactoryDriverOrder(nex_database)
        nex_database = nepi_nex.activateAllDrivers(nex_database)
      #self.printND()
    rospy.set_param("~nex_database",nex_database)
    self.drivers_files = drivers_files
    self.drivers_ordered_list = nepi_nex.getDriversOrderedList(nex_database)
    self.drivers_active_list = nepi_nex.getDriversActiveOrderedList(nex_database)
    ## Next process active driver processes


    ###############################
    # Get list of available device paths
    available_paths_list = self.getAvailableDevPaths()
    # Get list of active nodes
    warnings.filterwarnings('ignore', '.*unclosed.*', ) 
    node_namespace_list = nepi_ros.get_node_list()
    node_list = []
    for i in range(len(node_namespace_list)):
      node_list.append(node_namespace_list[i].split("/")[-1])

    ###############################installDriverPkgCb
    #self.publishMsg(node_list,level = "WARN")
    for nex_name in self.drivers_ordered_list:
      if nex_name in self.drivers_active_list:
        nex_dict = nex_database[nex_name]
        #self.publishMsg(nex_dict,level = "WARN")
        discovery_name = nex_dict['discovery_name']
        discovery_file = nex_dict['discovery_file_name']
        discovery_path = nex_dict['discovery_file_path']
        discovery_module = nex_dict['discovery_module_name']
        discovery_class_name = nex_dict['discovery_class_name']
        discovery_method = nex_dict['discovery_method']
        discovery_process = nex_dict['discovery_process']
        discovery_ids = nex_dict['discovery_ids']
        discovery_ignore_ids = nex_dict['discovery_ignore_ids']

        driver_name = nex_dict['driver_name']
        driver_file = nex_dict['driver_file_name']
        driver_path = nex_dict['driver_file_path']
        driver_module = nex_dict['driver_module_name']
        driver_class = nex_dict['driver_class_name']
        driver_interfaces = nex_dict['driver_interfaces']
        driver_options_1 = nex_dict['driver_options_1']
        driver_option_1 = nex_dict['driver_set_option_1']

        #rospy.logwarn("DRV_MGR: Processing discovery process for driver %s with method %s and process %s",nex_name,discovery_method,discovery_process)
        # Check Auto-Node processes
        if discovery_method == 'AUTO' and discovery_process == "LAUNCH":
          discovery_node_name = discovery_name.lower() + "_discovery"
          if discovery_node_name not in node_list:
            #Setup required param server nex_dict for discovery node
            dict_param_name = discovery_node_name + "/nex_dict"
            rospy.set_param(dict_param_name,nex_dict)
            #Try and launch node
            self.publishMsg("")
            self.publishMsg("Launching discovery process: " + discovery_node_name + " with nex_dict " + str(nex_dict))
            [success, msg, sub_process] = nepi_nex.launchDriverNode(discovery_file, discovery_node_name)
            if success:
              nex_database[nex_name]['msg'] = "Discovery process lanched"
              self.discovery_active_dict[nex_name]=dict()
              self.discovery_active_dict[nex_name]['process'] = "LAUNCH"
              self.discovery_active_dict[nex_name]['node_name'] = discovery_node_name
              self.discovery_active_dict[nex_name]['subprocess'] = sub_process
            else:
              nex_database[nex_name]['msg'] = msg
          else:
            nex_database[nex_name]['msg'] = "Discovery process running"

        # Run Auto-Run processes 
        if discovery_method == 'AUTO' and discovery_process == "RUN":
          pass # ToDo

        # Call Auto-Call processes 
        if discovery_method == 'AUTO' and discovery_process == "CALL":
          if discovery_class_name not in self.imported_classes_dict.keys():
            self.publishMsg("")
            self.publishMsg("Imported discovery class " + discovery_class_name + " for driver " + nex_name)
            [success, msg,imported_class] = nepi_nex.importDriverClass(discovery_file,discovery_path,discovery_module,discovery_class_name)
            if success:
              self.publishMsg("Instantiating discovery class " + discovery_class_name + " with nex_dict " + str(nex_dict))
              discovery_class = imported_class()
              self.imported_classes_dict[discovery_class_name] = discovery_class
              self.publishMsg("Instantiated discovery class " + discovery_class_name + " for driver " + nex_name)
            else: 
              self.publishMsg("Failed to import discovery class " + discovery_class_name + " for driver " + nex_name)
          else:
            #self.publishMsg("")
            #self.publishMsg("Calling discovery function for class: " + discovery_class_name + " for driver " + nex_name)
            discovery_class = self.imported_classes_dict[discovery_class_name]
            self.active_paths_list = discovery_class.discoveryFunction(available_paths_list, self.active_paths_list, self.base_namespace, nex_dict)
        #self.publishMsg("Active Path List " + str(self.active_paths_list))
        # Do Manual processes 
        if discovery_method == 'MANUAL':
          if 'SERIAL' in driver_interfaces:
            pass # ToDo
          if 'SERIALUSB' in driver_interfaces:
            pass # ToDo
          if 'USB' in driver_interfaces:
            pass # ToDo
          if 'IP' in driver_interfaces:
            pass # ToDo
        time.sleep(1)

      ################################    
      ## Check and purge disabled driver proccess that might be running
      # First check on running nodes
      purge_list = []
      for nex_name in self.drivers_ordered_list:
        if nex_name not in self.drivers_active_list and nex_name in self.discovery_active_dict.keys():
            node_name = self.discovery_active_dict[nex_name]['node_name']
            if node_name in node_list:
              process = self.discovery_active_dict[nex_name]['process']
              if process == "LAUNCH":
                sub_process = self.discovery_active_dict[nex_name]['subprocess']
                success = nepi_nex.killDriverNode(node_name,sub_process)
                if success:
                  purge_list.append(nex_name)
      # purge from active discovery dict
      for nex_name in purge_list:
        if nex_name in self.discovery_active_dict.keys():
          del self.discovery_active_dict[nex_name]
    # Publish Status
    self.publish_status()
    # And now that we are finished, start a timer for the next runDiscovery()
    nepi_ros.sleep(self.DRIVER_UPDATE_CHECK_INTERVAL,100)
    rospy.Timer(rospy.Duration(1), self.checkAndUpdateCb, oneshot=True)
   
  

  ###################
  ## Drivers Mgr Callbacks

  def selectDriverCb(self,msg):
    self.publishMsg(msg)
    driver_name = msg.data
    nex_database = rospy.get_param("~nex_database",self.init_nex_database)
    if driver_name in nex_database.keys() or driver_name == "NONE":
      self.selected_driver = driver_name
    self.publish_status()

  def updateStateCb(self,msg):
    self.publishMsg(msg)
    driver_name = msg.name
    new_active_state = msg.active_state
    nex_database = rospy.get_param("~nex_database",self.init_nex_database)
    if driver_name in nex_database.keys():
      driver = nex_database[driver_name]
      active_state = driver['active']
    if new_active_state != active_state:
      if new_active_state == True:
        nex_database = nepi_nex.activateDriver(driver_name,nex_database)
      else:
        nex_database = nepi_nex.disableDriver(driver_name,nex_database)
    rospy.set_param("~nex_database",nex_database)
    self.publish_status()


  def updateOrderCb(self,msg):
    self.publishMsg(msg)
    driver_name = msg.name
    move_cmd = msg.move_cmd
    moveFunction = self.getOrderUpdateFunction(move_cmd)
    nex_database = rospy.get_param("~nex_database",self.init_nex_database)
    if driver_name in nex_database.keys():
      nex_database = moveFunction(driver_name,nex_database)
    rospy.set_param("~nex_database",nex_database)
    self.publish_status()


  def getOrderUpdateFunction(self,move_cmd):
    if move_cmd == 'top':
      updateFunction = nepi_nex.moveDriverTop
    elif move_cmd == 'bottom':
      updateFunction = nepi_nex.moveDriverBottom
    elif move_cmd == 'up':
      updateFunction = nepi_nex.moveDriverUp
    elif move_cmd == 'down':
      updateFunction = nepi_nex.moveDriverDown
    else:
      updateFunction = self.moveDriverNone
    return updateFunction

  def moveDriverNone(self,driver_name,nex_database):
    return nex_database
    
  def updateMsgCb(self,msg):
    self.publishMsg(msg)
    driver_name = msg.name
    msg_data = msg.data
    nex_database = rospy.get_param("~nex_database",self.init_nex_database)
    if driver_name in nex_database.keys():
      nex_database[driver_name]['msg'] = msg_data
    rospy.set_param("~nex_database",nex_database)
    self.publish_status()

  def updateOption1Cb(self,msg):
    self.publishMsg(msg)
    driver_name = msg.name
    option = msg.option_str
    nex_database = rospy.get_param("~nex_database",self.init_nex_database)
    if driver_name in nex_database.keys():
      driver = nex_database[driver_name]
      options = driver['driver_options_1']
      if option in options:
        nex_database[driver_name]['driver_set_option_1'] = option
    rospy.set_param("~nex_database",nex_database)
    self.publish_status()

  def updateOption2Cb(self,msg):
    self.publishMsg(msg)
    driver_name = msg.name
    option = msg.option_str
    nex_database = rospy.get_param("~nex_database",self.init_nex_database)
    if driver_name in nex_database.keys():
      driver = nex_database[driver_name]
      options = driver['driver_options_2']
      if option in options:
        nex_database[driver_name]['driver_set_option_2'] = option
    rospy.set_param("~nex_database",nex_database)
    self.publish_status()

  def installDriverPkgCb(self,msg):
    self.publishMsg(msg)
    pkg_name = msg.data
    nex_database = rospy.get_param("~nex_database",self.init_nex_database)
    if pkg_name in self.drivers_install_files:
     [success,nex_database]  = nepi_nex.installDriverPkg(pkg_name,nex_database,self.drivers_install_folder,self.drivers_folder)
    rospy.set_param("~nex_database",nex_database)
    self.publish_status()

  def removeDriverCb(self,msg):
    self.publishMsg(msg)
    driver_name = msg.data
    nex_database = rospy.get_param("~nex_database",self.init_nex_database)
    backup_folder = None
    backup_enabled = rospy.get_param("~backup_enabled",self.init_backup_enabled)
    if backup_enabled:
      backup_folder = self.drivers_install_folder
    if driver_name in nex_database:
      [success,nex_database] = nepi_nex.removeDriver(driver_name,nex_database,backup_folder)
    rospy.set_param("~nex_database",nex_database)
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
    # reset drivers dict
    rospy.set_param("~backup_enabled",True)
    self.drivers_files = nepi_nex.getDriverFilesList(self.drivers_folder)
    self.drivers_install_files = nepi_nex.getDriverPackagesList(self.drivers_install_folder)
    nex_database = nepi_nex.getDriversgetDriversDict(self.drivers_folder)
    nex_database = nepi_nex.setFactoryDriverOrder(nex_database)
    nex_database = activateAllDrivers(nex_database)
    rospy.set_param("~nex_database",nex_database)
    self.publish_status()


  def refreshCb(self,msg):
    self.refresh()

  def refresh(self):
    # refresh drivers dict
    self.drivers_files = nepi_nex.getDriverFilesList(self.drivers_folder)
    self.drivers_install_files = nepi_nex.getDriverPackagesList(self.drivers_install_folder)
    nex_database = rospy.get_param("~nex_database",self.init_nex_database)
    nex_database = nepi_nex.refreshDriversDict(self.drivers_folder,nex_database)
    rospy.set_param("~nex_database",nex_database)
    self.publish_status()

  def saveConfigCb(self, msg):  # Just update Class init values. Saving done by Config IF system
    pass # Left empty for sim, Should update from param server

  def setCurrentAsDefault(self):
    rospy.set_param("~backup_enabled", True)
    self.initParamServerValues(do_updates = False)

  def updateFromParamServer(self):
    #rospy.logwarn("DRV_MGR: Debugging: param_dict = " + str(param_dict))
    #Run any functions that need updating on value change
    # Don't need to run any additional functionsinstallDriverPkgCb
    pass
    
  def initParamServerValues(self,do_updates = True):
      self.selected_driver = 'NONE'
      self.publishMsg("Setting init values to param values")
      self.init_backup_enabled = rospy.get_param("~backup_enabled", True)
      self.drivers_files = nepi_nex.getDriverFilesList(self.drivers_folder)
      self.drivers_install_files = nepi_nex.getDriverPackagesList(self.drivers_install_folder)
      #self.publishMsg(str(rospy.get_param("~nex_database")))
      try:
        nex_database = rospy.get_param("~nex_database")
        nex_database = nepi_nex.updateDriversDict(self.drivers_folder,nex_database)
        self.publishMsg("Got nex_database values from param server")
      except:
        nex_database = nepi_nex.getDriversDict(self.drivers_folder)
        nex_database = nepi_nex.setFactoryDriverOrder(nex_database)
        nex_database = nepi_nex.activateAllDrivers(nex_database)
        self.publishMsg("Got nex_database values from reading in file data")
      self.init_nex_database = nex_database
      rospy.set_param("~nex_database",nex_database)
      #self.printND()
      self.resetParamServer(do_updates)
      #self.printND()

  def resetParamServer(self,do_updates = True):
      rospy.set_param("~backup_enabled",self.init_backup_enabled)
      rospy.set_param("~nex_database",self.init_nex_database)
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
  NepiDriversMgr()







