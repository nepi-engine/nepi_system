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
from nepi_edge_sdk_base import nepi_msg
from nepi_edge_sdk_base import nepi_drv

from std_msgs.msg import Empty, String, Int32, Bool, Header
from nepi_ros_interfaces.msg import DriversStatus, DriverStatus, UpdateState, UpdateOrder, UpdateOption #, DriverUpdateMsg
from nepi_ros_interfaces.srv import SystemStorageFolderQuery, SystemStorageFolderQueryResponse
from nepi_ros_interfaces.srv import DriverStatusQuery, DriverStatusQueryResponse

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


  #######################
  ### Node Initialization
  DEFAULT_NODE_NAME = "drivers_mgr" # Can be overwitten by luanch command
  def __init__(self):
    #### APP NODE INIT SETUP ####
    nepi_ros.init_node(name= self.DEFAULT_NODE_NAME)
    self.node_name = nepi_ros.get_node_name()
    self.base_namespace = nepi_ros.get_base_namespace()
    nepi_msg.createMsgPublishers(self)
    nepi_msg.publishMsgInfo(self,"Starting Initialization Processes")
    ##############################

    # Get driver folder paths
    self.drivers_folder = DRIVERS_FALLBACK_FOLDER
    #nepi_msg.publishMsgInfo(self,"Driver folder set to " + self.drivers_folder)
    self.drivers_files = nepi_drv.getDriverFilesList(self.drivers_folder)
    #nepi_msg.publishMsgInfo(self,"Driver folder files " + str(self.drivers_files))
    # Get Install Drivers Folder

    NEPI_BASE_NAMESPACE = nepi_ros.get_base_namespace()
    get_folder_name_service = NEPI_BASE_NAMESPACE + 'system_storage_folder_query'
    nepi_msg.publishMsgInfo(self,"Waiting for system storage folder query service " + get_folder_name_service)
    rospy.wait_for_service(get_folder_name_service)
    nepi_msg.publishMsgInfo(self,"Calling system storage folder query service " + get_folder_name_service)
    try:
        nepi_msg.publishMsgInfo(self,"Getting storage folder query service " + get_folder_name_service)
        folder_query_service = rospy.ServiceProxy(get_folder_name_service, SystemStorageFolderQuery)
        response = folder_query_service('nepi_src/nepi_drivers')
        nepi_msg.publishMsgInfo(self,"Got storage folder path" + response.folder_path)
        time.sleep(1)
        self.save_data_root_directory = response.folder_path
    except Exception as e:
        self.save_data_root_directory = DRIVERS_INSTALL_FALLBACK_FOLDER
        nepi_msg.publishMsgWarn(self,"Failed to obtain system data folder, falling back to: " + DRIVERS_INSTALL_FALLBACK_FOLDER + " " + str(e))
    if os.path.exists(self.save_data_root_directory):
        self.drivers_install_folder = self.save_data_root_directory
    else:
        self.save_data_root_directory = ""

    nepi_msg.publishMsgInfo(self,"Driver folder set to " + self.drivers_install_folder)
    self.drivers_install_files = nepi_drv.getDriverPackagesList(self.drivers_install_folder)
    nepi_msg.publishMsgInfo(self,"Driver install packages folder files " + str(self.drivers_install_files))

    # Setup Node Status Publisher
    self.drivers_status_pub = rospy.Publisher("~status", DriversStatus, queue_size=1, latch=True)
    self.driver_status_pub = rospy.Publisher("~status_driver", DriverStatus, queue_size=1, latch=True)

    # Setup message publisher and init param server
    nepi_msg.publishMsgInfo(self,"Starting Initialization Processes")


    self.initParamServerValues(do_updates = False)

    self.save_cfg_if = SaveCfgIF(updateParamsCallback=self.initParamServerValues, 
                                 paramsModifiedCallback=self.updateFromParamServer)
        
 
    ## Mgr ROS Setup 
    mgr_reset_sub = rospy.Subscriber('~factory_reset', Empty, self.resetMgrCb, queue_size = 10)
    mgr_reset_sub = rospy.Subscriber('~refresh_drivers', Empty, self.refreshCb, queue_size = 10)


    # Drivers Management Scubscirbers
    rospy.Subscriber('~enable_all_drivers', Empty, self.enableAllCb, queue_size = 10)
    rospy.Subscriber('~disable_all_drivers', Empty, self.disableAllCb, queue_size = 10)
    rospy.Subscriber('~select_driver', String, self.selectDriverCb)
    rospy.Subscriber('~update_state', UpdateState, self.updateStateCb)
    rospy.Subscriber('~update_order', UpdateOrder, self.updateOrderCb)
    rospy.Subscriber('~update_option_1', UpdateOption, self.updateOption1Cb)
    rospy.Subscriber('~update_option_2', UpdateOption, self.updateOption2Cb)
    #rospy.Subscriber('~select_driver_status', DriverUpdateMsg, self.updateMsgCb)

    rospy.Subscriber('~install_driver_pkg', String, self.installDriverPkgCb)
    rospy.Subscriber('~backup_on_remeove', Bool, self.enableBackupCb)
    rospy.Subscriber('~remove_driver', String, self.removeDriverCb)

    # Start capabilities services
    rospy.Service('~driver_status_query', DriverStatusQuery, self.driverStatusService)

    # Setup a driver folder timed check
    nepi_ros.timer(nepi_ros.duration(1), self.checkAndUpdateCb, oneshot=True)
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


  def driverStatusService(self,request):
    driver_name = request.name
    response = self.getDriverStatusServiceMsg(driver_name)
    return response

  def getDriverStatusServiceMsg(self,driver_name):
    drvs_dict = nepi_ros.get_param(self,"~drvs_dict",self.init_drvs_dict)
    status_driver_msg = DriverStatusQueryResponse()
    status_driver_msg.name = driver_name
    if driver_name in drvs_dict.keys() and driver_name != 'NONE':
      drv_dict = drvs_dict[driver_name]
      status_driver_msg.group = drv_dict['NODE_DICT']['group']
      status_driver_msg.group_id  = drv_dict['NODE_DICT']['group_id']
      status_driver_msg.interfaces  = drv_dict['DISCOVERY_DICT']['interfaces']
      status_driver_msg.options_1_name  = drv_dict['DISCOVERY_DICT']['option_1_dict']['name']
      status_driver_msg.options_1  = drv_dict['DISCOVERY_DICT']['option_1_dict']['options']
      status_driver_msg.set_option_1  = drv_dict['DISCOVERY_DICT']['option_1_dict']['set_val']
      status_driver_msg.options_2_name  = drv_dict['DISCOVERY_DICT']['option_2_dict']['name']
      status_driver_msg.options_2  = drv_dict['DISCOVERY_DICT']['option_2_dict']['options']
      status_driver_msg.set_option_2  = drv_dict['DISCOVERY_DICT']['option_2_dict']['set_val']
      status_driver_msg.discovery = drv_dict['DISCOVERY_DICT']['method']
      status_driver_msg.other_users_list  = drv_dict['users']
      status_driver_msg.active_state  = drv_dict['active']
      status_driver_msg.order  = drv_dict['order']
      status_driver_msg.description = drv_dict['NODE_DICT']['description']
      status_driver_msg.msg_str = drv_dict['msg']
    return status_driver_msg



        
  # ln = sys._getframe().f_lineno ; 
  def printND(self):
    drvs_dict = nepi_ros.get_param(self,"~drvs_dict",self.init_drvs_dict)
    nepi_msg.publishMsgInfo(self,'')
    nepi_msg.publishMsgInfo(self,'*******************')
    nepi_msg.publishMsgInfo(self,'Printing Drv Driver Dictionary')
    for drv_name in drvs_dict.keys():
      drv_dict = drvs_dict[drv_name]
      nepi_msg.publishMsgInfo(self,'')
      nepi_msg.publishMsgInfo(self,drv_name)
      nepi_msg.publishMsgInfo(self,str(drv_dict))

  def publish_status(self):
    self.publish_drivers_status()
    self.publish_driver_status()


  def publish_drivers_status(self):
    self.last_status_drivers_msg = self.status_drivers_msg
    self.status_drivers_msg = self.getDriversStatusMsg()
    if not nepi_ros.is_shutdown():
      #nepi_msg.publishMsgWarn(self,"DriversStatus: " + str(self.status_drivers_msg))
      self.drivers_status_pub.publish(self.status_drivers_msg)
    self.save_cfg_if.saveConfig(do_param_updates = False) # Save config after initialization for drvt time

  def getDriversStatusMsg(self):
    drvs_dict = nepi_ros.get_param(self,"~drvs_dict",self.init_drvs_dict)
    self.drivers_ordered_list = nepi_drv.getDriversOrderedList(drvs_dict)
    self.drivers_active_list = nepi_drv.getDriversActiveOrderedList(drvs_dict)
    status_drivers_msg = DriversStatus()
    status_drivers_msg.drivers_path = self.drivers_folder
    status_drivers_msg.drivers_ordered_list = self.drivers_ordered_list
    status_drivers_msg.drivers_active_list = self.drivers_active_list
    status_drivers_msg.drivers_install_path = self.drivers_install_folder
    status_drivers_msg.drivers_install_list = self.drivers_install_files
    status_drivers_msg.driver_backup_path = self.drivers_install_folder
    status_drivers_msg.backup_removed_drivers = nepi_ros.get_param(self,"~backup_enabled",self.init_backup_enabled)
    status_drivers_msg.selected_driver = self.selected_driver
    return status_drivers_msg

  
  def publish_driver_status(self):
    self.status_driver_msg = self.getDriverStatusMsg()
    if not nepi_ros.is_shutdown():
      #nepi_msg.publishMsgWarn(self,"DriverStatus: " + str(self.status_driver_msg))
      self.driver_status_pub.publish(self.status_driver_msg)


  def getDriverStatusMsg(self):
    drvs_dict = nepi_ros.get_param(self,"~drvs_dict",self.init_drvs_dict)
    driver_name = self.selected_driver
    status_driver_msg = DriverStatus()
    status_driver_msg.name = driver_name
    if driver_name in drvs_dict.keys() and driver_name != 'NONE':
      drv_dict = drvs_dict[driver_name]
      status_driver_msg.group = drv_dict['NODE_DICT']['group']
      status_driver_msg.group_id  = drv_dict['NODE_DICT']['group_id']
      status_driver_msg.interfaces  = drv_dict['DISCOVERY_DICT']['interfaces']
      status_driver_msg.options_1_name  = drv_dict['DISCOVERY_DICT']['option_1_dict']['name']
      status_driver_msg.options_1  = drv_dict['DISCOVERY_DICT']['option_1_dict']['options']
      status_driver_msg.set_option_1  = drv_dict['DISCOVERY_DICT']['option_1_dict']['set_val']
      status_driver_msg.options_2_name  = drv_dict['DISCOVERY_DICT']['option_2_dict']['name']
      status_driver_msg.options_2  = drv_dict['DISCOVERY_DICT']['option_2_dict']['options']
      status_driver_msg.set_option_2  = drv_dict['DISCOVERY_DICT']['option_2_dict']['set_val']
      status_driver_msg.discovery = drv_dict['DISCOVERY_DICT']['method']
      status_driver_msg.other_users_list  = drv_dict['users']
      status_driver_msg.active_state  = drv_dict['active']
      status_driver_msg.order  = drv_dict['order']
      status_driver_msg.description = drv_dict['NODE_DICT']['description']
      status_driver_msg.msg_str = drv_dict['msg']
    return status_driver_msg

  
  def checkAndUpdateCb(self,_):
    ###############################
    ## First update Database
    drvs_dict = nepi_ros.get_param(self,"~drvs_dict",self.init_drvs_dict)
    drivers_files = nepi_drv.getDriverFilesList(self.drivers_folder)
    self.drivers_install_files = nepi_drv.getDriverPackagesList(self.drivers_install_folder)
    need_update = self.drivers_files != drivers_files
    if need_update:
      nepi_msg.publishMsgInfo(self,"Need to Update Drv Database")
      none_dict = dict(NoneDict = "None")
      drvs_dict = nepi_ros.get_param(self,"~drvs_dict",none_dict)
      if 'NoneDict' not in drvs_dict.keys():
        drvs_dict = nepi_drv.updateDriversDict(self.drivers_folder,drvs_dict)
        nepi_msg.publishMsgInfo(self,"Got drvs_dict values from param server")
      else:
        drvs_dict = nepi_drv.getDriversDict(self.drivers_folder)
        drvs_dict = nepi_drv.setFactoryDriverOrder(drvs_dict)
        drvs_dict = nepi_drv.activateAllDrivers(drvs_dict)
      #self.printND()
    nepi_ros.set_param(self,"~drvs_dict",drvs_dict)
    self.drivers_files = drivers_files
    self.drivers_ordered_list = nepi_drv.getDriversOrderedList(drvs_dict)
    self.drivers_active_list = nepi_drv.getDriversActiveOrderedList(drvs_dict)
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
    #nepi_msg.publishMsgInfo(self,str(node_list),level = "WARN")
    for drv_name in self.drivers_ordered_list:
      if drv_name in self.drivers_active_list:
        drv_dict = drvs_dict[drv_name]
        #self.publishMsgWarn(str(drv_dict))
        if drv_dict['NODE_DICT']['discovery_pkg_name'] != "None":
          discovery_path = drv_dict['path']
          discovery_name = drv_dict['DISCOVERY_DICT']['pkg_name']
          discovery_file = drv_dict['DISCOVERY_DICT']['file_name']
          discovery_module = drv_dict['DISCOVERY_DICT']['module_name']
          discovery_class_name = drv_dict['DISCOVERY_DICT']['class_name']
          discovery_interfaces = drv_dict['DISCOVERY_DICT']['interfaces']
          discovery_method = drv_dict['DISCOVERY_DICT']['method']
          discovery_process = drv_dict['DISCOVERY_DICT']['process']

          #nepi_msg.publishMsgWarn(self,"Processing discovery process for driver %s with method %s and process %s",drv_name,discovery_method,discovery_process)
          # Check Auto-Node processes
          if discovery_method == 'AUTO' and discovery_process == "LAUNCH":
            discovery_node_name = discovery_name.lower() + "_discovery"
            if discovery_node_name not in node_list:
              #Setup required param server drv_dict for discovery node
              dict_param_name = discovery_node_name + "/drv_dict"
              nepi_ros.set_param(self,dict_param_name,drv_dict)
              #Try and launch node
              nepi_msg.publishMsgInfo(self,"")
              nepi_msg.publishMsgInfo(self,"Launching discovery process: " + discovery_node_name + " with drv_dict " + str(drv_dict))
              [success, msg, sub_process] = nepi_drv.launchDriverNode(discovery_file, discovery_node_name)
              if success:
                drvs_dict[drv_name]['msg'] = "Discovery process lanched"
                self.discovery_active_dict[drv_name]=dict()
                self.discovery_active_dict[drv_name]['process'] = "LAUNCH"
                self.discovery_active_dict[drv_name]['node_name'] = discovery_node_name
                self.discovery_active_dict[drv_name]['subprocess'] = sub_process
              else:
                drvs_dict[drv_name]['msg'] = msg
            else:
              drvs_dict[drv_name]['msg'] = "Discovery process running"

          # Run Auto-Run processes 
          if discovery_method == 'AUTO' and discovery_process == "RUN":
            pass # ToDo

          # Call Auto-Call processes 
          if discovery_method == 'AUTO' and discovery_process == "CALL":
            if discovery_class_name not in self.imported_classes_dict.keys():
              nepi_msg.publishMsgInfo(self,"")
              nepi_msg.publishMsgInfo(self,"Imported discovery class " + discovery_class_name + " for driver " + drv_name)
              [success, msg,imported_class] = nepi_drv.importDriverClass(discovery_file,discovery_path,discovery_module,discovery_class_name)
              if success:
                nepi_msg.publishMsgInfo(self,"Instantiating discovery class " + discovery_class_name + " with drv_dict " + str(drv_dict))
                discovery_class = imported_class()
                self.imported_classes_dict[discovery_class_name] = discovery_class
                nepi_msg.publishMsgInfo(self,"Instantiated discovery class " + discovery_class_name + " for driver " + drv_name)
              else: 
                nepi_msg.publishMsgInfo(self,"Failed to import discovery class " + discovery_class_name + " for driver " + drv_name)
            else:
              #nepi_msg.publishMsgInfo(self,"")
              #nepi_msg.publishMsgInfo(self,"Calling discovery function for class: " + discovery_class_name + " for driver " + drv_name)
              discovery_class = self.imported_classes_dict[discovery_class_name]
              self.active_paths_list = discovery_class.discoveryFunction(available_paths_list, self.active_paths_list, self.base_namespace, drv_dict)
          #nepi_msg.publishMsgInfo(self,"Active Path List " + str(self.active_paths_list))
          # Do Manual processes 
          if discovery_method == 'MANUAL':
            if 'SERIAL' in discovery_interfaces:
              pass # ToDo
            if 'SERIALUSB' in discovery_interfaces:
              pass # ToDo
            if 'USB' in discovery_interfaces:
              pass # ToDo
            if 'IP' in discovery_interfaces:
              pass # ToDo

      ################################    
      ## Check and purge disabled driver proccess that might be running
      # First check on running nodes
      purge_list = []
      for drv_name in self.drivers_ordered_list:
        if drv_name not in self.drivers_active_list and drv_name in self.discovery_active_dict.keys():
            node_name = self.discovery_active_dict[drv_name]['node_name']
            if node_name in node_list:
              process = self.discovery_active_dict[drv_name]['process']
              if process == "LAUNCH":
                sub_process = self.discovery_active_dict[drv_name]['subprocess']
                success = nepi_drv.killDriverNode(node_name,sub_process)
                if success:
                  purge_list.append(drv_name)
      # purge from active discovery dict
      for drv_name in purge_list:
        if drv_name in self.discovery_active_dict.keys():
          del self.discovery_active_dict[drv_name]
    # Publish Status
    self.publish_status()
    # And now that we are finished, start a timer for the drvt runDiscovery()
    nepi_ros.sleep(self.DRIVER_UPDATE_CHECK_INTERVAL,100)
    nepi_ros.timer(nepi_ros.duration(1), self.checkAndUpdateCb, oneshot=True)
   
  

  ###################
  ## Drivers Mgr Callbacks
  def enableAllCb(self,msg):
    drvs_dict = nepi_ros.get_param(self,"~drvs_dict",self.init_drvs_dict)
    drvs_dict = nepi_drv.activateAllDrivers(drvs_dict)
    nepi_ros.set_param(self,"~drvs_dict",drvs_dict)
    self.publish_status()

  def disableAllCb(self,msg):
    drvs_dict = nepi_ros.get_param(self,"~drvs_dict",self.init_drvs_dict)
    drvs_dict = nepi_drv.disableAllDrivers(drvs_dict)
    nepi_ros.set_param(self,"~drvs_dict",drvs_dict)
    self.publish_status()


  def selectDriverCb(self,msg):
    nepi_msg.publishMsgInfo(self,str(msg))
    driver_name = msg.data
    drvs_dict = nepi_ros.get_param(self,"~drvs_dict",self.init_drvs_dict)
    if driver_name in drvs_dict.keys() or driver_name == "NONE":
      self.selected_driver = driver_name
    self.publish_status()

  def updateStateCb(self,msg):
    nepi_msg.publishMsgInfo(self,str(msg))
    driver_name = msg.name
    new_active_state = msg.active_state
    drvs_dict = nepi_ros.get_param(self,"~drvs_dict",self.init_drvs_dict)
    if driver_name in drvs_dict.keys():
      driver = drvs_dict[driver_name]
      active_state = driver['active']
    if new_active_state != active_state:
      if new_active_state == True:
        drvs_dict = nepi_drv.activateDriver(driver_name,drvs_dict)
      else:
        drvs_dict = nepi_drv.disableDriver(driver_name,drvs_dict)
    nepi_ros.set_param(self,"~drvs_dict",drvs_dict)
    self.publish_status()


  def updateOrderCb(self,msg):
    nepi_msg.publishMsgInfo(self,str(msg))
    driver_name = msg.name
    move_cmd = msg.move_cmd
    moveFunction = self.getOrderUpdateFunction(move_cmd)
    drvs_dict = nepi_ros.get_param(self,"~drvs_dict",self.init_drvs_dict)
    if driver_name in drvs_dict.keys():
      drvs_dict = moveFunction(driver_name,drvs_dict)
    nepi_ros.set_param(self,"~drvs_dict",drvs_dict)
    self.publish_status()


  def getOrderUpdateFunction(self,move_cmd):
    if move_cmd == 'top':
      updateFunction = nepi_drv.moveDriverTop
    elif move_cmd == 'bottom':
      updateFunction = nepi_drv.moveDriverBottom
    elif move_cmd == 'up':
      updateFunction = nepi_drv.moveDriverUp
    elif move_cmd == 'down':
      updateFunction = nepi_drv.moveDriverDown
    else:
      updateFunction = self.moveDriverNone
    return updateFunction

  def moveDriverNone(self,driver_name,drvs_dict):
    return drvs_dict
    
  def updateMsgCb(self,msg):
    nepi_msg.publishMsgInfo(self,str(msg))
    driver_name = msg.name
    msg_data = msg.data
    drvs_dict = nepi_ros.get_param(self,"~drvs_dict",self.init_drvs_dict)
    if driver_name in drvs_dict.keys():
      drvs_dict[driver_name]['msg'] = msg_data
    nepi_ros.set_param(self,"~drvs_dict",drvs_dict)
    self.publish_status()

  def updateOption1Cb(self,msg):
    nepi_msg.publishMsgInfo(self,str(msg))
    driver_name = msg.name
    option = msg.option_str
    drvs_dict = nepi_ros.get_param(self,"~drvs_dict",self.init_drvs_dict)
    if driver_name in drvs_dict.keys():
      driver = drvs_dict[driver_name]
      options = driver['DISCOVERY_DICT']['option_1_dict']['options']
      if option in options:
        drvs_dict[driver_name]['DISCOVERY_DICT']['option_1_dict']['set_val'] = option
    nepi_ros.set_param(self,"~drvs_dict",drvs_dict)
    self.publish_status()

  def updateOption2Cb(self,msg):
    nepi_msg.publishMsgInfo(self,str(msg))
    driver_name = msg.name
    option = msg.option_str
    drvs_dict = nepi_ros.get_param(self,"~drvs_dict",self.init_drvs_dict)
    if driver_name in drvs_dict.keys():
      driver = drvs_dict[driver_name]
      options = driver['DISCOVERY_DICT']['option_2_dict']['options']
      if option in options:
        drvs_dict[driver_name]['DISCOVERY_DICT']['option_2_dict']['set_val'] = option
    nepi_ros.set_param(self,"~drvs_dict",drvs_dict)
    self.publish_status()

  def installDriverPkgCb(self,msg):
    nepi_msg.publishMsgInfo(self,str(msg))
    pkg_name = msg.data
    drvs_dict = nepi_ros.get_param(self,"~drvs_dict",self.init_drvs_dict)
    if pkg_name in self.drivers_install_files:
     [success,drvs_dict]  = nepi_drv.installDriverPkg(pkg_name,drvs_dict,self.drivers_install_folder,self.drivers_folder)
    nepi_ros.set_param(self,"~drvs_dict",drvs_dict)
    self.publish_status()

  def removeDriverCb(self,msg):
    nepi_msg.publishMsgInfo(self,str(msg))
    driver_name = msg.data
    drvs_dict = nepi_ros.get_param(self,"~drvs_dict",self.init_drvs_dict)
    backup_folder = None
    backup_enabled = nepi_ros.get_param(self,"~backup_enabled",self.init_backup_enabled)
    if backup_enabled:
      backup_folder = self.drivers_install_folder
    if driver_name in drvs_dict:
      [success,drvs_dict] = nepi_drv.removeDriver(driver_name,drvs_dict,backup_folder)
    nepi_ros.set_param(self,"~drvs_dict",drvs_dict)
    self.publish_status()


  def enableBackupCb(self,msg):
    nepi_msg.publishMsgInfo(self,str(msg))
    backup_enabled = msg.data
    nepi_ros.set_param(self,"~backup_enabled",backup_enabled)
    self.publish_status()



   #######################
  ### Mgr Config Functions

  def resetMgrCb(self,msg):
    self.resetMgr()

  def resetMgr(self):
    # reset drivers dict
    nepi_ros.set_param(self,"~backup_enabled",True)
    self.drivers_files = nepi_drv.getDriverFilesList(self.drivers_folder)
    self.drivers_install_files = nepi_drv.getDriverPackagesList(self.drivers_install_folder)
    drvs_dict = nepi_drv.getDriversgetDriversDict(self.drivers_folder)
    drvs_dict = nepi_drv.setFactoryDriverOrder(drvs_dict)
    drvs_dict = activateAllDrivers(drvs_dict)
    nepi_ros.set_param(self,"~drvs_dict",drvs_dict)
    self.publish_status()


  def refreshCb(self,msg):
    self.refresh()

  def refresh(self):
    # refresh drivers dict
    self.drivers_files = nepi_drv.getDriverFilesList(self.drivers_folder)
    self.drivers_install_files = nepi_drv.getDriverPackagesList(self.drivers_install_folder)
    drvs_dict = nepi_ros.get_param(self,"~drvs_dict",self.init_drvs_dict)
    drvs_dict = nepi_drv.refreshDriversDict(self.drivers_folder,drvs_dict)
    nepi_ros.set_param(self,"~drvs_dict",drvs_dict)
    self.publish_status()

  def saveConfigCb(self, msg):  # Just update Class init values. Saving done by Config IF system
    pass # Left empty for sim, Should update from param server

  def setCurrentAsDefault(self):
    nepi_ros.set_param(self,"~backup_enabled", True)
    self.initParamServerValues(do_updates = False)

  def updateFromParamServer(self):
    #nepi_msg.publishMsgWarn(self,"Debugging: param_dict = " + str(param_dict))
    #Run any functions that need updating on value change
    # Don't need to run any additional functionsinstallDriverPkgCb
    pass
    
  def initParamServerValues(self,do_updates = True):
      self.selected_driver = 'NONE'
      nepi_msg.publishMsgInfo(self,"Setting init values to param values")
      self.init_backup_enabled = nepi_ros.get_param(self,"~backup_enabled", True)
      self.drivers_files = nepi_drv.getDriverFilesList(self.drivers_folder)
      self.drivers_install_files = nepi_drv.getDriverPackagesList(self.drivers_install_folder)
      #nepi_msg.publishMsgInfo(self,str(nepi_ros.get_param(self,"~drvs_dict")))
      none_dict = dict(NoneDict = "None")
      drvs_dict = nepi_ros.get_param(self,"~drvs_dict",none_dict)
      if 'NoneDict' not in drvs_dict.keys():
        drvs_dict = nepi_drv.updateDriversDict(self.drivers_folder,drvs_dict)
        nepi_msg.publishMsgInfo(self,"Got drvs_dict values from param server")
      else:
        drvs_dict = nepi_drv.getDriversDict(self.drivers_folder)
        drvs_dict = nepi_drv.setFactoryDriverOrder(drvs_dict)
        drvs_dict = nepi_drv.activateAllDrivers(drvs_dict)
        nepi_msg.publishMsgInfo(self,"Got drvs_dict values from reading in file data")
      self.init_drvs_dict = drvs_dict
      nepi_ros.set_param(self,"~drvs_dict",drvs_dict)
      #self.printND()
      self.resetParamServer(do_updates)
      #self.printND()

  def resetParamServer(self,do_updates = True):
      nepi_ros.set_param(self,"~backup_enabled",self.init_backup_enabled)
      nepi_ros.set_param(self,"~drvs_dict",self.init_drvs_dict)
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
    nepi_msg.publishMsgInfo(self,"Shutting down: Executing script cleanup actions")


#########################################
# Main
#########################################
if __name__ == '__main__':
  NepiDriversMgr()







